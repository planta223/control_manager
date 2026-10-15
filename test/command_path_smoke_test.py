#!/usr/bin/env python3
"""Localhost-only command fixtures; diagnostics are not hardware feedback."""
import copy
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8
from control_manager.msg import VehicleCommand, ControlStatus
from smoke_test import observe, wait_for
from state_split_smoke_test import start, settle


def run():
    node=rclpy.create_node('command_path_fixture')
    values=observe(node)
    pubs={key:node.create_publisher(VehicleCommand,'/control/'+key,10)
          for key in ['auto_cmd','manual_cmd','selected_cmd']}
    modes=node.create_publisher(UInt8,'/control/mode_request',10)
    durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
    estop=node.create_publisher(Bool,'/control/estop',durable)
    active={'auto':True,'manual':True,'invalid_manual':False,'direct':False,'kind':'good'}
    sent={}
    def publish():
        stamp=node.get_clock().now().to_msg()
        for key,speed,steer in [('auto',.8,.2),('manual',.4,-.1)]:
            if not active[key]: continue
            c=VehicleCommand(target_speed_mps=speed,target_steer_rad=steer,
                             target_brake=False,valid=not (key=='manual' and active['invalid_manual']),
                             reason='TEST_'+key)
            c.header.stamp=stamp; c.header.frame_id='base_link'
            sent[(key,stamp.sec,stamp.nanosec)]=copy.deepcopy(c)
            pubs[key+'_cmd'].publish(c)
        if active['direct']:
            c=VehicleCommand(target_speed_mps=.6,target_steer_rad=.1,target_brake=False,valid=True)
            c.header.stamp=copy.deepcopy(stamp); c.header.frame_id='base_link'
            kind=active['kind']
            if kind=='invalid': c.valid=False
            if kind=='nan_steer': c.target_steer_rad=float('nan')
            if kind=='inf_speed': c.target_speed_mps=float('inf')
            if kind=='speed_limit': c.target_speed_mps=2.1
            if kind=='steer_limit': c.target_steer_rad=-.6
            if kind=='negative_speed': c.target_speed_mps=-.1
            if kind=='brake_with_speed': c.target_brake=True
            if kind=='stale': c.header.stamp.sec-=5
            if kind=='future': c.header.stamp.sec+=5
            if kind=='zero': c.header.stamp.sec=c.header.stamp.nanosec=0
            if kind=='wrong_frame': c.header.frame_id='map'
            pubs['selected_cmd'].publish(c)
    timer=node.create_timer(.05,publish)
    interface=start('vehicle_interface','command_interface','operating_limits_verified:=true',
                    'max_steer_rad:=0.5','max_speed_mps:=2.0')
    selector=start('control_selector','command_selector')
    def safe():
        c=values.get('protected_cmd')
        return c is not None and not c.valid and c.target_brake and c.target_speed_mps==0 and c.target_steer_rad==0
    def choose(mode,speed):
        modes.publish(UInt8(data=mode))
        wait_for(node,lambda:'status' in values and values['status'].mode==mode and
                 values.get('protected_cmd') is not None and values['protected_cmd'].valid and
                 values['protected_cmd'].target_speed_mps==speed,'mode selection '+str(mode))
    try:
        wait_for(node,lambda:'protected_cmd' in values and values['protected_cmd'].reason=='IDLE' and
                 values['protected_cmd'].target_brake and values['protected_cmd'].target_speed_mps==0,
                 'IDLE emits neutral stop with healthy sources')
        for mode,key,speed in [(ControlStatus.AUTO,'auto',.8),(ControlStatus.MANUAL,'manual',.4)]:
            choose(mode,speed)
            c=values['selected_cmd']; t=c.header.stamp
            assert c==sent[(key,t.sec,t.nanosec)]
            print('PASS: source command fields and measurement stamp preserved: '+key,flush=True)
            active[key]=False
            wait_for(node,lambda:safe() and values['selected_cmd'].reason=='SOURCE_UNAVAILABLE',
                     key+' timeout stops despite other fresh source')
            active[key]=True
            wait_for(node,lambda:values['protected_cmd'].valid and values['protected_cmd'].target_speed_mps==speed,
                     key+' source recovers')
        active['invalid_manual']=True
        wait_for(node,lambda:safe() and values['selected_cmd'].reason=='INVALID_SOURCE_COMMAND',
                 'invalid manual source is neutralized, no AUTO fallback')
        active['invalid_manual']=False
        wait_for(node,lambda:values['protected_cmd'].valid,'valid manual source recovers')
        for mode,speed in [(ControlStatus.AUTO,.8),(ControlStatus.MANUAL,.4),(ControlStatus.IDLE,0.)]:
            choose(mode,speed)
            estop.publish(Bool(data=True))
            wait_for(node,lambda:safe() and values['protected_cmd'].reason=='E_STOP' and
                     values['status'].fsm_state==ControlStatus.FAULT,'E-STOP overrides mode '+str(mode))
            settle(node,.7)
            assert values['protected_cmd'].reason=='E_STOP'
            estop.publish(Bool(data=False))
            wait_for(node,lambda:values['protected_cmd'].valid,'explicit E-STOP reset')
        for topic in ['global_pose','motion_state','guidance_status','object_state']:
            assert not {i.node_name for i in node.get_subscriptions_info_by_topic('/control/'+topic)} & {
                'control_selector','vehicle_interface'}
        assert values['vehicle_interface_status'].dry_run
        assert {i.node_name for i in node.get_subscriptions_info_by_topic('/control/selected_cmd')
                if i.node_namespace=='/control'}=={'vehicle_interface'}
        print('PASS: direct command path, no state/object dependencies, diagnostics are dry-run only',flush=True)
        selector.close(); selector=None
        wait_for(node,safe,'selector shutdown expires at vehicle interface')
        active['direct']=True
        wait_for(node,lambda:values['protected_cmd'].valid,'isolated direct selected command fixture accepted')
        for kind,reason in [('invalid','INVALID_OR_STALE_COMMAND'),
                            ('nan_steer','INVALID_COMMAND_VALUES'),('inf_speed','INVALID_COMMAND_VALUES'),
                            ('negative_speed','INVALID_COMMAND_VALUES'),('brake_with_speed','INVALID_COMMAND_VALUES'),
                            ('speed_limit','COMMAND_OUT_OF_RANGE'),('steer_limit','COMMAND_OUT_OF_RANGE'),
                            ('stale','INVALID_OR_STALE_COMMAND'),('future','INVALID_OR_STALE_COMMAND'),
                            ('zero','INVALID_OR_STALE_COMMAND'),('wrong_frame','INVALID_OR_STALE_COMMAND')]:
            active['kind']=kind
            wait_for(node,lambda:safe() and values['protected_cmd'].reason==reason,'actuator protection: '+kind)
            active['kind']='good'
            wait_for(node,lambda:values['protected_cmd'].valid,'protected command recovers: '+kind)
        active['direct']=False
        wait_for(node,safe,'direct selected publisher silence stops actuator boundary')
        estop.publish(Bool(data=True))
        wait_for(node,lambda:safe() and values['protected_cmd'].reason=='E_STOP','E-STOP wins over missing command')
        print('ALL COMMAND PATH SMOKE CHECKS PASSED',flush=True)
    finally:
        if selector is not None: selector.close()
        interface.close(); node.destroy_timer(timer); node.destroy_node()


if __name__=='__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
