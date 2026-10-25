#!/usr/bin/env python3
"""Isolated traffic ingress/recording test using the handoff's real publish method.

No GPU inference, camera device, vehicle interface, trans, or actuator UDP.
"""
from collections import defaultdict
import ast
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import sys
import time
from types import SimpleNamespace

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from control_manager.msg import GlobalPose, MotionState, ControlStatus, VehicleCommand
import yaml

assert os.environ.get('TRAFFIC_RECORDING_ISOLATED_TEST') == '1'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
OUT=Path('/evidence')
ROOT=Path('/workspace/src/control_manager')
HANDOFF=Path('/workspace/traffic_sign/CONTROL_TEAM_HANDOFF_20261025/CONTROL_TEAM_HANDOFF_20261025/baqu_traffic_light')
sys.path.insert(0,str(HANDOFF))
from reference.node import TrafficLightNode, TrafficLight

prefix=Path(get_package_prefix('control_manager'))/'lib/control_manager'
recorder=Path(get_package_prefix('vehicle_recorder'))/'lib/vehicle_recorder/record'
processes=[]
logs=[]

def start(args,name):
    log=(OUT/(name+'.log')).open('w');logs.append(log)
    proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(proc)
    (OUT/(name+'-command.json')).write_text(json.dumps(args,indent=2)+'\n')
    return proc

def stop(proc):
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=8)

def main():
    rclpy.init()
    node=rclpy.create_node('traffic_recording_fixture')
    # Check shipped recorder entry points, not just the fixture override below.
    record_config=yaml.safe_load((ROOT/'config/record_test.yaml').read_text())
    settings=record_config['/vehicle_recorder']['ros__parameters']
    default=yaml.safe_load(Path('/workspace/src/vehicle_recorder/config/record.yaml').read_text())['/vehicle_recorder']['ros__parameters']
    tree=ast.parse(Path('/workspace/src/vehicle_recorder/scripts/recorder_node.py').read_text())
    assignment=next(n for n in tree.body if isinstance(n,ast.Assign)
                    and any(isinstance(t,ast.Name) and t.id=='TOPICS' for t in n.targets))
    defaults=ast.literal_eval(assignment.value)
    for topic in ['/sensors/detectTraffic','/control/sensors/traffic']:
        assert topic in settings['topics'] and topic in default['topics'] and topic in defaults

    overlay={'/**':{'ros__parameters':{'object_state_enabled':False,'mission_limit_enabled':False,
        'mission_sections_enabled':False}},
        '/control/motion_control':{'ros__parameters':{'route_progress_enabled':False}},
        '/control/motion_planning':{'ros__parameters':{'route_progress_enabled':False,'speed_profile.enabled':False}}}
    # Two-point fixture only; production final_lap keeps its 6787-point profile.
    overlay_path=OUT/'fixture.yaml';overlay_path.write_text(yaml.safe_dump(overlay))
    def command(executable):
        args=[str(prefix/executable),'--ros-args','-r','__ns:=/control']
        for filename in ['vehicle.yaml','pipeline.yaml','mission.yaml','control.yaml','modes/final_lap.yaml']:
            args+=['--params-file',str(ROOT/'config'/filename)]
        return args+['--params-file',str(overlay_path)]
    for executable in ['sensor_interface_node','object_manager_node','motion_planning_node','motion_control_node']:
        start(command(executable),executable)

    pub=node.create_publisher(TrafficLight,'/sensors/detectTraffic',
        QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE))
    source=[];relayed=[];commands=[]
    subs=[node.create_subscription(TrafficLight,'/control/sensors/traffic',
        relayed.append,qos_profile_sensor_data,raw=True),
        node.create_subscription(VehicleCommand,'/control/auto_cmd',commands.append,10)]
    publishers={k:node.create_publisher(cls,'/control/'+k,10) for k,cls in [
        ('global_pose',GlobalPose),('motion_state',MotionState),('status',ControlStatus),('reference_path',PathMessage)]}
    def inputs():
        stamp=node.get_clock().now().to_msg()
        pose=GlobalPose(x_m=1.,y_m=0.,yaw_rad=0.,position_valid=True,valid=True)
        pose.header.frame_id='map';pose.header.stamp=stamp
        motion=MotionState(speed_mps=1.,speed_valid=True)
        motion.header.frame_id='base_link';motion.header.stamp=stamp
        status=ControlStatus(mode=ControlStatus.AUTO,fsm_state=ControlStatus.DRIVING,resume_sequence=1)
        status.header.frame_id='base_link';status.header.stamp=stamp
        path=PathMessage();path.header.frame_id='map';path.header.stamp=stamp
        for x in [0.,80.]:
            p=PoseStamped();p.header=path.header;p.pose.position.x=x;p.pose.orientation.w=1.;path.poses.append(p)
        for name,msg in [('global_pose',pose),('motion_state',motion),('status',status),('reference_path',path)]:
            publishers[name].publish(msg)
    timer=node.create_timer(.05,inputs)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.005)
    def wait(predicate,label,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate():return
        raise AssertionError(label)
    wait(lambda:commands and commands[-1].valid and not commands[-1].target_brake,'baseline control')
    baseline=(commands[-1].target_speed_mps,commands[-1].target_steer_rad,commands[-1].target_brake,commands[-1].reason)
    assert baseline[0]>0
    # Exercise the real production record config, with only fixture-provided topics selected.
    settings['topics']=['/sensors/detectTraffic','/control/sensors/traffic','/control/auto_cmd','/control/motion_plan']
    settings['reference_path_topic']='';settings['interactive_review']=False;settings['log_root']=str(OUT/'runs')
    test_config=OUT/'record.yaml';test_config.write_text(yaml.safe_dump(record_config))
    rec=start([str(recorder),'--ros-args','--params-file',str(test_config)],'record')
    wait(lambda:pub.get_subscription_count()>=2 and bool(node.get_publishers_info_by_topic('/control/sensors/traffic')),'traffic ingress and recorder ready')
    spin(.25)
    assert not node.get_publishers_info_by_topic('/control/object_state')
    assert not node.get_publishers_info_by_topic('/control/mission_limit')
    for topic in ['/sensors/detectTraffic','/control/sensors/traffic']:
        owners={(i.node_name,i.node_namespace) for i in node.get_subscriptions_info_by_topic(topic)}
        assert not any(name in ['motion_planning','motion_control','control_selector','mission_based_limit','object_manager'] for name,_ in owners),owners

    # Invoke the unmodified handoff's actual method and publish its real ROS message.
    class Publisher:
        def publish(self,msg):
            assert msg.traffic_id==0  # Handoff does not invent an intersection ID.
            # Serialize once: DDS-generated padding from a second serialization
            # is not part of the application fields. Compare actual sent CDR.
            payload=serialize_message(msg)
            source.append(payload);pub.publish(payload)
    cases=[('R',True,.91),('Y',True,.75),('G',True,.88),('Unknown',True,0.),('Unknown',False,0.)]
    transition_samples=[]
    for status,valid,confidence in cases:
        before=len(commands)
        stamp=node.get_clock().now().to_msg()
        adapter=SimpleNamespace(last_receive=time.monotonic(),
            cached_payload=((stamp.sec,stamp.nanosec),SimpleNamespace(status=status,confidence=confidence),valid),
            cached_received=time.monotonic(),last_stamp=(stamp.sec,stamp.nanosec),input_error=False,
            cfg={'input_timeout_sec':.5,'max_frame_age_sec':.5},usable=lambda stamp:True,publisher=Publisher())
        for _ in range(4):
            TrafficLightNode.publish_cached(adapter)
            spin(.05)
        observed=commands[before:]
        assert observed
        assert all((m.target_speed_mps,m.target_steer_rad,m.target_brake,m.reason)==baseline for m in observed),status
        transition_samples.append({'status':status,'valid':valid,'control_samples':len(observed),'control_unchanged':True})
    wait(lambda:len(relayed)==len(source),'all traffic relayed')
    assert relayed==source,'relay modified or lost payloads'
    spin(.3);stop(rec)
    files=list((OUT/'runs').glob('*/metadata.json'));assert len(files)==1
    meta=json.loads(files[0].read_text());assert meta['complete'] and not meta['undiscovered_topics'],meta.get('errors')
    rows=defaultdict(list)
    with sqlite3.connect((files[0].parent/'bag/bag_0.db3').resolve().as_uri()+'?mode=ro',uri=True) as db:
        for name,payload in db.execute('select t.name,m.data from messages m join topics t on t.id=m.topic_id order by m.id'):
            rows[name].append(payload)
    assert rows['/sensors/detectTraffic']==source
    assert rows['/control/sensors/traffic']==source
    assert rows['/control/auto_cmd'] and rows['/control/motion_plan']
    result={'traffic_messages_per_topic':len(source),'raw_and_relay_cdr_identical':True,
        'original_timestamps_and_repeated_results_preserved':True,'normal_unknown_and_invalid_preserved':True,
        'actual_handoff_publish_method_tested':True,'traffic_id_unspecified_zero':True,
        'control_unchanged':transition_samples,'no_traffic_control_consumers':True,
        'record_complete':meta['complete'],'gpu_inference_tested':False}
    (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
    node.destroy_timer(timer);node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    try:main()
    finally:
        for proc in reversed(processes):stop(proc)
        for log in logs:log.close()
