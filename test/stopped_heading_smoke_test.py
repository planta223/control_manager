#!/usr/bin/env python3
"""GNSS pipeline fixture. Run in a temporary --rm container, network=none, no trans/devices."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
import rclpy
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
import yaml
from control_manager.msg import GlobalPose, MissionMap, MissionLimit, MotionPlan, PauseRequest, VehicleCommand, ControlStatus

assert os.environ.get('CONTROL_HEADING_ISOLATED_TEST') == '1'
assert os.environ.get('ROS_DOMAIN_ID') == '179' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(os.environ['CONTROL_HEADING_TEST_LOG'])
OUT.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('stopped_heading_fixture')
values = {}
subscriptions = []
for key, cls, topic in [('pose', GlobalPose, '/control/global_pose'),
                         ('auto', VehicleCommand, '/control/auto_cmd'),
                         ('selected', VehicleCommand, '/control/selected_cmd'),
                         ('plan', MotionPlan, '/control/motion_plan'),
                         ('status', ControlStatus, '/control/status')]:
    subscriptions.append(node.create_subscription(cls, topic, lambda m, k=key: values.__setitem__(k, m), 10))
odom_pub = node.create_publisher(Odometry, '/gnss/odom', qos_profile_sensor_data)
path_pub = node.create_publisher(PathMsg, '/control/global_path', QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
limit_pub = node.create_publisher(MissionLimit, '/control/mission_limit', 10)
request_pub = node.create_publisher(PauseRequest, '/control/pause_request', 10)
state = dict(x=4.0, yaw=0.0, yaw_variance=1e12, speed=0.0, stop=False, send=True)


def publish():
    stamp = node.get_clock().now().to_msg()
    if state['send']:
        msg = Odometry(); msg.header.stamp = stamp; msg.header.frame_id = 'map'; msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = state['x']
        msg.pose.pose.orientation.z = math.sin(state['yaw'] / 2)
        msg.pose.pose.orientation.w = math.cos(state['yaw'] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .01
        msg.pose.covariance[35] = state['yaw_variance']
        msg.twist.twist.linear.x = state['speed']; msg.twist.covariance[0] = .01
        odom_pub.publish(msg)
    path = PathMsg(); path.header.stamp = stamp; path.header.frame_id = 'map'
    for x, y in [(0., 0.), (5., 0.), (5., 6.)]:
        pose = PoseStamped(); pose.pose.position.x = x; pose.pose.position.y = y; pose.pose.orientation.w = 1.
        path.poses.append(pose)
    path_pub.publish(path)
    limit = MissionLimit(valid=True, stop_required=state['stop'], max_speed_mps=1.)
    limit.header.frame_id = 'map'; limit.header.stamp = stamp; limit.reason = 'TEST_MISSION_STOP' if state['stop'] else 'TEST_GO'
    limit_pub.publish(limit)


def pump(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def wait(predicate, label, timeout=12):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)
        if predicate():
            print('PASS:', label, flush=True)
            return
        if process.poll() is not None:
            raise AssertionError('launch exited: ' + str(process.returncode))
    raise AssertionError(label + ': ' + str({k: getattr(v, 'reason', '') for k, v in values.items()}))


def reason(text):
    return 'auto' in values and values['auto'].reason == text


def held():
    return 'auto' in values and values['auto'].valid and values['auto'].reason.startswith('HEADING_HOLD_ACTIVE:')


def tracked():
    return 'auto' in values and values['auto'].valid and values['auto'].reason.startswith('PURE_PURSUIT_ACTIVE')


def request(pause):
    msg = PauseRequest(pause=pause); msg.header.stamp = node.get_clock().now().to_msg()
    request_pub.publish(msg)


def pause():
    request(True)
    wait(lambda: values.get('status') and values['status'].paused and reason('AUTO_PAUSED'), 'manual pause acknowledged')


def resume():
    old = values['status'].resume_sequence
    request(False)
    wait(lambda: not values['status'].paused and values['status'].resume_sequence == old + 1, 'resume acknowledged')


operation = yaml.safe_load((ROOT / 'config/modes/test.yaml').read_text())
operation['/**']['ros__parameters']['mission_limit_enabled'] = True
operation['/control/motion_planning']['ros__parameters']['cruise_speed_mps'] = 1.
control = yaml.safe_load((ROOT / 'config/control.yaml').read_text())
control['/control/motion_control']['ros__parameters'].update(
    heading_bootstrap_timeout_s=.6,
    stopped_heading=dict(enabled=True, stop_speed_mps=.1, reacquire_speed_mps=.7,
                         max_distance_m=.5, max_reacquire_time_s=.65))
(OUT / 'operation.yaml').write_text(yaml.safe_dump(operation))
(OUT / 'control.yaml').write_text(yaml.safe_dump(control))
xml = ET.parse(ROOT / 'launch/control_manager.xml')
# The fixture supplies MissionLimit; no perception/mission implementation is claimed.
for element in list(xml.getroot()):
    if element.tag == 'node' and element.attrib['name'] == 'mission_based_limit':
        xml.getroot().remove(element)
xml.write(OUT / 'fixture.xml', encoding='unicode')
command = ['ros2', 'launch', str(OUT / 'fixture.xml'), 'operation_file:=' + str(OUT / 'operation.yaml'),
           'control_file:=' + str(OUT / 'control.yaml')]
(OUT / 'command.txt').write_text('\n'.join(command) + '\n')
output = (OUT / 'launch.log').open('w')
process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
timer = node.create_timer(.05, publish)
try:
    wait(lambda: values.get('status') and values['status'].paused and values.get('pose'), 'startup paused')
    resume()
    wait(lambda: reason('HEADING_BOOTSTRAP_ACTIVE'), 'first departure keeps the original straight bootstrap')
    assert values['auto'].target_steer_rad == 0 and values['auto'].target_speed_mps == 1.
    wait(lambda: reason('HEADING_BOOTSTRAP_TIMEOUT'), 'initial yaw acquisition is still bounded')
    state.update(yaw_variance=.02, speed=.8)
    wait(tracked, 'fresh yaw starts normal path tracking')
    state.update(yaw_variance=1e12, yaw=math.pi / 2)
    wait(lambda: values['pose'].heading_unavailable and not values['auto'].valid,
         'uncommanded heading loss while moving stops instead of using memory')
    state.update(yaw_variance=.02, yaw=0.)
    wait(tracked, 'fresh yaw recovers')

    pause()
    state['speed'] = 0.
    pump(.2)
    state.update(yaw_variance=1e12, yaw=math.pi / 2)
    wait(lambda: values['pose'].heading_unavailable, 'yaw expires during the stop')
    pump(.8)  # Longer than both test departure timeouts: stationary waiting is not timed out.
    resume()
    wait(held, 'manual stop resumes with remembered yaw and path tracking')
    assert values['auto'].target_steer_rad > 0.05
    assert not values['pose'].valid and abs(values['pose'].yaw_rad - math.pi / 2) < 1e-6
    print('PASS: held yaw steers into the turn; raw GlobalPose is never rewritten', flush=True)
    state.update(speed=.3, x=4.1)
    pump(.1)
    state.update(speed=.8, yaw=0., yaw_variance=.02)
    wait(tracked, 'new valid yaw takes priority after departure')

    state.update(stop=True, speed=0.)
    wait(lambda: reason('PLAN_STOP'), 'mission stop accepted without a pause request')
    pump(.15)
    state.update(yaw_variance=1e12, yaw=math.pi / 2)
    pump(.8)
    sequence = values['status'].resume_sequence
    state['stop'] = False
    wait(held, 'mission go uses remembered yaw without a resume sequence')
    assert values['status'].resume_sequence == sequence and values['auto'].target_steer_rad > .05
    state['speed'] = .2
    wait(lambda: reason('HEADING_HOLD_REACQUIRE_TIMEOUT'), 'movement without fresh yaw times out')
    assert not values['auto'].valid and values['auto'].target_brake
    request(False); pump(.2)
    assert reason('HEADING_HOLD_REACQUIRE_TIMEOUT')

    state.update(yaw=0., yaw_variance=.02, speed=.8)
    wait(tracked, 'fresh yaw recovers after timeout')
    state.update(stop=True, speed=0.)
    wait(lambda: reason('PLAN_STOP'), 'next planned stop')
    pump(.15)
    state.update(yaw_variance=1e12, yaw=math.pi / 2)
    pump(.15)
    state.update(stop=False, x=4.8)
    wait(lambda: reason('HEADING_HOLD_DISTANCE_EXCEEDED'), 'displacement invalidates remembered yaw')

    state.update(x=4.1, yaw=0., yaw_variance=.02, speed=.8)
    wait(tracked, 'tracking restored for source-loss case')
    pause(); state['speed'] = 0.; pump(.2)
    state.update(yaw_variance=1e12, yaw=math.pi / 2); pump(.15)
    state['send'] = False; pump(.7)
    state['send'] = True
    wait(lambda: values['pose'].position_valid, 'position returns after a source gap')
    resume()
    wait(lambda: not values['auto'].valid and not held() and not reason('HEADING_BOOTSTRAP_ACTIVE'),
         'source loss discards memory; resume never renews a blind straight window')

    # Exercise the actual section policy node separately from the TEST stop/go publisher.
    section_params = {'mission_limit_enabled': True, 'mission_sections_enabled': True,
                      'mission_baseline_enabled': True, 'cruise_speed_mps': 1.,
                      'section_ids': ['fixture'], 'sections.fixture.map_section': 'fixture'}
    section_file = OUT / 'section-policy.yaml'
    section_file.write_text(yaml.safe_dump({'/control/mission_based_limit': {'ros__parameters': section_params}}))
    section_command = [get_package_prefix('control_manager') + '/lib/control_manager/mission_based_limit_node', '--ros-args',
                       '-r', '__ns:=/control', '--params-file', str(section_file),
                       '-r', '/control/global_pose:=/test/mission_pose',
                       '-r', '/control/mission_map:=/test/mission_map',
                       '-r', '/control/mission_limit:=/test/mission_limit']
    (OUT / 'section-command.txt').write_text('\n'.join(section_command) + '\n')
    section_output = (OUT / 'section.log').open('w')
    section_process = subprocess.Popen(section_command, stdout=section_output, stderr=subprocess.STDOUT)
    section_values = []
    section_sub = node.create_subscription(MissionLimit, '/test/mission_limit', section_values.append, 10)
    section_pose_pub = node.create_publisher(GlobalPose, '/test/mission_pose', 10)
    section_map_pub = node.create_publisher(MissionMap, '/test/mission_map', QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    position_valid = [True]

    def section_publish():
        pose = GlobalPose(); pose.header.stamp = node.get_clock().now().to_msg(); pose.header.frame_id = 'map'
        pose.x_m = 1.; pose.position_valid = position_valid[0]; pose.valid = False
        pose.heading_unavailable = True; pose.yaw_variance_rad2 = 1e12
        pose.position_variance_x_m2 = pose.position_variance_y_m2 = .01
        section_pose_pub.publish(pose)
        data = MissionMap(); data.header = pose.header; data.path.header = pose.header
        for x in (0., 20.):
            point = PoseStamped(); point.pose.position.x = x; point.pose.orientation.w = 1.
            data.path.poses.append(point)
        data.stations_m = [0., 20.]; data.section_names = ['fixture']
        data.section_start_s_m = [10.]; data.section_end_s_m = [12.]
        section_map_pub.publish(data)

    section_timer = node.create_timer(.05, section_publish)
    try:
        wait(lambda: section_values and section_values[-1].valid,
             'real mission section policy accepts fresh position with expired yaw')
        position_valid[0] = False
        wait(lambda: not section_values[-1].valid, 'mission still rejects invalid position')
    finally:
        section_process.send_signal(signal.SIGINT); section_process.wait(timeout=8)
        section_output.close(); node.destroy_timer(section_timer); node.destroy_subscription(section_sub)
    print('PASS: initial bootstrap, manual/mission stop yaw memory, turn steering and loss bounds', flush=True)
finally:
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
    output.close(); node.destroy_timer(timer); node.destroy_node(); rclpy.shutdown()
