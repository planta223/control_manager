"""고규범 신규 로직 (2026-10-21): observer, recorder and trans; no command producer.

The SSH stdin controller is launched separately by the operator client.
Default is an isolated preview command topic and no UDP transmission.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
import manual_control


def support(context):
    live = LaunchConfiguration('live').perform(context).lower()
    if live not in ('true', 'false'):
        raise ValueError('live must be true or false')
    active = live == 'true'
    topic = '/cmd' if active else '/brake_test/preview_cmd'
    source = Path(manual_control.__file__).resolve().parent
    snapshots = [str(source/name) for name in ('brake_test_logic.py', 'brake_test_node.py', 'brake_report.py')]
    snapshots.append(str(Path(__file__).resolve()))
    return [
        Node(package='vehicle_recorder', executable='stop_observer', name='brake_test_stop_observer',
             parameters=[dict(command_topic=topic)], output='log'),
        Node(package='vehicle_recorder', executable='record', name='brake_test_recorder',
             parameters=[dict(snapshot_files=snapshots,
                              log_root=LaunchConfiguration('log_root').perform(context))], output='log'),
        Node(package='trans', executable='trans', name='brake_test_trans',
             parameters=[dict(cmd_topic=topic, test_mode=not active)], output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('live', default_value='false'),
        DeclareLaunchArgument('log_root', default_value='/workspace/log/vehicle_recorder'),
        OpaqueFunction(function=support)])
