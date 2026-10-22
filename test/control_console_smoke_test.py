#!/usr/bin/env python3
"""PTY operator integration; run only in a temporary, network-isolated container."""
import json
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from nav_msgs.msg import Odometry
import yaml

assert os.environ.get('ROS_DOMAIN_ID') == '179'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('CONTROL_CONSOLE_ISOLATED_TEST') == '1', 'Use network=none, no devices and no trans'
OUT = Path(os.environ['CONTROL_CONSOLE_TEST_LOG'])
OUT.mkdir(parents=True, exist_ok=True)
ROOT = Path(__file__).resolve().parents[1]
EXECUTABLE = get_package_prefix('control_manager') + '/lib/control_manager/control_console'
MAP = '/workspace/decision/script/map_generator/test/review/maps/MAP_preliminary.json'

rclpy.init()
fixture = rclpy.create_node('console_fixture')
speed = 0.0
publisher = fixture.create_publisher(Odometry, '/gnss/odom', 10)


def odometry():
    msg = Odometry()
    msg.header.stamp = fixture.get_clock().now().to_msg()
    msg.header.frame_id = 'map'
    msg.child_frame_id = 'base_link'
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = 0.01
    msg.twist.twist.linear.x = speed
    msg.twist.covariance[0] = 0.01
    publisher.publish(msg)


fixture.create_timer(0.05, odometry)


class Console:
    def __init__(self, label, *args):
        self.master, slave = pty.openpty()
        self.path = OUT / label
        self.path.mkdir()
        self.command = [EXECUTABLE, '--log-root', str(self.path), *args]
        self.process = subprocess.Popen(self.command, stdin=slave, stdout=slave, stderr=slave,
                                        start_new_session=True)
        os.close(slave)
        self.output = ''

    def pump(self):
        if select.select([self.master], [], [], 0)[0]:
            try:
                self.output += os.read(self.master, 65536).decode('utf-8', errors='replace')
            except OSError:
                pass

    def key(self, value):
        os.write(self.master, value.encode())

    def run_dir(self):
        matches = [p.parent for p in self.path.glob('*/metadata.json')]
        return matches[0] if matches else None

    def events(self):
        run = self.run_dir()
        if run is None:
            return []
        return [json.loads(line) for line in (run / 'events.jsonl').read_text().splitlines() if line]

    def close(self):
        if self.process.poll() is None:
            # Failure cleanup only, inside the isolated test container.
            self.process.kill()
            self.process.wait(timeout=5)
            run = self.run_dir()
            if run:
                pid = json.loads((run / 'metadata.json').read_text()).get('launch_pid')
                if pid:
                    try:
                        os.killpg(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
        self.pump()
        (self.path / 'terminal.log').write_text(self.output)
        (self.path / 'command.json').write_text(json.dumps(self.command, indent=2))
        os.close(self.master)


def wait(console, condition, label, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(fixture, timeout_sec=0.02)
        console.pump()
        if condition():
            print('PASS:', label, flush=True)
            return
    raise AssertionError(label + '\n' + console.output[-2500:])


def clean_graph(console):
    wait(console, lambda: not any(ns == '/control' for _, ns in fixture.get_node_names_and_namespaces()),
         'all managed nodes stopped')


def ack_count(console, pause, exiting=False):
    return sum(e['event'] == 'acknowledged' and e['pause'] == pause and e['exiting'] == exiting
               for e in console.events())


try:
    # The production test mode publishes actuator_raw. Keep this fixture explicitly dry.
    dry_operation = yaml.safe_load((ROOT / 'config/modes/test.yaml').read_text())
    dry_operation['/control/vehicle_interface']['ros__parameters']['dry_run'] = True
    dry_file = OUT / 'dry_operation.yaml'
    dry_file.write_text(yaml.safe_dump(dry_operation))
    original_mission = (ROOT / 'config/mission.yaml').read_bytes()
    console = Console('menu_and_logging', '--map-file', MAP, '--operation-file', str(dry_file),
                      '--cruise-speed-kph', '5.0', '--max-accel-kphps', '2.0')
    try:
        wait(console, lambda: '주행 모드 선택' in console.output, 'mode selection menu')
        console.key('1')
        wait(console, lambda: '일시정지 상태로 준비됐습니다.' in console.output, 'test mode starts paused')
        run = console.run_dir()
        assert '목표속도: 5 km/h' in console.output
        metadata = json.loads((run / 'metadata.json').read_text())
        assert metadata['requested_cruise_speed_kph'] == metadata['cruise_speed_kph'] == 5.0
        assert metadata['requested_max_accel_kphps'] == metadata['max_accel_kphps'] == 2.0
        assert metadata['max_accel_mps2'] == 2.0/3.6
        assert '속도 명령 가속도 제한: 2 (km/h)/s' in console.output
        resolved = yaml.safe_load((run / 'config/operation.yaml').read_text())
        assert resolved['/control/motion_planning']['ros__parameters']['cruise_speed_mps'] == 5/3.6
        assert (ROOT / 'config/mission.yaml').read_bytes() == original_mission
        assert 'cruise_speed_kph' not in dry_operation['/**']['ros__parameters']
        print('PASS: trial speed applied to snapshots and real planner; source YAML preserved', flush=True)
        first_pid = json.loads((run / 'metadata.json').read_text())['launch_pid']
        wait(console, lambda: (run / 'launch.log').read_text().count('[DRY_RUN]') >= 2
             and '[WARN]' in (run / 'launch.log').read_text(), 'periodic logs and warnings preserved in launch.log')
        assert '[DRY_RUN]' not in console.output and '[WARN]' not in console.output
        assert list((run / 'ros').rglob('*.log'))
        print('PASS: child logs hidden from terminal; ROS log files also retained', flush=True)

        duplicate = Console('duplicate', '--mode', 'test', '--log-root', str(console.path))
        try:
            wait(duplicate, lambda: duplicate.process.poll() is not None, 'duplicate console exits')
            assert duplicate.process.returncode != 0 and '이미 실행 중' in duplicate.output
        finally:
            duplicate.close()

        console.key('1')
        wait(console, lambda: ack_count(console, False) == 1, 'resume acknowledged')
        console.key('2')
        wait(console, lambda: ack_count(console, True) == 1, 'pause acknowledged')
        console.key('1')
        wait(console, lambda: ack_count(console, False) == 2, 'second resume acknowledged')
        assert json.loads((run / 'metadata.json').read_text())['launch_pid'] == first_pid
        sequences = [e['resume_sequence'] for e in console.events() if e['event'] == 'control_status']
        assert max(sequences) == 2
        print('PASS: pause/resume preserves the launch PID and selector resume sequence', flush=True)
        console.key('3')
        wait(console, lambda: console.process.poll() is not None, 'dry-run quit completes')
        assert console.process.returncode == 0
        assert any(e['event'] == 'exit_ready' and e['verification'] == 'dry_run_stop_command' for e in console.events())
        metadata = json.loads((run / 'metadata.json').read_text())
        assert metadata['docker_image_id'] and metadata['launch_returncode'] == 0
        assert (run / 'config/source_mode.yaml').exists() and (run / 'config/map.json').exists()
        clean_graph(console)
    finally:
        console.close()

    for option in ('cruise-speed-kph', 'max-accel-kphps', 'max-decel-kphps', 'max-lateral-accel-mps2'):
        for label, value in [('zero', '0'), ('negative', '-1'), ('nan', 'nan'), ('infinite', 'inf')]:
            console = Console('invalid_' + option + '_' + label, '--mode', 'test', '--' + option, value)
            try:
                wait(console, lambda: console.process.poll() is not None, 'invalid ' + option + ' rejected: ' + label)
                assert console.process.returncode != 0 and 'finite and positive' in console.output
                assert console.run_dir() is None
            finally:
                console.close()

    console = Console('missing_variable_limits', '--mode', 'test', '--variable-speed')
    try:
        wait(console, lambda: console.process.poll() is not None, 'variable speed requires explicit limits')
        assert console.process.returncode != 0 and 'variable-speed requires' in console.output
        assert console.run_dir() is None
    finally:
        console.close()

    console = Console('variable_speed', '--mode', 'preliminary_lap', '--variable-speed',
                      '--cruise-speed-kph', '20', '--max-accel-kphps', '5',
                      '--max-decel-kphps', '5', '--max-lateral-accel-mps2', '1.5')
    try:
        wait(console, lambda: '일시정지 상태로 준비됐습니다.' in console.output, 'variable-speed console ready paused')
        run = console.run_dir()
        metadata = json.loads((run/'metadata.json').read_text())
        assert metadata['variable_speed'] and metadata['max_decel_kphps'] == 5.0
        assert metadata['speed_profile']['speed_profile.max_lateral_accel_mps2'] == 1.5
        assert metadata['speed_profile']['speed_profile.planned_decel_mps2'] == 5/3.6
        assert '속도 상한: 20 km/h' in console.output and '계획 목표속도:' in console.output
        resolved = yaml.safe_load((run/'config/operation.yaml').read_text())
        planner = resolved['/control/motion_planning']['ros__parameters']
        lateral = resolved['/control/motion_control']['ros__parameters']
        assert planner['route_progress_enabled'] and planner['route_progress'] == lateral['route_progress']
        assert planner['speed_profile']['enabled']
        console.key('3')
        wait(console, lambda: console.process.poll() is not None, 'variable-speed dry-run quit completes')
        assert console.process.returncode == 0
        clean_graph(console)
    finally:
        console.close()

    # No UDP sender is started. Real-output mode only publishes in this isolated DDS graph.
    speed = 1.0
    operation = yaml.safe_load((ROOT / 'config/modes/test.yaml').read_text())
    operation['/control/vehicle_interface']['ros__parameters']['dry_run'] = False
    operation['/control/control_selector']['ros__parameters']['start_paused'] = False
    custom = OUT / 'live_output_fixture.yaml'
    custom.write_text(yaml.safe_dump(operation))
    console = Console('measured_stop', '--mode', 'test', '--operation-file', str(custom))
    try:
        wait(console, lambda: '일시정지 상태로 준비됐습니다.' in console.output and '3.6 km/h' in console.output,
             'startup is forced paused; measured speed displayed')
        run = console.run_dir()
        assert yaml.safe_load(custom.read_text())['/control/control_selector']['ros__parameters']['start_paused'] is False
        assert yaml.safe_load((run / 'config/operation.yaml').read_text())['/control/control_selector']['ros__parameters']['start_paused'] is True
        console.key('3')
        wait(console, lambda: ack_count(console, True, exiting=True) == 1, 'quit pause acknowledged')
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(fixture, timeout_sec=0.02)
            console.pump()
        assert console.process.poll() is None
        assert not any(e['event'] == 'exit_ready' for e in console.events())
        print('PASS: live-output quit waits while measured speed is nonzero', flush=True)
        speed = 0.0
        wait(console, lambda: console.process.poll() is not None, 'quit after measured stop')
        assert console.process.returncode == 0
        assert any(e['event'] == 'exit_ready' and e['verification'] == 'measured_stop' for e in console.events())
        clean_graph(console)
    finally:
        console.close()

    console = Console('interrupt', '--mode', 'test')
    try:
        wait(console, lambda: '일시정지 상태로 준비됐습니다.' in console.output, 'interrupt fixture ready')
        console.process.send_signal(signal.SIGINT)
        wait(console, lambda: console.process.poll() is not None, 'Ctrl+C uses the acknowledged stop procedure')
        assert console.process.returncode == 0
        assert any(e['event'] == 'exit_ready' for e in console.events())
        clean_graph(console)
    finally:
        console.close()

    if not Path('/workspace/decision/script/map_generator/test/review/maps/MAP_final.json').exists():
        console = Console('missing_map', '--mode', 'final_lap')
        try:
            wait(console, lambda: console.process.poll() is not None, 'missing map reported in the terminal')
            assert console.process.returncode != 0 and '지도 파일이 없습니다' in console.output
            assert (console.run_dir() / 'console-error.log').exists()
            assert 'launch_pid' not in json.loads((console.run_dir() / 'metadata.json').read_text())
        finally:
            console.close()
    print('PASS: terminal menu, retained logs, acknowledgements, stop-before-exit and failure reporting', flush=True)
finally:
    fixture.destroy_node()
    rclpy.shutdown()
