#!/usr/bin/env python3
"""Verify both console flags on real C++ nodes, paused/dry-run, with no trans."""
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
import yaml

assert os.environ.get('ROS_DOMAIN_ID') == '188'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
assert os.environ.get('CONTROL_CONSOLE_ISOLATED_TEST') == '1'
ROOT = Path(__file__).resolve().parents[1]
OUT = Path('/evidence/integration')
OUT.mkdir(parents=True, exist_ok=True)
NAMES = ['stopped_heading.max_distance_m', 'stopped_heading.max_reacquire_time_s']


def run():
    fixture = rclpy.create_node('heading_option_fixture')
    operation = yaml.safe_load((ROOT/'config/modes/test.yaml').read_text())
    operation['/control/vehicle_interface']['ros__parameters']['dry_run'] = True
    mode_file = OUT/'dry-mode.yaml'
    mode_file.write_text(yaml.safe_dump(operation))
    expected_defaults = yaml.safe_load((ROOT/'config/control.yaml').read_text())[
        '/control/motion_control']['ros__parameters']['stopped_heading']
    results = []
    for label, flags, expected in [
        ('legacy', [], [expected_defaults['max_distance_m'], expected_defaults['max_reacquire_time_s']]),
        ('distance-only', ['--stopped-heading-max-distance-m', '2.5'], [2.5, expected_defaults['max_reacquire_time_s']]),
        ('distance-off', ['--stopped-heading-max-distance-m', '0'], [0., expected_defaults['max_reacquire_time_s']]),
        ('both', ['--stopped-heading-max-distance-m', '2.5',
                  '--stopped-heading-max-reacquire-time-s', '6.5'], [2.5, 6.5])]:
        output = bytearray()
        master, slave = pty.openpty()
        command = [sys.executable, str(ROOT/'scripts/control_console'), '--mode', 'test',
                   '--operation-file', str(mode_file), '--log-root', str(OUT/label), *flags]
        process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
        os.close(slave)
        client = fixture.create_client(GetParameters, '/control/motion_control/get_parameters')

        def spin():
            rclpy.spin_once(fixture, timeout_sec=.02)
            if select.select([master], [], [], 0)[0]:
                try: output.extend(os.read(master, 65536))
                except OSError: pass

        def wait(predicate, timeout=25.):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                spin()
                if predicate(): return
                assert process.poll() is None, output.decode(errors='replace')
            raise AssertionError(output.decode(errors='replace'))

        try:
            wait(lambda: '일시정지 상태로 준비됐습니다.'.encode() in output)
            wait(client.service_is_ready)
            future = client.call_async(GetParameters.Request(names=NAMES))
            wait(future.done)
            actual = [v.double_value for v in future.result().values]
            assert all(math.isclose(a, b) for a, b in zip(actual, expected)), (actual, expected)
            journal = next((OUT/label).glob('*/metadata.json'))
            metadata = json.loads(journal.read_text())
            assert metadata['dry_run'] is True
            assert 'trans_node' not in [name for name, _ in fixture.get_node_names_and_namespaces()]
            os.write(master, b'q')
            wait(lambda: process.poll() is not None)
            assert process.returncode == 0, output.decode(errors='replace')
            assert json.loads(journal.read_text())['exit_code'] == 0
            results.append(dict(case=label, passed=True, actual=dict(zip(NAMES, actual))))
            print('PASS:', label, actual, flush=True)
        finally:
            if process.poll() is None:
                process.kill(); process.wait()
                for journal in (OUT/label).glob('*/metadata.json'):
                    launch_pid = json.loads(journal.read_text()).get('launch_pid')
                    if launch_pid:
                        try: os.killpg(launch_pid, signal.SIGKILL)
                        except ProcessLookupError: pass
            (OUT/(label+'-terminal.log')).write_bytes(output)
            (OUT/(label+'-command.json')).write_text(json.dumps(command, indent=2))
            fixture.destroy_client(client)
            os.close(master)
        end = time.monotonic()+8
        while any(ns == '/control' for _, ns in fixture.get_node_names_and_namespaces()):
            assert time.monotonic() < end, 'old control nodes remained'
            rclpy.spin_once(fixture, timeout_sec=.05)
    (OUT/'result.json').write_text(json.dumps(results, indent=2))
    fixture.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try: run()
    finally: rclpy.shutdown()
