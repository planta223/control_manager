#!/usr/bin/env python3
"""Direct ROS startup rejects overrides that disable/exceed vehicle acceleration ceilings."""
import resource
import rclpy
from operation_smoke_test import command
from smoke_test import LOG, Stack, wait_for


def run():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    node = rclpy.create_node('vehicle_limits_fixture')
    cases = [
        ('accel_exceeded', 'control_selector_node', ('max_accel_mps2:=2.0',)),
        ('accel_disabled', 'control_selector_node', ('max_accel_mps2:=0.0',)),
        ('lateral_exceeded', 'motion_planning_node', ('speed_profile.enabled:=true',
            'speed_profile.max_lateral_accel_mps2:=2.0', 'speed_profile.planned_decel_mps2:=1.0')),
        ('lateral_disabled', 'motion_planning_node', ('speed_profile.enabled:=true',
            'speed_profile.max_lateral_accel_mps2:=0.0', 'speed_profile.planned_decel_mps2:=1.0')),
        ('decel_exceeded', 'motion_planning_node', ('speed_profile.enabled:=true',
            'vehicle.limits.max_decel_kphps:=2.0', 'speed_profile.planned_decel_mps2:=1.0')),
        ('decel_disabled', 'control_selector_node', ('vehicle.limits.max_decel_kphps:=2.0',
            'max_decel_mps2:=0.0')),
    ]
    try:
        for label, executable, params in cases:
            stack = Stack(command(executable, *params), label)
            try:
                wait_for(node, lambda: stack.process.poll() is not None, label)
                assert stack.process.returncode != 0
                output = (LOG / (label + '.log')).read_text()
                assert 'must not disable or exceed vehicle.limits.' in output or 'must be positive' in output
            finally:
                stack.close()
        print('PASS: direct node overrides cannot bypass vehicle acceleration ceilings', flush=True)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    rclpy.init()
    try:
        run()
    finally:
        rclpy.shutdown()
