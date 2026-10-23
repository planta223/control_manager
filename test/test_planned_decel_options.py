"""Resolve real console configuration without launching ROS or touching source YAML."""
from pathlib import Path
import runpy
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
CONSOLE = ROOT / 'scripts/control_console'
MODULE = runpy.run_path(str(CONSOLE))


class PlannedDecelerationOptions(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.share = self.root / 'share'
        shutil.copytree(ROOT / 'config', self.share / 'config')
        self.map = self.root / 'map.json'
        self.map.write_text('{}')
        self.args = SimpleNamespace(
            mode='test', variable_speed=True, cruise_speed_kph=20.,
            max_accel_kphps=5., max_decel_kphps=5., planned_decel_kphps=3.,
            max_lateral_accel_mps2=1.5, map_file=str(self.map), operation_file=None,
            vehicle_file=None, pipeline_file=None, mission_file=None, control_file=None,
            stop_speed_mps=.1, stop_hold_s=.5)
        self.args.steering_zero_deg = 0.

    def set_vehicle_decel(self, value):
        path = self.share / 'config/vehicle.yaml'
        data = yaml.safe_load(path.read_text())
        data['/**']['ros__parameters']['vehicle']['limits']['max_decel_kphps'] = value
        path.write_text(yaml.safe_dump(data))

    def prepare(self):
        before = {p: p.read_bytes() for p in self.share.rglob('*.yaml')}
        journal = MODULE['Journal'](self.root / 'runs', 'test')
        self.addCleanup(journal.stream.close)
        with patch.dict(MODULE['prepare'].__globals__,
                        get_package_share_directory=lambda _: str(self.share)):
            MODULE['prepare'](self.args, journal)
        self.assertTrue(all(p.read_bytes() == contents for p, contents in before.items()))
        return yaml.safe_load((journal.path / 'config/operation.yaml').read_text()), journal.metadata

    def test_independent_plan_and_command_values_reach_separate_nodes(self):
        data, metadata = self.prepare()
        self.assertEqual(data['/control/control_selector']['ros__parameters']['max_decel_mps2'], 5 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['planned_decel_mps2'], 3 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_accel_mps2'], 5 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_decel_mps2'], 5 / 3.6)
        self.assertEqual(metadata['requested_planned_decel_kphps'], 3.)

    def test_steering_zero_is_recorded_with_shifted_model_limits(self):
        for zero in (-1.2, 1.2):
            with self.subTest(zero=zero):
                self.args.steering_zero_deg = zero
                data, metadata = self.prepare()
                conversion = data['/control/vehicle_interface']['ros__parameters']['trans']
                self.assertEqual(conversion['steering_zero_deg'], zero)
                limits = data['/**']['ros__parameters']['vehicle']['limits']
                self.assertAlmostEqual(limits['min_steering_deg'], max(-17.01, -17.01-zero))
                self.assertAlmostEqual(limits['max_steering_deg'], min(17.01, 17.01-zero))
                self.assertEqual(metadata['requested_steering_zero_deg'], zero)

    def test_zero_calibration_preserves_existing_model_limits(self):
        data, _ = self.prepare()
        self.assertEqual(data['/control/vehicle_interface']['ros__parameters']['trans']['steering_zero_deg'], 0.)
        self.assertNotIn('vehicle', data['/**']['ros__parameters'])

    def test_steering_zero_is_available_without_variable_speed(self):
        self.args.variable_speed = False
        self.args.steering_zero_deg = -1.2
        data, _ = self.prepare()
        self.assertEqual(data['/control/vehicle_interface']['ros__parameters']['trans']['steering_zero_deg'], -1.2)

    def test_invalid_or_endpoint_steering_zero_is_rejected(self):
        for zero in (float('nan'), float('inf'), -17.01, 17.01, -20., 20.):
            with self.subTest(zero=zero), self.assertRaisesRegex(RuntimeError, 'steering-zero-deg'):
                self.args.steering_zero_deg = zero
                self.prepare()

    def test_omitting_new_option_keeps_previous_command_fallback(self):
        self.args.planned_decel_kphps = None
        data, _ = self.prepare()
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['planned_decel_mps2'], 5 / 3.6)

    def test_planner_acceleration_uses_vehicle_fallback_and_retains_source_yaml(self):
        self.args.max_accel_kphps = None
        data, _ = self.prepare()
        vehicle = yaml.safe_load((self.share / 'config/vehicle.yaml').read_text())
        expected = vehicle['/**']['ros__parameters']['vehicle']['limits']['max_accel_kphps'] / 3.6
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_accel_mps2'], expected)

    def test_equal_plan_and_command_limits_are_allowed(self):
        self.args.planned_decel_kphps = 5.
        self.prepare()

    def test_stronger_planning_assumption_than_command_limit_is_rejected(self):
        self.args.planned_decel_kphps = 6.
        with self.assertRaisesRegex(RuntimeError, 'must not exceed the effective --max-decel-kphps'):
            self.prepare()

    def test_vehicle_fallback_accepts_smaller_explicit_plan(self):
        self.set_vehicle_decel(4.)
        self.args.max_decel_kphps = None
        data, _ = self.prepare()
        self.assertNotIn('max_decel_mps2', data['/control/control_selector']['ros__parameters'])
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['planned_decel_mps2'], 3 / 3.6)

    def test_existing_mission_plan_is_preserved_without_cli_overrides(self):
        self.set_vehicle_decel(4.)
        self.args.max_decel_kphps = self.args.planned_decel_kphps = None
        path = self.share / 'config/mission.yaml'
        data = yaml.safe_load(path.read_text())
        data['/control/motion_planning']['ros__parameters']['speed_profile']['planned_decel_mps2'] = .5
        path.write_text(yaml.safe_dump(data))
        resolved, _ = self.prepare()
        self.assertNotIn('planned_decel_mps2', resolved['/control/motion_planning']['ros__parameters']['speed_profile'])

    def test_plan_option_does_not_replace_required_command_deceleration(self):
        self.args.max_decel_kphps = None
        self.set_vehicle_decel(0.)
        with self.assertRaisesRegex(RuntimeError, 'max-decel-kphps'):
            self.prepare()

    def test_invalid_cli_assumptions_fail_before_launch(self):
        for value in ('0', '-1', 'nan', 'inf'):
            with self.subTest(value=value):
                result = subprocess.run([sys.executable, str(CONSOLE), '--mode', 'test',
                    '--variable-speed', '--planned-decel-kphps', value],
                    text=True, capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('planned-decel-kphps must be finite and positive', result.stderr)

    def test_plan_option_requires_variable_speed(self):
        result = subprocess.run([sys.executable, str(CONSOLE), '--mode', 'test',
            '--planned-decel-kphps', '3'], text=True, capture_output=True, timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('planned-decel-kphps requires --variable-speed', result.stderr)


if __name__ == '__main__':
    unittest.main()
