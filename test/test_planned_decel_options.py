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

    def test_final_lap_console_keeps_start_first_association_in_both_nodes(self):
        self.args.mode = 'final_lap'
        data, _ = self.prepare()
        control = data['/control/motion_control']['ros__parameters']
        planner = data['/control/motion_planning']['ros__parameters']
        self.assertTrue(control['route_progress_enabled'])
        self.assertTrue(planner['route_progress_enabled'])
        self.assertEqual(planner['route_progress'], control['route_progress'])
        self.assertEqual(control['route_progress']['expected_points'], 6787)
        self.assertEqual(control['route_progress']['initial_search_m'], 10.)

    def test_independent_plan_and_command_values_reach_separate_nodes(self):
        data, metadata = self.prepare()
        self.assertEqual(data['/control/control_selector']['ros__parameters']['max_decel_mps2'], 5 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['planned_decel_mps2'], 3 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_accel_mps2'], 5 / 3.6)
        self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_decel_mps2'], 5 / 3.6)
        self.assertEqual(metadata['requested_planned_decel_kphps'], 3.)

    def test_heading_bootstrap_timeout_reaches_control_and_metadata(self):
        for timeout in (0.5, 5., 20.):
            with self.subTest(timeout=timeout):
                self.args.heading_bootstrap_timeout_s = timeout
                data, metadata = self.prepare()
                self.assertEqual(data['/control/motion_control']['ros__parameters']['heading_bootstrap_timeout_s'], timeout)
                self.assertEqual(metadata['requested_heading_bootstrap_timeout_s'], timeout)

    def test_omitting_bootstrap_timeout_keeps_yaml(self):
        data, metadata = self.prepare()
        self.assertNotIn('heading_bootstrap_timeout_s', data['/control/motion_control']['ros__parameters'])
        self.assertIsNone(metadata['requested_heading_bootstrap_timeout_s'])

    def test_invalid_bootstrap_timeout_cli_fails_before_launch(self):
        for value in ('0', '-1', 'nan', 'inf'):
            with self.subTest(value=value):
                result = subprocess.run([sys.executable, str(CONSOLE), '--mode', 'test',
                    '--heading-bootstrap-timeout-s', value], text=True, capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('heading-bootstrap-timeout-s must be finite and positive', result.stderr)

    def test_path_distance_option_reaches_both_distance_gates(self):
        for distance in (0., 3., 100.):
            with self.subTest(distance=distance):
                self.args.max_path_distance_m = distance
                data, metadata = self.prepare()
                expected = sys.float_info.max if distance == 0 else distance
                self.assertEqual(data['/control/motion_control']['ros__parameters']['max_cross_track_m'], expected)
                self.assertEqual(data['/control/motion_planning']['ros__parameters']['speed_profile']['max_path_distance_m'], expected)
                self.assertEqual(metadata['requested_max_path_distance_m'], distance)

    def test_omitting_path_distance_option_keeps_yaml(self):
        data, metadata = self.prepare()
        self.assertNotIn('max_cross_track_m', data['/control/motion_control']['ros__parameters'])
        self.assertNotIn('max_path_distance_m', data['/control/motion_planning']['ros__parameters']['speed_profile'])
        self.assertIsNone(metadata['requested_max_path_distance_m'])

    def test_invalid_path_distance_cli_fails_before_launch(self):
        for value in ('-1', 'nan', 'inf'):
            with self.subTest(value=value):
                result = subprocess.run([sys.executable, str(CONSOLE), '--mode', 'test',
                    '--max-path-distance-m', value], text=True, capture_output=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('max-path-distance-m must be finite and nonnegative', result.stderr)

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

    def test_heading_options_reach_run_configuration_and_metadata(self):
        self.args.stopped_heading_max_distance_m = 2.5
        self.args.stopped_heading_max_reacquire_time_s = 6.5
        data, metadata = self.prepare()
        heading = data['/control/motion_control']['ros__parameters']['stopped_heading']
        self.assertEqual(heading, dict(max_distance_m=2.5, max_reacquire_time_s=6.5))
        self.assertEqual(metadata['requested_stopped_heading_max_distance_m'], 2.5)
        self.assertEqual(metadata['requested_stopped_heading_max_reacquire_time_s'], 6.5)

    def test_heading_options_are_independent_and_work_without_variable_speed(self):
        self.args.variable_speed = False
        for option, key in (('stopped_heading_max_distance_m', 'max_distance_m'),
                            ('stopped_heading_max_reacquire_time_s', 'max_reacquire_time_s')):
            with self.subTest(option=option):
                self.args.stopped_heading_max_distance_m = None
                self.args.stopped_heading_max_reacquire_time_s = None
                setattr(self.args, option, 4.)
                data, _ = self.prepare()
                self.assertEqual(data['/control/motion_control']['ros__parameters']['stopped_heading'], {key: 4.})

    def test_omitting_heading_options_preserves_custom_control_yaml(self):
        path = self.share / 'config/control.yaml'
        data = yaml.safe_load(path.read_text())
        heading = data['/control/motion_control']['ros__parameters']['stopped_heading']
        heading.update(max_distance_m=4., max_reacquire_time_s=7.)
        path.write_text(yaml.safe_dump(data))
        resolved, metadata = self.prepare()
        self.assertNotIn('stopped_heading', resolved.get('/control/motion_control', {}).get('ros__parameters', {}))
        control_arg = next(arg for arg in metadata['command'] if arg.startswith('control_file:='))
        actual = yaml.safe_load(Path(control_arg.split(':=', 1)[1]).read_text())
        self.assertEqual(actual['/control/motion_control']['ros__parameters']['stopped_heading'], heading)

    def test_explicit_heading_options_override_mode_specific_values(self):
        path = self.share / 'config/modes/test.yaml'
        data = yaml.safe_load(path.read_text())
        data['/control/motion_control']['ros__parameters']['stopped_heading'] = dict(
            max_distance_m=4., max_reacquire_time_s=7., enabled=True)
        path.write_text(yaml.safe_dump(data))
        self.args.stopped_heading_max_distance_m = 2.5
        resolved, _ = self.prepare()
        self.assertEqual(resolved['/control/motion_control']['ros__parameters']['stopped_heading'],
                         dict(max_distance_m=2.5, max_reacquire_time_s=7., enabled=True))

    def test_invalid_heading_limits_fail_before_launch(self):
        for flag in ('stopped-heading-max-distance-m', 'stopped-heading-max-reacquire-time-s'):
            values = ('-1', 'nan', 'inf') if flag.endswith('distance-m') else ('0', '-1', 'nan', 'inf')
            requirement = 'nonnegative' if flag.endswith('distance-m') else 'positive'
            for value in values:
                with self.subTest(flag=flag, value=value):
                    result = subprocess.run([sys.executable, str(CONSOLE), '--mode', 'test',
                        '--'+flag, value], text=True, capture_output=True, timeout=10)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn(flag+' must be finite and '+requirement, result.stderr)

    def test_zero_heading_distance_is_preserved_in_run_configuration(self):
        self.args.stopped_heading_max_distance_m = 0.
        data, metadata = self.prepare()
        self.assertEqual(data['/control/motion_control']['ros__parameters']['stopped_heading']['max_distance_m'], 0.)
        self.assertEqual(metadata['requested_stopped_heading_max_distance_m'], 0.)


if __name__ == '__main__':
    unittest.main()
