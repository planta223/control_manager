#!/usr/bin/env python3
"""Schema and GNSS projection regressions without ROS or hardware."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from global_path_map import load_route, load_final_map, mission_metadata, project_route


class GlobalPathMapTest(unittest.TestCase):
    def setUp(self):
        (ROOT / 'log/tmp').mkdir(parents=True, exist_ok=True)
        self.data = {
            'format': 'gps-final-map', 'version': 1, 'reference_point': 'rear_axle_center',
            'coordinate_system': {'geographic': 'WGS84', 'local': 'ENU', 'distance_unit': 'm'},
            'closed': False, 'points': [
                {'index': 1, 's_m': 0., 'lat': 37., 'lon': 127., 'e': 999., 'n': -999.},
                {'index': 2, 's_m': 1., 'lat': 37.00001, 'lon': 127.00001}]}

    def load(self, data):
        with tempfile.TemporaryDirectory(dir=ROOT / 'log/tmp') as directory:
            filename = Path(directory) / 'route.json'
            filename.write_text(json.dumps(data))
            return load_route(filename)

    def test_rear_axle_coordinates_preserved_without_offset(self):
        route = self.load(self.data)
        self.assertEqual(route[0], (37., 127.))
        projected = project_route(route, 37., 127.)
        self.assertEqual(projected[0], (0., 0.))  # No .559 m second lever-arm correction.
        self.assertGreater(projected[1][0], 0.)
        self.assertGreater(projected[1][1], 0.)

    def test_reject_wrong_schema_reference_order_and_nonfinite(self):
        invalid = []
        for key, value in [('format', 'gps-map-project'), ('format', 'gps-background-map'),
                           ('version', 2), ('reference_point', 'gnss_antenna'), ('closed', True),
                           ('points', [])]:
            data = copy.deepcopy(self.data); data[key] = value; invalid.append(data)
        for key, value in [('lat', float('nan')), ('lon', 181.), ('index', 3),
                           ('s_m', 0.), ('s_m', float('inf'))]:
            data = copy.deepcopy(self.data); data['points'][1][key] = value; invalid.append(data)
        for data in invalid:
            with self.subTest(data=data), self.assertRaises(ValueError):
                self.load(data)

    def test_project_bounds_and_datum(self):
        for datum in [(float('nan'), 127.), (91., 127.), (37., 181.)]:
            with self.assertRaises(ValueError):
                project_route([(37., 127.), (37.0001, 127.)], *datum)
        with self.assertRaises(ValueError):
            project_route([(37., 128.), (37.0001, 128.)], 37., 127.)
        with self.assertRaises(ValueError):
            project_route([(37., 127.), (37., 127.)], 37., 127.)
        # Longitude wrap across +/-180 follows gnss_cxx::normalizeAngle.
        self.assertGreater(project_route([(0., -179.9999), (.0001, -179.9999)], 0., 179.9999)[0][0], 0.)

    def test_actual_final_map(self):
        filename = ROOT.parents[1] / 'decision/script/map_generator/test/review/maps/MAP_2026-10-15_195100_test.json'
        if not filename.is_file():
            self.skipTest('workspace final-map fixture unavailable')
        route = load_route(filename)
        data = json.loads(filename.read_text())
        origin = data['coordinate_system']['origin']
        points = project_route(route, *origin)
        self.assertEqual(len(points), 5066)
        self.assertEqual(route[0], (data['points'][0]['lat'], data['points'][0]['lon']))
        self.assertTrue(all(math.dist(a, b) < .11 for a, b in zip(points, points[1:])))
        shifted = project_route(route, origin[0] + .0001, origin[1])
        self.assertAlmostEqual(points[0][1] - shifted[0][1], 11.1, delta=.1)

    def test_mission_metadata_matches_original_stations(self):
        data = copy.deepcopy(self.data)
        data['task_sections'] = [{'name': 'turn', 'start': {'s_m': 0.2}, 'end': {'s_m': .8}}]
        self.assertEqual(mission_metadata(data), ([0., 1.], ['turn'], [.2], [.8]))
        data['task_sections'][0]['end']['s_m'] = 2.
        with self.assertRaises(ValueError):
            mission_metadata(data)
        data['task_sections'] = []
        self.assertEqual(mission_metadata(data), ([0., 1.], [], [], []))


if __name__ == '__main__':
    unittest.main()
