"""Read exported rear-axle routes; match gnss_cxx::LocalCartesian XY exactly."""
import json
import math
from pathlib import Path


def coordinate(value, limit):
    if (isinstance(value, bool) or not isinstance(value, (int, float)) or
            not math.isfinite(value) or abs(value) > limit):
        raise ValueError('invalid WGS84 coordinate')
    return float(value)


def load_route(filename):
    data = json.loads(Path(filename).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or data.get('format') != 'gps-final-map' or data.get('version') != 1:
        raise ValueError('map_file requires gps-final-map version 1 (MAP_*.json)')
    if data.get('reference_point') != 'rear_axle_center':
        raise ValueError('map_file must contain a rear_axle_center route')
    system = data.get('coordinate_system', {})
    if not isinstance(system, dict) or any(system.get(k) != v for k, v in (
            ('geographic', 'WGS84'), ('local', 'ENU'), ('distance_unit', 'm'))):
        raise ValueError('map_file requires WGS84 / ENU / m')
    if data.get('closed') is not False:
        raise ValueError('only the existing open final-map route is supported')
    points = data.get('points')
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError('map_file requires at least two ordered points')
    route = []
    previous_s = -1.0
    for index, point in enumerate(points, 1):
        if not isinstance(point, dict) or point.get('index') != index:
            raise ValueError('map_file point indices must be consecutive from 1')
        s = point.get('s_m')
        if (isinstance(s, bool) or not isinstance(s, (int, float)) or
                not math.isfinite(s) or s < 0 or s <= previous_s):
            raise ValueError('map_file s_m must be finite, nonnegative and increasing')
        previous_s = s
        route.append((coordinate(point.get('lat'), 90), coordinate(point.get('lon'), 180)))
    return route


def project_route(route, latitude, longitude):
    """Use lat/lon, not export e/n: export uses a different datum/projection.

    XY has no altitude dependence in the current GNSS implementation. The map
    already describes the rear axle; do not apply the antenna lever arm again.
    """
    lat0 = math.radians(coordinate(latitude, 90))
    lon0 = math.radians(coordinate(longitude, 180))
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    w = math.sqrt(1.0 - e2 * math.sin(lat0) ** 2)
    north_scale = 6378137.0 * (1.0 - e2) / (w * w * w)
    east_scale = 6378137.0 / w * math.cos(lat0)
    result = []
    for latitude, longitude in route:
        delta = math.fmod(math.radians(longitude) - lon0 + math.pi, 2.0 * math.pi)
        if delta < 0:
            delta += 2.0 * math.pi
        east = (delta - math.pi) * east_scale
        north = (math.radians(latitude) - lat0) * north_scale
        # gnss_cxx documents a local operating area below 5 km from the datum.
        if not math.isfinite(east) or not math.isfinite(north) or math.hypot(east, north) >= 5000:
            raise ValueError('map is outside the GNSS local projection range (5 km)')
        result.append((east, north))
    if not any(math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-6
               for a, b in zip(result, result[1:])):
        raise ValueError('map_file contains no usable segment')
    return result
