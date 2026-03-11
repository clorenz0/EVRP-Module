from math import radians, sin, cos, sqrt, asin, hypot
from enum import Enum
import math


class DistanceType(Enum):
    EUCLIDEAN = 'euclidean'
    MANHATTAN = 'manhattan'
    HAVERSINE = 'haversine'
    CHEBYSHEV = 'chebyshev'


def calculate_distance(point_1, point_2, distance_type=None, integer=False):
    if distance_type == DistanceType.EUCLIDEAN:
        result = math.sqrt((point_2[0] - point_1[0]) ** 2 + (point_2[1] - point_1[1]) ** 2) + 0.5
    elif distance_type == DistanceType.MANHATTAN or not distance_type:
        result = abs(point_1[0] - point_2[0]) + abs(point_1[1] - point_2[1])
    elif distance_type == DistanceType.HAVERSINE:
        result = calculate_haversine(point_1, point_2)
    elif distance_type == DistanceType.CHEBYSHEV:
        result = max(abs(point_1[0] - point_2[0]), abs(point_1[1] - point_2[1]))
    else:
        return "The distance type is not supported"
    return int(result) if integer else result


def calculate_haversine(point_1, point_2):
    r = 6371
    lat1, lon1 = map(math.radians, point_1)
    lat2, lon2 = map(math.radians, point_2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    a = min(max(a, 0), 1)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c
