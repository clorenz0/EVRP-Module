from enum import Enum
import os
from distance.distance_type import DistanceType, calculate_distance
from instance.import_data import read_file_tsp, read_file_bh, read_file_tw, read_file_md, read_file_bss, read_file_hf, process_string_instance


class InstanceType(Enum):
    TSP = 'tsp'
    BHCVRP = 'bhcvrp'
    MDCVRP = 'mdcvrp'
    VRPTW = 'vrptw'
    BSS = 'bss'
    HFVRP = 'hfvrp'
    EVRP = 'evrp'


def process_files(instance_type, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None, vehicle_maximum_travel_distance=None, path=None, integer=False):
    if isinstance(instance_type, str) and (instance_type.endswith('.txt') or instance_type.endswith('.json')):
        return process_string_instance(instance_type, distance_type, vehicle_max_time, vehicle_speed, vehicle_maximum_travel_distance, integer)
    problem_data = {
        InstanceType.TSP: {
            'read_function': read_file_tsp,
            'path': path if path else ['./instances_data/tsp_instances']
        },
        InstanceType.BHCVRP: {
            'read_function': read_file_bh,
            'path': path if path else ['./instances_data/bhcvrp_instances']
        },
        InstanceType.MDCVRP: {
            'read_function': read_file_md,
            'path': path if path else ['./instances_data/mdcvrp_instances/C-mdvrp']
        },
        InstanceType.VRPTW: {
            'read_function': read_file_tw,
            'path': path if path else ['./instances_data/vrptw_instances/test']
        },
        InstanceType.BSS: {
            'read_function': read_file_bss,
            'path': path if path else ['./instances_data/bss_instances']
        },
        InstanceType.HFVRP: {
            'read_function': read_file_hf,
            'path': path if path else ['./instances_data/hfvrp_instances']
        },
        InstanceType.EVRP: {
            'read_function': read_file_evrp,
            'path': path if path else ['./instances_data/evrp_instances']
        },
    }
    total_data = {}
    read_function = problem_data[instance_type]['read_function']
    for directory in problem_data[instance_type]['path']:
        files = os.listdir(directory)
        for file in files:
            if instance_type == InstanceType.BSS:
                if file.endswith('.json'):
                    route = os.path.join(directory, file)
                    data = read_function(route, distance_type, vehicle_max_time, vehicle_speed, vehicle_maximum_travel_distance, integer)
                    total_data[file] = data
            else:
                if file.endswith('.txt'):
                    route = os.path.join(directory, file)
                    data = read_function(route, distance_type, vehicle_max_time, vehicle_speed, vehicle_maximum_travel_distance, integer)
                    total_data[file] = data
    return total_data
