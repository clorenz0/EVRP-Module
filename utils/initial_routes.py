import json
from problem.strategy_type import HeuristicType, MetaheuristicType
from distance.distance_type import DistanceType
from instance.instance_type import process_files

with open('./instances/initial_routes_euclidean.json') as file:
    data = json.load(file)
    for row in data:
        instance_path = row['instance']
        routes = row['initial_routes']
        if instance_path.startswith("./instances/vrptw_instances"):
            execute(
                problem_type=ProblemType.CVRP,
                instance=instance_path,
                distance_type=DistanceType.EUCLIDEAN,
                time_limit=60,
                executions=10,
                vehicle_maximum_travel_distance=None,
                vehicle_max_time=None,
                vehicle_speed=None,
                heuristic=None,
                metaheuristic=None,
                initial_routes=routes
            )