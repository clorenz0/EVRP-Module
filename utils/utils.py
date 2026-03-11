from problem.problem_type import ProblemType
from distance.distance_type import DistanceType
from instance.instance_type import process_files, InstanceType

ALL_DISTANCES = [
    DistanceType.EUCLIDEAN,
    DistanceType.HAVERSINE,
    DistanceType.MANHATTAN,
    DistanceType.CHEBYSHEV
]

base_problems = [
    {
        "instance_type": InstanceType.TSP,
        "problem_type": ProblemType.TSP,
        "path": ['../../instances_data/tsp_instances']
    },
    {
        "instance_type": InstanceType.BHCVRP,
        "problem_type": ProblemType.CVRP,
        "path": ['../../instances_data/bhcvrp_instances']
    },
    {
        "instance_type": InstanceType.HFVRP,
        "problem_type": ProblemType.CVRP,
        "path": ['../../instances_data/hfvrp_instances']
    },
    {
        "instance_type": InstanceType.VRPTW,
        "problem_type": ProblemType.CVRP,
        "path": ['../../instances_data/vrptw_instances/test']
    },
    {
        "instance_type": InstanceType.VRPTW,
        "problem_type": ProblemType.VRPTW,
        "path": ['../../instances_data/vrptw_instances']
    },
    {
        "instance_type": InstanceType.MDCVRP,
        "problem_type": ProblemType.MDVRP,
        "path": ['../../instances_data/mdcvrp_instances/C-mdvrp']
    },
    {
        "instance_type": InstanceType.BHCVRP,
        "problem_type": ProblemType.VRPPD,
        "path": ['../../instances_data/bhcvrp_instances']
    },
    {
        "instance_type": InstanceType.MDCVRP,
        "problem_type": ProblemType.VRPPD,
        "path": ['../../instances_data/mdcvrp_instances/test']
    }
]

problems_data = []
for problem in base_problems:
    for distance in ALL_DISTANCES:
        problems_data.append({
            **problem,
            "distance_type": distance
        })


def get_data_for_predictions():
    results = {
        "TSP": {
            "Instance": [],
            "Distance": [],
            "Nodes": [],
            "Objective": [],
            "Method": [],
            "Time": [],
        },
        "VRPTW": {
            "Instance": [],
            "Distance": [],
            "Vehicles": [],
            "Vehicles Capacity": [],
            "Demands": [],
            "Nodes": [],
            "Objective": [],
            "Method": [],
            "Time": [],
            "Routes": [],
            "Load Factor": [],
            "Avg TW Start": [],
            "Avg TW End": [],
        },
        "MDVRP": {
            "Instance": [],
            "Distance": [],
            "Vehicles": [],
            "Vehicles Capacity": [],
            "Demands": [],
            "Nodes": [],
            "Objective": [],
            "Method": [],
            "Time": [],
            "Routes": [],
            "Load Factor": [],
            "Avg Depot-Client Distance": [],
        },
        "VRPPD": {
            "Instance": [],
            "Distance": [],
            "Vehicles": [],
            "Vehicles Capacity": [],
            "Demands": [],
            "Nodes": [],
            "Objective": [],
            "Method": [],
            "Time": [],
            "Routes": [],
            "Load Factor": [],
            "Avg Pickup-Delivery Distance": [],
        }
    }

    for row in problems_data:
        problem_type = row["problem_type"].value.upper()
        if problem_type in results:
            instance_data = process_files(row["instance_type"], row["distance_type"], None,
                                          None, None, row["path"])
            try:
                with open(
                        f"../../problem/solutions/{row['distance_type'].value}/solutions_{row['problem_type'].value}/all_solutions_{row['problem_type'].value}_{row['distance_type'].value}.txt",
                        "r") as file:
                    lines = file.readlines()

                header = lines[0].strip().split(";")
                records = [dict(zip(header, line.strip().split(";"))) for line in lines[1:]]

                for record in records:
                    for instance, data in instance_data.items():
                        if record["Instance"] == instance:
                            results[problem_type]["Distance"].append(row["distance_type"].value)
                            results[problem_type]["Instance"].append(instance)
                            results[problem_type]["Nodes"].append(
                                int(data["num_locations"]) if "num_locations" in data
                                else len(data["distance_matrix"])
                            )
                            results[problem_type]["Objective"].append(float(record["Objective"]))
                            results[problem_type]["Method"].append(
                                f"{record['Heuristic']}and{record['Metaheuristic']}")
                            results[problem_type]["Time"].append(float(record["Time"]))

                            if problem_type != 'TSP':
                                vehicles = int(data["num_vehicles"])
                                avg_demand = float(sum(data["demands"]) / len(data["demands"]))
                                avg_capacity = float(sum(data["vehicle_capacities"]) / len(data["vehicle_capacities"]))
                                results[problem_type]["Vehicles"].append(vehicles)
                                results[problem_type]["Demands"].append(avg_demand)
                                results[problem_type]["Vehicles Capacity"].append(avg_capacity)
                                routes_val = int(record["Routes"]) if int(record["Routes"]) > 0 else 1
                                results[problem_type]["Routes"].append(routes_val)
                                num_nodes = int(data["num_locations"]) if "num_locations" in data else len(
                                    data["distance_matrix"])
                                if vehicles * avg_capacity != 0:
                                    load_factor = (avg_demand * num_nodes) / (vehicles * avg_capacity)
                                else:
                                    load_factor = 0.0
                                results[problem_type]["Load Factor"].append(load_factor)
                            if problem_type == "VRPTW":
                                time_windows = data.get("time_windows", [])
                                if time_windows:
                                    starts = [tw[0] for tw in time_windows]
                                    ends = [tw[1] for tw in time_windows]
                                    avg_start = sum(starts) / len(starts)
                                    avg_end = sum(ends) / len(ends)
                                    tight_tw = [1 for s, e in time_windows if (e - s) <= 50]

                                    results[problem_type]["Avg TW Start"].append(avg_start)
                                    results[problem_type]["Avg TW End"].append(avg_end)
                                else:
                                    results[problem_type]["Avg TW Start"].append(0)
                                    results[problem_type]["Avg TW End"].append(0)

                            elif problem_type == "VRPPD":
                                pickup_delivery_pairs = data.get("pickup_delivery_pairs", [])
                                pd_count = len(pickup_delivery_pairs)
                                total_pd_distance = 0
                                total_service_time = 0

                                for pickup, delivery in pickup_delivery_pairs:
                                    total_pd_distance += data["distance_matrix"][pickup][delivery]
                                    total_service_time += data.get("service_times", {}).get(pickup, 0)
                                    total_service_time += data.get("service_times", {}).get(delivery, 0)

                                avg_pd_distance = total_pd_distance / pd_count if pd_count else 0
                                results[problem_type]["Avg Pickup-Delivery Distance"].append(avg_pd_distance)

                            elif problem_type == "MDVRP":
                                depots = data.get("depots", [])
                                clients = data.get("customers", [])
                                num_depots = len(depots)
                                total_depot_client_dist = 0
                                for client in clients:
                                    min_dist = min(
                                        data["distance_matrix"][client][depot] for depot in depots
                                    )
                                    total_depot_client_dist += min_dist
                                avg_depot_client_dist = total_depot_client_dist / len(clients) if clients else 0
                                results[problem_type]["Avg Depot-Client Distance"].append(avg_depot_client_dist)

            except FileNotFoundError:
                print(f"Warning: File not found for {row['problem_type'].value} with {row['distance_type'].value}")
                continue

    return results
