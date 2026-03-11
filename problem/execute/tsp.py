import os
from ortools.constraint_solver import pywrapcp
from problem.strategy_type import HeuristicType, MetaheuristicType
from distance.distance_type import DistanceType, calculate_distance
from instance.instance_type import process_files
from utils.execute_algorithm import get_distance_and_solution_name, execute_solution


# [START distance_callback]
def compute_euclidean_distance_matrix(locations, distance_type):
    """Creates callback to return distance between points."""
    distances = {}
    for from_counter, from_node in enumerate(locations):
        distances[from_counter] = {}
        for to_counter, to_node in enumerate(locations):
            if from_counter == to_counter:
                distances[from_counter][to_counter] = 0
            else:
                # Euclidean distance
                distances[from_counter][to_counter] = int(
                    calculate_distance(from_node, to_node, distance_type)
                )
    return distances


def save_solution(data, manager, routing, solution, instance, heuristic, metaheuristic, elapsed_time, i, distance_type):
    """Saves solution to a text file."""
    distance_type, solution_name = get_distance_and_solution_name(distance_type, heuristic, metaheuristic)
    solutions_dir = os.path.join(f"problem/{distance_type}/solutions_tsp_{i}/solutions_{solution_name}")
    try:
        os.makedirs(solutions_dir, exist_ok=True)
        print(f"Directory {solutions_dir} created successfully or already exists.")
    except OSError as error:
        print(f"Error creating directory {solutions_dir}: {error}")
        return
    file_name = os.path.join(solutions_dir, f"solution_{instance}")
    try:
        with open(file_name, 'w') as f:
            f.write(f"Instance: {instance}\n\n")
            f.write(f"Objective: {solution.ObjectiveValue()}\n\n")
            f.write(f"Execution Time: {elapsed_time}\n\n")
            if heuristic:
                f.write(f"Heuristic: {heuristic}\n\n")
            if metaheuristic:
                f.write(f"Metaheuristic: {metaheuristic}\n\n")
            f.write(f"Distance type: {distance_type}\n\n")
            index = routing.Start(0)
            plan_output = "Route:\n"
            route_distance = 0
            while not routing.IsEnd(index):
                plan_output += f" {manager.IndexToNode(index)} ->"
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
            plan_output += f" {manager.IndexToNode(index)}\n"
            f.write(plan_output)
            f.write(f"Objective: {route_distance}\n")
        print(f"Solution saved successfully in {file_name}")
    except OSError as error:
        print(f"Error writing to file {file_name}: {error}")


def execute(i, instance_type, time_limit, distance_type: DistanceType = None, heuristic: HeuristicType = None,
            metaheuristic: MetaheuristicType = None, initial_routes=None):
    """Entry point of the program."""
    # Instantiate the data problem.
    instances_data = process_files(instance_type)
    for instance, data in instances_data.items():
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["locations"]), data["num_vehicles"], data["depot"]
        )
        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)
        distance_matrix = compute_euclidean_distance_matrix(data["locations"], distance_type)

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return distance_matrix[from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        # Setting first solution heuristic.

        execute_solution(
            save_solution, heuristic, metaheuristic, i, distance_type, routing, time_limit, data, manager, instance, initial_routes, True
        )
