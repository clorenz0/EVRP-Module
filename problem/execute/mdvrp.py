"""Simple Vehicles Routing Problem."""
from ortools.constraint_solver import pywrapcp
import os
from problem.strategy_type import HeuristicType, MetaheuristicType
from distance.distance_type import DistanceType, calculate_distance
from instance.instance_type import process_files
from utils.execute_algorithm import get_distance_and_solution_name, execute_solution


def save_solution(data, manager, routing, solution, instance, heuristic, metaheuristic, elapsed_time, i, distance_type):
    """Saves solution to a file."""
    distance_type, solution_name = get_distance_and_solution_name(distance_type, heuristic, metaheuristic)
    output_dir = os.path.join(f"problem/{distance_type}/solutions_mdvrp_{i}/solutions_{solution_name}")
    try:
        os.makedirs(output_dir, exist_ok=True)
        print(f"Directory {output_dir} created successfully or already exists.")
    except OSError as error:
        print(f"Error creating directory {output_dir}: {error}")
        return
    filename = os.path.join(output_dir, f'{instance}')
    try:
        with open(filename, 'w') as f:
            f.write(f"Instance: {instance}\n\n")
            f.write(f"Objective: {solution.ObjectiveValue()}\n\n")
            f.write(f"Execution Time: {elapsed_time}\n\n")
            if heuristic:
                f.write(f"Heuristic: {heuristic}\n\n")
            if metaheuristic:
                f.write(f"Metaheuristic: {metaheuristic}\n\n")
            f.write(f"Distance type: {distance_type}\n\n")
            max_route_distance = 0
            for vehicle_id in range(data["num_vehicles"]):
                index = routing.Start(vehicle_id)
                plan_output = f"Route for vehicle {vehicle_id}:\n"
                route_distance = 0
                while not routing.IsEnd(index):
                    plan_output += f" {manager.IndexToNode(index)} -> "
                    previous_index = index
                    index = solution.Value(routing.NextVar(index))
                    route_distance += routing.GetArcCostForVehicle(
                        previous_index, index, vehicle_id
                    )
                plan_output += f"{manager.IndexToNode(index)}\n"
                plan_output += f"Distance of the route: {route_distance}m\n\n"
                f.write(plan_output)
                max_route_distance += route_distance
            f.write(f"Total Distance of all routes: {max_route_distance}m\n")
        print(f"Solution saved successfully in {filename}")
    except OSError as error:
        print(f"Error writing to file {filename}: {error}")


def execute(
        i, instance_type, time_limit, vehicle_maximum_travel_distance, distance_type: DistanceType = None,
        heuristic: HeuristicType = None,
        metaheuristic: MetaheuristicType = None, initial_routes=None
):
    """Entry point of the program."""
    # Instantiate the data problem.
    instances_data = process_files(instance_type=instance_type, distance_type=distance_type, integer=True)
    for instance, data in instances_data.items():
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_vehicles"], data["starts"], data["ends"]
        )
        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.
        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data["distance_matrix"][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = "Distance"
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            vehicle_maximum_travel_distance,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name,
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        execute_solution(
            save_solution, heuristic, metaheuristic, i, distance_type, routing, time_limit, data, manager, instance, initial_routes
        )
