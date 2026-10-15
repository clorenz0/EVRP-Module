"""
Puente OR-Tools -> ACO (opcional).

Construye con una heuristica constructiva de OR-Tools (PATH_CHEAPEST_ARC, SAVINGS, ...) una solucion
inicial para el MISMO modelo EVRP que usa problem/execute/evrp.py (mismas restricciones de carga y
bateria) y la devuelve como rutas de ACO. El ACO no se modifica por dentro: solo recibe esa solucion
como una hormiga mas, que compite, se mejora con la busqueda local y deposita feromona.

Solo se usa la heuristica (sin metaheuristica): ACO es quien hace de metaheuristica.
Si OR-Tools no halla solucion completa, se devuelve None y ACO corre como siempre.
"""
from functools import partial

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from problem.execute import evrp
from problem.strategy_type import HeuristicType


def as_heuristic(value):
    """Acepta HeuristicType o su nombre ('PATH_CHEAPEST_ARC')."""
    if isinstance(value, HeuristicType):
        return value
    try:
        return HeuristicType[str(value).upper()]
    except KeyError as error:
        valid = ", ".join(h.name for h in HeuristicType)
        raise ValueError(f"Heuristica desconocida '{value}'. Disponibles: {valid}") from error


def build_initial_routes(data, heuristic, time_limit=2, station_copies=3):
    """
    Devuelve una lista de rutas [nodos] (incluye deposito al inicio y al final) o None.
    `data` es el diccionario de la instancia (ya con fuel_capacity y fuel_consumption_rate).
    """
    heuristic = as_heuristic(heuristic)
    # Re-planeacion dinamica: ACO ya separo las posiciones de arranque de las estaciones (data['vehicle_starts']);
    # el modelo de OR-Tools las espera como nodos de arranque de cada vehiculo, igual que en evrp.execute
    starts = data.get('vehicle_starts')
    initial_loads = initial_fuels = None
    if starts:
        start_nodes = [st['node'] for st in starts]
        data = dict(data, charging_stations=list(data['charging_stations']) + start_nodes)
        initial_loads = [st['load'] for st in starts]
        initial_fuels = [st['battery'] for st in starts]
    # mismo modelo que evrp.execute: varias visitas por estacion (los nodos copia se
    # devuelven con el id de la estacion original, que es el que entiende ACO)
    data = evrp.expand_station_copies(data, station_copies, keep_single=[st['node'] for st in starts] if starts else ())
    data = evrp.forbid_depot_to_station(data)      # del deposito no se va directo a una estacion
    origin = data.get('station_origin', {})
    if starts:
        manager = pywrapcp.RoutingIndexManager(data['num_locations'], data['num_vehicles'],
                                               [st['node'] for st in starts], [data['depot']] * data['num_vehicles'])
    else:
        manager = pywrapcp.RoutingIndexManager(data['num_locations'], data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    dist_idx = routing.RegisterTransitCallback(partial(evrp.create_distance_evaluator(data), manager))
    routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)
    demand_idx = routing.RegisterUnaryTransitCallback(partial(evrp.create_demand_evaluator(data), manager))
    evrp.add_capacity_constraints(routing, manager, data, demand_idx, initial_loads)
    fuel_idx = routing.RegisterTransitCallback(partial(evrp.create_fuel_evaluator(data), manager))
    evrp.add_fuel_constraints(routing, manager, data, fuel_idx, initial_fuels)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = getattr(routing_enums_pb2.FirstSolutionStrategy, heuristic.value)
    params.time_limit.FromSeconds(int(time_limit))
    solution = routing.SolveWithParameters(params)
    if solution is None:
        return None

    routes, served = [], 0
    depot = data['depot']
    stations = set(data['charging_stations'])
    for v in range(data['num_vehicles']):
        index, nodes = routing.Start(v), []
        while True:
            nodes.append(origin.get(manager.IndexToNode(index), manager.IndexToNode(index)))
            if routing.IsEnd(index):
                break
            index = solution.Value(routing.NextVar(index))
        if len(nodes) > 2:
            routes.append(nodes)
            served += sum(1 for n in nodes if n != depot and n not in stations)
    n_clients = data['num_locations'] - len(stations) - 1
    return routes if served == n_clients else None
