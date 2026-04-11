"""
evrp.py — Solver para el Electric Vehicle Routing Problem (EVRP)

Extiende el VRPTW añadiendo una dimensión de batería (Fuel).
Estructura de nodos:
    0          → depósito
    1..N       → clientes
    N+1..N+M   → estaciones de carga (charging stations)

Las estaciones de carga son nodos opcionales (AddDisjunction con costo 0)
y son los únicos nodos donde el SlackVar de la dimensión Fuel puede ser > 0
(es decir, donde la batería puede recargarse).
"""

from functools import partial
import os

from ortools.constraint_solver import pywrapcp

from distance.distance_type import DistanceType, calculate_distance
from instance.instance_type import process_files, InstanceType
from problem.strategy_type import HeuristicType, MetaheuristicType
from utils.execute_algorithm import get_distance_and_solution_name, execute_solution


# ═══════════════════════════════════════════════════════════════
# 1. EVALUADORES DE DISTANCIA Y DEMANDA
# ═══════════════════════════════════════════════════════════════

def create_distance_evaluator(data, distance_type):
    """
    Precalcula la matriz de distancias entre todos los nodos
    (depósito + clientes + estaciones de carga).
    Devuelve un callback en O(1) para OR-Tools.
    """
    _distances = {}
    for from_node in range(data['num_locations']):
        _distances[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                _distances[from_node][to_node] = 0
            else:
                _distances[from_node][to_node] = calculate_distance(
                    point_1=data['locations'][from_node],
                    point_2=data['locations'][to_node],
                    distance_type=distance_type,
                    integer=True
                )

    def distance_evaluator(manager, from_node, to_node):
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return distance_evaluator


def create_demand_evaluator(data):
    """Devuelve la demanda del nodo actual (0 para depósito y estaciones)."""
    _demands = data['demands']

    def demand_evaluator(manager, from_node):
        return _demands[manager.IndexToNode(from_node)]

    return demand_evaluator


# ═══════════════════════════════════════════════════════════════
# 2. DIMENSIÓN DE CAPACIDAD DE CARGA
# ═══════════════════════════════════════════════════════════════

def add_capacity_constraints(routing, manager, data, demand_evaluator_index):
    """
    Restricción de capacidad de carga del vehículo.
    Las estaciones de carga tienen demanda 0, por lo que no afectan esta dimensión.

    Sobre la penalización de clientes (drop_penalty):
    ─────────────────────────────────────────────────
    OR-Tools permite descartar un cliente si el costo de servirlo supera la
    penalización. Una penalización baja (ej. 100_000) puede ser competitiva
    con el costo de distancia y el solver preferirá omitir clientes.

    PRECONDICIÓN DE FACTIBILIDAD:
        sum(demandas_clientes) ≤ num_vehicles × vehicle_capacity

    Si esa condición no se cumple, es IMPOSIBLE servir todos los clientes
    independientemente de la penalización. En ese caso el solver descartará
    clientes aunque la penalización sea infinita.
    Comprueba los parámetros de la instancia (num_vehicles, vehicle_capacity).
    """
    vehicle_capacity = data['vehicle_capacity']
    routing.AddDimension(
        demand_evaluator_index,
        0,                   # sin slack en capacidad
        vehicle_capacity,
        True,                # empieza en 0
        'Capacity'
    )
    capacity_dimension = routing.GetDimensionOrDie('Capacity')

    # Verificar factibilidad antes de construir el modelo
    num_clients = data['num_locations'] - len(data['charging_stations']) - 1
    total_demand = sum(data['demands'][1: num_clients + 1])
    total_capacity = data['num_vehicles'] * vehicle_capacity
    if total_demand > total_capacity:
        import warnings
        warnings.warn(
            f"INSTANCIA INFACTIBLE: demanda total ({total_demand}) > "
            f"capacidad total de la flota ({data['num_vehicles']} × {vehicle_capacity} = {total_capacity}). "
            f"Algunos clientes serán descartados inevitablemente. "
            f"Aumenta num_vehicles o vehicle_capacity.",
            stacklevel=2
        )

    # Penalización suficientemente alta para desincentivar descartes:
    # debe superar con margen el costo máximo posible de un arco.
    # Se usa 10× el valor mayor entre la distancia máxima de la matriz y
    # la demanda máxima, escalado a un orden de magnitud seguro.
    max_distance = max(
        data['distance_matrix'][i][j]
        for i in range(data['num_locations'])
        for j in range(data['num_locations'])
        if i != j
    )
    drop_penalty = max(10_000_000, int(max_distance) * 100)

    # Clientes: visita obligatoria (descarte solo si la instancia es infactible)
    for node in range(1, num_clients + 1):
        node_index = manager.NodeToIndex(node)
        capacity_dimension.SlackVar(node_index).SetValue(0)
        routing.AddDisjunction([node_index], drop_penalty)

    # Estaciones de carga: opcionales sin penalización (costo 0)
    for cs_node in data['charging_stations']:
        node_index = manager.NodeToIndex(cs_node)
        routing.AddDisjunction([node_index], 0)


# ═══════════════════════════════════════════════════════════════
# 3. DIMENSIÓN DE BATERÍA (FUEL) — núcleo del EVRP
# ═══════════════════════════════════════════════════════════════

def create_fuel_evaluator(data):
    """
    Callback de tránsito para la dimensión de batería.
    El valor es NEGATIVO: viajar de i a j consume energía proporcional
    a la distancia. OR-Tools acumula este valor, drenando la batería.

    Consumo = distancia * fuel_consumption_rate  (negativo para drenar)
    """
    _consumption = {}
    rate = data['fuel_consumption_rate']

    for from_node in range(data['num_locations']):
        _consumption[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                _consumption[from_node][to_node] = 0
            else:
                dist = data['distance_matrix'][from_node][to_node]
                _consumption[from_node][to_node] = -int(dist * rate)

    def fuel_evaluator(manager, from_node, to_node):
        return _consumption[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return fuel_evaluator


def add_fuel_constraints(routing, manager, data, fuel_evaluator_index):
    """
    Añade la dimensión de batería al modelo.

    Lógica:
    - CumulVar(nodo): nivel de batería al LLEGAR al nodo.
    - SlackVar(nodo): cantidad de energía recargada en ese nodo.
    - Solo las estaciones de carga pueden tener SlackVar > 0.
    - Los vehículos parten con la batería llena (fix_start_cumul_to_zero=False,
      CumulVar del inicio fijado a fuel_capacity).

    Restricciones garantizadas por AddDimension():
    - CumulVar(nodo) <= fuel_capacity (capacidad máxima de batería)
    - SlackVar(nodo) <= fuel_capacity (recarga máxima posible en un nodo)
    - Estas restricciones son suficientes para evitar sobrecarga.
    """
    fuel_capacity = data['fuel_capacity']

    routing.AddDimension(
        fuel_evaluator_index,
        fuel_capacity,       # slack máximo (recarga máxima posible en un nodo)
        fuel_capacity,       # capacidad máxima de batería
        False,               # NO fijar inicio a 0: los vehículos salen con batería llena
        'Fuel'
    )
    fuel_dimension = routing.GetDimensionOrDie('Fuel')

    # Vehículos salen con batería llena
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)
        fuel_dimension.CumulVar(start_index).SetValue(fuel_capacity)

    charging_set = set(data['charging_stations'])

    for node in range(data['num_locations']):
        if node == data['depot']:
            continue
        index = manager.NodeToIndex(node)

        if node in charging_set:
            # Estación de carga: puede recargar
            # Preferir llegar con batería llena (reduce ansiedad de rango)
            routing.AddVariableMaximizedByFinalizer(fuel_dimension.CumulVar(index))
        else:
            # Cliente: no puede recargar
            fuel_dimension.SlackVar(index).SetValue(0)


# ═══════════════════════════════════════════════════════════════
# 4. GUARDAR SOLUCIÓN
# ═══════════════════════════════════════════════════════════════

def save_solution(data, manager, routing, assignment, instance, heuristic, metaheuristic,
                  elapsed_time, i, distance_type):
    """
    Exporta la solución a un archivo .txt con el mismo estilo que el resto del repo.
    Añade información de batería (Fuel) en cada nodo de la ruta.
    """
    distance_type_str, solution_name = get_distance_and_solution_name(distance_type, heuristic, metaheuristic)
    output_dir = os.path.join(f"problem/{distance_type_str}/solutions_evrp_{i}/solutions_{solution_name}")

    try:
        os.makedirs(output_dir, exist_ok=True)
    except OSError as error:
        print(f"Error creating directory {output_dir}: {error}")
        return

    filename = os.path.join(output_dir, f'{instance}')
    charging_set = set(data['charging_stations'])
    cs_names = data.get('charging_station_names', {})

    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(f'Instance: {instance}\n\n')
            f.write(f'Objective: {assignment.ObjectiveValue()}\n\n')
            f.write(f'Execution Time: {elapsed_time}\n\n')
            if heuristic:
                f.write(f'Heuristic: {heuristic}\n\n')
            if metaheuristic:
                f.write(f'Metaheuristic: {metaheuristic}\n\n')
            f.write(f'Distance type: {distance_type_str}\n\n')
            f.write(f'Fuel capacity: {data["fuel_capacity"]}\n')
            f.write(f'Fuel consumption rate: {data["fuel_consumption_rate"]}\n\n')

            capacity_dimension = routing.GetDimensionOrDie('Capacity')
            fuel_dimension = routing.GetDimensionOrDie('Fuel')

            # Nodos descartados (dropped)
            num_clients = data['num_locations'] - len(data['charging_stations']) - 1
            dropped_clients = []
            for node in range(1, num_clients + 1):
                index = manager.NodeToIndex(node)
                if assignment.Value(routing.NextVar(index)) == index:
                    dropped_clients.append(node)

            if dropped_clients:
                f.write(f'Dropped clients: {dropped_clients}\n\n')

            total_distance = 0
            total_load = 0

            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                plan_output = f'Route for vehicle {vehicle_id}:\n'
                route_distance = 0

                while not routing.IsEnd(index):
                    node = manager.IndexToNode(index)
                    load_var = capacity_dimension.CumulVar(index)
                    fuel_var = fuel_dimension.CumulVar(index)

                    # Etiqueta especial para estaciones de carga
                    if node in charging_set:
                        cs_label = f'[CS:{cs_names.get(node, node)}]'
                        plan_output += (
                            f' {node}{cs_label} '
                            f'Load({assignment.Min(load_var)}) '
                            f'Fuel({assignment.Min(fuel_var)}) ->'
                        )
                    else:
                        plan_output += (
                            f' {node} '
                            f'Load({assignment.Min(load_var)}) '
                            f'Fuel({assignment.Min(fuel_var)}) ->'
                        )

                    previous_index = index
                    index = assignment.Value(routing.NextVar(index))
                    route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

                # Nodo final (depósito de llegada)
                load_var = capacity_dimension.CumulVar(index)
                fuel_var = fuel_dimension.CumulVar(index)
                plan_output += (
                    f' {manager.IndexToNode(index)} '
                    f'Load({assignment.Min(load_var)}) '
                    f'Fuel({assignment.Min(fuel_var)})\n'
                )
                plan_output += f'Distance of the route: {route_distance}m\n'
                plan_output += f'Load of the route: {assignment.Min(load_var)}\n\n'

                f.write(plan_output)
                total_distance += route_distance
                total_load += assignment.Min(load_var)

            f.write(f'Total Distance of all routes: {total_distance}m\n\n')
            f.write(f'Total Load of all routes: {total_load}\n\n')

        print(f"Solution saved successfully in {filename}")
    except OSError as error:
        print(f"Error writing to file {filename}: {error}")


# ═══════════════════════════════════════════════════════════════
# 5. PUNTO DE ENTRADA
# ═══════════════════════════════════════════════════════════════

def execute(
        i, instance_type, time_limit,
        vehicle_maximum_travel_distance=None,   # → fuel_capacity
        vehicle_max_time=None,                  # no usado en EVRP base
        vehicle_speed=None,                     # → fuel_consumption_rate
        distance_type: DistanceType = None,
        heuristic: HeuristicType = None,
        metaheuristic: MetaheuristicType = None,
        initial_routes=None
):
    instances_data = process_files(
        instance_type, distance_type,
        vehicle_max_time, vehicle_speed, vehicle_maximum_travel_distance
    )

    for instance, data in instances_data.items():
        # ── Separar carga de mercancía vs. batería eléctrica ──────────────────
        # 'vehicle_capacity' (carga de mercancía) viene del archivo de instancia
        # y NO debe usarse como capacidad de batería.
        # Los parámetros de batería se imponen explícitamente aquí para evitar
        # que process_files/read_file_evrp copie capacidad_vehiculo a fuel_capacity.
        if vehicle_maximum_travel_distance is not None:
            data['fuel_capacity'] = vehicle_maximum_travel_distance
        if vehicle_speed is not None:
            data['fuel_consumption_rate'] = vehicle_speed

        if 'fuel_capacity' not in data:
            raise KeyError(
                "El diccionario de datos no contiene 'fuel_capacity'. "
                "Asegúrate de pasar vehicle_maximum_travel_distance al llamar a execute()."
            )
        if 'fuel_consumption_rate' not in data:
            raise KeyError(
                "El diccionario de datos no contiene 'fuel_consumption_rate'. "
                "Asegúrate de pasar vehicle_speed al llamar a execute()."
            )

        # ── Índice Manager ────────────────────────────────────────────────────
        manager = pywrapcp.RoutingIndexManager(
            data['num_locations'],
            data['num_vehicles'],
            data['depot']
        )

        # ── Modelo de ruteo ───────────────────────────────────────────────────
        routing = pywrapcp.RoutingModel(manager)

        # ── Costo de arco: distancia ──────────────────────────────────────────
        distance_evaluator_index = routing.RegisterTransitCallback(
            partial(create_distance_evaluator(data, distance_type), manager)
        )
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

        # ── Dimensión: capacidad de carga ─────────────────────────────────────
        demand_evaluator_index = routing.RegisterUnaryTransitCallback(
            partial(create_demand_evaluator(data), manager)
        )
        add_capacity_constraints(routing, manager, data, demand_evaluator_index)

        # ── Dimensión: batería (EVRP) ─────────────────────────────────────────
        fuel_evaluator_index = routing.RegisterTransitCallback(
            partial(create_fuel_evaluator(data), manager)
        )
        add_fuel_constraints(routing, manager, data, fuel_evaluator_index)

        # ── Resolver ──────────────────────────────────────────────────────────
        execute_solution(
            save_solution, heuristic, metaheuristic, i, distance_type,
            routing, time_limit, data, manager, instance, initial_routes
        )