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

Función objetivo (escalarización ponderada):
    F = f1(distancia) + recharge_weight * f2(nº de recargas)
                       + costo fijo por f3(nº de vehículos)
                       + time_weight * f4(tiempo real de viaje)
Todas las distancias usadas en el modelo (costo de arco, batería, penalización de
descarte) provienen de la MISMA fuente: data['distance_matrix']. No se recalcula
la distancia por otro camino en ningún punto del módulo.

f4 (tiempo) solo está disponible cuando la instancia se cargó con
DistanceType.OSRM (data['time_matrix'] viene del servicio /table de OSRM,
ver distance/osrm_client.py). Con otros DistanceType, data['time_matrix'] es
None y time_weight debe quedarse en 0.
"""

from functools import partial
import os

from ortools.constraint_solver import pywrapcp

from distance.distance_type import DistanceType
from instance.instance_type import process_files, InstanceType
from problem.strategy_type import HeuristicType, MetaheuristicType
from utils.execute_algorithm import get_distance_and_solution_name, execute_solution


# ═══════════════════════════════════════════════════════════════
# 1. EVALUADORES DE DISTANCIA Y DEMANDA
# ═══════════════════════════════════════════════════════════════

def create_distance_evaluator(data):
    """
    Callback de distancia PURA (sin penalizaciones), en O(1) para OR-Tools.
    Se construye directamente a partir de data['distance_matrix'] — la misma
    matriz que usan la dimensión Fuel y el cálculo de drop_penalty — para que
    nunca haya dos fuentes de distancia distintas en el modelo.
    """
    distance_matrix = data['distance_matrix']
    _distances = {}
    for from_node in range(data['num_locations']):
        _distances[from_node] = {
            to_node: (0 if from_node == to_node else int(distance_matrix[from_node][to_node]))
            for to_node in range(data['num_locations'])
        }

    def distance_evaluator(manager, from_node, to_node):
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return distance_evaluator


def create_objective_evaluator(data, recharge_weight=0, time_weight=0):
    """
    Callback de costo de arco para el objetivo ponderado:
        costo(i, j) = distancia(i, j)
                    + recharge_weight            si j es estación de carga
                    + time_weight * tiempo(i, j)  si hay data['time_matrix'] (solo con OSRM)

    recharge_weight es un proxy de "número de recargas" (f2): al cobrarse cada
    vez que se ENTRA a una estación, penaliza visitarlas más veces de las
    necesarias.
    time_weight pondera f4 (tiempo real de viaje, en segundos) — solo disponible
    cuando la instancia se cargó con DistanceType.OSRM (ver distance/osrm_client.py).
    Con todos los pesos en 0 el costo de arco es distancia pura.
    """
    distance_matrix = data['distance_matrix']
    time_matrix = data.get('time_matrix')
    charging_set = set(data['charging_stations'])

    if time_weight and time_matrix is None:
        raise ValueError(
            "time_weight > 0 pero data['time_matrix'] es None. "
            "El tiempo real solo está disponible al cargar la instancia con "
            "DistanceType.OSRM."
        )

    _cost = {}
    for from_node in range(data['num_locations']):
        _cost[from_node] = {}
        for to_node in range(data['num_locations']):
            if from_node == to_node:
                _cost[from_node][to_node] = 0
            else:
                cost = int(distance_matrix[from_node][to_node])
                if to_node in charging_set:
                    cost += recharge_weight
                if time_weight:
                    cost += int(time_weight * time_matrix[from_node][to_node])
                _cost[from_node][to_node] = cost

    def objective_evaluator(manager, from_node, to_node):
        return _cost[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return objective_evaluator


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
                  elapsed_time, i, distance_type, recharge_weight=0, vehicle_fixed_cost=0, time_weight=0):
    """
    Exporta la solución a un archivo .txt con el mismo estilo que el resto del repo.
    Añade información de batería (Fuel) en cada nodo de la ruta.

    La distancia reportada se recalcula directamente desde data['distance_matrix'],
    NO desde routing.GetArcCostForVehicle: el costo de arco registrado en el modelo
    es el objetivo ponderado (distancia + penalización de recarga), así que leerlo
    como si fuera "distancia" daría un número mezclado y mal etiquetado.
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
            f.write(f'Vehicle capacity: {data["vehicle_capacity"]}\n')
            f.write(f'Battery capacity: {data["fuel_capacity"]}\n')
            f.write(f'Battery consumption rate: {data["fuel_consumption_rate"]}\n\n')

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

            distance_matrix = data['distance_matrix']
            time_matrix = data.get('time_matrix')
            total_distance = 0
            total_load = 0
            total_recharges = 0
            total_time = 0
            vehicles_used = 0

            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                plan_output = f'Route for vehicle {vehicle_id}:\n'
                route_distance = 0
                route_recharges = 0
                route_time = 0
                route_has_nodes = False

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
                            f'Bat({assignment.Min(fuel_var)}) ->'
                        )
                        route_recharges += 1
                    else:
                        plan_output += (
                            f' {node} '
                            f'Load({assignment.Min(load_var)}) '
                            f'Bat({assignment.Min(fuel_var)}) ->'
                        )

                    if node != data['depot']:
                        route_has_nodes = True

                    previous_node = node
                    index = assignment.Value(routing.NextVar(index))
                    next_node = manager.IndexToNode(index)
                    route_distance += int(distance_matrix[previous_node][next_node])
                    if time_matrix is not None:
                        route_time += int(time_matrix[previous_node][next_node])

                # Nodo final (depósito de llegada)
                load_var = capacity_dimension.CumulVar(index)
                fuel_var = fuel_dimension.CumulVar(index)
                plan_output += (
                    f' {manager.IndexToNode(index)} '
                    f'Load({assignment.Min(load_var)}) '
                    f'Bat({assignment.Min(fuel_var)})\n'
                )
                plan_output += f'Distance of the route: {route_distance}km\n'
                plan_output += f'Recharges in the route: {route_recharges}\n'
                if time_matrix is not None:
                    plan_output += f'Time of the route: {route_time}s\n'
                plan_output += f'Load of the route: {assignment.Min(load_var)}\n\n'

                f.write(plan_output)
                total_distance += route_distance
                total_load += assignment.Min(load_var)
                total_recharges += route_recharges
                total_time += route_time
                if route_has_nodes:
                    vehicles_used += 1

            f.write(f'Total Distance of all routes: {total_distance}km\n\n')
            f.write(f'Total Load of all routes: {total_load}\n\n')
            if time_matrix is not None:
                f.write(f'Total Time of all routes: {total_time}s\n\n')

            f.write('--- Desglose del objetivo ponderado ---\n')
            f.write(f'f1 Distancia total (pura, sin penalizaciones): {total_distance}\n')
            f.write(f'f2 Numero de recargas (visitas a estaciones): {total_recharges}'
                    f'  [peso recharge_weight = {recharge_weight}]\n')
            f.write(f'f3 Numero de vehiculos usados: {vehicles_used}'
                    f'  [costo fijo vehicle_fixed_cost = {vehicle_fixed_cost}]\n')
            if time_matrix is not None:
                f.write(f'f4 Tiempo total real de viaje (segundos, OSRM): {total_time}'
                        f'  [peso time_weight = {time_weight}]\n')
            f.write(f'Objetivo ponderado F = f1 + recharge_weight*f2_arcos + vehicle_fixed_cost*f3'
                    f'{" + time_weight*f4_arcos" if time_matrix is not None else ""}: '
                    f'{assignment.ObjectiveValue()}\n\n')

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
        initial_routes=None,
        recharge_weight: int = 0,        # peso de f2 (nº de recargas): 0 = comportamiento anterior
        vehicle_fixed_cost: int = 0,     # peso de f3 (nº de vehículos): 0 = comportamiento anterior
        time_weight: int = 0             # peso de f4 (tiempo real, requiere DistanceType.OSRM): 0 = ignora tiempo
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

        # ── Costo de arco: objetivo ponderado (distancia + f2 recargas + f4 tiempo) ──
        objective_evaluator_index = routing.RegisterTransitCallback(
            partial(create_objective_evaluator(data, recharge_weight, time_weight), manager)
        )
        routing.SetArcCostEvaluatorOfAllVehicles(objective_evaluator_index)

        # ── f3: costo fijo por vehículo usado (minimiza nº de vehículos) ───────
        for vehicle_id in range(data['num_vehicles']):
            routing.SetFixedCostOfVehicle(vehicle_fixed_cost, vehicle_id)

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
            partial(save_solution, recharge_weight=recharge_weight,
                   vehicle_fixed_cost=vehicle_fixed_cost, time_weight=time_weight),
            heuristic, metaheuristic, i, distance_type,
            routing, time_limit, data, manager, instance, initial_routes
        )