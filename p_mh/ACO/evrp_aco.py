"""
evrp_aco.py — Solver alternativo para el EVRP usando Ant Colony Optimization
(ACO), en vez de OR-Tools.

No modifica ni depende de problem/execute/evrp.py — es un solver
independiente que carga la instancia con el mismo mecanismo
(instance.instance_type.process_files) y escribe la solución en el MISMO
formato de texto que evrp.py's save_solution (misma sintaxis de
"Route for vehicle N:", "NODO[CS:nombre] Load(..) Bat(..) ->", "Distance of
the route: ...km", etc.). Esto permite:
  - Comparar directamente el objetivo (distancia total) contra una corrida
    de evrp.py sobre la misma instancia.
  - Reusar tal cual los scripts de simulation/ (build_routes.py, etc.), que
    solo leen ese formato de texto — la solución de ACO se puede visualizar
    en SUMO exactamente igual que una de OR-Tools.

Se guarda en una carpeta separada (solutions_ACO) para no pisar nunca los
resultados de OR-Tools sobre la misma instancia.

Algoritmo — construcción + búsqueda local + Rank-AS, basado en tres papers:

1) Construcción probabilística consciente de batería (Thymianis et al.,
   "EVRP: Literature Review... with a Novel ACO Method", CEC 2022):
   en cada paso, clientes Y estaciones de carga alcanzables directo compiten
   en la MISMA ruleta ponderada por feromona^alpha * heuristica^beta *
   energia^gamma, donde el término de energía hace que una estación sea
   cada vez más atractiva a medida que la batería restante baja (carga
   proactiva, no solo "rescate" de último momento), y prohíbe encadenar dos
   estaciones seguidas.
2) Búsqueda local 2-opt intra-ruta sobre la mejor hormiga de cada iteración
   (Liu et al., "Hybrid BSO-ACO for Dynamic VRP on Real-World Road
   Networks", IEEE Access 2022): la construcción por hormigas sola cae en
   óptimos locales; 2-opt post-construcción es lo que le da a OR-Tools su
   ventaja de calidad, y aquí se aplica el mismo principio (solo intra-ruta;
   relocate/exchange entre rutas queda fuera de este alcance).
3) Actualización de feromona Rank-AS (Nie et al., "ACO for EVRP with
   Capacity and Charging Time Constraints", SMC 2022): en vez de que todas
   las hormigas depositen, solo las w-1 mejores de la iteración depositan
   (ponderado por su rango) más un refuerzo extra a la mejor solución
   encontrada hasta el momento — de las 5 variantes de ACO comparadas en ese
   paper, Rank-AS fue la de mejor desempeño general en EVRP.

Clientes que ningún vehículo puede servir quedan "dropped" y penalizan el
costo, igual que el AddDisjunction(drop_penalty) de evrp.py.
"""

import os
import random
import time

from distance.distance_type import DistanceType
from instance.instance_type import process_files, InstanceType
from utils.execute_algorithm import get_distance_and_solution_name


# ═══════════════════════════════════════════════════════════════
# 1. CONSTRUCCIÓN DE SOLUCIÓN (una hormiga)
# ═══════════════════════════════════════════════════════════════

def _reachable(distance_matrix, rate, from_node, to_node, battery):
    """True si hay batería suficiente para ir directo de from_node a to_node."""
    return battery - distance_matrix[from_node][to_node] * rate >= 0


def _nearest_reachable_station(distance_matrix, rate, from_node, battery, stations):
    """Estación alcanzable directo desde from_node más cercana, o None.
    Excluye from_node de las candidatas: si el vehículo ya está sobre una
    estación, "la más cercana" no puede ser ella misma (distancia 0), o el
    rescate quedaría dando vueltas sin avanzar nunca."""
    best, best_dist = None, None
    for s in stations:
        if s == from_node:
            continue
        if _reachable(distance_matrix, rate, from_node, s, battery):
            d = distance_matrix[from_node][s]
            if best_dist is None or d < best_dist:
                best, best_dist = s, d
    return best


def _energy_term(current_is_recharged, candidate_is_station, battery, fuel_capacity):
    """
    Término de energía h_ij, adaptado de Thymianis et al.: hace que una
    estación de carga sea cada vez más atractiva a medida que baja la
    batería restante, y que dos estaciones seguidas sean poco atractivas
    (justo después de recargar, current_is_recharged=True fuerza h≈0 hacia
    otra estación). Siempre > 0 para poder elevarlo a una potencia (gamma).

    A propósito NO depende de la distancia (a diferencia del paper original,
    que usa h0=1/distancia como base): la heurística de distancia ya la
    aporta eta en _roulette_pick_combined, y sumar otro término basado en
    1/distancia aquí duplicaba esa señal, sesgando hacia lo que fuera más
    cercano (estación o cliente) mucho más agresivo de lo previsto.

    Con batería llena (urgencia=0), una estación pesa ~0 — queda
    prácticamente excluida de la ruleta aunque esté cerca, y los clientes se
    eligen solo por feromona/distancia como antes. Recién cuando la batería
    empieza a agotarse de verdad, el peso de las estaciones crece (~urgencia)
    y empiezan a competir en la misma ruleta. Verificado: con la versión
    anterior (base 1.0 neutral) las estaciones seguían ganando solo por
    cercanía incluso con batería casi llena.
    """
    if current_is_recharged:
        return 1e-6 if candidate_is_station else 1.0
    urgency = max(fuel_capacity / max(battery, 1e-6) - 1.0, 0.0)
    if candidate_is_station:
        return max(urgency, 1e-6)
    return 1.0


def _roulette_pick_combined(client_candidates, station_candidates, pheromone, distance_matrix,
                             current, battery, fuel_capacity, stations_set, current_is_recharged,
                             alpha, beta, gamma, rng):
    """
    Elige el siguiente nodo (cliente o estación) por ruleta ponderada
    feromona^alpha * heuristica^beta * energia^gamma. Clientes y estaciones
    alcanzables compiten en la MISMA ruleta (a diferencia de una regla dura
    de "solo estación si no queda otra opción").
    """
    candidates = client_candidates + station_candidates
    if not candidates:
        return None
    weights = []
    for c in candidates:
        dist = max(distance_matrix[current][c], 1e-9)
        both_stations = current in stations_set and c in stations_set
        eta = 0.0 if both_stations else 1.0 / dist
        tau = pheromone[current][c]
        h = _energy_term(current_is_recharged, c in stations_set, battery, fuel_capacity)
        weights.append((tau ** alpha) * (eta ** beta) * (h ** gamma))
    total = sum(weights)
    if total <= 0:
        return rng.choice(candidates)
    r = rng.uniform(0, total)
    acc = 0.0
    for c, w in zip(candidates, weights):
        acc += w
        if acc >= r:
            return c
    return candidates[-1]


def construct_solution(data, pheromone, alpha, beta, gamma, rng, proactive_charge_threshold=0.15):
    """
    Construye una solución completa (todas las rutas de todos los vehículos)
    para una hormiga. Devuelve:
        routes: lista (por vehículo) de listas de dicts
                {'node': int, 'load': int, 'battery': num}
                (incluye depósito de salida y de llegada)
        dropped_clients: lista de nodos cliente que ningún vehículo pudo servir
        total_distance: distancia total recorrida (todas las rutas)

    proactive_charge_threshold: fracción de fuel_capacity por debajo de la
    cual una estación siquiera ENTRA a la ruleta como candidata (además del
    peso por urgencia de _energy_term). Sin este umbral duro, con batería
    alta el peso de una estación es muy chico pero no cero, y en miles de
    sorteos por corrida terminaba "cayendo" alguna vez solo por estar cerca
    — visitas de carga sin ninguna necesidad real (verificado).
    """
    depot = data['depot']
    distance_matrix = data['distance_matrix']
    demands = data['demands']
    vehicle_capacity = data['vehicle_capacity']
    fuel_capacity = data['fuel_capacity']
    rate = data['fuel_consumption_rate']
    stations = set(data['charging_stations'])
    num_clients = data['num_locations'] - len(stations) - 1
    unvisited = set(range(1, num_clients + 1))

    routes = []
    total_distance = 0

    for _vehicle_id in range(data['num_vehicles']):
        if not unvisited:
            break

        route = [{'node': depot, 'load': 0, 'battery': fuel_capacity}]
        current = depot
        load = 0
        battery = fuel_capacity
        visited_any = False
        just_recharged = True  # depot = batería llena, igual que salir de una estación

        while True:
            if not unvisited:
                break

            client_candidates = [
                c for c in unvisited
                if load + demands[c] <= vehicle_capacity
                and _reachable(distance_matrix, rate, current, c, battery)
            ]
            # Estaciones NO son candidatas: justo despues de recargar (evita
            # encadenar dos paradas seguidas), ni con bateria por encima del
            # umbral proactivo (evita cargar sin necesidad real).
            consider_stations = not just_recharged and battery < fuel_capacity * proactive_charge_threshold
            station_candidates = [
                s for s in stations if _reachable(distance_matrix, rate, current, s, battery)
            ] if consider_stations else []

            nxt = _roulette_pick_combined(
                client_candidates, station_candidates, pheromone, distance_matrix,
                current, battery, fuel_capacity, stations, just_recharged, alpha, beta, gamma, rng
            )

            if nxt is None:
                # ¿Queda algún cliente por capacidad, solo bloqueado por batería?
                capacity_ok_remaining = [c for c in unvisited if load + demands[c] <= vehicle_capacity]
                if capacity_ok_remaining:
                    station = _nearest_reachable_station(distance_matrix, rate, current, battery, stations)
                    if station is not None:
                        total_distance += distance_matrix[current][station]
                        battery -= distance_matrix[current][station] * rate
                        battery = fuel_capacity
                        current = station
                        just_recharged = True
                        route.append({'node': current, 'load': load, 'battery': battery})
                        continue
                # Sin candidatos alcanzables (por capacidad o por batería sin
                # estación de rescate): termina la ruta de este vehiculo.
                break

            total_distance += distance_matrix[current][nxt]
            battery -= distance_matrix[current][nxt] * rate
            if nxt in stations:
                battery = fuel_capacity  # recarga a full (misma política que evrp.py)
                current = nxt
                just_recharged = True
                route.append({'node': current, 'load': load, 'battery': battery})
            else:
                load += demands[nxt]
                current = nxt
                unvisited.discard(nxt)
                just_recharged = False
                route.append({'node': current, 'load': load, 'battery': battery})
                visited_any = True

        # Volver al depósito (recargando en el camino si hace falta)
        if not _reachable(distance_matrix, rate, current, depot, battery):
            station = _nearest_reachable_station(distance_matrix, rate, current, battery, stations)
            if station is not None:
                total_distance += distance_matrix[current][station]
                battery -= distance_matrix[current][station] * rate
                battery = fuel_capacity
                current = station
                route.append({'node': current, 'load': load, 'battery': battery})

        total_distance += distance_matrix[current][depot]
        battery -= distance_matrix[current][depot] * rate
        route.append({'node': depot, 'load': load, 'battery': battery})

        if visited_any:
            routes.append(route)

    dropped_clients = sorted(unvisited)
    return routes, dropped_clients, total_distance


# ═══════════════════════════════════════════════════════════════
# 2. BÚSQUEDA LOCAL — 2-opt intra-ruta
# ═══════════════════════════════════════════════════════════════

def _route_distance(route, distance_matrix):
    return sum(distance_matrix[route[k]['node']][route[k + 1]['node']] for k in range(len(route) - 1))


def _rebuild_route_state(nodes, distance_matrix, demands, rate, fuel_capacity, stations_set):
    """
    Recalcula load/battery a lo largo de una secuencia de nodos ya decidida
    (usado tras una permutación 2-opt). Devuelve None si viola batería
    (negativa) en algún punto — la carga nunca se viola por reordenar
    dentro de la MISMA ruta: es una suma acumulada de las mismas demandas
    positivas, así que el máximo acumulado no cambia con el orden.
    """
    route = [{'node': nodes[0], 'load': 0, 'battery': fuel_capacity}]
    load = 0
    battery = fuel_capacity
    for k in range(len(nodes) - 1):
        a, b = nodes[k], nodes[k + 1]
        battery -= distance_matrix[a][b] * rate
        if battery < -1e-6:
            return None
        if b in stations_set:
            battery = fuel_capacity
        else:
            load += demands[b]
        route.append({'node': b, 'load': load, 'battery': battery})
    return route


def _two_opt_route(route, data):
    """
    2-opt clásico dentro de una sola ruta (depósito de salida/llegada fijos):
    prueba invertir segmentos [i..j] y se queda con la mejora si sigue
    siendo factible en batería (recalculada desde cero tras cada inversión).
    """
    distance_matrix = data['distance_matrix']
    demands = data['demands']
    rate = data['fuel_consumption_rate']
    fuel_capacity = data['fuel_capacity']
    stations_set = set(data['charging_stations'])

    best_nodes = [stop['node'] for stop in route]
    best_distance = _route_distance(route, distance_matrix)

    improved = True
    while improved:
        improved = False
        n = len(best_nodes)
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                candidate = best_nodes[:i] + best_nodes[i:j + 1][::-1] + best_nodes[j + 1:]
                new_dist = sum(
                    distance_matrix[candidate[k]][candidate[k + 1]] for k in range(len(candidate) - 1)
                )
                if new_dist < best_distance - 1e-9:
                    rebuilt = _rebuild_route_state(candidate, distance_matrix, demands, rate, fuel_capacity, stations_set)
                    if rebuilt is not None:
                        best_nodes = candidate
                        best_distance = new_dist
                        improved = True
                        break
            if improved:
                break

    return _rebuild_route_state(best_nodes, distance_matrix, demands, rate, fuel_capacity, stations_set)


def local_search(routes, data):
    """Aplica 2-opt a cada ruta de una solución. Devuelve (rutas_mejoradas, distancia_total)."""
    improved_routes = []
    for route in routes:
        rebuilt = _two_opt_route(route, data)
        improved_routes.append(rebuilt if rebuilt is not None else route)
    total_distance = sum(_route_distance(r, data['distance_matrix']) for r in improved_routes)
    return improved_routes, total_distance


# ═══════════════════════════════════════════════════════════════
# 3. FEROMONAS — Rank-AS (Nie et al.)
# ═══════════════════════════════════════════════════════════════

def _init_pheromone(num_locations, tau0=1.0):
    return [[tau0] * num_locations for _ in range(num_locations)]


def _deposit(pheromone, routes, amount):
    for route in routes:
        for k in range(len(route) - 1):
            a, b = route[k]['node'], route[k + 1]['node']
            pheromone[a][b] += amount
            pheromone[b][a] += amount


def _update_pheromone_rank_as(pheromone, ants_results, rho, best_routes, best_cost, w):
    """
    Rank-AS: solo depositan las (w-1) hormigas mejor rankeadas de la
    iteración, ponderadas por su rango (la mejor pesa w-1, la siguiente w-2,
    ...), más un refuerzo extra (peso w) para la mejor solución encontrada
    hasta el momento (best-so-far), igual que en Nie et al.
    """
    n = len(pheromone)
    for i in range(n):
        for j in range(n):
            pheromone[i][j] *= (1.0 - rho)
            if pheromone[i][j] < 1e-6:
                pheromone[i][j] = 1e-6

    ranked = sorted(ants_results, key=lambda t: t[1])[:max(0, w - 1)]
    for rank, (routes, cost) in enumerate(ranked, start=1):
        weight = w - rank
        if weight > 0 and cost > 0:
            _deposit(pheromone, routes, weight / cost)

    if best_routes is not None and best_cost > 0:
        _deposit(pheromone, best_routes, w / best_cost)


# ═══════════════════════════════════════════════════════════════
# 4. COSTO (con penalización por clientes descartados, igual criterio que
#    el drop_penalty de evrp.py: debe superar con margen cualquier ahorro
#    posible de simplemente no servir al cliente)
# ═══════════════════════════════════════════════════════════════

def _solution_cost(data, total_distance, dropped_clients):
    if not dropped_clients:
        return total_distance
    n = data['num_locations']
    dm = data['distance_matrix']
    max_distance = max(dm[i][j] for i in range(n) for j in range(n) if i != j)
    drop_penalty = max(10_000_000, int(max_distance) * 100)
    return total_distance + drop_penalty * len(dropped_clients)


# ═══════════════════════════════════════════════════════════════
# 5. GUARDAR SOLUCIÓN (mismo formato de texto que evrp.py::save_solution)
# ═══════════════════════════════════════════════════════════════

def save_solution(data, routes, dropped_clients, total_distance, instance,
                   elapsed_time, i, distance_type, num_iterations):
    distance_type_str, solution_name = get_distance_and_solution_name(distance_type, None, "ACO")
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
            f.write(f'Objective: {int(total_distance)}\n\n')
            f.write(f'Execution Time: {elapsed_time}\n\n')
            f.write(f'Metaheuristic: ACO ({num_iterations} iteraciones)\n\n')
            f.write(f'Distance type: {distance_type_str}\n\n')
            f.write(f'Vehicle capacity: {data["vehicle_capacity"]}\n')
            f.write(f'Battery capacity: {data["fuel_capacity"]}\n')
            f.write(f'Battery consumption rate: {data["fuel_consumption_rate"]}\n\n')

            if dropped_clients:
                f.write(f'Dropped clients: {dropped_clients}\n\n')

            total_load = 0
            for vehicle_id, route in enumerate(routes):
                plan_output = f'Route for vehicle {vehicle_id}:\n'
                route_distance = 0

                for k in range(len(route) - 1):
                    node = route[k]['node']
                    if node in charging_set:
                        cs_label = f'[CS:{cs_names.get(node, node)}]'
                        plan_output += f' {node}{cs_label} Load({route[k]["load"]}) Bat({int(route[k]["battery"])}) ->'
                    else:
                        plan_output += f' {node} Load({route[k]["load"]}) Bat({int(route[k]["battery"])}) ->'
                    route_distance += data['distance_matrix'][node][route[k + 1]['node']]

                last = route[-1]
                plan_output += f' {last["node"]} Load({last["load"]}) Bat({int(last["battery"])})\n'
                plan_output += f'Distance of the route: {int(route_distance)}km\n'
                plan_output += f'Load of the route: {last["load"]}\n\n'

                f.write(plan_output)
                total_load += last['load']

            f.write(f'Total Distance of all routes: {int(total_distance)}km\n\n')
            f.write(f'Total Load of all routes: {total_load}\n\n')

        print(f"Solution saved successfully in {filename}")
    except OSError as error:
        print(f"Error writing to file {filename}: {error}")


# ═══════════════════════════════════════════════════════════════
# 6. PUNTO DE ENTRADA
# ═══════════════════════════════════════════════════════════════

def execute(
        i, instance_type, time_limit,
        vehicle_maximum_travel_distance=None,   # -> fuel_capacity
        vehicle_speed=None,                     # -> fuel_consumption_rate
        distance_type: DistanceType = None,
        num_ants=None,
        alpha=1.0,       # importancia de la feromona
        beta=1.5,        # importancia de la heuristica (1/distancia)
        gamma=2.5,       # importancia del termino de energia (Thymianis et al.)
        rho=0.5,         # tasa de evaporacion
        rank_w=6,        # Rank-AS: depositan las (w-1) mejores hormigas + best-so-far
        proactive_charge_threshold=0.15,  # % de fuel_capacity por debajo del cual se considera cargar
        seed=None,
):
    instances_data = process_files(
        instance_type, distance_type,
        None, vehicle_speed, vehicle_maximum_travel_distance
    )

    for instance, data in instances_data.items():
        if vehicle_maximum_travel_distance is not None:
            data['fuel_capacity'] = vehicle_maximum_travel_distance
        if vehicle_speed is not None:
            data['fuel_consumption_rate'] = vehicle_speed

        if 'fuel_capacity' not in data:
            raise KeyError(
                "El diccionario de datos no contiene 'fuel_capacity'. "
                "Asegurate de pasar vehicle_maximum_travel_distance al llamar a execute()."
            )
        if 'fuel_consumption_rate' not in data:
            raise KeyError(
                "El diccionario de datos no contiene 'fuel_consumption_rate'. "
                "Asegurate de pasar vehicle_speed al llamar a execute()."
            )

        rng = random.Random(seed)
        ants = num_ants or min(30, max(10, data['num_locations']))
        w = min(rank_w, ants + 1)
        pheromone = _init_pheromone(data['num_locations'])

        best_routes, best_dropped, best_cost = None, None, float('inf')
        start_time = time.time()
        iterations = 0

        print(f"Corriendo ACO sobre '{instance}' ({ants} hormigas, limite {time_limit}s)...")

        while time.time() - start_time < time_limit:
            ants_results = []
            iter_best = None  # (routes, dropped, cost)
            for _ in range(ants):
                routes, dropped, total_distance = construct_solution(
                    data, pheromone, alpha, beta, gamma, rng, proactive_charge_threshold
                )
                cost = _solution_cost(data, total_distance, dropped)
                ants_results.append((routes, cost))
                if iter_best is None or cost < iter_best[2]:
                    iter_best = (routes, dropped, cost)

            # Busqueda local 2-opt sobre la mejor hormiga de esta iteracion.
            # No cambia que nodos se visitan (mismo set por ruta), solo el
            # orden, asi que dropped_clients sigue siendo valido.
            ls_routes, ls_distance = local_search(iter_best[0], data)
            ls_cost = _solution_cost(data, ls_distance, iter_best[1])
            if ls_cost < iter_best[2]:
                iter_best = (ls_routes, iter_best[1], ls_cost)
                ants_results.append((ls_routes, ls_cost))  # que la feromona tambien aprenda de la version mejorada

            if iter_best[2] < best_cost:
                best_routes, best_dropped, best_cost = iter_best

            _update_pheromone_rank_as(pheromone, ants_results, rho, best_routes, best_cost, w)
            iterations += 1

        elapsed_time = time.time() - start_time

        if best_routes is None:
            print(f"ACO no encontro ninguna solucion para la instancia {instance}!")
            continue

        print(f"  {iterations} iteraciones, mejor objetivo: {int(best_cost)}"
              + (f" (incluye penalizacion por {len(best_dropped)} cliente(s) descartado(s))" if best_dropped else ""))

        total_distance_only = best_cost if not best_dropped else sum(
            data['distance_matrix'][route[k]['node']][route[k + 1]['node']]
            for route in best_routes for k in range(len(route) - 1)
        )
        save_solution(
            data, best_routes, best_dropped, total_distance_only, instance,
            elapsed_time, i, distance_type, iterations
        )
