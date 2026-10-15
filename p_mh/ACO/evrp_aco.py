"""
evrp_aco.py — Solver alternativo para el EVRP usando Ant Colony Optimization
(ACO), en vez de OR-Tools.

No modifica ni depende de problem/execute/evrp.py — es un solver
independiente que carga la instancia con el mismo mecanismo
(instance.instance_type.process_files) y escribe la solución en el MISMO
formato de texto que evrp.py's save_solution (misma sintaxis de
"Route for vehicle N:", "NODO[CS:nombre] Load(..) Bat(..) ->", "Distance of
the route: ...km", "Time of the route", "Total recharges", "Minimum battery
(SoC) reached", etc.). Esto permite:
  - Comparar directamente el objetivo (distancia total) contra una corrida
    de evrp.py sobre la misma instancia.
  - Reusar tal cual los scripts de simulation/ (build_routes.py, etc.), que
    solo leen ese formato de texto — la solución de ACO se puede visualizar
    en SUMO exactamente igual que una de OR-Tools.

Se guarda en una carpeta separada (solutions_ACO) para no pisar nunca los
resultados de OR-Tools sobre la misma instancia.

Comparabilidad con OR-Tools (evrp.py) — mismas reglas del modelo
-----------------------------------------------------------------
  - Consumo por defecto según el tipo de distancia (si no se pasa
    vehicle_speed): 3.0 con OSRM, 6.0 con Haversine (ver run.py).
  - Aritmética entera idéntica: costo de arco = int(dist) y consumo de
    arco = int(dist * rate), igual que create_distance_evaluator /
    create_fuel_evaluator de evrp.py.
  - Mismo objetivo: distancia total + drop_penalty por cliente descartado.
  - Misma política de carga: recarga completa al llegar a una estación.
  - La batería nunca puede quedar negativa, incluido el retorno al depósito:
    un vehículo solo se mueve a un nodo desde el que todavía pueda volver al
    depósito, directo o pasando por una estación de carga.
  - Las estaciones NO tienen límite de visitas (OR-Tools las trata como nodos
    de una sola visita; ver simulation/README.md). Lo que sí se penaliza es
    cargar sin necesidad: una parada de carga que se podría quitar sin romper
    la factibilidad de batería suma charge_penalty al costo interno de la
    hormiga y la búsqueda local la elimina cuando no empeora la distancia.
  - Se emiten rutas vacías para los vehículos no usados (como OR-Tools) y la
    batería de una estación se reporta al LLEGAR (antes de recargar).

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
from p_mh.ACO.strategies import make_initial_strategy, make_pheromone_strategy


# Consumo de batería por unidad de distancia según el tipo de distancia
# (mismos valores que documenta run.py: 6 para HAVERSINE, 3 para OSRM).
DEFAULT_CONSUMPTION_RATE = {
    DistanceType.OSRM: 3.0,
    DistanceType.HAVERSINE: 6.0,
}


# ═══════════════════════════════════════════════════════════════
# 0. MATRICES ENTERAS (misma aritmética que evrp.py)
# ═══════════════════════════════════════════════════════════════

def _prepare_matrices(data):
    """
    Agrega a `data`:
      cost_matrix   : int(dist)         -> costo de arco (como create_distance_evaluator)
      energy_matrix : int(dist * rate)  -> consumo de arco (como create_fuel_evaluator)
      charge_penalty: distancia media (>0) de un arco; costo interno de una
                      parada de carga innecesaria.
    `distance_matrix` (float) se conserva solo para la heurística 1/dist.
    """
    n = data['num_locations']
    dm = data['distance_matrix']
    rate = data['fuel_consumption_rate']
    data['cost_matrix'] = [[0 if i == j else int(dm[i][j]) for j in range(n)] for i in range(n)]
    data['energy_matrix'] = [[0 if i == j else int(dm[i][j] * rate) for j in range(n)] for i in range(n)]
    positives = [data['cost_matrix'][i][j] for i in range(n) for j in range(n) if i != j and data['cost_matrix'][i][j] > 0]
    data['charge_penalty'] = (sum(positives) // len(positives)) if positives else 1

    # Batería mínima necesaria en cada nodo para poder cerrar la ruta (directo
    # al depósito, o vía una estación desde la que se llegue al depósito).
    em = data['energy_matrix']
    depot = data['depot']
    cap = data['fuel_capacity']
    stations = data['charging_stations']
    inf = float('inf')
    need = []
    for node in range(n):
        best = em[node][depot]
        for s in stations:
            if s != node and em[s][depot] <= cap and em[node][s] < best:
                best = em[node][s]
        need.append(best if best <= cap else inf)
    data['finish_need'] = need
    data['station_set'] = set(stations)

    # Listas de candidatos: clientes ordenados por cercanía a cada nodo.
    num_clients = data.get('num_clients', n - len(stations) - 1)
    clients = list(range(1, num_clients + 1))
    data['client_neighbors'] = [
        sorted((c for c in clients if c != node), key=lambda c, node=node: dm[node][c])
        for node in range(n)
    ]


# ═══════════════════════════════════════════════════════════════
# 1. CONSTRUCCIÓN DE SOLUCIÓN (una hormiga)
# ═══════════════════════════════════════════════════════════════

def _can_finish(need, node, battery):
    """
    True si desde `node`, con `battery`, el vehículo todavía puede cerrar la
    ruta: llegar directo al depósito, o llegar a una estación (distinta de
    `node`) desde la que, ya con batería llena, alcance el depósito.
    Esta es la restricción de "poder volver al depósito o cargar cerca".
    `need[node]` (precalculado en _prepare_matrices) es la batería mínima
    que hace falta en `node` para cumplirla, así que la consulta es O(1).
    """
    return battery >= need[node]


def _rescue_station(data, current, battery, load, unvisited, stations):
    """
    Estación alcanzable desde `current` más cercana que además sirve de algo:
    tras recargar, algún cliente pendiente (que quepa por carga) queda
    alcanzable y con retorno garantizado. None si no existe.
    """
    em = data['energy_matrix']
    dm = data['distance_matrix']
    need = data['finish_need']
    cap = data['fuel_capacity']
    demands = data['demands']
    vcap = data['vehicle_capacity']

    best, best_dist = None, None
    for s in stations:
        if s == current or em[current][s] > battery:
            continue
        if not _can_finish(need, s, cap):
            continue
        useful = any(
            load + demands[c] <= vcap
            and em[s][c] <= cap
            and _can_finish(need, c, cap - em[s][c])
            for c in unvisited
        )
        if not useful:
            continue
        d = dm[current][s]
        if best_dist is None or d < best_dist:
            best, best_dist = s, d
    return best


def _closing_station(data, current, battery, stations):
    """Estación para cerrar la ruta: alcanzable ahora y desde la que se llega al depósito con batería llena."""
    em = data['energy_matrix']
    dm = data['distance_matrix']
    depot = data['depot']
    cap = data['fuel_capacity']
    best, best_total = None, None
    for s in stations:
        if s != current and em[current][s] <= battery and em[s][depot] <= cap:
            total = dm[current][s] + dm[s][depot]
            if best_total is None or total < best_total:
                best, best_total = s, total
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
    y empiezan a competir en la misma ruleta.
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


def construct_solution(data, pheromone, alpha, beta, gamma, rng, proactive_charge_threshold=0.15,
                       candidate_list_size=20):
    """
    Construye una solución completa (todas las rutas de todos los vehículos)
    para una hormiga. Devuelve:
        routes: lista (por vehículo usado) de listas de dicts
                {'node': int, 'load': int, 'battery': int}
                (incluye depósito de salida y de llegada). 'battery' es el
                nivel AL LLEGAR al nodo (en una estación, antes de recargar).
        dropped_clients: lista de nodos cliente que ningún vehículo pudo servir
        total_distance: distancia total (suma de int(dist) por arco)

    Invariante de factibilidad: el vehículo solo avanza a un nodo si desde él
    puede volver al depósito (directo, o pasando por una estación). Por eso la
    batería nunca queda negativa, ni siquiera en el retorno al depósito.

    proactive_charge_threshold: fracción de fuel_capacity por debajo de la
    cual una estación siquiera ENTRA a la ruleta como candidata (además del
    peso por urgencia de _energy_term).
    """
    depot = data['depot']
    distance_matrix = data['distance_matrix']
    cm = data['cost_matrix']
    em = data['energy_matrix']
    demands = data['demands']
    vehicle_capacity = data['vehicle_capacity']
    fuel_capacity = data['fuel_capacity']
    need = data['finish_need']
    neighbors = data['client_neighbors']
    stations = set(data['charging_stations'])
    num_clients = data.get('num_clients', data['num_locations'] - len(stations) - 1)
    unvisited = set(range(1, num_clients + 1))
    starts = data.get('vehicle_starts')   # re-planeacion: cada vehiculo arranca en su posicion, con su bateria y carga

    routes = []

    for _vehicle_id in range(data['num_vehicles']):
        if not unvisited:
            break

        if starts:
            start_node, battery, load = starts[_vehicle_id]['node'], starts[_vehicle_id]['battery'], starts[_vehicle_id]['load']
        else:
            start_node, battery, load = depot, fuel_capacity, 0
        route = [{'node': start_node, 'load': load, 'battery': battery}]
        served = []                 # clientes de esta ruta (para revertir si algo sale mal)
        current = start_node
        just_recharged = battery >= fuel_capacity  # depot = batería llena, igual que salir de una estación

        while unvisited:
            # Lista de candidatos: los `candidate_list_size` clientes factibles
            # más cercanos (si ninguno es factible, se revisan todos).
            client_candidates = []
            for c in neighbors[current]:
                if c in unvisited and load + demands[c] <= vehicle_capacity \
                        and em[current][c] <= battery and battery - em[current][c] >= need[c]:
                    client_candidates.append(c)
                    if len(client_candidates) >= candidate_list_size:
                        break
            # Estaciones NO son candidatas justo después de recargar (evita
            # encadenar dos paradas seguidas), ni con batería por encima del
            # umbral proactivo (evita cargar sin necesidad real).
            consider_stations = not just_recharged and battery < fuel_capacity * proactive_charge_threshold
            station_candidates = [
                s for s in stations
                if s != current and em[current][s] <= battery
                and _can_finish(need, s, fuel_capacity)
            ] if consider_stations else []

            nxt = _roulette_pick_combined(
                client_candidates, station_candidates, pheromone, distance_matrix,
                current, battery, fuel_capacity, stations, just_recharged, alpha, beta, gamma, rng
            )

            if nxt is None:
                # Quedan clientes que caben por carga pero la batería actual no
                # alcanza: rescate en una estación desde la que realmente se
                # pueda seguir sirviendo. Si acabamos de recargar y aun así no
                # hay candidatos, el cliente es inalcanzable: cerrar ruta.
                if not just_recharged and any(load + demands[c] <= vehicle_capacity for c in unvisited):
                    station = _rescue_station(data, current, battery, load, unvisited, stations)
                    if station is not None:
                        battery -= em[current][station]
                        route.append({'node': station, 'load': load, 'battery': battery})
                        battery = fuel_capacity
                        current = station
                        just_recharged = True
                        continue
                break

            battery -= em[current][nxt]
            route.append({'node': nxt, 'load': load + (0 if nxt in stations else demands[nxt]), 'battery': battery})
            if nxt in stations:
                battery = fuel_capacity  # recarga a full (misma política que evrp.py)
                just_recharged = True
            else:
                load += demands[nxt]
                unvisited.discard(nxt)
                served.append(nxt)
                just_recharged = False
            current = nxt

        # Volver al depósito: directo, o pasando antes por una estación.
        if em[current][depot] > battery:
            station = _closing_station(data, current, battery, stations)
            if station is None:
                # No debería ocurrir (lo garantiza el invariante). Por seguridad,
                # se descarta esta ruta y sus clientes vuelven a pendientes.
                unvisited.update(served)
                continue
            battery -= em[current][station]
            route.append({'node': station, 'load': load, 'battery': battery})
            battery = fuel_capacity
            current = station

        battery -= em[current][depot]
        route.append({'node': depot, 'load': load, 'battery': battery})

        if served:
            routes.append(route)

    dropped_clients = sorted(unvisited)
    total_distance = sum(_route_distance(r, cm) for r in routes)
    return routes, dropped_clients, total_distance


# ═══════════════════════════════════════════════════════════════
# 2. BÚSQUEDA LOCAL — 2-opt intra-ruta + poda de cargas innecesarias
# ═══════════════════════════════════════════════════════════════

def _route_distance(route, cost_matrix):
    return sum(cost_matrix[route[k]['node']][route[k + 1]['node']] for k in range(len(route) - 1))


def _nodes_distance(nodes, cost_matrix):
    return sum(cost_matrix[nodes[k]][nodes[k + 1]] for k in range(len(nodes) - 1))


def _rebuild_route_state(nodes, data):
    """
    Recalcula load/battery a lo largo de una secuencia de nodos ya decidida
    (tras una permutación 2-opt o quitar una estación). Devuelve None si viola
    batería (negativa) en algún punto, incluido el último tramo al depósito.
    La carga nunca se viola por reordenar dentro de la MISMA ruta (suma
    acumulada de las mismas demandas positivas).
    'battery' es el nivel al llegar a cada nodo (antes de recargar).
    """
    em = data['energy_matrix']
    demands = data['demands']
    fuel_capacity = data['fuel_capacity']
    stations_set = set(data['charging_stations'])

    if len(nodes) > 1 and nodes[0] == data['depot'] and nodes[1] in stations_set:
        return None     # del deposito no se va directo a una estacion de carga
    sb, sl = data.get('start_battery'), data.get('start_load')
    load = sl.get(nodes[0], 0) if sl else 0
    battery = sb.get(nodes[0], fuel_capacity) if sb else fuel_capacity
    route = [{'node': nodes[0], 'load': load, 'battery': battery}]
    for k in range(len(nodes) - 1):
        a, b = nodes[k], nodes[k + 1]
        battery -= em[a][b]
        if battery < 0:
            return None
        arrival = battery
        if b in stations_set:
            battery = fuel_capacity
        else:
            load += demands[b]
        route.append({'node': b, 'load': load, 'battery': arrival})
    return route


def _feasible(nodes, data):
    """True si la secuencia de nodos nunca deja la batería negativa (versión ligera de _rebuild_route_state)."""
    em = data['energy_matrix']
    cap = data['fuel_capacity']
    stations_set = data['station_set']
    if len(nodes) > 1 and nodes[0] == data['depot'] and nodes[1] in stations_set:
        return False    # del deposito no se va directo a una estacion de carga
    sb = data.get('start_battery')
    battery = cap if sb is None else sb.get(nodes[0], cap)
    for k in range(len(nodes) - 1):
        a, b = nodes[k], nodes[k + 1]
        battery -= em[a][b]
        if battery < 0:
            return False
        if b in stations_set:
            battery = cap
    return True


def _unnecessary_stops(route, data):
    """Número de paradas de carga que se podrían quitar sin dejar la ruta sin batería."""
    stations_set = data['station_set']
    nodes = [stop['node'] for stop in route]
    count = 0
    for k in range(1, len(nodes) - 1):
        if nodes[k] in stations_set and _feasible(nodes[:k] + nodes[k + 1:], data):
            count += 1
    return count


def _two_opt_route(nodes, data):
    """
    2-opt clásico dentro de una sola ruta (depósito de salida/llegada fijos):
    prueba invertir segmentos [i..j] y se queda con la mejora si sigue
    siendo factible en batería (recalculada desde cero tras cada inversión).
    Recibe y devuelve secuencias de nodos.
    """
    cm = data['cost_matrix']
    best_nodes = list(nodes)
    best_distance = _nodes_distance(best_nodes, cm)

    improved = True
    while improved:
        improved = False
        n = len(best_nodes)
        for i in range(1, n - 2):
            for j in range(i + 1, n - 1):
                candidate = best_nodes[:i] + best_nodes[i:j + 1][::-1] + best_nodes[j + 1:]
                new_dist = _nodes_distance(candidate, cm)
                if new_dist < best_distance and _feasible(candidate, data):
                    best_nodes = candidate
                    best_distance = new_dist
                    stats = data.get('ls_stats')
                    if stats is not None:
                        stats['2-opt'] = stats.get('2-opt', 0) + 1
                    improved = True
                    break
            if improved:
                break

    return best_nodes


def _prune_stations(nodes, data):
    """Quita paradas de carga innecesarias (factibles sin ellas) si la distancia no empeora."""
    cm = data['cost_matrix']
    stations_set = data['station_set']
    changed = True
    while changed:
        changed = False
        current_dist = _nodes_distance(nodes, cm)
        for k in range(1, len(nodes) - 1):
            if nodes[k] in stations_set:
                candidate = nodes[:k] + nodes[k + 1:]
                if _nodes_distance(candidate, cm) <= current_dist and _feasible(candidate, data):
                    nodes = candidate
                    changed = True
                    break
    return nodes


def _repair(nodes, data):
    """
    Si la secuencia no es factible en batería, intenta arreglarla insertando UNA
    estación en la mejor posición. Devuelve la secuencia factible o None.
    """
    if _feasible(nodes, data):
        return nodes
    cm = data['cost_matrix']
    best, best_d = None, None
    for k in range(1, len(nodes)):
        for s in data['charging_stations']:
            if s == nodes[k - 1] or s == nodes[k]:
                continue
            cand = nodes[:k] + [s] + nodes[k:]
            d = _nodes_distance(cand, cm)
            if (best_d is None or d < best_d) and _feasible(cand, data):
                best, best_d = cand, d
    return best


def _finalize_route(nodes, data):
    """Repara (si hace falta) y poda cargas innecesarias. None si no se puede dejar factible."""
    nodes = _repair(nodes, data)
    if nodes is None:
        return None
    return _prune_stations(nodes, data)


def _find_improving_move(R, loads, data, neighbor_count):
    """
    Busca (primera mejora) un movimiento ENTRE rutas que reduzca la distancia:
      - relocate / Or-opt: mover un segmento de 1 a 3 clientes consecutivos a
        otra ruta (en ambos sentidos), junto a uno de los vecinos cercanos de
        sus extremos;
        Tambien dentro de la misma ruta (relocate-intra).
      - swap: intercambiar dos clientes de rutas distintas;
      - 2-opt*: intercambiar las colas de dos rutas.
    El delta aproximado se calcula en O(1); el movimiento se acepta solo si,
    tras reparar batería (insertando estación si hace falta) y podar cargas
    innecesarias, la distancia REAL baja y capacidad/batería se cumplen.
    Devuelve (a, b, nueva_ruta_a, nueva_ruta_b) o None.
    """
    cm = data['cost_matrix']
    demands = data['demands']
    vcap = data['vehicle_capacity']
    stations_set = data['station_set']
    neighbors = data['client_neighbors']

    dl = data.get('depot_like') or {data['depot']}      # deposito (y posiciones de arranque en re-planeacion)
    sl = data.get('start_load')
    pos = {}
    for r_idx, route in enumerate(R):
        for k, node in enumerate(route):
            if node not in stations_set and node not in dl:
                pos[node] = (r_idx, k)
    prefix = []     # prefix[r][k] = carga acumulada de R[r][:k+1] (para 2-opt*)
    for route in R:
        acc, row = (sl.get(route[0], 0) if sl else 0), []
        for node in route:
            acc += demands[node]
            row.append(acc)
        prefix.append(row)

    for a, ra in enumerate(R):
        dist_a = _nodes_distance(ra, cm)
        for i in range(1, len(ra) - 1):
            for L in (1, 2, 3):
                if i + L > len(ra) - 1:
                    break
                seg = ra[i:i + L]
                if any(n in stations_set or n in dl for n in seg):
                    break
                seg_load = sum(demands[n] for n in seg)
                p, nx = ra[i - 1], ra[i + L]
                gain = cm[p][seg[0]] + cm[seg[-1]][nx] - cm[p][nx]
                new_a_base = ra[:i] + ra[i + L:]

                for end_idx, e in ((0, seg[0]), (L - 1, seg[-1])):
                    for u in neighbors[e][:neighbor_count]:
                        if u not in pos or u in seg:
                            continue
                        b, ku = pos[u]
                        if b == a:
                            # relocate DENTRO de la misma ruta: se inserta en la ruta
                            # sin el segmento (new_a_base), junto a u
                            ku_b = ku if ku < i else ku - L
                            for j, oriented in (
                                (ku_b + 1, seg if end_idx == 0 else seg[::-1]),
                                (ku_b, seg[::-1] if end_idx == 0 else seg),
                            ):
                                left, right = new_a_base[j - 1], new_a_base[j]
                                add = cm[left][oriented[0]] + cm[oriented[-1]][right] - cm[left][right]
                                if add - gain >= 0:
                                    continue
                                cand = new_a_base[:j] + list(oriented) + new_a_base[j:]
                                if cand == ra:
                                    continue
                                new_r = _finalize_route(cand, data)
                                if new_r is not None and _nodes_distance(new_r, cm) < dist_a:
                                    return a, a, new_r, None, 'relocate-intra'
                            continue
                        if loads[b] + seg_load > vcap:
                            continue
                        rb = R[b]
                        # insertar DESPUES de u (e queda pegado a u) y ANTES de u
                        for j, oriented in (
                            (ku + 1, seg if end_idx == 0 else seg[::-1]),
                            (ku, seg[::-1] if end_idx == 0 else seg),
                        ):
                            left, right = rb[j - 1], rb[j]
                            add = cm[left][oriented[0]] + cm[oriented[-1]][right] - cm[left][right]
                            if add - gain >= 0:
                                continue
                            new_b = _finalize_route(rb[:j] + list(oriented) + rb[j:], data)
                            if new_b is None:
                                continue
                            new_a = _finalize_route(new_a_base, data)
                            if new_a is None:
                                continue
                            before = dist_a + _nodes_distance(rb, cm)
                            after = _nodes_distance(new_a, cm) + _nodes_distance(new_b, cm)
                            if after < before:
                                return a, b, new_a, new_b, ('relocate' if L == 1 else 'or-opt')

        # swap de clientes sueltos
        for i in range(1, len(ra) - 1):
            x = ra[i]
            if x in stations_set:
                continue
            for y in neighbors[x][:neighbor_count]:
                if y not in pos:
                    continue
                b, j = pos[y]
                if b <= a:
                    continue
                rb = R[b]
                if loads[a] - demands[x] + demands[y] > vcap or loads[b] - demands[y] + demands[x] > vcap:
                    continue
                p, nx = ra[i - 1], ra[i + 1]
                q, m = rb[j - 1], rb[j + 1]
                delta = (cm[p][y] + cm[y][nx] - cm[p][x] - cm[x][nx]
                         + cm[q][x] + cm[x][m] - cm[q][y] - cm[y][m])
                if delta >= 0:
                    continue
                new_a = _finalize_route(ra[:i] + [y] + ra[i + 1:], data)
                new_b = _finalize_route(rb[:j] + [x] + rb[j + 1:], data)
                if new_a is None or new_b is None:
                    continue
                if _nodes_distance(new_a, cm) + _nodes_distance(new_b, cm) < dist_a + _nodes_distance(rb, cm):
                    return a, b, new_a, new_b, 'swap'

        # 2-opt*: intercambiar las colas de dos rutas distintas. Se corta ra
        # despues de ra[i] y rb justo antes de y (vecino cercano de ra[i]), asi
        # la nueva arista ra[i] -> y es corta.
        for i in range(len(ra) - 1):
            x, nx_a = ra[i], ra[i + 1]
            for y in neighbors[x][:neighbor_count]:
                if y not in pos:
                    continue
                b, ky = pos[y]
                if b == a:
                    continue
                rb = R[b]
                j = ky - 1
                pre_a, pre_b = prefix[a][i], prefix[b][j]
                if pre_a + (loads[b] - pre_b) > vcap or pre_b + (loads[a] - pre_a) > vcap:
                    continue
                delta = cm[x][y] + cm[rb[j]][nx_a] - cm[x][nx_a] - cm[rb[j]][y]
                if delta >= 0:
                    continue
                new_a = _finalize_route(ra[:i + 1] + rb[ky:], data)
                new_b = _finalize_route(rb[:j + 1] + ra[i + 1:], data)
                if new_a is None or new_b is None:
                    continue
                if _nodes_distance(new_a, cm) + _nodes_distance(new_b, cm) < dist_a + _nodes_distance(rb, cm):
                    return a, b, new_a, new_b, '2opt*'
    return None


def _route_loads(R, data):
    """Carga de cada ruta (incluye la que el vehiculo ya llevaba al arrancar en re-planeacion)."""
    demands, sl = data['demands'], data.get('start_load')
    if not sl:
        return [sum(demands[n] for n in r) for r in R]
    return [sl.get(r[0], 0) + sum(demands[n] for n in r) for r in R]


def _inter_route_search(R, data, neighbor_count=8, max_moves=300):
    """Aplica movimientos entre rutas (relocate/Or-opt/swap) mientras mejoren."""
    demands = data['demands']
    R = [list(r) for r in R]
    for _ in range(max_moves):
        loads = _route_loads(R, data)
        move = _find_improving_move(R, loads, data, neighbor_count)
        if move is None:
            break
        a, b, new_a, new_b, kind = move
        stats = data.get('ls_stats')
        if stats is not None:
            stats[kind] = stats.get(kind, 0) + 1
        R[a] = new_a
        if new_b is not None:
            R[b] = new_b
        R = [r for r in R if len(r) > 2]   # ruta que quedó vacía (solo depósito) desaparece
    return R


def local_search(routes, data, rounds=3):
    """
    Búsqueda local completa: movimientos entre rutas (relocate, Or-opt, swap)
    + 2-opt intra-ruta + poda de cargas innecesarias, repetido hasta que no
    mejora. Devuelve (rutas_mejoradas, distancia_total).
    """
    cm = data['cost_matrix']
    R = [[stop['node'] for stop in r] for r in routes]
    best_total = sum(_nodes_distance(r, cm) for r in R)
    for _ in range(rounds):
        R = _inter_route_search(R, data)
        R = [_prune_stations(_two_opt_route(r, data), data) for r in R]
        total = sum(_nodes_distance(r, cm) for r in R)
        if total >= best_total:
            best_total = min(total, best_total)
            break
        best_total = total

    rebuilt = [_rebuild_route_state(r, data) for r in R]
    if any(r is None for r in rebuilt):
        rebuilt = routes   # no debería pasar; se conserva la solución original
    total_distance = sum(_route_distance(r, cm) for r in rebuilt)
    return rebuilt, total_distance


# ═══════════════════════════════════════════════════════════════
# 2b. PERTURBACIÓN (ILS) sobre la mejor solución: destruir + reconstruir
# ═══════════════════════════════════════════════════════════════

def _perturb_and_repair(routes, data, rng, max_tries=8):
    """
    Destruye una parte de la solución y la reconstruye con inserción más barata:
      - 50 %: quita todos los clientes de una ruta al azar;
      - 50 %: quita un cliente al azar y sus m vecinos más cercanos
              (m entre n/15 y n/6, mínimo 3).
    Los clientes quitados se reinsertan (orden aleatorio) en la posición de
    menor costo que respete capacidad y batería (con inserción de estación
    si hace falta); si hay vehículos libres, también pueden abrir una ruta
    nueva. Devuelve rutas (con estado de batería/carga) o None si algún
    cliente no se pudo reinsertar de forma factible.
    """
    cm = data['cost_matrix']
    demands = data['demands']
    vcap = data['vehicle_capacity']
    stations_set = data['station_set']
    depot = data['depot']
    neighbors = data['client_neighbors']

    R = [[s['node'] for s in r] for r in routes]
    dl = data.get('depot_like') or {depot}
    clients = [x for r in R for x in r if x not in stations_set and x not in dl]
    n = len(clients)
    if n < 4:
        return None

    if len(R) > 1 and rng.random() < 0.5:
        ridx = rng.randrange(len(R))
        removed = [x for x in R[ridx] if x not in stations_set and x not in dl]
    else:
        seed_client = rng.choice(clients)
        m = rng.randint(max(3, n // 15), max(5, n // 6))
        removed = [seed_client] + neighbors[seed_client][:m - 1]
    removed_set = set(removed)

    new_R = []
    for r in R:
        kept = [x for x in r if x not in removed_set]
        if len(kept) > 2:
            fixed = _finalize_route(kept, data)
            if fixed is None:
                return None
            new_R.append(fixed)
    R = new_R
    loads = _route_loads(R, data)
    starts = data.get('vehicle_starts')
    sl = data.get('start_load') or {}

    rng.shuffle(removed)
    for c in removed:
        options = []   # (delta, indice_ruta o -1 para ruta nueva, posicion)
        for r_idx, r in enumerate(R):
            if loads[r_idx] + demands[c] > vcap:
                continue
            for j in range(1, len(r)):
                left, right = r[j - 1], r[j]
                options.append((cm[left][c] + cm[c][right] - cm[left][right], r_idx, j, None))
        if starts:
            used = {r[0] for r in R}
            for stt in starts:      # vehiculos sin ruta: pueden abrir una nueva desde SU posicion de arranque
                if stt['node'] not in used and demands[c] + stt['load'] <= vcap:
                    options.append((cm[stt['node']][c] + cm[c][depot], -1, 1, stt['node']))
        elif len(R) < data['num_vehicles']:
            options.append((cm[depot][c] + cm[c][depot], -1, 1, depot))
        options.sort(key=lambda t: t[0])

        placed = False
        for delta, r_idx, j, new_start in options[:max_tries]:
            if r_idx == -1:
                cand = _finalize_route([new_start, c, depot], data)
                if cand is not None:
                    R.append(cand)
                    loads.append(demands[c] + sl.get(new_start, 0))
                    placed = True
                    break
            else:
                cand = _finalize_route(R[r_idx][:j] + [c] + R[r_idx][j:], data)
                if cand is not None:
                    R[r_idx] = cand
                    loads[r_idx] += demands[c]
                    placed = True
                    break
        if not placed:
            return None

    rebuilt = [_rebuild_route_state(r, data) for r in R]
    if any(r is None for r in rebuilt):
        return None
    return rebuilt


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


def _mmas_bounds(best_cost, rho, num_locations):
    """Límites MAX-MIN (Stützle & Hoos): tau_max = 1/(rho*L_best), tau_min = tau_max/(2n)."""
    tau_max = 1.0 / (rho * max(best_cost, 1e-9))
    return tau_max / (2.0 * num_locations), tau_max


def _update_pheromone_mmas(pheromone, iter_best, best, rho, iteration, gb_period):
    """
    MAX-MIN Ant System: evapora y deposita SOLO la mejor hormiga (la de la
    iteración; cada `gb_period` iteraciones, la mejor global). La feromona se
    acota a [tau_min, tau_max] para evitar estancamiento prematuro.
    iter_best / best: (routes, cost).
    """
    n = len(pheromone)
    tau_min, tau_max = _mmas_bounds(best[1], rho, n)
    for i in range(n):
        row = pheromone[i]
        for j in range(n):
            row[j] *= (1.0 - rho)
    routes, cost = best if (gb_period and iteration % gb_period == 0) else iter_best
    if cost > 0:
        _deposit(pheromone, routes, 1.0 / cost)
    for i in range(n):
        row = pheromone[i]
        for j in range(n):
            row[j] = min(max(row[j], tau_min), tau_max)
    return tau_min, tau_max


# ═══════════════════════════════════════════════════════════════
# 4. COSTO
#    objective = distancia + drop_penalty por cliente descartado (idéntico al
#    objetivo de evrp.py). cost (interno) = objective + penalización por
#    paradas de carga innecesarias; guía la búsqueda pero NO se reporta.
# ═══════════════════════════════════════════════════════════════

def _drop_penalty(data):
    n = data['num_locations']
    cm = data['cost_matrix']
    max_distance = max(cm[i][j] for i in range(n) for j in range(n) if i != j)
    return max(10_000_000, int(max_distance) * 100)


def _idle_return_cost(data, routes):
    """Re-planeacion: un vehiculo sin clientes igual vuelve de su posicion al deposito (OR-Tools cuenta ese arco)."""
    starts = data.get('vehicle_starts')
    if not starts:
        return 0
    used = {r[0]['node'] for r in routes}
    cm, depot = data['cost_matrix'], data['depot']
    return sum(cm[st['node']][depot] for st in starts if st['node'] not in used)


def _objective(data, routes, dropped_clients):
    distance = sum(_route_distance(r, data['cost_matrix']) for r in routes) + _idle_return_cost(data, routes)
    return distance + (_drop_penalty(data) * len(dropped_clients) if dropped_clients else 0)


def _solution_cost(data, routes, dropped_clients):
    unnecessary = sum(_unnecessary_stops(r, data) for r in routes)
    return _objective(data, routes, dropped_clients) + data['charge_penalty'] * unnecessary


# ═══════════════════════════════════════════════════════════════
# 5. GUARDAR SOLUCIÓN (mismo formato de texto que evrp.py::save_solution)
# ═══════════════════════════════════════════════════════════════

def save_solution(data, routes, dropped_clients, objective, instance,
                   elapsed_time, i, distance_type, num_iterations, heuristic=None):
    # con heuristica inicial el nombre es <HEURISTICA>_and_ACO (igual que <HEURISTICA>_and_<MH> en OR-Tools)
    heuristic = getattr(heuristic, "name", heuristic)   # solo el nombre (SAVINGS), no HeuristicType.SAVINGS
    distance_type_str, solution_name = get_distance_and_solution_name(distance_type, heuristic, "ACO")
    output_dir = os.path.join(f"problem/{distance_type_str}/solutions_evrp_{i}/solutions_{solution_name}")

    try:
        os.makedirs(output_dir, exist_ok=True)
    except OSError as error:
        print(f"Error creating directory {output_dir}: {error}")
        return

    filename = os.path.join(output_dir, f'{instance}')
    charging_set = set(data['charging_stations'])
    cs_names = data.get('charging_station_names', {})
    cm = data['cost_matrix']
    time_matrix = data.get('time_matrix')
    depot = data['depot']
    fuel_capacity = data['fuel_capacity']

    # Un bloque por vehículo de la flota (como OR-Tools): los no usados salen
    # como ruta vacía depósito -> depósito.
    all_routes = list(routes)
    starts = data.get('vehicle_starts')
    if starts:
        # re-planeacion: la ruta del vehiculo k es la que arranca en SU posicion; si no tiene clientes va directo al deposito
        by_start = {r[0]['node']: r for r in routes}
        all_routes = []
        for stt in starts:
            if stt['node'] in by_start:
                all_routes.append(by_start[stt['node']])
            else:
                left = stt['battery'] - data['energy_matrix'][stt['node']][depot]
                all_routes.append([{'node': stt['node'], 'load': stt['load'], 'battery': stt['battery']},
                                   {'node': depot, 'load': stt['load'], 'battery': left}])
    while len(all_routes) < data['num_vehicles']:
        all_routes.append([
            {'node': depot, 'load': 0, 'battery': fuel_capacity},
            {'node': depot, 'load': 0, 'battery': fuel_capacity},
        ])

    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(f'Instance: {instance}\n\n')
            f.write(f'Objective: {int(objective)}\n\n')
            f.write(f'Execution Time: {elapsed_time}\n\n')
            if heuristic:
                f.write(f'Heuristic: {heuristic}\n\n')      # ACO + heuristica de OR-Tools (la re-planeacion la lee de aqui)
            f.write(f'Metaheuristic: ACO ({num_iterations} iteraciones)\n\n')
            f.write(f'Distance type: {distance_type_str}\n\n')
            f.write(f'Vehicle capacity: {data["vehicle_capacity"]}\n')
            f.write(f'Battery capacity: {data["fuel_capacity"]}\n')
            f.write(f'Battery consumption rate: {data["fuel_consumption_rate"]}\n\n')

            if dropped_clients:
                f.write(f'Dropped clients: {dropped_clients}\n\n')
            if data.get('extra_info'):
                f.write(f'{data["extra_info"]}\n\n')

            total_distance = 0
            total_load = 0
            total_time = 0
            total_recharges = 0
            min_soc = None

            for vehicle_id, route in enumerate(all_routes):
                plan_output = f'Route for vehicle {vehicle_id}:\n'
                route_distance = 0
                route_time = 0

                for k in range(len(route) - 1):
                    node = route[k]['node']
                    soc = int(route[k]['battery'])
                    min_soc = soc if min_soc is None else min(min_soc, soc)
                    if starts and node in data['depot_like'] and node != depot:
                        plan_output += f' {node}[CS:{cs_names.get(node, node)}] Load({route[k]["load"]}) Bat({soc}) ->'     # posicion actual del vehiculo (como en OR-Tools)
                    elif node in charging_set:
                        total_recharges += 1
                        cs_label = f'[CS:{cs_names.get(node, node)}]'
                        plan_output += f' {node}{cs_label} Load({route[k]["load"]}) Bat({soc}) ->'
                    else:
                        plan_output += f' {node} Load({route[k]["load"]}) Bat({soc}) ->'
                    nxt = route[k + 1]['node']
                    route_distance += cm[node][nxt]
                    if time_matrix is not None:
                        route_time += int(time_matrix[node][nxt])

                last = route[-1]
                last_soc = int(last['battery'])
                min_soc = last_soc if min_soc is None else min(min_soc, last_soc)
                plan_output += f' {last["node"]} Load({last["load"]}) Bat({last_soc})\n'
                plan_output += f'Distance of the route: {route_distance}km\n'
                if time_matrix is not None:
                    plan_output += f'Time of the route: {route_time}s\n'
                plan_output += f'Load of the route: {last["load"]}\n\n'

                f.write(plan_output)
                total_distance += route_distance
                total_load += last['load']
                total_time += route_time

            f.write(f'Total Distance of all routes: {total_distance}km\n\n')
            f.write(f'Total Load of all routes: {total_load}\n\n')
            if time_matrix is not None:
                f.write(f'Total Time of all routes: {total_time}s\n\n')
            f.write(f'Total recharges: {total_recharges}\n\n')
            if min_soc is not None:
                f.write(f'Minimum battery (SoC) reached: {min_soc}\n\n')
            prior_km = data.get('prior_distance_km')
            if prior_km:
                f.write(f'Distance already traveled before this sub-problem: {prior_km:.3f}km\n\n')
                f.write(f'Total Distance including distance before event: {total_distance + prior_km:.3f}km\n\n')

        print(f"Solution saved successfully in {filename}")
    except OSError as error:
        print(f"Error writing to file {filename}: {error}")


# ═══════════════════════════════════════════════════════════════
# 6. PUNTO DE ENTRADA
# ═══════════════════════════════════════════════════════════════

# Búsqueda local en paralelo (workers > 1): los datos de la instancia se envían
# UNA vez a cada proceso (initializer); por tarea solo viajan las rutas.
_WORKER_DATA = None


def _init_worker(data):
    global _WORKER_DATA
    _WORKER_DATA = data


def _local_search_worker(routes):
    return local_search(routes, _WORKER_DATA)

def _setup_vehicle_states(data, vehicle_states):
    """
    Re-planeacion dinamica: cada vehiculo arranca en su posicion actual (nodo virtual start_node) con su bateria y
    carga parciales y termina en el deposito. Las posiciones de arranque llegan en data['charging_stations'] (asi las
    trata OR-Tools); aqui se separan: dejan de ser estaciones y pasan a ser nodos "tipo deposito" que nadie visita.
    """
    if len(vehicle_states) != data['num_vehicles']:
        raise ValueError(f"vehicle_states debe tener un elemento por vehiculo ({data['num_vehicles']}), recibi {len(vehicle_states)}.")
    depot = data['depot']
    cap = data['fuel_capacity']
    starts = [{'node': vs.get('start_node', depot), 'battery': int(vs.get('initial_fuel', cap)),
               'load': int(vs.get('initial_load', 0))} for vs in vehicle_states]
    start_nodes = {st['node'] for st in starts}
    if len(start_nodes) != len(starts):
        raise ValueError("Cada vehiculo debe tener un start_node distinto (ACO identifica la ruta por su nodo de arranque).")
    data['num_clients'] = data['num_locations'] - len(data['charging_stations']) - 1
    data['charging_stations'] = [s for s in data['charging_stations'] if s not in start_nodes]
    data['vehicle_starts'] = starts
    data['start_battery'] = {st['node']: st['battery'] for st in starts}
    data['start_load'] = {st['node']: st['load'] for st in starts}
    data['depot_like'] = {depot} | start_nodes


def execute(
        i, instance_type, time_limit,
        vehicle_maximum_travel_distance=None,   # -> fuel_capacity
        vehicle_speed=None,                     # -> fuel_consumption_rate (None = según distance_type)
        distance_type: DistanceType = None,
        num_ants=None,
        alpha=None,      # importancia de la feromona (None = automatico: 1.0 si n<=50 nodos, 2.0 si n>50; ver ablacion en el informe)
        beta=1.5,        # importancia de la heuristica (1/distancia)
        gamma=2.5,       # importancia del termino de energia (Thymianis et al.)
        rho=0.5,         # tasa de evaporacion
        rank_w=6,        # Rank-AS: depositan las (w-1) mejores hormigas + best-so-far
        proactive_charge_threshold=0.15,  # % de fuel_capacity por debajo del cual se considera cargar
        seed=None,
        candidate_list_size=20,   # clientes factibles mas cercanos evaluados por paso de construccion
        local_search_top=3,       # a cuantas de las mejores hormigas de cada iteracion se les aplica busqueda local
        pheromone_scheme='mmas',  # 'mmas' (MAX-MIN Ant System, por defecto) o 'rank_as' (Nie et al.)
        gb_period=5,              # MMAS: cada cuantas iteraciones deposita la mejor global en vez de la de la iteracion
        workers=1,                # >1: la busqueda local de las mejores hormigas corre en paralelo (procesos)
        ils_per_iter=2,           # perturbaciones (destruir+reconstruir+busqueda local) de la mejor solucion por iteracion (0 = apagado)
        restart_patience=0,       # reinicia la feromona tras N iteraciones sin mejorar el mejor del ciclo (0 = nunca; con 10 empeoro los resultados)
        log_progress=False,       # imprime cada mejora del mejor global con su tiempo (curva de convergencia)
        save_files=True,          # False = no escribe el archivo de solucion (lo usa explainability/ para generar muchas corridas)
        verbose=True,             # False = sin mensajes de consola
        initial_heuristic=None,   # opcional: heuristica constructiva de OR-Tools (HeuristicType o nombre) que aporta la solucion inicial
        initial_time_limit=2,     # segundos que se le dan a esa heuristica
        vehicle_states=None,      # re-planeacion dinamica: por vehiculo {'start_node','initial_fuel','initial_load'} (como en evrp.py)
        data_override=None,       # re-planeacion dinamica: dict de datos ya construido (salta process_files); ver evrp.py
        instance_name=None,       # nombre de la instancia/archivo de salida cuando se usa data_override
):
    """Devuelve {instancia: {'objective', 'distance', 'n_dropped', 'n_routes', 'recharges', 'iterations', 'elapsed', 'alpha_used'}}."""
    results = {}
    if initial_heuristic is not None:
        from p_mh.ACO import ortools_seed as ortools_seed_mod
        initial_heuristic = ortools_seed_mod.as_heuristic(initial_heuristic)
    if vehicle_speed is None and data_override is not None:
        vehicle_speed = data_override.get('fuel_consumption_rate')
    if vehicle_speed is None:
        vehicle_speed = DEFAULT_CONSUMPTION_RATE.get(distance_type)
        if vehicle_speed is None:
            raise ValueError(
                "Pasa vehicle_speed (consumo de batería por unidad de distancia): no hay un "
                f"valor por defecto para distance_type={distance_type}. "
                "Por defecto: 3.0 con OSRM, 6.0 con Haversine."
            )
        if verbose:
            print(f"vehicle_speed no indicado -> consumo por defecto {vehicle_speed} para {distance_type.value}")

    if data_override is not None:
        instances_data = {instance_name or "dynamic_replan": data_override}
    else:
        instances_data = process_files(
            instance_type, distance_type,
            None, vehicle_speed, vehicle_maximum_travel_distance
        )

    for instance, data in instances_data.items():
        if vehicle_maximum_travel_distance is not None:
            data['fuel_capacity'] = int(vehicle_maximum_travel_distance)
        data['fuel_consumption_rate'] = vehicle_speed

        if 'fuel_capacity' not in data:
            raise KeyError(
                "El diccionario de datos no contiene 'fuel_capacity'. "
                "Asegurate de pasar vehicle_maximum_travel_distance al llamar a execute()."
            )

        if vehicle_states is not None:
            _setup_vehicle_states(data, vehicle_states)
        _prepare_matrices(data)
        if log_progress:
            data['ls_stats'] = {}   # cuenta cuantas veces mejora cada operador de busqueda local

        # Con pocas iteraciones disponibles (instancias grandes, Python) la
        # feromona debe pesar mas; con instancias chicas (muchas iteraciones)
        # un alpha alto la satura y empeora (medido: 30c con alpha=3 -> +6.7%).
        alpha = alpha if alpha is not None else (1.0 if data['num_locations'] <= 50 else 2.0)

        rng = random.Random(seed)
        ants = num_ants or min(30, max(10, data['num_locations']))
        w = min(rank_w, ants + 1)
        pheromone = _init_pheromone(data['num_locations'])

        # Strategy: solucion inicial (NoInitial / ORToolsHeuristic) y regla de feromona (MMAS / Rank-AS)
        seed_solution = make_initial_strategy(initial_heuristic, initial_time_limit).build(data, verbose)
        pheromone_strategy = make_pheromone_strategy(
            pheromone_scheme, rho, data['num_locations'], gb_period, w)

        best_routes, best_dropped, best_cost = None, None, float('inf')
        executor = None
        if workers and workers > 1:
            from concurrent.futures import ProcessPoolExecutor
            executor = ProcessPoolExecutor(max_workers=workers, initializer=_init_worker, initargs=(data,))
        start_time = time.time()
        iterations = 0
        cycle_best, stall, cycle_iter, restarts = None, 0, 0, 0

        if verbose:
            print(f"Corriendo ACO sobre '{instance}' ({ants} hormigas, limite {time_limit}s, "
                  f"consumo {vehicle_speed}, bateria {data['fuel_capacity']})...")

        while time.time() - start_time < time_limit:
            ants_results = []
            ant_solutions = []  # (routes, dropped, cost)
            for _ in range(ants):
                routes, dropped, _total = construct_solution(
                    data, pheromone, alpha, beta, gamma, rng, proactive_charge_threshold,
                    candidate_list_size
                )
                cost = _solution_cost(data, routes, dropped)
                ants_results.append((routes, cost))
                ant_solutions.append((routes, dropped, cost))

            # Busqueda local (entre rutas + 2-opt + poda de cargas) sobre las
            # `local_search_top` mejores hormigas. No cambia que clientes se
            # visitan, asi que dropped_clients sigue siendo valido.
            if seed_solution is not None:
                ant_solutions.append(seed_solution)   # compite como una hormiga mas (solo en la 1a iteracion)
                seed_solution = None
            ant_solutions.sort(key=lambda t: t[2])
            iter_best = ant_solutions[0]
            top = ant_solutions[:max(1, local_search_top)]
            if executor is not None and len(top) > 1:
                ls_results = list(executor.map(_local_search_worker, [t[0] for t in top]))
            else:
                ls_results = [local_search(t[0], data) for t in top]
            for (routes, dropped, cost), (ls_routes, _ls_distance) in zip(top, ls_results):
                ls_cost = _solution_cost(data, ls_routes, dropped)
                if ls_cost < cost:
                    ants_results.append((ls_routes, ls_cost))  # que la feromona tambien aprenda de la version mejorada
                    if ls_cost < iter_best[2]:
                        iter_best = (ls_routes, dropped, ls_cost)

            if iter_best[2] < best_cost:
                best_routes, best_dropped, best_cost = iter_best
                if log_progress:
                    print(f"  PROGRESS t={time.time() - start_time:.2f}s it={iterations} best={_objective(data, best_routes, best_dropped)}")

            # Perturbacion (ILS) sobre la mejor solucion global: destruir una
            # parte, reconstruir con insercion barata y pulir con busqueda local.
            # Se acepta si no empeora (igual costo = otra solucion equivalente).
            if ils_per_iter and best_routes is not None and not best_dropped:
                for _ in range(ils_per_iter):
                    cand = _perturb_and_repair(best_routes, data, rng)
                    if cand is None:
                        continue
                    c_routes, _c_dist = local_search(cand, data)
                    c_cost = _solution_cost(data, c_routes, [])
                    if c_cost <= best_cost:
                        if c_cost < best_cost and log_progress:
                            print(f"  PROGRESS(ILS) t={time.time() - start_time:.2f}s it={iterations} best={_objective(data, c_routes, [])}")
                        best_routes, best_cost = c_routes, c_cost
                        ants_results.append((c_routes, c_cost))
                        if c_cost < iter_best[2]:
                            iter_best = (c_routes, [], c_cost)

            # Mejor del "ciclo" actual (desde el ultimo reinicio de feromona).
            # La feromona aprende del ciclo, no del mejor global, para que tras
            # un reinicio la busqueda explore otra zona en vez de volver al mismo optimo.
            if cycle_best is None or iter_best[2] < cycle_best[2]:
                cycle_best = iter_best
                stall = 0
            else:
                stall += 1

            pheromone = pheromone_strategy.update(pheromone, ants_results, iter_best, cycle_best, cycle_iter)
            iterations += 1
            cycle_iter += 1

            # Reinicio de feromona por estancamiento
            if restart_patience and stall >= restart_patience:
                pheromone = _init_pheromone(data['num_locations'])
                cycle_best, stall, cycle_iter = None, 0, 0
                restarts += 1
                if log_progress:
                    print(f"  RESTART t={time.time() - start_time:.2f}s it={iterations}")

        elapsed_time = time.time() - start_time
        if executor is not None:
            executor.shutdown()

        if best_routes is None:
            if verbose:
                print(f"ACO no encontro ninguna solucion para la instancia {instance}!")
            continue

        if log_progress:
            print(f"  LS_STATS {data['ls_stats']} restarts={restarts}")
        objective = _objective(data, best_routes, best_dropped)
        if verbose:
            print(f"  {iterations} iteraciones, mejor objetivo: {objective}"
                  + (f" (incluye penalizacion por {len(best_dropped)} cliente(s) descartado(s))" if best_dropped else ""))

        if save_files:
            save_solution(
                data, best_routes, best_dropped, objective, instance,
                elapsed_time, i, distance_type, iterations,
                heuristic=initial_heuristic
            )

        stations_set = set(data['charging_stations'])
        results[instance] = {
            'objective': objective,                                   # distancia + penalizacion por descartes
            'distance': _objective(data, best_routes, []),            # solo distancia
            'n_dropped': len(best_dropped),
            'n_routes': len(best_routes),
            'recharges': sum(1 for r in best_routes for s in r if s['node'] in stations_set),
            'iterations': iterations,
            'elapsed': elapsed_time,
            'alpha_used': alpha,
        }
    return results
