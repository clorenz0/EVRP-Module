"""
feasibility.py -- comprueba, de forma independiente del solucionador, que una solucion EVRP cumple TODAS las restricciones.

Se usa al guardar cada solucion (OR-Tools y ACO) y deja el resultado en el .txt ('Feasibility check: ...'), de modo que
una solucion con clientes sin atender, bateria negativa, sobrecarga, etc. nunca pase por valida sin que se vea.

Restricciones que verifica
  1. Cada cliente se atiende exactamente una vez (sin faltantes ni repetidos).
  2. Cada ruta sale de su nodo de arranque (el deposito, o la posicion del vehiculo al re-planificar) y termina en el deposito.
  3. La carga acumulada nunca supera la capacidad del vehiculo.
  4. La bateria nunca queda negativa (incluido el tramo de regreso al deposito); cada estacion la deja llena.
  5. Del deposito no se va directo a una estacion, ni de una estacion a otra.
  6. Una estacion no se visita mas veces que sus copias (solo OR-Tools; ACO no tiene tope).
Las comprobaciones usan las mismas conversiones enteras (metros) que los solucionadores.
"""
from collections import Counter

from utils.units import cap_int, energy_int


def check_routes(data, routes, n_clients, copies=None):
    """
    data   : diccionario de datos del problema (distance_matrix en km, demands, depot, charging_stations, ...)
    routes : lista de rutas; cada ruta es la lista de nodos visitados con ids ORIGINALES (copias de estacion ya mapeadas)
    n_clients : numero de clientes (ids 1..n_clients)
    copies : visitas maximas por estacion (None = sin tope)
    Devuelve la lista de violaciones (vacia = solucion factible).
    """
    problems = []
    dm = data['distance_matrix']
    depot = data['depot']
    rate = data['fuel_consumption_rate']
    cap = cap_int(data.get('fuel_capacity_units', data['fuel_capacity']))
    veh_cap = data['vehicle_capacity']
    demands = data['demands']
    origin = data.get('station_origin', {})
    start_battery = data.get('start_battery') or {}
    start_load = data.get('start_load') or {}
    starts = set(start_battery) | set(start_load)
    stations = {origin.get(s, s) for s in data['charging_stations']}

    # 1. clientes atendidos una sola vez
    seen = Counter(n for r in routes for n in r if 1 <= n <= n_clients)
    missing = [c for c in range(1, n_clients + 1) if seen[c] == 0]
    repeated = sorted(c for c, k in seen.items() if k > 1)
    if missing:
        problems.append(f"{len(missing)} customer(s) not served: {missing[:10]}{'...' if len(missing) > 10 else ''}")
    if repeated:
        problems.append(f"{len(repeated)} customer(s) served more than once: {repeated[:10]}")

    visits = Counter()
    for k, r in enumerate(routes):
        if len(r) < 2:
            continue
        # 2. arranque y fin
        if r[0] != depot and r[0] not in starts:
            problems.append(f"route {k} does not start at the depot (starts at node {r[0]})")
        if r[-1] != depot:
            problems.append(f"route {k} does not end at the depot (ends at node {r[-1]})")
        load = start_load.get(r[0], 0)
        battery = start_battery.get(r[0], cap)
        for step, (a, b) in enumerate(zip(r, r[1:])):
            # 5. arcos prohibidos (la posicion de arranque de un vehiculo en re-planeacion no cuenta como estacion)
            a_is_station = a in stations and not (step == 0 and a in starts)
            if a == depot and b in stations and b not in starts:
                problems.append(f"route {k}: depot -> station {b}")
            if a_is_station and b in stations and b not in starts:
                problems.append(f"route {k}: station {a} -> station {b}")
            # 4. bateria
            battery -= energy_int(dm[a][b], rate)
            if battery < 0:
                problems.append(f"route {k}: battery below zero on arc {a} -> {b}")
                battery = 0     # se sigue para contar otras violaciones sin repetir esta
            if b in stations and b not in starts:
                battery = cap
                visits[b] += 1
            elif b != depot:
                load += demands[b]
                # 3. capacidad
                if load > veh_cap:
                    problems.append(f"route {k}: load {load} exceeds vehicle capacity {veh_cap} at node {b}")
                    load = veh_cap
    # 6. visitas por estacion
    if copies:
        over = {s: v for s, v in visits.items() if v > copies}
        if over:
            problems.append(f"stations visited more than {copies} times: {over}")
    return problems


def report_lines(problems, limit=10):
    """Lineas para el .txt: 'Feasibility check: PASSED' o FAILED con el detalle (en ingles, como el resto del archivo)."""
    if not problems:
        return ["Feasibility check: PASSED (all customers served, capacity, battery and routing rules satisfied)"]
    lines = [f"Feasibility check: FAILED ({len(problems)} violation(s))"]
    lines += [f"  - {p}" for p in problems[:limit]]
    if len(problems) > limit:
        lines.append(f"  - ... and {len(problems) - limit} more")
    return lines
