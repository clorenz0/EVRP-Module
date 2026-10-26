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

Función objetivo: distancia total (costo de arco = data['distance_matrix'], en METROS enteros: ver utils/units.py;
los archivos de solución vuelven a km con 3 decimales y la batería a unidades con decimales).
Todas las distancias usadas en el modelo (costo de arco, batería, penalización de
descarte) provienen de la MISMA fuente: data['distance_matrix']. No se recalcula
la distancia por otro camino en ningún punto del módulo.

data['time_matrix'] solo está disponible cuando la instancia se cargó con
DistanceType.OSRM (viene del servicio /table de OSRM, ver distance/osrm_client.py).
Con otros DistanceType es None. No se usa en el costo de arco.

Visitas repetidas a una estación (station_copies) y paradas innecesarias
───────────────────────────────────────────────────────────────────────────
En OR-Tools un nodo solo se visita una vez, de modo que una estación física
era utilizable una sola vez por solución (en la realidad varios vehículos, o
el mismo en dos momentos, pueden cargar en ella). execute(station_copies=K)
reemplaza cada estación por K nodos idénticos (misma ubicación, mismas
distancias; entre copias de una misma estación la distancia es 0). Las copias
son opcionales (costo de descarte 0) y, para no explorar permutaciones equivalentes,
llegar a la copia k cuesta k metros más en el evaluador (desempate; la distancia reportada no cambia). En los archivos de
solución cada copia se escribe con el id de la estación original, así que
simulation/ y friedman_ranking.py no cambian.

La función objetivo sigue siendo UNA sola: la distancia total (más la penalización
por cliente descartado, que solo actúa si la instancia es infactible). Parar en una
estación "por gusto" ya se penaliza sola, porque el desvío suma distancia; no se añade
ningún término al objetivo. El archivo de solución informa cuántas paradas de carga serían
innecesarias (línea 'Unnecessary charging stops'), solo como diagnóstico.
Con station_copies=1 se obtiene el modelo anterior.

Re-planeación dinámica (vehicle_states)
────────────────────────────────────────
Por defecto todo vehículo arranca en el depósito con batería llena y carga 0
(el comportamiento de siempre). Para re-optimizar un sub-problema a mitad de
ruta (ej. un vehículo ya en camino, con batería parcial y algo ya entregado),
execute() acepta vehicle_states: una lista de largo num_vehicles, con un dict
opcional por vehículo:
    {'start_node': int, 'initial_fuel': num, 'initial_load': num}
Si se omite una clave, usa el default de siempre para esa clave. Si
vehicle_states es None, el comportamiento es exactamente el de antes.

IMPORTANTE: 'start_node' debe ser un nodo que NO sea también una parada
obligatoria (cliente) en esa misma corrida — típicamente el depósito, una
estación de carga, o un nodo "virtual" que el llamador agrega a data para
representar la posición actual del vehículo. Reutilizar el índice de un
cliente pendiente como start_node no está soportado (OR-Tools trataría el
inicio de ruta y la parada de ese cliente como dos ocurrencias separadas).

Re-planeación dinámica (data_override)
────────────────────────────────────────
execute() normalmente carga los datos desde archivo vía process_files()
(basado en instance_type/distance_type). Para re-optimizar un sub-problema
con una matriz de distancia/tiempo "viva" (ej. medida en tiempo real desde
SUMO/TraCI, reflejando congestión actual — ver simulation/dynamic_replanning.py),
se puede pasar data_override: un dict con la misma forma que produce
read_file_evrp (num_locations, locations, demands, distance_matrix,
charging_stations, vehicle_capacity, fuel_capacity, fuel_consumption_rate,
etc.). Si se da, process_files() se salta por completo y ese dict se usa tal
cual — instance_type/distance_type dejan de tener efecto sobre los datos
(distance_type se sigue usando solo para nombrar la carpeta de salida).
"""

from functools import partial
import os

from ortools.constraint_solver import pywrapcp

from distance.distance_type import DistanceType
from instance.instance_type import process_files, InstanceType
from problem.strategy_type import HeuristicType, MetaheuristicType
from utils.execute_algorithm import get_distance_and_solution_name, execute_solution
from utils.feasibility import check_routes, report_lines
from utils.units import DIST_SCALE, cost_int, energy_int, cap_int


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
            to_node: (0 if from_node == to_node else cost_int(distance_matrix[from_node][to_node]))
            for to_node in range(data['num_locations'])
        }
    if not COPY_ORDER_CP:
        # desempate entre copias de una misma estacion: llegar a la copia k cuesta k metros mas (solo en el evaluador;
        # la distancia reportada sale de la matriz y no cambia)
        for group in data.get('station_copies_of', {}).values():
            for rank, copy_node in enumerate(group):
                if rank:
                    for from_node in range(data['num_locations']):
                        if from_node != copy_node:
                            _distances[from_node][copy_node] += rank
    if FORBID_BY_COST:
        # arcos prohibidos con un costo enorme (solo en el evaluador; la matriz y la distancia reportada no cambian)
        for a_, b_ in data.get('forbidden_arcs', ()):
            _distances[a_][b_] = FORBIDDEN_ARC_COST

    def distance_evaluator(manager, from_node, to_node):
        return _distances[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return distance_evaluator


# Arcos prohibidos: se guardan en data['forbidden_arcs'] como pares (origen, destino). Se aplican con un costo enorme en el
# evaluador de distancia (create_distance_evaluator) o, si FORBID_BY_COST es False, quitando el valor del dominio de
# NextVar (apply_forbidden_arcs). La matriz de distancias NO se modifica, asi que la distancia reportada es la real.


# Como se aplican los arcos prohibidos: True (por defecto) = costo enorme en el evaluador de distancia; False = se quitan
# del dominio de NextVar. Quitarlos del dominio hace que los movimientos de la busqueda local que usan esos arcos pasen
# los filtros y los rechace despues la propagacion (cara): con muchas estaciones OR-Tools tarda decenas de segundos en
# la primera solucion o no la encuentra (150c, 350c). Con costo, el filtro de costo los descarta al instante.
# EVRP_FORBID_BY_COST=0 vuelve a quitarlos del dominio (variante cadenas_por_dominio de diagnostico_ortools.py).
FORBID_BY_COST = os.environ.get("EVRP_FORBID_BY_COST", "1") != "0"
# Orden entre copias de una estacion: False (por defecto) = desempate por costo; True = restriccion ActiveVar(k+1) <= ActiveVar(k).
COPY_ORDER_CP = os.environ.get("EVRP_COPY_ORDER_CP", "0") == "1"
FORBIDDEN_ARC_COST = 1_000_000_000      # 1 000 000 km en metros; mayor que descartar 100 clientes (penalizacion 1e7 c/u)


def _add_forbidden(data, pairs):
    new = dict(data)
    new['forbidden_arcs'] = set(data.get('forbidden_arcs', ())) | set(pairs)
    return new


def apply_forbidden_arcs(routing, manager, data):
    """Quita del dominio de cada NextVar los arcos de data['forbidden_arcs'] (el deposito se aplica a cada vehiculo)."""
    if FORBID_BY_COST:
        return      # ya van en el costo del evaluador de distancia (create_distance_evaluator)
    depot = data['depot']
    depot_starts = [routing.Start(v) for v in range(routing.vehicles())
                    if manager.IndexToNode(routing.Start(v)) == depot]
    for a, b in data.get('forbidden_arcs', ()):
        ib = manager.NodeToIndex(b)
        if ib < 0:
            continue
        for ia in (depot_starts if a == depot else [manager.NodeToIndex(a)]):
            if ia >= 0:
                routing.NextVar(ia).RemoveValue(ib)


# Recarga completa forzada en cada estacion (ver add_fuel_constraints). Desactivada por defecto: en el diagnostico
# (diagnostico_ortools.py) no mejoro la busqueda y en 100c/150c la empeoro. EVRP_FULL_RECHARGE=1 la activa.
FULL_RECHARGE = os.environ.get("EVRP_FULL_RECHARGE", "0") == "1"
# Cota Cumul + Slack <= capacidad en cada estacion (la bateria al salir no supera la capacidad). Solo se desactiva
# para diagnostico (diagnostico_ortools.py): sin ella el modelo permitiria cargar por encima de la bateria fisica.
RECHARGE_BOUND = True


def drop_penalty_of(data):
    """Penalización por cliente descartado (la misma que usa add_capacity_constraints)."""
    n = data['num_locations']
    max_distance = max(data['distance_matrix'][i][j] for i in range(n) for j in range(n) if i != j)
    return max(10_000_000, cost_int(max_distance) * 100)


def expand_station_copies(data, copies, keep_single=()):
    """
    Devuelve una copia de `data` donde cada estación de carga tiene `copies` nodos (el original y copies-1
    nuevos al final, para no mover ningún índice existente). keep_single: nodos que no se copian (p. ej. las
    posiciones de arranque de los vehículos en re-planeación). Mapa copia -> original en data['station_origin'].
    """
    copies = max(1, int(copies or 1))
    new = dict(data)
    stations = [s for s in data['charging_stations'] if s not in set(keep_single)]
    origin, dm, tm = {}, [list(r) for r in data['distance_matrix']], None
    tm = [list(r) for r in data['time_matrix']] if data.get('time_matrix') is not None else None
    locations = list(data['locations']) if data.get('locations') is not None else None
    demands = list(data['demands'])
    names = dict(data.get('charging_station_names', {}))
    new_stations = list(data['charging_stations'])
    copies_of = {s: [s] for s in stations}

    def clone(src):
        """Agrega un nodo igual a `src` (misma fila y columna de distancias; 0 km hacia src y hacia sus otros clones)."""
        idx = len(dm)                           # indice del nuevo nodo
        for row in dm:                          # nueva columna: igual que la original
            row.append(row[src])
        if tm is not None:
            for row in tm:
                row.append(row[src])
        dm.append(list(dm[src]))                # nueva fila: igual que la original (ya incluye la columna nueva)
        if tm is not None:
            tm.append(list(tm[src]))
        if locations is not None:
            locations.append(locations[src])
        demands.append(0)
        names[idx] = names.get(src, src)
        new_stations.append(idx)
        return idx

    for k in range(1, copies):
        for s in stations:
            idx = clone(s)
            origin[idx] = s
            copies_of[s].append(idx)

    # Nodo de SALIDA para cada visita a estacion (cada copia): la visita pasa a ser el par llegada -> salida. Al nodo de
    # salida se le fija la bateria en la capacidad (domino de su variable de la dimension), y la recarga de la llegada queda
    # determinada por propagacion. Asi "la bateria al salir no supera la capacidad" ya no es una restriccion propia del
    # CP (que los filtros de la busqueda local no conocen y con la que OR-Tools no encontraba ninguna solucion en 350c)
    # sino parte de la dimension. Un estacion solo se puede dejar por su nodo de salida y a el solo se llega desde la
    # llegada (arcos prohibidos por costo, como el resto de las reglas). Ver add_fuel_constraints y merge_station_repeats.
    entries = list(stations) + [c for grp in copies_of.values() for c in grp[1:]]
    exit_of = {}
    for s in entries:
        e = clone(s)
        origin[e] = origin.get(s, s)
        exit_of[s] = e
    n_total = len(dm)
    banned = set()
    for s, e in exit_of.items():
        for x in range(n_total):
            if x != s and x != e:
                banned.add((s, x))              # desde la llegada solo se va a su salida
                banned.add((x, e))              # a la salida solo se llega desde su llegada
    new.update(distance_matrix=dm, time_matrix=tm, num_locations=len(dm), demands=demands,
               charging_stations=new_stations, charging_station_names=names,
               station_origin=origin, station_copies_of=copies_of, station_exit=exit_of,
               station_copies=copies)
    new['forbidden_arcs'] = set(data.get('forbidden_arcs', ())) | banned
    if locations is not None:
        new['locations'] = locations
    return new


def forbid_depot_to_station(data):
    """
    Desde el deposito no se puede ir directo a una estacion de carga (el vehiculo sale con la bateria llena, asi que
    esa parada no aporta nada). Marca como prohibido el arco deposito -> estacion (incluidas las copias).
    Devuelve una copia de `data`; no modifica el original. Las posiciones de arranque de re-planeacion no son el deposito.
    """
    return _add_forbidden(data, [(data['depot'], s_) for s_ in data['charging_stations']])


def forbid_station_chains(data, keep=()):
    """
    De una estacion de carga no se va directo a otra: tras recargar (bateria llena) el vehiculo va a un cliente o al
    deposito; encadenar estaciones solo suma distancia. Marca como prohibido el arco estacion -> estacion (incluidas
    las copias). `keep`: nodos que no son estaciones reales (posiciones de arranque en re-planeacion; un vehiculo con
    poca bateria SI puede ir directo desde alli a una estacion). Devuelve una copia de `data`.
    Supone que todo cliente es alcanzable con una carga desde alguna estacion (se verifica al construir la instancia).
    """
    keep = set(keep)
    origin = data.get('station_origin', {})
    stations = [s_ for s_ in data['charging_stations'] if s_ not in keep]
    # entre copias de la MISMA estacion no se prohibe (ver expand_station_copies); si aparece, se fusiona al guardar
    return _add_forbidden(data, [(a, b) for a in stations for b in stations
                                 if origin.get(a, a) != origin.get(b, b)])


def merge_station_repeats(nodes, stations):
    """Fusiona paradas consecutivas en la MISMA estacion (copias de una estacion, 0 km entre ellas): la bateria ya esta llena,
    asi que la segunda parada no aporta nada. Recibe ids originales y devuelve la ruta sin esas repeticiones."""
    out = []
    for n in nodes:
        if out and n == out[-1] and n in stations:
            continue
        out.append(n)
    return out


def unnecessary_stops(nodes, data):
    """
    Cantidad de paradas de carga que se podrían quitar de la ruta (lista de nodos con ids originales) sin dejar
    la batería negativa en ningún tramo. Misma regla que usa ACO. Sirve para penalizar cargas "por gusto".
    """
    em, cap, stations = data['distance_matrix'], cap_int(data['fuel_capacity']), set(data['charging_stations'])
    rate = data['fuel_consumption_rate']

    def feasible(seq):
        bat = cap
        for a, b in zip(seq, seq[1:]):
            bat -= energy_int(em[a][b], rate)
            if bat < 0:
                return False
            if b in stations:
                bat = cap
        return True
    return sum(1 for k in range(1, len(nodes) - 1)
               if nodes[k] in stations and feasible(nodes[:k] + nodes[k + 1:]))


def create_demand_evaluator(data):
    """Devuelve la demanda del nodo actual (0 para depósito y estaciones)."""
    _demands = data['demands']

    def demand_evaluator(manager, from_node):
        return _demands[manager.IndexToNode(from_node)]

    return demand_evaluator


# ═══════════════════════════════════════════════════════════════
# 2. DIMENSIÓN DE CAPACIDAD DE CARGA
# ═══════════════════════════════════════════════════════════════

def add_capacity_constraints(routing, manager, data, demand_evaluator_index, initial_loads=None):
    """
    Restricción de capacidad de carga del vehículo.
    Las estaciones de carga tienen demanda 0, por lo que no afectan esta dimensión.

    initial_loads: lista opcional (largo num_vehicles) con la carga ya
    acumulada de cada vehículo al arrancar (re-planeación dinámica). Si es
    None, todos arrancan en 0 (comportamiento de siempre).

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
        0,                                # sin slack en capacidad
        vehicle_capacity,
        initial_loads is None,            # fix_start_cumul_to_zero: solo si no hay carga inicial
        'Capacity'
    )
    capacity_dimension = routing.GetDimensionOrDie('Capacity')

    if initial_loads is not None:
        for vehicle_id in range(data['num_vehicles']):
            start_index = routing.Start(vehicle_id)
            capacity_dimension.CumulVar(start_index).SetValue(initial_loads[vehicle_id])

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
    drop_penalty = drop_penalty_of(data)

    # Clientes: visita obligatoria (descarte solo si la instancia es infactible)
    for node in range(1, num_clients + 1):
        node_index = manager.NodeToIndex(node)
        capacity_dimension.SlackVar(node_index).SetValue(0)
        routing.AddDisjunction([node_index], drop_penalty)

    # Estaciones de carga: opcionales sin penalización (costo 0)
    for cs_node in data['charging_stations']:
        node_index = manager.NodeToIndex(cs_node)
        routing.AddDisjunction([node_index], 0)

    # Copias de una misma estacion: para romper simetrias se prefiere usar la copia k antes que la k+1. Por defecto se hace
    # con un desempate de k metros en el costo de los arcos que llegan a la copia k (ver create_distance_evaluator), que
    # los filtros de la busqueda local entienden. La restriccion ActiveVar(k+1) <= ActiveVar(k) es una restriccion propia
    # que los filtros no conocen: la busqueda propone movimientos que luego la propagacion rechaza (muy caro con muchas
    # copias). EVRP_COPY_ORDER_CP=1 la reactiva.
    if COPY_ORDER_CP:
        solver = routing.solver()
        for group in data.get('station_copies_of', {}).values():
            for a, b in zip(group, group[1:]):
                solver.Add(routing.ActiveVar(manager.NodeToIndex(b)) <= routing.ActiveVar(manager.NodeToIndex(a)))


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
                _consumption[from_node][to_node] = -energy_int(dist, rate)

    def fuel_evaluator(manager, from_node, to_node):
        return _consumption[manager.IndexToNode(from_node)][manager.IndexToNode(to_node)]

    return fuel_evaluator


def add_fuel_constraints(routing, manager, data, fuel_evaluator_index, initial_fuels=None):
    """
    Añade la dimensión de batería al modelo.

    Lógica:
    - CumulVar(nodo): nivel de batería al LLEGAR al nodo.
    - SlackVar(nodo): cantidad de energía recargada en ese nodo.
    - Solo las estaciones de carga pueden tener SlackVar > 0.
    - Los vehículos parten con la batería llena por defecto
      (fix_start_cumul_to_zero=False, CumulVar del inicio fijado a fuel_capacity),
      o con la batería parcial indicada en initial_fuels (re-planeación dinámica).

    initial_fuels: lista opcional (largo num_vehicles) con la batería restante
    de cada vehículo al arrancar. Si es None, todos arrancan con fuel_capacity
    (comportamiento de siempre).

    Restricciones garantizadas por AddDimension():
    - CumulVar(nodo) <= fuel_capacity (capacidad máxima de batería)
    - SlackVar(nodo) <= fuel_capacity (recarga máxima posible en un nodo)
    - Estas restricciones son suficientes para evitar sobrecarga.
    """
    fuel_capacity = cap_int(data['fuel_capacity'])     # el modelo trabaja en metros (ver utils/units.py)

    routing.AddDimension(
        fuel_evaluator_index,
        fuel_capacity,       # slack máximo (recarga máxima posible en un nodo)
        fuel_capacity,       # capacidad máxima de batería
        False,               # NO fijar inicio a 0: los vehículos salen con batería llena (o parcial)
        'Fuel'
    )
    fuel_dimension = routing.GetDimensionOrDie('Fuel')

    charging_set = set(data['charging_stations'])
    solver = routing.solver()

    # Vehículos salen con batería llena, o con la batería parcial indicada
    for vehicle_id in range(data['num_vehicles']):
        start_index = routing.Start(vehicle_id)
        start_fuel = fuel_capacity if initial_fuels is None else cap_int(initial_fuels[vehicle_id])
        fuel_dimension.CumulVar(start_index).SetValue(start_fuel)
        # CORRECCIÓN DE RECARGA: CumulVar(nodo) <= capacidad solo acota el nivel al
        # LLEGAR; sin esto, Cumul + Slack podía superar la capacidad y el vehículo
        # salía con más energía que la batería física (el siguiente nodo "llegaba"
        # con 100 aunque el tramo consumiera). Desde el inicio no se recarga, salvo
        # que el inicio sea una estación (re-planeación), y nunca por encima de la capacidad.
        if manager.IndexToNode(start_index) in charging_set:
            solver.Add(fuel_dimension.CumulVar(start_index) + fuel_dimension.SlackVar(start_index) <= fuel_capacity)
        else:
            fuel_dimension.SlackVar(start_index).SetValue(0)

    exit_of = data.get('station_exit', {})
    exit_nodes = set(exit_of.values())
    for node in range(data['num_locations']):
        if node == data['depot']:
            continue
        index = manager.NodeToIndex(node)

        if node in exit_nodes:
            # Salida de una visita a estacion: sale con la bateria llena. Es un dominio (no una restriccion propia), asi
            # que la recarga de la llegada (Slack) queda = capacidad - bateria al llegar, y los filtros de la busqueda
            # local la entienden. Nunca se recarga por encima de la capacidad.
            fuel_dimension.CumulVar(index).SetValue(fuel_capacity)
            fuel_dimension.SlackVar(index).SetValue(0)
        elif node in exit_of:
            pass        # llegada a estacion: el Slack (0..capacidad) lo determina el nodo de salida
        elif node in charging_set:
            # Estación de carga: puede recargar
            # Preferir llegar con batería llena (reduce ansiedad de rango)
            routing.AddVariableMaximizedByFinalizer(fuel_dimension.CumulVar(index))
            # La batería al SALIR (llegada + recarga) no puede superar la capacidad.
            if RECHARGE_BOUND:
                solver.Add(fuel_dimension.CumulVar(index) + fuel_dimension.SlackVar(index) <= fuel_capacity)
            if FULL_RECHARGE:
                # Toda recarga es COMPLETA (la bateria sale al maximo), igual que en ACO y en la descripcion del modelo.
                # Con la recarga libre (0..capacidad) OR-Tools debe BUSCAR el valor de la recarga de cada estacion
                # (y de cada copia); en las instancias grandes esa busqueda no termina (millones de ramas, ninguna
                # solucion). Fijada la recarga, todos los niveles de bateria quedan determinados por propagacion.
                # Solo se exige si la estacion esta activa (las copias sin usar quedan libres).
                full = solver.IsEqualCstVar(fuel_dimension.CumulVar(index) + fuel_dimension.SlackVar(index), fuel_capacity)
                solver.Add(routing.ActiveVar(index) <= full)
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

    La distancia reportada se recalcula directamente desde data['distance_matrix']
    (misma fuente que el costo de arco), no desde routing.GetArcCostForVehicle.
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

    # Objetivo = distancia real + penalizacion por descartes (el costo por visita a estacion es solo guia de busqueda)
    dm_ = data['distance_matrix']
    pre_distance, pre_dropped = 0, 0
    origin_ = data.get('station_origin', {})
    check_input = []        # rutas (ids originales, con arranque y fin) para el validador de factibilidad
    for vehicle_id in range(data['num_vehicles']):
        idx = routing.Start(vehicle_id)
        nodes_ = [origin_.get(manager.IndexToNode(idx), manager.IndexToNode(idx))]
        while not routing.IsEnd(idx):
            nxt = assignment.Value(routing.NextVar(idx))
            pre_distance += cost_int(dm_[manager.IndexToNode(idx)][manager.IndexToNode(nxt)])
            idx = nxt
            nodes_.append(origin_.get(manager.IndexToNode(idx), manager.IndexToNode(idx)))
        check_input.append(merge_station_repeats(nodes_, set(data['charging_stations'])))
    n_clients_ = data['num_locations'] - len(data['charging_stations']) - 1
    for node in range(1, n_clients_ + 1):
        i_ = manager.NodeToIndex(node)
        if assignment.Value(routing.NextVar(i_)) == i_:
            pre_dropped += 1
    objective_value = pre_distance + pre_dropped * drop_penalty_of(data)
    copies_ = data.get('station_copies')
    feasibility = check_routes(data, check_input, n_clients_, copies=copies_)
    if feasibility:
        print(f"WARNING: infeasible solution for {instance}: {feasibility[0]}"
              f"{' (+%d more)' % (len(feasibility) - 1) if len(feasibility) > 1 else ''}")

    try:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(f'Instance: {instance}\n\n')
            # Contexto opcional (ej. descripcion de un evento de congestion que
            # disparo esta re-optimizacion) -- ver simulation/dynamic_replanning.py.
            # evrp.py no sabe que es un "evento", solo escribe lo que le pasen.
            extra_info = data.get('extra_info')
            if extra_info:
                f.write(f'{extra_info}\n\n')
            f.write(f'Objective: {objective_value / DIST_SCALE:.3f}\n\n')
            f.write(f'Execution Time: {elapsed_time}\n\n')
            if heuristic:
                f.write(f'Heuristic: {heuristic}\n\n')
            if metaheuristic:
                f.write(f'Metaheuristic: {metaheuristic}\n\n')
            f.write(f'Distance type: {distance_type_str}\n\n')
            f.write(f'Vehicle capacity: {data["vehicle_capacity"]}\n')
            f.write(f'Battery capacity: {data["fuel_capacity"]}\n')
            f.write(f'Battery consumption rate: {data["fuel_consumption_rate"]}\n\n')
            origin = data.get('station_origin', {})
            if data.get('station_copies'):
                f.write(f'Station copies: {data["station_copies"]}\n\n')

            capacity_dimension = routing.GetDimensionOrDie('Capacity')
            fuel_dimension = routing.GetDimensionOrDie('Fuel')

            # Nodos descartados (dropped)
            num_clients = data['num_locations'] - len(data['charging_stations']) - 1
            dropped_clients = []
            for node in range(1, num_clients + 1):
                index = manager.NodeToIndex(node)
                if assignment.Value(routing.NextVar(index)) == index:
                    dropped_clients.append(node)

            f.write(f'Customers served: {num_clients - len(dropped_clients)}/{num_clients}\n')
            f.write(f'Dropped customers: {len(dropped_clients)}\n')
            f.write('\n'.join(report_lines(feasibility)) + '\n\n')
            if dropped_clients:
                f.write(f'Dropped clients: {dropped_clients}\n\n')

            distance_matrix = data['distance_matrix']
            time_matrix = data.get('time_matrix')
            total_distance = 0
            total_real_km = 0.0
            total_load = 0
            total_time = 0
            total_recharges = 0
            min_soc = None

            n_unnecessary = 0
            for vehicle_id in range(data['num_vehicles']):
                index = routing.Start(vehicle_id)
                plan_output = f'Route for vehicle {vehicle_id}:\n'
                route_distance = 0
                route_real_km = 0.0     # distancia real (sin truncar por arco)
                route_time = 0
                route_nodes = []

                while not routing.IsEnd(index):
                    node = origin.get(manager.IndexToNode(index), manager.IndexToNode(index))
                    if route_nodes and node == route_nodes[-1] and node in charging_set:
                        # segunda parada seguida en la misma estacion (otra copia): se fusiona con la anterior
                        previous_node = node
                        index = assignment.Value(routing.NextVar(index))
                        next_node = origin.get(manager.IndexToNode(index), manager.IndexToNode(index))
                        route_distance += cost_int(distance_matrix[previous_node][next_node])
                        route_real_km += float(distance_matrix[previous_node][next_node])
                        if time_matrix is not None:
                            route_time += int(time_matrix[previous_node][next_node])
                        continue
                    route_nodes.append(node)
                    load_var = capacity_dimension.CumulVar(index)
                    fuel_var = fuel_dimension.CumulVar(index)
                    soc = assignment.Min(fuel_var) / DIST_SCALE     # unidades de bateria (el modelo usa metros)
                    min_soc = soc if min_soc is None else min(min_soc, soc)

                    # Etiqueta especial para estaciones de carga
                    if node in charging_set:
                        total_recharges += 1
                        cs_label = f'[CS:{cs_names.get(node, node)}]'
                        plan_output += (
                            f' {node}{cs_label} '
                            f'Load({assignment.Min(load_var)}) '
                            f'Bat({soc:.1f}) ->'
                        )
                    else:
                        plan_output += (
                            f' {node} '
                            f'Load({assignment.Min(load_var)}) '
                            f'Bat({soc:.1f}) ->'
                        )

                    previous_node = node
                    index = assignment.Value(routing.NextVar(index))
                    next_node = origin.get(manager.IndexToNode(index), manager.IndexToNode(index))
                    route_distance += cost_int(distance_matrix[previous_node][next_node])
                    route_real_km += float(distance_matrix[previous_node][next_node])
                    if time_matrix is not None:
                        route_time += int(time_matrix[previous_node][next_node])

                # Nodo final (depósito de llegada)
                load_var = capacity_dimension.CumulVar(index)
                fuel_var = fuel_dimension.CumulVar(index)
                soc = assignment.Min(fuel_var) / DIST_SCALE
                min_soc = soc if min_soc is None else min(min_soc, soc)
                end_node = origin.get(manager.IndexToNode(index), manager.IndexToNode(index))
                route_nodes.append(end_node)
                n_unnecessary += unnecessary_stops(route_nodes, data)
                plan_output += (
                    f' {end_node} '
                    f'Load({assignment.Min(load_var)}) '
                    f'Bat({soc:.1f})\n'
                )
                plan_output += f'Distance of the route: {route_distance / DIST_SCALE:.3f}km\n'
                plan_output += f'Real distance of the route: {route_real_km:.2f}km\n'
                if time_matrix is not None:
                    plan_output += f'Time of the route: {route_time}s\n'
                plan_output += f'Load of the route: {assignment.Min(load_var)}\n\n'

                f.write(plan_output)
                total_distance += route_distance
                total_real_km += route_real_km
                total_load += assignment.Min(load_var)
                total_time += route_time

            f.write(f'Total Distance of all routes: {total_distance / DIST_SCALE:.3f}km\n\n')
            f.write(f'Real distance (km): {total_real_km:.2f}\n\n')
            f.write(f'Total Load of all routes: {total_load}\n\n')
            if time_matrix is not None:
                f.write(f'Total Time of all routes: {total_time}s\n\n')
            f.write(f'Total recharges: {total_recharges}\n\n')
            f.write(f'Unnecessary charging stops: {n_unnecessary}\n\n')
            if min_soc is not None:
                f.write(f'Minimum battery (SoC) reached: {min_soc:.1f}\n\n')

            # Distancia ya recorrida ANTES de este sub-problema (ej. lo que cada
            # vehiculo ya manejo antes de un evento de congestion) -- sin esto,
            # el Objective de arriba solo refleja el costo del tramo restante,
            # no el costo real total del viaje completo.
            prior_distance_km = data.get('prior_distance_km')
            if prior_distance_km:
                f.write(f'Distance already traveled before this sub-problem: {prior_distance_km:.3f}km\n\n')
                f.write(f'Total Distance including distance before event: '
                        f'{total_real_km + prior_distance_km:.3f}km\n\n')

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
        vehicle_states=None,    # re-planeación dinámica: ver docstring del módulo
        data_override=None,     # re-planeación dinámica: dict de datos ya construido (salta process_files)
        instance_name=None,     # nombre a usar para la clave/archivo de salida cuando se usa data_override
        station_copies=3        # veces que puede usarse cada estacion en una solucion (1 = modelo anterior: una sola vez)
):
    if data_override is not None:
        instances_data = {instance_name or "dynamic_replan": data_override}
    else:
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

        if vehicle_states is not None and len(vehicle_states) != data['num_vehicles']:
            raise ValueError(
                f"vehicle_states debe tener un elemento por vehículo "
                f"({data['num_vehicles']}), recibí {len(vehicle_states)}."
            )

        # ── Estaciones con varias visitas posibles y costo por visita ────────
        start_nodes = [vs.get('start_node') for vs in vehicle_states] if vehicle_states is not None else []
        if vehicle_states is not None:
            data = dict(data,
                        start_battery={vs.get('start_node', data['depot']): cap_int(vs.get('initial_fuel', data['fuel_capacity']))
                                       for vs in vehicle_states},
                        start_load={vs.get('start_node', data['depot']): vs.get('initial_load', 0) for vs in vehicle_states})
        data = expand_station_copies(data, station_copies, keep_single=start_nodes)
        data = forbid_depot_to_station(data)
        data = forbid_station_chains(data, keep=start_nodes)

        # ── Índice Manager ────────────────────────────────────────────────────
        # Sin vehicle_states: todos arrancan y terminan en el depósito (de siempre).
        # Con vehicle_states: cada vehículo puede arrancar en un nodo distinto
        # (su posición actual), pero siempre termina en el depósito real.
        if vehicle_states is not None:
            starts = [vs.get('start_node', data['depot']) for vs in vehicle_states]
            ends = [data['depot']] * data['num_vehicles']
            manager = pywrapcp.RoutingIndexManager(
                data['num_locations'], data['num_vehicles'], starts, ends
            )
        else:
            manager = pywrapcp.RoutingIndexManager(
                data['num_locations'],
                data['num_vehicles'],
                data['depot']
            )

        # ── Modelo de ruteo ───────────────────────────────────────────────────
        routing = pywrapcp.RoutingModel(manager)

        # ── Costo de arco: distancia ──────────────────────────────────────────
        distance_evaluator_index = routing.RegisterTransitCallback(
            partial(create_distance_evaluator(data), manager)
        )
        routing.SetArcCostEvaluatorOfAllVehicles(distance_evaluator_index)

        initial_loads = None
        initial_fuels = None
        if vehicle_states is not None:
            initial_loads = [vs.get('initial_load', 0) for vs in vehicle_states]
            initial_fuels = [vs.get('initial_fuel', data['fuel_capacity']) for vs in vehicle_states]

        # ── Dimensión: capacidad de carga ─────────────────────────────────────
        demand_evaluator_index = routing.RegisterUnaryTransitCallback(
            partial(create_demand_evaluator(data), manager)
        )
        add_capacity_constraints(routing, manager, data, demand_evaluator_index, initial_loads)

        # ── Dimensión: batería (EVRP) ─────────────────────────────────────────
        fuel_evaluator_index = routing.RegisterTransitCallback(
            partial(create_fuel_evaluator(data), manager)
        )
        add_fuel_constraints(routing, manager, data, fuel_evaluator_index, initial_fuels)
        apply_forbidden_arcs(routing, manager, data)

        # ── Resolver ──────────────────────────────────────────────────────────
        execute_solution(
            save_solution, heuristic, metaheuristic, i, distance_type,
            routing, time_limit, data, manager, instance, initial_routes
        )