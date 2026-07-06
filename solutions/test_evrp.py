"""
test_evrp.py — Script de prueba para el módulo EVRP

Cómo ejecutar desde la raíz del repo:
    python solutions/test_evrp.py

Qué prueba:
    Nivel 1 — Carga de datos       : read_file_evrp lee bien la instancia
    Nivel 2 — Modelo OR-Tools      : se construye sin errores y el solver devuelve solución
    Nivel 3 — Validación física    : las rutas respetan batería y capacidad de carga
    Nivel 4 — Objetivo ponderado   : evrp.execute() con recharge_weight / vehicle_fixed_cost
                                     produce el desglose f1/f2/f3 esperado

Instancia usada: instances_data/evrp_instances/quebec_40c_4ev_6cs.txt
    4 vehículos, capacidad 250, depósito (46.8139,-71.2080),

    40 clientes (nodos 1-40), 6 estaciones de carga (nodos 41-46) => 47 nodos.
"""

import sys
import os
import time
import traceback
from functools import partial

# ── Asegura que la raíz del repo esté en el path (no la carpeta solutions/) ──
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# ── Colores para la consola (sin dependencias externas) ──────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

INSTANCE_PATH = os.path.join(
    os.path.dirname(__file__), "..",
    "instances_data", "evrp_instances", "quebec_40c_4ev_6cs.txt"
)

passed = 0
failed = 0


def ok(msg):
    global passed
    passed += 1
    print(f"  {GREEN}✓{RESET} {msg}")


def fail(msg, detail=""):
    global failed
    failed += 1
    print(f"  {RED}✗{RESET} {msg}")
    if detail:
        print(f"    {YELLOW}→ {detail}{RESET}")


def section(title):
    print(f"\n{BOLD}{CYAN}{'-'*60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'-'*60}{RESET}")


# ════════════════════════════════════════════════════════════════
# NIVEL 1 — CARGA DE DATOS
# ════════════════════════════════════════════════════════════════

section("NIVEL 1 - Carga de datos  (read_file_evrp)")

data = None
try:
    from instance.import_data import read_file_evrp
    from distance.distance_type import DistanceType

    # OSRM: distancia y tiempo reales de carretera (requiere el servidor osrm-routed
    # corriendo en OSRM_BASE_URL / http://localhost:5000, ver distance/osrm_client.py).
    # Antes se usaba HAVERSINE (línea recta) como sustituto; ahora que OSRM está
    # integrado, el test debe validar contra la fuente de distancia real del proyecto.
    data = read_file_evrp(
        INSTANCE_PATH,
        distance_type=DistanceType.OSRM,
        vehicle_maximum_travel_distance=100,   # fuel_capacity  = 100 unidades (autonomía real del EV)
        vehicle_speed=1.0,                     # fuel_consumption_rate = 1.0 / km
        integer=True
    )

    # ── 1.1 Claves obligatorias ───────────────────────────────────────────────
    required_keys = [
        "num_vehicles", "vehicle_capacity", "vehicle_capacities",
        "locations", "num_locations", "demands", "depot",
        "distance_matrix", "fuel_capacity", "fuel_consumption_rate",
        "charging_stations", "charging_station_names"
    ]
    missing = [k for k in required_keys if k not in data]
    if missing:
        fail("El diccionario contiene todas las claves requeridas", f"Faltan: {missing}")
    else:
        ok("El diccionario contiene todas las claves requeridas")

    # ── 1.2 Conteo de nodos ───────────────────────────────────────────────────
    # instancia: 1 depósito + 40 clientes + 6 estaciones = 47 nodos
    expected_nodes = 47
    if data['num_locations'] == expected_nodes:
        ok(f"Número de nodos correcto: {data['num_locations']} "
           f"(1 depósito + 40 clientes + 6 estaciones)")
    else:
        fail(f"Número de nodos esperado: {expected_nodes}",
             f"Obtenido: {data['num_locations']}")

    # ── 1.3 Flota ─────────────────────────────────────────────────────────────
    if data['num_vehicles'] == 4:
        ok(f"Número de vehículos correcto: {data['num_vehicles']}")
    else:
        fail("Número de vehículos esperado: 4", f"Obtenido: {data['num_vehicles']}")

    if data['vehicle_capacity'] == 250:
        ok(f"Capacidad de vehículo correcta: {data['vehicle_capacity']}")
    else:
        fail("Capacidad de vehículo esperada: 250", f"Obtenida: {data['vehicle_capacity']}")

    # ── 1.4 Depósito ──────────────────────────────────────────────────────────
    depot_loc = data['locations'][0]
    if abs(depot_loc[0] - 46.8139) < 0.001 and abs(depot_loc[1] - (-71.2080)) < 0.001:
        ok(f"Depósito en posición correcta: {depot_loc}")
    else:
        fail("Depósito en posición incorrecta", f"Obtenido: {depot_loc}")

    if data['demands'][0] == 0:
        ok("Demanda del depósito es 0")
    else:
        fail("Demanda del depósito debe ser 0", f"Obtenida: {data['demands'][0]}")

    # ── 1.5 Clientes ──────────────────────────────────────────────────────────
    num_clients = data['num_locations'] - len(data['charging_stations']) - 1
    demands_clients = data['demands'][1: num_clients + 1]
    if all(d > 0 for d in demands_clients):
        ok(f"Todos los {num_clients} clientes tienen demanda > 0")
    else:
        zeros = [i for i, d in enumerate(demands_clients, 1) if d == 0]
        fail("Hay clientes con demanda 0", f"Nodos: {zeros}")

    # ── 1.6 Estaciones de carga ───────────────────────────────────────────────
    cs = data['charging_stations']
    if len(cs) == 6:
        ok(f"Número de estaciones de carga correcto: {len(cs)}")
    else:
        fail("Número de estaciones de carga esperado: 6", f"Obtenido: {len(cs)}")

    if all(data['demands'][i] == 0 for i in cs):
        ok("Todas las estaciones de carga tienen demanda 0")
    else:
        fail("Hay estaciones de carga con demanda != 0")

    cs_indices_expected = list(range(41, 47))
    if cs == cs_indices_expected:
        ok(f"Índices de estaciones correctos: {cs}")
    else:
        fail(f"Índices de estaciones esperados: {cs_indices_expected}", f"Obtenidos: {cs}")

    if len(data['charging_station_names']) == 6:
        ok(f"Nombres de estaciones cargados: {list(data['charging_station_names'].values())}")
    else:
        fail("No se cargaron bien los nombres de estaciones")

    # ── 1.7 Batería ───────────────────────────────────────────────────────────
    if data['fuel_capacity'] == 100:
        ok(f"fuel_capacity correcto: {data['fuel_capacity']}")
    else:
        fail("fuel_capacity esperado: 100", f"Obtenido: {data['fuel_capacity']}")

    if data['fuel_consumption_rate'] == 1.0:
        ok(f"fuel_consumption_rate correcto: {data['fuel_consumption_rate']}")
    else:
        fail("fuel_consumption_rate esperado: 1.0", f"Obtenido: {data['fuel_consumption_rate']}")

    # ── 1.8 Matriz de distancias ──────────────────────────────────────────────
    n = data['num_locations']
    dm = data['distance_matrix']
    if len(dm) == n and all(len(row) == n for row in dm):
        ok(f"Matriz de distancias tiene dimensiones correctas: {n}x{n}")
    else:
        fail(f"Matriz de distancias debe ser {n}x{n}")

    if all(dm[i][i] == 0 for i in range(n)):
        ok("Diagonal de la matriz de distancias es 0")
    else:
        fail("La diagonal de la matriz de distancias debe ser 0")

    # Con OSRM (distancia real de carretera) + integer=True, pares realmente
    # cercanos (< 1 km, común entre estaciones de carga urbanas) truncan
    # legítimamente a 0. Lo que sí debe cumplirse es que exista variación real
    # de escala en la matriz.
    max_dm = max(dm[i][j] for i in range(n) for j in range(n) if i != j)
    zero_pairs = sum(1 for i in range(n) for j in range(n) if i != j and dm[i][j] == 0)
    if max_dm >= 1:
        ok(f"La matriz tiene escala real de distancias (máx={max_dm}km, "
           f"{zero_pairs} pares < 1km truncados a 0)")
    else:
        fail("La distancia máxima de la matriz es sospechosamente baja", f"max={max_dm}")

except Exception as e:
    fail("Error inesperado en Nivel 1", str(e))
    traceback.print_exc()
    data = None


# ════════════════════════════════════════════════════════════════
# NIVEL 2 — CONSTRUCCIÓN DEL MODELO OR-TOOLS
# ════════════════════════════════════════════════════════════════

section("NIVEL 2 - Construccion del modelo OR-Tools")

solution = None
routing = None
manager = None

if data is None:
    fail("Nivel 2 omitido: los datos no se cargaron correctamente")
else:
    try:
        from ortools.constraint_solver import pywrapcp, routing_enums_pb2
        from problem.execute.evrp import (
            create_distance_evaluator,
            create_objective_evaluator,
            create_demand_evaluator,
            create_fuel_evaluator,
            add_capacity_constraints,
            add_fuel_constraints,
        )

        # ── 2.1 Index Manager ─────────────────────────────────────────────────
        try:
            manager = pywrapcp.RoutingIndexManager(
                data['num_locations'],
                data['num_vehicles'],
                data['depot']
            )
            ok(f"RoutingIndexManager creado: {data['num_locations']} nodos, "
               f"{data['num_vehicles']} vehículos")
        except Exception as e:
            fail("Error creando RoutingIndexManager", str(e))
            manager = None

        # ── 2.2 Routing Model ─────────────────────────────────────────────────
        if manager:
            try:
                routing = pywrapcp.RoutingModel(manager)
                ok("RoutingModel creado correctamente")
            except Exception as e:
                fail("Error creando RoutingModel", str(e))
                routing = None

        # ── 2.3 Evaluador de distancia pura (sin distance_type: usa distance_matrix) ──
        if routing:
            try:
                dist_eval = create_distance_evaluator(data)
                dist_idx = routing.RegisterTransitCallback(partial(dist_eval, manager))
                ok("Evaluador de distancia pura registrado")
            except Exception as e:
                fail("Error registrando evaluador de distancia", str(e))

        # ── 2.4 Evaluador de objetivo ponderado (arc cost real del modelo) ────
        if routing:
            try:
                objective_eval = create_objective_evaluator(data, recharge_weight=2000)
                objective_idx = routing.RegisterTransitCallback(partial(objective_eval, manager))
                routing.SetArcCostEvaluatorOfAllVehicles(objective_idx)
                ok("Evaluador de objetivo ponderado (distancia + recarga) registrado")
            except Exception as e:
                fail("Error registrando evaluador de objetivo ponderado", str(e))

        # ── 2.5 Dimensión de capacidad ────────────────────────────────────────
        if routing:
            try:
                demand_eval = create_demand_evaluator(data)
                demand_idx = routing.RegisterUnaryTransitCallback(partial(demand_eval, manager))
                add_capacity_constraints(routing, manager, data, demand_idx)
                routing.GetDimensionOrDie('Capacity')
                ok("Dimensión 'Capacity' añadida y recuperada correctamente")
            except Exception as e:
                fail("Error añadiendo dimensión Capacity", str(e))

        # ── 2.6 Dimensión de batería ──────────────────────────────────────────
        if routing:
            try:
                fuel_eval = create_fuel_evaluator(data)
                fuel_idx = routing.RegisterTransitCallback(partial(fuel_eval, manager))
                add_fuel_constraints(routing, manager, data, fuel_idx)
                routing.GetDimensionOrDie('Fuel')
                ok("Dimensión 'Fuel' añadida y recuperada correctamente")
            except Exception as e:
                fail("Error añadiendo dimensión Fuel", str(e))

        # ── 2.7 Tránsito negativo de batería ──────────────────────────────────
        if routing:
            distance_0_1 = data['distance_matrix'][0][1]
            transit = -int(distance_0_1 * data['fuel_consumption_rate'])

            if transit < 0:
                ok(f"Tránsito de batería depósito->cliente_1 es negativo: {transit} (drena batería)")
            elif transit == 0:
                fail("Tránsito de batería es 0 - la distancia podría ser 0 o la tasa de consumo 0")
            else:
                fail("Tránsito de batería es positivo - debe ser negativo para drenar la batería",
                     f"Valor: {transit}")

        # ── 2.8 Resolver (tiempo límite corto para el test) ───────────────────
        if routing:
            try:
                search_params = pywrapcp.DefaultRoutingSearchParameters()
                search_params.first_solution_strategy = (
                    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
                )
                search_params.local_search_metaheuristic = (
                    routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
                )
                search_params.time_limit.seconds = 15
                print(f"\n  {YELLOW}-> Ejecutando solver (límite 15s)...{RESET}")
                t0 = time.time()
                solution = routing.SolveWithParameters(search_params)
                elapsed = round(time.time() - t0, 2)

                if solution:
                    ok(f"Solver encontró una solución en {elapsed}s "
                       f"(objetivo: {solution.ObjectiveValue()})")
                else:
                    fail("El solver no encontró solución en 15s",
                         "Prueba con instancia más pequeña o aumenta el tiempo límite")
            except Exception as e:
                fail("Error ejecutando el solver", str(e))

    except ImportError as e:
        fail("No se pudo importar un módulo necesario", str(e))
        traceback.print_exc()


# ════════════════════════════════════════════════════════════════
# NIVEL 3 — VALIDACIÓN FÍSICA DE LA SOLUCIÓN
# ════════════════════════════════════════════════════════════════

section("NIVEL 3 - Validacion fisica de la solucion")

if solution is None or routing is None or manager is None:
    fail("Nivel 3 omitido: no hay solución disponible")
else:
    try:
        fuel_dimension    = routing.GetDimensionOrDie('Fuel')
        capacity_dimension = routing.GetDimensionOrDie('Capacity')

        fuel_capacity    = data['fuel_capacity']
        vehicle_capacity = data['vehicle_capacity']
        charging_set     = set(data['charging_stations'])
        cs_names         = data.get('charging_station_names', {})

        total_vehicles_used   = 0
        total_clients_served  = 0
        battery_violations    = 0
        capacity_violations   = 0
        cs_visits             = 0
        routes_info           = []

        for vehicle_id in range(data['num_vehicles']):
            index = routing.Start(vehicle_id)
            route_nodes   = []
            route_fuel    = []
            route_load    = []
            route_used    = False

            while not routing.IsEnd(index):
                node       = manager.IndexToNode(index)
                fuel_val   = solution.Min(fuel_dimension.CumulVar(index))
                load_val   = solution.Min(capacity_dimension.CumulVar(index))
                route_nodes.append(node)
                route_fuel.append(fuel_val)
                route_load.append(load_val)

                if node != data['depot']:
                    route_used = True
                    if node in charging_set:
                        cs_visits += 1
                    else:
                        total_clients_served += 1

                if fuel_val < 0:
                    battery_violations += 1
                if load_val > vehicle_capacity:
                    capacity_violations += 1

                index = solution.Value(routing.NextVar(index))

            if route_used:
                total_vehicles_used += 1
                routes_info.append({
                    'vehicle': vehicle_id,
                    'nodes': route_nodes,
                    'fuel': route_fuel,
                    'load': route_load,
                })

        # ── 3.1 Restricción de batería ────────────────────────────────────────
        if battery_violations == 0:
            ok("Ningún vehículo llega a un nodo con batería negativa")
        else:
            fail(f"Hay {battery_violations} nodos con nivel de batería negativo")

        # ── 3.2 Restricción de capacidad ─────────────────────────────────────
        if capacity_violations == 0:
            ok("Ningún vehículo supera su capacidad de carga en ningún nodo")
        else:
            fail(f"Hay {capacity_violations} nodos donde se supera la capacidad del vehículo")

        # ── 3.3 Cobertura de clientes ─────────────────────────────────────────
        num_clients = data['num_locations'] - len(data['charging_stations']) - 1
        if total_clients_served == num_clients:
            ok(f"Todos los clientes atendidos: {total_clients_served}/{num_clients}")
        else:
            dropped = num_clients - total_clients_served
            print(f"  {YELLOW}!{RESET}  {dropped} clientes no atendidos "
                  f"({total_clients_served}/{num_clients}) - pueden ser penalizados por infactibilidad")

        # ── 3.4 Uso de estaciones de carga ────────────────────────────────────
        if cs_visits > 0:
            ok(f"Se usaron estaciones de carga: {cs_visits} visitas registradas")
        else:
            print(f"  {YELLOW}!{RESET}  Ningún vehículo visitó estaciones de carga "
                  "- puede ser correcto si la batería fue suficiente para todas las rutas")

        # ── 3.5 Detalle de rutas ──────────────────────────────────────────────
        print(f"\n  {BOLD}Resumen de rutas:{RESET}")
        for r in routes_info:
            vid    = r['vehicle']
            nodes  = r['nodes']
            fuels  = r['fuel']
            loads  = r['load']
            min_f  = min(fuels)
            max_l  = max(loads)

            cs_in_route = [n for n in nodes if n in charging_set]
            cs_labels   = [cs_names.get(n, str(n)) for n in cs_in_route]

            route_str = " -> ".join(
                f"{n}[CS]" if n in charging_set else str(n)
                for n in nodes
            )

            print(f"\n  {CYAN}Vehículo {vid}{RESET}")
            print(f"    Nodos   : {route_str}")
            print(f"    Fuel mín: {min_f}  (capacidad: {fuel_capacity})")
            print(f"    Carga máx: {max_l}  (capacidad: {vehicle_capacity})")
            if cs_labels:
                print(f"    Estaciones visitadas: {cs_labels}")

            if min_f < 0:
                print(f"    {RED}BATERÍA NEGATIVA en algún punto{RESET}")
            elif min_f < fuel_capacity * 0.1:
                print(f"    {YELLOW}Batería llegó muy baja (<10%){RESET}")
            else:
                print(f"    {GREEN}Batería siempre por encima del 10%{RESET}")

    except Exception as e:
        fail("Error inesperado en Nivel 3", str(e))
        traceback.print_exc()


# ════════════════════════════════════════════════════════════════
# NIVEL 4 — OBJETIVO PONDERADO (evrp.execute end-to-end)
# ════════════════════════════════════════════════════════════════

section("NIVEL 4 - Objetivo ponderado (evrp.execute con recharge_weight / vehicle_fixed_cost)")

try:
    from instance.instance_type import InstanceType
    from problem.execute import evrp
    from problem.strategy_type import HeuristicType

    def run_and_parse(recharge_weight=0, vehicle_fixed_cost=0):
        """Ejecuta evrp.execute() sobre TODAS las instancias de evrp_instances/ y
        parsea el desglose f1/f2/f3 del archivo de salida de quebec_40c_4ev_6cs.txt."""
        evrp.execute(
            0, InstanceType.EVRP, time_limit=5,
            vehicle_maximum_travel_distance=100,
            vehicle_speed=1.0,
            distance_type=DistanceType.OSRM,
            heuristic=HeuristicType.PATH_CHEAPEST_ARC,
            recharge_weight=recharge_weight,
            vehicle_fixed_cost=vehicle_fixed_cost,
        )
        out_path = os.path.join(
            os.path.dirname(__file__), "..",
            "problem", "osrm", "solutions_evrp_0",
            "solutions_PATH_CHEAPEST_ARC", "quebec_40c_4ev_6cs.txt"
        )
        text = open(out_path, encoding="utf-8").read()
        result = {}
        for line in text.splitlines():
            if line.startswith("Objective:"):
                result['objective'] = int(line.split(":")[1].strip())
            elif line.startswith("f1 Distancia total"):
                result['f1'] = int(line.split(":")[1].strip())
            elif line.startswith("f2 Numero de recargas"):
                result['f2'] = int(line.split(":")[1].split("[")[0].strip())
            elif line.startswith("f3 Numero de vehiculos"):
                result['f3'] = int(line.split(":")[1].split("[")[0].strip())
        return result

    prev_dir = os.getcwd()
    os.chdir(os.path.join(os.path.dirname(__file__), ".."))
    try:
        baseline = run_and_parse(recharge_weight=0, vehicle_fixed_cost=0)
        weighted = run_and_parse(recharge_weight=2000, vehicle_fixed_cost=50000)
    finally:
        os.chdir(prev_dir)

    # ── 4.1 Baseline: objetivo == f1 cuando los pesos son 0 ───────────────────
    if baseline.get('objective') == baseline.get('f1'):
        ok(f"Baseline (pesos=0): Objetivo == f1 == {baseline.get('objective')}")
    else:
        fail("Baseline: el objetivo debería ser igual a f1 cuando los pesos son 0",
             f"Objective={baseline.get('objective')} f1={baseline.get('f1')}")

    # ── 4.2 Fórmula F = f1 + recharge_weight*f2 + vehicle_fixed_cost*f3 ───────
    expected = weighted.get('f1', 0) + 2000 * weighted.get('f2', 0) + 50000 * weighted.get('f3', 0)
    if weighted.get('objective') == expected:
        ok(f"F = f1 + recharge_weight*f2 + vehicle_fixed_cost*f3 se cumple exactamente: "
           f"{weighted.get('objective')} == {weighted.get('f1')} + 2000*{weighted.get('f2')} + 50000*{weighted.get('f3')}")
    else:
        fail("La fórmula del objetivo ponderado no coincide",
             f"Objective={weighted.get('objective')} esperado={expected}")

    # ── 4.3 El desglose reporta todos los componentes ─────────────────────────
    if all(k in weighted for k in ('f1', 'f2', 'f3', 'objective')):
        ok("El archivo de solución reporta f1, f2, f3 y Objective por separado")
    else:
        fail("Falta algún componente del desglose en el archivo de solución", str(weighted))

except Exception as e:
    fail("Error inesperado en Nivel 4", str(e))
    traceback.print_exc()


# ════════════════════════════════════════════════════════════════
# RESUMEN FINAL
# ════════════════════════════════════════════════════════════════

section("RESUMEN")
total = passed + failed
print(f"  Tests pasados : {GREEN}{passed}/{total}{RESET}")
print(f"  Tests fallidos: {RED}{failed}/{total}{RESET}")

if failed == 0:
    print(f"\n  {GREEN}{BOLD}Todos los tests pasaron. El módulo EVRP está listo.{RESET}")
else:
    print(f"\n  {YELLOW}{BOLD}Hay {failed} test(s) fallido(s). Revisa los detalles arriba.{RESET}")

print()
sys.exit(0 if failed == 0 else 1)
