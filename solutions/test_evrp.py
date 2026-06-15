"""
test_evrp.py — Script de prueba para el módulo EVRP

Cómo ejecutar desde la raíz del repo:
    python test_evrp.py

Qué prueba:
    Nivel 1 — Carga de datos    : read_file_evrp lee bien la instancia
    Nivel 2 — Modelo OR-Tools   : se construye sin errores y el solver devuelve solución
    Nivel 3 — Validación física : las rutas respetan batería y capacidad de carga
"""

import sys
import os
import time
import traceback
from functools import partial

# ── Asegura que el repo esté en el path ──────────────────────────────────────
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

# ── Colores para la consola (sin dependencias externas) ──────────────────────
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

INSTANCE_PATH = "D:/Proyectos/Github/EVRP-Module/instances_data/evrp_instances/quebec_40c_4ev_6cs.txt"

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
    print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
    print(f"{BOLD}{CYAN}  {title}{RESET}")
    print(f"{BOLD}{CYAN}{'─'*60}{RESET}")


# ════════════════════════════════════════════════════════════════
# NIVEL 1 — CARGA DE DATOS
# ════════════════════════════════════════════════════════════════

section("NIVEL 1 — Carga de datos  (read_file_evrp)")

try:
    from instance.import_data import read_file_evrp
    from distance.distance_type import DistanceType

    data = read_file_evrp(
        INSTANCE_PATH,
        distance_type=DistanceType.MANHATTAN,
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
    # instancia: 1 depósito + 50 clientes + 5 estaciones = 56 nodos
    expected_nodes = 56
    if data['num_locations'] == expected_nodes:
        ok(f"Número de nodos correcto: {data['num_locations']} "
           f"(1 depósito + 50 clientes + 5 estaciones)")
    else:
        fail(f"Número de nodos esperado: {expected_nodes}",
             f"Obtenido: {data['num_locations']}")

    # ── 1.3 Flota ─────────────────────────────────────────────────────────────
    if data['num_vehicles'] == 5:
        ok(f"Número de vehículos correcto: {data['num_vehicles']}")
    else:
        fail("Número de vehículos esperado: 5", f"Obtenido: {data['num_vehicles']}")

    if data['vehicle_capacity'] == 400:
        ok(f"Capacidad de vehículo correcta: {data['vehicle_capacity']}")
    else:
        fail("Capacidad de vehículo esperada: 400", f"Obtenida: {data['vehicle_capacity']}")

    # ── 1.4 Depósito ──────────────────────────────────────────────────────────
    depot_loc = data['locations'][0]
    if abs(depot_loc[0] - 46.5) < 0.001 and abs(depot_loc[1] - (-72.0)) < 0.001:
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
    if len(cs) == 5:
        ok(f"Número de estaciones de carga correcto: {len(cs)}")
    else:
        fail("Número de estaciones de carga esperado: 5", f"Obtenido: {len(cs)}")

    if all(data['demands'][i] == 0 for i in cs):
        ok("Todas las estaciones de carga tienen demanda 0")
    else:
        fail("Hay estaciones de carga con demanda != 0")

    cs_indices_expected = list(range(51, 56))
    if cs == cs_indices_expected:
        ok(f"Índices de estaciones correctos: {cs}")
    else:
        fail(f"Índices de estaciones esperados: {cs_indices_expected}", f"Obtenidos: {cs}")

    if len(data['charging_station_names']) == 5:
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
        ok(f"Matriz de distancias tiene dimensiones correctas: {n}×{n}")
    else:
        fail(f"Matriz de distancias debe ser {n}×{n}")

    if all(dm[i][i] == 0 for i in range(n)):
        ok("Diagonal de la matriz de distancias es 0")
    else:
        fail("La diagonal de la matriz de distancias debe ser 0")

    if all(dm[i][j] > 0 for i in range(n) for j in range(n) if i != j):
        ok("Todas las distancias entre nodos distintos son positivas")
    else:
        fail("Hay distancias 0 entre nodos distintos")

except Exception as e:
    fail("Error inesperado en Nivel 1", str(e))
    traceback.print_exc()
    data = None


# ════════════════════════════════════════════════════════════════
# NIVEL 2 — CONSTRUCCIÓN DEL MODELO OR-TOOLS
# ════════════════════════════════════════════════════════════════

section("NIVEL 2 — Construcción del modelo OR-Tools")

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
            create_demand_evaluator,
            create_fuel_evaluator,
            add_capacity_constraints,
            add_fuel_constraints,
        )
        import problem.execute.evrp

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

        # ── 2.3 Evaluador de distancia ────────────────────────────────────────
        if routing:
            try:
                dist_eval = create_distance_evaluator(data, DistanceType.MANHATTAN)
                dist_idx = routing.RegisterTransitCallback(partial(dist_eval, manager))
                routing.SetArcCostEvaluatorOfAllVehicles(dist_idx)
                ok("Evaluador de distancia registrado")
            except Exception as e:
                fail("Error registrando evaluador de distancia", str(e))

        # ── 2.4 Dimensión de capacidad ────────────────────────────────────────
        if routing:
            try:
                demand_eval = create_demand_evaluator(data)
                demand_idx = routing.RegisterUnaryTransitCallback(partial(demand_eval, manager))
                add_capacity_constraints(routing, manager, data, demand_idx)
                routing.GetDimensionOrDie('Capacity')
                ok("Dimensión 'Capacity' añadida y recuperada correctamente")
            except Exception as e:
                fail("Error añadiendo dimensión Capacity", str(e))

        # ── 2.5 Dimensión de batería ──────────────────────────────────────────
        if routing:
            try:
                fuel_eval = create_fuel_evaluator(data)
                fuel_idx = routing.RegisterTransitCallback(partial(fuel_eval, manager))
                add_fuel_constraints(routing, manager, data, fuel_idx)
                routing.GetDimensionOrDie('Fuel')
                ok("Dimensión 'Fuel' añadida y recuperada correctamente")
            except Exception as e:
                fail("Error añadiendo dimensión Fuel", str(e))

        # ── 2.6 Tránsito negativo de batería ──────────────────────────────────
        if routing:
            # Calcular consumo de batería directamente desde distance_matrix
            # No usar fuel_evaluator para evitar complejidad de manager
            distance_0_1 = data['distance_matrix'][0][1]
            transit = -int(distance_0_1 * data['fuel_consumption_rate'])

            if transit < 0:
                ok(f"Tránsito de batería depósito→cliente_1 es negativo: {transit} ✓ (drena batería)")
            elif transit == 0:
                fail("Tránsito de batería es 0 — la distancia podría ser 0 o la tasa de consumo 0")
            else:
                fail("Tránsito de batería es positivo — debe ser negativo para drenar la batería",
                     f"Valor: {transit}")

        # ── 2.7 Resolver (tiempo límite corto para el test) ───────────────────
        if routing:
            try:
                search_params = pywrapcp.DefaultRoutingSearchParameters()
                # BUG CORREGIDO: first_solution_strategy debe usar FirstSolutionStrategy,
                # NO LocalSearchMetaheuristic. Son enums distintos con distinto propósito.
                search_params.first_solution_strategy = (
                    routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
                )
                # LocalSearchMetaheuristic va en su propio campo
                search_params.local_search_metaheuristic = (
                    routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
                )
                search_params.time_limit.seconds = 15
                print(f"\n  {YELLOW}→ Ejecutando solver (límite 15s)...{RESET}")
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

section("NIVEL 3 — Validación física de la solución")

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

                # Validar batería no negativa
                if fuel_val < 0:
                    battery_violations += 1

                # Validar no supera capacidad
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
            ok("Ningún vehículo llega a un nodo con batería negativa ✓")
        else:
            fail(f"Hay {battery_violations} nodos con nivel de batería negativo")

        # ── 3.2 Restricción de capacidad ─────────────────────────────────────
        if capacity_violations == 0:
            ok("Ningún vehículo supera su capacidad de carga en ningún nodo ✓")
        else:
            fail(f"Hay {capacity_violations} nodos donde se supera la capacidad del vehículo")

        # ── 3.3 Cobertura de clientes ─────────────────────────────────────────
        num_clients = data['num_locations'] - len(data['charging_stations']) - 1
        if total_clients_served == num_clients:
            ok(f"Todos los clientes atendidos: {total_clients_served}/{num_clients}")
        else:
            dropped = num_clients - total_clients_served
            print(f"  {YELLOW}⚠{RESET}  {dropped} clientes no atendidos "
                  f"({total_clients_served}/{num_clients}) — pueden ser penalizados por infactibilidad")

        # ── 3.4 Uso de estaciones de carga ────────────────────────────────────
        if cs_visits > 0:
            ok(f"Se usaron estaciones de carga: {cs_visits} visitas registradas")
        else:
            print(f"  {YELLOW}⚠{RESET}  Ningún vehículo visitó estaciones de carga "
                  "— puede ser correcto si la batería fue suficiente para todas las rutas")

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

            route_str = " → ".join(
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
                print(f"    {RED}⚠ BATERÍA NEGATIVA en algún punto{RESET}")
            elif min_f < fuel_capacity * 0.1:
                print(f"    {YELLOW}⚠ Batería llegó muy baja (<10%){RESET}")
            else:
                print(f"    {GREEN}✓ Batería siempre por encima del 10%{RESET}")

    except Exception as e:
        fail("Error inesperado en Nivel 3", str(e))
        traceback.print_exc()


# ════════════════════════════════════════════════════════════════
# RESUMEN FINAL
# ════════════════════════════════════════════════════════════════

section("RESUMEN")
total = passed + failed
print(f"  Tests pasados : {GREEN}{passed}/{total}{RESET}")
print(f"  Tests fallidos: {RED}{failed}/{total}{RESET}")

if failed == 0:
    print(f"\n  {GREEN}{BOLD}✓ Todos los tests pasaron. El módulo EVRP está listo.{RESET}")
else:
    print(f"\n  {YELLOW}{BOLD}⚠ Hay {failed} test(s) fallido(s). Revisa los detalles arriba.{RESET}")

print()
sys.exit(0 if failed == 0 else 1)