"""
diagnostico_ortools.py -- por que OR-Tools devuelve soluciones vacias o con clientes descartados.

No cambia el modelo. Ejecuta problem/execute/evrp.py tal cual y, por fuera, mide que hace el solucionador:
estado final de OR-Tools, cuantos clientes descarto, cuanto tardo en la primera solucion, cuantas soluciones
encontro y cuanto cuesta el arco medio del modelo (la escala que usa Guided Local Search). Repite cada instancia
con variantes que desactivan UN cambio a la vez, para saber cual rompe la busqueda:

  actual                modelo tal como esta
  sin_cadenas           sin la regla estacion -> estacion
  sin_deposito_estacion sin la regla deposito -> estacion
  sin_prohibiciones     sin ninguna de las dos reglas
  escala_1              costos en km enteros (equivale al modelo anterior, truncado)
  escala_100            costos en centesimas de km
  1_copia               una sola copia por estacion (station_copies=1)
  PATH_CHEAPEST_ARC     primera solucion explicita en vez de AUTOMATIC
  recarga_libre         la recarga en estaciones puede ser parcial (modelo anterior); "actual" fija recarga completa
  sin_suma              sin la cota Cumul+Slack<=capacidad en estaciones (y recarga libre)
  sin_saltos_copias     sin prohibir el salto entre copias de una misma estacion
  modelo_desnudo        solo lo nativo de OR-Tools: sin orden de copias, sin cota, sin arcos prohibidos
  SAVINGS / PARALLEL_INSERTION / GLOBAL_CHEAPEST_ARC   otras primeras soluciones
  2_copias              dos copias por estacion
  sin_orden_copias      sin la restriccion "la copia k+1 solo si esta la k"
  sin_finalizer         sin AddVariableMaximizedByFinalizer en las estaciones

Uso (raiz del repo, con OSRM o su cache disponible):
    python diagnostico_ortools.py --quick                  # 30c y 100c, 8 variantes (~5 min)
    python diagnostico_ortools.py                          # 30c, 50c, 100c y 220c (~11 min)
    python diagnostico_ortools.py --instances quebec_100c_12ev_12cs --time 30 --variants actual escala_1

Salida: diagnostico_ortools.txt (tabla y lectura automatica). No escribe soluciones en problem/.
"""
import argparse
import os
import sys
import time
import traceback

from distance.distance_type import DistanceType
from problem.strategy_type import HeuristicType, MetaheuristicType
from problem.execute import evrp
from utils import units

REPORT = "diagnostico_ortools.txt"
INSTANCES_DIR = os.path.join("instances_data", "evrp_instances")
DEFAULT_INSTANCES = ["quebec_30c_6ev_4cs", "quebec_50c_5ev_5cs", "quebec_100c_12ev_12cs", "quebec_220c_26ev_20cs"]
QUICK_INSTANCES = ["quebec_30c_6ev_4cs", "quebec_100c_12ev_12cs"]

STATUS = {0: "NOT_SOLVED", 1: "SUCCESS", 2: "PARTIAL_SUCCESS_LOCAL_OPTIMUM_NOT_REACHED", 3: "FAIL",
          4: "FAIL_TIMEOUT", 5: "INVALID", 6: "INFEASIBLE", 7: "OPTIMAL"}

# Variantes pensadas para aislar que elemento del modelo hace crecer el tiempo con el numero de estaciones-copia.
# "actual" fija la recarga completa; "recarga_libre" es el modelo anterior.
VARIANTS = {
    "actual": {},
    "sin_cadenas": {"chains": False},
    "sin_deposito_estacion": {"depot": False},
    "sin_prohibiciones": {"chains": False, "depot": False},
    "escala_1": {"scale": 1},
    "escala_100": {"scale": 100},
    "1_copia": {"copies": 1},
    "PATH_CHEAPEST_ARC": {"heuristic": HeuristicType.PATH_CHEAPEST_ARC},
    "recarga_libre": {"full_recharge": False},
    "sin_suma": {"full_recharge": False, "no_bound": True},
    "sin_saltos_copias": {"no_copy_hops": True},
    "modelo_desnudo": {"full_recharge": False, "no_bound": True, "no_order": True, "no_forbidden": True},
    "SAVINGS": {"heuristic": HeuristicType.SAVINGS},
    "PARALLEL_INSERTION": {"heuristic": HeuristicType.PARALLEL_CHEAPEST_INSERTION},
    "GLOBAL_CHEAPEST_ARC": {"heuristic": HeuristicType.GLOBAL_CHEAPEST_ARC},
    "lns": {"lns": True},
    "lns_insercion": {"lns_insertion": True},
    "lns_insercion_GLOBAL": {"lns_insertion": True, "heuristic": HeuristicType.GLOBAL_CHEAPEST_ARC},
    "lns_PARALLEL": {"lns": True, "heuristic": HeuristicType.PARALLEL_CHEAPEST_INSERTION},
    "lns_GLOBAL_CHEAPEST_ARC": {"lns": True, "heuristic": HeuristicType.GLOBAL_CHEAPEST_ARC},
    "2_copias": {"copies": 2},
    "sin_orden_copias": {"no_order": True},
    "sin_finalizer": {"no_finalizer": True},
}
# una variante por cada heuristica de primera solucion (con Guided Local Search), para ver cuales dan soluciones completas
for _h in HeuristicType:
    VARIANTS.setdefault(_h.name, {"heuristic": _h})
LOG_SEARCH = False
LNS_APPLIED = []

# ---------------------------------------------------------------- sondas (se enganchan al modelo sin tocarlo)
PROBE = {}
_CFG = {}
_orig = {
    "chains": evrp.forbid_station_chains,
    "depot": evrp.forbid_depot_to_station,
    "apply": evrp.apply_forbidden_arcs,
    "save": evrp.save_solution,
    "scale": units.DIST_SCALE,
    "expand": evrp.expand_station_copies,
    "fuel": evrp.add_fuel_constraints,
    "params": evrp.pywrapcp.DefaultRoutingSearchParameters,
}


def _hooked_apply(routing, manager, data):
    """Se llama justo antes de resolver: aplica los arcos prohibidos y registra el modelo y un callback por solucion."""
    if _CFG.get("no_forbidden"):
        data = dict(data, forbidden_arcs=set())
    elif _CFG.get("no_copy_hops"):
        origin = data.get("station_origin", {})
        data = dict(data, forbidden_arcs={(a, b) for a, b in data.get("forbidden_arcs", ())
                                          if origin.get(a, a) != origin.get(b, b)})
    _orig["apply"](routing, manager, data)
    n = data["num_locations"]
    costs = [units.cost_int(data["distance_matrix"][i][j]) for i in range(n) for j in range(n) if i != j]
    n_clients = n - len(data["charging_stations"]) - 1
    PROBE.clear()
    PROBE.update(
        t0=time.time(), events=[], nodes=n, vehicles=data["num_vehicles"], stations=len(data["charging_stations"]),
        forbidden=len(data.get("forbidden_arcs", ())), penalty=evrp.drop_penalty_of(data),
        mean_cost=sum(costs) / len(costs), max_cost=max(costs), n_clients=n_clients,
    )

    def on_solution():
        now = time.time() - PROBE["t0"]
        dropped = None
        try:
            dropped = sum(1 for c in range(1, n_clients + 1)
                          if routing.NextVar(manager.NodeToIndex(c)).Value() == manager.NodeToIndex(c))
        except Exception:
            pass
        PROBE["events"].append((now, dropped))

    routing.AddAtSolutionCallback(on_solution)


def _hooked_save(data, manager, routing, assignment, instance, heuristic, metaheuristic, elapsed, i, distance_type):
    """Sustituye a save_solution: no escribe archivos, solo anota el resultado."""
    n_clients = data["num_locations"] - len(data["charging_stations"]) - 1
    origin = data.get("station_origin", {})
    stations = set(data["charging_stations"])
    dm = data["distance_matrix"]
    dropped = sum(1 for c in range(1, n_clients + 1)
                  if assignment.Value(routing.NextVar(manager.NodeToIndex(c))) == manager.NodeToIndex(c))
    km, recharges, chains = 0.0, 0, 0
    for v in range(data["num_vehicles"]):
        idx, prev = routing.Start(v), None
        while True:
            node = origin.get(manager.IndexToNode(idx), manager.IndexToNode(idx))
            if prev is not None:
                km += dm[prev][node]
                if prev in stations and node in stations:
                    chains += 1
            if node in stations:
                recharges += 1
            prev = node
            if routing.IsEnd(idx):
                break
            idx = assignment.Value(routing.NextVar(idx))
    depot = data["depot"]
    cap = units.cap_int(data["fuel_capacity"])
    rate = data["fuel_consumption_rate"]
    lost = [c for c in range(1, n_clients + 1)
            if assignment.Value(routing.NextVar(manager.NodeToIndex(c))) == manager.NodeToIndex(c)]
    far = [c for c in lost if units.energy_int(dm[depot][c] + dm[c][depot], rate) > cap]   # ida y vuelta sin recargar no cabe
    PROBE.update(lost=lost, far_lost=len(far))
    PROBE.update(dropped=dropped, km=km, recharges=recharges, chains=chains, elapsed=elapsed,
                 status=STATUS.get(routing.status(), str(routing.status())))


def _configure(opts):
    evrp.forbid_station_chains = _orig["chains"] if opts.get("chains", True) else (lambda data, keep=(): data)
    evrp.forbid_depot_to_station = _orig["depot"] if opts.get("depot", True) else (lambda data: data)
    scale = opts.get("scale", _orig["scale"])
    units.DIST_SCALE = scale
    evrp.DIST_SCALE = scale
    _CFG.clear()
    _CFG.update(opts)
    evrp.FULL_RECHARGE = opts.get("full_recharge", False)
    evrp.RECHARGE_BOUND = not opts.get("no_bound", False)
    evrp.apply_forbidden_arcs = _hooked_apply
    evrp.save_solution = _hooked_save

    if opts.get("no_order"):
        def expand_without_order(*a, **k):
            data = dict(_orig["expand"](*a, **k))
            data.pop("station_copies_of", None)      # sin esto no se anade ActiveVar(k+1) <= ActiveVar(k)
            return data
        evrp.expand_station_copies = expand_without_order
    else:
        evrp.expand_station_copies = _orig["expand"]

    if opts.get("no_finalizer"):
        def fuel_without_finalizer(routing, *a, **k):
            routing.AddVariableMaximizedByFinalizer = lambda *args, **kw: None
            return _orig["fuel"](routing, *a, **k)
        evrp.add_fuel_constraints = fuel_without_finalizer
    else:
        evrp.add_fuel_constraints = _orig["fuel"]

    def params():
        p = _orig["params"]()
        if LOG_SEARCH:
            p.log_search = True       # el log de OR-Tools sale por la consola
        if _CFG.get("lns_insertion"):
            # operadores de reinsercion por vecindad: reconstruyen con una heuristica de insercion un grupo de nodos cercanos,
            # incluidos los descartados, asi pueden insertar a la vez una estacion y varios clientes de una zona lejana
            ops = p.local_search_operators
            names = ("use_global_cheapest_insertion_path_lns", "use_local_cheapest_insertion_path_lns",
                     "use_global_cheapest_insertion_close_nodes_lns", "use_local_cheapest_insertion_close_nodes_lns",
                     "use_global_cheapest_insertion_expensive_chain_lns", "use_local_cheapest_insertion_expensive_chain_lns",
                     "use_relocate_path_global_cheapest_insertion_insert_unperformed", "use_inactive_lns", "use_path_lns")
            applied = [n for n in names if hasattr(ops, n)]
            for n in applied:
                setattr(ops, n, 2)     # BOOL_TRUE
            PROBE.setdefault("lns_applied", applied)
            LNS_APPLIED[:] = applied
        if _CFG.get("lns"):
            # operadores de vecindario grande: re-optimizan varios nodos a la vez, asi pueden insertar un cliente lejano
            # junto con la estacion que necesita (los operadores normales insertan un solo nodo por movimiento)
            ops = p.local_search_operators
            for name in ("use_path_lns", "use_inactive_lns", "use_full_path_lns", "use_tsp_lns"):
                if hasattr(ops, name):
                    setattr(ops, name, 2)      # BOOL_TRUE
        return p
    evrp.pywrapcp.DefaultRoutingSearchParameters = params


def run_one(instance, name, opts, time_limit):
    _configure(opts)
    PROBE.clear()
    PROBE["status"] = "SIN_SOLUCION"
    try:
        evrp.execute(
            i=0, instance_type=os.path.join(INSTANCES_DIR, instance + ".txt"), time_limit=time_limit,
            vehicle_maximum_travel_distance=100, vehicle_speed=3.0, distance_type=DistanceType.OSRM,
            heuristic=opts.get("heuristic"), metaheuristic=MetaheuristicType.GUIDED_LOCAL_SEARCH,
            station_copies=opts.get("copies", 3),
        )
    except Exception:
        PROBE["status"] = "ERROR: " + traceback.format_exc().strip().splitlines()[-1][:120]
    ev = PROBE.get("events", [])
    return {
        "instance": instance, "variant": name, "status": PROBE.get("status"),
        "dropped": PROBE.get("dropped"), "n_clients": PROBE.get("n_clients"), "km": PROBE.get("km"),
        "recharges": PROBE.get("recharges"), "chains": PROBE.get("chains"),
        "first_s": ev[0][0] if ev else None, "first_dropped": ev[0][1] if ev else None,
        "n_solutions": len(ev), "last_s": ev[-1][0] if ev else None,
        "forbidden": PROBE.get("forbidden"), "penalty": PROBE.get("penalty"),
        "mean_cost": PROBE.get("mean_cost"), "max_cost": PROBE.get("max_cost"),
        "nodes": PROBE.get("nodes"), "lost": PROBE.get("lost"), "far_lost": PROBE.get("far_lost"),
    }


def fmt(v, nd=1):
    return "-" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instances", nargs="*", default=None)
    ap.add_argument("--variants", nargs="*", default=None, choices=list(VARIANTS))
    ap.add_argument("--time", type=int, default=20, help="segundos por corrida (por defecto 20)")
    ap.add_argument("--quick", action="store_true", help="solo 30c y 100c")
    ap.add_argument("--all-heuristics", action="store_true",
                    help="prueba 'actual' y las 15 heuristicas de primera solucion, todas con Guided Local Search")
    ap.add_argument("--log", action="store_true", help="imprime el log de busqueda de OR-Tools (redirigir a un archivo)")
    args = ap.parse_args()
    global LOG_SEARCH
    LOG_SEARCH = args.log
    instances = args.instances or (QUICK_INSTANCES if args.quick else DEFAULT_INSTANCES)
    variants = args.variants or list(VARIANTS)
    if args.all_heuristics:
        variants = ["actual"] + [h.name for h in HeuristicType]

    rows = []
    for inst in instances:
        for name in variants:
            print(f"[{inst}] {name} ...", flush=True)
            r = run_one(inst, name, VARIANTS[name], args.time)
            rows.append(r)
            print(f"    estado={r['status']} descartados={fmt(r['dropped'])}/{fmt(r['n_clients'])} km={fmt(r['km'], 2)} "
                  f"1a solucion={fmt(r['first_s'])}s soluciones={r['n_solutions']}", flush=True)

    lines = [f"Diagnostico de OR-Tools (GLS, {args.time} s por corrida, heuristica AUTOMATIC salvo indicacion)", ""]
    head = (f"{'instancia':24s}{'variante':24s}{'estado':12s}{'descart.':>9s}{'km':>9s}{'recarg.':>8s}{'cadenas':>8s}"
            f"{'1a sol(s)':>10s}{'desc.1a':>8s}{'n_sol':>6s}{'ult(s)':>8s}{'arcos_proh':>11s}{'costo_medio':>12s}")
    lines += [head, "-" * len(head)]
    for r in rows:
        lines.append(f"{r['instance']:24s}{r['variant']:24s}{str(r['status'])[:11]:12s}{fmt(r['dropped']):>9s}"
                     f"{fmt(r['km'], 2):>9s}{fmt(r['recharges']):>8s}{fmt(r['chains']):>8s}{fmt(r['first_s']):>10s}"
                     f"{fmt(r['first_dropped']):>8s}{r['n_solutions']:>6d}{fmt(r['last_s']):>8s}{fmt(r['forbidden']):>11s}"
                     f"{fmt(r['mean_cost'], 0):>12s}")
    for r in rows:
        if r["variant"].startswith("lns_insercion"):
            lines += ["", f"Operadores LNS de insercion aplicados ({r['variant']}): {LNS_APPLIED or 'ninguno (el protobuf de esta version no los tiene)'}"]
            break
    lines += ["", "Clientes descartados (cuantos de ellos son 'lejanos': la ida y vuelta desde el deposito no cabe en una carga):"]
    for r in rows:
        if r["lost"]:
            lines.append(f"  {r['instance']} / {r['variant']}: {len(r['lost'])} descartados, {r['far_lost']} lejanos; ids {r['lost'][:12]}")
    lines += ["", "Modelo (instancia, nodos, estaciones con copias, penalizacion por descarte, costo de arco medio y maximo):"]
    seen = set()
    for r in rows:
        if r["variant"] == "actual" and r["instance"] not in seen:
            seen.add(r["instance"])
            lines.append(f"  {r['instance']}: nodos={r['nodes']} penalizacion={fmt(r['penalty'])} "
                         f"costo_medio={fmt(r['mean_cost'], 0)} costo_max={fmt(r['max_cost'], 0)} (en unidades internas)")
    lines += ["", "Lectura automatica:"]
    for inst in instances:
        ok = [r["variant"] for r in rows if r["instance"] == inst and r["dropped"] == 0]
        bad = [r["variant"] for r in rows if r["instance"] == inst and r["dropped"] != 0]
        base = next((r for r in rows if r["instance"] == inst and r["variant"] == "actual"), None)
        if base is None:
            continue
        if base["dropped"] == 0:
            lines.append(f"  {inst}: el modelo actual NO descarta clientes ({fmt(base['km'], 2)} km).")
        else:
            lines.append(f"  {inst}: el modelo actual descarta {fmt(base['dropped'])} clientes. "
                         f"Variantes que lo arreglan: {ok or 'ninguna'}. Siguen fallando: {bad}.")
    open(REPORT, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\nGuardado en {REPORT}")


if __name__ == "__main__":
    sys.exit(main())
