"""
regresion_estatico.py -- prueba de regresion del EVRP ESTATICO.

Objetivo: garantizar que un cambio en el codigo (por ejemplo, para la re-planeacion dinamica) NO altera los resultados
del EVRP estatico. Se resuelven instancias estaticas pequenas con combinaciones fijas y se comparan, contra una linea
base guardada ANTES del cambio, el objetivo, los clientes atendidos, la factibilidad y las rutas completas (nodo a nodo,
con carga y bateria en cada parada). Si todo coincide, el cambio no afecta lo estatico.

Por que es determinista: con un limite de TIEMPO, OR-Tools puede devolver rutas distintas en cada corrida aunque el
codigo no cambie (depende de cuantas soluciones alcanza a explorar). Aqui la busqueda se corta por NUMERO de soluciones
(SOLUTION_LIMIT) y el limite de tiempo solo es una cota de seguridad muy holgada. Mismo codigo => mismas rutas.

No toca tus experimentos: no modifica ningun archivo del proyecto ni escribe en problem/<distancia>/solutions_*; las
soluciones de la prueba van a una carpeta temporal que se borra al terminar. Solo escribe la linea base (--guardar).

Uso (desde la raiz del repo):
    python tests/regresion_estatico.py --guardar                      # linea base con el codigo actual
    python tests/regresion_estatico.py --guardar --evrp RUTA/evrp.py  # linea base con una copia guardada de evrp.py
    python tests/regresion_estatico.py                                # compara el codigo actual con la linea base
Devuelve codigo de salida 0 si todo es identico y 1 si hay alguna diferencia.
"""
import argparse
import datetime
import hashlib
import importlib.util
import json
import os
import re
import shutil
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

BASE_FILE = os.path.join(REPO, "tests", "regresion_estatico_base.json")

# Casos fijos: instancias estaticas pequenas + combinaciones deterministas. Parametros iguales a run.py.
CASES = [
    ("quebec_30c_6ev_4cs", "PATH_CHEAPEST_ARC", "GREEDY_DESCENT"),
    ("quebec_30c_6ev_4cs", "PATH_CHEAPEST_ARC", "GUIDED_LOCAL_SEARCH"),
    ("quebec_40c_4ev_6cs", "PATH_CHEAPEST_ARC", "GREEDY_DESCENT"),
    ("quebec_40c_4ev_6cs", "PATH_CHEAPEST_ARC", "GUIDED_LOCAL_SEARCH"),
]
BATTERY = 100           # vehicle_maximum_travel_distance en run.py
CONSUMPTION = 3.0       # vehicle_speed en run.py (consumo por km con OSRM)
STATION_COPIES = 3
SOLUTION_LIMIT = 300    # la busqueda para tras esta cantidad de soluciones (determinista)
SAFETY_TIME = 600       # cota de seguridad en segundos; si se alcanza, el resultado deja de ser determinista


def load_evrp(path):
    """Carga evrp.py desde `path` en lugar de problem/execute/evrp.py (para la linea base con una copia guardada)."""
    import problem.execute as pkg
    spec = importlib.util.spec_from_file_location("problem.execute.evrp", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["problem.execute.evrp"] = module
    spec.loader.exec_module(module)
    pkg.evrp = module
    return module


def limit_by_solutions():
    """Hace que las busquedas paren por numero de soluciones y no por tiempo (sin tocar utils/execute_algorithm.py)."""
    from ortools.constraint_solver import pywrapcp
    import utils.execute_algorithm as ea

    class _Pywrapcp:
        def DefaultRoutingSearchParameters(self):
            params = pywrapcp.DefaultRoutingSearchParameters()
            params.solution_limit = SOLUTION_LIMIT
            return params

        def __getattr__(self, name):
            return getattr(pywrapcp, name)

    ea.pywrapcp = _Pywrapcp()


def parse_solution(path):
    """Extrae del .txt lo que debe coincidir (todo menos el tiempo de ejecucion)."""
    with open(path, encoding="utf-8") as f:
        text = f.read()
    grab = lambda pat: (re.search(pat, text, re.M).group(1).strip() if re.search(pat, text, re.M) else None)
    routes = re.findall(r"^Route for vehicle \d+:\n(.*)$", text, re.M)
    return {
        "objective": grab(r"^Objective: (.*)$"),
        "served": grab(r"^Customers served: (.*)$"),
        "feasibility": grab(r"^Feasibility check: (.*)$"),
        "completed_by_repair": grab(r"^Customers inserted by the completion step: (.*)$"),
        "total_distance": grab(r"^Total Distance of all routes: (.*)$"),
        "real_distance_km": grab(r"^Real distance \(km\): (.*)$"),
        "recharges": grab(r"^Total recharges: (.*)$"),
        "routes": [r.strip() for r in routes],
        "execution_time": float(grab(r"^Execution Time: (.*)$") or 0),
    }


def run_cases():
    from problem.problem_type import ProblemType, execute
    from problem.strategy_type import HeuristicType, MetaheuristicType
    from distance.distance_type import DistanceType

    limit_by_solutions()
    results, cwd = {}, os.getcwd()
    tmp = tempfile.mkdtemp(prefix="regresion_evrp_")
    try:
        os.chdir(tmp)       # save_solution escribe en rutas relativas: asi todo queda en la carpeta temporal
        for instance, h, m in CASES:
            key = f"{instance} | {h} + {m}"
            print(f"\n>>> {key}")
            execute(problem_type=ProblemType.EVRP,
                    instance=os.path.join(REPO, "instances_data", "evrp_instances", instance + ".txt"),
                    distance_type=DistanceType.OSRM, time_limit=SAFETY_TIME, executions=1,
                    vehicle_maximum_travel_distance=BATTERY, vehicle_max_time=None, vehicle_speed=CONSUMPTION,
                    station_copies=STATION_COPIES,
                    heuristic=HeuristicType[h], metaheuristic=MetaheuristicType[m])
            found = [os.path.join(d, f) for d, _, fs in os.walk(tmp) for f in fs
                     if f == instance + ".txt" and f"solutions_{h}_and_{m}" in d]
            if not found:
                results[key] = {"error": "OR-Tools no devolvio solucion"}
                continue
            res = parse_solution(found[0])
            if res["execution_time"] >= 0.95 * SAFETY_TIME:
                res["warning"] = "se alcanzo la cota de tiempo: el resultado puede no ser determinista"
            results[key] = res
            shutil.rmtree(os.path.join(tmp, "problem"), ignore_errors=True)
    finally:
        os.chdir(cwd)
        shutil.rmtree(tmp, ignore_errors=True)
    return results


def compare(base, now):
    fields = ["objective", "served", "feasibility", "completed_by_repair", "total_distance", "real_distance_km",
              "recharges", "routes", "error"]
    all_ok = True
    print("\n" + "=" * 78 + "\nRESULTADO DE LA REGRESION (estatico)\n" + "=" * 78)
    for key in sorted(set(base) | set(now)):
        b, n = base.get(key), now.get(key)
        if b is None or n is None:
            print(f"[DIFERENTE] {key}: caso {'nuevo' if b is None else 'que falta'} respecto a la linea base")
            all_ok = False
            continue
        diffs = [f for f in fields if b.get(f) != n.get(f)]
        for w in (b.get("warning"), n.get("warning")):
            if w:
                print(f"[AVISO] {key}: {w}")
        if not diffs:
            print(f"[IDENTICO] {key}: objetivo {n.get('objective')}, {n.get('served')} clientes, "
                  f"{len(n.get('routes', []))} rutas")
            continue
        all_ok = False
        print(f"[DIFERENTE] {key}")
        for f in diffs:
            if f == "routes":
                for i, (rb, rn) in enumerate(zip(b.get("routes", []), n.get("routes", []))):
                    if rb != rn:
                        print(f"    ruta {i} antes : {rb}\n    ruta {i} ahora : {rn}")
                if len(b.get("routes", [])) != len(n.get("routes", [])):
                    print(f"    numero de rutas: {len(b.get('routes', []))} -> {len(n.get('routes', []))}")
            else:
                print(f"    {f}: {b.get(f)!r} -> {n.get(f)!r}")
    print("=" * 78)
    print("OK: el codigo actual da EXACTAMENTE los mismos resultados estaticos que la linea base."
          if all_ok else "ATENCION: hay diferencias en el caso estatico (ver arriba).")
    return all_ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--guardar", action="store_true", help="guarda la linea base en lugar de comparar")
    ap.add_argument("--evrp", help="ruta a una copia de evrp.py a usar en lugar de problem/execute/evrp.py")
    ap.add_argument("--base", default=BASE_FILE, help="archivo de la linea base (json)")
    args = ap.parse_args()

    evrp_path = os.path.abspath(args.evrp) if args.evrp else os.path.join(REPO, "problem", "execute", "evrp.py")
    if args.evrp:
        load_evrp(evrp_path)
    with open(evrp_path, "rb") as f:
        evrp_sha = hashlib.sha256(f.read()).hexdigest()[:12]
    import ortools
    print(f"evrp.py usado: {os.path.relpath(evrp_path, REPO)} (sha256 {evrp_sha}) | OR-Tools {ortools.__version__}")

    results = run_cases()
    for r in results.values():
        r.pop("execution_time", None)

    if args.guardar:
        payload = {"creado": datetime.datetime.now().isoformat(timespec="seconds"),
                   "evrp_py": os.path.relpath(evrp_path, REPO).replace(os.sep, "/"),
                   "evrp_sha256": evrp_sha, "ortools": ortools.__version__, "solution_limit": SOLUTION_LIMIT,
                   "resultados": results}
        with open(args.base, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print(f"\nLinea base guardada en {args.base} ({len(results)} casos).")
        return 0

    if not os.path.exists(args.base):
        print(f"No existe la linea base {args.base}. Generala primero con --guardar.")
        return 1
    with open(args.base, encoding="utf-8") as f:
        base = json.load(f)
    if base.get("ortools") != ortools.__version__:
        print(f"[AVISO] la linea base se genero con OR-Tools {base.get('ortools')} y ahora hay {ortools.__version__}: "
              f"una version distinta de OR-Tools puede cambiar las rutas aunque el codigo sea el mismo.")
    print(f"Linea base: {base.get('creado')} | evrp.py sha256 {base.get('evrp_sha256')}")
    return 0 if compare(base["resultados"], results) else 1


if __name__ == "__main__":
    sys.exit(main())
