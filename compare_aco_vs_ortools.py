"""
compare_aco_vs_ortools.py — ACO vs Guided Local Search (OR-Tools) sobre las
instancias EVRP, mismas condiciones (OSRM, bateria 100, consumo 3.0, mismo
time_limit). Sin simulacion.

Uso (desde la raiz del repo, con el .venv y el servidor OSRM corriendo, o con
las matrices ya en instances_data/osrm_cache):

    python compare_aco_vs_ortools.py
    python compare_aco_vs_ortools.py --time 10 --seeds 1 2 3 --skip-ortools

--skip-ortools reutiliza las soluciones GLS ya guardadas en
problem/osrm/solutions_evrp_0/solutions_GUIDED_LOCAL_SEARCH/ en vez de volver
a correr OR-Tools.

Ademas del objetivo reporta si cada solucion es fisicamente factible: la
bateria se recalcula arco a arco desde la matriz (arranca llena, se recarga a
100 al llegar a una estacion) y debe ser >= 0 SIEMPRE, incluido el tramo que
sale de una estacion.
"""
import argparse
import os
import re
import statistics

from distance.distance_type import DistanceType
from instance.instance_type import InstanceType, process_files
from p_mh.ACO import evrp_aco

CAP = 100
RATE = 3.0
INSTANCES = [
    "quebec_30c_6ev_4cs", "quebec_40c_4ev_6cs", "quebec_50c_5ev_5cs",
    "quebec_60c_8ev_8cs", "quebec_90c_10ev_10cs",
]


def parse(path):
    txt = open(path, encoding="utf-8").read()
    obj = int(float(re.search(r"Objective: ([\d.]+)", txt).group(1)))
    rech = int(re.search(r"Total recharges: (\d+)", txt).group(1))
    routes = []
    for blk in re.findall(r"Route for vehicle \d+:\n (.*?)\nDistance", txt):
        routes.append([int(re.match(r"(\d+)", s.strip()).group(1)) for s in blk.split("->")])
    return obj, rech, routes


def strict_check(routes, data):
    """(factible, nodos_servidos_ok): bateria fisica >= 0 y todos los clientes una vez."""
    dm, cs = data["distance_matrix"], set(data["charging_stations"])
    n_clients = data["num_locations"] - len(cs) - 1
    served, ok = [], True
    for nodes in routes:
        bat = CAP
        for a, b in zip(nodes, nodes[1:]):
            bat -= int(dm[a][b] * RATE)
            if bat < 0:
                ok = False
            if b in cs:
                bat = CAP
            elif b != 0:
                served.append(b)
    return ok and sorted(served) == list(range(1, n_clients + 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--time", type=int, default=10)
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    ap.add_argument("--skip-ortools", action="store_true")
    args = ap.parse_args()

    base = "problem/osrm"
    gls_dir = f"{base}/solutions_evrp_0/solutions_GUIDED_LOCAL_SEARCH"
    if not args.skip_ortools:
        from problem.execute import evrp
        from problem.strategy_type import MetaheuristicType
        print("== OR-Tools GLS ==")
        evrp.execute(
            i=0, instance_type=InstanceType.EVRP, time_limit=args.time,
            vehicle_maximum_travel_distance=CAP, vehicle_speed=RATE,
            distance_type=DistanceType.OSRM, heuristic=None,
            metaheuristic=MetaheuristicType.GUIDED_LOCAL_SEARCH,
        )

    aco_dirs = {}
    for seed in args.seeds:
        print(f"== ACO seed {seed} ==")
        evrp_aco.execute(
            i=1000 + seed, instance_type=InstanceType.EVRP, time_limit=args.time,
            vehicle_maximum_travel_distance=CAP, vehicle_speed=RATE,
            distance_type=DistanceType.OSRM, seed=seed,
        )
        aco_dirs[seed] = f"{base}/solutions_evrp_{1000 + seed}/solutions_ACO"

    data_all = process_files(InstanceType.EVRP, DistanceType.OSRM, None, RATE, CAP)

    # Solo el MEJOR de ACO (entre las semillas) contra el MEJOR de GLS.
    print(f"\n{'instancia':<24}{'GLS':>6}{'ok':>4}{'rech':>5} |{'ACO mejor':>10}{'semilla':>8}{'ok':>4}{'rech':>5}{'dif':>6}{'gap%':>7}")
    for name in INSTANCES:
        fname = f"{name}.txt"
        data = data_all[fname]
        g_obj, g_rech, g_routes = parse(f"{gls_dir}/{fname}")
        g_ok = strict_check(g_routes, data)
        res = {seed: parse(f"{d}/{fname}") for seed, d in aco_dirs.items()}
        best_seed = min(res, key=lambda s: res[s][0])
        a_obj, a_rech, a_routes = res[best_seed]
        a_ok = strict_check(a_routes, data)
        print(f"{name:<24}{g_obj:>6}{'si' if g_ok else 'NO':>4}{g_rech:>5} |{a_obj:>10}{best_seed:>8}{'si' if a_ok else 'NO':>4}{a_rech:>5}"
              f"{a_obj - g_obj:>+6}{(a_obj - g_obj) / g_obj * 100:>+7.1f}")
    print("\nok = factible con bateria fisica estricta (>=0 en todos los tramos). dif/gap < 0: ACO mejor que GLS.")


if __name__ == "__main__":
    main()
