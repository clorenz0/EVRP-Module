"""
run_aco.py — ejecuta el ACO de forma independiente (sin OR-Tools).

Ejemplos (desde la raiz del repo):

    python run_aco.py                                  # las 5 instancias, las 6 semillas, 10 s
    python run_aco.py --instance quebec_50c_5ev_5cs    # una instancia, las 6 semillas
    python run_aco.py --instance quebec_50c_5ev_5cs --seed 3        # una instancia, una semilla
    python run_aco.py --seeds 1 2 3 --time 20          # semillas elegidas, 20 s por instancia

Para cada (instancia, semilla) guarda la solucion en
    problem/osrm/solutions_evrp_<1000+semilla>/solutions_ACO/<instancia>.txt
y al final deja la MEJOR de las semillas de cada instancia en
    problem/osrm/solutions_evrp_best/solutions_ACO/<instancia>.txt
(esa carpeta es la que lee friedman_ranking.py). Imprime la tabla de mejores y
de que semilla salio cada una.

ACO + heuristica de OR-Tools (opcional): la heuristica solo aporta la solucion inicial; ACO sigue
siendo independiente y no se toca su codigo. Se elige aqui con --heuristic o en ACO_PARAMS:

    python run_aco.py --heuristic PATH_CHEAPEST_ARC            # guarda como solutions_PATH_CHEAPEST_ARC_and_ACO
    python run_aco.py --heuristic SAVINGS --seed 3 --instance quebec_50c_5ev_5cs

Heuristicas disponibles: las de problem/strategy_type.py (HeuristicType). No se combina con
metaheuristicas de OR-Tools (GLS, TS, ...): ACO ya cumple ese papel.

Los parametros del algoritmo estan abajo, en ACO_PARAMS.
"""
import argparse
import glob
import os
import re
import shutil

from p_mh.ACO import evrp_aco
from distance.distance_type import DistanceType
from instance.instance_type import InstanceType

############
#  PARAMETROS DEL ACO
############
# vehicle_maximum_travel_distance => capacidad de bateria (fuel_capacity)
# vehicle_speed => consumo de bateria por unidad de distancia (3.0 con OSRM, como run.py)
# num_ants => hormigas por iteracion (None = automatico)
# alpha => importancia de la feromona (None = automatico: 1.0 si n<=50 nodos, 2.0 si n>50)
# beta => importancia de la heuristica 1/distancia
# gamma => importancia del termino de energia (carga proactiva, Thymianis et al.)
# rho => evaporacion
# proactive_charge_threshold => fraccion de bateria bajo la cual una estacion entra a la ruleta
# candidate_list_size => clientes factibles mas cercanos evaluados por paso
# local_search_top => a cuantas mejores hormigas por iteracion se les aplica la busqueda local
#     (relocate, Or-opt, swap, 2-opt*, 2-opt, relocate intra-ruta, poda de cargas)
# pheromone_scheme => 'mmas' (MAX-MIN, por defecto) o 'rank_as'
# ils_per_iter => perturbaciones (destruir+reconstruir+busqueda local) de la mejor solucion por iteracion
# restart_patience => reinicio de feromona tras N iteraciones sin mejora (0 = apagado; con 10 empeora)
# initial_heuristic => None (ACO solo) o una heuristica de OR-Tools, p.ej. HeuristicType.PATH_CHEAPEST_ARC o 'SAVINGS'
# initial_time_limit => segundos que se le dan a esa heuristica
# workers => >1 paraleliza la busqueda local (procesos); en Windows exige el `if __name__ == "__main__"`
ACO_PARAMS = dict(
    vehicle_maximum_travel_distance=100,
    vehicle_speed=3.0,
    distance_type=DistanceType.OSRM,
    num_ants=None,
    alpha=None,
    beta=1.5,
    gamma=2.5,
    rho=0.5,
    proactive_charge_threshold=0.15,
    candidate_list_size=20,
    local_search_top=3,
    pheromone_scheme='mmas',
    ils_per_iter=2,
    restart_patience=0,
    workers=1,
    initial_heuristic=None,
    initial_time_limit=2,
)

ALL_SEEDS = [1, 2, 3, 4, 5, 6]
INSTANCES_DIR = os.path.join("instances_data", "evrp_instances")
OUT_ROOT = os.path.join("problem", "osrm")


def objective_of(path):
    with open(path, encoding="utf-8") as f:
        return int(re.search(r"Objective: (\d+)", f.read()).group(1))


def resolve_instances(names):
    """'all' o lista de nombres (con o sin .txt) -> lista de rutas de instancia."""
    if not names or names == ["all"]:
        return sorted(glob.glob(os.path.join(INSTANCES_DIR, "*.txt")))
    paths = []
    for name in names:
        path = name if os.path.isfile(name) else os.path.join(INSTANCES_DIR, name if name.endswith(".txt") else name + ".txt")
        if not os.path.isfile(path):
            raise SystemExit(f"No se encontro la instancia '{name}' (busque {path}).")
        paths.append(path)
    return paths


def main():
    ap = argparse.ArgumentParser(description="ACO independiente para EVRP")
    ap.add_argument("--instance", nargs="+", default=["all"],
                    help="nombre(s) de instancia (ej. quebec_50c_5ev_5cs) o 'all' (por defecto)")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--seed", type=int, help="usar solo esta semilla")
    group.add_argument("--seeds", nargs="+", help="lista de semillas, o 'all' (1-6, por defecto)")
    ap.add_argument("--time", type=int, default=10, help="segundos por instancia y semilla (por defecto 10)")
    ap.add_argument("--heuristic", default=None,
                    help="heuristica de OR-Tools que aporta la solucion inicial (ej. PATH_CHEAPEST_ARC); por defecto ACO solo")
    args = ap.parse_args()
    params = dict(ACO_PARAMS)
    if args.heuristic:
        params["initial_heuristic"] = args.heuristic
    combo = f"{params['initial_heuristic'].name if hasattr(params['initial_heuristic'], 'name') else params['initial_heuristic']}_and_ACO" \
        if params.get("initial_heuristic") else "ACO"

    if args.seed is not None:
        seeds = [args.seed]
    elif args.seeds and args.seeds != ["all"]:
        seeds = [int(s) for s in args.seeds]
    else:
        seeds = ALL_SEEDS
    instances = resolve_instances(args.instance)
    print(f"{combo} | instancias: {[os.path.basename(p) for p in instances]} | semillas: {seeds} | {args.time}s")

    results = {}   # nombre -> {semilla: (objetivo, ruta_archivo)}
    for seed in seeds:
        out_i = 1000 + seed
        for path in instances:
            print(f"\n=== {os.path.basename(path)} | semilla {seed} ===")
            evrp_aco.execute(
                i=out_i,
                instance_type=path,                 # ruta .txt = una sola instancia
                time_limit=args.time,
                seed=seed,
                **params,
            )
            name = os.path.basename(path)
            sol = os.path.join(OUT_ROOT, f"solutions_evrp_{out_i}", f"solutions_{combo}", name)
            if os.path.isfile(sol):
                results.setdefault(name, {})[seed] = (objective_of(sol), sol)

    best_dir = os.path.join(OUT_ROOT, "solutions_evrp_best", f"solutions_{combo}")
    os.makedirs(best_dir, exist_ok=True)
    print(f"\n{'instancia':<28}{'mejor ACO':>10}{'semilla':>9}   objetivo por semilla")
    for name in sorted(results):
        by_seed = results[name]
        best_seed = min(by_seed, key=lambda s: by_seed[s][0])
        shutil.copyfile(by_seed[best_seed][1], os.path.join(best_dir, name))
        per_seed = {s: by_seed[s][0] for s in sorted(by_seed)}
        print(f"{name:<28}{by_seed[best_seed][0]:>10}{best_seed:>9}   {per_seed}")
    print(f"\nMejores guardadas en {best_dir}")


if __name__ == "__main__":
    main()
