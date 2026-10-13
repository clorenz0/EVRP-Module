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
import random
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

############
#  QUE EJECUTAR (se edita aqui; la consola es opcional y tiene prioridad si se usa)
############
# HEURISTIC:
#   None                        -> TODO: ACO solo (todas las semillas) + ACO con CADA heuristica de OR-Tools
#   "ACO"                       -> solo ACO independiente
#   HeuristicType.SAVINGS (o "SAVINGS") -> solo ACO + esa heuristica
#   [HeuristicType.SAVINGS, HeuristicType.PATH_CHEAPEST_ARC] -> ACO + cada una de esas
# Cada combinacion ACO+heuristica es UNA corrida (una semilla sorteada de SEEDS), como las de OR-Tools.
HEURISTIC = None
INSTANCES = "all"            # "all" o lista de nombres, p.ej. ["quebec_50c_5ev_5cs"]
SEEDS = [1, 2, 3, 4, 5, 6]   # semillas de ACO solo; en las combinaciones con heuristica se sortea una
TIME_LIMIT = 10              # segundos por instancia y semilla

ALL_SEEDS = SEEDS
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


def combo_name(heuristic):
    return "ACO" if heuristic is None else f"{heuristic.name}_and_ACO"


def run_combo(heuristic, instances, seeds, time_limit):
    """Una combinacion: ACO solo (heuristic=None, varias semillas) o ACO + heuristica (una semilla)."""
    params = dict(ACO_PARAMS)
    params["initial_heuristic"] = heuristic
    combo = combo_name(heuristic)
    hybrid = heuristic is not None
    if hybrid and len(seeds) > 1:
        seeds = [random.choice(seeds)]
    print(f"\n##### {combo} | instancias: {[os.path.basename(p) for p in instances]} | semillas: {seeds} | {time_limit}s")

    results = {}   # nombre -> {semilla: (objetivo, ruta_archivo)}
    for seed in seeds:
        out_i = 0 if hybrid else 1000 + seed   # hibrido: junto a las combinaciones de OR-Tools (solutions_evrp_0)
        for path in instances:
            print(f"\n=== {combo} | {os.path.basename(path)} | semilla {seed} ===")
            evrp_aco.execute(i=out_i, instance_type=path, time_limit=time_limit, seed=seed, **params)
            name = os.path.basename(path)
            sol = os.path.join(OUT_ROOT, f"solutions_evrp_{out_i}", f"solutions_{combo}", name)
            if os.path.isfile(sol):
                results.setdefault(name, {})[seed] = (objective_of(sol), sol)

    summary = {n: min(v[0] for v in by.values()) for n, by in results.items()}
    if hybrid:
        print(f"\n{combo} (semilla {seeds[0]}): " + ", ".join(f"{n[7:10]}={o}" for n, o in sorted(summary.items())))
        return summary
    best_dir = os.path.join(OUT_ROOT, "solutions_evrp_best", f"solutions_{combo}")
    os.makedirs(best_dir, exist_ok=True)
    print(f"\n{'instancia':<28}{'mejor ACO':>10}{'semilla':>9}   objetivo por semilla")
    for name in sorted(results):
        by_seed = results[name]
        best_seed = min(by_seed, key=lambda s: by_seed[s][0])
        shutil.copyfile(by_seed[best_seed][1], os.path.join(best_dir, name))
        print(f"{name:<28}{by_seed[best_seed][0]:>10}{best_seed:>9}   {({s: by_seed[s][0] for s in sorted(by_seed)})}")
    print(f"Mejores guardadas en {best_dir}")
    return summary


def parse_heuristics(value):
    """None -> [None] + todas; 'ACO' -> [None]; nombre/enum/lista -> [heuristicas]."""
    from p_mh.ACO.ortools_seed import as_heuristic
    from problem.strategy_type import HeuristicType
    if value is None:
        return [None] + list(HeuristicType)
    items = value if isinstance(value, (list, tuple)) else [value]
    return [None if (isinstance(v, str) and v.upper() == "ACO") else as_heuristic(v) for v in items]


def main():
    ap = argparse.ArgumentParser(description="ACO independiente para EVRP (todo se puede fijar arriba, en el archivo)")
    ap.add_argument("--instance", nargs="+", default=None, help="sobrescribe INSTANCES")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--seed", type=int, help="sobrescribe SEEDS con una sola semilla")
    group.add_argument("--seeds", nargs="+", help="sobrescribe SEEDS ('all' = 1-6)")
    ap.add_argument("--time", type=int, default=None, help="sobrescribe TIME_LIMIT")
    ap.add_argument("--heuristic", nargs="+", default=None,
                    help="sobrescribe HEURISTIC: 'ACO' (solo ACO), 'all' (todo) o nombre(s) de heuristica")
    args = ap.parse_args()

    if args.seed is not None:
        seeds = [args.seed]
    elif args.seeds:
        seeds = ALL_SEEDS if args.seeds == ["all"] else [int(x) for x in args.seeds]
    else:
        seeds = list(SEEDS)
    instances = resolve_instances(args.instance if args.instance else ([INSTANCES] if isinstance(INSTANCES, str) else INSTANCES))
    time_limit = args.time if args.time is not None else TIME_LIMIT
    if args.heuristic:
        wanted = None if args.heuristic == ["all"] else (args.heuristic if len(args.heuristic) > 1 else args.heuristic[0])
    else:
        wanted = HEURISTIC
    combos = parse_heuristics(wanted)
    print(f"Combinaciones a ejecutar ({len(combos)}): {[combo_name(h) for h in combos]}")

    table = {}
    for h in combos:
        try:
            table[combo_name(h)] = run_combo(h, instances, seeds, time_limit)
        except Exception as error:   # una combinacion que falla no detiene las demas
            print(f"!! {combo_name(h)} fallo: {type(error).__name__}: {error}")
    print("\n=== RESUMEN ===")
    for name, summary in table.items():
        print(f"{name:<52}" + "  ".join(f"{n[7:10]}={o}" for n, o in sorted(summary.items())))
    print(f"\nLos hibridos quedan en {os.path.join(OUT_ROOT, 'solutions_evrp_0')}; ACO solo en solutions_evrp_<1000+semilla>. Siguiente paso: python friedman_ranking.py")


if __name__ == "__main__":
    main()
