"""
Registro de algoritmos explicables.

Cada algoritmo se registra con:
  - param_space : lista de dimensiones {name, low, high, kind} que se muestrean para generar datos
                  (kind: 'float' | 'int' | 'bool').
  - runner      : funcion (instance_path, scenario, params, seed, time_limit) -> dict de resultado.
  - fixed       : parametros que no se varian (valores por defecto del algoritmo).

Para sumar otra metaheuristica poblacional basta con otro register(...) con su propio
espacio de parametros: dataset, sustituto y explicadores no cambian. Si se mezclan varios
algoritmos en una misma tabla, la columna `algorithm` permite explicar "por que uno se
comporto mejor que otro" (el explicador recibe esa columna como una variable mas).
"""
import os

ALGORITHMS = {}


def register(name, param_space, runner, fixed=None):
    ALGORITHMS[name] = {"param_space": param_space, "runner": runner, "fixed": fixed or {}}


# ───────────────────────── ACO ─────────────────────────

ACO_PARAM_SPACE = [
    {"name": "alpha", "low": 0.0, "high": 3.0, "kind": "float"},       # peso de la feromona
    {"name": "beta", "low": 0.5, "high": 4.0, "kind": "float"},        # peso de la heuristica 1/distancia
    {"name": "gamma", "low": 0.5, "high": 4.0, "kind": "float"},       # peso del termino de energia
    {"name": "rho", "low": 0.1, "high": 0.9, "kind": "float"},         # evaporacion
    {"name": "num_ants", "low": 10, "high": 40, "kind": "int"},
    {"name": "local_search_top", "low": 1, "high": 10, "kind": "int"},
    {"name": "ils_per_iter", "low": 0, "high": 4, "kind": "int"},
    {"name": "candidate_list_size", "low": 8, "high": 30, "kind": "int"},
    {"name": "proactive_charge_threshold", "low": 0.05, "high": 0.4, "kind": "float"},
    {"name": "mmas", "low": 0, "high": 1, "kind": "bool"},             # 1 = MAX-MIN, 0 = Rank-AS
]


def run_aco(instance_path, scenario, params, seed, time_limit):
    from p_mh.ACO import evrp_aco
    from distance.distance_type import DistanceType

    kwargs = dict(params)
    kwargs["pheromone_scheme"] = "mmas" if kwargs.pop("mmas") else "rank_as"
    result = evrp_aco.execute(
        i=0, instance_type=instance_path, time_limit=time_limit,
        vehicle_maximum_travel_distance=scenario["battery"], vehicle_speed=scenario["consumption"],
        distance_type=DistanceType.OSRM, seed=seed, save_files=False, verbose=False, **kwargs,
    )
    return result[os.path.basename(instance_path)]


register("ACO", ACO_PARAM_SPACE, run_aco)

# Para una nueva metaheuristica poblacional:
#   register("GA", GA_PARAM_SPACE, run_ga)
# donde run_ga devuelve un dict con al menos {'distance', 'n_dropped', 'iterations', 'elapsed', 'recharges'}.
