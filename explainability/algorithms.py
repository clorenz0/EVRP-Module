"""
Registro de algoritmos explicables.

Patron Strategy: cada algoritmo es una clase que hereda de AlgorithmStrategy (nombre, espacio de
parametros y metodo run) y se registra con register(...).

Para sumar otra metaheuristica poblacional basta con otra estrategia con su propio
espacio de parametros: dataset, sustituto y explicadores no cambian. Si se mezclan varios
algoritmos en una misma tabla, la columna `algorithm` permite explicar "por que uno se
comporto mejor que otro" (el explicador recibe esa columna como una variable mas).
"""
import os
from abc import ABC, abstractmethod

ALGORITHMS = {}


class AlgorithmStrategy(ABC):
    """
    Strategy: un algoritmo explicable. Declara su espacio de parametros y como ejecutarse; el resto
    del modulo (dataset, sustituto, explicadores) solo conoce esta interfaz.

      name        : nombre con el que se registra y aparece en la columna `algorithm`.
      param_space : lista de dimensiones {name, low, high, kind} que se muestrean para generar datos
                    (kind: 'float' | 'int' | 'bool').
      fixed       : parametros que no se varian (valores por defecto del algoritmo).
      run(...)    : ejecuta y devuelve un dict con al menos
                    {'distance', 'objective', 'n_dropped', 'n_routes', 'recharges', 'iterations', 'elapsed'}.
    """
    name = "base"
    param_space = []
    fixed = {}
    scenario_space = None   # None = SCENARIO_SPACE de dataset.py (bateria y consumo); [] = sin escenario variable
    result_keys = ("distance", "objective", "n_dropped", "n_routes", "recharges", "iterations", "elapsed")

    @abstractmethod
    def run(self, instance_path, scenario, params, seed, time_limit):
        """Ejecuta el algoritmo sobre una instancia y devuelve el dict de resultado."""


def register(strategy):
    """Registra una estrategia (instancia de AlgorithmStrategy) por su nombre."""
    ALGORITHMS[strategy.name] = strategy
    return strategy


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


class ACOStrategy(AlgorithmStrategy):
    name = "ACO"
    param_space = ACO_PARAM_SPACE

    def run(self, instance_path, scenario, params, seed, time_limit):
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


register(ACOStrategy())

# Simulacion: SUMO_REPLAN (explicabilidad del simulador y de la combinacion con la optimizacion)
from explainability import simulation  # noqa: E402,F401  (se registra al importar)

# Para una nueva metaheuristica poblacional:
#   class GAStrategy(AlgorithmStrategy):
#       name = "GA"; param_space = GA_PARAM_SPACE
#       def run(self, instance_path, scenario, params, seed, time_limit): ...
#   register(GAStrategy())
