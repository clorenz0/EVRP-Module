"""
Patron Strategy en ACO.

evrp_aco.execute() es el esqueleto del algoritmo (Template Method): construir con las hormigas,
busqueda local, perturbacion, actualizar la feromona. Los dos puntos donde hay variantes reales
se delegan a estrategias intercambiables:

  - PheromoneStrategy        como se actualiza la feromona en cada iteracion
        MMASStrategy         MAX-MIN Ant System (por defecto)
        RankASStrategy       Rank-AS (Nie et al.)
  - InitialSolutionStrategy  de donde sale la solucion inicial que compite como una hormiga mas
        NoInitial            ninguna: ACO solo (por defecto)
        ORToolsHeuristic     solucion de una heuristica constructiva de OR-Tools (opcional)

Son adaptadores delgados: cada una llama a las mismas funciones de evrp_aco que antes se invocaban
directamente, en el mismo orden y con los mismos argumentos. No cambian ningun calculo ni el orden
de uso del generador aleatorio. Se puede comprobar con verificar_equivalencia_aco.py.

Para agregar una variante nueva (p. ej. otro esquema de feromona) basta una clase que herede de la
estrategia, y una entrada en el registro correspondiente.
"""
from abc import ABC, abstractmethod


def _core():
    # import diferido: evrp_aco importa este modulo
    from p_mh.ACO import evrp_aco
    return evrp_aco


# ───────────────────────── Feromona ─────────────────────────

class PheromoneStrategy(ABC):
    """Regla de actualizacion de la feromona al final de cada iteracion."""
    name = "base"

    @abstractmethod
    def update(self, pheromone, ants_results, iter_best, cycle_best, cycle_iter):
        """
        Actualiza la feromona y devuelve la matriz a usar en la proxima iteracion (puede ser una nueva).
        ants_results : lista de (rutas, costo) de las hormigas de la iteracion.
        iter_best / cycle_best : (rutas, dropped, costo) mejor de la iteracion / del ciclo.
        cycle_iter   : iteracion dentro del ciclo actual (0 = primera tras un reinicio).
        """


class MMASStrategy(PheromoneStrategy):
    """MAX-MIN Ant System: deposita solo una hormiga y acota la feromona a [tau_min, tau_max]."""
    name = "mmas"

    def __init__(self, rho, num_locations, gb_period):
        self.rho, self.num_locations, self.gb_period = rho, num_locations, gb_period

    def update(self, pheromone, ants_results, iter_best, cycle_best, cycle_iter):
        core = _core()
        if cycle_iter == 0:
            # MMAS: cada ciclo arranca en tau_max (exploracion maxima)
            _, tau_max = core._mmas_bounds(cycle_best[2], self.rho, self.num_locations)
            pheromone = core._init_pheromone(self.num_locations, tau_max)
        core._update_pheromone_mmas(
            pheromone, (iter_best[0], iter_best[2]), (cycle_best[0], cycle_best[2]),
            self.rho, cycle_iter, self.gb_period
        )
        return pheromone


class RankASStrategy(PheromoneStrategy):
    """Rank-AS: depositan las (w-1) mejores hormigas de la iteracion y la mejor global."""
    name = "rank_as"

    def __init__(self, rho, w):
        self.rho, self.w = rho, w

    def update(self, pheromone, ants_results, iter_best, cycle_best, cycle_iter):
        _core()._update_pheromone_rank_as(pheromone, ants_results, self.rho, cycle_best[0], cycle_best[2], self.w)
        return pheromone


PHEROMONE_STRATEGIES = {"mmas": MMASStrategy, "rank_as": RankASStrategy}


def make_pheromone_strategy(name, rho, num_locations, gb_period, w):
    if name == "mmas":
        return MMASStrategy(rho, num_locations, gb_period)
    if name == "rank_as":
        return RankASStrategy(rho, w)
    raise ValueError(f"pheromone_scheme desconocido '{name}'. Disponibles: {', '.join(PHEROMONE_STRATEGIES)}")


# ───────────────────────── Solucion inicial ─────────────────────────

class InitialSolutionStrategy(ABC):
    """Aporta (o no) una solucion inicial que compite como una hormiga mas en la primera iteracion."""
    name = "base"

    @abstractmethod
    def build(self, data, verbose=True):
        """Devuelve (rutas, dropped, costo) o None."""


class NoInitial(InitialSolutionStrategy):
    """ACO independiente: no hay solucion inicial."""
    name = "none"

    def build(self, data, verbose=True):
        return None


class ORToolsHeuristic(InitialSolutionStrategy):
    """Solucion construida por una heuristica de OR-Tools (PATH_CHEAPEST_ARC, SAVINGS, ...)."""

    def __init__(self, heuristic, time_limit=2):
        self.heuristic, self.time_limit = heuristic, time_limit
        self.name = getattr(heuristic, "name", str(heuristic))

    def build(self, data, verbose=True):
        core = _core()
        from p_mh.ACO.ortools_seed import build_initial_routes
        node_routes = build_initial_routes(data, self.heuristic, self.time_limit)
        rebuilt = [core._rebuild_route_state(r, data) for r in node_routes] if node_routes else None
        if rebuilt and all(r is not None for r in rebuilt):
            if verbose:
                print(f"  Solucion inicial de OR-Tools ({self.name}): {core._objective(data, rebuilt, [])}")
            return rebuilt, [], core._solution_cost(data, rebuilt, [])
        if verbose:
            print(f"  OR-Tools ({self.name}) no dio solucion factible completa: ACO arranca sin solucion inicial")
        return None


def make_initial_strategy(initial_heuristic, time_limit=2):
    """None -> NoInitial; una heuristica (HeuristicType) -> ORToolsHeuristic."""
    if initial_heuristic is None:
        return NoInitial()
    return ORToolsHeuristic(initial_heuristic, time_limit)
