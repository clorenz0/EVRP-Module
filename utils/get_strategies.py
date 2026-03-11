def get_strategies(heuristic, metaheuristic):
    if heuristic and not metaheuristic:
        return [heuristic.value], None
    if heuristic and metaheuristic:
        return [heuristic.value], [metaheuristic.value]
    if metaheuristic and not heuristic:
        return None, [metaheuristic.value]
    if not heuristic and not metaheuristic:
        first_solution_strategies = [
            "PATH_CHEAPEST_ARC",
            "PATH_MOST_CONSTRAINED_ARC",
            "EVALUATOR_STRATEGY",
            "SAVINGS",
            "SWEEP",
            "CHRISTOFIDES",
            "ALL_UNPERFORMED",
            "BEST_INSERTION",
            "PARALLEL_CHEAPEST_INSERTION",
            "SEQUENTIAL_CHEAPEST_INSERTION",
            "LOCAL_CHEAPEST_INSERTION",
            "LOCAL_CHEAPEST_COST_INSERTION",
            "GLOBAL_CHEAPEST_ARC",
            "LOCAL_CHEAPEST_ARC",
            "FIRST_UNBOUND_MIN_VALUE",
        ]
        local_search_metaheuristics = [
            "GUIDED_LOCAL_SEARCH",
            "SIMULATED_ANNEALING",
            "TABU_SEARCH",
            "GENERIC_TABU_SEARCH",
        ]
        return first_solution_strategies, local_search_metaheuristics
