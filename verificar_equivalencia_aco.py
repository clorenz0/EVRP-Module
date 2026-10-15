"""
verificar_equivalencia_aco.py — comprueba que una refactorizacion NO cambia los resultados de ACO.

Corre ACO con un reloj simulado (cada consulta de tiempo avanza una cantidad fija), de modo que el
numero de iteraciones es fijo y la ejecucion es determinista: misma semilla => mismas rutas exactas.

    python verificar_equivalencia_aco.py --save    # guarda la referencia (hacerlo ANTES de refactorizar)
    python verificar_equivalencia_aco.py --check   # compara con la referencia (igualdad exacta de rutas)

La referencia se guarda en verificar_equivalencia_aco.json. Casos: 2 instancias x 2 semillas x
{MAX-MIN, Rank-AS}. Si OR-Tools o routingpy no estan instalados, se usan sustitutos vacios (ACO solo lo necesita
para el modo opcional con heuristica, que este chequeo no ejecuta).
"""
import argparse
import json
import os
import sys
import types

REF = "verificar_equivalencia_aco.json"
CASES = [(inst, seed, scheme) for inst in ("quebec_30c_6ev_4cs", "quebec_50c_5ev_5cs")
         for seed in (1, 2) for scheme in ("mmas", "rank_as")]
TICKS = 60          # consultas de reloj antes de parar (fija el numero de iteraciones)


def _stub_missing_libs():
    """Sustitutos vacios de librerias ausentes (OSRM se lee de instances_data/osrm_cache)."""
    try:
        import routingpy  # noqa: F401
    except ImportError:
        from unittest import mock
        rp, routers = types.ModuleType("routingpy"), types.ModuleType("routingpy.routers")
        routers.OSRM = mock.MagicMock()
        rp.routers = routers
        sys.modules.update({"routingpy": rp, "routingpy.routers": routers})
    try:
        import ortools  # noqa: F401
    except ImportError:
        from unittest import mock
        root = types.ModuleType("ortools")
        cs = types.ModuleType("ortools.constraint_solver")
        cs.routing_enums_pb2, cs.pywrapcp = mock.MagicMock(), mock.MagicMock()
        root.constraint_solver = cs
        sys.modules.update({"ortools": root, "ortools.constraint_solver": cs,
                            "ortools.constraint_solver.routing_enums_pb2": cs.routing_enums_pb2,
                            "ortools.constraint_solver.pywrapcp": cs.pywrapcp})


def run_case(inst, seed, scheme):
    from p_mh.ACO import evrp_aco
    from distance.distance_type import DistanceType

    captured = {}

    class FakeClock:
        def __init__(self): self.t = 0.0
        def time(self):
            self.t += 1.0
            return self.t

    def fake_save(data, routes, dropped, objective, instance, elapsed, i, distance_type, iterations, *a, **k):
        captured["routes"] = [[(s["node"], s["load"], s["battery"]) for s in r] for r in routes]
        captured["dropped"] = sorted(dropped)
        captured["objective"], captured["iterations"] = objective, iterations

    real_time, real_save = evrp_aco.time, evrp_aco.save_solution
    evrp_aco.time, evrp_aco.save_solution = FakeClock(), fake_save
    try:
        evrp_aco.execute(
            i=0, instance_type=os.path.join("instances_data", "evrp_instances", inst + ".txt"), time_limit=TICKS,
            vehicle_maximum_travel_distance=100, vehicle_speed=3.0, distance_type=DistanceType.OSRM,
            seed=seed, pheromone_scheme=scheme, verbose=False, save_files=True,
        )
    finally:
        evrp_aco.time, evrp_aco.save_solution = real_time, real_save
    return captured


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--save", action="store_true")
    g.add_argument("--check", action="store_true")
    args = ap.parse_args()
    _stub_missing_libs()

    results = {f"{i}|{s}|{m}": run_case(i, s, m) for i, s, m in CASES}
    for k, v in results.items():
        print(f"  {k}: objetivo {v['objective']}, {v['iterations']} iteraciones")
    if args.save:
        json.dump(results, open(REF, "w"), indent=1)
        print(f"Referencia guardada en {REF}")
        return
    ref = json.load(open(REF))
    bad = [k for k in results if json.loads(json.dumps(results[k])) != ref[k]]
    if bad:
        print(f"\nDIFERENCIAS en {len(bad)} caso(s): {bad}")
        raise SystemExit(1)
    print(f"\nOK: los {len(results)} casos son identicos a la referencia (rutas, cargas, bateria, objetivo, iteraciones).")


if __name__ == "__main__":
    main()
