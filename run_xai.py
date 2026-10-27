"""
run_xai.py — lanzador del paquete explainability (version inicial: SHAP sobre un modelo sustituto del ACO).

Paso 1: generar los datos (corre el algoritmo muchas veces; reanudable):
    python run_xai.py generate --algorithm ACO --rows 300 --time 3 --out explainability/data/aco.csv
    # repartido en dos terminales:  --part 0/2   y   --part 1/2   (con --out distinto cada una)

Paso 2: explicar:
    python run_xai.py explain --data explainability/data/aco.csv --target distance --out explainability/out
    python run_xai.py explain --data explainability/data/aco.csv --row 17        # ademas explica la corrida #17

Simulacion y combinacion (puntos de vista 2 y 3): tablas a partir de los reportes de simulation/dynamic_replan_results
    python run_xai.py sim-table                      # escribe explainability/data/sim.csv y combo.csv
    python run_xai.py explain --data explainability/data/sim.csv   --target total_distance --title SIMULACION   --out explainability/out_sim
    python run_xai.py explain --data explainability/data/combo.csv --target gain_distance_pct --title COMBINACION --out explainability/out_combo
Retroalimentacion (por que mejora o empeora cada vehiculo al re-planificar):
    python run_xai.py replan-report --run quebec_50c_5ev_5cs_WITH_feedback_20261006_215236   # relato de un evento
    python run_xai.py explain --data explainability/data/veh.csv --target gain_time_vehicle_s --title RETROALIMENTACION --out explainability/out_veh
y, si se quieren mas filas con eventos distintos (requiere SUMO y OSRM; ~minutos por fila):
    python run_xai.py generate --algorithm SUMO_REPLAN --rows 40 --out explainability/data/sim_gen.csv

Requisitos para explain: pip install scikit-learn shap pandas matplotlib
Se ejecuta desde la raiz del repo, con el servidor OSRM (o las matrices en instances_data/osrm_cache).
"""
import argparse
import os


def cmd_generate(args):
    from explainability import dataset
    k, n = (int(x) for x in args.part.split("/"))
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    dataset.generate(args.algorithm, args.out, n_rows=args.rows, time_limit=args.time,
                     sampling_seed=args.sampling_seed, part=(k, n))
    print(f"Datos en {args.out}")


def cmd_explain(args):
    from explainability import report
    from explainability.explainers import EXPLAINERS
    from explainability.surrogate import load_dataset, train_surrogate

    X, y, groups, df = load_dataset(args.data, target=args.target, algorithm=args.algorithm)
    print(f"Datos: {len(X)} corridas validas, {X.shape[1]} variables explicativas")
    model, metrics = train_surrogate(X, y, groups)
    explainer = EXPLAINERS[args.method]()
    explanation = explainer.explain(model, X)

    print("\n" + report.text_report(explanation, model, y, metrics, args.target, algorithm=args.title or args.algorithm or "ACO"))
    report.save_outputs(explanation, args.out)
    if args.row is not None:
        print("\n" + report.local_text(explanation, model, y, args.row))
        report.save_local_plot(explanation, args.row, args.out)
    print(f"\nTablas y graficos en {args.out}/")


def cmd_sim_table(args):
    from explainability import simulation
    from explainability import feedback
    sim, combo = simulation.build_tables(args.results)
    veh = feedback.build_vehicle_table(args.results)
    veh.to_csv(args.out_veh, index=False)
    print(f"Por vehiculo: {len(veh)} filas (evento x vehiculo) -> {args.out_veh}")
    os.makedirs(os.path.dirname(args.out_sim) or ".", exist_ok=True)
    sim.to_csv(args.out_sim, index=False)
    combo.to_csv(args.out_combo, index=False)
    print(f"Simulacion : {len(sim)} corridas ({sim['instance'].nunique()} instancias) -> {args.out_sim}")
    print(f"Combinacion: {len(combo)} pares sin/con retroalimentacion -> {args.out_combo}")
    noisy = int((combo["baseline_spread_pct"] > 1.0).sum()) if len(combo) else 0
    if noisy:
        print(f"Aviso: {noisy} par(es) tienen reportes SIN retroalimentacion del mismo evento que difieren >1 % entre si "
              f"(columna baseline_spread_pct): probablemente de versiones distintas del modelo. Descartalos o repite esas corridas.")
    if len(combo) < 20:
        print("Aviso: pocas filas; el sustituto aprendera casi solo la instancia. Genera eventos distintos con "
              "`generate --algorithm SUMO_REPLAN` antes de interpretar.")


def cmd_replan_report(args):
    from explainability import feedback, simulation
    reports = list(feedback.iter_reports(args.results))
    withs = [(n, t) for n, t in reports if "_WITH_" in n and args.run in n]
    if not withs:
        raise SystemExit(f"No hay un reporte CON retroalimentacion que contenga '{args.run}'")
    name, txt = withs[-1]
    head = simulation.parse_text(name, txt)
    path = os.path.join(simulation.INSTANCES_DIR, head["instance"] + ".txt")
    n_clients, stations = feedback._instance_shape(path)
    pairs = simulation.manifest_pairs(args.results)
    texts = dict(reports)
    if pairs is not None and pairs.get(name) in texts:
        # pareja exacta del manifiesto: mismo plan inicial, mismo evento y mismo replanificador
        no = [texts[pairs[name]]]
        print(f"[{name}] linea base sin retroalimentacion: {pairs[name]} (pareja exacta del manifiesto)\n")
    else:
        no = [t for n, t in reports if "_NO_" in n and (lambda r: r and r["event_key"] == head["event_key"])(simulation.parse_text(n, t))]
        print(f"[{name}] baselines sin retroalimentacion del mismo evento: {len(no)}"
              f"{' (sin manifiesto: pueden ser de planes distintos)' if len(no) > 1 else ''}\n")
    print(feedback.explain_replan(name, txt, no, n_clients, stations))


def main():
    ap = argparse.ArgumentParser(description="Modulo de explicabilidad (XAI)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    g = sub.add_parser("generate", help="generar la tabla de corridas")
    g.add_argument("--algorithm", default="ACO")
    g.add_argument("--rows", type=int, default=300, help="numero de corridas (filas)")
    g.add_argument("--time", type=int, default=3, help="segundos por corrida (por defecto 3: es para explicar, no para optimizar)")
    g.add_argument("--out", default=os.path.join("explainability", "data", "aco.csv"))
    g.add_argument("--part", default="0/1", help="k/n para repartir las filas entre n procesos")
    g.add_argument("--sampling-seed", type=int, default=0)
    g.set_defaults(func=cmd_generate)

    t = sub.add_parser("sim-table", help="construir las tablas de simulacion y combinacion desde los reportes de SUMO")
    t.add_argument("--results", default=os.path.join("simulation", "dynamic_replan_results"))
    t.add_argument("--out-sim", default=os.path.join("explainability", "data", "sim.csv"))
    t.add_argument("--out-combo", default=os.path.join("explainability", "data", "combo.csv"))
    t.add_argument("--out-veh", default=os.path.join("explainability", "data", "veh.csv"))
    t.set_defaults(func=cmd_sim_table)

    r = sub.add_parser("replan-report", help="relato de una re-planificacion: que cambio en cada vehiculo y cuanto costo/ahorro")
    r.add_argument("--run", required=True, help="parte del nombre de un reporte CON retroalimentacion, p. ej. quebec_50c_5ev_5cs_WITH_feedback_20261006_215236")
    r.add_argument("--results", default=os.path.join("simulation", "dynamic_replan_results"))
    r.set_defaults(func=cmd_replan_report)

    e = sub.add_parser("explain", help="entrenar el sustituto y explicarlo")
    e.add_argument("--data", default=os.path.join("explainability", "data", "aco.csv"))
    e.add_argument("--target", default="distance",
                   choices=["distance", "distance_per_client", "iterations", "recharges",
                            "total_distance", "extra_time_s", "gain_distance_pct", "gain_time_pct",
                            "gain_time_vehicle_s", "gain_distance_vehicle_km"])
    e.add_argument("--title", default=None, help="nombre del punto de vista en el informe (p. ej. SIMULACION, COMBINACION)")
    e.add_argument("--algorithm", default=None, help="filtrar por algoritmo si la tabla tiene varios")
    e.add_argument("--method", default="shap", choices=["shap"])
    e.add_argument("--row", type=int, default=None, help="indice de una corrida a explicar de forma local")
    e.add_argument("--out", default=os.path.join("explainability", "out"))
    e.set_defaults(func=cmd_explain)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
