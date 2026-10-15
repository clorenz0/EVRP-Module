"""
run_xai.py — lanzador del paquete explainability (version inicial: SHAP sobre un modelo sustituto del ACO).

Paso 1: generar los datos (corre el algoritmo muchas veces; reanudable):
    python run_xai.py generate --algorithm ACO --rows 300 --time 3 --out explainability/data/aco.csv
    # repartido en dos terminales:  --part 0/2   y   --part 1/2   (con --out distinto cada una)

Paso 2: explicar:
    python run_xai.py explain --data explainability/data/aco.csv --target distance --out explainability/out
    python run_xai.py explain --data explainability/data/aco.csv --row 17        # ademas explica la corrida #17

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

    print("\n" + report.text_report(explanation, model, y, metrics, args.target, algorithm=args.algorithm or "ACO"))
    report.save_outputs(explanation, args.out)
    if args.row is not None:
        print("\n" + report.local_text(explanation, model, y, args.row))
        report.save_local_plot(explanation, args.row, args.out)
    print(f"\nTablas y graficos en {args.out}/")


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

    e = sub.add_parser("explain", help="entrenar el sustituto y explicarlo")
    e.add_argument("--data", default=os.path.join("explainability", "data", "aco.csv"))
    e.add_argument("--target", default="distance", choices=["distance", "distance_per_client", "iterations", "recharges"])
    e.add_argument("--algorithm", default=None, help="filtrar por algoritmo si la tabla tiene varios")
    e.add_argument("--method", default="shap", choices=["shap"])
    e.add_argument("--row", type=int, default=None, help="indice de una corrida a explicar de forma local")
    e.add_argument("--out", default=os.path.join("explainability", "out"))
    e.set_defaults(func=cmd_explain)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
