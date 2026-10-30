# -*- coding: utf-8 -*-
"""
Experimentos pendientes del capitulo 3 (epigrafes 3.4 y 3.5). Ejecutar desde la raiz del repo:

    python docs_thesis_assets/correr_experimentos_cap3.py sim-fav      # Tabla 12 (congestion, 3 algoritmos x 5 instancias)
    python docs_thesis_assets/correr_experimentos_cap3.py sim-desfav   # Tabla 13 (accidente/cierre/congestion x 3 replanificadores)
    python docs_thesis_assets/correr_experimentos_cap3.py tablas       # agrega resultados -> CSV, Figura 17 y Wilcoxon
    python docs_thesis_assets/correr_experimentos_cap3.py xai          # imprime/ejecuta los comandos de los 3 puntos de vista

Requisitos: SUMO (SUMO_HOME) y las redes de simulation/ ya construidas. Usa solo los scripts existentes del repo
(build_routes, build_battery_scenario, dynamic_replanning); no modifica codigo del repo.
Opciones: --dry-run (solo muestra comandos), --instances 30 40 50 (subconjunto), --force (rehacer escenarios),
          --replanners ortools (sim-fav / sim-desfav solo con OR-Tools; por defecto ortools y aco).
"""
import argparse, csv, glob, json, os, subprocess, sys, itertools

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "simulation"))
import run as simrun                                     # simulation/run.py: mapas instancia->red y rutas de escenario
from explainability import simulation as xsim            # parser de reportes

INST = {30: "quebec_30c_6ev_4cs", 40: "quebec_40c_4ev_6cs", 50: "quebec_50c_5ev_5cs",
        60: "quebec_60c_8ev_8cs", 90: "quebec_90c_10ev_10cs",
        # instancias grandes: no estan en el valor por defecto de --instances; pedirlas explicitamente
        70: "quebec_70c_9ev_9cs", 100: "quebec_100c_12ev_12cs", 150: "quebec_150c_18ev_15cs",
        220: "quebec_220c_26ev_20cs", 350: "quebec_350c_40ev_30cs"}
OSRM = os.path.join(REPO, "problem", "osrm")
# nombre mostrado -> (carpeta de solucion, heuristica, metaheuristica, replanner)
ALGOS = {
    "ACO": (os.path.join(OSRM, "solutions_evrp_1001", "solutions_ACO"), "", None, "aco"),
    "PATH_CHEAPEST_ARC + ACO": (os.path.join(OSRM, "solutions_evrp_0", "solutions_PATH_CHEAPEST_ARC_and_ACO"),
                                "PATH_CHEAPEST_ARC", None, "aco"),
    "SEQUENTIAL_CHEAPEST_INSERTION + GLS": (os.path.join(OSRM, "solutions_evrp_0",
                                "solutions_SEQUENTIAL_CHEAPEST_INSERTION_and_GUIDED_LOCAL_SEARCH"),
                                "SEQUENTIAL_CHEAPEST_INSERTION", "GUIDED_LOCAL_SEARCH", "ortools"),
}
FAILED = []
RES = os.path.join(REPO, "simulation", "dynamic_replan_results")
SPECS = os.path.join(RES, "specs_cap3")
MANIFEST = os.path.join(RES, "manifest_cap3.csv")
OUTDIR = os.path.join(REPO, "docs_thesis_assets", "resultados_cap3")


def sh(cmd, dry):
    print("\n$", " ".join(cmd))
    if dry:
        return 0
    return subprocess.run(cmd, cwd=REPO).returncode


def paths(n, algo):
    inst = os.path.join(REPO, "instances_data", "evrp_instances", INST[n] + ".txt")
    sol = os.path.join(ALGOS[algo][0], INST[n] + ".txt")
    return inst, sol


def ensure_scenario(n, algo, args):
    inst, sol = paths(n, algo)
    out = simrun.output_dir_for(inst, sol)
    cfg = os.path.join(out, "scenario_battery.sumocfg")
    if os.path.isfile(cfg) and not args.force:
        return True
    net = simrun.net_file_for(inst)
    for script in ("build_routes.py", "build_battery_scenario.py"):
        rc = sh([sys.executable, os.path.join("simulation", script), "--instance", inst, "--solution", sol,
                 "--net-file", net, "--output-dir", out], args.dry_run)
        if rc:
            print(f"\n  >> FALLO {script} para {INST[n]} / {algo}; se omite esta combinacion y se sigue con la siguiente.")
            FAILED.append((INST[n], algo, script))
            return False
    return True


def new_reports(before):
    return sorted(set(glob.glob(os.path.join(RES, "*_feedback_*.txt"))) - before)


def log(row):
    os.makedirs(RES, exist_ok=True)
    new = not os.path.isfile(MANIFEST)
    with open(MANIFEST, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["table", "n", "algo", "event", "replanner", "mode", "report"])
        if new:
            w.writeheader()
        w.writerow(row)


def run_pair(table, n, algo, event, replanner, args):
    """Una rama CON y otra SIN retroalimentacion, con el MISMO evento (event-spec compartido por instancia+evento)."""
    inst, sol = paths(n, algo)
    if not ensure_scenario(n, algo, args):
        return
    os.makedirs(SPECS, exist_ok=True)
    spec = os.path.join(SPECS, f"{INST[n]}_{event}.json")   # compartido por los 3 algoritmos: mismo evento
    _, heur, meta, own = ALGOS[algo]
    base = [sys.executable, os.path.join("simulation", "dynamic_replanning.py"), "--instance", inst, "--solution", sol,
            "--config", simrun.battery_cfg_for(inst, sol), "--net-file", simrun.net_file_for(inst),
            "--event", event, "--verified-event", "--event-spec", spec, "--event-seed", "1",
            "--replanner", replanner, "--heuristic", heur]
    if meta:
        base += ["--metaheuristic", meta]
    if getattr(args, "matriz_incremental", False):
        base += ["--matriz-incremental"]
    for mode, extra in (("WITH", []), ("NO", ["--no-replan"])):
        before = set(glob.glob(os.path.join(RES, "*_feedback_*.txt")))
        rc = sh(base + extra, args.dry_run)
        if rc:
            print("  ERROR, continuo con el siguiente")
            continue
        for rep in new_reports(before):
            log(dict(table=table, n=n, algo=algo, event=event, replanner=replanner, mode=mode,
                     report=os.path.basename(rep)))


def stage_sim_fav(args):
    for n, algo in itertools.product(args.instances, ALGOS):
        if ALGOS[algo][3] in args.replanners:
            run_pair("T12", n, algo, "congestion", ALGOS[algo][3], args)


def stage_sim_desfav(args):
    n = args.unfav_instance
    # Cada plan se replanifica con los MISMOS algoritmos que lo generaron (igual que en sim-fav):
    #   ACO -> ACO, PATH_CHEAPEST_ARC + ACO -> ACO (semilla PATH_CHEAPEST_ARC), SEQ+GLS -> OR-Tools (SEQ+GLS).
    # Se elimino la fila cruzada "PATH_CHEAPEST_ARC + ACO replanificado con OR-Tools" (2026-10-29): OR-Tools no tiene ACO
    # como metaheuristica y replanificaba solo con la heuristica, que a veces ni encuentra solucion inicial.
    for event in ("accident", "closure", "congestion"):
        for algo in ("ACO", "PATH_CHEAPEST_ARC + ACO", "SEQUENTIAL_CHEAPEST_INSERTION + GLS"):
            if ALGOS[algo][3] in args.replanners:
                run_pair("T13", n, algo, event, ALGOS[algo][3], args)


def stage_tablas(args):
    os.makedirs(OUTDIR, exist_ok=True)
    if not os.path.isfile(MANIFEST):
        raise SystemExit("Aun no hay resultados: primero ejecuta 'sim-fav' y/o 'sim-desfav' (el manifiesto se crea al terminar cada corrida).")
    rows = list(csv.DictReader(open(MANIFEST, encoding="utf-8")))
    data = []
    for r in rows:
        info = xsim.parse_result_file(os.path.join(RES, r["report"]))
        if info:
            data.append({**r, "dist": info["total_distance"], "retraso_s": info["extra_time_s"],
                         "sin_terminar": info["n_unfinished"]})
    out = os.path.join(OUTDIR, "resultados_sim.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(data[0].keys())); w.writeheader(); w.writerows(data)
    print("->", out)
    # pares CON/SIN (ultimo reporte de cada celda)
    cell = {}
    for d in data:
        cell[(d["table"], d["n"], d["algo"], d["event"], d["replanner"], d["mode"])] = d
    pairs = []
    for (t, n, a, e, rp, m), d in cell.items():
        if m == "WITH" and (t, n, a, e, rp, "NO") in cell:
            s = cell[(t, n, a, e, rp, "NO")]
            pairs.append(dict(table=t, n=n, algo=a, event=e, replanner=rp,
                              dist_sin=s["dist"], dist_con=d["dist"], red_dist=100 * (1 - d["dist"] / s["dist"]),
                              ret_sin=s["retraso_s"], ret_con=d["retraso_s"],
                              red_ret=100 * (1 - d["retraso_s"] / s["retraso_s"]) if s["retraso_s"] else 0.0,
                              sin_terminar_con=d["sin_terminar"]))
    out = os.path.join(OUTDIR, "pares_sin_con.csv")
    if pairs:
        with open(out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(pairs[0].keys())); w.writeheader(); w.writerows(pairs)
        print("->", out)
    t12 = [p for p in pairs if p["table"] == "T12"]
    if t12:
        try:
            from scipy.stats import wilcoxon
            a_ = [p["dist_sin"] for p in t12]; b_ = [p["dist_con"] for p in t12]
            print("Wilcoxon (distancia sin vs con, Tabla 12): n=%d p=%.6g" % (len(t12), wilcoxon(a_, b_).pvalue))
        except Exception as ex:
            print("Wilcoxon no disponible:", ex)
        try:
            import matplotlib; matplotlib.use("Agg")
            import matplotlib.pyplot as plt, numpy as np
            algos = list(ALGOS); ns = sorted({p["n"] for p in t12})
            fig, ax = plt.subplots(figsize=(9, 4.6)); w = 0.13; x = np.arange(len(ns))
            for k, a in enumerate(algos):
                for j, (key, lab, hatch) in enumerate((("dist_sin", "sin", "//"), ("dist_con", "con", ""))):
                    v = [next((p[key] for p in t12 if p["n"] == n_ and p["algo"] == a), np.nan) for n_ in ns]
                    ax.bar(x + (2 * k + j - 2.5) * w, v, w, label=f"{a} ({lab})", hatch=hatch, color=f"C{k}",
                           alpha=0.55 if lab == "sin" else 1.0)
            ax.set_xticks(x); ax.set_xticklabels([f"{n_} clientes" for n_ in ns]); ax.set_ylabel("Distancia total (km)")
            ax.legend(fontsize=7, ncol=3, loc="upper left"); fig.tight_layout()
            out = os.path.join(OUTDIR, "fig_barras_sin_con.png"); fig.savefig(out, dpi=200); print("->", out)
        except Exception as ex:
            print("Figura no generada:", ex)
    for p in pairs:
        print("%s %3s %-38s %-10s %-8s dist %.2f->%.2f (%.1f %%) retraso %.0f->%.0f s (%.1f %%)" % (
            p["table"], p["n"], p["algo"], p["event"], p["replanner"], p["dist_sin"], p["dist_con"], p["red_dist"],
            p["ret_sin"], p["ret_con"], p["red_ret"]))


def stage_xai(args):
    py = sys.executable
    steps = [
        ("Punto de vista 1 - optimizacion: regenerar la tabla con el modelo final y las 10 instancias",
         [py, "run_xai.py", "generate", "--algorithm", "ACO", "--rows", "600", "--time", "3"]),
        ("Punto de vista 1: explicar (SHAP)", [py, "run_xai.py", "explain", "--data", "explainability/data/aco.csv",
                                               "--target", "distance", "--title", "OPTIMIZACION",
                                               "--out", "explainability/out/optimizacion"]),
        ("Puntos 2 y 3: construir tablas de simulacion y combinacion desde los reportes", [py, "run_xai.py", "sim-table"]),
        ("Punto de vista 2 - simulacion", [py, "run_xai.py", "explain", "--data", "explainability/data/sim.csv",
                                           "--title", "SIMULACION", "--out", "explainability/out/simulacion"]),
        ("Punto de vista 3 - combinacion", [py, "run_xai.py", "explain", "--data", "explainability/data/combo.csv",
                                            "--title", "COMBINACION", "--out", "explainability/out/combinacion"]),
        ("Punto de vista 3 - combinacion, objetivo = ganancia por retroalimentacion (%)",
         [py, "run_xai.py", "explain", "--data", "explainability/data/combo.csv", "--target", "gain_distance_pct",
          "--title", "COMBINACION_GANANCIA", "--out", "explainability/out/combinacion_ganancia"]),
        ("Relato de una replanificacion (cambiar --run por el nombre de un reporte WITH)",
         [py, "run_xai.py", "replan-report", "--run", "quebec_50c_5ev_5cs_WITH_feedback_20261026_172758"]),
    ]
    for title, cmd in steps:
        print("\n#", title)
        sh(cmd, args.dry_run or "--run" in cmd)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=["sim-fav", "sim-desfav", "tablas", "xai"])
    ap.add_argument("--instances", type=int, nargs="+", default=[30, 40, 50, 60, 90], choices=sorted(INST))
    ap.add_argument("--unfav-instance", type=int, default=50)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--matriz-incremental", action="store_true",
                    help="sim-fav / sim-desfav: matriz en vivo incremental con cache (ver simulation/live_matrix.py)")
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--replanners", nargs="+", choices=["ortools", "aco"], default=["ortools", "aco"],
                    help="sim-fav / sim-desfav: replanificadores a ejecutar (por defecto ambos)")
    a = ap.parse_args()
    {"sim-fav": stage_sim_fav, "sim-desfav": stage_sim_desfav, "tablas": stage_tablas, "xai": stage_xai}[a.stage](a)
    if FAILED:
        print("\nCombinaciones que fallaron al construir el escenario:")
        for f in FAILED:
            print("  ", f)
