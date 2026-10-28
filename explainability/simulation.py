"""
Explicabilidad de la SIMULACION y de la COMBINACION optimizacion + simulacion.

Reutiliza los mismos pasos que la optimizacion (dataset -> sustituto -> explicador): solo cambia de donde
salen las filas y cual es la variable de respuesta.

  Punto de vista 2 — SIMULACION  (tabla `sim`)
      Fila = una corrida de simulation/dynamic_replanning.py (con o sin retroalimentacion).
      Variables: instancia (f_), escenario (s_) y evento (e_: instante, tramos afectados, velocidad impuesta,
                 tipo de evento e_is_accident / e_is_closure (ambos 0 = congestion), duracion del bloqueo e_duration_s,
                 lentitud medida en SUMO, e_replan = 1 si se re-planifica).
      Respuesta: distancia total y tiempo extra reales medidos en SUMO.
      Pregunta: que hace que un evento de trafico cueste mas, y cuanto cambia re-planificar?

  Punto de vista 3 — COMBINACION  (tabla `combo`)
      Fila = un par (sin retroalimentacion, con retroalimentacion) ante el MISMO evento.
      Respuesta: ganancia de re-planificar, gain_distance_pct y gain_time_pct (en %).
      Variables: instancia, evento y algoritmo que genero el plan (columna `algorithm`, se vuelve a_<nombre>).
      Pregunta: de que depende que la retroalimentacion de SUMO mejore la solucion del optimizador:
                del evento, de la instancia o del algoritmo?

Las filas salen de dos fuentes:
  1. los reportes que ya deja dynamic_replanning.py en simulation/dynamic_replan_results (build_tables), o
  2. corridas nuevas lanzadas por SimulationStrategy (barre instante y severidad del evento), que se
     registra como algoritmo "SUMO_REPLAN" y se usa con `python run_xai.py generate --algorithm SUMO_REPLAN`.

Despues se explica igual que la optimizacion: `python run_xai.py explain --data <tabla>.csv --target <respuesta>`.
"""
import glob
import importlib.util
import os
import re
import subprocess
import sys
import time

from explainability.algorithms import AlgorithmStrategy, register

RESULTS_DIR = os.path.join("simulation", "dynamic_replan_results")
INSTANCES_DIR = os.path.join("instances_data", "evrp_instances")

NAME_RE = re.compile(r"(?P<inst>quebec_\d+c_\d+ev_\d+cs)_(?P<mode>NO|WITH)_feedback_(?P<stamp>\d{8}_\d{6})\.txt$")
EVENT_RE = re.compile(
    r"Event: congestion forced at checkpoint (?P<ck>[\d.]+)s on (?P<edges>\d+) real edge\(s\) "
    r"\(speed limit lowered to (?P<speed>[\d.]+) m/s\)\. Probe \w+ -> node \d+: "
    r"travel time (?P<before>[\d.]+)s -> (?P<after>[\d.]+)s")
# Eventos verificados (simulation/event_selector.py: congestion, accident, closure). Es la linea que escribe EventSpec.describe().
EVENT_V2_RE = re.compile(
    r"Event: (?P<kind>congestion|accident|closure) at checkpoint (?P<ck>[\d.]+)s on (?P<edges>\d+) real edge\(s\)"
    r"(?: \((?P<extra>[^)]*)\))?\. Target \S+ -> node \d+: "
    r"travel time (?P<before>[\d.]+)s -> (?P<after>[\d.]+)s")
SPEED_RE = re.compile(r"speed limit lowered to (?P<v>[\d.]+) m/s")
DURATION_RE = re.compile(r"blocked for (?P<v>[\d.]+) s")
TOTAL_RE = re.compile(r"TOTAL \(finished vehicles only\): extra time=(?P<t>[\d.]+)s, total distance=(?P<d>[\d.]+)km")
EDGES_RE = re.compile(r"(?:Congested|Affected) edges: (?P<ids>[^\n]*?)\.\s*$", re.M)


# ───────────────────────── lectura de los reportes ─────────────────────────

def _num(pattern, txt):
    m = re.search(pattern, txt, re.M)
    return float(m.group(1)) if m else None


def parse_result_file(path):
    """Un reporte de dynamic_replanning.py -> dict (o None si no es un reporte completo)."""
    return parse_text(os.path.basename(path), open(path, encoding="utf-8", errors="replace").read())


def parse_text(name, txt):
    m = NAME_RE.search(name)
    if not m:
        return None
    ev, tot, ed = EVENT_RE.search(txt), TOTAL_RE.search(txt), EDGES_RE.search(txt)
    ev2 = None if ev else EVENT_V2_RE.search(txt)
    if not ((ev or ev2) and tot):
        return None
    if ev:      # formato anterior: congestion con sonda (sin selector)
        kind, ck, n_edges, speed, duration = "congestion", float(ev["ck"]), int(ev["edges"]), float(ev["speed"]), 0.0
        ev = {"before": ev["before"], "after": ev["after"]}
    else:       # eventos verificados: congestion / accident / closure
        extra = ev2["extra"] or ""
        sp, du = SPEED_RE.search(extra), DURATION_RE.search(extra)
        kind, ck, n_edges = ev2["kind"], float(ev2["ck"]), int(ev2["edges"])
        speed, duration = (float(sp["v"]) if sp else 0.0), (float(du["v"]) if du else 0.0)   # 0 = no aplica a este tipo
        ev = {"before": ev2["before"], "after": ev2["after"]}
    heur = re.search(r"^Heuristic: (\S+)", txt, re.M)
    meta = re.search(r"^Metaheuristic: (\S+)", txt, re.M)
    before, after = float(ev["before"]), float(ev["after"])
    return {
        "instance": m["inst"], "mode": m["mode"], "stamp": m["stamp"],
        "e_checkpoint": ck, "e_n_edges": n_edges, "e_speed": speed,
        "e_is_accident": int(kind == "accident"), "e_is_closure": int(kind == "closure"), "e_duration_s": duration,
        "event_kind": kind,
        "e_probe_before_s": before, "e_probe_after_s": after,
        "e_probe_slowdown_pct": round(100.0 * (after - before) / before, 2) if before else 0.0,
        "extra_time_s": float(tot["t"]), "total_distance": float(tot["d"]),
        "battery": _num(r"^Battery capacity: ([\d.]+)", txt), "consumption": _num(r"^Battery consumption rate: ([\d.]+)", txt),
        "n_unfinished": len(set(re.findall(r"(ev_\d+): DID NOT finish", txt))),
        "plan_algorithm": "+".join(x for x in (heur and heur[1], meta and meta[1]) if x) or None,
        "event_key": (m["inst"], ed["ids"] if ed else f"{n_edges}@{ck}"),
    }


def read_runs(results_dir=RESULTS_DIR):
    """Reportes sueltos de la carpeta y los que estan dentro de los .zip de la misma carpeta (sin duplicar nombres)."""
    import zipfile
    seen, runs = set(), []
    for p in sorted(glob.glob(os.path.join(results_dir, "*_feedback_*.txt"))):
        seen.add(os.path.basename(p))
        r = parse_result_file(p)
        if r:
            r["file"] = os.path.basename(p)
        runs.append(r)
    for z in sorted(glob.glob(os.path.join(results_dir, "*.zip"))):
        with zipfile.ZipFile(z) as zf:
            for member in zf.namelist():
                base = os.path.basename(member)
                if base in seen or not NAME_RE.search(base):
                    continue
                seen.add(base)
                runs.append(parse_text(base, zf.read(member).decode("utf-8", errors="replace")))
    return [r for r in runs if r]


MANIFEST_NAME = "manifest_cap3.csv"


def manifest_cells(results_dir=RESULTS_DIR):
    """
    Celdas del manifiesto de docs_thesis_assets/correr_experimentos_cap3.py: {(n, plan, evento, replanificador):
    {"WITH": reporte, "NO": reporte}}, quedandose con el ultimo reporte de cada celda. Una misma celda que aparece en
    la Tabla 12 y en la 13 (misma instancia, plan, evento y replanificador) se cuenta UNA sola vez.
    Devuelve None si no hay manifiesto.
    """
    import csv
    path = os.path.join(results_dir, MANIFEST_NAME)
    if not os.path.isfile(path):
        return None
    cells = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (row["n"], row["algo"], row["event"], row["replanner"])
            cells.setdefault(key, {})[row["mode"]] = row["report"]
    return cells


def manifest_pairs(results_dir=RESULTS_DIR, unique=False):
    """
    {reporte WITH: reporte NO} segun el manifiesto (pareja exacta: misma tabla, instancia, plan, evento y
    replanificador), o None si no hay manifiesto. unique=True: una sola pareja por celda aunque la celda aparezca en la
    Tabla 12 y en la 13 (para tablas de datos, para no contarla dos veces).
    """
    if unique:
        cells = manifest_cells(results_dir)
        if cells is None:
            return None
        return {m["WITH"]: m["NO"] for m in cells.values() if "WITH" in m and "NO" in m}
    import csv
    path = os.path.join(results_dir, MANIFEST_NAME)
    if not os.path.isfile(path):
        return None
    cells = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            cells.setdefault((row["table"], row["n"], row["algo"], row["event"], row["replanner"]), {})[row["mode"]] = row["report"]
    return {m["WITH"]: m["NO"] for m in cells.values() if "WITH" in m and "NO" in m}


def _instance_row(path, cache, battery, consumption):
    """
    Caracteristicas de la instancia y del escenario (las mismas que usa la optimizacion). battery y consumption salen
    del propio reporte ('Battery capacity:' / 'Battery consumption rate:'), no de valores fijos.
    """
    if battery is None or consumption is None:
        raise SystemExit(f"El reporte de {os.path.basename(path)} no trae 'Battery capacity:' / 'Battery consumption rate:'")
    key = (path, float(battery), float(consumption))
    if key not in cache:
        from distance.distance_type import DistanceType
        from explainability.features import instance_features, scenario_features
        from instance.instance_type import process_files
        data = process_files(path, DistanceType.OSRM, None, consumption, battery)[os.path.basename(path)]
        cache[key] = {**instance_features(data), **scenario_features(battery, consumption, data)}
    return cache[key]


def build_tables(results_dir=RESULTS_DIR, instances_dir=INSTANCES_DIR):
    """Devuelve (tabla_simulacion, tabla_combinacion) como DataFrames con el formato que espera surrogate.load_dataset."""
    import pandas as pd
    runs = read_runs(results_dir)
    if not runs:
        raise SystemExit(f"No hay reportes en {results_dir}")
    # Con manifiesto (experimentos del capitulo 3) se usan SOLO sus reportes y cada corrida CON se compara con su SIN
    # exacta: mismo plan inicial, mismo evento y mismo replanificador. Sin manifiesto, se empareja por evento como antes
    # (eso mezcla lineas base de planes distintos si hay varios planes para el mismo evento).
    cells = manifest_cells(results_dir)
    pair_of = {}
    if cells is not None:
        by_file = {r.get("file"): r for r in runs if r.get("file")}
        keep = []
        for (n, plan, event, replanner), modes in sorted(cells.items()):
            w, b = by_file.get(modes.get("WITH")), by_file.get(modes.get("NO"))
            for r in (w, b):
                if r is not None:
                    r["plan_from_manifest"], r["replanner"] = plan, replanner
                    keep.append(r)
            if w is not None and b is not None:
                pair_of[id(w)] = b
        missing = [m for modes in cells.values() for m in modes.values() if m not in by_file]
        if missing:
            print(f"Aviso: {len(missing)} reporte(s) del manifiesto no estan en {results_dir}: {missing[:5]}")
        runs = keep
    cache = {}
    event_cols = ("e_checkpoint", "e_n_edges", "e_speed", "e_is_accident", "e_is_closure", "e_duration_s", "e_probe_before_s", "e_probe_after_s", "e_probe_slowdown_pct")

    sim_rows = []
    for k, r in enumerate(runs):
        feats = _instance_row(os.path.join(instances_dir, r["instance"] + ".txt"), cache, r["battery"], r["consumption"])
        sim_rows.append({
            "row_id": k, "algorithm": "SUMO_REPLAN", "instance": r["instance"] + ".txt", "seed": 0, "time_limit": 0,
            "status": "ok", "n_dropped": r["n_unfinished"], **feats,
            **{c: r[c] for c in event_cols}, "e_replan": int(r["mode"] == "WITH"),
            "total_distance": r["total_distance"], "extra_time_s": r["extra_time_s"], "distance": r["total_distance"],
        })

    by_key = {}
    for r in runs:
        by_key.setdefault((r["event_key"], r["mode"]), []).append(r)
    combo_rows = []
    for r in runs:
        if r["mode"] != "WITH":
            continue
        if cells is not None:
            base = [pair_of[id(r)]] if id(r) in pair_of else None
        else:
            base = by_key.get((r["event_key"], "NO"))
        if not base:
            continue            # sin su comparacion sin retroalimentacion no hay ganancia que medir
        d0 = sum(b["total_distance"] for b in base) / len(base)
        t0 = sum(b["extra_time_s"] for b in base) / len(base)
        spread = 100.0 * (max(b["total_distance"] for b in base) - min(b["total_distance"] for b in base)) / d0 if d0 else 0.0
        feats = _instance_row(os.path.join(instances_dir, r["instance"] + ".txt"), cache, r["battery"], r["consumption"])
        combo_rows.append({
            "row_id": len(combo_rows),
            "algorithm": r.get("plan_from_manifest") or r["plan_algorithm"] or "desconocido",
            "instance": r["instance"] + ".txt", "seed": 0, "time_limit": 0, "status": "ok",
            "n_dropped": r["n_unfinished"], **feats, **{c: r[c] for c in event_cols},
            "distance_no_feedback": d0, "distance_with_feedback": r["total_distance"],
            "gain_distance_pct": round(100.0 * (d0 - r["total_distance"]) / d0, 3) if d0 else 0.0,
            "gain_time_pct": round(100.0 * (t0 - r["extra_time_s"]) / t0, 3) if t0 else 0.0,
            "distance": r["total_distance"],
            "n_baselines": len(base), "baseline_spread_pct": round(spread, 2),   # si es alta, los NO de este evento no coinciden entre si
            "event_kind": r["event_kind"], "replanner": r.get("replanner", ""),
            **({"e_replanner_ortools": int(r.get("replanner") == "ortools")} if r.get("replanner") else {}),
            "report_with": r.get("file", ""), "report_no": base[0].get("file", "") if len(base) == 1 else "",
        })
    return pd.DataFrame(sim_rows), pd.DataFrame(combo_rows)


# ───────────────────────── generacion de corridas nuevas ─────────────────────────

class SimulationStrategy(AlgorithmStrategy):
    """
    Corre dynamic_replanning.py dos veces sobre la misma solucion y el mismo evento (sin y con retroalimentacion)
    variando el instante del evento y su severidad. Requiere SUMO, OSRM y las fases 1-4 de simulation/ construidas
    para la instancia (y el algoritmo cuya solucion se usa). Cada fila tarda varios minutos: unas 30-60 filas
    bastan para una primera explicacion.
    """
    name = "SUMO_REPLAN"
    scenario_space = []              # bateria y consumo son fijos en la simulacion (100 y 3,0)
    param_space = [
        {"name": "checkpoint_time", "low": 600, "high": 2400, "kind": "int"},      # instante del evento (s)
        {"name": "congested_speed", "low": 1.0, "high": 8.0, "kind": "float"},     # velocidad impuesta (m/s)
        {"name": "congestion_fraction", "low": 0.2, "high": 0.8, "kind": "float"}, # parte de la ruta afectada
    ]
    result_keys = ("distance", "objective", "n_dropped", "gain_distance_pct", "gain_time_pct", "elapsed")

    def __init__(self, plan_algorithm="PATH_CHEAPEST_ARC_and_GUIDED_LOCAL_SEARCH",
                 solutions_root=os.path.join("problem", "osrm", "solutions_evrp_0"), timeout=3600,
                 name="SUMO_REPLAN", replanner="auto", event_kind="congestion"):
        # plan_algorithm: carpeta solutions_<plan_algorithm> con el plan inicial; replanner: quien re-planifica
        # ('auto' = el mismo algoritmo del plan; 'aco' / 'ortools' lo fuerzan, ver dynamic_replanning.py --replanner)
        self.plan_algorithm, self.solutions_root, self.timeout = plan_algorithm, solutions_root, timeout
        self.name, self.replanner, self.event_kind = name, replanner, event_kind
        if event_kind != "congestion":     # accidente / cierre: evento verificado (selector); la velocidad y la fraccion no aplican
            self.param_space = [{"name": "checkpoint_time", "low": 600, "high": 2400, "kind": "int"}]
            if event_kind == "accident":
                self.param_space.append({"name": "accident_duration", "low": 300, "high": 1200, "kind": "int"})

    @staticmethod
    def _sim_helpers():
        path = os.path.join("simulation", "run.py")
        spec = importlib.util.spec_from_file_location("sim_run", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)          # solo define funciones y constantes
        return mod

    def _launch(self, instance_path, solution, params, replan, event_spec=None):
        helpers = self._sim_helpers()
        cmd = [sys.executable, os.path.join("simulation", "dynamic_replanning.py"),
               "--instance", instance_path, "--solution", solution,
               "--checkpoint-time", str(params["checkpoint_time"]),
               "--replanner", self.replanner,
               "--config", helpers.battery_cfg_for(instance_path, solution),
               "--net-file", helpers.net_file_for(instance_path)]
        if self.event_kind == "congestion":
            cmd += ["--congested-speed", str(params["congested_speed"]),
                    "--congestion-fraction", str(params["congestion_fraction"])]
        else:       # el mismo evento en las dos ramas: la primera lo elige y lo guarda, la segunda lo carga
            cmd += ["--event", self.event_kind, "--verified-event", "--event-spec", event_spec]
            if self.event_kind == "accident":
                cmd += ["--accident-duration", str(params["accident_duration"])]
        if not replan:
            cmd.append("--no-replan")
        started = time.time()
        subprocess.run(cmd, check=True, timeout=self.timeout, stdout=subprocess.DEVNULL)
        mode = "WITH" if replan else "NO"
        base = os.path.splitext(os.path.basename(instance_path))[0]
        files = [p for p in glob.glob(os.path.join(RESULTS_DIR, f"{base}_{mode}_feedback_*.txt"))
                 if os.path.getmtime(p) >= started - 1]
        if not files:
            raise RuntimeError("dynamic_replanning.py no dejo reporte")
        result = parse_result_file(max(files, key=os.path.getmtime))
        if result is None:
            raise RuntimeError("reporte incompleto")
        return result

    def run(self, instance_path, scenario, params, seed, time_limit):
        t0 = time.time()
        solution = os.path.join(self.solutions_root, f"solutions_{self.plan_algorithm}", os.path.basename(instance_path))
        spec = None
        if self.event_kind != "congestion":
            import tempfile
            spec = os.path.join(tempfile.mkdtemp(prefix="evspec_"), "event.json")
        no = self._launch(instance_path, solution, params, replan=False, event_spec=spec)
        with_ = self._launch(instance_path, solution, params, replan=True, event_spec=spec)
        d0, d1, t_0, t_1 = no["total_distance"], with_["total_distance"], no["extra_time_s"], with_["extra_time_s"]
        return {
            "distance": d1, "objective": d1, "n_dropped": with_["n_unfinished"] + no["n_unfinished"],
            "gain_distance_pct": round(100.0 * (d0 - d1) / d0, 3) if d0 else 0.0,
            "gain_time_pct": round(100.0 * (t_0 - t_1) / t_0, 3) if t_0 else 0.0,
            "elapsed": time.time() - t0,
        }


register(SimulationStrategy())
# El mismo lazo con ACO como planificador y re-planificador (plan = mejor ACO de run_aco.py): permite explicar, en una misma
# tabla, que cambia cuando el algoritmo del lazo es ACO en vez de OR-Tools (la columna `algorithm` se vuelve a_<nombre>).
register(SimulationStrategy(plan_algorithm="ACO", solutions_root=os.path.join("problem", "osrm", "solutions_evrp_best"),
                            name="SUMO_REPLAN_ACO"))
# Accidentes y cierres de calle verificados (event_selector): mismo lazo, otro tipo de evento.
for _kind in ("accident", "closure"):
    register(SimulationStrategy(name=f"SUMO_REPLAN_{_kind.upper()}", event_kind=_kind))
    register(SimulationStrategy(plan_algorithm="ACO", solutions_root=os.path.join("problem", "osrm", "solutions_evrp_best"),
                                name=f"SUMO_REPLAN_ACO_{_kind.upper()}", event_kind=_kind))
