"""
Explicabilidad del PROCESO DE RETROALIMENTACION (optimizacion + simulacion).

La tabla `combo` de simulation.py explica CUANTO mejora re-planificar (una fila por evento). Este modulo explica
POR QUE y COMO: baja al nivel del vehiculo y de las decisiones que cambia el optimizador cuando recibe la
informacion de SUMO.

  1. build_vehicle_table  -> tabla `veh`: una fila por (evento, vehiculo), con
        r_  estado del vehiculo en el instante del evento (paradas pendientes, avance, distancia recorrida, ...)
        c_  cambios que introdujo la re-planificacion en su ruta (clientes que recibe, clientes que cede,
            reordenamiento, estaciones nuevas)
        e_/f_/s_  evento e instancia (como en las otras tablas)
     respuesta: ganancia del vehiculo (tiempo y distancia sin retroalimentacion - con retroalimentacion).
     Se explica con el mismo sustituto + SHAP:  python run_xai.py explain --data explainability/data/veh.csv
     --target gain_time_vehicle_s --title "RETROALIMENTACION"
  2. explain_replan       -> relato de UN evento (sin modelo de aprendizaje): que paso con cada vehiculo, que clientes
     cambiaron de vehiculo, que estaciones entraron o salieron y cuanto costo/ahorro tuvo cada uno.
     Es la explicacion local y determinista; SHAP da la global.

Fuente: los reportes de simulation/dynamic_replanning.py (carpeta dynamic_replan_results y los .zip que haya).
Como el reporte de re-planificacion escribe los nodos del sub-problema (clientes pendientes 1..P, luego las
estaciones, luego las posiciones de arranque), aqui se reconstruye la correspondencia con los nodos originales
con la misma regla que usa dynamic_replanning.py.
"""
import glob
import os
import re
import zipfile

from explainability.simulation import INSTANCES_DIR, RESULTS_DIR, NAME_RE, parse_text, _instance_row

NODE_RE = re.compile(r"(\d+)(?:\[CS:[^\]]*\])?")
FINISH_RE = re.compile(r"finished -- extra time=([\d.]+)s, total distance \(before\+after\)=([\d.]+)km")


def _nodes(line):
    return [int(m.group(1)) for m in NODE_RE.finditer(line.split("(distance=")[0])]


def iter_reports(results_dir=RESULTS_DIR):
    """(nombre, texto) de cada reporte, sueltos y dentro de .zip (sin repetir nombres)."""
    seen = set()
    for p in sorted(glob.glob(os.path.join(results_dir, "*_feedback_*.txt"))):
        seen.add(os.path.basename(p))
        yield os.path.basename(p), open(p, encoding="utf-8", errors="replace").read()
    for z in sorted(glob.glob(os.path.join(results_dir, "*.zip"))):
        with zipfile.ZipFile(z) as zf:
            for member in zf.namelist():
                base = os.path.basename(member)
                if base not in seen and NAME_RE.search(base):
                    seen.add(base)
                    yield base, zf.read(member).decode("utf-8", errors="replace")


# ───────────────────────── lectura de un reporte ─────────────────────────

def parse_with(txt, n_clients, stations):
    """
    Reporte CON retroalimentacion -> {vehiculo: dict}. n_clients y stations (ids originales, ordenados) del
    problema original, para traducir los nodos del sub-problema a ids originales.
    """
    vehicles, order = {}, []
    blocks = re.split(r"\n(?=Vehicle ev_\d+:)", txt.split("Original routes (before this re-planning event):", 1)[-1])
    for b in blocks:
        m = re.match(r"Vehicle (ev_\d+):", b)
        if not m:
            continue
        lines = [ln for ln in b.splitlines()]
        try:
            orig = _nodes(lines[2])
            done_line = lines[4]
        except IndexError:
            continue
        dm = re.search(r"distance=([\d.]+)km, (\d+) stop", done_line)
        if not dm:
            continue
        inter = orig[1:-1]
        n_done = int(dm.group(2))
        vehicles[m.group(1)] = {
            "planned": inter, "n_done": n_done, "done_km": float(dm.group(1)),
            "pending": inter[n_done:],
        }
        order.append(m.group(1))

    pool = []
    for vid in order:
        for n in vehicles[vid]["pending"]:
            if n <= n_clients and n not in pool:
                pool.append(n)
    sub_to_orig = {i + 1: n for i, n in enumerate(pool)}
    for j, s in enumerate(stations):
        sub_to_orig[len(pool) + 1 + j] = s

    updated = re.findall(r"Route for vehicle (\d+):\n (.*?)\nDistance", txt, re.S)
    for (idx, line), vid in zip(updated, order):
        nodes = [int(m.group(1)) for m in NODE_RE.finditer(re.sub(r"Load\(\d+\)|Bat\(-?[\d.]+\)", "", line))]
        route = []
        for k, n in enumerate(nodes):
            if k == 0:
                continue                      # posicion actual del vehiculo (nodo virtual)
            route.append(0 if (k == len(nodes) - 1) else sub_to_orig.get(n))
        vehicles[vid]["updated"] = [n for n in route if n is not None]

    done = txt.split("Completion stats", 1)[-1]
    for vid, t, d in re.findall(r"(ev_\d+): " + FINISH_RE.pattern, done):
        if vid in vehicles:
            vehicles[vid]["with_time_s"], vehicles[vid]["with_dist_km"] = float(t), float(d)
    return vehicles


def parse_no(txt):
    """Reporte SIN retroalimentacion -> {vehiculo: (tiempo extra, distancia total)}."""
    out = {}
    for blk in re.split(r"\n(?=Vehicle ev_\d+:)", txt):
        m = re.match(r"Vehicle (ev_\d+):", blk)
        f = FINISH_RE.search(blk) if m else None
        if f:
            out[m.group(1)] = (float(f.group(1)), float(f.group(2)))
    return out


# ───────────────────────── cambios por vehiculo ─────────────────────────

def vehicle_changes(v, n_clients):
    """Cambios de la re-planificacion en la ruta pendiente de un vehiculo."""
    before = [n for n in v["pending"] if n <= n_clients]
    after = [n for n in v.get("updated", []) if 0 < n <= n_clients]
    st_before = [n for n in v["pending"] if n > n_clients]
    st_after = [n for n in v.get("updated", []) if n > n_clients]
    b, a = set(before), set(after)
    return {
        "clients_in": sorted(a - b), "clients_out": sorted(b - a), "kept": sorted(a & b),
        "reordered": int(b == a and before != after) if before else 0,
        "stations_before": st_before, "stations_after": st_after,
        "stations_new": [s for s in st_after if s not in st_before],
        "stations_dropped": [s for s in st_before if s not in st_after],
    }


# ───────────────────────── tabla por vehiculo ─────────────────────────

def _instance_shape(instance_path):
    from distance.distance_type import DistanceType
    from instance.instance_type import process_files
    # solo se usan el numero de clientes y las estaciones: bateria y consumo no influyen aqui
    d = process_files(instance_path, DistanceType.OSRM, None, 3.0, 100)[os.path.basename(instance_path)]
    stations = sorted(d["charging_stations"])
    return d["num_locations"] - len(stations) - 1, stations


def build_vehicle_table(results_dir=RESULTS_DIR, instances_dir=INSTANCES_DIR):
    """DataFrame `veh`: una fila por (evento, vehiculo) con formato compatible con surrogate.load_dataset."""
    import pandas as pd
    reports = [(n, t, parse_text(n, t)) for n, t in iter_reports(results_dir)]
    reports = [(n, t, r) for n, t, r in reports if r]
    # Con manifiesto: cada reporte CON se compara con su SIN exacto (mismo plan, evento y replanificador) y cada celda
    # se cuenta una vez. Sin manifiesto: todos los SIN del mismo evento (puede mezclar planes distintos).
    from explainability.simulation import manifest_pairs
    pairs = manifest_pairs(results_dir, unique=True)
    texts = {n: t for n, t, r in reports}
    no_by_key = {}
    for n, t, r in reports:
        if r["mode"] == "NO":
            no_by_key.setdefault(r["event_key"], []).append(parse_no(t))

    cache, shapes, rows = {}, {}, []
    for n, t, r in reports:
        if r["mode"] != "WITH":
            continue
        if pairs is not None:
            if n not in pairs or pairs[n] not in texts:
                continue
            baselines = [parse_no(texts[pairs[n]])]
        elif r["event_key"] in no_by_key:
            baselines = no_by_key[r["event_key"]]
        else:
            continue
        path = os.path.join(instances_dir, r["instance"] + ".txt")
        if path not in shapes:
            shapes[path] = _instance_shape(path)
        n_clients, stations = shapes[path]
        veh = parse_with(t, n_clients, stations)
        feats = _instance_row(path, cache, r["battery"], r["consumption"])
        probe = re.search(r"Probe (ev_\d+)", t)
        for vid, v in veh.items():
            base = [b[vid] for b in baselines if vid in b]
            if not base or "with_time_s" not in v or "updated" not in v:
                continue
            t0 = sum(x[0] for x in base) / len(base)
            d0 = sum(x[1] for x in base) / len(base)
            ch = vehicle_changes(v, n_clients)
            inter = len(v["planned"])
            rows.append({
                "row_id": len(rows), "algorithm": r["plan_algorithm"] or "desconocido", "instance": r["instance"] + ".txt",
                "seed": 0, "time_limit": 0, "status": "ok", "n_dropped": 0, "event": f"{n}", "vehicle": vid,
                **feats,
                "e_checkpoint": r["e_checkpoint"], "e_n_edges": r["e_n_edges"], "e_speed": r["e_speed"],
                "e_probe_slowdown_pct": r["e_probe_slowdown_pct"],
                "r_is_probe": int(bool(probe) and probe.group(1) == vid),
                "r_planned_stops": inter, "r_pending_stops": len(v["pending"]),
                "r_progress": round(v["n_done"] / inter, 3) if inter else 1.0,
                "r_done_km": v["done_km"],
                "r_station_pending": int(any(s > n_clients for s in v["pending"])),
                "c_clients_in": len(ch["clients_in"]), "c_clients_out": len(ch["clients_out"]),
                "c_reordered": ch["reordered"], "c_stations_new": len(ch["stations_new"]),
                "c_stations_dropped": len(ch["stations_dropped"]),
                "time_no_feedback_s": t0, "time_with_feedback_s": v["with_time_s"],
                "dist_no_feedback_km": d0, "dist_with_feedback_km": v["with_dist_km"],
                "gain_time_vehicle_s": round(t0 - v["with_time_s"], 1),
                "gain_distance_vehicle_km": round(d0 - v["with_dist_km"], 3),
                "distance": v["with_dist_km"],
            })
    return pd.DataFrame(rows)


# ───────────────────────── relato de un evento ─────────────────────────

def explain_replan(with_name, with_txt, no_runs, n_clients, stations):
    """
    Texto que explica una re-planificacion: evento, y por vehiculo que cambio y que costo/ahorro tuvo.
    no_runs: lista de reportes SIN retroalimentacion (texto) del mismo evento; se promedian.
    """
    head = parse_text(with_name, with_txt)
    veh = parse_with(with_txt, n_clients, stations)
    base = [parse_no(t) for t in no_runs]
    L = [f"RETROALIMENTACION — {head['instance']}, evento a los {head['e_checkpoint']:.0f} s: {head['e_n_edges']} tramos a "
         f"{head['e_speed']} m/s; el sondeo pasa de {head['e_probe_before_s']:.0f} s a {head['e_probe_after_s']:.0f} s "
         f"({head['e_probe_slowdown_pct']:+.0f} %).", f"Plan original generado con: {head['plan_algorithm']}.", ""]
    tot_t = tot_d = 0.0
    for vid, v in veh.items():
        ch = vehicle_changes(v, n_clients)
        L.append(f"{vid}: {v['n_done']} de {len(v['planned'])} paradas hechas ({v['done_km']:.1f} km); "
                 f"{len(v['pending'])} pendientes.")
        if ch["clients_out"] or ch["clients_in"]:
            L.append(f"  - cede los clientes {ch['clients_out'] or 'ninguno'} y recibe {ch['clients_in'] or 'ninguno'}.")
        elif ch["reordered"]:
            L.append("  - mismos clientes, en otro orden.")
        else:
            L.append("  - su ruta pendiente no cambia.")
        if ch["stations_new"] or ch["stations_dropped"]:
            L.append(f"  - estaciones: entran {ch['stations_new'] or 'ninguna'}, salen {ch['stations_dropped'] or 'ninguna'}.")
        b = [x[vid] for x in base if vid in x]
        if b and "with_time_s" in v:
            t0, d0 = sum(x[0] for x in b) / len(b), sum(x[1] for x in b) / len(b)
            gt, gd = t0 - v["with_time_s"], d0 - v["with_dist_km"]
            tot_t += gt
            tot_d += gd
            L.append(f"  - resultado en SUMO: tiempo extra {t0:.0f} s -> {v['with_time_s']:.0f} s ({gt:+.0f} s), "
                     f"distancia {d0:.1f} km -> {v['with_dist_km']:.1f} km ({gd:+.1f} km).")
    L += ["", f"Balance de la flota: {tot_t:+.0f} s y {tot_d:+.1f} km (positivo = mejora con retroalimentacion)."]
    return "\n".join(L)
