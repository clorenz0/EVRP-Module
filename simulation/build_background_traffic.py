"""
build_background_traffic.py — FASE 3: genera tráfico de fondo aleatorio sobre
la red real (con randomTrips.py) y lo combina con las rutas del EVRP
(routes.rou.xml, de la Fase 2) en un mismo escenario, para comparar el tiempo
real de viaje (con congestión) contra el tiempo que había estimado OSRM.

No modifica nada de la Fase 1/2 — solo agrega un route-file más al .sumocfg.

Flujo:
    randomTrips.py (tráfico de fondo aleatorio sobre network.net.xml)
        -> background.rou.xml (ya ruteado, prefijo 'bg_' para distinguirlo)
    scenario.sumocfg -> ahora carga routes.rou.xml (EVRP) + background.rou.xml
    sumo (headless) -> tripinfo.xml
    Se compara la duración real (con tráfico) de cada vehículo 'ev_*' contra
    el "Time of the route" que reportó evrp.py usando distancia/tiempo OSRM.

Uso:
    python simulation/build_background_traffic.py --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
"""
import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

SUMO_HOME = os.environ.get("SUMO_HOME", r"D:\SUMO")


def find_sumo_tool(name):
    for ext in ("", ".exe"):
        cand = os.path.join(SUMO_HOME, "bin", name + ext)
        if os.path.isfile(cand):
            return cand
    return name


def parse_args():
    p = argparse.ArgumentParser(description="Fase 3: tráfico de fondo con randomTrips.py + comparación de tiempos vs OSRM")
    p.add_argument("--net-file", default=os.path.join("simulation", "sumo_network", "network.net.xml"))
    p.add_argument("--scenario-dir", default=os.path.join("simulation", "sumo_scenario"))
    p.add_argument("--solution", required=True, help="Solución EVRP (.txt) para leer el 'Time of the route' de OSRM")
    p.add_argument("--end", type=float, default=6000, help="Fin de la ventana de inserción de tráfico de fondo (s)")
    p.add_argument("--period", type=float, default=2.0, help="Segundos promedio entre inserciones de tráfico de fondo")
    p.add_argument("--seed", type=int, default=42)
    return p.parse_args()


def run_random_trips(net_file, scenario_dir, end, period, seed):
    trips_out = os.path.join(scenario_dir, "background.trips.xml")
    routes_out = os.path.join(scenario_dir, "background.rou.xml")
    cmd = [
        sys.executable, os.path.join(SUMO_HOME, "tools", "randomTrips.py"),
        "-n", net_file,
        "-o", trips_out,
        "-r", routes_out,
        "-b", "0", "-e", str(end),
        "-p", str(period),
        "--seed", str(seed),
        "--prefix", "bg_",
        "--fringe-factor", "5",   # favorece viajes que entran/salen por el borde de la red (mas realista)
        "--validate",
        "--remove-loops",
    ]
    print(f"Generando trafico de fondo: begin=0 end={end}s period={period}s seed={seed}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if not os.path.isfile(routes_out):
        print("ERROR randomTrips.py:\n" + result.stderr[-3000:])
        sys.exit(1)
    n_vehicles = open(routes_out, encoding="utf-8").read().count("<vehicle ")
    print(f"OK: background.rou.xml generado con {n_vehicles} vehiculos de fondo")
    return routes_out


def update_sumocfg(scenario_dir, net_file, background_routes):
    cfg_path = os.path.join(scenario_dir, "scenario.sumocfg")
    root = ET.Element("configuration")
    inp = ET.SubElement(root, "input")
    ET.SubElement(inp, "net-file", value=os.path.relpath(net_file, scenario_dir))
    ET.SubElement(inp, "route-files", value="routes.rou.xml,background.rou.xml")
    t = ET.SubElement(root, "time")
    ET.SubElement(t, "begin", value="0")
    pr = ET.SubElement(root, "processing")
    ET.SubElement(pr, "time-to-teleport", value="-1")
    from xml.dom import minidom
    raw = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(raw)
    lines = [l for l in dom.toprettyxml(indent="  ").split("\n") if l.strip() and not l.startswith("<?xml")]
    with open(cfg_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"OK: scenario.sumocfg actualizado (routes.rou.xml + background.rou.xml)")
    return cfg_path


def parse_osrm_times(solution_path):
    """Lee 'Time of the route: Xs' en orden -> {vehicle_index: segundos}."""
    times, vid = {}, -1
    with open(solution_path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("Route for vehicle"):
                vid += 1
            elif line.startswith("Time of the route:"):
                times[vid] = float(re.search(r"([\d.]+)", line).group(1))
    return times


def run_sim_and_compare(cfg_path, scenario_dir, osrm_times):
    tripinfo = os.path.join(scenario_dir, "tripinfo_phase3.xml")
    cmd = [
        find_sumo_tool("sumo"), "-c", cfg_path,
        "--no-warnings", "true",
        "--tripinfo-output", tripinfo,
        "--time-to-teleport", "-1",
    ]
    print("\nCorriendo simulacion con trafico de fondo (headless)...")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if not os.path.isfile(tripinfo):
        print("ERROR sumo:\n" + result.stderr[-3000:])
        sys.exit(1)

    tree = ET.parse(tripinfo)
    rows = []
    for t in tree.getroot().findall("tripinfo"):
        vid = t.get("id")
        if vid.startswith("ev_"):
            idx = int(vid.split("_")[1])
            sim_duration = float(t.get("duration"))
            osrm_duration = osrm_times.get(idx)
            rows.append((vid, idx, osrm_duration, sim_duration))

    print(f"\n{'='*70}\n  FASE 3 — Tiempo real (con trafico) vs. estimado por OSRM\n{'='*70}")
    print(f"  {'Vehiculo':<10}{'OSRM (s)':>12}{'Simulado (s)':>16}{'Diferencia':>14}{'  %':>8}")
    for vid, idx, osrm_s, sim_s in sorted(rows, key=lambda r: r[1]):
        if osrm_s is None:
            print(f"  {vid:<10}{'N/D':>12}{sim_s:>16.1f}{'':>14}")
            continue
        diff = sim_s - osrm_s
        pct = (diff / osrm_s) * 100
        print(f"  {vid:<10}{osrm_s:>12.1f}{sim_s:>16.1f}{diff:>+14.1f}{pct:>+7.1f}%")
    print(f"{'='*70}")
    os.remove(tripinfo)


def main():
    args = parse_args()
    if not os.path.isfile(args.net_file):
        sys.exit(f"ERROR: no se encontro {args.net_file}. Corre primero build_real_network.py")
    routes_rou = os.path.join(args.scenario_dir, "routes.rou.xml")
    if not os.path.isfile(routes_rou):
        sys.exit(f"ERROR: no se encontro {routes_rou}. Corre primero build_routes.py (Fase 2)")

    bg_routes = run_random_trips(args.net_file, args.scenario_dir, args.end, args.period, args.seed)
    cfg = update_sumocfg(args.scenario_dir, args.net_file, bg_routes)
    osrm_times = parse_osrm_times(args.solution)
    run_sim_and_compare(cfg, args.scenario_dir, osrm_times)

    print(f"\nListo. Para visualizar con trafico de fondo:\n  sumo-gui -c {cfg}")


if __name__ == "__main__":
    main()
