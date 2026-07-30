"""
build_routes.py — FASE 2: convierte la solución final del EVRP (rutas por
vehículo, con paradas en estaciones de carga cuando aplique) en trips reales
de SUMO sobre la red generada en la Fase 1 (build_real_network.py), corre
duarouter para calcular el camino real por las calles, y deja un .sumocfg
listo para abrir en sumo-gui.

No modifica ni importa nada de problem/, distance/ ni instance/ — solo LEE
como texto plano el archivo de instancia (.txt) y el archivo de solución que
ya produce evrp.py (save_solution), exactamente como quedan en disco.

Flujo:
    instancia (.txt) + solución (.txt)
        -> nodos (lat/lon) de cada parada por ruta
        -> snapeo de cada parada al arco real más cercano en network.net.xml
        -> trips.xml (un <trip> por vehículo, con via=arcos intermedios
           en el orden exacto de la ruta EVRP, + <stop> en cada parada:
           duración proporcional a la demanda en clientes, fija en estaciones)
        -> duarouter: trips.xml -> routes.rou.xml (ruta real calle-por-calle)
        -> scenario.sumocfg

Fase 2 = solo visualización básica. Sin tráfico de fondo (Fase 3) ni
dispositivo de batería nativo de SUMO (Fase 4) todavía — la parada en
estaciones de carga sigue siendo un placeholder fijo, no una carga real.

Uso:
    python simulation/build_routes.py \
        --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
        --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
"""
import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from xml.dom import minidom

sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import sumolib  # noqa: E402

STOP_DURATION_S = 10          # parada fija en estaciones de carga (placeholder de visualización; carga real = Fase 4)
MIN_CLIENT_STOP_S = 10        # piso de duración para la parada de un cliente, aunque su demanda sea baja
SERVICE_TIME_PER_UNIT_S = 2   # segundos de descarga por unidad de demanda del cliente (ej. demand=20 -> 40s)
SNAP_RADIUS_M = 2000  # radio de búsqueda del arco real más cercano a cada parada.
                       # Las coordenadas de las instancias EVRP son sintéticas (no
                       # necesariamente sobre una calle real); se observaron paradas
                       # hasta ~900m del arco transitable más cercano.


# ---------------------------------------------------------------------------
# PARSEO (solo lectura de texto plano, no importa código del EVRP)
# ---------------------------------------------------------------------------
def parse_instance(path: str) -> dict:
    """Extrae depósito, clientes y estaciones de carga (id -> lat/lon/nombre)."""
    locations, clients, stations = [], {}, {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            fields = [x.strip() for x in line.split(",")]
            if fields[0] == "DEPOSITO" and len(fields) >= 3:
                locations.append((float(fields[1]), float(fields[2])))
            elif fields[0].startswith("C_") and len(fields) >= 4:
                idx = len(locations)
                locations.append((float(fields[1]), float(fields[2])))
                clients[idx] = {"demand": int(fields[3])}
            elif len(fields) >= 4:
                try:
                    int(fields[0])
                    idx = len(locations)
                    locations.append((float(fields[2]), float(fields[3])))
                    stations[idx] = {"name": fields[1]}
                except ValueError:
                    pass
    return {"node_coords": {i: c for i, c in enumerate(locations)}, "clients": clients, "stations": stations}


def parse_solution(path: str) -> dict:
    """Extrae la lista de rutas (secuencia de índices de nodo) desde el .txt de save_solution."""
    routes, current = [], None
    pat = re.compile(r"(\d+)(?:\[CS:[^\]]+\])?")
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("Route for vehicle"):
                if current is not None:
                    routes.append(current)
                current = []
            elif current is not None and "->" in line:
                for seg in line.split("->"):
                    m = pat.match(seg.strip())
                    if m:
                        current.append(int(m.group(1)))
            elif line.startswith("Distance of the route") and current is not None:
                routes.append(current)
                current = None
    if current:
        routes.append(current)
    return {"routes": routes}


# ---------------------------------------------------------------------------
# SNAPPING: coordenada EVRP -> arco real de la red
# ---------------------------------------------------------------------------
def snap_route_sequential(net, route, node_coords: dict, max_radius=SNAP_RADIUS_M, max_candidates=6):
    """
    Snapea cada nodo de UNA ruta (en el orden en que se visitan) a un arco
    real, verificando que haya un camino real (ruteable, con net.getShortestPath)
    desde el arco elegido para la parada anterior.

    Por qué: snapear cada nodo de forma independiente (al arco más cercano,
    sin mirar el resto de la ruta) podía elegir dos arcos reales sin conexión
    directa entre paradas consecutivas (ej. por sentido único) — duarouter no
    podía construir la ruta y descartaba el vehículo completo. Snapeando en
    orden y verificando conectividad contra la parada anterior, se prefiere
    un candidato un poco más lejos pero conectado, en vez del más cercano
    pero inalcanzable.

    Devuelve {node_idx: edge_id} solo para los nodos de esta ruta, o None si
    algún nodo no tiene ningún arco real dentro del radio de búsqueda.
    """
    snapped = {}
    prev_edge_obj = None
    for node in route:
        lat, lon = node_coords[node]
        x, y = net.convertLonLat2XY(lon, lat)
        candidates = sorted(
            (t for t in net.getNeighboringEdges(x, y, r=max_radius) if t[0].allows("passenger")),
            key=lambda t: t[1]
        )[:max_candidates]
        if not candidates:
            return None

        if prev_edge_obj is None:
            chosen = candidates[0][0]
        else:
            chosen = None
            for edge_obj, _dist in candidates:
                path, _cost = net.getShortestPath(prev_edge_obj, edge_obj, vClass="passenger")
                if path is not None:
                    chosen = edge_obj
                    break
            if chosen is None:
                # Ninguno de los candidatos cercanos conecta con la parada anterior.
                # Se usa el más cercano de todos modos (mejor esfuerzo); duarouter
                # con --repair puede resolverlo igual, o descartar el vehículo si no.
                chosen = candidates[0][0]

        snapped[node] = chosen.getID()
        prev_edge_obj = chosen
    return snapped


def snap_single_node(net, node, node_coords: dict, max_radius=SNAP_RADIUS_M):
    """Snapea un solo nodo (sin contexto de ruta) al arco real más cercano.
    Para nodos que no forman parte de ninguna ruta (ej. una estación de carga
    que ningún vehículo visitó) pero que igual se quieren mostrar en el mapa."""
    lat, lon = node_coords[node]
    x, y = net.convertLonLat2XY(lon, lat)
    candidates = sorted(
        (t for t in net.getNeighboringEdges(x, y, r=max_radius) if t[0].allows("passenger")),
        key=lambda t: t[1]
    )
    return candidates[0][0].getID() if candidates else None


# ---------------------------------------------------------------------------
# TRIPS.XML
# ---------------------------------------------------------------------------
def build_trips(routes, net, instance, out_path):
    root = ET.Element("routes")
    vtype = ET.SubElement(
        root, "vType", id="ev_type", vClass="passenger",
        length="4.5", maxSpeed="33.33", accel="2.6", decel="4.5", sigma="0.5",
    )
    del vtype  # placeholder de físicas reales llega en Fase 4 (Battery Device)

    colors = ["1,0,0", "0,0.7,0", "0,0,1", "1,0.5,0", "0.5,0,0.5", "0,0.8,0.8"]
    written, skipped_routes = 0, []

    for vid, route in enumerate(routes):
        snapped = snap_route_sequential(net, route, instance["node_coords"])
        if snapped is None:
            skipped_routes.append(vid)
            continue
        edges = [snapped[n] for n in route]
        # Colapsar arcos consecutivos repetidos (dos paradas que cayeron en el mismo arco real)
        dedup = [edges[0]]
        for e in edges[1:]:
            if e != dedup[-1]:
                dedup.append(e)
        if len(dedup) < 2:
            skipped_routes.append(vid)
            continue

        trip = ET.SubElement(
            root, "trip",
            id=f"ev_{vid}", type="ev_type",
            depart=str(vid * 10),
            **{"from": dedup[0]}, to=dedup[-1],
            via=" ".join(dedup[1:-1]),
            color=colors[vid % len(colors)],
        )
        for node, edge in zip(route[1:-1], edges[1:-1]):
            if node in instance["clients"]:
                demand = instance["clients"][node]["demand"]
                duration = max(MIN_CLIENT_STOP_S, demand * SERVICE_TIME_PER_UNIT_S)
                label = f"cliente_{node} (demanda={demand})"
            else:
                demand = None
                duration = STOP_DURATION_S
                label = instance["stations"].get(node, {}).get("name", str(node))

            ET.SubElement(trip, "stop", lane=f"{edge}_0", duration=str(duration), parking="true")
            trip.append(ET.Comment(f" parada: nodo {node} ({label}) "))
        written += 1

    _write_pretty_xml(root, out_path)
    print(f"  OK: trips.xml con {written} vehículo(s)"
          + (f", {len(skipped_routes)} ruta(s) omitida(s) por falta de snapping: {skipped_routes}"
             if skipped_routes else ""))


def _write_pretty_xml(root, path):
    raw = ET.tostring(root, encoding="unicode")
    dom = minidom.parseString(raw)
    lines = [l for l in dom.toprettyxml(indent="  ").split("\n") if l.strip() and not l.startswith("<?xml")]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# DUAROUTER
# ---------------------------------------------------------------------------
def find_sumo_tool(name):
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        cand = os.path.join(sumo_home, "bin", name)
        for ext in ("", ".exe"):
            if os.path.isfile(cand + ext):
                return cand + ext
    return name


def run_duarouter(net_file, trips_file, out_dir):
    routes_out = os.path.join(out_dir, "routes.rou.xml")
    cmd = [
        find_sumo_tool("duarouter"),
        "--net-file", net_file,
        "--route-files", trips_file,
        "--output-file", routes_out,
        "--repair", "--repair.from", "--repair.to",
        "--remove-loops",
        "--ignore-errors",
    ]
    print("\n  Corriendo duarouter...")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if not os.path.isfile(routes_out):
        print("  ERROR duarouter:\n" + result.stderr[-3000:])
        sys.exit(1)
    print(f"  OK: routes.rou.xml generado")
    if result.stderr.strip():
        print("  Advertencias duarouter (informativo):\n" + result.stderr[-1500:])
    return routes_out


def write_sumocfg(out_dir, net_file):
    cfg = os.path.join(out_dir, "scenario.sumocfg")
    root = ET.Element("configuration")
    inp = ET.SubElement(root, "input")
    ET.SubElement(inp, "net-file", value=os.path.relpath(net_file, out_dir))
    ET.SubElement(inp, "route-files", value="routes.rou.xml")
    t = ET.SubElement(root, "time")
    ET.SubElement(t, "begin", value="0")
    pr = ET.SubElement(root, "processing")
    ET.SubElement(pr, "time-to-teleport", value="-1")
    _write_pretty_xml(root, cfg)
    print(f"  OK: scenario.sumocfg")
    return cfg


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
# Rutas absolutas (basadas en la ubicacion de este archivo, no en el cwd) para
# que se pueda correr con el boton Play de PyCharm sin configurar argumentos
# ni working directory.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_INSTANCE = os.path.join(REPO_ROOT, "instances_data", "evrp_instances", "quebec_40c_4ev_6cs.txt")
DEFAULT_SOLUTION = os.path.join(
    REPO_ROOT, "problem", "osrm", "solutions_evrp_0",
    "solutions_PATH_CHEAPEST_ARC", "quebec_40c_4ev_6cs.txt"
)
DEFAULT_NET_FILE = os.path.join(REPO_ROOT, "simulation", "sumo_network", "network.net.xml")
DEFAULT_OUTPUT_DIR = os.path.join(REPO_ROOT, "simulation", "sumo_scenario")


def parse_args():
    p = argparse.ArgumentParser(description="Fase 2: rutas EVRP -> trips SUMO -> duarouter -> sumocfg")
    p.add_argument("--instance", default=DEFAULT_INSTANCE)
    p.add_argument("--solution", default=DEFAULT_SOLUTION)
    p.add_argument("--net-file", default=DEFAULT_NET_FILE)
    p.add_argument("--output-dir", "-o", default=DEFAULT_OUTPUT_DIR)
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Instancia : {args.instance}")
    print(f"Solucion  : {args.solution}")
    print(f"Red       : {args.net_file}\n")

    inst = parse_instance(args.instance)
    sol = parse_solution(args.solution)
    print(f"Nodos en la instancia: {len(inst['node_coords'])}  Rutas en la solucion: {len(sol['routes'])}")

    net = sumolib.net.readNet(args.net_file)

    trips_path = os.path.join(args.output_dir, "trips.xml")
    build_trips(sol["routes"], net, inst, trips_path)

    run_duarouter(args.net_file, trips_path, args.output_dir)
    cfg = write_sumocfg(args.output_dir, args.net_file)

    print(f"\nListo. Para visualizar:\n  sumo-gui -c {cfg}")


if __name__ == "__main__":
    main()
