"""
generate_sumo_scenario.py — Convierte la solución del solver EVRP a un
escenario SUMO completo con red reescalada, energía correcta y paradas de clientes.

Fixes en esta versión:
  - vType con parámetros de física vehicular requeridos por emissionClass=Energy/unknown
    (sin estos, SUMO devuelve consumo=0 sin advertir nada).
  - device.battery.consumption eliminado — no es un param real de SUMO; el modelo
    de física calcula el consumo automáticamente.
  - Paradas de clientes (pickup) y de carga en orden correcto dentro de cada ruta.
  - write_routes recibe instance y edge_lens para poder posicionar las paradas.
"""
import argparse, math, os, re, subprocess, xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Genera escenario SUMO reescalado a partir de solución EVRP")
    p.add_argument("--instance",        "-i", required=True)
    p.add_argument("--solution",        "-s", required=True)
    p.add_argument("--output-dir",      "-o", default="sumo_scenario")
    p.add_argument("--target-max-edge", type=float, default=1500.0)
    p.add_argument("--target-min-edge", type=float, default=100.0)
    p.add_argument("--max-speed",       type=float, default=13.89)
    p.add_argument("--service-time",    type=int,   default=2,
                   help="Segundos de servicio por unidad de demanda en cada cliente (default=2)")
    return p.parse_args()

# ---------------------------------------------------------------------------
# PARSEO
# ---------------------------------------------------------------------------
def parse_instance(path: str) -> dict:
    locations, clients, stations = [], {}, {}
    num_vehicles = vehicle_capacity = section = None
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                if "CLIENTES"   in line: section = "clients"
                if "ESTACIONES" in line: section = "stations"
                if "DEPOSITO"   in line: section = "depot"
                if "PROBLEMA"   in line: section = "problem"
                continue
            fields = [x.strip() for x in line.split(",")]
            if section == "problem" and len(fields) == 2:
                try: num_vehicles, vehicle_capacity = int(fields[0]), int(fields[1])
                except: pass
            if fields[0] == "DEPOSITO" and len(fields) >= 3:
                locations.append((float(fields[1]), float(fields[2])))
            if fields[0].startswith("C_") and len(fields) >= 4:
                try:
                    coord  = (float(fields[1]), float(fields[2]))
                    demand = int(fields[3])
                    idx    = len(locations)
                    locations.append(coord)
                    clients[idx] = {"coord": coord, "demand": demand}
                except: pass
            if len(fields) >= 4:
                try:
                    int(fields[0])
                    coord = (float(fields[2]), float(fields[3]))
                    idx   = len(locations)
                    locations.append(coord)
                    stations[idx] = {"name": fields[1], "lat": coord[0], "lon": coord[1]}
                except: pass
    return {
        "depot": 0,
        "depot_coord":      locations[0] if locations else None,
        "clients":          clients,
        "stations":         stations,
        "node_coords":      {i: c for i, c in enumerate(locations)},
        "num_vehicles":     num_vehicles,
        "vehicle_capacity": vehicle_capacity,
    }

def parse_solution(path: str) -> dict:
    fc = fcr = None
    routes, dropped, current = [], [], None
    pat = re.compile(r'(\d+)(?:\[CS:[^\]]+\])?')
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if   line.startswith("Fuel capacity:"):        fc  = int(line.split(":")[1].strip())
            elif line.startswith("Fuel consumption rate:"): fcr = float(line.split(":")[1].strip())
            elif line.startswith("Dropped clients:"):
                m = re.search(r'\[([^\]]*)\]', line)
                if m and m.group(1).strip():
                    dropped = [int(x.strip()) for x in m.group(1).split(",")]
            elif line.startswith("Route for vehicle"):
                if current is not None: routes.append(current)
                current = []
            elif current is not None and "->" in line:
                for seg in line.split("->"):
                    m = pat.match(seg.strip())
                    if m: current.append(int(m.group(1)))
            elif line.startswith("Distance of the route") and current is not None:
                routes.append(current); current = None
    if current is not None: routes.append(current)
    return {"fuel_capacity": fc, "fuel_consumption_rate": fcr,
            "routes": routes, "dropped_clients": dropped}

# ---------------------------------------------------------------------------
# GEOMETRÍA
# ---------------------------------------------------------------------------
def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R, p = 6_371_000.0, math.pi / 180
    a = (0.5 - math.cos((lat2-lat1)*p)/2
         + math.cos(lat1*p)*math.cos(lat2*p)*(1-math.cos((lon2-lon1)*p))/2)
    return 2*R*math.asin(math.sqrt(a))

def latlon_to_xy(lat, lon, lat0, lon0) -> tuple:
    R, p = 6_371_000.0, math.pi/180
    return R*(lon-lon0)*p*math.cos(lat0*p), R*(lat-lat0)*p

def compute_scale_factor(instance, routes, tmax, tmin) -> float:
    coords, edges, seen = instance["node_coords"], [], set()
    for route in routes:
        for a, b in zip(route, route[1:]):
            if (a, b) not in seen:
                seen.add((a, b))
                ca, cb = coords.get(a), coords.get(b)
                if ca and cb: edges.append(haversine_m(*ca, *cb))
    if not edges: return 1.0
    mx = max(edges)
    print(f"\n  📏 Edge más larga: {mx/1000:.1f} km, k = {tmax/mx:.6f}")
    return tmax / mx

def build_xy_coords(instance, scale) -> dict:
    lat0, lon0 = instance["depot_coord"]
    return {
        idx: (round(latlon_to_xy(lat, lon, lat0, lon0)[0]*scale, 3),
              round(latlon_to_xy(lat, lon, lat0, lon0)[1]*scale, 3))
        for idx, (lat, lon) in instance["node_coords"].items()
    }

# ---------------------------------------------------------------------------
# GENERACIÓN DE XML
# ---------------------------------------------------------------------------
def write_nodes(out_path, xy_coords, routes):
    active = {n for r in routes for n in r}
    root   = ET.Element("nodes")
    for idx in sorted(active):
        if idx in xy_coords:
            x, y = xy_coords[idx]
            ET.SubElement(root, "node", id=f"n{idx}", x=str(x), y=str(y), type="priority")
    _write_pretty_xml(root, out_path)
    print(f"\n  ✅ nodes.nod.xml ({len(active)} nodos)")

def write_edges(out_path, xy_coords, routes, max_speed) -> dict:
    seen, lens, root = set(), {}, ET.Element("edges")
    for route in routes:
        for a, b in zip(route, route[1:]):
            eid = f"e{a}_{b}"
            if eid not in seen:
                seen.add(eid)
                xa, ya = xy_coords.get(a, (0, 0))
                xb, yb = xy_coords.get(b, (0, 0))
                length = max(round(math.hypot(xb-xa, yb-ya), 3), 1.0)
                lens[eid] = length
                ET.SubElement(root, "edge", id=eid, **{"from": f"n{a}"}, to=f"n{b}",
                              length=str(length), numLanes="1", speed=str(max_speed))
    _write_pretty_xml(root, out_path)
    print(f"  ✅ edges.edg.xml ({len(seen)} arcos)")
    return lens

def write_additional(out_path, instance, routes, edge_lens, fuel_capacity) -> set:
    """Genera additional.add.xml con chargingStations posicionadas en el arco de entrada."""
    root       = ET.Element("additional")
    visited_cs = {n for r in routes for n in r if n in instance["stations"]}
    # Tasa que llena la batería completa en 300 s (1 Wh/timestep con timestep=1 s)
    charge_rate = round(fuel_capacity / 300.0, 4)
    for cs_idx in sorted(visited_cs):
        cs   = instance["stations"][cs_idx]
        lane = elen = None
        for route in routes:
            for i, n in enumerate(route):
                if n == cs_idx and i > 0:
                    eid = f"e{route[i-1]}_{cs_idx}"
                    if eid in edge_lens:
                        lane, elen = f"{eid}_0", edge_lens[eid]
                        break
            if lane: break
        if not lane: continue
        s, e = round(elen * 0.10, 2), round(elen * 0.90, 2)
        if e - s < 10: s, e = 0.0, elen
        ET.SubElement(root, "chargingStation",
                      id=f"cs_{cs_idx}", name=cs["name"],
                      lane=lane, startPos=str(s), endPos=str(e),
                      chargePerTimeStep=str(charge_rate),
                      chargeEfficiency="1.0", chargeDelay="0")
    _write_pretty_xml(root, out_path)
    print(f"  ✅ additional.add.xml ({len(visited_cs)} chargingStations, tasa={charge_rate} Wh/s)")
    return visited_cs


def write_routes(out_path: str, solution: dict, instance: dict,
                 visited_cs: set, scale: float, max_speed: float,
                 edge_lens: dict, service_time_per_unit: int = 2):
    """
    Genera vehicles.rou.xml con:

    1. vType con parámetros de FÍSICA VEHICULAR para Energy/unknown
       ─────────────────────────────────────────────────────────────
       emissionClass='Energy/unknown' usa las ecuaciones de Newton para
       calcular la potencia requerida en cada paso. SUMO necesita saber
       la masa, el drag aerodinámico y la eficiencia del motor.
       SIN estos atributos, todas las fuerzas dan 0 → consumo = 0 Wh.

    2. Paradas ORDENADAS por posición en la ruta
       ──────────────────────────────────────────
       SUMO exige que los <stop> aparezcan en el mismo orden que los
       arcos de la ruta. Se procesan en secuencia: para cada nodo
       visitado se añade parada de cliente (pickup) o de CS (carga).

    Parámetros de física (turismo eléctrico típico ~1 500 kg):
        mass                   : 1 500 kg
        frontSurfaceArea       : 2.6 m²
        airDragCoefficient     : 0.35  (Cd)
        internalMomentOfInertia: 0.01 kg·m²
        radialDragCoefficient  : 0.1
        rollDragCoefficient    : 0.01  (Crr)
        constantPowerIntake    : 100 W (accesorios)
        propulsionEfficiency   : 0.9
        recuperationEfficiency : 0.0  (sin frenada regenerativa)
    """
    fc      = solution["fuel_capacity"]
    clients = instance["clients"]

    root = ET.Element("routes")

    # ------------------------------------------------------------------
    #  vType — PHYSICS FIX
    # ------------------------------------------------------------------
    vtype = ET.SubElement(root, "vType",
        id="ev_type",
        vClass="passenger",
        emissionClass="Energy/unknown",
        length="4.5",
        maxSpeed=str(max_speed),
        accel="2.6",
        decel="4.5",
        sigma="0.5",
        # Parámetros de física — obligatorios para Energy/unknown
        mass="1500",
        frontSurfaceArea="2.6",
        airDragCoefficient="0.35",
        internalMomentOfInertia="0.01",
        radialDragCoefficient="0.1",
        rollDragCoefficient="0.01",
        constantPowerIntake="100",
        propulsionEfficiency="0.9",
        recuperationEfficiency="0.0",
        stoppingThreshold="0.1",
    )

    # Dispositivo de batería
    ET.SubElement(vtype, "param", key="has.battery.device",                    value="true")
    ET.SubElement(vtype, "param", key="device.battery.maximumBatteryCapacity", value=str(fc))
    ET.SubElement(vtype, "param", key="device.battery.actualBatteryCapacity",  value=str(fc))
    ET.SubElement(vtype, "param", key="device.battery.recuperationEfficiency", value="0.0")

    print(f"\n  🔋 vType configurado:")
    print(f"    emissionClass          : Energy/unknown  (modelo físico newtoniano)")
    print(f"    mass / Cd / Crr        : 1500 kg / 0.35 / 0.01")
    print(f"    propulsionEfficiency   : 0.9")
    print(f"    maximumBatteryCapacity : {fc} Wh  (carga inicial = 100%)")

    total_client_stops = 0

    for vid, route in enumerate(solution["routes"]):
        edges = [f"e{a}_{b}" for a, b in zip(route, route[1:])]
        ET.SubElement(root, "route", id=f"route_{vid}", edges="  ".join(edges))

        veh = ET.SubElement(root, "vehicle",
                            id=f"ev_{vid}", type="ev_type",
                            route=f"route_{vid}", depart=str(vid * 5),
                            color=["1,0,0","0,0.7,0","0,0,1","1,0.5,0","0.5,0,0.5"][vid % 5])

        # --------------------------------------------------------------
        #  Paradas EN ORDEN DE RUTA
        #
        #  Iteramos desde el segundo nodo (route[1]) — el primero es el
        #  depósito de salida. El depósito de llegada (route[-1] == 0)
        #  no está en `clients` ni en `visited_cs`, así que se ignora.
        # --------------------------------------------------------------
        cs_in_route     = 0
        client_in_route = 0

        for i, node in enumerate(route[1:], start=1):
            prev_node    = route[i - 1]
            arriving_eid = f"e{prev_node}_{node}"

            if node in visited_cs:
                # Parada de carga: el vehículo espera en la CS hasta 300 s.
                # SUMO carga automáticamente mientras espera (gracias a
                # has.battery.device y la definición del chargingStation).
                ET.SubElement(veh, "stop",
                              chargingStation=f"cs_{node}",
                              duration="300")
                cs_in_route += 1

            elif node in clients:
                # Parada de recogida (pickup del cliente).
                # Posición: 95 % del arco de entrada (junto al nodo cliente).
                # Duración: demand × service_time_per_unit, mínimo 10 s.
                elen     = edge_lens.get(arriving_eid, 100.0)
                stop_pos = round(min(elen - 0.5, elen * 0.95), 2)
                demand   = clients[node]["demand"]
                duration = max(10, demand * service_time_per_unit)
                ET.SubElement(veh, "stop",
                              lane=f"{arriving_eid}_0",
                              endPos=str(stop_pos),
                              duration=str(duration))
                client_in_route  += 1
                total_client_stops += 1

        print(f"    ev_{vid}: {len(route)-2} nodos intermedios  "
              f"→ {client_in_route} paradas-cliente + {cs_in_route} paradas-CS")

    _write_pretty_xml(root, out_path)
    print(f"  ✅ vehicles.rou.xml  ({len(solution['routes'])} vehículos, "
          f"{total_client_stops} paradas-cliente, {len(visited_cs)} CS visitadas)")


# ---------------------------------------------------------------------------
# NETCONVERT + SUMOCFG
# ---------------------------------------------------------------------------
def build_network(out_dir) -> bool:
    nc = "netconvert"
    sh = os.environ.get("SUMO_HOME", "")
    if sh:
        c = os.path.join(sh, "bin", "netconvert")
        if os.path.isfile(c) or os.path.isfile(c + ".exe"): nc = c
    cmd = [
        nc,
        "--node-files",  os.path.join(out_dir, "nodes.nod.xml"),
        "--edge-files",  os.path.join(out_dir, "edges.edg.xml"),
        "--output-file", os.path.join(out_dir, "network.net.xml"),
        "--no-warnings", "--junctions.join", "false",
        "--no-internal-links", "--roundabouts.guess", "false",
    ]
    print(f"\n  🌐 Ejecutando netconvert...")
    try:
        r  = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        np = os.path.join(out_dir, "network.net.xml")
        if r.returncode == 0 and os.path.isfile(np) and os.path.getsize(np) > 0:
            print(f"  ✅ network.net.xml generado"); return True
        print(f"  ❌ ERROR netconvert:\n{r.stderr[-1000:]}"); return False
    except FileNotFoundError:
        print("  ❌ ERROR: 'netconvert' no encontrado. Configura SUMO_HOME."); return False

def write_sumocfg(out_dir) -> str:
    cfg  = os.path.join(out_dir, "scenario.sumocfg")
    root = ET.Element("configuration")
    inp  = ET.SubElement(root, "input")
    ET.SubElement(inp, "net-file",         value="network.net.xml")
    ET.SubElement(inp, "route-files",      value="vehicles.rou.xml")
    ET.SubElement(inp, "additional-files", value="additional.add.xml")
    t  = ET.SubElement(root, "time")
    ET.SubElement(t,  "begin", value="0"); ET.SubElement(t, "end", value="86400")
    pr = ET.SubElement(root, "processing")
    ET.SubElement(pr, "collision.action",  value="warn")
    ET.SubElement(pr, "time-to-teleport", value="-1")
    _write_pretty_xml(root, cfg)
    print(f"  ✅ scenario.sumocfg")
    return cfg

def _write_pretty_xml(root, path):
    raw   = ET.tostring(root, encoding="unicode")
    dom   = minidom.parseString(raw)
    lines = [l for l in dom.toprettyxml(indent="  ").split("\n")
             if l.strip() and not l.startswith("<?xml")]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def main():
    args = parse_args()
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    print(f"\n{'='*60}\n  EVRP -> SUMO (física + clientes + reescalado)\n{'='*60}")

    inst  = parse_instance(args.instance)
    sol   = parse_solution(args.solution)
    print(f"\n  Nodos    : {len(inst['node_coords'])}  "
          f"(depósito + {len(inst['clients'])} clientes + {len(inst['stations'])} CS)")
    print(f"  Rutas    : {len(sol['routes'])}")
    print(f"  Dropped  : {sol['dropped_clients']}")

    scale = compute_scale_factor(inst, sol["routes"], args.target_max_edge, args.target_min_edge)
    xy    = build_xy_coords(inst, scale)

    print(f"\n  📝 Generando XMLs...")
    write_nodes(os.path.join(args.output_dir, "nodes.nod.xml"), xy, sol["routes"])
    elens = write_edges(os.path.join(args.output_dir, "edges.edg.xml"),
                        xy, sol["routes"], args.max_speed)
    vcs   = write_additional(os.path.join(args.output_dir, "additional.add.xml"),
                             inst, sol["routes"], elens, sol["fuel_capacity"])
    write_routes(
        os.path.join(args.output_dir, "vehicles.rou.xml"),
        sol, inst, vcs, scale, args.max_speed, elens,
        service_time_per_unit=args.service_time,
    )
    build_network(args.output_dir)
    write_sumocfg(args.output_dir)
    print(f"\n{'='*60}\n  📦 Listo en: {args.output_dir}/\n{'='*60}\n")

if __name__ == "__main__":
    main()