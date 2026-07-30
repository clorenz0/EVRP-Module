"""
dynamic_replanning.py — PUNTO 4: cierra el ciclo completo de re-planeación
dinámica del EVRP, uniendo los Puntos 1-3.

Flujo:
    1. Corre SUMO con TraCI hasta un checkpoint (Punto 2: extracción de estado).
    2. Extrae el estado real de cada vehículo activo con paradas pendientes
       (posición, batería, carga ya recolectada, paradas cumplidas/pendientes).
    3. Dispara un evento de congestión controlado cerca de un vehículo (Punto 3).
    4. Construye una matriz de distancia/tiempo VIVA (con la congestión ya
       reflejada) para un sub-problema: posición actual de cada vehículo
       activo + el pool de sus clientes pendientes + todas las estaciones.
    5. Llama a evrp.execute() con data_override + vehicle_states para
       re-optimizar ese sub-problema con OR-Tools (el mismo motor de siempre,
       ninguna heurística/metaheurística nueva).
    6. Convierte la nueva solución en una ruta real de SUMO (encadenando
       traci.simulation.findRoute() entre paradas consecutivas) e inyecta la
       ruta al vehículo activo con traci.vehicle.setRoute().
    7. Corre unos pasos más para confirmar que el vehículo sigue el plan
       nuevo, y cierra.

No modifica nada de problem/, distance/, instance/ salvo el parámetro nuevo
y opcional data_override de evrp.execute() (Fase de re-planeación).

Uso:
    python simulation/dynamic_replanning.py \
        --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
        --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
        --checkpoint-time 1200
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import sumolib  # noqa: E402
import traci  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from build_routes import (  # noqa: E402
    parse_instance, parse_solution, snap_single_node, _write_pretty_xml,
    MIN_CLIENT_STOP_S, SERVICE_TIME_PER_UNIT_S, STOP_DURATION_S,
)
from problem.execute import evrp  # noqa: E402
from distance.distance_type import DistanceType  # noqa: E402
from problem.strategy_type import HeuristicType  # noqa: E402

CONGESTED_SPEED_MPS = 2.0
CONGESTION_FRACTION = 0.4
REPLAN_TIME_LIMIT = 15
REPLAN_INSTANCE_NAME = "dynamic_replan.txt"


def write_replanned_scenario(out_dir, net_file, vehicle_routes: dict, vehicle_stops: dict):
    """
    Exporta las rutas YA re-optimizadas a un .rou.xml + .sumocfg independiente
    (sin TraCI), para abrir en sumo-gui manualmente desde el segundo 0 con el
    plan nuevo — un sumo-gui manejado por TraCI no se puede "soltar" para que
    quede libre: al cortar la conexión sin un cierre limpio, SUMO lo toma como
    error y se apaga (visto en la practica: 'peer shutdown' + 'Quitting on error').

    vehicle_routes: {vehicle_id: [edge_id, ...]} — la ruta real completa de
    cada vehiculo (ya reconstruida con chain_real_route).
    vehicle_stops: {vehicle_id: [(edge_id, duration_s, label), ...]} — las
    paradas EN ORDEN a lo largo de esa ruta (clientes/estaciones pendientes
    del sub-problema re-optimizado). Sin esto los vehiculos pasan de largo
    sin detenerse en nadie.
    """
    root = ET.Element("routes")
    ET.SubElement(
        root, "vType", id="ev_type", vClass="passenger",
        length="4.5", maxSpeed="33.33", accel="2.6", decel="4.5", sigma="0.5",
    )
    colors = ["1,0,0", "0,0.7,0", "0,0,1", "1,0.5,0", "0.5,0,0.5", "0,0.8,0.8"]
    for i, (vid, edges) in enumerate(vehicle_routes.items()):
        ET.SubElement(root, "route", id=f"route_{vid}", edges=" ".join(edges))
        veh = ET.SubElement(
            root, "vehicle", id=vid, type="ev_type", route=f"route_{vid}",
            depart=str(i * 5), color=colors[i % len(colors)],
        )
        for edge_id, duration, label in vehicle_stops.get(vid, []):
            ET.SubElement(veh, "stop", lane=f"{edge_id}_0", duration=str(duration), parking="true")
            veh.append(ET.Comment(f" parada: {label} "))

    routes_path = os.path.join(out_dir, "routes_dynamic_replan.rou.xml")
    _write_pretty_xml(root, routes_path)

    cfg_root = ET.Element("configuration")
    inp = ET.SubElement(cfg_root, "input")
    ET.SubElement(inp, "net-file", value=os.path.relpath(net_file, out_dir))
    ET.SubElement(inp, "route-files", value="routes_dynamic_replan.rou.xml")
    t = ET.SubElement(cfg_root, "time")
    ET.SubElement(t, "begin", value="0")
    pr = ET.SubElement(cfg_root, "processing")
    ET.SubElement(pr, "time-to-teleport", value="-1")
    cfg_path = os.path.join(out_dir, "scenario_dynamic_replan.sumocfg")
    _write_pretty_xml(cfg_root, cfg_path)

    return cfg_path


def parse_args():
    p = argparse.ArgumentParser(description="Punto 4: orquestador de re-planeacion dinamica EVRP")
    p.add_argument("--instance", required=True)
    p.add_argument("--solution", required=True)
    p.add_argument("--config", default=os.path.join(REPO_ROOT, "simulation", "sumo_scenario", "scenario_battery.sumocfg"))
    p.add_argument("--net-file", default=os.path.join(REPO_ROOT, "simulation", "sumo_network", "network.net.xml"))
    p.add_argument("--checkpoint-time", type=float, default=1200.0)
    p.add_argument("--run-after", type=float, default=300.0,
                    help="Segundos extra de simulacion tras inyectar la ruta nueva, para confirmar que se sigue")
    p.add_argument("--gui", action="store_true",
                    help="Usa sumo-gui en vez de sumo headless, para ver el proceso completo en vivo")
    return p.parse_args()


def get_battery_param(vid, key):
    try:
        return float(traci.vehicle.getParameter(vid, key))
    except traci.TraCIException:
        return None


def step_to_checkpoint(routes_by_vehicle, checkpoint_time):
    stops_done = {vid: 0 for vid in routes_by_vehicle}
    was_stopped = {vid: False for vid in routes_by_vehicle}
    while traci.simulation.getTime() < checkpoint_time:
        if traci.simulation.getMinExpectedNumber() == 0:
            break
        traci.simulationStep()
        for vid in list(traci.vehicle.getIDList()):
            if vid not in routes_by_vehicle:
                continue
            is_stopped = bool(traci.vehicle.getStopState(vid) & 1)
            if is_stopped and not was_stopped[vid]:
                stops_done[vid] += 1
            was_stopped[vid] = is_stopped
    return stops_done


def read_original_params(solution_path):
    params = {}
    with open(solution_path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("Vehicle capacity:"):
                params["vehicle_capacity"] = int(float(re.search(r"([\d.]+)", line).group(1)))
            elif line.startswith("Battery capacity:"):
                params["fuel_capacity"] = int(float(re.search(r"([\d.]+)", line).group(1)))
            elif line.startswith("Battery consumption rate:"):
                params["fuel_consumption_rate"] = float(re.search(r"([\d.]+)", line).group(1))
    return params


def chain_real_route(node_seq, node_edge):
    """Encadena traci.simulation.findRoute() entre paradas consecutivas de una
    secuencia de nodos del sub-problema, para obtener la lista real de arcos
    (calle por calle) que representa esa ruta, con las condiciones actuales
    de trafico."""
    full_edges = []
    for a, b in zip(node_seq, node_seq[1:]):
        stage = traci.simulation.findRoute(node_edge[a], node_edge[b])
        seg = list(stage.edges)
        if not seg:
            return None
        if full_edges and full_edges[-1] == seg[0]:
            seg = seg[1:]
        full_edges.extend(seg)
    return full_edges


def main():
    args = parse_args()
    inst = parse_instance(args.instance)
    sol = parse_solution(args.solution)
    net = sumolib.net.readNet(args.net_file)
    original_params = read_original_params(args.solution)
    routes_by_vehicle = {f"ev_{vid}": route for vid, route in enumerate(sol["routes"])}

    sumo_bin = "sumo-gui" if args.gui else "sumo"
    cmd = [sumo_bin, "-c", args.config, "--no-warnings", "true",
           "--device.battery.probability", "1.0", "--time-to-teleport", "-1"]
    if args.gui:
        cmd += ["--start"]  # arranca corriendo, sin esperar que le den play manualmente
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        cand = os.path.join(sumo_home, "bin", f"{sumo_bin}.exe")
        if os.path.isfile(cand):
            cmd[0] = cand

    print(f"Config     : {args.config}")
    print(f"Checkpoint : {args.checkpoint_time}s")
    print(f"GUI        : {'si' if args.gui else 'no'}\n")
    traci.start(cmd)

    try:
        stops_done = step_to_checkpoint(routes_by_vehicle, args.checkpoint_time)
        active_ids = set(traci.vehicle.getIDList())

        # ── 1. Estado real de cada vehiculo activo con paradas pendientes (Punto 2) ──
        vehicles_state = {}
        for vid, planned_route in routes_by_vehicle.items():
            if vid not in active_ids:
                continue
            intermediate = planned_route[1:-1]
            n_done = min(stops_done[vid], len(intermediate))
            pending = intermediate[n_done:]
            if not pending:
                continue
            battery_wh = get_battery_param(vid, "device.battery.actualBatteryCapacity")
            max_wh = get_battery_param(vid, "device.battery.maximumBatteryCapacity")
            load_collected = sum(
                inst["clients"][n]["demand"] for n in intermediate[:n_done] if n in inst["clients"]
            )
            vehicles_state[vid] = {
                "current_edge": traci.vehicle.getRoadID(vid),
                "battery_wh": battery_wh,
                "max_wh": max_wh,
                "pending_nodes": pending,
                "load_collected": load_collected,
            }
            print(f"{vid}: en {vehicles_state[vid]['current_edge']}, "
                  f"bateria={battery_wh:.0f}/{max_wh:.0f}Wh, "
                  f"{len(pending)} paradas pendientes: {pending}")

        if not vehicles_state:
            sys.exit("Ningun vehiculo activo con paradas pendientes en este checkpoint — nada que re-planear.")

        # ── 2. Congestion controlada cerca del primer vehiculo activo (Punto 3) ──
        # Se imprime ANTES/DESPUES del evento: es la evidencia concreta de que
        # la retroalimentacion es real (SUMO mide un tiempo de viaje distinto
        # tras el evento, con datos actuales, no la estimacion original de OSRM).
        vids = list(vehicles_state.keys())
        first_vid = vids[0]
        probe_target = vehicles_state[first_vid]["pending_nodes"][0]
        probe_edge = snap_single_node(net, probe_target, inst["node_coords"])

        print(f"\n{'─' * 70}\n  RETROALIMENTACION: midiendo impacto de un evento de congestion\n{'─' * 70}")
        stage_before = traci.simulation.findRoute(vehicles_state[first_vid]["current_edge"], probe_edge)
        print(f"  ANTES  ({first_vid} -> nodo {probe_target}): "
              f"tiempo={stage_before.travelTime:.1f}s  distancia={stage_before.length:.0f}m")

        route_edges = list(stage_before.edges)
        n_congest = max(1, int(len(route_edges) * CONGESTION_FRACTION))
        start_idx = max(0, (len(route_edges) - n_congest) // 2)
        congested_edges = route_edges[start_idx:start_idx + n_congest]
        for e in congested_edges:
            traci.edge.setMaxSpeed(e, CONGESTED_SPEED_MPS)
        print(f"  Congestion forzada en {len(congested_edges)} arco(s) reales "
              f"(limite bajado a {CONGESTED_SPEED_MPS} m/s ~ {CONGESTED_SPEED_MPS * 3.6:.0f} km/h)")

        stage_after = traci.simulation.findRoute(vehicles_state[first_vid]["current_edge"], probe_edge)
        delta_t = stage_after.travelTime - stage_before.travelTime
        pct = (delta_t / stage_before.travelTime * 100) if stage_before.travelTime else 0
        print(f"  DESPUES ({first_vid} -> nodo {probe_target}): "
              f"tiempo={stage_after.travelTime:.1f}s  distancia={stage_after.length:.0f}m"
              f"   ->  +{delta_t:.1f}s ({pct:+.1f}%)")
        print(f"{'─' * 70}\n"
              f"  Esta diferencia es la señal real que ahora se usa para construir\n"
              f"  la matriz que ve OR-Tools (paso siguiente), no la estimacion original.\n"
              f"{'─' * 70}\n")

        # ── 3. Sub-instancia con matriz VIVA (Punto 3) ────────────────────────
        # Orden de nodos EXIGIDO por evrp.py: 0=deposito, 1..C=clientes,
        # C+1..fin=nodos "opcionales" (estaciones reales + posiciones de
        # vehiculo, tratadas igual: visita opcional sin penalizacion).
        depot_edge = snap_single_node(net, 0, inst["node_coords"])
        node_edge = {0: depot_edge}
        node_coord = {0: inst["node_coords"][0]}

        pending_pool = []
        for vid in vids:
            for n in vehicles_state[vid]["pending_nodes"]:
                if n in inst["clients"] and n not in pending_pool:
                    pending_pool.append(n)

        idx = 1
        client_sub_node = {}
        for n in pending_pool:
            node_edge[idx] = snap_single_node(net, n, inst["node_coords"])
            node_coord[idx] = inst["node_coords"][n]
            client_sub_node[n] = idx
            idx += 1
        num_clients = len(pending_pool)

        station_sub_nodes = []
        station_names = {}
        for n in sorted(inst["stations"]):
            node_edge[idx] = snap_single_node(net, n, inst["node_coords"])
            node_coord[idx] = inst["node_coords"][n]
            station_sub_nodes.append(idx)
            station_names[idx] = inst["stations"][n]["name"]
            idx += 1

        vehicle_start_node = {}
        for vid in vids:
            node_edge[idx] = vehicles_state[vid]["current_edge"]
            x, y = traci.vehicle.getPosition(vid)
            lon, lat = traci.simulation.convertGeo(x, y)
            node_coord[idx] = (lat, lon)
            vehicle_start_node[vid] = idx
            station_sub_nodes.append(idx)  # tratado como nodo opcional, ver arriba
            station_names[idx] = f"posicion_actual_{vid}"
            idx += 1

        num_locations = idx
        demands = [0] * num_locations
        for n, sub_idx in client_sub_node.items():
            demands[sub_idx] = inst["clients"][n]["demand"]

        print(f"Sub-instancia: {num_locations} nodos ({len(vids)} vehiculos, "
              f"{num_clients} clientes pendientes, {len(inst['stations'])} estaciones)")
        print("Consultando matriz de tiempo/distancia VIVA con SUMO "
              "(findRoute, con la congestion ya activa)...")

        distance_matrix = [[0.0] * num_locations for _ in range(num_locations)]
        time_matrix = [[0.0] * num_locations for _ in range(num_locations)]
        for a in range(num_locations):
            for b in range(num_locations):
                if a == b:
                    continue
                stage_ab = traci.simulation.findRoute(node_edge[a], node_edge[b])
                distance_matrix[a][b] = stage_ab.length / 1000.0  # m -> km
                time_matrix[a][b] = stage_ab.travelTime

        # ── 4. vehicle_states + data_override, re-optimizar con OR-Tools ─────
        vehicle_states = []
        for vid in vids:
            st = vehicles_state[vid]
            soc_fraction = 1.0
            if st["battery_wh"] is not None and st["max_wh"]:
                soc_fraction = min(1.0, st["battery_wh"] / st["max_wh"])
            vehicle_states.append({
                "start_node": vehicle_start_node[vid],
                "initial_fuel": int(soc_fraction * original_params["fuel_capacity"]),
                "initial_load": int(st["load_collected"]),
            })

        sub_data = {
            "num_vehicles": len(vids),
            "vehicle_capacity": original_params["vehicle_capacity"],
            "vehicle_capacities": [original_params["vehicle_capacity"]] * len(vids),
            "fuel_capacity": original_params["fuel_capacity"],
            "fuel_consumption_rate": original_params["fuel_consumption_rate"],
            "locations": [node_coord[i] for i in range(num_locations)],
            "num_locations": num_locations,
            "demands": demands,
            "depot": 0,
            "distance_matrix": distance_matrix,
            "time_matrix": time_matrix,
            "charging_stations": station_sub_nodes,
            "charging_station_names": station_names,
        }

        print("\nRe-optimizando con OR-Tools (evrp.execute con data_override + vehicle_states)...\n")
        evrp.execute(
            0, "dynamic_replan_instance", time_limit=REPLAN_TIME_LIMIT,
            distance_type=DistanceType.OSRM,
            heuristic=HeuristicType.PATH_CHEAPEST_ARC,
            vehicle_states=vehicle_states,
            data_override=sub_data,
            instance_name=REPLAN_INSTANCE_NAME,
        )

        # ── 5. Leer la nueva solucion e inyectarla como ruta real en SUMO ────
        new_sol_path = os.path.join(
            REPO_ROOT, "problem", "osrm", "solutions_evrp_0",
            "solutions_PATH_CHEAPEST_ARC", REPLAN_INSTANCE_NAME
        )
        if not os.path.isfile(new_sol_path):
            sys.exit(f"ERROR: no se genero solucion nueva en {new_sol_path} (revisar log de arriba)")
        new_sol = parse_solution(new_sol_path)

        client_sub_to_orig = {v: k for k, v in client_sub_node.items()}

        print(f"\n{'=' * 70}\n  NUEVO PLAN (re-optimizado con datos en vivo de SUMO)\n{'=' * 70}")
        vehicle_routes = {}
        vehicle_stops = {}
        for k, new_route in enumerate(new_sol["routes"]):
            if k >= len(vids):
                break
            vid = vids[k]
            print(f"\n{vid}: nueva secuencia (nodos del sub-problema): {new_route}")

            full_edges = chain_real_route(new_route, node_edge)
            if not full_edges:
                print(f"  ADVERTENCIA: no se pudo reconstruir una ruta real para {vid}, se deja como estaba")
                continue
            vehicle_routes[vid] = full_edges

            try:
                traci.vehicle.setRoute(vid, full_edges)
                print(f"  OK: ruta real inyectada ({len(full_edges)} arcos)")
            except traci.TraCIException as e:
                print(f"  ERROR inyectando ruta a {vid}: {e}")
                continue

            stops = []
            for node in new_route[1:-1]:
                if node in client_sub_to_orig:
                    demand = demands[node]
                    duration = max(MIN_CLIENT_STOP_S, demand * SERVICE_TIME_PER_UNIT_S)
                    label = f"cliente {client_sub_to_orig[node]} (demanda={demand})"
                else:
                    duration = STOP_DURATION_S
                    label = station_names.get(node, f"nodo {node}")
                stops.append((node_edge[node], duration, label))
                try:
                    traci.vehicle.setStop(vid, node_edge[node], duration=duration)
                except traci.TraCIException:
                    pass
            vehicle_stops[vid] = stops
            print(f"  {len(stops)} parada(s): {[s[2] for s in stops]}")

        print(f"\nCorriendo {args.run_after}s mas para confirmar que el vehiculo sigue el plan nuevo...")
        end_time = traci.simulation.getTime() + args.run_after
        while traci.simulation.getTime() < end_time and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
        print(f"Tiempo final de simulacion: {traci.simulation.getTime():.0f}s")

        # ── 6. Exportar el plan nuevo a un escenario independiente ───────────
        # Un sumo-gui manejado por TraCI no se puede "soltar" para dejarlo
        # abierto (cortar la conexion sin cierre limpio lo hace fallar) — se
        # exporta un .sumocfg aparte para abrir manualmente, a tu ritmo.
        out_dir = os.path.dirname(args.config)
        replan_cfg = write_replanned_scenario(out_dir, args.net_file, vehicle_routes, vehicle_stops)
        print(f"\nPlan nuevo exportado a un escenario independiente:\n  {replan_cfg}")
        print(f"Para verlo (desde el segundo 0, ya con el plan re-optimizado):\n"
              f"  sumo-gui -c {replan_cfg}")

    finally:
        traci.close()


if __name__ == "__main__":
    main()
