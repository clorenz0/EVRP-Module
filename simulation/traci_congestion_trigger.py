"""
traci_congestion_trigger.py — PUNTO 3: dispara un evento de congestión
controlado y reproducible (baja el límite de velocidad de un tramo real vía
TraCI), y confirma que se refleja en el tiempo/distancia de viaje que SUMO
reporta con datos ACTUALES (traci.simulation.findRoute), no con la
estimación estática de OSRM que usó evrp.py al planear.

Esta es la pieza de retroalimentación que necesita el Punto 4: en vez de que
la sub-instancia de re-planeación reutilice la matriz original de OSRM, se
construye una matriz "viva" con build_live_time_matrix(), consultando a SUMO
el tiempo/distancia real entre cada par de nodos relevantes EN ESE MOMENTO
de la simulación (con la congestión ya reflejada).

No cierra el ciclo completo todavía (eso es el Punto 4: tomar esta matriz
viva + el estado del Punto 2 y volver a llamar a evrp.execute()) — este
script solo prueba que la señal de retroalimentación es real y medible.

Uso:
    python simulation/traci_congestion_trigger.py \
        --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
        --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
        --config simulation/sumo_scenario/scenario_battery.sumocfg \
        --checkpoint-time 1200
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import sumolib  # noqa: E402
import traci  # noqa: E402

from build_routes import parse_instance, parse_solution, snap_single_node  # noqa: E402

CONGESTED_SPEED_MPS = 2.0     # ~7 km/h: velocidad forzada en el tramo congestionado
CONGESTION_FRACTION = 0.4     # fraccion (del medio) de los arcos de la ruta que se congestionan


def build_live_time_matrix(node_edges: dict) -> dict:
    """
    node_edges: {node_idx: edge_id} de los nodos relevantes (posición actual
    del vehículo + paradas pendientes).

    Devuelve {(node_a, node_b): {'time_s': float, 'distance_m': float}} usando
    traci.simulation.findRoute(), que calcula con las condiciones ACTUALES de
    tráfico de la simulación — no la estimación estática que calculó OSRM al
    planear. Esta es la matriz "viva" que el Punto 4 le pasaría a evrp.execute()
    en vez de la original.
    """
    nodes = list(node_edges.keys())
    matrix = {}
    for a in nodes:
        for b in nodes:
            if a == b:
                continue
            stage = traci.simulation.findRoute(node_edges[a], node_edges[b])
            matrix[(a, b)] = {"time_s": stage.travelTime, "distance_m": stage.length}
    return matrix


def parse_args():
    p = argparse.ArgumentParser(description="Punto 3: evento de congestion controlado + verificacion de retroalimentacion")
    p.add_argument("--instance", required=True)
    p.add_argument("--solution", required=True)
    p.add_argument("--config", default=os.path.join("simulation", "sumo_scenario", "scenario_battery.sumocfg"))
    p.add_argument("--net-file", default=os.path.join("simulation", "sumo_network", "network.net.xml"))
    p.add_argument("--checkpoint-time", type=float, default=1200.0)
    return p.parse_args()


def main():
    args = parse_args()
    inst = parse_instance(args.instance)
    sol = parse_solution(args.solution)
    net = sumolib.net.readNet(args.net_file)
    routes_by_vehicle = {f"ev_{vid}": route for vid, route in enumerate(sol["routes"])}

    cmd = ["sumo", "-c", args.config, "--no-warnings", "true",
           "--device.battery.probability", "1.0", "--time-to-teleport", "-1"]
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        cand = os.path.join(sumo_home, "bin", "sumo.exe")
        if os.path.isfile(cand):
            cmd[0] = cand

    print(f"Config      : {args.config}")
    print(f"Checkpoint  : {args.checkpoint_time}s\n")
    traci.start(cmd)

    stops_done = {vid: 0 for vid in routes_by_vehicle}
    was_stopped = {vid: False for vid in routes_by_vehicle}

    try:
        while traci.simulation.getTime() < args.checkpoint_time:
            if traci.simulation.getMinExpectedNumber() == 0:
                print("La simulacion termino antes del checkpoint.")
                break
            traci.simulationStep()
            for vid in list(traci.vehicle.getIDList()):
                if vid not in routes_by_vehicle:
                    continue
                is_stopped = bool(traci.vehicle.getStopState(vid) & 1)
                if is_stopped and not was_stopped[vid]:
                    stops_done[vid] += 1
                was_stopped[vid] = is_stopped

        # ── Elegir un vehiculo activo con al menos una parada pendiente ──────
        active_ids = set(traci.vehicle.getIDList())
        target_vid, target_node, current_edge = None, None, None
        for vid, planned_route in routes_by_vehicle.items():
            if vid not in active_ids:
                continue
            intermediate = planned_route[1:-1]
            n_done = min(stops_done[vid], len(intermediate))
            pending = intermediate[n_done:]
            if pending:
                target_vid = vid
                target_node = pending[0]
                current_edge = traci.vehicle.getRoadID(vid)
                break

        if target_vid is None:
            sys.exit("No hay ningun vehiculo activo con paradas pendientes en este checkpoint.")

        target_edge_id = snap_single_node(net, target_node, inst["node_coords"])
        print(f"Vehiculo elegido: {target_vid}")
        print(f"  Arco actual real       : {current_edge}")
        print(f"  Proxima parada pendiente: nodo {target_node} -> arco {target_edge_id}\n")

        # ── Baseline: tiempo/distancia SIN congestion, con datos actuales ────
        stage_before = traci.simulation.findRoute(current_edge, target_edge_id)
        print(f"ANTES  de la congestion: tiempo={stage_before.travelTime:.1f}s  "
              f"distancia={stage_before.length:.0f}m  ({len(stage_before.edges)} arcos)")

        # ── Disparar congestion controlada en una porcion de esa ruta ────────
        route_edges = list(stage_before.edges)
        n_congest = max(1, int(len(route_edges) * CONGESTION_FRACTION))
        start_idx = max(0, (len(route_edges) - n_congest) // 2)
        congested_edges = route_edges[start_idx:start_idx + n_congest]

        for e in congested_edges:
            traci.edge.setMaxSpeed(e, CONGESTED_SPEED_MPS)
        print(f"\nCongestion forzada en {len(congested_edges)} arco(s) (limite bajado a "
              f"{CONGESTED_SPEED_MPS} m/s ~ {CONGESTED_SPEED_MPS * 3.6:.0f} km/h):")
        print(f"  {congested_edges}")

        # ── Re-consultar con la congestion ya activa ──────────────────────────
        stage_after = traci.simulation.findRoute(current_edge, target_edge_id)
        print(f"\nDESPUES de la congestion: tiempo={stage_after.travelTime:.1f}s  "
              f"distancia={stage_after.length:.0f}m  ({len(stage_after.edges)} arcos)")

        delta_t = stage_after.travelTime - stage_before.travelTime
        pct = (delta_t / stage_before.travelTime * 100) if stage_before.travelTime else 0
        print(f"\n{'='*70}")
        print(f"  RETROALIMENTACION CONFIRMADA: +{delta_t:.1f}s ({pct:+.1f}%) en el tiempo de viaje")
        print(f"  Esta diferencia es justo lo que el Punto 4 usaria para actualizar")
        print(f"  la matriz de tiempo antes de volver a llamar a evrp.execute().")
        print(f"{'='*70}")

        # ── Demostrar la matriz "viva" reutilizable para el Punto 4 ───────────
        planned_route = routes_by_vehicle[target_vid]
        intermediate = planned_route[1:-1]
        n_done = min(stops_done[target_vid], len(intermediate))
        pending_nodes = intermediate[n_done:][:4]  # limitar a 4 para que la demo sea rapida

        node_edges = {"actual": current_edge}
        for n in pending_nodes:
            node_edges[n] = snap_single_node(net, n, inst["node_coords"])

        print(f"\nMatriz viva de ejemplo (posicion actual + {len(pending_nodes)} paradas pendientes):")
        live_matrix = build_live_time_matrix(node_edges)
        for (a, b), vals in live_matrix.items():
            print(f"  {a} -> {b}: {vals['time_s']:.1f}s, {vals['distance_m']:.0f}m")

    finally:
        traci.close()


if __name__ == "__main__":
    main()
