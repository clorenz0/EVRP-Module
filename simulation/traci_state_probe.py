"""
traci_state_probe.py — PUNTO 2 de la re-planeación dinámica: correr SUMO de
forma pausable (paso a paso, vía TraCI) y extraer, en un checkpoint dado, el
estado real de cada vehículo activo: posición (lon/lat), batería restante,
y qué nodos EVRP (clientes/estaciones) ya visitó vs. cuáles le faltan.

Todavía NO re-planea nada (eso es el punto 4) — este script solo prueba que
se puede pausar la simulación y leer el estado correctamente. La salida se
imprime en el mismo formato que espera 'vehicle_states' en evrp.execute()
(Punto 1), para que sea directo conectar ambos pasos después.

Uso:
    python simulation/traci_state_probe.py \
        --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
        --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
        --config simulation/sumo_scenario/scenario_battery.sumocfg \
        --checkpoint-time 1200
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import traci  # noqa: E402

from build_routes import parse_instance, parse_solution  # noqa: E402


def parse_args():
    p = argparse.ArgumentParser(description="Punto 2: pausar SUMO y extraer estado real de cada vehiculo")
    p.add_argument("--instance", required=True)
    p.add_argument("--solution", required=True)
    p.add_argument("--config", default=os.path.join("simulation", "sumo_scenario", "scenario_battery.sumocfg"))
    p.add_argument("--checkpoint-time", type=float, default=1200.0,
                    help="Segundo de simulacion en el que se pausa y se lee el estado")
    return p.parse_args()


def get_battery_wh(vid):
    try:
        return float(traci.vehicle.getParameter(vid, "device.battery.actualBatteryCapacity"))
    except traci.TraCIException:
        return None


def main():
    args = parse_args()
    inst = parse_instance(args.instance)
    sol = parse_solution(args.solution)

    # route_of[f"ev_{vid}"] = lista de nodos EVRP en el orden planeado (incluye depot inicial/final)
    routes_by_vehicle = {f"ev_{vid}": route for vid, route in enumerate(sol["routes"])}

    cmd = [
        "sumo", "-c", args.config,
        "--no-warnings", "true",
        "--device.battery.probability", "1.0",
        "--time-to-teleport", "-1",
    ]
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        cand = os.path.join(sumo_home, "bin", "sumo.exe")
        if os.path.isfile(cand):
            cmd[0] = cand

    print(f"Config      : {args.config}")
    print(f"Checkpoint  : {args.checkpoint_time}s\n")
    traci.start(cmd)

    # stops_done[vid] = cuantas paradas (en orden) ya arranco ese vehiculo
    stops_done = {vid: 0 for vid in routes_by_vehicle}
    was_stopped = {vid: False for vid in routes_by_vehicle}
    # seen_ids: vehiculos que aparecieron ACTIVOS al menos una vez antes del
    # checkpoint. Sirve para distinguir "ya termino su ruta" (estuvo activo y
    # ya no) de "nunca tuvo ruta valida" (duarouter lo descarto por completo,
    # nunca llega a aparecer aunque la simulacion avance).
    seen_ids = set()

    try:
        while traci.simulation.getTime() < args.checkpoint_time:
            if traci.simulation.getMinExpectedNumber() == 0:
                print("La simulacion termino antes del checkpoint (todas las rutas completadas).")
                break
            traci.simulationStep()

            for vid in list(traci.vehicle.getIDList()):
                if vid not in routes_by_vehicle:
                    continue
                seen_ids.add(vid)
                stop_state = traci.vehicle.getStopState(vid)
                is_stopped = bool(stop_state & 1)
                if is_stopped and not was_stopped[vid]:
                    stops_done[vid] += 1
                was_stopped[vid] = is_stopped

        # ── Extraer estado de cada vehiculo activo en el checkpoint ──────────
        active_ids = set(traci.vehicle.getIDList())
        print(f"Tiempo real alcanzado: {traci.simulation.getTime():.0f}s")
        print(f"Vehiculos activos en el checkpoint: {sorted(active_ids)}\n")

        results = {}
        for vid, planned_route in routes_by_vehicle.items():
            if vid not in active_ids:
                if vid in seen_ids:
                    print(f"{vid}: ya completo su ruta antes del checkpoint (no aplica re-planeacion)")
                else:
                    print(f"{vid}: SIN RUTA VALIDA desde el inicio (duarouter lo descarto por un arco "
                          f"sin conexion entre dos paradas — no llego a existir en la simulacion)")
                continue

            x, y = traci.vehicle.getPosition(vid)
            lon, lat = traci.simulation.convertGeo(x, y)
            battery_wh = get_battery_wh(vid)

            # nodos intermedios planeados (sin el deposito inicial/final)
            intermediate_nodes = planned_route[1:-1]
            n_done = min(stops_done[vid], len(intermediate_nodes))
            visited_nodes = intermediate_nodes[:n_done]
            pending_nodes = intermediate_nodes[n_done:]

            load_collected = sum(
                inst["clients"][n]["demand"] for n in visited_nodes if n in inst["clients"]
            )

            print(f"{vid}:")
            print(f"  Posicion real       : lat={lat:.5f}, lon={lon:.5f}")
            print(f"  Bateria restante    : {battery_wh:.1f} Wh" if battery_wh is not None else "  Bateria: N/D")
            print(f"  Paradas cumplidas   : {n_done}/{len(intermediate_nodes)}  -> nodos {visited_nodes}")
            print(f"  Paradas pendientes  : {pending_nodes}")
            print(f"  Carga ya recolectada: {load_collected}")
            print()

            results[vid] = {
                "lat": lat, "lon": lon, "battery_wh": battery_wh,
                "pending_nodes": pending_nodes, "load_collected": load_collected,
            }

        print("Listo. Este estado es lo que en el Punto 4 se usaria para construir\n"
              "la sub-instancia EVRP y volver a llamar a evrp.execute() con vehicle_states.")

    finally:
        traci.close()


if __name__ == "__main__":
    main()
