"""
build_battery_scenario.py — FASE 4: activa el Battery Device nativo de SUMO
sobre el escenario de la Fase 2 (rutas reales por calle) para simular consumo
de energía físico (Newtoniano: masa, arrastre, fricción) en vez de la tasa
lineal simplificada que usa el modelo OR-Tools, y conecta las paradas en
estaciones de carga a chargingStations reales que sí recargan la batería.

Mapeo de parámetros OR-Tools -> SUMO (decisión de diseño, ver conversación):
    OR-Tools 'fuel_capacity'/'fuel_consumption_rate' NO son energía real —
    son un presupuesto lineal de kilómetros (100 unidades = 100 km de rango,
    a 1 unidad/km). El Battery Device de SUMO calcula consumo real en Wh a
    partir de física de vehículo, así que poner maximumBatteryCapacity=100
    (Wh) produciría una batería absurda que se agota en segundos.

    En vez de igualar el NÚMERO, se preserva el RANGO: se calibra la batería
    para que, bajo la física de un EV típico (1500 kg) manejando estas rutas
    reales, el rango resultante sea el mismo que 'fuel_capacity' km del EVRP.
    Esto se hace en dos pasadas:
        1) Pasada de calibración: batería "infinita", se mide el consumo real
           (Wh/km) que da SUMO en esta red con este tráfico.
        2) Pasada final: capacidad = (Wh/km medido) * fuel_capacity (km),
           y las paradas en estaciones de carga usan un chargingStation real
           (no solo una parada genérica) para recargar de verdad.

No modifica nada de problem/, distance/, instance/ ni de las Fases 1-2 —
reutiliza el parseo y snapping de build_routes.py por import.

Uso:
    python simulation/build_battery_scenario.py \
        --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
        --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
"""
import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import sumolib  # noqa: E402

from build_routes import (  # noqa: E402
    parse_instance, parse_solution, snap_nodes_to_edges,
    find_sumo_tool, _write_pretty_xml,
    MIN_CLIENT_STOP_S, SERVICE_TIME_PER_UNIT_S,
)

CALIBRATION_CAPACITY_WH = 1e8   # "infinita" para la pasada de calibración (nunca se agota)
CS_STOP_DURATION_S = 300        # duración fija de la parada de carga (5 min, sesión de carga rápida típica)
CHARGE_POWER_W = 150_000        # 150 kW: potencia típica de un cargador DC rápido real

# LIMITACIÓN CONOCIDA: el <stop duration=...> mantiene al vehículo cargando la
# duración completa aunque llegue al 100% antes — SUMO no corta la carga sola.
# Con potencia realista el sobre-consumo por esto es pequeño, pero puede seguir
# viéndose actualBatteryCapacity > maximumBatteryCapacity en el tripinfo crudo.
# Una versión más precisa cortaría la parada dinámicamente por TraCI cuando el
# SOC llegue a 100% (fuera de alcance de esta Fase 4 inicial).


# ---------------------------------------------------------------------------
# vType con física real + Battery Device
# ---------------------------------------------------------------------------
def add_battery_vtype(root, battery_capacity_wh):
    vtype = ET.SubElement(
        root, "vType", id="ev_type", vClass="passenger",
        emissionClass="Energy/unknown",
        length="4.5", maxSpeed="33.33", accel="2.6", decel="4.5", sigma="0.5",
        # Física de un EV típico (~1500 kg) — Energy/unknown la necesita para
        # calcular la fuerza requerida en cada paso; sin esto el consumo da 0.
        mass="1500", frontSurfaceArea="2.6", airDragCoefficient="0.35",
        internalMomentOfInertia="0.01", radialDragCoefficient="0.1",
        rollDragCoefficient="0.01", constantPowerIntake="100",
        propulsionEfficiency="0.9", recuperationEfficiency="0.0",
        stoppingThreshold="0.1",
    )
    ET.SubElement(vtype, "param", key="has.battery.device", value="true")
    ET.SubElement(vtype, "param", key="device.battery.maximumBatteryCapacity", value=str(battery_capacity_wh))
    ET.SubElement(vtype, "param", key="device.battery.actualBatteryCapacity", value=str(battery_capacity_wh))
    ET.SubElement(vtype, "param", key="device.battery.recuperationEfficiency", value="0.0")


def build_battery_trips(routes, snapped, instance, out_path, battery_capacity_wh, charging_enabled):
    root = ET.Element("routes")
    add_battery_vtype(root, battery_capacity_wh)

    colors = ["1,0,0", "0,0.7,0", "0,0,1", "1,0.5,0", "0.5,0,0.5", "0,0.8,0.8"]
    written, skipped_routes, cs_used = 0, [], set()

    for vid, route in enumerate(routes):
        edges = [snapped.get(n) for n in route]
        if any(e is None for e in edges):
            skipped_routes.append(vid)
            continue
        dedup = [edges[0]]
        for e in edges[1:]:
            if e != dedup[-1]:
                dedup.append(e)
        if len(dedup) < 2:
            skipped_routes.append(vid)
            continue

        trip = ET.SubElement(
            root, "trip", id=f"ev_{vid}", type="ev_type", depart=str(vid * 10),
            **{"from": dedup[0]}, to=dedup[-1], via=" ".join(dedup[1:-1]),
            color=colors[vid % len(colors)],
        )
        seen_edges = {dedup[0]}  # arcos ya usados por una parada anterior en ESTE vehiculo
        for node, edge in zip(route[1:-1], edges[1:-1]):
            # Si esta parada cae en el mismo arco real que la parada inmediatamente
            # anterior (dos nodos EVRP distintos snapeados al mismo arco), un
            # <stop chargingStation=...> ahi queda "detrás" del stop previo para SUMO
            # (posiciones no coherentes) y rompe la ruta. En ese caso se degrada a
            # parada genérica (sin carga real) en vez de bloquear todo el vehículo.
            edge_shared = edge in seen_edges
            seen_edges.add(edge)

            if node in instance["clients"]:
                demand = instance["clients"][node]["demand"]
                duration = max(MIN_CLIENT_STOP_S, demand * SERVICE_TIME_PER_UNIT_S)
                ET.SubElement(trip, "stop", lane=f"{edge}_0", duration=str(duration), parking="true")
                trip.append(ET.Comment(f" cliente {node} (demanda={demand}) "))
            elif node in instance["stations"] and charging_enabled and not edge_shared:
                ET.SubElement(trip, "stop", chargingStation=f"cs_{node}", duration=str(CS_STOP_DURATION_S))
                trip.append(ET.Comment(f" recarga: nodo {node} ({instance['stations'][node]['name']}) "))
                cs_used.add(node)
            else:
                # estacion de carga en pasada de calibracion, o degradada por arco compartido
                reason = "calibracion" if not charging_enabled else "arco compartido con parada anterior"
                ET.SubElement(trip, "stop", lane=f"{edge}_0", duration=str(CS_STOP_DURATION_S), parking="true")
                trip.append(ET.Comment(f" parada sin carga real ({reason}): nodo {node} "))
        written += 1

    _write_pretty_xml(root, out_path)
    print(f"  OK: {written} vehiculo(s), {len(cs_used)} estacion(es) de carga en uso"
          + (f", {len(skipped_routes)} ruta(s) omitida(s): {skipped_routes}" if skipped_routes else ""))
    return cs_used


def write_charging_additional(out_path, instance, snapped, cs_used, net, power_w):
    root = ET.Element("additional")
    for node in sorted(cs_used):
        edge_id = snapped[node]
        edge = net.getEdge(edge_id)
        length = edge.getLength()
        s, e = round(length * 0.1, 2), round(length * 0.9, 2)
        if e - s < 5:
            s, e = 0.0, length
        ET.SubElement(
            root, "chargingStation", id=f"cs_{node}", name=instance["stations"][node]["name"],
            lane=f"{edge_id}_0", startPos=str(s), endPos=str(e),
            # Nombres reales del XSD de esta version de SUMO: 'power' (W) y 'efficiency'
            # (chargePerTimeStep/chargeEfficiency son de versiones viejas de SUMO y se
            # ignoran silenciosamente aqui, por eso no se cargaba nada).
            power=str(round(power_w, 1)), efficiency="1.0", chargeDelay="0",
        )
    _write_pretty_xml(root, out_path)
    print(f"  OK: additional.add.xml con {len(cs_used)} chargingStation(s), potencia={power_w:.0f} W")


# ---------------------------------------------------------------------------
# DUAROUTER (con nombre de salida explícito, para no chocar con routes.rou.xml de la Fase 2)
# ---------------------------------------------------------------------------
def run_duarouter_to(net_file, trips_file, output_path, additional_file=None):
    cmd = [
        find_sumo_tool("duarouter"),
        "--net-file", net_file,
        "--route-files", trips_file,
        "--output-file", output_path,
        "--repair", "--repair.from", "--repair.to",
        "--remove-loops",
        "--ignore-errors",
    ]
    if additional_file:
        # necesario para que duarouter reconozca los <stop chargingStation="..."/>
        cmd += ["--additional-files", additional_file]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        sys.exit("ERROR: duarouter tardo mas de 120s (timeout de seguridad) — revisar antes de reintentar.")
    if not os.path.isfile(output_path):
        print("  ERROR duarouter:\n" + result.stderr[-3000:])
        sys.exit(1)
    print(f"  OK: {os.path.basename(output_path)} generado")
    if result.stderr.strip():
        print("  Advertencias duarouter (informativo):\n" + result.stderr[-1200:])
    return output_path


# ---------------------------------------------------------------------------
# SIMULACIÓN HEADLESS Y LECTURA DE RESULTADOS
# ---------------------------------------------------------------------------
def run_headless(cfg_path, tripinfo_path, extra_args=None):
    cmd = [
        find_sumo_tool("sumo"), "-c", cfg_path,
        "--no-warnings", "true",
        "--tripinfo-output", tripinfo_path,
        "--device.battery.probability", "1.0",
        "--time-to-teleport", "-1",
    ] + (extra_args or [])
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        sys.exit("ERROR: sumo (headless) tardo mas de 180s (timeout de seguridad) — revisar antes de reintentar.")
    if not os.path.isfile(tripinfo_path):
        print("ERROR sumo:\n" + result.stderr[-3000:])
        sys.exit(1)


def read_tripinfo_battery(tripinfo_path):
    """Devuelve {vehicle_id: {distance_km, energy_consumed_wh, energy_charged_wh, final_soc_wh}}."""
    tree = ET.parse(tripinfo_path)
    out = {}
    for t in tree.getroot().findall("tripinfo"):
        vid = t.get("id")
        dist_km = float(t.get("routeLength", 0)) / 1000.0
        b = t.find("battery")
        if b is not None:
            out[vid] = {
                "distance_km": dist_km,
                # Nombres reales en SUMO 1.26: totalEnergyConsumed / totalEnergyCharged
                # (energyConsumed/energyCharged, que usan versiones mas viejas, no existen aqui)
                "energy_consumed_wh": float(b.get("totalEnergyConsumed", 0)),
                "energy_charged_wh": float(b.get("totalEnergyCharged", 0)),
                "final_soc_wh": float(b.get("actualBatteryCapacity", 0)),
            }
    return out


def write_sumocfg(out_dir, net_file, routes_file, additional_file=None):
    cfg = os.path.join(out_dir, "scenario_battery.sumocfg")
    root = ET.Element("configuration")
    inp = ET.SubElement(root, "input")
    ET.SubElement(inp, "net-file", value=os.path.relpath(net_file, out_dir))
    ET.SubElement(inp, "route-files", value=os.path.basename(routes_file))
    if additional_file:
        ET.SubElement(inp, "additional-files", value=os.path.basename(additional_file))
    t = ET.SubElement(root, "time")
    ET.SubElement(t, "begin", value="0")
    pr = ET.SubElement(root, "processing")
    ET.SubElement(pr, "time-to-teleport", value="-1")
    _write_pretty_xml(root, cfg)
    return cfg


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def parse_args():
    p = argparse.ArgumentParser(description="Fase 4: Battery Device real de SUMO calibrado al rango del EVRP")
    p.add_argument("--instance", required=True)
    p.add_argument("--solution", required=True)
    p.add_argument("--net-file", default=os.path.join("simulation", "sumo_network", "network.net.xml"))
    p.add_argument("--output-dir", "-o", default=os.path.join("simulation", "sumo_scenario"))
    return p.parse_args()


def get_evrp_fuel_capacity_km(solution_path):
    with open(solution_path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("Battery capacity:"):
                return float(re.search(r"([\d.]+)", line).group(1))
    sys.exit("ERROR: no se encontro 'Battery capacity:' en la solucion")


def main():
    args = parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    inst = parse_instance(args.instance)
    sol = parse_solution(args.solution)
    fuel_capacity_km = get_evrp_fuel_capacity_km(args.solution)
    print(f"Rango EVRP a preservar (Battery capacity de la solucion): {fuel_capacity_km} km\n")

    net = sumolib.net.readNet(args.net_file)
    snapped = snap_nodes_to_edges(net, inst["node_coords"])

    # ---- Pasada 1: calibración (batería "infinita", sin carga real) ----
    print("== Pasada 1/2: calibracion de consumo real (Wh/km) ==")
    trips_cal = os.path.join(args.output_dir, "trips_battery_calibration.xml")
    build_battery_trips(sol["routes"], snapped, inst, trips_cal, CALIBRATION_CAPACITY_WH, charging_enabled=False)
    routes_cal_renamed = os.path.join(args.output_dir, "routes_calibration.rou.xml")
    run_duarouter_to(args.net_file, trips_cal, routes_cal_renamed)
    cfg_cal = write_sumocfg(args.output_dir, args.net_file, routes_cal_renamed)
    tripinfo_cal = os.path.join(args.output_dir, "tripinfo_calibration.xml")
    run_headless(cfg_cal, tripinfo_cal)
    cal_data = read_tripinfo_battery(tripinfo_cal)

    total_wh = sum(v["energy_consumed_wh"] for v in cal_data.values())
    total_km = sum(v["distance_km"] for v in cal_data.values())
    if total_km == 0:
        sys.exit("ERROR: 0 km recorridos en la calibracion, no se puede calibrar la bateria")
    wh_per_km = total_wh / total_km
    battery_capacity_wh = wh_per_km * fuel_capacity_km
    print(f"  Consumo real medido: {wh_per_km:.1f} Wh/km  (sobre {total_km:.1f} km, {len(cal_data)} vehiculos)")
    print(f"  Capacidad de bateria calibrada para {fuel_capacity_km} km de rango: {battery_capacity_wh:.0f} Wh\n")

    # ---- Pasada 2: escenario final (bateria calibrada + carga real en CS) ----
    print("== Pasada 2/2: escenario final con carga real en estaciones ==")
    charge_power_w = CHARGE_POWER_W  # potencia fija realista (150 kW), no calibrada a la capacidad
    trips_final = os.path.join(args.output_dir, "trips_battery.xml")
    cs_used = build_battery_trips(sol["routes"], snapped, inst, trips_final, battery_capacity_wh, charging_enabled=True)

    # El additional.add.xml con las chargingStation debe existir ANTES de duarouter:
    # necesita conocerlas para poder resolver los <stop chargingStation="cs_..">.
    additional_file = None
    if cs_used:
        additional_file = os.path.join(args.output_dir, "additional_battery.add.xml")
        write_charging_additional(additional_file, inst, snapped, cs_used, net, charge_power_w)

    routes_final_renamed = os.path.join(args.output_dir, "routes_battery.rou.xml")
    run_duarouter_to(args.net_file, trips_final, routes_final_renamed, additional_file)

    cfg_final = write_sumocfg(args.output_dir, args.net_file, routes_final_renamed, additional_file)
    tripinfo_final = os.path.join(args.output_dir, "tripinfo_battery.xml")
    run_headless(cfg_final, tripinfo_final)
    final_data = read_tripinfo_battery(tripinfo_final)

    print(f"\n{'='*78}\n  FASE 4 — Resultado del Battery Device (fisica real vs. tasa lineal OR-Tools)\n{'='*78}")
    print(f"  Capacidad calibrada : {battery_capacity_wh:.0f} Wh  (<-> {fuel_capacity_km} km del EVRP)")
    print(f"  Tasa OR-Tools       : 1 unidad/km ({fuel_capacity_km} unidades = {fuel_capacity_km} km de rango)")
    print(f"  Tasa real (SUMO)    : {wh_per_km:.1f} Wh/km (fisica newtoniana, no lineal)\n")
    # tripinfo no expone "energia cargada" directamente en esta version de SUMO;
    # se infiere: lo que hubiera quedado sin cargar (capacidad-consumido) vs lo real.
    print(f"  {'Vehiculo':<10}{'Dist(km)':>10}{'Consumido(Wh)':>16}{'Cargado~(Wh)':>14}{'SOC final(Wh)':>16}{'SOC final(%)':>14}")
    for vid, d in sorted(final_data.items()):
        would_be_wh = battery_capacity_wh - d["energy_consumed_wh"]
        charged_inferred = max(0.0, d["final_soc_wh"] - would_be_wh)
        soc_pct = d["final_soc_wh"] / battery_capacity_wh * 100
        flag = "  (*)" if soc_pct > 100 else ""
        print(f"  {vid:<10}{d['distance_km']:>10.1f}{d['energy_consumed_wh']:>16.1f}"
              f"{charged_inferred:>14.1f}{d['final_soc_wh']:>16.1f}{min(soc_pct, 999.9):>13.1f}%{flag}")
    print(f"{'='*78}")
    print("  (*) SOC > 100%: la parada de carga (duracion fija) siguio cargando tras\n"
          "      llegar al 100% — limitacion conocida de esta version sin TraCI.")
    print(f"\nListo. Para visualizar con bateria activa:\n  sumo-gui -c {cfg_final}")


if __name__ == "__main__":
    main()
