"""
test_sumo.py — Script de prueba para simulación SUMO del escenario EVRP
Compatible con SUMO ≥1.14. Detecta automáticamente los parámetros de batería.
"""
import argparse
import json
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import pandas as pd

try:
    import traci
except ImportError:
    sys.exit("ERROR: 'traci' no encontrado. pip install traci")

# ==============================================================================
# 1. ARGUMENTOS
# ==============================================================================
def parse_args():
    p = argparse.ArgumentParser(description="Ejecuta simulación EVRP en SUMO")
    p.add_argument("--config",       "-c", required=True)
    p.add_argument("--gui",          action="store_true")
    p.add_argument("--output-dir",   "-o", default="sumo_outputs")
    p.add_argument("--step-length",  type=float, default=1.0)
    p.add_argument("--max-steps",    type=int,   default=100_000)
    p.add_argument("--soc-interval", type=int,   default=10)
    return p.parse_args()

# ==============================================================================
# 2. PREPARAR SALIDAS
# ==============================================================================
def prepare_paths(out_dir: str) -> dict:
    Path(out_dir).mkdir(parents=True, exist_ok=True)
    return {
        "tripinfo":         os.path.join(out_dir, "tripinfo.xml"),
        "battery":          os.path.join(out_dir, "battery.xml"),
        "emissions":        os.path.join(out_dir, "emissions.xml"),
        "chargingstations": os.path.join(out_dir, "chargingstations.xml"),
        "summary_csv":      os.path.join(out_dir, "evrp_summary.csv"),
        "charging_csv":     os.path.join(out_dir, "charging_events.csv"),
        "soc_csv":          os.path.join(out_dir, "soc_timeseries.csv"),
        "metadata_json":    os.path.join(out_dir, "evrp_metadata.json"),
    }

# ==============================================================================
# 3. DETECCIÓN DE PARÁMETROS DE BATERÍA (con fallbacks por versión de SUMO)
# ==============================================================================
def _get_battery_param(veh_id: str, traci_module):
    """
    Prueba nombres canónicos y aliases hasta encontrar el correcto para
    esta versión de SUMO.
    """
    for key in [
        "device.battery.actualBatteryCapacity",  # nombre canónico ≥1.14
        "device.battery.charge",                  # alias en algunas versiones
        "device.battery.energyLevel",
    ]:
        try:
            val = traci_module.vehicle.getParameter(veh_id, key)
            return float(val)
        except traci_module.TraCIException:
            continue
    return None

def _get_battery_capacity(veh_id: str, traci_module):
    for key in [
        "device.battery.maximumBatteryCapacity",  # nombre canónico ≥1.14
        "device.battery.capacity",
    ]:
        try:
            return float(traci_module.vehicle.getParameter(veh_id, key))
        except traci_module.TraCIException:
            continue
    return None

# ==============================================================================
# 4. SIMULACIÓN
# ==============================================================================
def run_simulation(args, paths: dict):
    sumo_bin = "sumo-gui" if args.gui else "sumo"
    if os.environ.get("SUMO_HOME"):
        cand = os.path.join(os.environ["SUMO_HOME"], "bin", sumo_bin)
        if os.path.isfile(cand) or os.path.isfile(cand + ".exe"):
            sumo_bin = cand

    cmd = [
        sumo_bin, "--configuration-file", args.config,
        "--step-length", str(args.step_length),
        "--no-warnings", "true",
        # Forzar activación del dispositivo de batería para todos los vehículos
        # (respaldo por si el param has.battery.device del vType no es procesado).
        "--device.battery.probability", "1.0",
        # Outputs
        "--tripinfo-output",         paths["tripinfo"],
        "--battery-output",          paths["battery"],
        "--emission-output",         paths["emissions"],
        "--chargingstations-output", paths["chargingstations"],
        "--vehroute-output",         os.path.join(args.output_dir, "vehroutes.xml"),
        "--fcd-output",              os.path.join(args.output_dir, "fcd.xml"),
        "--collision.action", "warn",
        "--time-to-teleport", "-1",
    ]
    print(f"\n▶ Iniciando SUMO: {os.path.basename(sumo_bin)}")
    print(f"  --device.battery.probability = 1.0  (batería activa en todos los vehículos)\n")
    traci.start(cmd)

    soc_records  = []
    stop_records = []   # registro de paradas (cliente + CS)
    step         = 0
    active        = set()

    # Para detectar inicio/fin de paradas por vehículo
    stop_state = {}     # vid -> {"start": step, "lane": str, "cs": str|None}

    try:
        while step < args.max_steps:
            traci.simulationStep()
            step += 1
            current = set(traci.vehicle.getIDList())
            new     = current - active
            active  = current

            if new:
                print(f"  [paso {step:5d}] Nuevos vehículos: {new}")
                for vid in new:
                    stop_state[vid] = None

            # Registrar SOC periódicamente
            if step % args.soc_interval == 0 and current:
                for vid in current:
                    actual = _get_battery_param(vid, traci)
                    cap    = _get_battery_capacity(vid, traci)
                    if actual is not None and cap is not None and cap > 0:
                        soc_records.append({
                            "step":       step,
                            "time_s":     round(step * args.step_length, 2),
                            "vehicle_id": vid,
                            "soc_wh":     round(actual, 4),
                            "soc_pct":    round(actual / cap * 100, 2),
                        })

            # Detectar paradas (cliente o CS) en curso
            for vid in current:
                try:
                    stop_count = traci.vehicle.getStopState(vid)
                    is_stopped = bool(stop_count & 1)  # bit 0 = detenido
                    cs_id      = traci.vehicle.getParameter(vid, "chargingStation") \
                                 if is_stopped else ""
                except traci.TraCIException:
                    is_stopped, cs_id = False, ""

                prev = stop_state.get(vid)
                if is_stopped and prev is None:
                    stop_state[vid] = {"start": step, "cs": cs_id or None}
                elif not is_stopped and prev is not None:
                    dur = (step - prev["start"]) * args.step_length
                    stop_records.append({
                        "vehicle_id":  vid,
                        "start_step":  prev["start"],
                        "end_step":    step,
                        "duration_s":  round(dur, 1),
                        "is_cs_stop":  prev["cs"] is not None,
                        "cs_id":       prev["cs"] or "",
                    })
                    stop_state[vid] = None

            if traci.simulation.getMinExpectedNumber() == 0 and not active:
                print(f"\n✓ Simulación completada en paso {step} "
                      f"({round(step * args.step_length, 1)} s sim)")
                break

    except traci.TraCIException as e:
        print(f"\n⚠ Error TraCI en paso {step}: {e}")
    finally:
        traci.close()

    return soc_records, stop_records

# ==============================================================================
# 5. PARSEO XML ROBUSTO
# ==============================================================================
def _safe(v, d=0.0):
    try:    return float(v)
    except: return d

def parse_tripinfo(p):
    if not os.path.isfile(p): return pd.DataFrame()
    tree = ET.parse(p)
    recs = []
    for t in tree.getroot().findall("tripinfo"):
        row = {
            "vehicle_id":     t.get("id"),
            "route_length_m": _safe(t.get("routeLength")),
            "duration_s":     _safe(t.get("duration")),
            "waiting_time_s": _safe(t.get("waitingTime")),
            "vaporized":      t.get("vaporized", "False"),
        }
        # En SUMO ≥1.15 la energía va en subelemento <battery>
        b = t.find("battery")
        if b is not None:
            row["energy_consumed_wh"] = _safe(b.get("energyConsumed"))
            row["energy_charged_wh"]  = _safe(b.get("energyCharged"))
            row["final_soc_wh"]       = _safe(b.get("actualBatteryCapacity"))
        else:
            row["energy_consumed_wh"] = _safe(t.get("energyConsumed"))
            row["energy_charged_wh"]  = _safe(t.get("energyCharged"))
            row["final_soc_wh"]       = 0.0
        d_km = row["route_length_m"] / 1000.0
        row["consumption_wh_per_km"] = round(
            row["energy_consumed_wh"] / d_km, 4) if d_km > 0 else 0
        row["completed"] = row["vaporized"] not in ("True", "1", True)
        recs.append(row)
    return pd.DataFrame(recs)

def parse_battery(p):
    if not os.path.isfile(p): return pd.DataFrame()
    tree = ET.parse(p)
    recs = []
    for ts in tree.getroot().findall("timestep"):
        t_s = _safe(ts.get("time"))
        for v in ts.findall("vehicle"):
            # SUMO exporta 'actualBatteryCapacity' (canónico) o 'charge' (alias)
            act = v.get("actualBatteryCapacity") or v.get("charge")
            cap = v.get("maximumBatteryCapacity") or v.get("capacity")
            recs.append({
                "time_s":         t_s,
                "vehicle_id":     v.get("id"),
                "actual_cap_wh":  _safe(act),
                "max_cap_wh":     _safe(cap),
                "consumption_wh": _safe(v.get("Consum")),
                "charging_flag":  v.get("chargingStationId", ""),
            })
    df = pd.DataFrame(recs)
    if not df.empty:
        df["soc_pct"] = (df["actual_cap_wh"] / df["max_cap_wh"].replace(0, 1) * 100).round(2)
    return df

def parse_charging(p):
    if not os.path.isfile(p): return pd.DataFrame()
    tree = ET.parse(p)
    recs = []
    for cs in tree.getroot().findall("chargingstation"):
        sid = cs.get("id")
        for v in cs.findall("vehicle"):
            recs.append({
                "station_id":        sid,
                "vehicle_id":        v.get("id"),
                "charging_start_s":  _safe(v.get("chargingBegin")),
                "charging_end_s":    _safe(v.get("chargingEnd")),
                "energy_charged_wh": _safe(v.get("totalEnergyCharged")),
            })
    df = pd.DataFrame(recs)
    if not df.empty:
        df["duration_s"] = df["charging_end_s"] - df["charging_start_s"]
    return df

# ==============================================================================
# 6. GUARDAR Y RESUMEN
# ==============================================================================
def save_results(paths, tri, chg, soc, stops, meta, out_dir):
    if not tri.empty:
        tri.to_csv(paths["summary_csv"], index=False)
        print(f"  ✓ evrp_summary.csv")
    if not chg.empty:
        chg.to_csv(paths["charging_csv"], index=False)
        print(f"  ✓ charging_events.csv  ({len(chg)} eventos)")
    else:
        print("  ⚠ Sin eventos de carga registrados en chargingstations.xml")
    if not soc.empty:
        soc.to_csv(paths["soc_csv"], index=False)
        print(f"  ✓ soc_timeseries.csv")
    if stops:
        pd.DataFrame(stops).to_csv(os.path.join(out_dir, "stops.csv"), index=False)
        print(f"  ✓ stops.csv  ({len(stops)} paradas detectadas por TraCI)")
    if meta:
        with open(paths["metadata_json"], "w") as f:
            json.dump(meta, f, indent=2)
        print(f"  ✓ evrp_metadata.json")

def print_summary(meta, tri):
    print("\n" + "="*60 + "\n  RESUMEN EVRP — SIMULACIÓN SUMO\n" + "="*60)
    if not meta:
        print("  Sin datos de resumen."); return
    print(f"  Vehículos totales      : {meta['total_vehicles']}")
    print(f"  Completaron ruta       : {meta['completed_vehicles']}/{meta['total_vehicles']}")
    print(f"  Distancia total        : {meta['total_distance_km']:.2f} km")
    print(f"  Energía consumida      : {meta['total_energy_consumed_wh']:.2f} Wh")
    print(f"  Energía cargada en CS  : {meta['total_energy_charged_wh']:.2f} Wh")
    print(f"  Consumo medio          : {meta['avg_consumption_wh_per_km']:.2f} Wh/km")
    if not tri.empty:
        print("\n  Detalle por vehículo:")
        cols = [c for c in ["vehicle_id","route_length_m","energy_consumed_wh",
                             "final_soc_wh","completed"] if c in tri.columns]
        print(tri[cols].to_string(index=False))
    print("\n" + "="*60)

# ==============================================================================
# 7. MAIN
# ==============================================================================
def main():
    args  = parse_args()
    paths = prepare_paths(args.output_dir)
    if not os.path.isfile(args.config):
        sys.exit(f"ERROR: config '{args.config}' no existe")

    print(f"  EVRP — Test de Simulación SUMO\n  Config : {args.config}")
    t0           = time.time()
    soc, stops   = run_simulation(args, paths)
    print(f"\n  Tiempo total: {round(time.time() - t0, 2)} s")

    tri    = parse_tripinfo(paths["tripinfo"])
    bat    = parse_battery(paths["battery"])
    chg    = parse_charging(paths["chargingstations"])
    soc_df = bat if not bat.empty else pd.DataFrame(soc)
    if not soc_df.empty and "actual_cap_wh" in soc_df.columns:
        soc_df = soc_df.rename(columns={"actual_cap_wh": "soc_wh"})

    meta = {}
    if not tri.empty:
        total_dist_km = tri["route_length_m"].sum() / 1000.0
        meta = {
            "total_vehicles":           len(tri),
            "completed_vehicles":       int(tri["completed"].sum()),
            "total_distance_km":        round(total_dist_km, 2),
            "total_energy_consumed_wh": round(tri["energy_consumed_wh"].sum(), 2),
            "total_energy_charged_wh":  round(
                chg["energy_charged_wh"].sum() if not chg.empty else 0, 2),
            "avg_consumption_wh_per_km": round(
                tri["energy_consumed_wh"].sum() / total_dist_km
                if total_dist_km > 0 else 0, 2),
        }

    save_results(paths, tri, chg, soc_df, stops, meta, args.output_dir)
    print_summary(meta, tri)

if __name__ == "__main__":
    main()