"""
run.py — menu interactivo de consola para la simulacion SUMO del EVRP.

En vez de recordar/escribir los comandos de cada fase con todos sus flags,
corre:

    python simulation/run.py

y elige la opcion por numero. Cada opcion invoca el script correspondiente
(exactamente los mismos comandos documentados en simulation/README.md) y
pide solo los parametros que de verdad cambian entre corridas (instancia,
solucion, checkpoint), con un valor por defecto pre-cargado (la instancia
piloto usada en todas las pruebas) para poder aceptar con solo Enter.

No reemplaza a los scripts individuales -- son un espejo de ellos, uno por
uno, para no tener que teclear rutas largas cada vez.
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_OSM_FILE = os.path.join(REPO_ROOT, "osrm_data", "quebec-latest.osm.pbf")
DEFAULT_CLIPPED_OSM = os.path.join(REPO_ROOT, "simulation", "sumo_network", "clipped.osm")
DEFAULT_NET_FILE = os.path.join(REPO_ROOT, "simulation", "sumo_network", "network.net.xml")
DEFAULT_INSTANCE = os.path.join(REPO_ROOT, "instances_data", "evrp_instances", "quebec_40c_4ev_6cs.txt")
DEFAULT_SOLUTION = os.path.join(
    REPO_ROOT, "problem", "osrm", "solutions_evrp_0",
    "solutions_PATH_CHEAPEST_ARC", "quebec_40c_4ev_6cs.txt"
)
DEFAULT_BATTERY_CFG = os.path.join(REPO_ROOT, "simulation", "sumo_scenario", "scenario_battery.sumocfg")

SCENARIOS = {
    "1": ("scenario.sumocfg (Fase 2 - rutas basicas)",
          os.path.join(REPO_ROOT, "simulation", "sumo_scenario", "scenario.sumocfg")),
    "2": ("scenario_battery.sumocfg (Fase 4 - bateria real)",
          DEFAULT_BATTERY_CFG),
    "3": ("scenario_dynamic_replan.sumocfg (EVRP dinamico - ultimo plan re-optimizado)",
          os.path.join(REPO_ROOT, "simulation", "sumo_scenario", "scenario_dynamic_replan.sumocfg")),
}


def ask(label, default):
    suffix = f" [{default}]" if default not in (None, "") else ""
    val = input(f"{label}{suffix}: ").strip()
    return val if val else default


def ask_float(label, default):
    val = ask(label, str(default))
    try:
        return float(val)
    except ValueError:
        print(f"  (valor invalido, uso {default})")
        return default


def run_script(script_name, args):
    script_path = os.path.join(REPO_ROOT, "simulation", script_name)
    cmd = [sys.executable, script_path] + args
    print(f"\n$ {' '.join(cmd)}\n")
    result = subprocess.run(cmd, cwd=REPO_ROOT)
    if result.returncode != 0:
        print(f"\n[{script_name} termino con error, codigo {result.returncode}]")
    else:
        print(f"\n[{script_name} termino OK]")
    input("\nEnter para volver al menu...")


def opt_fase1():
    print("\n--- Fase 1: red vial real (OSM -> SUMO) ---")
    if not os.path.isfile(DEFAULT_OSM_FILE):
        print(f"AVISO: no se encontro {DEFAULT_OSM_FILE}")
        print("Descargalo antes (ver README, Fase 1).")
        input("\nEnter para volver al menu...")
        return
    instance = ask("Instancia para derivar el bbox", DEFAULT_INSTANCE)
    run_script("extract_osm_bbox.py", ["--osm-file", DEFAULT_OSM_FILE, "--instance", instance])
    run_script("build_real_network.py", ["--osm-file", DEFAULT_CLIPPED_OSM])


def opt_fase2():
    print("\n--- Fase 2: rutas EVRP sobre la red real ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    run_script("build_routes.py", ["--instance", instance, "--solution", solution])


def opt_fase3():
    print("\n--- Fase 3: trafico de fondo (PENDIENTE, no se ha logrado correr con exito) ---")
    print("Ver README: se colgo ~10h en el intento anterior. Se recomienda probar")
    print("primero con una ventana corta antes de escalar.")
    confirm = ask("Continuar de todas formas? (s/n)", "n")
    if confirm.lower() != "s":
        return
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    end = ask("--end (ventana de insercion, s)", "600")
    period = ask("--period (s promedio entre inserciones)", "5")
    run_script("build_background_traffic.py", ["--solution", solution, "--end", end, "--period", period])


def opt_fase4():
    print("\n--- Fase 4: escenario de bateria real ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    run_script("build_battery_scenario.py", ["--instance", instance, "--solution", solution])


def opt_dynamic():
    print("\n--- EVRP dinamico: orquestador de re-planeacion (Punto 4) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("dynamic_replanning.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
    ])


def opt_probe():
    print("\n--- [Diagnostico] Estado de vehiculos en un checkpoint (Punto 2) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("traci_state_probe.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
    ])


def opt_congestion():
    print("\n--- [Diagnostico] Evento de congestion aislado (Punto 3) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("traci_congestion_trigger.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
    ])


def opt_gui():
    print("\n--- Abrir un escenario en sumo-gui ---")
    for key, (label, _) in SCENARIOS.items():
        print(f"  {key}) {label}")
    choice = ask("Cual escenario", "2")
    entry = SCENARIOS.get(choice)
    if entry is None:
        print("Opcion invalida.")
        return
    label, path = entry
    if not os.path.isfile(path):
        print(f"AVISO: no existe {path} todavia -- corre la fase correspondiente primero.")
        input("\nEnter para volver al menu...")
        return
    print(f"\nAbriendo sumo-gui con {label}...")
    subprocess.Popen(["sumo-gui", "-c", path], cwd=REPO_ROOT)


MENU = [
    ("1", "Fase 1 - Construir red vial real (OSM -> SUMO)", opt_fase1),
    ("2", "Fase 2 - Generar rutas EVRP sobre la red real", opt_fase2),
    ("3", "Fase 3 - Trafico de fondo (PENDIENTE, no recomendado)", opt_fase3),
    ("4", "Fase 4 - Escenario de bateria real", opt_fase4),
    ("5", "EVRP dinamico - re-planeacion (Punto 4, orquestador completo)", opt_dynamic),
    ("6", "[Diagnostico] Ver estado de vehiculos en un checkpoint", opt_probe),
    ("7", "[Diagnostico] Evento de congestion aislado", opt_congestion),
    ("8", "Abrir un escenario en sumo-gui", opt_gui),
]


def main():
    while True:
        print("\n" + "=" * 60)
        print("  Simulacion SUMO del EVRP -- menu")
        print("=" * 60)
        for key, label, _ in MENU:
            print(f"  {key}) {label}")
        print("  0) Salir")
        choice = input("\nElige una opcion: ").strip()
        if choice == "0":
            break
        match = next((fn for key, _, fn in MENU if key == choice), None)
        if match is None:
            print("Opcion invalida.")
            continue
        try:
            match()
        except KeyboardInterrupt:
            print("\nInterrumpido.")
        except Exception as exc:
            print(f"\nError inesperado: {exc}")
            input("\nEnter para volver al menu...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSaliendo.")
