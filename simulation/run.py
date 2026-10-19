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

# Las 5 instancias piloto NO estan todas en la misma ciudad (ver README): cada
# una necesita la red/config de SU zona, si no TraCI falla con arcos "None"
# (coordenadas que no existen en la red cargada). Instancia -> red.net.xml.
NET_QUEBEC = os.path.join(REPO_ROOT, "simulation", "sumo_network", "network.net.xml")
NET_MONTREAL = os.path.join(REPO_ROOT, "simulation", "sumo_network_montreal", "network.net.xml")
NET_TROISRIVIERES = os.path.join(REPO_ROOT, "simulation", "sumo_network_troisrivieres", "network.net.xml")
INSTANCE_NET_FILE = {
    "quebec_40c_4ev_6cs": NET_QUEBEC,
    "quebec_60c_8ev_8cs": NET_QUEBEC,
    "quebec_30c_6ev_4cs": NET_MONTREAL,
    "quebec_50c_5ev_5cs": NET_TROISRIVIERES,
    "quebec_90c_10ev_10cs": NET_TROISRIVIERES,
    # instancias grandes (70-350 clientes), mismas tres redes
    "quebec_70c_9ev_9cs": NET_MONTREAL,
    "quebec_100c_12ev_12cs": NET_QUEBEC,
    "quebec_150c_18ev_15cs": NET_TROISRIVIERES,
    "quebec_220c_26ev_20cs": NET_QUEBEC,
    "quebec_350c_40ev_30cs": NET_MONTREAL,
}


def instance_base(instance_path):
    return os.path.splitext(os.path.basename(instance_path))[0]


def net_file_for(instance_path):
    base = instance_base(instance_path)
    net_file = INSTANCE_NET_FILE.get(base)
    if net_file is None:
        print(f"  AVISO: no se reconoce la zona de '{base}' -- usando la red de Quebec City por defecto "
              f"(si la instancia es de otra zona, esto va a fallar).")
        return NET_QUEBEC
    return net_file


def algo_from_solution(solution_path):
    """
    Extrae el nombre del algoritmo (ej. 'GUIDED_LOCAL_SEARCH') de la carpeta
    'solutions_<algo>' que contiene el archivo de solucion.

    Por que hace falta: las distintas combinaciones de heuristica/metaheuristica
    para la MISMA instancia asignan rutas completamente distintas a cada
    vehiculo (ev_0 en PATH_CHEAPEST_ARC puede ser ev_1 en GUIDED_LOCAL_SEARCH).
    Si Fase 2/4 se construyen una sola vez por instancia (sin distinguir
    algoritmo) y despues se reusan para otra solucion, el escenario de SUMO
    queda construido con las rutas de UN algoritmo mientras dynamic_replanning.py
    cree que esta siguiendo las de OTRO — paradas contadas mal, vehiculos que
    "no existen" que en realidad existen con otro indice, etc. (bug real
    encontrado corriendo quebec_60c con PATH_CHEAPEST_ARC vs GUIDED_LOCAL_SEARCH).
    """
    parent = os.path.basename(os.path.dirname(solution_path))
    if parent.startswith("solutions_"):
        return parent[len("solutions_"):]
    return "default"


def output_dir_for(instance_path, solution_path=None):
    """Carpeta de escenario SUMO para esta instancia -- por algoritmo si se
    da `solution_path` (recomendado siempre que la fase dependa de QUE rutas
    tiene cada vehiculo: Fase 2, Fase 4, re-planeacion dinamica, diagnosticos)."""
    parts = [REPO_ROOT, "simulation", "sumo_scenario", instance_base(instance_path)]
    if solution_path:
        parts.append(algo_from_solution(solution_path))
    return os.path.join(*parts)


def battery_cfg_for(instance_path, solution_path):
    """
    Config de Fase 4 de ESTA instancia+algoritmo. Si todavia no se genero
    para esa combinacion especifica, cae al viejo escenario por-instancia
    (sin distinguir algoritmo, compatibilidad con corridas previas a este
    fix) y avisa, porque probablemente corresponde a OTRO algoritmo.
    """
    per_combo = os.path.join(output_dir_for(instance_path, solution_path), "scenario_battery.sumocfg")
    if os.path.isfile(per_combo):
        return per_combo
    legacy = os.path.join(output_dir_for(instance_path), "scenario_battery.sumocfg")
    if os.path.isfile(legacy):
        print(f"  AVISO: no existe Fase 4 para '{algo_from_solution(solution_path)}' todavia -- "
              f"usando el escenario generico de la instancia (puede ser de OTRO algoritmo). "
              f"Corre la opcion 4 con esta solucion para generar el correcto.")
        return legacy
    return DEFAULT_BATTERY_CFG

SCENARIO_FILES = {
    "1": "scenario.sumocfg (Fase 2 - rutas basicas)",
    "2": "scenario_battery.sumocfg (Fase 4 - bateria real)",
    "3": "scenario_dynamic_replan.sumocfg (EVRP dinamico - ultimo plan re-optimizado)",
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
    run_script("build_routes.py", [
        "--instance", instance, "--solution", solution,
        "--net-file", net_file_for(instance), "--output-dir", output_dir_for(instance, solution),
    ])


def opt_fase3():
    print("\n--- Fase 3: trafico de fondo ---")
    print("Nota (ver README): a esta escala el trafico de fondo aleatorio no genera")
    print("congestion detectable en las rutas de los EV (muy disperso sobre la red).")
    print("Para demostrar congestion real, usa la opcion 5 (EVRP dinamico) o la 7.")
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    end = ask("--end (ventana de insercion, s; ~end/period vehiculos)", "3000")
    period = ask("--period (s promedio entre inserciones)", "2")
    run_script("build_background_traffic.py", ["--solution", solution, "--end", end, "--period", period])


def opt_fase4():
    print("\n--- Fase 4: escenario de bateria real ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    run_script("build_battery_scenario.py", [
        "--instance", instance, "--solution", solution,
        "--net-file", net_file_for(instance), "--output-dir", output_dir_for(instance, solution),
    ])


def opt_dynamic():
    print("\n--- EVRP dinamico: orquestador de re-planeacion (Punto 4) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("dynamic_replanning.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
        "--config", battery_cfg_for(instance, solution), "--net-file", net_file_for(instance),
    ])


def opt_probe():
    print("\n--- [Diagnostico] Estado de vehiculos en un checkpoint (Punto 2) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("traci_state_probe.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
        "--config", battery_cfg_for(instance, solution),
    ])


def opt_congestion():
    print("\n--- [Diagnostico] Evento de congestion aislado (Punto 3) ---")
    instance = ask("Instancia (.txt)", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt)", DEFAULT_SOLUTION)
    checkpoint = ask_float("Checkpoint (s)", 1200.0)
    run_script("traci_congestion_trigger.py", [
        "--instance", instance, "--solution", solution,
        "--checkpoint-time", str(checkpoint),
        "--config", battery_cfg_for(instance, solution), "--net-file", net_file_for(instance),
    ])


def opt_gui():
    print("\n--- Abrir un escenario en sumo-gui ---")
    instance = ask("Instancia (.txt) a la que pertenece el escenario", DEFAULT_INSTANCE)
    solution = ask("Solucion (.txt) -- define la carpeta por algoritmo", DEFAULT_SOLUTION)
    out_dir = output_dir_for(instance, solution)
    for key, label in SCENARIO_FILES.items():
        print(f"  {key}) {label}")
    choice = ask("Cual escenario", "2")
    label = SCENARIO_FILES.get(choice)
    if label is None:
        print("Opcion invalida.")
        return
    filename = label.split(" ", 1)[0]
    if choice == "3":
        out_dir = os.path.join(out_dir, f"replanificacion_{algo_from_solution(solution)}")
    path = os.path.join(out_dir, filename)
    if not os.path.isfile(path):
        print(f"AVISO: no existe {path} todavia -- corre la fase correspondiente primero.")
        input("\nEnter para volver al menu...")
        return
    print(f"\nAbriendo sumo-gui con {label}...")
    subprocess.Popen(["sumo-gui", "-c", path], cwd=REPO_ROOT)


MENU = [
    ("1", "Fase 1 - Construir red vial real (OSM -> SUMO)", opt_fase1),
    ("2", "Fase 2 - Generar rutas EVRP sobre la red real", opt_fase2),
    ("3", "Fase 3 - Trafico de fondo", opt_fase3),
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
