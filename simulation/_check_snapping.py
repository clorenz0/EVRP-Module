"""Diagnostico rapido (sin duarouter/sumo): para una instancia+solucion+red,
verifica cuantos vehiculos quedarian fuera al construir los trips (snapping
fallido o ruta degenerada a un solo arco), sin correr el pipeline completo."""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.environ.get("SUMO_HOME", r"D:\SUMO"), "tools"))
import sumolib  # noqa: E402
from build_routes import parse_instance, parse_solution, snap_route_sequential  # noqa: E402

instance_path = sys.argv[1]
solution_path = sys.argv[2]
net_path = sys.argv[3]

inst = parse_instance(instance_path)
sol = parse_solution(solution_path)
net = sumolib.net.readNet(net_path)

print(f"Instancia: {instance_path}")
print(f"Vehiculos en la solucion: {len(sol['routes'])}")

skipped = []
for vid, route in enumerate(sol["routes"]):
    snapped = snap_route_sequential(net, route, inst["node_coords"])
    if snapped is None:
        skipped.append((vid, "snapping fallido (sin conexion real entre paradas)"))

if skipped:
    print(f"OMITIDOS: {len(skipped)}/{len(sol['routes'])}")
    for vid, reason in skipped:
        print(f"  vehiculo {vid}: {reason}")
else:
    print(f"OK: los {len(sol['routes'])} vehiculos se pueden construir sin omisiones")
