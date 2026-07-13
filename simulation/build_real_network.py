"""
build_real_network.py — FASE 1: construye la red vial real de SUMO a partir
del .osm.pbf original (el mismo dato base usado para OSRM), sin pasar por los
binarios procesados de OSRM (.osrm*), que son un formato propio incompatible
con SUMO.

Uso:
    python simulation/build_real_network.py

Por decisión explícita, NO se recorta el .osm.pbf por instancia: se procesa
la provincia completa de Quebec una sola vez (con netconvert) y el resultado
(network.net.xml) queda disponible para cualquier instancia cuyas coordenadas
caigan dentro de esa región. Es un costo único, no por instancia.

Usa el typemap estándar que trae SUMO (osmNetconvert.typ.xml) para quedarnos
solo con vías transitables por vehículos motorizados (descarta edificios,
uso de suelo, límites administrativos, etc. que trae el .osm.pbf crudo).
"""
import argparse
import os
import subprocess
import sys
import time


def find_sumo_tool(name):
    """Ubica un binario de SUMO usando SUMO_HOME; usa 'name' vía PATH como respaldo."""
    sumo_home = os.environ.get("SUMO_HOME", "")
    if sumo_home:
        candidate = os.path.join(sumo_home, "bin", name)
        for ext in ("", ".exe"):
            if os.path.isfile(candidate + ext):
                return candidate + ext
    return name


def parse_args():
    p = argparse.ArgumentParser(
        description="Fase 1: construir la red vial real de SUMO desde el .osm.pbf original"
    )
    p.add_argument(
        "--osm-file", default=os.path.join("osrm_data", "quebec-latest.osm.pbf"),
        help="Extracto OSM crudo (el mismo usado para OSRM) — NO los .osrm* procesados"
    )
    p.add_argument("--output-dir", "-o", default=os.path.join("simulation", "sumo_network"))
    p.add_argument(
        "--typemap", default=None,
        help="Typemap de netconvert (por defecto: SUMO_HOME/data/typemap/osmNetconvert.typ.xml)"
    )
    return p.parse_args()


def build_network(osm_file, output_dir, typemap):
    os.makedirs(output_dir, exist_ok=True)
    net_out = os.path.join(output_dir, "network.net.xml")

    netconvert = find_sumo_tool("netconvert")
    if typemap is None:
        sumo_home = os.environ.get("SUMO_HOME", "")
        typemap = os.path.join(sumo_home, "data", "typemap", "osmNetconvert.typ.xml")

    cmd = [
        netconvert,
        "--osm-files", osm_file,
        "--output-file", net_out,
        "--type-files", typemap,
        # Limpieza estándar recomendada por SUMO para redes importadas de OSM
        "--geometry.remove",
        "--roundabouts.guess",
        "--ramps.guess",
        "--junctions.join",
        "--tls.guess-signals",
        "--tls.discard-simple",
        "--tls.join",
        "--no-turnarounds.tls",
        "--remove-edges.isolated",
    ]

    print(f"OSM origen : {osm_file} ({os.path.getsize(osm_file) / 1e6:.0f} MB)")
    print(f"Typemap    : {typemap}")
    print(f"Salida     : {net_out}")
    print("Corriendo netconvert sobre la provincia completa (sin recorte) — puede tardar varios minutos...\n")

    t0 = time.time()
    result = subprocess.run(cmd, capture_output=True, text=True)
    elapsed = time.time() - t0

    if result.returncode != 0 or not os.path.isfile(net_out):
        print("ERROR netconvert:")
        print(result.stderr[-3000:])
        sys.exit(1)

    size_mb = os.path.getsize(net_out) / 1e6
    print(f"\nOK: network.net.xml generado en {elapsed:.1f}s ({size_mb:.1f} MB)")
    if result.stderr.strip():
        print("\nAdvertencias de netconvert (informativo, no bloquean):")
        print(result.stderr[-2000:])


def main():
    args = parse_args()
    if not os.path.isfile(args.osm_file):
        sys.exit(f"ERROR: no se encontró {args.osm_file}")
    build_network(args.osm_file, args.output_dir, args.typemap)


if __name__ == "__main__":
    main()
