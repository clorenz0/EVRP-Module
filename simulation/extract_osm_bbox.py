"""
extract_osm_bbox.py — Recorta el .osm.pbf original (el mismo dato base usado
para OSRM) a un bounding box, y lo convierte a XML de OSM plano.

Por qué existe este paso:
    El netconvert de SUMO instalado en esta máquina NO tiene soporte para
    .osm.pbf (build sin protobuf) — falla intentando leer el binario como
    texto. Este script usa pyosmium (lee el .pbf directamente, sin pasar por
    OSRM ni por ninguna conversión previa) para producir un .osm XML que
    netconvert sí puede leer.

    Se usa osmium.BackReferenceWriter, que además de filtrar por bbox
    resuelve automáticamente las referencias "hacia atrás": si una vía (way)
    tiene aunque sea un nodo dentro del bbox, se incluye completa (con TODOS
    sus nodos, incluso los que caen fuera del recorte) para no dejar calles
    con extremos colgantes que rompan la topología de la red.

Uso:
    python simulation/extract_osm_bbox.py --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt

    o con bbox manual:
    python simulation/extract_osm_bbox.py --bbox -71.319,46.7489,-71.1465,46.8756
"""
import argparse
import os
import sys
import time

import osmium


def bbox_from_instance(instance_path: str, margin_deg: float) -> tuple:
    """Calcula (west, south, east, north) a partir de las coordenadas de la
    instancia (depósito + clientes + estaciones de carga), con margen."""
    lats, lons = [], []
    with open(instance_path, encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            fields = [x.strip() for x in line.split(",")]
            if fields[0] == "DEPOSITO" and len(fields) >= 3:
                lats.append(float(fields[1])); lons.append(float(fields[2]))
            elif fields[0].startswith("C_") and len(fields) >= 4:
                lats.append(float(fields[1])); lons.append(float(fields[2]))
            elif len(fields) >= 4:
                try:
                    int(fields[0])
                    lats.append(float(fields[2])); lons.append(float(fields[3]))
                except ValueError:
                    pass
    if not lats:
        sys.exit(f"ERROR: no se pudieron leer coordenadas de {instance_path}")
    return (
        min(lons) - margin_deg, min(lats) - margin_deg,
        max(lons) + margin_deg, max(lats) + margin_deg,
    )


class BBoxWayFilter(osmium.SimpleHandler):
    """Marca en `writer` toda way que tenga al menos un nodo dentro del bbox.
    BackReferenceWriter se encarga de completar nodos/ways referenciados."""

    def __init__(self, writer, bbox):
        super().__init__()
        self.writer = writer
        self.west, self.south, self.east, self.north = bbox
        self.ways_kept = 0

    def way(self, w):
        for n in w.nodes:
            loc = n.location
            if loc.valid() and self.west <= loc.lon <= self.east and self.south <= loc.lat <= self.north:
                self.writer.add_way(w)
                self.ways_kept += 1
                return


def parse_args():
    p = argparse.ArgumentParser(description="Recorta el .osm.pbf a un bbox y lo convierte a XML para netconvert")
    p.add_argument("--osm-file", default=os.path.join("osrm_data", "quebec-latest.osm.pbf"))
    p.add_argument("--instance", default=None,
                    help="Instancia EVRP de la que derivar el bbox (con --margin-deg de holgura)")
    p.add_argument("--bbox", default=None,
                    help="Bbox manual 'west,south,east,north' (alternativa a --instance)")
    p.add_argument("--margin-deg", type=float, default=0.03,
                    help="Margen en grados alrededor de la instancia (~3km por defecto)")
    p.add_argument("--output", "-o", default=os.path.join("simulation", "sumo_network", "clipped.osm"))
    return p.parse_args()


def main():
    args = parse_args()
    if not os.path.isfile(args.osm_file):
        sys.exit(f"ERROR: no se encontró {args.osm_file}")

    if args.bbox:
        bbox = tuple(float(x) for x in args.bbox.split(","))
    elif args.instance:
        bbox = bbox_from_instance(args.instance, args.margin_deg)
    else:
        sys.exit("ERROR: pasa --instance o --bbox")

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    if os.path.exists(args.output):
        os.remove(args.output)

    print(f"Origen : {args.osm_file}")
    print(f"Bbox   : west={bbox[0]:.4f} south={bbox[1]:.4f} east={bbox[2]:.4f} north={bbox[3]:.4f}")
    print(f"Salida : {args.output}\n")

    t0 = time.time()
    writer = osmium.BackReferenceWriter(args.output, ref_src=args.osm_file, overwrite=True)
    try:
        handler = BBoxWayFilter(writer, bbox)
        handler.apply_file(args.osm_file, locations=True)
    finally:
        writer.close()

    elapsed = time.time() - t0
    size_mb = os.path.getsize(args.output) / 1e6
    print(f"OK: {handler.ways_kept} ways dentro del bbox. {args.output} generado en {elapsed:.1f}s ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
