"""
archivar_resultados.py — ordena las carpetas de soluciones sin perder ni sobreescribir nada.

    python archivar_resultados.py --dry-run      # solo muestra que haria
    python archivar_resultados.py                # lo hace

Hace tres cosas sobre problem/osrm (solutions_evrp_<n>/solutions_<algoritmo>):
  1. Renombra solutions_HeuristicType.X_and_ACO -> solutions_X_and_ACO (nombre limpio). Si ya existe
     una carpeta con el nombre limpio, la vieja se archiva en vez de renombrarse.
  2. Archiva las carpetas con resultados de otra configuracion (consumo distinto de 3.0): no son
     comparables y solo estorban.
  3. Con --all archiva TODAS las carpetas de solutions_evrp_<n> indicadas en --runs (por defecto 0),
     para empezar una campaña nueva limpia.
Lo archivado va a problem/osrm/archivo/<AAAA-MM-DD>/..., donde AAAA-MM-DD es la fecha de esos resultados.
"""
import argparse
import glob
import os
import re

from utils.archive import archive_folder

ROOT = os.path.join("problem", "osrm")
RATE = 3.0


def consumption_of(folder):
    for path in sorted(glob.glob(os.path.join(folder, "quebec_*.txt"))):
        m = re.search(r"Battery consumption rate: ([\d.]+)", open(path, encoding="utf-8").read())
        if m:
            return float(m.group(1))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--all", action="store_true", help="archivar todas las carpetas de las corridas --runs")
    ap.add_argument("--runs", nargs="+", default=["0"], help="numeros de solutions_evrp_<n> a revisar (por defecto 0)")
    args = ap.parse_args()
    tag = "[simulacion] " if args.dry_run else ""

    for run in args.runs:
        run_dir = os.path.join(ROOT, f"solutions_evrp_{run}")
        for folder in sorted(glob.glob(os.path.join(run_dir, "solutions_*"))):
            name = os.path.basename(folder)
            if args.all:
                print(f"{tag}archivar {folder} -> {archive_folder(folder, ROOT, args.dry_run)}")
                continue
            if "HeuristicType." in name:
                clean = os.path.join(run_dir, name.replace("HeuristicType.", ""))
                if os.path.exists(clean):
                    print(f"{tag}archivar {folder} (ya existe {os.path.basename(clean)}) -> {archive_folder(folder, ROOT, args.dry_run)}")
                else:
                    print(f"{tag}renombrar {name} -> {os.path.basename(clean)}")
                    if not args.dry_run:
                        os.rename(folder, clean)
                continue
            rate = consumption_of(folder)
            if rate is not None and abs(rate - RATE) > 1e-9:
                print(f"{tag}archivar {folder} (consumo {rate}, no comparable) -> {archive_folder(folder, ROOT, args.dry_run)}")


if __name__ == "__main__":
    main()
