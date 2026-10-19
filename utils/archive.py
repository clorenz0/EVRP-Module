"""
Archivo de resultados: en vez de sobreescribir una carpeta de soluciones al repetir un experimento,
se mueve a  problem/<distancia>/archivo/<AAAA-MM-DD>/<ruta original>  (fecha = ultima modificacion
de los resultados). Asi los experimentos viejos quedan separados por fecha, no entran al Friedman y
no se pierden.
"""
import os
import shutil
from datetime import datetime


def _newest_mtime(folder):
    times = [os.path.getmtime(os.path.join(r, f)) for r, _, fs in os.walk(folder) for f in fs]
    return max(times) if times else os.path.getmtime(folder)


def archive_folder(folder, root=os.path.join("problem", "osrm"), dry_run=False, copy=False):
    """Mueve (o copia, con copy=True) `folder` a <root>/archivo/<fecha>/<ruta relativa a root>. Devuelve el destino."""
    folder = os.path.normpath(folder)
    day = datetime.fromtimestamp(_newest_mtime(folder)).strftime("%Y-%m-%d")
    rel = os.path.relpath(folder, root)
    dest = os.path.join(root, "archivo", day, rel)
    n = 2
    while os.path.exists(dest):               # misma fecha y mismo nombre: no pisar
        dest = os.path.join(root, "archivo", day, f"{rel}_{n}")
        n += 1
    if not dry_run:
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        (shutil.copytree if copy else shutil.move)(folder, dest)
    return dest


def archive_if_exists(folder, root=os.path.join("problem", "osrm")):
    """
    Si la carpeta ya tiene resultados, guarda una COPIA fechada antes de que se sobreescriban. Es copia
    y no movimiento para que una corrida parcial (una sola instancia) no deje incompleta la carpeta.
    """
    if os.path.isdir(folder) and any(True for _ in os.scandir(folder)):
        dest = archive_folder(folder, root, copy=True)
        print(f"  [archivo] resultados previos copiados a {dest}")
        return dest
    return None
