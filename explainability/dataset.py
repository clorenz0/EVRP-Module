"""
Generacion de la tabla de datos para explicar un algoritmo.

Cada fila = una corrida del algoritmo con
  (instancia, escenario [bateria, consumo], parametros del algoritmo, semilla) -> resultado.

Las filas se muestrean con Latin Hypercube (cobertura uniforme del espacio de parametros con pocas
corridas). El proceso es REANUDABLE: si se interrumpe, al volver a correr se saltan las filas
que ya estan en el CSV (el muestreo es determinista dada la semilla de muestreo y n_rows).
Se puede repartir entre varias terminales con part=(k, n): cada una procesa las filas id % n == k
y escribe su propio CSV.
"""
import csv
import glob
import os
import time

import numpy as np

from distance.distance_type import DistanceType
from instance.instance_type import process_files
from explainability.algorithms import ALGORITHMS
from explainability.features import instance_features, scenario_features

SCENARIO_SPACE = [
    {"name": "battery", "low": 70, "high": 150, "kind": "int"},
    {"name": "consumption", "low": 2.0, "high": 4.0, "kind": "float"},
]
INSTANCES_DIR = os.path.join("instances_data", "evrp_instances")


def _scale(u, dim):
    if dim["kind"] == "bool":
        return int(u >= 0.5)
    value = dim["low"] + u * (dim["high"] - dim["low"])
    return int(round(value)) if dim["kind"] == "int" else round(float(value), 4)


def sample_rows(algorithm, n_rows, sampling_seed=0, instances=None):
    """Lista de n_rows dicts {row_id, instance, seed, scenario{}, params{}} (muestreo Latin Hypercube)."""
    spec = ALGORITHMS[algorithm]
    scenario_space = SCENARIO_SPACE if spec.scenario_space is None else spec.scenario_space
    dims = scenario_space + spec.param_space
    rng = np.random.default_rng(sampling_seed)
    unit = np.empty((n_rows, len(dims)))
    for j in range(len(dims)):
        unit[:, j] = (rng.permutation(n_rows) + rng.random(n_rows)) / n_rows

    instances = instances or sorted(glob.glob(os.path.join(INSTANCES_DIR, "*.txt")))
    order = [instances[i] for i in rng.permutation(len(instances))]
    rows = []
    for r in range(n_rows):
        values = {d["name"]: _scale(unit[r, j], d) for j, d in enumerate(dims)}
        rows.append({
            "row_id": r,
            "instance": order[r % len(order)],
            "seed": int(rng.integers(1, 10 ** 6)),
            "scenario": {d["name"]: values[d["name"]] for d in scenario_space},
            "params": {d["name"]: values[d["name"]] for d in spec.param_space},
        })
    return rows


def _done_ids(path):
    if not os.path.isfile(path):
        return set()
    with open(path, newline="", encoding="utf-8") as f:
        return {int(r["row_id"]) for r in csv.DictReader(f)}


def generate(algorithm, out_csv, n_rows=200, time_limit=3, sampling_seed=0, part=(0, 1), instances=None, log=print):
    """Corre las filas pendientes y las agrega al CSV (una a una, para no perder nada si se corta)."""
    spec = ALGORITHMS[algorithm]
    rows = [r for r in sample_rows(algorithm, n_rows, sampling_seed, instances) if r["row_id"] % part[1] == part[0]]
    done = _done_ids(out_csv)
    pending = [r for r in rows if r["row_id"] not in done]
    log(f"{algorithm}: {len(rows)} filas asignadas, {len(done)} ya hechas, {len(pending)} pendientes "
        f"(~{len(pending) * time_limit / 60:.1f} min a {time_limit}s por corrida)")

    writer, f = None, None
    t0 = time.time()
    try:
        for k, row in enumerate(pending, start=1):
            path, sc, params = row["instance"], row["scenario"], row["params"]
            record = {"row_id": row["row_id"], "algorithm": algorithm,
                      "instance": os.path.basename(path), "seed": row["seed"], "time_limit": time_limit}
            try:
                battery, consumption = sc.get("battery", 100), sc.get("consumption", 3.0)
                data = process_files(path, DistanceType.OSRM, None, consumption, battery)[os.path.basename(path)]
                record.update(instance_features(data))
                record.update(scenario_features(battery, consumption, data))
                record.update({f"p_{k_}": v for k_, v in params.items()})
                res = spec.run(path, sc, params, row["seed"], time_limit)
                record.update({k_: res[k_] for k_ in spec.result_keys})
                record["status"] = "ok"
            except Exception as error:   # una corrida fallida no detiene el resto
                record["status"] = f"error: {type(error).__name__}: {error}"[:200]
            if writer is None:
                f = open(out_csv, "a", newline="", encoding="utf-8")
                header_needed = os.path.getsize(out_csv) == 0
                writer = csv.DictWriter(f, fieldnames=list(record.keys()), extrasaction="ignore")
                if header_needed:
                    writer.writeheader()
            writer.writerow({key: record.get(key, "") for key in writer.fieldnames})
            f.flush()
            if k % 10 == 0 or k == len(pending):
                log(f"  {k}/{len(pending)} filas ({time.time() - t0:.0f}s)")
    finally:
        if f:
            f.close()
    return out_csv
