"""
friedman_ranking.py — ranking de Friedman de ACO frente a todas las
combinaciones heuristica/metaheuristica de OR-Tools.

Diseño del experimento
  - Bloques  : las instancias EVRP (una fila por instancia): 10 por defecto, 5 con --instances 5.
  - Tratamientos: cada combinacion de OR-Tools (carpeta solutions_<combo>) y ACO.
  - Medida   : distancia total REAL en km (sin truncar por arco). Menor es mejor. El Objective de los
               archivos es la suma de arcos truncados a km enteros y subestima la distancia real;
               solo se usa para comprobar la consistencia del archivo.
  - Solo participan algoritmos con todas las instancias resueltas Y todas factibles con
    bateria fisica estricta (>= 0 en todos los tramos, sin clientes descartados). Una
    combinacion con algun error, falta o solucion infactible se EXCLUYE completa
    (Friedman necesita bloques completos); el script lista cada exclusion y su motivo.
  - ACO entra con UNA semilla (sorteada entre las corridas completas de run_aco.py, o
    la que se indique), la misma en todas las instancias, igual que cada combinacion
    de OR-Tools es una sola corrida.

Uso (raiz del repo, OSRM o cache disponible):
    python friedman_ranking.py                     # semilla de ACO sorteada
    python friedman_ranking.py --aco-seed 3        # semilla fija
    python friedman_ranking.py --pick-seed 42      # sorteo repetible
    python friedman_ranking.py --instances 5       # solo las 5 instancias originales
    python friedman_ranking.py --holm              # ademas, Holm contra ACO (opcional)

Salida: ranking por rango medio, estadistico de Friedman (con correccion por empates),
p-valor y un CSV con el ranking.
"""
import argparse
import csv
import glob
import math
import os
import random
import re

from distance.distance_type import DistanceType
from utils.trace import TraceLog, timestamp
from instance.instance_type import InstanceType, process_files
from utils.units import DIST_SCALE, cost_int, energy_int, cap_int

CAP = 100
RATE = 3.0
INSTANCES_5 = [
    "quebec_30c_6ev_4cs.txt", "quebec_40c_4ev_6cs.txt", "quebec_50c_5ev_5cs.txt",
    "quebec_60c_8ev_8cs.txt", "quebec_90c_10ev_10cs.txt",
]
INSTANCES_10 = INSTANCES_5 + [
    "quebec_70c_9ev_9cs.txt", "quebec_100c_12ev_12cs.txt", "quebec_150c_18ev_15cs.txt",
    "quebec_220c_26ev_20cs.txt", "quebec_350c_40ev_30cs.txt",
]
INSTANCES = list(INSTANCES_10)   # por defecto los 10 bloques; --instances 5 usa solo las originales


# ───────────────────────── estadistica (sin scipy) ─────────────────────────

def _gammaincc(a, x):
    """Gamma incompleta regularizada superior Q(a, x) (Numerical Recipes)."""
    if x <= 0:
        return 1.0
    gln = math.lgamma(a)
    if x < a + 1:   # serie para P(a,x)
        ap, s, d = a, 1.0 / a, 1.0 / a
        for _ in range(500):
            ap += 1
            d *= x / ap
            s += d
            if abs(d) < abs(s) * 1e-14:
                break
        return 1.0 - s * math.exp(-x + a * math.log(x) - gln)
    b = x + 1 - a   # fraccion continua para Q(a,x)
    c, d = 1e300, 1.0 / b
    h = d
    for i in range(1, 500):
        an = -i * (i - a)
        b += 2
        d = an * d + b
        d = 1e-300 if abs(d) < 1e-300 else d
        c = b + an / c
        c = 1e-300 if abs(c) < 1e-300 else c
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1) < 1e-14:
            break
    return math.exp(-x + a * math.log(x) - gln) * h


def chi2_sf(stat, df):
    return _gammaincc(df / 2.0, stat / 2.0)


def average_ranks(values):
    """Rangos 1..k (menor valor = rango 1), con empates promediados."""
    order = sorted(range(len(values)), key=lambda i: values[i])
    ranks = [0.0] * len(values)
    i = 0
    while i < len(order):
        j = i
        while j + 1 < len(order) and values[order[j + 1]] == values[order[i]]:
            j += 1
        avg = (i + j) / 2.0 + 1.0
        for k in range(i, j + 1):
            ranks[order[k]] = avg
        i = j + 1
    return ranks


def friedman(rank_matrix):
    """rank_matrix[i][j]: rango del algoritmo j en la instancia i. Devuelve (Q corregido, df, p)."""
    n, k = len(rank_matrix), len(rank_matrix[0])
    rj = [sum(row[j] for row in rank_matrix) for j in range(k)]
    q = 12.0 / (n * k * (k + 1)) * sum(r * r for r in rj) - 3.0 * n * (k + 1)
    # correccion por empates
    tie_sum = 0.0
    for row in rank_matrix:
        counts = {}
        for r in row:
            counts[r] = counts.get(r, 0) + 1
        tie_sum += sum(t ** 3 - t for t in counts.values())
    correction = 1.0 - tie_sum / (n * k * (k * k - 1))
    q = q / correction if correction > 0 else float('nan')
    return q, k - 1, chi2_sf(q, k - 1)


def holm(pvalues):
    """Ajuste de Holm. pvalues: lista; devuelve ajustados en el mismo orden."""
    m = len(pvalues)
    order = sorted(range(m), key=lambda i: pvalues[i])
    adj = [0.0] * m
    running = 0.0
    for pos, i in enumerate(order):
        running = max(running, min(1.0, (m - pos) * pvalues[i]))
        adj[i] = running
    return adj


# ───────────────────────── lectura y factibilidad ─────────────────────────

def unnecessary_stops(nodes, dm, cs):
    """Paradas de carga que se podrian quitar sin dejar la bateria negativa (misma regla en todos los algoritmos)."""
    cap = cap_int(CAP)      # el modelo trabaja en metros (utils/units.py)

    def feasible(seq):
        bat = cap
        for a, b in zip(seq, seq[1:]):
            bat -= energy_int(dm[a][b], RATE)
            if bat < 0:
                return False
            if b in cs:
                bat = cap
        return True
    return sum(1 for k in range(1, len(nodes) - 1) if nodes[k] in cs and feasible(nodes[:k] + nodes[k + 1:]))


STATION_CHAIN = {}   # (archivo) -> rutas con un salto estacion -> estacion: archivo excluido
DEPOT_TO_STATION = {}   # (archivo) -> rutas que salen del deposito directo a una estacion: archivo excluido
LEGACY = {}   # (archivo) -> soluciones del modelo anterior (distancias truncadas a km enteros): excluidas, hay que repetir la corrida
INCONSISTENT = {}   # (archivo) -> (Objective guardado, distancia real): archivo excluido
DROPPED = {}   # (archivo) -> clientes sin atender: archivo excluido (la solucion no es completa)
NOTED = {}     # (archivo) -> paradas de carga innecesarias encontradas (solo informativo: el objetivo es solo distancia)


def read_solution(path, data, max_visits=None):
    """Devuelve el objetivo (solo distancia), o None si hay clientes descartados / es infactible / no existe.
    max_visits: si se indica, tambien es infactible visitar una estacion mas veces que eso (el limite
    tambien se lee del encabezado 'Station copies: K' de las soluciones de OR-Tools)."""
    if not os.path.isfile(path):
        return None
    txt = open(path, encoding="utf-8").read()
    md = re.search(r"Dropped customers: (\d+)", txt)
    if "Dropped clients" in txt or (md and int(md.group(1)) > 0):
        DROPPED[path] = int(md.group(1)) if md else -1
        return None
    m = re.search(r"Battery consumption rate: ([\d.]+)", txt)
    if m and abs(float(m.group(1)) - RATE) > 1e-9:   # archivo viejo de otra configuracion (no se re-ejecuto)
        return None
    obj_txt = re.search(r"Objective: ([\d.]+)", txt).group(1)
    if "." not in obj_txt:          # formato anterior: km enteros truncados por arco
        LEGACY[path] = 1
        return None
    obj = float(obj_txt)
    dm, cs = data["distance_matrix"], set(data["charging_stations"])
    n_clients = data["num_locations"] - len(cs) - 1
    served = []
    station_visits = {}
    unnecessary = 0
    mc = re.search(r"Station copies: (\d+)", txt)
    if mc:
        max_visits = int(mc.group(1)) if max_visits is None else min(max_visits, int(mc.group(1)))
    for blk in re.findall(r"Route for vehicle \d+:\n (.*?)\nDistance", txt):
        nodes = [int(re.match(r"(\d+)", s.strip()).group(1)) for s in blk.split("->")]
        bat = cap_int(CAP)
        if len(nodes) > 1 and nodes[0] == 0 and nodes[1] in cs:
            DEPOT_TO_STATION[path] = DEPOT_TO_STATION.get(path, 0) + 1     # del deposito no se va directo a una estacion
            return None
        if any(a in cs and b in cs for a, b in zip(nodes, nodes[1:])):
            STATION_CHAIN[path] = STATION_CHAIN.get(path, 0) + 1           # de una estacion no se va directo a otra
            return None
        unnecessary += unnecessary_stops(nodes, dm, cs)
        for a, b in zip(nodes, nodes[1:]):
            bat -= energy_int(dm[a][b], RATE)
            if bat < 0:
                return None
            if b in cs:
                bat = cap_int(CAP)
                station_visits[b] = station_visits.get(b, 0) + 1
            elif b != 0:
                served.append(b)
    if sorted(served) != list(range(1, n_clients + 1)):
        return None
    # control de consistencia: el Objective guardado debe ser la distancia real de las rutas (sin descartes);
    # si no coincide, el modelo que genero el archivo tenia una matriz o un objetivo distinto al real
    real = 0
    real_km = 0.0     # distancia real (km, sin truncar por arco): es la medida que usa el ranking
    for blk in re.findall(r"Route for vehicle \d+:\n (.*?)\nDistance", txt):
        ns = [int(re.match(r"(\d+)", s.strip()).group(1)) for s in blk.split("->")]
        real += sum(cost_int(dm[a][b]) for a, b in zip(ns, ns[1:]))
        real_km += sum(dm[a][b] for a, b in zip(ns, ns[1:]))
    if abs(real / DIST_SCALE - obj) > 0.0015:
        INCONSISTENT[path] = (obj, round(real / DIST_SCALE, 3))
        return None
    if max_visits is not None and any(v > max_visits for v in station_visits.values()):
        return None
    if unnecessary:
        NOTED[path] = unnecessary
    # El Objective guardado es la distancia en km (el modelo trabaja en metros enteros: utils/units.py); el control de
    # consistencia de arriba lo compara con la distancia recalculada. El ranking usa los km reales de las rutas.
    return round(real_km, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ortools-root", default=os.path.join("problem", "osrm", "solutions_evrp_0"))
    # (ACO: ver --aco-root / --aco-seed abajo)
    ap.add_argument("--aco-root", default=os.path.join("problem", "osrm"),
                    help="carpeta con solutions_evrp_<1000+semilla>/solutions_ACO (las que deja run_aco.py)")
    ap.add_argument("--aco-seed", default="random",
                    help="semilla de ACO a usar en el ranking (1-6) o 'random' (por defecto): una sola semilla, "
                         "igual que cada combinacion de OR-Tools es una sola corrida")
    ap.add_argument("--pick-seed", type=int, default=None,
                    help="semilla del sorteo cuando --aco-seed random (para poder repetir el sorteo)")
    ap.add_argument("--instances", type=int, choices=(5, 10), default=10,
                    help="numero de instancias (bloques): 5 = las originales, 10 = originales + 5 grandes (por defecto)")
    ap.add_argument("--holm", action="store_true", help="ademas, compara cada algoritmo contra ACO con Holm (opcional)")
    ap.add_argument("--out", default=None, help="CSV del ranking (por defecto con fecha y hora, no se sobreescribe)")
    ap.add_argument("--keel-out", default=None, help="CSV formato KEEL (por defecto con fecha y hora)")
    args = ap.parse_args()
    INSTANCES[:] = INSTANCES_5 if args.instances == 5 else INSTANCES_10
    stamp = timestamp()
    out_dir = os.path.join("problem", "osrm", "trazas")
    os.makedirs(out_dir, exist_ok=True)
    args.out = args.out or os.path.join(out_dir, f"friedman_ranking_{stamp}.csv")
    args.keel_out = args.keel_out or os.path.join(out_dir, f"friedman_keel_{stamp}.csv")
    with TraceLog("friedman", "ranking" + ("_holm" if args.holm else ""), stamp=stamp):
        run(args)


def run(args):
    data_all = process_files(InstanceType.EVRP, DistanceType.OSRM, None, RATE, CAP)

    # ── Combinaciones de OR-Tools: solo las que tienen todas las instancias y TODAS factibles ──
    algos = {}       # nombre -> [objetivo por instancia]
    excluded = {}    # nombre -> motivo
    for folder in sorted(glob.glob(os.path.join(args.ortools_root, "solutions_*"))):
        name = os.path.basename(folder)[len("solutions_"):].replace("HeuristicType.", "")   # solo el nombre
        if name == "ACO":      # corridas viejas de ACO guardadas junto a OR-Tools: ACO entra por --aco-root
            continue
        missing = [f for f in INSTANCES if not os.path.isfile(os.path.join(folder, f))]
        if missing:
            excluded[name] = f"faltan {len(missing)} instancia(s) (error o sin ejecutar)"
            continue
        values = [read_solution(os.path.join(folder, f), data_all[f]) for f in INSTANCES]
        bad = sum(v is None for v in values)
        if bad:
            excluded[name] = f"{bad} solucion(es) infactible(s) o con clientes descartados"
            continue
        algos[name] = values

    # ── ACO (y ACO + heuristica): UNA semilla (sorteada o elegida), la misma en todas las instancias ──
    def aco_combos(seed):
        root = os.path.join(args.aco_root, f"solutions_evrp_{1000 + seed}")
        return {os.path.basename(d)[len("solutions_"):]: d for d in glob.glob(os.path.join(root, "solutions_ACO"))
                if all(os.path.isfile(os.path.join(d, f)) for f in INSTANCES)}

    available = [s_ for s_ in range(1, 100) if "ACO" in aco_combos(s_)]
    if not available:
        raise SystemExit("No hay corridas de ACO completas: ejecuta run_aco.py (todas las instancias) primero.")
    if args.aco_seed == "random":
        rng = random.Random(args.pick_seed)
        aco_seed = rng.choice(available)
        print(f"Semilla de ACO sorteada entre {available}: {aco_seed}"
              + (f" (sorteo con --pick-seed {args.pick_seed})" if args.pick_seed is not None else ""))
    else:
        aco_seed = int(args.aco_seed)
        if aco_seed not in available:
            raise SystemExit(f"La semilla {aco_seed} no tiene todas las instancias; disponibles: {available}")
    aco_names = []
    for name, d in sorted(aco_combos(aco_seed).items()):
        values = [read_solution(os.path.join(d, f), data_all[f]) for f in INSTANCES]
        if any(v is None for v in values):
            excluded[name] = f"semilla {aco_seed}: solucion infactible"
            continue
        algos[name] = values
        aco_names.append(name)
    if "ACO" not in algos:
        raise SystemExit(f"La semilla {aco_seed} de ACO tiene alguna solucion infactible: no se puede rankear.")

    if DEPOT_TO_STATION:
        print(f"\nExcluidas por salir del deposito directo a una estacion ({len(DEPOT_TO_STATION)} archivo(s)):")
        for path_ in sorted(DEPOT_TO_STATION)[:15]:
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}")
    if DROPPED:
        print(f"\nExcluidas por dejar clientes sin atender ({len(DROPPED)} archivo(s)):")
        for path_, k_ in sorted(DROPPED.items())[:30]:
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}"
                  f"{': ' + str(k_) + ' cliente(s) sin atender' if k_ > 0 else ''}")
    if STATION_CHAIN:
        print(f"\nExcluidas por ir de una estacion directo a otra ({len(STATION_CHAIN)} archivo(s)):")
        for path_ in sorted(STATION_CHAIN)[:15]:
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}")
    if LEGACY:
        print(f"\nExcluidas por ser del modelo anterior, con distancias truncadas a km enteros ({len(LEGACY)} archivo(s)): repetir la corrida.")
        for path_ in sorted(LEGACY)[:15]:
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}")
    if INCONSISTENT:
        print(f"\nExcluidas por Objective distinto de la distancia real ({len(INCONSISTENT)} archivo(s)): "
              f"el modelo que las genero no es el actual.")
        for path_, (o_, r_) in sorted(INCONSISTENT.items())[:15]:
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}: Objective {o_} vs real {r_}")
    if NOTED:
        print(f"\nSoluciones con paradas de carga innecesarias ({len(NOTED)}; informativo, no cambia el objetivo):")
        for path_, k_ in sorted(NOTED.items()):
            print(f"  - {os.path.basename(os.path.dirname(path_))[len('solutions_'):]} / {os.path.basename(path_)[7:10]}: {k_}")
    if excluded:
        print(f"\nExcluidas del ranking ({len(excluded)}):")
        for name, reason in excluded.items():
            print(f"  - {name}: {reason}")

    names = list(algos)
    n, k = len(INSTANCES), len(names)
    if k < 3:
        raise SystemExit(f"Solo hay {k} algoritmos completos y factibles; Friedman necesita al menos 3.")
    rank_matrix = [average_ranks([algos[a][i] for a in names]) for i in range(n)]
    mean_rank = [sum(rank_matrix[i][j] for i in range(n)) / n for j in range(k)]
    q, df, p = friedman(rank_matrix)

    order = sorted(range(k), key=lambda j: mean_rank[j])
    aco_set = {names.index(n_) for n_ in aco_names}
    aco_j = names.index("ACO")
    p_holm = {}
    if args.holm:
        se = math.sqrt(k * (k + 1) / (6.0 * n))
        others = [j for j in range(k) if j != aco_j]
        pvals = [math.erfc(abs(mean_rank[j] - mean_rank[aco_j]) / se / math.sqrt(2)) for j in others]
        p_holm = dict(zip(others, holm(pvals)))

    print(f"\nFriedman: {k} algoritmos x {n} instancias | Q = {q:.3f}, gl = {df}, p = {p:.4g}")
    header = f"{'pos':>3}  {'algoritmo':<58}{'rango':>6}  " + "  ".join(f"{i[7:10]:>7}" for i in INSTANCES)
    print(header + ("  p Holm vs ACO" if args.holm else ""))
    rows = []
    for pos, j in enumerate(order, start=1):
        a = names[j]
        objs = [str(algos[a][i]) for i in range(n)]
        ph = "" if (j == aco_j or not args.holm) else f"{p_holm[j]:.3f}"
        mark = f"  <== ACO (semilla {aco_seed})" if j in aco_set else ""
        print(f"{pos:>3}  {a:<58}{mean_rank[j]:>6.2f}  " + "  ".join(f"{o:>7}" for o in objs) + (f"  {ph:>6}" if args.holm else "") + mark)
        rows.append([pos, a, round(mean_rank[j], 3)] + objs + ([ph] if args.holm else []))

    aco_pos = order.index(aco_j) + 1
    print(f"\nACO (semilla {aco_seed}) queda en la posicion {aco_pos} de {k} (rango medio {mean_rank[aco_j]:.2f}).")
    if args.holm:
        print("Aviso: con pocas instancias el test tiene poca potencia; un p Holm alto significa que no se puede afirmar diferencia con ACO.")

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["posicion", "algoritmo", "rango_medio"] + [i.replace(".txt", "") for i in INSTANCES] + (["p_holm_vs_ACO"] if args.holm else []))
        w.writerows(rows)
    print(f"CSV: {args.out}")

    # CSV formato KEEL: filas = instancias, columnas = algoritmos (orden alfabetico), primera celda vacia
    keel_names = sorted(names)
    with open(args.keel_out, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([""] + keel_names)
        for i, inst in enumerate(INSTANCES):
            w.writerow([inst.replace("quebec_", "").replace(".txt", "")] + [algos[a][i] for a in keel_names])
    print(f"CSV KEEL ({len(keel_names)} algoritmos): {args.keel_out}")


if __name__ == "__main__":
    main()
