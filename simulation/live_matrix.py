"""
live_matrix.py -- matriz de distancia/tiempo "en vivo" para la re-planificacion, completa o incremental.

Matriz COMPLETA (comportamiento original): traci.simulation.findRoute() para cada par de nodos del sub-problema.
Crece con el cuadrado del numero de nodos (en la instancia de 350 clientes, ~160 000 consultas).

Matriz INCREMENTAL (--matriz-incremental en dynamic_replanning.py):
  1. Antes del evento se calcula UNA vez la matriz base entre deposito, todos los clientes y todas las estaciones,
     guardando tambien la RUTA (lista de tramos) de cada par. Se guarda en cache (simulation/_cache_matriz/) y se
     reutiliza en todas las corridas de la misma instancia y red.
  2. En el evento solo se recalculan con findRoute:
       - los pares cuya ruta base pasa por un tramo afectado por el evento;
       - las filas y columnas de la posicion actual de cada vehiculo.
     El resto se toma de la base.

Por que es exacta: con el modo de ruteo por defecto de findRoute, el coste de cada tramo es su tiempo teorico
(longitud / velocidad maxima); el trafico de los demas vehiculos no interviene. El evento solo EMPEORA los tramos
afectados (baja su velocidad o los cierra), asi que una ruta que no pasaba por ellos sigue siendo la mas corta y con
el mismo coste. Si en el futuro se usara un ruteo que dependa del trafico (routingMode agregado con device.rerouting),
habria que recalcular tambien los pares que pasan por tramos cuyo tiempo haya cambiado.
Salvo empates: si dos rutas cuestan exactamente lo mismo, SUMO podria elegir otra; el tiempo es igual y la distancia
podria diferir unos metros. --verificar-matriz lo comprueba calculando ambas matrices en la misma corrida.
"""
import hashlib
import json
import os
import time

import numpy as np

CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cache_matriz")


def _progress(label, done, total, t0, every):
    if done == total or done % every == 0:
        el = time.time() - t0
        eta = el / done * (total - done) if done else 0
        print(f"  {label}: {done}/{total} filas ({100.0 * done / total:.0f} %), "
              f"{el / 60:.1f} min transcurridos, ~{eta / 60:.1f} min restantes", flush=True)


def full_matrix(traci, edges, label="matriz completa"):
    """Todos los pares con findRoute (metodo original). edges: lista de tramos, uno por nodo."""
    n = len(edges)
    dist = [[0.0] * n for _ in range(n)]
    tt = [[0.0] * n for _ in range(n)]
    t0 = time.time()
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            st = traci.simulation.findRoute(edges[a], edges[b])
            dist[a][b] = st.length / 1000.0
            tt[a][b] = st.travelTime
        _progress(label, a + 1, n, t0, max(1, n // 20))
    return dist, tt


def cache_path(instance_base, net_file, base_edges):
    key = hashlib.sha256((os.path.abspath(net_file) + "|" + str(os.path.getsize(net_file)) + "|" +
                          "|".join(base_edges)).encode("utf-8")).hexdigest()[:16]
    return os.path.join(CACHE_DIR, f"{instance_base}_{key}.npz")


def build_base(traci, base_edges):
    """Matriz base ANTES del evento, con la ruta de cada par (tramos como enteros)."""
    n = len(base_edges)
    edge_id = {}
    dist = np.zeros((n, n)); tt = np.zeros((n, n))
    flat_rows, lens = [], np.zeros(n * n, dtype=np.int32)
    t0 = time.time()
    for a in range(n):
        row = []
        for b in range(n):
            if a == b:
                continue
            st = traci.simulation.findRoute(base_edges[a], base_edges[b])
            dist[a, b] = st.length / 1000.0
            tt[a, b] = st.travelTime
            ids = [edge_id.setdefault(e, len(edge_id)) for e in st.edges]
            lens[a * n + b] = len(ids)
            row.extend(ids)
        flat_rows.append(np.asarray(row, dtype=np.int32))
        _progress("matriz base (una sola vez por instancia)", a + 1, n, t0, max(1, n // 20))
    flat = np.concatenate(flat_rows) if flat_rows else np.zeros(0, dtype=np.int32)
    offsets = np.concatenate([[0], np.cumsum(lens)]).astype(np.int64)
    names = np.array(sorted(edge_id, key=edge_id.get), dtype=object)
    return dict(dist=dist, tt=tt, flat=flat, offsets=offsets, edge_names=names, base_edges=np.array(base_edges, dtype=object))


def load_or_build_base(traci, path, base_edges):
    if os.path.isfile(path):
        z = np.load(path, allow_pickle=True)
        base = {k: z[k] for k in z.files}
        if list(base["base_edges"]) == list(base_edges):
            print(f"  Matriz base cargada de la cache: {path}")
            return base
        print("  La cache no corresponde a estos nodos; se recalcula.")
    print(f"  Calculando la matriz base ({len(base_edges)} nodos; se guarda en cache para las demas corridas)...")
    base = build_base(traci, base_edges)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp.npz"
    np.savez_compressed(tmp, **base)
    os.replace(tmp, path)
    print(f"  Matriz base guardada en {path}")
    return base


def affected_base_pairs(base, changed_edges):
    """Pares (a, b) de la base cuya ruta pasa por alguno de los tramos cambiados."""
    names = list(base["edge_names"])
    idx = {e: i for i, e in enumerate(names)}
    ids = [idx[e] for e in changed_edges if e in idx]
    if not ids:
        return set()
    hits = np.nonzero(np.isin(base["flat"], np.asarray(ids, dtype=np.int32)))[0]
    pair = np.searchsorted(base["offsets"], hits, side="right") - 1
    n = len(base["base_edges"])
    return {(int(p) // n, int(p) % n) for p in np.unique(pair)}


def incremental_matrix(traci, base, sub_edges, sub_to_base, changed_edges):
    """sub_edges: tramo de cada nodo del sub-problema; sub_to_base: indice en la base o None (posicion de vehiculo)."""
    n = len(sub_edges)
    affected = affected_base_pairs(base, changed_edges)
    dist = [[0.0] * n for _ in range(n)]
    tt = [[0.0] * n for _ in range(n)]
    n_query = 0
    for a in range(n):
        for b in range(n):
            if a == b:
                continue
            ba, bb = sub_to_base[a], sub_to_base[b]
            if ba is None or bb is None or (ba, bb) in affected:
                st = traci.simulation.findRoute(sub_edges[a], sub_edges[b])
                dist[a][b] = st.length / 1000.0
                tt[a][b] = st.travelTime
                n_query += 1
            else:
                dist[a][b] = float(base["dist"][ba, bb])
                tt[a][b] = float(base["tt"][ba, bb])
    print(f"  Matriz incremental: {n_query} consultas a SUMO de {n * (n - 1)} pares "
          f"({len(affected)} pares de la base pasan por los tramos del evento: {', '.join(changed_edges)})")
    return dist, tt


def compare(full, inc, out_json=None):
    (fd, ft), (idd, it) = full, inc
    fd, ft, idd, it = map(np.asarray, (fd, ft, idd, it))
    res = {"nodos": int(fd.shape[0]),
           "max_dif_tiempo_s": float(np.abs(ft - it).max()), "max_dif_distancia_km": float(np.abs(fd - idd).max()),
           "pares_distintos_tiempo": int((np.abs(ft - it) > 1e-6).sum()),
           "pares_distintos_distancia": int((np.abs(fd - idd) > 1e-9).sum())}
    res["identicas"] = res["pares_distintos_tiempo"] == 0 and res["pares_distintos_distancia"] == 0
    print("\n" + "=" * 70 + "\nVERIFICACION matriz completa vs incremental\n" + "=" * 70)
    for k, v in res.items():
        print(f"  {k}: {v}")
    print("  RESULTADO: " + ("IDENTICAS" if res["identicas"] else "DIFERENTES (ver pares distintos)") + "\n" + "=" * 70)
    if out_json:
        os.makedirs(os.path.dirname(out_json), exist_ok=True)
        with open(out_json, "w", encoding="utf-8") as f:
            json.dump(res, f, indent=2)
    return res
