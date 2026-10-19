"""
event_selector.py -- seleccion VERIFICADA y aplicacion de eventos de trafico en SUMO (congestion, accidente, cierre).

Un evento solo se acepta si cumple, por este orden:
  1. AFECTA a un vehiculo de la flota: los arcos se toman de la ruta restante de un vehiculo con paradas pendientes
     (nunca arcos que contengan un cliente, una estacion, el deposito o la posicion actual de algun vehiculo).
  2. IMPACTO: el tiempo del siguiente tramo de ese vehiculo sube al menos `impact_threshold` (0.20 por defecto;
     es el criterio r_t = 1 de la Ecuacion (8) de la tesis, con Delta = 20 %).
  3. CONECTIVIDAD (cierre y accidente): con los arcos eliminados, el deposito, las estaciones, los clientes pendientes
     y la posicion de cada vehiculo siguen en la misma componente fuertemente conexa del grafo transitable. Ningun
     punto queda aislado.
  4. NADIE VARADO: tras aplicar el evento, cada vehiculo puede llegar al deposito o a alguna estacion con la bateria
     que le queda (distancia real de SUMO x consumo).
Si un candidato falla alguna prueba se revierte y se prueba otro. El resultado (arcos, vehiculo, pruebas) se guarda en
un JSON (`EventSpec`) para aplicar EXACTAMENTE el mismo evento en las dos ramas del experimento (con y sin
retroalimentacion).

Tipos:
  - congestion: setMaxSpeed(arco, velocidad) -- como hasta ahora.
  - closure   : arco prohibido para todos los vehiculos (permanente).
  - accident  : igual que closure pero con duracion `accident_duration` s; el arco se reabre solo (EventController.tick).

La logica de grafos y de seleccion no depende de SUMO (recibe `net` y `traci` como argumentos), asi que se prueba con
objetos simulados.
"""
import json
import os
from dataclasses import dataclass, field, asdict

KINDS = ("congestion", "accident", "closure")
MIN_EDGE_LEN_M = 40.0        # no se eligen arcos muy cortos
SKIP_FIRST_M = 200.0         # ni los primeros metros de la ruta del vehiculo (ya esta encima)
SEGMENT_LENGTHS = (1, 2, 3)  # cuantos arcos contiguos puede afectar un evento


# ───────────────────────── grafo (sin SUMO) ─────────────────────────

def _neighbors(edge, forward, vclass):
    nxt = edge.getOutgoing() if forward else edge.getIncoming()
    return [e for e in nxt if e.allows(vclass)]


def reachable(net, start_id, removed, forward=True, vclass="passenger"):
    """Ids de arcos alcanzables desde `start_id` (hacia delante o hacia atras) sin pasar por `removed`."""
    start = net.getEdge(start_id)
    seen = {start_id}
    stack = [start]
    while stack:
        e = stack.pop()
        for n in _neighbors(e, forward, vclass):
            nid = n.getID()
            if nid in removed or nid in seen:
                continue
            seen.add(nid)
            stack.append(n)
    return seen


def keys_connected(net, depot_edge, key_edges, removed, vclass="passenger"):
    """True si todos los `key_edges` estan en la misma componente fuertemente conexa que el deposito
    tras quitar `removed`. Devuelve (ok, lista_de_arcos_aislados)."""
    removed = set(removed)
    fwd = reachable(net, depot_edge, removed, True, vclass)
    bwd = reachable(net, depot_edge, removed, False, vclass)
    bad = sorted(e for e in set(key_edges) if e not in removed and not (e in fwd and e in bwd))
    return (not bad), bad


def candidate_segments(route_edges, edge_len, protected, rng,
                       min_len=MIN_EDGE_LEN_M, skip_first_m=SKIP_FIRST_M, lengths=SEGMENT_LENGTHS):
    """Tramos contiguos de 1 a 3 arcos de `route_edges` aptos para el evento, en orden aleatorio reproducible."""
    allowed = []
    run = 0.0
    for i, e in enumerate(route_edges):
        ok = (i > 0 and run >= skip_first_m and e not in protected and edge_len(e) >= min_len and not e.startswith(":"))
        allowed.append(ok)
        run += edge_len(e)
    cands = []
    for n in lengths:
        for i in range(len(route_edges) - n + 1):
            if all(allowed[i:i + n]):
                cands.append(route_edges[i:i + n])
    rng.shuffle(cands)
    return cands


# ───────────────────────── especificacion del evento ─────────────────────────

@dataclass
class EventSpec:
    kind: str
    seed: int
    checkpoint_s: float
    vehicle: str
    target_node: int
    edges: list
    params: dict = field(default_factory=dict)      # speed (congestion), duration (accident)
    checks: dict = field(default_factory=dict)      # resultado de cada prueba (para auditar el escenario)

    def describe(self):
        c = self.checks
        extra = ""
        if self.kind == "congestion":
            extra = f" (speed limit lowered to {self.params.get('speed')} m/s)"
        elif self.kind == "accident":
            extra = f" (edges blocked for {self.params.get('duration')} s)"
        elif self.kind == "closure":
            extra = " (edges closed for all vehicles)"
        return (f"Event: {self.kind} at checkpoint {self.checkpoint_s:.0f}s on {len(self.edges)} real edge(s){extra}. "
                f"Target {self.vehicle} -> node {self.target_node}: travel time {c.get('time_before_s', 0):.1f}s -> "
                f"{c.get('time_after_s', 0):.1f}s ({c.get('impact_pct', 0):+.1f}%), distance {c.get('dist_before_m', 0):.0f}m -> "
                f"{c.get('dist_after_m', 0):.0f}m. Verified: impact>=threshold, keys connected={c.get('keys_connected')}, "
                f"no vehicle stranded={c.get('none_stranded')}. Affected edges: {', '.join(self.edges)}.")


def save_spec(spec, path):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(asdict(spec), f, indent=2, ensure_ascii=False)


def load_spec(path):
    with open(path, encoding="utf-8") as f:
        return EventSpec(**json.load(f))


# ───────────────────────── aplicar / revertir (TraCI) ─────────────────────────

def _lanes(traci, edge):
    return [f"{edge}_{i}" for i in range(traci.edge.getLaneNumber(edge))]


def snapshot(traci, edges):
    snap = {}
    for e in edges:
        for ln in _lanes(traci, e):
            snap[ln] = (traci.lane.getMaxSpeed(ln), list(traci.lane.getAllowed(ln)), list(traci.lane.getDisallowed(ln)))
    return snap


def restore(traci, snap):
    for ln, (speed, allowed, disallowed) in snap.items():
        traci.lane.setMaxSpeed(ln, speed)
        traci.lane.setAllowed(ln, allowed)          # lista vacia = todos permitidos
        if disallowed:
            traci.lane.setDisallowed(ln, disallowed)


def apply(traci, kind, edges, params):
    for e in edges:
        if kind == "congestion":
            traci.edge.setMaxSpeed(e, float(params["speed"]))
        else:                                        # closure / accident
            traci.edge.setDisallowed(e, ["all"])


def reroute_all(traci, vids=None):
    """Tras cerrar arcos, los vehiculos con rutas que los usan deben desviarse (SUMO conserva sus paradas).
    Es lo mismo en las dos ramas, para que la unica diferencia sea reoptimizar el orden/asignacion de clientes."""
    for vid in (vids if vids is not None else traci.vehicle.getIDList()):
        try:
            traci.vehicle.rerouteTraveltime(vid)
        except Exception:
            pass


class EventController:
    """Mantiene el evento activo y reabre los arcos del accidente cuando termina su duracion."""

    def __init__(self, traci, spec, snap, t_start):
        self.traci, self.spec, self.snap = traci, spec, snap
        self.t_end = t_start + float(spec.params["duration"]) if spec.kind == "accident" else None
        self.closed = self.t_end is not None

    def tick(self, now):
        if self.closed and now >= self.t_end:
            restore(self.traci, self.snap)
            reroute_all(self.traci)
            self.closed = False


# ───────────────────────── pruebas con TraCI ─────────────────────────

def _leg(traci, a, b):
    st = traci.simulation.findRoute(a, b)
    return (list(st.edges), float(st.travelTime), float(st.length))


def none_stranded(traci, vehicles_state, depot_edge, station_edges, fuel_capacity, rate):
    """Cada vehiculo debe poder llegar al deposito o a alguna estacion con su bateria actual."""
    for vid, st in vehicles_state.items():
        if not st.get("max_wh"):
            continue
        battery_units = st["battery_wh"] / st["max_wh"] * fuel_capacity
        reach = False
        for target in [depot_edge] + list(station_edges):
            edges, _, length = _leg(traci, st["current_edge"], target)
            if edges and length / 1000.0 * rate <= battery_units:
                reach = True
                break
        if not reach:
            return False, vid
    return True, None


def select_event(kind, traci, net, vehicles_state, node_edge_of, depot_edge, station_edges, key_edges,
                 rng, seed, checkpoint_s, fuel_capacity, rate, impact_threshold=0.20,
                 congested_speed=2.0, accident_duration=600.0, vclass="passenger", max_tries=60):
    """Selecciona, verifica y deja APLICADO un evento. Devuelve (EventSpec, EventController).
    node_edge_of(n) -> arco de SUMO del nodo n; key_edges: arcos que deben seguir conectados."""
    if kind not in KINDS:
        raise ValueError(f"tipo de evento desconocido: {kind}")
    params = {"speed": congested_speed} if kind == "congestion" else (
        {"duration": accident_duration} if kind == "accident" else {})
    all_keys = set(key_edges) | {st["current_edge"] for st in vehicles_state.values()}
    protected = set(all_keys)
    # puntos que ya estaban aislados ANTES del evento (p. ej. un nodo mal ajustado a la red): no se le atribuyen al evento
    _, base_bad = keys_connected(net, depot_edge, all_keys, set(), vclass)
    base_bad = set(base_bad)
    order = sorted(vehicles_state, key=lambda v: (-len(vehicles_state[v]["pending_nodes"]), v))
    order = [v for v in order if len(vehicles_state[v]["pending_nodes"]) >= 2] or order
    tries = 0
    for vid in order:
        st = vehicles_state[vid]
        target = st["pending_nodes"][0]
        tgt_edge = node_edge_of(target)
        before_edges, t_before, d_before = _leg(traci, st["current_edge"], tgt_edge)
        if not before_edges or t_before <= 0:
            continue
        for seg in candidate_segments(before_edges, lambda e: net.getEdge(e).getLength(), protected, rng):
            tries += 1
            if tries > max_tries:
                raise RuntimeError(f"sin evento valido tras {max_tries} candidatos (tipo={kind}); relajar umbral o cambiar semilla")
            connected = True
            if kind != "congestion":
                _, bad = keys_connected(net, depot_edge, all_keys, set(seg), vclass)
                connected = not (set(bad) - base_bad)
                if not connected:
                    continue
            snap = snapshot(traci, seg)
            apply(traci, kind, seg, params)
            after_edges, t_after, d_after = _leg(traci, st["current_edge"], tgt_edge)
            impact = (t_after - t_before) / t_before if after_edges else float("inf")
            ok_impact = bool(after_edges) and impact >= impact_threshold
            ok_strand, stranded_vid = (True, None)
            if ok_impact and kind != "congestion":
                ok_strand, stranded_vid = none_stranded(traci, vehicles_state, depot_edge, station_edges, fuel_capacity, rate)
            if not (ok_impact and ok_strand):
                restore(traci, snap)
                continue
            if kind != "congestion":
                reroute_all(traci)
            spec = EventSpec(kind=kind, seed=seed, checkpoint_s=checkpoint_s, vehicle=vid, target_node=target,
                             edges=list(seg), params=params,
                             checks={"time_before_s": t_before, "time_after_s": t_after, "dist_before_m": d_before,
                                     "dist_after_m": d_after, "impact_pct": impact * 100.0,
                                     "impact_threshold": impact_threshold, "keys_connected": connected,
                                     "none_stranded": ok_strand, "candidates_tried": tries})
            return spec, EventController(traci, spec, snap, checkpoint_s)
    raise RuntimeError(f"no se encontro un evento valido (tipo={kind}, candidatos probados={tries})")


def apply_spec(spec, traci, net, vehicles_state, depot_edge, key_edges, vclass="passenger"):
    """Re-aplica un evento guardado (segunda rama del experimento), repitiendo la prueba de conectividad."""
    if spec.kind != "congestion":
        all_keys = set(key_edges) | {s["current_edge"] for s in vehicles_state.values()}
        _, base_bad = keys_connected(net, depot_edge, all_keys, set(), vclass)
        _, bad = keys_connected(net, depot_edge, all_keys, set(spec.edges), vclass)
        new_bad = set(bad) - set(base_bad)
        if new_bad:
            raise RuntimeError(f"el evento guardado aisla puntos en este estado: {sorted(new_bad)[:5]}")
    snap = snapshot(traci, spec.edges)
    apply(traci, spec.kind, spec.edges, spec.params)
    if spec.kind != "congestion":
        reroute_all(traci)
    return EventController(traci, spec, snap, spec.checkpoint_s)
