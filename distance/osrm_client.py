"""
osrm_client.py — Cliente para el servicio /table de OSRM (distancia y tiempo reales de carretera).

Usa routingpy en vez de llamar a la API HTTP a mano. Requiere un servidor OSRM
propio corriendo (Docker + extracto de la región, ver osrm_data/) — no se usa
el servidor público de demo, no es apto para experimentos reproducibles.

Por qué una función bulk y no un branch en distance_type.calculate_distance():
    OSRM se consulta con UNA sola llamada al servicio /table para toda la matriz.
    Llamarlo par a par (como hacen las métricas geométricas) implicaría cientos
    de peticiones HTTP por instancia — inviable en tiempo.

Unidades devueltas (consistentes con el resto del repo, que trabaja en km):
    distance_matrix -> km (float)     [OSRM devuelve metros; se convierte aquí]
    time_matrix     -> segundos (float), tal cual lo entrega OSRM

Se cachea en disco por instancia (hash de las coordenadas + perfil) para no
volver a consultar el servidor en cada corrida de experimentos.
"""

import hashlib
import json
import os

from routingpy.routers import OSRM

DEFAULT_BASE_URL = os.environ.get("OSRM_BASE_URL", "http://localhost:5000")
CACHE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "instances_data", "osrm_cache"
)

# Costo (en km) asignado a un par de nodos que OSRM reporta como inalcanzable
# por carretera (entrada null en la respuesta). Debe ser grande para que el
# solver evite ese arco, pero finito para no romper el modelo.
UNREACHABLE_PENALTY_KM = 1_000_000
UNREACHABLE_PENALTY_SECONDS = 1_000_000


def _cache_key(osrm_coords, profile):
    payload = json.dumps({"locations": osrm_coords, "profile": profile}, sort_keys=True)
    return hashlib.sha1(payload.encode("utf-8")).hexdigest()


def _load_cache(key):
    path = os.path.join(CACHE_DIR, f"{key}.json")
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    return None


def _save_cache(key, result):
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, f"{key}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(result, f)


def get_osrm_matrix(locations, base_url=None, profile="driving", use_cache=True):
    """
    locations: lista de (lat, lon) en el mismo orden que los nodos del problema
               (0 = depósito, luego clientes, luego estaciones de carga).

    Devuelve: {'distance_matrix': [[km, ...], ...], 'time_matrix': [[segundos, ...], ...]}
    """
    base_url = base_url or DEFAULT_BASE_URL
    # OSRM espera pares [lon, lat] — al revés de como se guardan en el resto del repo.
    osrm_coords = [[lon, lat] for lat, lon in locations]

    key = _cache_key(osrm_coords, profile)
    if use_cache:
        cached = _load_cache(key)
        if cached is not None:
            return cached

    client = OSRM(base_url=base_url)
    matrix = client.matrix(locations=osrm_coords, profile=profile, annotations=("duration", "distance"))

    if matrix.distances is None or matrix.durations is None:
        raise RuntimeError(
            f"El servidor OSRM en {base_url} no devolvió distancias/duraciones. "
            "Verifica que esté corriendo (osrm-routed) y que las coordenadas caigan "
            "dentro de la región del mapa cargado."
        )

    distance_matrix = [
        [(meters / 1000.0) if meters is not None else UNREACHABLE_PENALTY_KM for meters in row]
        for row in matrix.distances
    ]
    time_matrix = [
        [seconds if seconds is not None else UNREACHABLE_PENALTY_SECONDS for seconds in row]
        for row in matrix.durations
    ]

    result = {"distance_matrix": distance_matrix, "time_matrix": time_matrix}
    if use_cache:
        _save_cache(key, result)
    return result
