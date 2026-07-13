# Simulación SUMO del EVRP

Convierte la solución final del EVRP (rutas por vehículo, con paradas en
estaciones de carga) en una simulación de tráfico real sobre calles reales
de OpenStreetMap, usando [SUMO](https://sumo.dlr.de/).

Todo lo de esta carpeta es independiente del resto del proyecto (`problem/`,
`distance/`, `instance/`): solo lee, como texto plano, un archivo de
instancia `.txt` y un archivo de solución `.txt` (los que ya produce
`evrp.py`). No requiere tocar ni importar ese código.

## Prerrequisitos

1. **Docker Desktop** — solo si necesitas generar una solución EVRP nueva
   con distancias reales de OSRM (ver raíz del repo). Si ya tienes un
   archivo de solución (`problem/osrm/...`), no hace falta para esta parte.
2. **SUMO** ≥ 1.26 instalado, con la variable de entorno `SUMO_HOME`
   apuntando a la carpeta de instalación (ej. `D:\SUMO`) y `SUMO_HOME\bin`
   en el `PATH` (para que `sumo`, `sumo-gui`, `netconvert`, `duarouter` se
   reconozcan directo en la terminal).
   Descarga: https://sumo.dlr.de/docs/Downloads.php
3. Dependencias de Python (ya están en `requirements.txt` de la raíz):
   ```bash
   pip install -r requirements.txt
   ```
   (agrega `osmium` y `pyproj`, específicos de esta carpeta).

## Fase 1 — Red vial real (una sola vez, ~30 min)

Descarga el extracto de OpenStreetMap de Quebec (mismo dato base que usa
OSRM — ver `osrm_data/` en la raíz del repo) y colócalo en
`osrm_data/quebec-latest.osm.pbf` si no lo tienes ya:

```bash
curl -L -o osrm_data/quebec-latest.osm.pbf https://download.geofabrik.de/north-america/canada/quebec-latest.osm.pbf
```

Luego, recorta la zona de la instancia que vas a simular y construye la red:

```bash
python simulation/extract_osm_bbox.py --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt
python simulation/build_real_network.py --osm-file simulation/sumo_network/clipped.osm
```

- `extract_osm_bbox.py` tarda ~25-30 min (dos pasadas sobre el `.pbf` de
  ~1.1 GB) — es normal, no se cuelga. Genera `simulation/sumo_network/clipped.osm`.
- `build_real_network.py` tarda ~30-40s. Genera `simulation/sumo_network/network.net.xml`
  (la red real de Quebec City — no se sube a git por tamaño, ~180MB).

Este paso solo hay que repetirlo si cambias de instancia/ciudad. El
resultado se reutiliza en las fases siguientes.

## Fase 2 — Rutas del EVRP sobre la red real

```bash
python simulation/build_routes.py \
  --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
  --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
```

Genera `simulation/sumo_scenario/{trips.xml, routes.rou.xml, scenario.sumocfg}`.

Visualizar:

```bash
sumo-gui -c simulation/sumo_scenario/scenario.sumocfg
```

Dale ▶️ (play) o barra espaciadora — arranca pausado. Deberías ver los
vehículos de la solución EVRP recorriendo calles reales y deteniéndose en
cada cliente/estación de su ruta.

> Nota: no todos los vehículos garantizan tener ruta válida — si dos paradas
> consecutivas caen en calles sin conexión directa (ej. sentido único),
> `duarouter` descarta ese vehículo y lo reporta en consola. Es una
> limitación conocida del snapping automático a la red real, no un error.

## Fase 3 — Tráfico de fondo (PENDIENTE, no ejecutar tal cual)

`simulation/build_background_traffic.py` existe pero **no se ha logrado
correr con éxito**: la generación de tráfico de fondo con `randomTrips.py`
sobre la red completa (79k arcos) se colgó ~10 horas en el intento. Sospecha
principal: falta el paquete `rtree` (indexado espacial), forzando búsquedas
por fuerza bruta. Antes de reintentar:

```bash
pip install rtree
```

y probar primero con una ventana de tiempo corta (`--end 600 --period 5`)
para verificar que termina en un tiempo razonable antes de escalar.

## Fase 4 — Batería real (Battery Device de SUMO)

```bash
python simulation/build_battery_scenario.py \
  --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
  --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
```

Corre en dos pasadas (calibración + escenario final) y termina imprimiendo
una tabla comparando el consumo lineal que asume OR-Tools (`fuel_consumption_rate`)
contra el consumo real que calcula SUMO con física de vehículo (masa, arrastre
aerodinámico, fricción de rodadura), además de cuánta energía cargó cada
vehículo en las estaciones que sí usó.

Genera `simulation/sumo_scenario/{routes_battery.rou.xml, additional_battery.add.xml, scenario_battery.sumocfg}`.

Visualizar con batería activa:

```bash
sumo-gui -c simulation/sumo_scenario/scenario_battery.sumocfg
```

> Limitación conocida: el SOC final de un vehículo puede reportarse por
> encima de 100% — la parada de carga tiene duración fija (300s) y SUMO no
> la corta sola al llegar al tope. Una versión más precisa cortaría la
> parada dinámicamente vía TraCI (no implementado en esta versión).

## Qué archivos son código y cuáles son generados

| Carpeta/archivo | Es código (va a git) | Se genera al correr |
|---|---|---|
| `simulation/*.py` | ✅ | — |
| `simulation/sumo_network/` | ❌ (gitignored, ~180MB) | Fase 1 |
| `simulation/sumo_scenario/` | ❌ (gitignored) | Fases 2 y 4 |
| `osrm_data/*.osm.pbf` | ❌ (gitignored, ~1.1GB) | Descarga manual |

Todo lo generado se reproduce corriendo los scripts en orden — no hace
falta bajarlo de ningún lado más que este repo.
