# Simulación SUMO del EVRP

Convierte la solución final del EVRP (rutas por vehículo, con paradas en
estaciones de carga) en una simulación de tráfico real sobre calles reales
de OpenStreetMap, usando [SUMO](https://sumo.dlr.de/).

Todo lo de esta carpeta es independiente del resto del proyecto (`problem/`,
`distance/`, `instance/`): solo lee, como texto plano, un archivo de
instancia `.txt` y un archivo de solución `.txt` (los que ya produce
`evrp.py`). No requiere tocar ni importar ese código.

## Menú interactivo (opcional)

En vez de escribir cada comando de las fases de abajo a mano, puedes correr:

```bash
python simulation/run.py
```

Muestra un menú numerado (Fase 1, 2, 4, EVRP dinámico, diagnósticos, abrir
`sumo-gui`) y pide solo los parámetros que cambian (instancia, solución,
checkpoint), con la instancia piloto ya precargada como valor por defecto —
das Enter para aceptarla. Internamente llama a los mismos scripts
documentados abajo, con los mismos flags.

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

- `extract_osm_bbox.py` usa un margen de **0.1° (~10km)** por defecto alrededor
  de la instancia. Tarda entre 5 y 30 min según el tamaño del área (dos pasadas
  sobre el `.pbf` de ~1.1 GB) — es normal, no se cuelga. Genera
  `simulation/sumo_network/clipped.osm`.
  **No bajes el margen por debajo de 0.1°**: con un margen chico (se probó con
  0.03°/~3km) algunos clientes de la instancia quedaron en "islas" desconectadas
  de la red — la única calle que los conectaba con el resto de la ciudad caía
  fuera del recorte, y `duarouter` los descartaba (ver Fase 2 más abajo).
- `build_real_network.py` tarda ~1 min. Genera `simulation/sumo_network/network.net.xml`
  (la red real de Quebec City — no se sube a git por tamaño, ~340MB).

Este paso solo hay que repetirlo si cambias de instancia/ciudad. El
resultado se reutiliza en las fases siguientes.

## Fase 2 — Rutas del EVRP sobre la red real

```bash
python simulation/build_routes.py \
  --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
  --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt"
```

> `--instance` y `--solution` son opcionales: si no se pasan, usan por
> defecto la instancia piloto (`quebec_40c_4ev_6cs`). Esto permite correr
> el script directo con el botón ▶️ **Run** de PyCharm, sin configurar
> argumentos ni "Working directory" — los defaults se calculan desde la
> ubicación del propio archivo, no desde el directorio de trabajo.

Genera `simulation/sumo_scenario/{trips.xml, routes.rou.xml, scenario.sumocfg}`.

Visualizar:

```bash
sumo-gui -c simulation/sumo_scenario/scenario.sumocfg
```

Dale ▶️ (play) o barra espaciadora — arranca pausado. Deberías ver los
vehículos de la solución EVRP recorriendo calles reales y deteniéndose en
cada cliente/estación de su ruta.

> El snapping (`snap_route_sequential` en `build_routes.py`) elige, para cada
> parada, el arco real más cercano que además tenga camino verificado
> (`net.getShortestPath`) desde la parada anterior — no solo el más cercano a
> ciegas. Si aun así un vehículo queda sin ruta válida, casi siempre es porque
> el recorte de la Fase 1 dejó una zona desconectada (ver el aviso sobre el
> margen arriba), no un problema de este script.

## Fase 3 — Tráfico de fondo

```bash
python simulation/build_background_traffic.py \
  --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
  --end 3000 --period 2
```

Genera tráfico de fondo aleatorio con `randomTrips.py` sobre la red completa
y lo combina con las rutas del EVRP en `scenario.sumocfg`, luego compara el
tiempo real de cada vehículo `ev_*` (con tráfico) contra el tiempo que había
estimado OSRM.

- `--end`/`--period` controlan la cantidad de autos de fondo: aprox.
  `end / period` vehículos (ej. `--end 2000 --period 2` → ~1000). No hay un
  tope fijo en el código, solo estos dos parámetros.
- **Historial**: la primera vez que se intentó correr sobre la red completa
  (79k arcos) se colgó ~10 horas. La causa raíz era la falta del paquete
  `rtree` (indexado espacial) — sin él, `sumolib` cae a búsqueda por fuerza
  bruta. Ya está en `requirements.txt` y verificado (probado con `--end 600
  --period 5` → 120 autos, y `--end 2000 --period 2` → 1000 autos, ambos en
  segundos, no horas). Asegúrate de correr los scripts con el `python` del
  `.venv` del proyecto (`.venv/Scripts/python.exe` en Windows) — un `python`
  del sistema sin `rtree` instalado reproduce el fallback lento.
- **Limitación conocida — no hay semáforos**: la red generada en la Fase 1
  no tiene ningún `<tlLogic>` (se verificó: 0 uniones tipo `traffic_light`,
  todas son `priority`/`right_before_left`/`dead_end`). `netconvert` recibe
  `--tls.guess-signals` pero no infirió ninguno para esta zona (posiblemente
  descartados por `--tls.discard-simple`/`--tls.join`, o el extracto OSM no
  trae suficientes tags `highway=traffic_signals`). Efecto práctico: la
  congestión que se ve en esta fase es solo por volumen/capacidad de la vía,
  no por tiempos de ciclo de semáforo — pendiente de investigar si hace
  falta para la tesis.
- **Segundo cuelgue (real, no de caché) y su fix**: al probar `--end 6000`
  la corrida quedó colgada >1h; con `--end 3000` se confirmó la causa exacta
  revisando el `tripinfo_phase3.xml` a medio escribir (XML sin cerrar =
  prueba de que `sumo` seguía vivo): de 1504 vehículos insertados, 1493
  terminaban bien (los 4 EV incluidos) y **11 vehículos de fondo quedaban
  embotellados sin terminar nunca**. La causa era la combinación de
  `time-to-teleport=-1` (desactiva el rescate de SUMO para vehículos
  atascados) + sin semáforos (arriba) + ningún `<end>` de simulación en el
  `.sumocfg` — unos pocos vehículos en un embotellamiento real en una
  intersección sin control dejaban la simulación corriendo indefinidamente
  esperando a que terminaran. Fix aplicado en `build_background_traffic.py`:
  `time-to-teleport` a 300s (el default de SUMO, en vez de `-1`) y un
  `<end>` explícito en el `.sumocfg` (`--end` + 1800s de margen) como límite
  duro adicional, más un timeout de 15 min en cada subproceso (`randomTrips.py`
  y `sumo`) que corta con un mensaje claro en vez de colgarse en silencio.
  Verificado con `--end 3000 --period 2` (1500 autos) y `--end 6000 --period 2`
  (3000 autos) tras el fix: ambos terminan sin colgarse.
- **Conclusión: el tráfico de fondo aleatorio no genera congestión detectable
  a esta escala.** Con 1500 y con 3000 vehículos de fondo, los tiempos de los
  4 EV salen prácticamente idénticos (ej. ev_2: -30.0% vs OSRM en ambos
  casos) — duplicar el tráfico no cambió nada. Tiene sentido: son viajes
  aleatorios repartidos sobre los ~79k arcos de toda Quebec City, así que la
  probabilidad de que caigan justo en las calles que usan los EV es baja y
  el efecto por arco es casi nulo. **Para demostrar congestión real (ej. en
  el video para la tutora), el evento de congestión controlado del Punto 3/4
  (`traci_congestion_trigger.py` / el bloque ANTES-DESPUÉS del orquestador
  dinámico, ver más abajo) es el mecanismo confiable — ya está verificado
  con un impacto medido de +34.4%.**

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

## EVRP dinámico (re-planeación por horizonte rodante)

Objetivo: reaccionar a cambios (congestión real) re-resolviendo con OR-Tools
solo la parte de la ruta que falta, en vez de un solver dedicado (ACO/GA) —
el mismo motor de siempre (Guided Local Search, Tabu Search, etc.), solo que
se lo llama varias veces con datos actualizados en vez de una sola vez con
datos estáticos. Enfoque respaldado por Ünal et al. (2025), que usa un
patrón de reacción similar (heurística de reasignación + SUMO para tráfico)
en vez de un metaheurístico poblacional nuevo.

Requiere haber corrido la **Fase 4** primero (necesita `scenario_battery.sumocfg`
y su Battery Device activo, para tener batería real que leer del checkpoint).

### Los 4 pasos del ciclo

1. **`vehicle_states` en `evrp.py`** — parámetro opcional de `execute()`: una
   lista (largo `num_vehicles`) con `{'start_node', 'initial_fuel', 'initial_load'}`
   por vehículo, para que arranque donde está realmente (no siempre depósito +
   batería llena). Probado contra las 20 heurísticas/metaheurísticas de
   OR-Tools del proyecto.
2. **`data_override` en `evrp.py`** — parámetro opcional de `execute()`: un
   dict de datos ya construido (con matriz de distancia/tiempo propia), que
   se usa tal cual en vez de leer un archivo de instancia con `process_files()`.
   Necesario para pasarle a OR-Tools una matriz medida en vivo por SUMO, no
   la estática de OSRM.
3. **`simulation/traci_state_probe.py`** — pausa una simulación SUMO en un
   checkpoint y extrae, por vehículo activo, posición real, batería restante
   y qué paradas ya cumplió vs. cuáles faltan (script de solo lectura, no
   modifica nada, útil para inspeccionar sin re-planear):
   ```bash
   python simulation/traci_state_probe.py \
     --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
     --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
     --config simulation/sumo_scenario/scenario_battery.sumocfg \
     --checkpoint-time 1200
   ```
4. **`simulation/traci_congestion_trigger.py`** — fuerza congestión real en
   un tramo (baja el límite de velocidad vía TraCI) y confirma, con
   `traci.simulation.findRoute()`, que el tiempo de viaje sube de verdad. Es
   el script donde se probó esto por primera vez, aislado. **Ya no hace falta
   correrlo aparte** — `dynamic_replanning.py` (más abajo) incluye la misma
   prueba antes/después integrada en su única corrida. Se deja disponible
   por si se quiere probar la retroalimentación sola, sin el resto del ciclo:
   ```bash
   python simulation/traci_congestion_trigger.py \
     --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
     --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
     --config simulation/sumo_scenario/scenario_battery.sumocfg \
     --checkpoint-time 1200
   ```

### El orquestador completo: `simulation/dynamic_replanning.py`

Une los 4 pasos anteriores en un solo flujo — **un solo comando para toda la demo**
(incluye el antes/después de la congestión impreso en consola, no hace falta
correr `traci_congestion_trigger.py` aparte para tener esa evidencia):

```bash
python simulation/dynamic_replanning.py \
  --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt \
  --solution "problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC/quebec_40c_4ev_6cs.txt" \
  --checkpoint-time 1200
```

Qué hace, en orden (y qué buscar en la consola si es para mostrar la
retroalimentación, ej. en un video para tu tutora):
1. Corre SUMO con TraCI hasta el checkpoint y extrae el estado real de cada
   vehículo activo con paradas pendientes.
2. Fuerza un evento de congestión controlado cerca de uno de ellos, e imprime
   un bloque **"ANTES / DESPUÉS"** con el tiempo de viaje real medido por SUMO
   antes y después del evento (ej. `+119.2s (+34.4%)`) — **esta es la prueba
   concreta de la retroalimentación**, todo en la misma corrida.
3. Arma una sub-instancia (posición actual de cada vehículo activo + el pool
   de sus clientes pendientes + todas las estaciones de carga), con una
   matriz de distancia/tiempo consultada **en vivo** a SUMO (`findRoute()`,
   con la congestión ya reflejada) — no la matriz estática original de OSRM.
   Se imprime `"Consultando matriz de tiempo/distancia VIVA con SUMO..."`.
4. Llama a `evrp.execute()` con `data_override` + `vehicle_states` para
   re-optimizar esa sub-instancia (mismo motor OR-Tools, ninguna heurística
   nueva).
5. Reconstruye la nueva solución como ruta real (encadenando `findRoute()`
   entre paradas consecutivas) e inyecta esa ruta y sus paradas al vehículo
   activo.
6. Exporta el plan resultante a un escenario **independiente** (sin TraCI):
   `simulation/sumo_scenario/scenario_dynamic_replan.sumocfg`.

Para el cierre visual (opcional, segundo y último comando), visualizar el
resultado desde el segundo 0, ya con el plan re-optimizado:

```bash
sumo-gui -c simulation/sumo_scenario/scenario_dynamic_replan.sumocfg
```

> **Por qué se exporta a un escenario aparte en vez de dejar la ventana de
> TraCI abierta**: un `sumo-gui` manejado por TraCI no se puede "soltar" para
> que quede libre — si el script termina sin un cierre limpio de la conexión,
> SUMO lo toma como un error (`peer shutdown`) y se apaga solo. Por eso el
> flujo real es: corre el orquestador (headless, rápido) → abre el `.sumocfg`
> exportado por separado, a tu ritmo.

> Nota de escala: en la instancia piloto hay un cliente bastante alejado del
> depósito (~20 km de detour real) — si al abrir la ventana solo ves 1-2
> vehículos, usa "zoom to fit" antes de darle play; los otros pueden estar
> circulando fuera del área visible por defecto. Confirmado con `tripinfo`
> headless que los 4 vehículos sí completan su ruta sin errores.

## Limitaciones conocidas

### Cada estación de carga se puede visitar una sola vez en toda la solución

En el modelo actual (`problem/execute/evrp.py`), cada estación de carga es
**un solo nodo** en el grafo que le pasamos a OR-Tools. Como cada nodo se
visita a lo sumo una vez (restricción estándar de VRP), esto significa que
una misma estación física no puede ser usada por dos vehículos distintos, ni
por el mismo vehículo dos veces, dentro de una misma solución.

El paper de referencia (Anastasiadou et al., ACO-DEVRP) resuelve esto
"duplicando" cada estación en β_i copias (mismo lugar físico, nodos
distintos para el solver), permitiendo múltiples visitas. Este proyecto no
implementa esa duplicación todavía.

**Evaluación de qué tan grande sería el cambio** (hecha 2026-07-29, sin
implementar aún):

- El cambio quedaría casi contenido en `instance/import_data.py` →
  `read_file_evrp()`: en vez de agregar un nodo por estación, agregar β
  copias (mismas coordenadas, demanda 0, mismo nombre en
  `charging_station_names` para que la solución se lea igual de claro).
- `evrp.py` **no necesitaría cambios**: `add_capacity_constraints` y
  `add_fuel_constraints` ya iteran `data['charging_stations']` como una
  lista genérica, sin asumir una sola aparición por estación. La fórmula
  `num_clients = num_locations - len(charging_stations) - 1` tampoco se ve
  afectada por cuántas copias haya.
- Los scripts de `simulation/` **no necesitarían cambios**: leen el archivo
  `.txt` de instancia original (una estación física = una línea), no la
  representación interna con copias — esa expansión sería un detalle
  interno de `read_file_evrp`, invisible para SUMO.
- Sí haría falta decidir β (cuántas copias por estación). El valor del
  paper (`β_i = 2 × num_clientes`) es un peor caso muy generoso pensado
  para instancias pequeñas; para las instancias de este proyecto
  (10-90 clientes) infla demasiado el modelo y podría ralentizar a
  OR-Tools sin necesidad. Un valor más razonable para esta escala sería
  `β = num_vehicles` (en el peor caso, cada vehículo visita esa estación
  una vez).
- Nota aparte: el orquestador de re-planeación dinámica
  (`dynamic_replanning.py`) arma su propia sub-instancia a mano, sin pasar
  por `read_file_evrp` — si se quiere que las copias también apliquen ahí,
  sería un segundo cambio pequeño e independiente.

**Conclusión**: cambio acotado, mayormente en un solo archivo. Pendiente de
implementar hasta que se decida si vale la pena para el alcance de la tesis.

## Qué archivos son código y cuáles son generados

| Carpeta/archivo | Es código (va a git) | Se genera al correr |
|---|---|---|
| `simulation/*.py` | ✅ | — |
| `simulation/sumo_network/` | ❌ (gitignored, ~340MB) | Fase 1 |
| `simulation/sumo_scenario/` | ❌ (gitignored) | Fases 2, 4 y re-planeación dinámica |
| `osrm_data/*.osm.pbf` | ❌ (gitignored, ~1.1GB) | Descarga manual |
| `problem/osrm/solutions_evrp_0/.../dynamic_replan.txt` | ❌ (solución temporal del sub-problema) | `dynamic_replanning.py` |

Todo lo generado se reproduce corriendo los scripts en orden — no hace
falta bajarlo de ningún lado más que este repo.
