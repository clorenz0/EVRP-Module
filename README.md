# EVRP-Module

Resolución del problema de ruteo de vehículos eléctricos (EVRP) y de su variante dinámica (DEVRP) con OR-Tools y
ACO, simulación del resultado en tráfico real con SUMO y explicabilidad (XAI) del proceso. Todo desde Python.

- **EVRP (estático):** se calcula un plan completo antes de salir. Solo optimización.
- **DEVRP (dinámico):** el plan se ejecuta en SUMO, ocurre un evento (congestión) y se **re-planifica** el resto de
  las rutas con el estado real de los vehículos. Necesita optimización y simulación.

## Contenido

1. [Qué quiero hacer](#1-qué-quiero-hacer) (tabla de decisión)
2. [Instalación y requisitos](#2-instalación-y-requisitos)
3. [Modelo común](#3-modelo-común)
4. [EVRP: solo optimización](#4-evrp-solo-optimización)
5. [Comparar algoritmos: ranking de Friedman](#5-comparar-algoritmos-ranking-de-friedman)
6. [DEVRP: optimización + simulación](#6-devrp-optimización--simulación)
7. [Explicabilidad (XAI)](#7-explicabilidad-xai)
8. [Flujo completo de extremo a extremo](#8-flujo-completo-de-extremo-a-extremo)
9. [Estructura del repositorio](#9-estructura-del-repositorio)
10. [Reproducibilidad y verificación](#10-reproducibilidad-y-verificación)

---

## 1. Qué quiero hacer

| Objetivo | Secciones | Necesita |
|---|---|---|
| Resolver EVRP con OR-Tools | 4.1 | Python, OSRM |
| Resolver EVRP con ACO solo | 4.2 | Python, OSRM |
| Resolver EVRP con ACO + heurística de OR-Tools | 4.3 | Python, OSRM |
| Comparar todos los algoritmos | 5 | resultados de 4.1 a 4.3 |
| Simular una solución en SUMO (sin re-planificar) | 6.1 a 6.3 | + SUMO |
| DEVRP con OR-Tools como re-planificador | 6.4 | + SUMO |
| DEVRP con ACO o ACO + heurística como re-planificador | 6.5 | + SUMO |
| Explicar la optimización | 7.1 | resultados de ACO, scikit-learn, shap |
| Explicar la simulación y la retroalimentación | 7.2 y 7.3 | reportes de 6.4 o 6.5 |
| Todo | 8 | todo lo anterior |

---

## 2. Instalación y requisitos

```bash
python -m venv .venv
.venv\Scripts\activate                     # Windows
pip install -r requirements.txt            # OR-Tools, routingpy, osmium, pyproj, rtree
pip install -r requirements_xai.txt        # solo para la explicabilidad (scikit-learn, shap, pandas, matplotlib)
```

Versiones de referencia para XAI: scikit-learn 1.9.1, shap 0.53.0, matplotlib 3.11.2.

**OSRM** (distancias y tiempos reales de carretera; obligatorio con `DistanceType.OSRM`):

```bash
docker compose -f docker-compose.osrm.yml up -d      # servidor en http://localhost:5000
docker compose -f docker-compose.osrm.yml down       # para detenerlo
```

Requiere el mapa procesado en `osrm_data/` (`quebec-latest.osrm`). Las matrices se guardan en
`instances_data/osrm_cache/`: una vez calculadas, no hace falta el servidor para repetir una instancia.

**SUMO** (solo secciones 6 y 7.2/7.3): SUMO ≥ 1.26 con `SUMO_HOME` definido y `SUMO_HOME\bin` en el `PATH`.
Detalle de las fases de simulación en [`simulation/README.md`](simulation/README.md).

Todos los comandos se ejecutan desde la raíz del repositorio.

---

## 3. Modelo común

- **Objetivo único:** minimizar la distancia total recorrida (más una penalización enorme por cliente sin atender,
  que solo actúa si la instancia es infactible). No se mezcla ningún otro término.
- **Nodos:** 0 = depósito, `1..N` = clientes, `N+1..N+M` = estaciones de carga.
- **Batería:** consumo lineal `consumo × distancia`. Cada recarga lleva la batería al máximo. Ninguna ruta puede
  quedar con batería negativa, incluido el regreso al depósito. Valores de las pruebas: batería 100, consumo 3,0 con
  OSRM (6,0 con Haversine).
- **Estaciones reutilizables:** una estación puede visitarse varias veces en una solución (por el mismo vehículo o
  por vehículos distintos). En ACO no tiene tope. En OR-Tools cada estación se reemplaza por `station_copies`
  nodos idénticos (por defecto 3, activados en orden para evitar simetrías). En los archivos de solución las copias
  salen con el id de la estación original. Con `station_copies=1` se recupera el modelo anterior (una sola visita).
- **Reglas de movimiento:** del depósito no se va directo a una estación, y de una estación no se va directo a otra
  (tras recargar, el vehículo va a un cliente o al depósito). En OR-Tools se marcan en `data['forbidden_arcs']`
  (`forbid_depot_to_station`, `forbid_station_chains`; solo entre estaciones distintas) y el evaluador de distancia les
  pone un costo enorme (la matriz y la distancia reportada no cambian). Quitarlos del dominio de `NextVar` filtra bien
  pero la propagación posterior rechaza muchos movimientos de la búsqueda local y, con muchas estaciones, OR-Tools tarda
  decenas de segundos en la primera solución o no la encuentra (`EVRP_FORBID_BY_COST=0` lo reactiva). En ACO son reglas
  de factibilidad. Dos copias seguidas de la misma estación (0 km entre ellas) se
  fusionan en una sola parada al guardar. El orden entre copias no es una restricción: se rompe la simetría con un
  desempate de k metros en el costo de llegar a la copia k (la restricción `ActiveVar(k+1) <= ActiveVar(k)` junto con
  la prohibición de saltos entre copias dejaba a OR-Tools sin solución en las instancias grandes; ver
  `diagnostico_ortools.py`, variante `orden_por_cp`). En la re-planificación se
  exceptúan las posiciones de arranque de los vehículos, que sí pueden ir directo a una estación.
- **Unidades:** OR-Tools exige costos enteros, así que la optimización trabaja en **metros enteros**
  (`utils/units.py`, `DIST_SCALE = 1000`; antes se truncaba a km enteros y se perdían ~0,5 km por arco, lo que hacía
  gratis los desvíos a estaciones). Los `.txt` se escriben en km con 3 decimales, sin redondear a enteros, y el
  `Objective` coincide con la distancia real (`Real distance (km)`).
- **Control de clientes:** cada solución incluye `Customers served: N/N` y `Dropped customers: 0`. Solo si hay
  clientes sin atender aparece además `Dropped clients: [...]`.
- **Validación de factibilidad:** al guardar, OR-Tools y ACO comprueban de forma independiente (`utils/feasibility.py`)
  que cada cliente se atiende una vez, que las rutas empiezan y terminan donde deben, que no se supera la capacidad,
  que la batería no queda negativa, que no hay saltos depósito → estación ni estación → estación y que no se excede
  `Station copies`. El resultado queda en el `.txt` como `Feasibility check: PASSED` o `FAILED` con el detalle, y se
  imprime un `WARNING` en consola.
- **Formato de solución:** OR-Tools y ACO escriben el mismo `.txt` en
  `problem/osrm/solutions_evrp_<n>/solutions_<algoritmo>/<instancia>.txt`, que es lo que leen la simulación, el
  ranking y la explicabilidad.
- **Instancias:** `instances_data/evrp_instances/quebec_<N>c_<V>ev_<S>cs.txt`: 30, 40, 50, 60 y 90 clientes (originales)
  y 70, 100, 150, 220 y 350 clientes (grandes, con estaciones reales de carga rápida de AFDC, `tipo_instalacion` =
  `DCFC_AFDC`). La zona (red SUMO) de cada una está en `INSTANCE_NET_FILE` de `simulation/run.py`.

---

## 4. EVRP: solo optimización

### 4.1 OR-Tools (heurística + metaheurística)

Se configura en `run.py`:

```python
execute(
    problem_type=ProblemType.EVRP, instance=InstanceType.EVRP, distance_type=DistanceType.OSRM,
    time_limit=10, executions=1,
    vehicle_maximum_travel_distance=100,   # batería
    vehicle_speed=3.0,                     # consumo por km
    station_copies=3,                      # visitas posibles por estación (1 = una sola)
    heuristic=HeuristicType.PATH_CHEAPEST_ARC,
    metaheuristic=MetaheuristicType.GUIDED_LOCAL_SEARCH,
)
```

```bash
python run.py
```

- Con `heuristic` y `metaheuristic` en `None` se ejecutan **todas las combinaciones**.
- La instancia se elige en la línea `INSTANCE` de `run.py`. El límite de tiempo es de 10 s hasta 90 clientes y
  crece con el tamaño (`TIME_BY_INSTANCE`: 100c = 60 s, 150c = 60 s, 220c = 120 s, 350c = 180 s); con menos tiempo, Guided Local
  Search puede devolver todos los clientes descartados en las instancias grandes.
- Resultado: `problem/osrm/solutions_evrp_0/solutions_<HEURISTICA>_and_<METAHEURISTICA>/<instancia>.txt`.
- Cada archivo informa `Station copies`, `Total recharges` y `Unnecessary charging stops` (diagnóstico de paradas de
  carga que se podrían quitar; no altera el objetivo).

### 4.2 ACO solo

Todo se configura al inicio de `run_aco.py` (`ACO_PARAMS`, `HEURISTIC`, `INSTANCES`, `SEEDS`, `TIME_LIMIT`,
`TIME_BY_INSTANCE`; `--time` anula los tiempos por instancia):

```python
HEURISTIC = "ACO"                         # solo ACO
INSTANCES = "all"                         # o ["quebec_50c_5ev_5cs"]
SEEDS = [1, 2, 3, 4, 5, 6]
TIME_LIMIT = 10
```

```bash
python run_aco.py                         # usa lo que dice el archivo
python run_aco.py --heuristic ACO         # solo ACO, sin importar el archivo
python run_aco.py --instance quebec_50c_5ev_5cs --seed 3 --time 20
```

- Resultado por semilla: `problem/osrm/solutions_evrp_<1000+semilla>/solutions_ACO/`.
- La mejor semilla de cada instancia se copia a `problem/osrm/solutions_evrp_best/solutions_ACO/`.
- ACO es estocástico: la semilla fija el resultado. Para el ranking se usa una sola semilla.
- Parámetros principales (`ACO_PARAMS`): `alpha` (`None` = automático: 1,0 hasta 50 nodos, 2,0 si hay más), `beta`,
  `gamma`, `rho`, `num_ants`, `local_search_top`, `ils_per_iter`, `pheromone_scheme` (`'mmas'` o `'rank_as'`).

### 4.3 ACO + heurística constructiva de OR-Tools

La heurística construye una solución inicial que entra a ACO como una hormiga más. ACO sigue siendo independiente:
no se toca su código. No se combina con metaheurísticas de OR-Tools (GLS, Tabu, SA): ACO cumple ese papel.

```python
HEURISTIC = None                                         # ACO solo + ACO con CADA heurística
HEURISTIC = HeuristicType.SAVINGS                        # solo ACO + SAVINGS
HEURISTIC = [HeuristicType.SAVINGS, HeuristicType.PATH_CHEAPEST_ARC]
```

```bash
python run_aco.py --heuristic SAVINGS
python run_aco.py --heuristic all
```

- Cada combinación es **una sola corrida** (una semilla sorteada de `SEEDS`), igual que cada combinación de OR-Tools.
- Resultado junto a las de OR-Tools: `problem/osrm/solutions_evrp_0/solutions_<HEURISTICA>_and_ACO/`, con
  `Heuristic: <HEURISTICA>` en el encabezado.
- Si la heurística no encuentra una solución completa, ACO arranca sin ella.

### 4.4 Orden de resultados y trazas

- Nada se sobreescribe: antes de ejecutar, `run_aco.py` archiva por fecha lo que ya existía en
  `problem/osrm/archivo/<AAAA-MM-DD>/`.
- Cada ejecución de `run_aco.py` y `friedman_ranking.py` guarda su salida en `problem/osrm/trazas/`.
- Para empezar una campaña limpia: `python archivar_resultados.py --all --dry-run` y luego sin `--dry-run`.
  Hay que hacerlo, por ejemplo, tras cambiar el modelo de OR-Tools (`station_copies`) o las unidades (los archivos
  anteriores al modelo en metros tienen `Objective` entero y el ranking los excluye como `LEGACY`).

---

## 5. Comparar algoritmos: ranking de Friedman

Compara ACO, ACO + heurística y todas las combinaciones de OR-Tools sobre las 10 instancias (`--instances 5` usa
solo las 5 originales).

```bash
python friedman_ranking.py                   # semilla de ACO sorteada
python friedman_ranking.py --aco-seed 3      # semilla fija
python friedman_ranking.py --pick-seed 42    # sorteo repetible
python friedman_ranking.py --holm            # además, Holm contra ACO
python friedman_ranking.py --instances 5     # solo las 5 instancias originales
```

- Bloques = instancias, tratamientos = algoritmos, medida = distancia real en km (menor es mejor).
- Solo participan algoritmos con todas las instancias completas y **factibles** (batería ≥ 0 en todo tramo, todos los
  clientes atendidos, sin superar `Station copies`, sin salir del depósito a una estación, sin saltos estación →
  estación y con `Objective` igual a la distancia real). El resto se lista como excluido, con el motivo
  (`DEPOT_TO_STATION`, saltos entre estaciones, `INCONSISTENT`, `LEGACY`).
- Los nombres salen limpios (`SAVINGS_and_ACO`, sin `HeuristicType.`).
- Salidas con fecha en `problem/osrm/trazas/`: `friedman_ranking_<fecha>.csv`, `friedman_keel_<fecha>.csv`
  (formato KEEL: filas = instancias, columnas = algoritmos) y el `.txt` con la traza.
- Con pocas instancias y decenas de algoritmos el test tiene poca potencia: un p alto no prueba que sean iguales.

---

## 6. DEVRP: optimización + simulación

Flujo: **plan inicial (sección 4) → escenario SUMO → evento → re-planificación → resultado medido en SUMO.**
Se puede usar el menú interactivo `python simulation/run.py`, que llama a los mismos scripts.

### 6.1 Red vial (una vez por zona)

```bash
python simulation/extract_osm_bbox.py --instance instances_data/evrp_instances/quebec_40c_4ev_6cs.txt
python simulation/build_real_network.py --osm-file simulation/sumo_network/clipped.osm
```

Las instancias usan tres redes: Montreal (30c), Quebec (40c y 60c) y Trois-Rivières (50c y 90c).

### 6.2 Escenario del plan (por instancia **y algoritmo**)

```bash
python simulation/build_routes.py --instance <instancia.txt> --solution <solucion.txt>
python simulation/build_background_traffic.py --solution <solucion.txt> --end 3000 --period 2
python simulation/build_battery_scenario.py --instance <instancia.txt> --solution <solucion.txt>
```

`<solucion.txt>` es cualquiera de la sección 4, por ejemplo
`problem/osrm/solutions_evrp_best/solutions_ACO/quebec_50c_5ev_5cs.txt`. El escenario se guarda por algoritmo en
`simulation/sumo_scenario/<instancia>/<algoritmo>/`, porque cada algoritmo asigna rutas distintas a cada vehículo.

### 6.3 Solo simulación (sin re-planificar)

```bash
sumo-gui -c simulation/sumo_scenario/<instancia>/<algoritmo>/scenario_battery.sumocfg
```

Verifica que las rutas son ejecutables, que los vehículos paran en clientes y estaciones, y compara el consumo
lineal del modelo con el de la batería real de SUMO.

### 6.4 DEVRP con OR-Tools

```bash
python simulation/dynamic_replanning.py \
  --instance instances_data/evrp_instances/quebec_50c_5ev_5cs.txt \
  --solution problem/osrm/solutions_evrp_0/solutions_PATH_CHEAPEST_ARC_and_GUIDED_LOCAL_SEARCH/quebec_50c_5ev_5cs.txt \
  --checkpoint-time 1200
```

1. SUMO corre hasta el instante del evento y se lee el estado real de cada vehículo (posición, batería, carga).
2. Se fuerza una congestión y se mide el tiempo de viaje antes y después.
3. Se arma el sub-problema (clientes pendientes, estaciones, posiciones actuales) con la matriz **viva** de SUMO.
4. Se re-planifica y las rutas nuevas se inyectan en SUMO.
5. Se simula hasta el final y se escribe el reporte en `simulation/dynamic_replan_results/`.

Línea base **sin retroalimentación** (mismo evento, plan original intacto):

```bash
python simulation/dynamic_replanning.py --instance ... --solution ... --no-replan
```

Parámetros del evento: `--checkpoint-time` (instante), `--congested-speed` (m/s, por defecto 2,0),
`--congestion-fraction` (parte de la ruta del sondeo afectada, por defecto 0,4).

Accidentes y cierres de calle (y congestión verificada) usan `simulation/event_selector.py`:

```bash
python simulation/dynamic_replanning.py --instance ... --solution ... --event accident --verified-event \
  --event-seed 1 --event-spec evento.json          # elige y guarda el evento
python simulation/dynamic_replanning.py --instance ... --solution ... --event accident --verified-event \
  --event-spec evento.json --no-replan             # misma condición, sin retroalimentación
```

El evento se acepta solo si encarece al menos un 20 % (`--impact-threshold`) el siguiente tramo de un vehículo de la
flota, no aísla ningún punto (depósito, estaciones, clientes pendientes y vehículos siguen conectados) y ningún
vehículo queda sin batería para llegar al depósito o a una estación. Detalles en `simulation/README.md`.

### 6.5 DEVRP con ACO o ACO + heurística

El re-planificador se elige con `--replanner`:

| Valor | Qué re-planifica |
|---|---|
| `auto` (por defecto) | el mismo algoritmo que generó el plan (se lee del encabezado de la solución) |
| `aco` | ACO, aunque el plan sea de OR-Tools |
| `ortools` | OR-Tools (con `--heuristic` y `--metaheuristic` si el plan era de ACO) |

```bash
# plan de ACO, re-planifica ACO
python simulation/dynamic_replanning.py \
  --instance instances_data/evrp_instances/quebec_50c_5ev_5cs.txt \
  --solution problem/osrm/solutions_evrp_best/solutions_ACO/quebec_50c_5ev_5cs.txt \
  --checkpoint-time 1200 --aco-seed 1

# plan de SAVINGS + ACO: la heurística construye la solución inicial del sub-problema y ACO la mejora
python simulation/dynamic_replanning.py \
  --instance ... \
  --solution problem/osrm/solutions_evrp_0/solutions_SAVINGS_and_ACO/quebec_50c_5ev_5cs.txt
```

- Cada vehículo arranca desde su posición actual, con su batería y carga parciales, y termina en el depósito. Un
  vehículo sin clientes pendientes vuelve al depósito y su arco cuenta en el objetivo, igual que en OR-Tools.
- ACO + heurística: la heurística se lee del encabezado `Heuristic:` (o del nombre de la carpeta
  `solutions_<H>_and_ACO` en soluciones antiguas).
- Para comparar algoritmos en el lazo, usa **el mismo evento** (instancia, instante, severidad) con cada plan.

---

## 7. Explicabilidad (XAI)

Paquete `explainability/`. Trabaja en tres pasos desacoplados: **datos → modelo sustituto (Random Forest) →
explicador (SHAP)**. Se explica el sustituto, no el solver, y su validez depende de la fidelidad medida (R² con
validación cruzada y dejando una instancia fuera). Con pocas instancias ese R² será bajo: debe reportarse tal cual.

Requiere `pip install -r requirements_xai.txt`.

### 7.1 Optimización (qué determina el resultado de ACO)

```bash
python run_xai.py generate --algorithm ACO --rows 300 --time 3 --out explainability/data/aco.csv
python run_xai.py explain  --data explainability/data/aco.csv --target distance --out explainability/out
python run_xai.py explain  --data explainability/data/aco.csv --row 17        # explica además la corrida 17
```

- `generate` corre ACO muchas veces (muestreo Latin Hypercube de instancia, batería, consumo y parámetros). Es
  reanudable y se puede repartir con `--part 0/2` y `--part 1/2`.
- Variables: `f_*` instancia, `s_*` escenario, `p_*` parámetros de ACO. Salida: importancias, resumen y dependencias
  SHAP, y el informe con R².

### 7.2 Simulación (qué hace que un evento cueste más)

```bash
python run_xai.py sim-table        # escribe explainability/data/sim.csv, combo.csv y veh.csv
python run_xai.py explain --data explainability/data/sim.csv --target total_distance --title SIMULACION --out explainability/out_sim
```

Lee los reportes de `simulation/dynamic_replan_results/` (y los `.zip` que haya), de congestión, accidente y cierre.
Variables del evento: `e_*` (instante, tramos afectados, velocidad, `e_is_accident` / `e_is_closure` (ambos 0 =
congestión), `e_duration_s`, lentitud medida en SUMO, `e_replan`). Para generar filas nuevas hay estrategias por tipo
de evento: `SUMO_REPLAN` (congestión), `SUMO_REPLAN_ACCIDENT`, `SUMO_REPLAN_CLOSURE` y sus versiones con ACO
(`SUMO_REPLAN_ACO_*`), con `python run_xai.py generate --algorithm <nombre>`.

### 7.3 Combinación: la retroalimentación (por qué mejora o empeora re-planificar)

Tres niveles, de lo general a lo concreto:

```bash
# a) por evento: ganancia de re-planificar frente a no hacerlo
python run_xai.py explain --data explainability/data/combo.csv --target gain_distance_pct --title COMBINACION --out explainability/out_combo

# b) por vehículo: qué cambios producen ganancia (clientes cedidos y recibidos, estaciones nuevas, reordenamiento)
python run_xai.py explain --data explainability/data/veh.csv --target gain_time_vehicle_s --title RETROALIMENTACION --out explainability/out_veh

# c) relato de un evento concreto (determinista, sin aprendizaje)
python run_xai.py replan-report --run quebec_50c_5ev_5cs_WITH_feedback_20261006_215236
```

- La tabla `combo` incluye la columna `algorithm` (OR-Tools, ACO, heurística + ACO), así que SHAP puede indicar cuánto
  del resultado se debe al algoritmo y cuánto al evento y a la instancia.
- **Datos:** las tablas son tan buenas como los reportes disponibles. Los reportes anteriores al 6 de octubre no traen
  las rutas originales y no sirven para la tabla por vehículo. Para más filas:

```bash
python run_xai.py generate --algorithm SUMO_REPLAN     --rows 40 --out explainability/data/sim_gen.csv   # plan de OR-Tools
python run_xai.py generate --algorithm SUMO_REPLAN_ACO --rows 40 --out explainability/data/sim_aco.csv   # plan de ACO
```

Cada fila lanza SUMO dos veces (sin y con retroalimentación) variando instante y severidad del evento: tarda minutos.

### 7.4 Añadir otro algoritmo o técnica

- **Otra metaheurística:** una clase que herede de `AlgorithmStrategy` (`explainability/algorithms.py`) con su espacio
  de parámetros y su `run`, y `register(...)`.
- **Otra técnica de XAI:** una clase que herede de `BaseExplainer` (`explainability/explainers.py`) y una línea en
  `EXPLAINERS`.

---

## 8. Flujo completo de extremo a extremo

Orden lógico para reproducir todo el trabajo:

1. Levantar OSRM (sección 2).
2. Archivar resultados de campañas anteriores si cambió el modelo: `python archivar_resultados.py --all`.
3. OR-Tools, todas las combinaciones: `python run.py` (4.1).
4. ACO y ACO + heurística: `python run_aco.py --heuristic all` (4.2 y 4.3).
5. Ranking: `python friedman_ranking.py --holm` (5).
6. Red SUMO de cada zona y escenario de los algoritmos elegidos (6.1 y 6.2).
7. DEVRP con el mismo evento para cada algoritmo, con y sin retroalimentación (6.4 y 6.5).
8. Datos de XAI de la optimización: `python run_xai.py generate` (7.1).
9. Tablas de simulación y combinación: `python run_xai.py sim-table` (7.2 y 7.3).
10. Explicaciones: `python run_xai.py explain ...` y `replan-report` (7).

---

## 9. Estructura del repositorio

```
run.py                     ejecuta OR-Tools (EVRP y otros VRP)
run_aco.py                 ejecuta ACO solo o combinado con heurísticas
friedman_ranking.py        ranking de Friedman (+ CSV KEEL)
run_xai.py                 explicabilidad: generate, explain, sim-table, replan-report
archivar_resultados.py     ordena y archiva resultados por fecha
verificar_equivalencia_aco.py  comprueba que un cambio no altera los resultados de ACO
problem/execute/evrp.py    modelo EVRP en OR-Tools (capacidad, batería, copias de estación)
p_mh/ACO/                  ACO: evrp_aco.py (algoritmo), strategies.py (Strategy), ortools_seed.py (puente)
distance/                  Euclidiana, Manhattan, Haversine, Chebyshev y OSRM
instance/, instances_data/ lectura de instancias y datos (incluye osrm_cache/)
simulation/                SUMO: red, escenarios, re-planificación dinámica, reportes y event_selector.py (eventos verificados)
explainability/            XAI: algorithms, dataset, surrogate, explainers, simulation, feedback, report
utils/                     trazas (trace.py), archivado (archive.py) y unidades (units.py: km <-> metros enteros)
predictions/               modelos de random forest de otras variantes VRP
problem/osrm/              soluciones, archivo/ (resultados archivados) y trazas/
```

---

## 10. Reproducibilidad y verificación

- **Semillas:** ACO con la misma semilla repite el resultado. El ranking usa una sola semilla; la mejor de varias se
  reporta aparte.
- **Equivalencia de ACO:** `python verificar_equivalencia_aco.py --check` ejecuta ACO con un reloj simulado
  (determinista) y compara las rutas exactas con `verificar_equivalencia_aco.json`. Úsalo tras cualquier cambio en
  `p_mh/ACO/`; `--save` regenera la referencia solo cuando el cambio de resultados es intencional.
- **Validación de soluciones:** el ranking recalcula la batería arco por arco desde la matriz OSRM, y comprueba
  capacidad, clientes atendidos una vez y visitas por estación. Una solución que lo incumple se excluye.
- **Unidades:** el `Objective` de cada `.txt` debe coincidir con `Real distance (km)`; si no, el ranking marca el
  archivo como `INCONSISTENT`.
- **OR-Tools en instancias grandes:** con estaciones opcionales, OR-Tools puede dejar sin atender algunos clientes
  lejanos al depósito (aquellos cuyo viaje de ida y vuelta no cabe en una carga: 1 cliente en 60c, 1 a 3 en 220c y 8 en
  350c con Guided Local Search). Atenderlos exige insertar a la vez una estación y el cliente, y la búsqueda local solo
  inserta un nodo por movimiento; más tiempo, LNS y otras heurísticas de arranque no lo cambian. ACO sí los atiende con
  las mismas reglas. Esas corridas quedan marcadas con `Feasibility check: FAILED` y el ranking las excluye y las lista,
  de modo que el ranking de OR-Tools se calcula sobre las instancias donde OR-Tools atiende a todos (30, 40, 50, 70, 90,
  100 y 150 clientes); en 60c, 220c y 350c solo ACO tiene solución completa y se reporta aparte.
  `python diagnostico_ortools.py` reproduce el análisis (variantes del modelo, heurísticas, clientes descartados).
- **Límites conocidos:** pocas instancias dan poca potencia estadística; el consumo de energía es lineal y no se validó
  contra el modelo de batería de SUMO; la congestión se induce de forma artificial y con una sola semilla.
