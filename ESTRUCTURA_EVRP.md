# ESTRUCTURA_EVRP

Mapa del repositorio: qué hace cada paquete y cómo se conectan. Para ejecutar el trabajo, ver `README.md`.

## Flujo general

```
instances_data/ ──► instance/ ──► distance/ (OSRM) ──► problem/execute/evrp.py (OR-Tools) ─┐
                                                   └─► p_mh/ACO/evrp_aco.py (ACO)  ───────┤
                                                                                           ▼
                                   problem/osrm/solutions_evrp_<n>/solutions_<algoritmo>/<instancia>.txt
                                                                                           │
                       friedman_ranking.py ◄───────────────────────────────────────────────┤
                       simulation/ (SUMO + TraCI) ◄────────────────────────────────────────┘
                              │  estado real, tiempos vivos, eventos
                              └──► re-optimización (OR-Tools o ACO) ──► nueva solución ──► SUMO
                       explainability/ (SHAP) ◄── ejecuciones de ACO y reportes de la simulación
```

## Paquetes

| Ruta | Función |
|---|---|
| `problem/execute/evrp.py` | Modelo EVRP en OR-Tools: capacidad, batería (dimensión Fuel), copias de estación, arcos prohibidos (`forbid_depot_to_station`, `forbid_station_chains`), `save_solution`. Punto de entrada `execute(...)`, también usado para re-planificar (`vehicle_states`, `data_override`). |
| `p_mh/ACO/` | ACO independiente: `evrp_aco.py` (algoritmo y `save_solution`), `strategies.py` (esquemas de feromona), `ortools_seed.py` (solución inicial con una heurística de OR-Tools). |
| `utils/units.py` | Escala de unidades: la optimización usa metros enteros (`DIST_SCALE = 1000`); los archivos se escriben en km. |
| `utils/feasibility.py` | Validador independiente de factibilidad (clientes, capacidad, batería, arcos prohibidos, visitas por estación); su resultado va en cada `.txt`. |
| `utils/trace.py`, `utils/archive.py` | Trazas de ejecución y archivado de resultados por fecha. |
| `distance/` | Euclidiana, Manhattan, Haversine, Chebyshev y OSRM. |
| `instance/`, `instances_data/` | Lectura de instancias y datos. 10 instancias: 30 a 90 clientes (originales) y 70 a 350 (grandes, con estaciones AFDC). |
| `simulation/` | Red SUMO por zona, escenarios, `dynamic_replanning.py` (lazo de retroalimentación) y `event_selector.py` (accidente, cierre y congestión verificados). |
| `explainability/` | XAI: `algorithms` (estrategias, incluidas las de simulación), `dataset`, `surrogate`, `explainers`, `simulation` y `feedback` (lectura de reportes), `report`. |
| `predictions/` | Modelos de random forest de otras variantes VRP (no usado por el EVRP). |

## Scripts de la raíz

| Script | Función |
|---|---|
| `run.py` | Ejecuta OR-Tools (una instancia o todas las combinaciones heurística + metaheurística). |
| `run_aco.py` | Ejecuta ACO solo o con una heurística; 6 semillas; tiempo por instancia. |
| `friedman_ranking.py` | Ranking de Friedman sobre 10 instancias (o 5 con `--instances 5`); valida factibilidad y consistencia. |
| `run_xai.py` | `generate`, `explain`, `sim-table`, `replan-report`. |
| `archivar_resultados.py` | Archiva resultados anteriores. |
| `verificar_equivalencia_aco.py` | Comprueba que un cambio no altera las rutas de ACO (`--check`). |
| `diagnostico_ortools.py` | Diagnóstico de OR-Tools: estado del solver, clientes descartados (y cuántos son lejanos), tiempo a la primera solución, variantes del modelo y heurísticas. No escribe soluciones. |
| `compare_aco_vs_ortools.py` | Comparación puntual de ACO y OR-Tools. |

## Formato de solución (`.txt`)

OR-Tools y ACO escriben el mismo formato. Campos principales: `Objective` (km, 3 decimales), `Station copies`, rutas con
carga y batería por parada, `Distance of the route` y `Real distance of the route`, `Customers served`,
`Dropped customers`, `Feasibility check`, `Total recharges`, `Unnecessary charging stops` y `Minimum battery (SoC) reached`.
Solo si hay descartes aparece `Dropped clients: [...]`.

## Reglas del modelo

- Objetivo único: distancia total. Ningún otro costo.
- Del depósito no se va directo a una estación y de una estación no se va directo a otra.
- Una estación puede visitarse varias veces (OR-Tools: `station_copies` copias; ACO: sin tope).
- En OR-Tools cada visita a estación es un par llegada → salida (`expand_station_copies`): el nodo de salida tiene la batería fijada en la capacidad (recarga completa, nunca por encima), sin restricciones propias del CP. Ambos nodos se fusionan en una sola parada al guardar.
- Batería lineal con el consumo; nunca negativa, incluido el regreso al depósito.
- Todos los clientes deben atenderse; un cliente descartado indica un modelo o una instancia infactible.

## Dependencias

Ver `requirements.txt` (optimización, OSRM, SUMO/TraCI) y `requirements_xai.txt` (scikit-learn, SHAP).
