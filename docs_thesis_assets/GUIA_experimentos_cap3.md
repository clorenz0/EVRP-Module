# Guía: experimentos del capítulo 3 (v8, 2026-10-27)

Todo se lanza desde la raíz del repo. `docs_thesis_assets/correr_experimentos_cap3.py` no modifica código del repo; añade `--dry-run` para ver los comandos sin ejecutarlos. Requiere SUMO (`SUMO_HOME`, probado con 1.26 y 1.27), las redes de `simulation/` y las soluciones del ranking definitivo.

## 0. Antes de nada: el EVRP estático no puede cambiar

```bash
python tests/regresion_estatico.py
```

Resuelve 30c y 40c con combinaciones fijas (búsqueda determinista, corte por número de soluciones) y compara objetivo, factibilidad y rutas completas con la línea base (`tests/regresion_estatico_base.json`, generada con `tests/respaldo/evrp_pre_replan_2026-10-24.py`). Debe decir `OK ... EXACTAMENTE los mismos resultados`. Correrla después de cualquier cambio en el código.

## Qué cambió y por qué hay que repetir las simulaciones (2026-10-24 a 2026-10-27)

1. **Todos los clientes atendidos (EVRP estático).** La reparación de `problem/execute/evrp.py` inserta los clientes que OR-Tools deja sin atender (inserción con arrepentimiento, expulsión y ruina y reconstrucción). Su tiempo cuenta en `Execution Time`.
2. **Re-planificación con OR-Tools.** La posición actual del vehículo ya no cuenta como estación de carga (antes podía "recargar" allí y salían rutas con batería negativa).
3. **Batería en SUMO** (`simulation/build_battery_scenario.py`, `dynamic_replanning.py`):
   - SUMO reciente configura la batería con `device.battery.capacity` / `device.battery.chargeLevel`. Con las claves antiguas usaba sus valores por defecto (35 000 Wh, carga inicial al 50 %): la calibración no tenía efecto y todos los vehículos llegaban al evento con ~47 %.
   - Autonomía = `Battery capacity` / `Battery consumption rate` **leídos de la solución** (100 / 3 = 33,3 km), no 100 km. Si cambia el consumo del vehículo en `run.py`, la simulación lo toma sola.
   - Recuperación al frenar desactivada de verdad (`recuperationEfficiency`) y física del vehículo como `<param>` (duarouter descartaba los atributos).
4. **Re-planificación (2026-10-28)** (`simulation/dynamic_replanning.py`, `problem/execute/evrp.py`):
   - Tiempo de re-planificación = máx(15 s, tiempo de la optimización estática de la instancia): 15 s hasta 90 clientes (sin cambios), 60 s en 100 y 150, 120 s en 220 y 180 s en 350. Con 15 s fijos, en 220 clientes OR-Tools dejaba >25 clientes a la reparación y ACO hacía 5-7 iteraciones. `--replan-time N` lo fija a mano.
   - Al final de cada re-planificación con OR-Tools se quitan las paradas de carga innecesarias (la ruta sigue factible sin ellas y no se alarga). El reporte lo indica con `Unnecessary charging stops removed after re-planning`. ACO ya las podaba. El EVRP estático no cambia (regresión: IDÉNTICO).
5. **Explicabilidad:** cada corrida CON se compara con su SIN exacta del manifiesto (mismo plan, evento y replanificador); antes se promediaban líneas base de planes distintos. Cada punto de vista escribe en su carpeta.

Resultados apartados (no borrados), cada carpeta con su `LEEME.txt`, en `simulation/dynamic_replan_results/`:
- `invalidos_pre_fix/`: replanificaciones de OR-Tools anteriores al punto 2 (FAILED).
- `historico_pre_modelo_final/`: simulaciones del 1 al 6 de octubre (versiones antiguas del modelo).
- `historico_bateria_por_defecto/`: todas las simulaciones hasta el 2026-10-27 (batería sin calibrar, punto 3).

Los eventos (`dynamic_replan_results/specs_cap3/*.json`) se conservan: la nueva tanda repite exactamente los mismos eventos.

## 1. Tabla 12 (congestión, 3 algoritmos)

```bash
python docs_thesis_assets/correr_experimentos_cap3.py sim-fav --force --instances 40 50 60 90 100 150 220 70
```

- `--force` es obligatorio esta vez: reconstruye los escenarios de batería con la calibración corregida.
- Se puede ir por partes (`--instances 40`, luego `--instances 50 60`, ...). La 220 es la más pesada; la 70 usa la red de Montreal (1,2 GB).
- Planes: ACO (`solutions_evrp_1001/solutions_ACO`), PATH_CHEAPEST_ARC + ACO y SEQUENTIAL_CHEAPEST_INSERTION + GUIDED_LOCAL_SEARCH (`solutions_evrp_0`). Cada uno se replanifica con su propio método (`aco` / `ortools`).
- Pendiente: 350 (cuando el resto esté validado) y 30 (su escenario sobre Montreal nunca se construyó). ACO en 70 y 220 fallaba en `duarouter`.
- `--replanners ortools` (o `aco`) limita las corridas a un replanificador.

## 2. Tabla 13 (accidente, cierre y congestión, 50 clientes)

```bash
python docs_thesis_assets/correr_experimentos_cap3.py sim-desfav --unfav-instance 50
```

Sin `--force` si el paso 1 ya reconstruyó los escenarios de 50 clientes.

## 3. Comprobar la tanda

```bash
python -c "import glob,os;fs=sorted(glob.glob('simulation/dynamic_replan_results/*WITH*'),key=os.path.getmtime);[print(os.path.basename(f),'|',next(l.strip() for l in open(f,encoding='utf-8') if l.startswith('Feasibility'))) for f in fs]"
```

Todas deben decir `PASSED` y ningún reporte debe contener `DID NOT finish`.

## 4. Tablas, Figura 17 y Wilcoxon de la Tabla 12

```bash
python docs_thesis_assets/correr_experimentos_cap3.py tablas
```

Salida en `docs_thesis_assets/resultados_cap3/` (`resultados_sim.csv`, `pares_sin_con.csv`, `fig_barras_sin_con.png`).

## 5. Explicabilidad (epígrafe 3.5)

No hace falta regenerar `explainability/data/aco.csv` (optimización, 10 instancias). Para simulación y combinación:

```bash
python run_xai.py sim-table
python run_xai.py explain --data explainability/data/aco.csv --target distance --title OPTIMIZACION --out explainability/out/optimizacion
python run_xai.py explain --data explainability/data/sim.csv --title SIMULACION --out explainability/out/simulacion
python run_xai.py explain --data explainability/data/combo.csv --title COMBINACION --out explainability/out/combinacion
python run_xai.py explain --data explainability/data/combo.csv --target gain_distance_pct --title COMBINACION_GANANCIA --out explainability/out/combinacion_ganancia
python docs_thesis_assets/pares_retroalimentacion.py
python run_xai.py replan-report --run <nombre de un reporte WITH>
```

- Mirar siempre la fidelidad del modelo sustituto, sobre todo "R2 dejando una instancia fuera": si es baja o negativa, el SHAP de ese punto de vista es solo exploratorio (pocas instancias simuladas).
- El efecto de la retroalimentación se reporta con la comparación pareada (`pares_retroalimentacion.py` → `explainability/out/pares_retroalimentacion_wilcoxon.txt`).
- `correr_experimentos_cap3.py xai` hace lo mismo pero empieza regenerando `aco.csv` (600 corridas, ~30 min).

## 6. Capturas de SUMO (Figuras 15 y 16)

Con el escenario construido, `python simulation/run.py` (opción "Abrir un escenario en sumo-gui"), o `--gui` en una corrida de `dynamic_replanning.py`. Figura 15: plan de PATH_CHEAPEST_ARC + ACO (50 clientes) en ejecución. Figura 16: instante posterior al evento, con la congestión y la ruta reinyectada.
