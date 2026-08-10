from p_mh.ACO import evrp_aco
from distance.distance_type import DistanceType
from instance.instance_type import InstanceType

############
#  RUN ACO
############
# Solver alternativo para EVRP (Ant Colony Optimization), independiente de
# OR-Tools. Lee la misma carpeta de instancias que run.py (InstanceType.EVRP)
# y escribe la solucion en el mismo formato que evrp.py, en una carpeta
# aparte (solutions_ACO) para poder comparar directamente contra una corrida
# de run.py sobre la misma instancia.
#
# Arguments:
# 1. i => indice de ejecucion (para nombrar/separar corridas repetidas)
# 2. instance_type => InstanceType.EVRP (carpeta de instancias)
# 3. time_limit => tiempo maximo en segundos que corre el algoritmo
# 4. vehicle_maximum_travel_distance => capacidad de bateria (fuel_capacity)
# 5. vehicle_speed => consumo de bateria por unidad de distancia
# 6. distance_type => misma opcion que en run.py (usa OSRM para comparar
#    contra una corrida real de evrp.py)
# 7. num_ants => cantidad de hormigas por iteracion (None = automatico)
# 8. alpha, beta => importancia relativa de feromona vs. heuristica (1/dist)
# 9. gamma => importancia del termino de energia/bateria (carga proactiva,
#    ver Thymianis et al. citado en el docstring de evrp_aco.py)
# 10. rho => tasa de evaporacion de feromona por iteracion
# 11. rank_w => Rank-AS: solo depositan las (w-1) mejores hormigas de cada
#    iteracion + refuerzo a la mejor solucion global (ver Nie et al.)
# 12. proactive_charge_threshold => fraccion de fuel_capacity por debajo de
#    la cual una estacion siquiera entra a la ruleta como candidata (0 =
#    desactivado, se comporta como el ACO original: solo carga de rescate
#    cuando ya no queda otra opcion). Valor bajo (0.15) por defecto: en las
#    pruebas, un umbral alto (0.4+) dejaba que algunas hormigas cargaran sin
#    necesitarlo de verdad, empeorando el resultado final.
# 13. seed => opcional, para resultados reproducibles

evrp_aco.execute(
    i=0,
    instance_type=InstanceType.EVRP,
    time_limit=10,
    vehicle_maximum_travel_distance=100,
    vehicle_speed=1.0,
    distance_type=DistanceType.OSRM,
    num_ants=None,
    alpha=1.0,
    beta=1.5,
    gamma=2.5,
    rho=0.5,
    rank_w=6,
    proactive_charge_threshold=0.15,
    seed=None,
)
