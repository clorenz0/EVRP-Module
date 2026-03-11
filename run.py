from problem.problem_type import ProblemType, execute
from problem.strategy_type import HeuristicType, MetaheuristicType
from distance.distance_type import DistanceType
from instance.instance_type import InstanceType

############
#  RUN
############
# Arguments:
# 1. problem_type => variante VRP que se desea ejecutar de las 5 disponibles
# 2. instance => si desea un conjunto de instancias del problema elegir de InstanceType.el grupo que desee, si es una única instancia poner la ruta relativa. Ejemplo: ./instances/hfvrp_instances/CVRP_1.txt
# 3. distance_type => elegir el tipo de distancia que se desea utilizar de las 4 disponibles (si no se elige por defecto se usa Manhattan)
# 4. executions => cantidad de ejecuciones (si no se elige, se pone 1 por defecto)
# 5. time_limit => tiempo máximo en segundos en que se va a demorar el algoritmo en devolver la solución
# 6. vehicle_maximum_travel_distance => distancia máxima que puede recorrer un vehículo (si no se elige se pone 500 por defecto, excepto a VRPTW que se pone 1000) (restricción válida para todos los problemas menos TSP y CVRP)
# 7. vehicle_max_time => máximo tiempo en que un vehículo debe completar su ruta (solo para VRPTW, se pone 1500 por defecto)
# 8. vehicle_speed => velocidad del vehículo (solo para VRPTW, se pone 83.33 km/h por defecto)
# 9. heuristic => heurística específica que se desea utilizar
# 10. metaheuristic => metaheurística específica que se desea utilizar
# 11. initial_routes => rutas iniciales para reoptimizar en tiempo real
# (si no se elige una heurística ni una metaheurística entonces se ejecutan todas por defecto)


execute(
    problem_type=ProblemType.EVRP,
    instance="./instances_data/evrp_instances/quebec_50c_5ev_5cs.txt",
    distance_type=DistanceType.MANHATTAN,
    time_limit=10,
    executions=1,
    vehicle_maximum_travel_distance=None,
    vehicle_max_time=None,
    vehicle_speed=None,
    heuristic=None,
    metaheuristic=MetaheuristicType.SIMULATED_ANNEALING,
    initial_routes=[[18, 54, 127, 186, 8, 3, 88, 98, 177, 157], [32, 86, 115, 94, 65, 51, 171, 174, 136, 189, 124], [75, 163, 182, 23, 194, 145, 195, 52, 92, 21], [106, 128, 46, 193, 125, 191, 84, 180, 82, 4, 72, 60], [114, 22, 150, 38, 159, 151, 16, 140, 187, 142, 111, 63, 56], [165, 188, 83, 118, 57, 143, 176, 36, 33, 121, 108], [135, 49, 117, 181, 7, 132, 185, 104, 161, 58, 184, 199], [123, 42, 87, 29, 79, 168, 112, 156, 50, 134, 170, 153], [200, 69, 141, 197, 103, 148, 70], [81, 138, 137, 183, 37, 55, 93], [130, 96, 59, 15, 105, 89, 169, 40, 152, 198, 14, 26], [91, 12, 116, 164, 66, 47, 160, 147, 129], [35, 166, 119, 126, 71, 9, 1, 99, 53, 144, 101], [162, 110, 77, 172, 25, 31, 80, 85, 41, 20], [149, 48, 133, 74, 28, 97, 196, 192, 19, 120, 30, 68], [76, 102, 146, 44, 131, 62], [109, 45, 179, 64, 100, 61, 24, 154, 173, 27, 178], [113, 13, 43, 34, 167, 5, 10, 95, 158, 190, 90, 2, 175], [73, 6, 11, 122, 139, 39, 17, 67, 78, 155, 107]]
    #initial_routes=[[258,166,89,147,46,153,215,143,330,337,160,5],[352,282,26,350,40,117,247,320,283,289,168,162,355,214,326],[111,374,264,396,307,32,65,67,39,62,333],[131,261,383,50,161,173,233,121,213,63],[113,288,14,68,394,74,178,6,77,357,327],[223,236,35,110,369,61,11,218,182,290,138],[193,324,156,133,210,382,338,256,205,19,55,120],[116,127,1,389,181,51,49,245,242,259],[172,109,75,183,123,228,226,292,17,104,158,227,395],[302,30,95,177,328,380,85,265,73],[335,385,303,144,24,93,229,204,295,103,220,122],[279,291,16,52,136,314,317,22,399,132],[80,315,221,294,151,270,322,284,58,351,356],[186,154,381,202,45,346,21,118,309,342],[225,125,231,188,323,278,78,387,400,262],[142,276,316,194,363,251,38,371,23,134,107],[47,31,248,108,362,171,243,239,192,128,287,27],[84,197,81,246,312,379,386,179,180,54,94,268],[378,164,69,222,159,304,71,211,59,102,253],[266,165,308,146,79],[191,86,199,82,359,254,293,60,53,296,313,36],[9,12,198,219,237,299,360,305,249,157],[255,29,28,212,169,135,140,112,90,393,196,321],[33,189,174,250,98,345,48,57,244,271,390,234],[358,216,334,195,285,18,238,297,329],[100,397,72,207,341,187,200,203,365,273,119,232,42],[184,353,96,25,70,87,298,201,99],[208,64,145,318,106,269,343],[277,141,252,370,274,170,4,190,275,56,240],[41,332,224,92,310,339,8,34,230,43,260,319,163],[3,372,130,91,344,349,301,280,392,139],[354,206,175,83,150,366],[241,311,76,13,300,15,257,101,7,149,375,97],[373,340,364,267,167,148,367,391,115],[126,66,398,331,325,368,37,20],[384,137,176,263,129,281,235,388,348,44,114,88],[217,185,209,10,152,286],[105,155,361,272,306,377,124,2,347,376,336]]
)
