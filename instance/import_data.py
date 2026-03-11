import random
import json
import os
from distance.distance_type import DistanceType, calculate_distance


#######################
# ONE INSTANCE
#######################


def process_string_instance(instance_string, distance_type: DistanceType = None, vehicle_max_time=None,
                            vehicle_speed=None, vehicle_maximum_travel_distance=None, integer=False):
    try:
        if 'bhcvrp_instances' in instance_string:
            data = read_file_bh(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                vehicle_maximum_travel_distance, integer)
        elif 'hfvrp_instances' in instance_string:
            data = read_file_hf(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                vehicle_maximum_travel_distance, integer)
        elif 'bss_instances' in instance_string:
            data = read_file_bss(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                 vehicle_maximum_travel_distance, integer)
        elif 'mdcvrp_instances' in instance_string:
            data = read_file_md(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                vehicle_maximum_travel_distance, integer)
        elif 'tsp_instances' in instance_string:
            data = read_file_tsp(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                 vehicle_maximum_travel_distance, integer)
        else:
            data = read_file_tw(instance_string, distance_type, vehicle_max_time, vehicle_speed,
                                vehicle_maximum_travel_distance, integer)
        return {os.path.basename(instance_string): data}
    except Exception as error:
        print(f"Instance not found {instance_string}")
        raise


#######################
# DATA BSS
#######################


def read_file_bss(file, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                  vehicle_maximum_travel_distance=None, integer=False):
    with open(file, 'r') as file:
        json_data = json.load(file)
        locations = []
        for depot in json_data["depots"]:
            depot_coords = (depot["coordinate_x"], depot["coordinate_y"])
            locations.append(depot_coords)
        for bus_stop in json_data["bus_stops"]:
            bus_stop_coords = (bus_stop["coordinate_x"], bus_stop["coordinate_y"])
            locations.append(bus_stop_coords)
    result = get_distance_matrix(locations, distance_type, integer)
    return {
        "locations": locations,
        "distance_matrix": result['distance_matrix'],
        "num_vehicles": 6,
        "vehicle_capacities": [7, 7, 3, 5, 4, 6],
        "demands": [0, 2, 2, 2, 3, 3, 3, 3, 3],
        "depot": 0
    }


#######################
# DATA TSP
#######################

def read_file_tsp(file, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                  vehicle_maximum_travel_distance=None, integer=False):
    with open(file, "r") as f:
        lines = f.readlines()
        node_coord_section_index = lines.index("NODE_COORD_SECTION\n")
        locations = []
        for line in lines[node_coord_section_index + 1:]:
            if line.strip() == "EOF":
                break
            parts = line.split()
            x_coord = float(parts[1])
            y_coord = float(parts[2])
            locations.append((x_coord, y_coord))
    return {
        "locations": locations,
        "num_locations": len(locations),
        "num_vehicles": 1,
        "depot": 0
    }


#######################
# DATA CVRPMD
#######################

def read_file_md(file, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                 vehicle_maximum_travel_distance=None, integer=False):
    locations = []
    demands = []
    num_locations = 0
    starts = []
    ends = []
    with open(file, 'r') as f:
        lines = f.readlines()
        num_depots = int(lines[0].split()[0])
        num_vehicles = int(lines[0].split()[2])
        capacities = []
        for i in range(1, num_vehicles + 1):
            capacities.append(int(lines[i].strip()))
        temp_lines = []
        for line in lines[num_vehicles + 1:]:
            if len(line.strip()) > 0:
                temp_lines.append(line)
        for line in temp_lines:
            data = line.split()
            if len(data) >= 3:
                x_coord = float(data[1])
                y_coord = float(data[2])
                locations.append((x_coord, y_coord))
                if len(data) == 4:
                    demand = int(data[3])
                else:
                    demand = 0
                demands.append(demand)
                num_locations += 1
        depot_indices = temp_lines[-num_depots:]
        for line in depot_indices:
            data = line.split()
            if len(data) >= 3:
                depot_index = int(data[0]) - 1
                starts.append(depot_index)
                ends.append(depot_index)
    result = create_pd(num_locations, locations, distance_type, integer)
    return {
        "distance_matrix": result['distance_matrix'],
        "num_vehicles": num_vehicles,
        "vehicle_capacities": capacities,
        "demands": demands,
        "starts": starts,
        "ends": ends,
        "pickups_deliveries": result['pickups_deliveries'],
        "depot": 0
    }


#######################
# DATA HFVRP
#######################


def read_file_hf(file, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                 vehicle_maximum_travel_distance=None, integer=False):
    locations = []
    demands = []
    num_locations = 0
    with open(file, 'r') as f:
        lines = f.readlines()
        num_vehicles = int(lines[0].split()[0])
        capacities = list(map(int, lines[1].strip().split()))
        temp_lines = []
        for line in lines[1:]:
            if len(line.strip()) > 0:
                temp_lines.append(line)
        for line in temp_lines[1:]:
            data = line.split()
            if len(data) >= 3:
                x_coord = float(data[1])
                y_coord = float(data[2])
                locations.append((x_coord, y_coord))
                if len(data) == 4 and data[3]:
                    demand = int(float(data[3]))
                else:
                    demand = 0
                demands.append(demand)
                num_locations += 1
    result = create_pd(num_locations, locations, distance_type, integer)
    return {
        "distance_matrix": result['distance_matrix'],
        "num_vehicles": num_vehicles,
        "vehicle_capacities": capacities,
        "demands": demands,
        "pickups_deliveries": result['pickups_deliveries'],
        "depot": 0
    }


#######################
# DATA BHCVRP
#######################

def read_file_bh(file, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                 vehicle_maximum_travel_distance=None, integer=False):
    locations = []
    demands = []
    num_locations = 0
    with open(file, 'r') as f:
        lines = f.readlines()
        capacity = int(float(lines[0].split()[0]))
        num_vehicles = int(lines[0].split()[3])
        capacities = [capacity] * num_vehicles
        for line in lines[1:]:
            if len(line.strip()) == 0 or line.strip().startswith("EOF"):
                break
            data = line.split()
            if len(data) == 5:
                x_coord = float(data[1])
                y_coord = float(data[2])
                locations.append((x_coord, y_coord))
                demand = float(data[3]) if data[3] else 0
                demands.append(int(demand))
                num_locations += 1
    result = create_pd(num_locations, locations, distance_type, integer)
    return {"distance_matrix": result['distance_matrix'], "num_vehicles": num_vehicles,
            "vehicle_capacities": capacities,
            "demands": demands, "depot": 0, "pickups_deliveries": result['pickups_deliveries']}


#######################
# DATA VRPTW
#######################

def read_file_tw(file_path, distance_type: DistanceType = None, vehicle_max_time=None, vehicle_speed=None,
                 vehicle_maximum_travel_distance=None, integer=False):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        num_vehicles, vehicle_capacities = map(int, lines[0].split())
        service_times = []
        locations = []
        demands = []
        time_windows = []
        for line in lines[2:]:
            parts = line.split()
            if len(parts) == 7:
                locations.append((int(parts[1]), int(parts[2])))
                demands.append(int(parts[3]))
                time_windows.append((int(parts[4]), int(parts[5])))
                service_times.append(int(parts[6]))
        result = get_distance_matrix(locations, distance_type, integer)
    return {
        "num_vehicles": num_vehicles,
        "vehicle_capacity": vehicle_capacities,
        "vehicle_capacities": [vehicle_capacities] * num_vehicles,
        "demands": demands,
        "distance_matrix": result['distance_matrix'],
        "locations": locations,
        "num_locations": result['num_locations'],
        "service_time": service_times,
        "time_windows": time_windows,
        "vehicle_max_distance": vehicle_maximum_travel_distance if vehicle_maximum_travel_distance else 1000,
        "vehicle_max_time": vehicle_max_time if vehicle_max_time else 1_500,
        "vehicle_speed": vehicle_speed if vehicle_speed else 5 * 60 / 3.6,
        "depot": 0
    }


def create_pd(num_locations, locations, distance_type, integer=False):
    all_ids = list(range(num_locations))
    all_ids.remove(0)
    random.shuffle(all_ids)
    pickups = all_ids[:num_locations // 2]
    deliveries = all_ids[num_locations // 2:]
    pickups_deliveries = [list(pair) for pair in zip(pickups, deliveries)]
    distance_matrix = []
    for i, coord1 in enumerate(locations):
        row = []
        for j, coord2 in enumerate(locations):
            if i == j:
                row.append(0)
            else:
                distance = calculate_distance(coord1, coord2, distance_type, integer)
                row.append(distance)
        distance_matrix.append(row)
    return {'pickups_deliveries': pickups_deliveries, 'distance_matrix': distance_matrix}


def get_distance_matrix(locations, distance_type, integer=False):
    num_locations = len(locations)
    distance_matrix = [[0] * num_locations for _ in range(num_locations)]
    for i in range(num_locations):
        for j in range(num_locations):
            if i != j:
                distance_matrix[i][j] = calculate_distance(locations[i], locations[j], distance_type, integer)
    return {'num_locations': num_locations, 'distance_matrix': distance_matrix}


#######################
# DATA EVRP
#######################

def read_file_evrp(file_path, distance_type=None, vehicle_max_time=None,
                       vehicle_speed=None, vehicle_maximum_travel_distance=None, integer=False):
    """
    Opción usando csv.DictReader (módulo estándar).
    ✓ Sin dependencias externas
    ✓ Rápido y simple
    ✓ Muy legible
    ✗ Un poco más código que pandas
    """
    import csv
    from io import StringIO

    fuel_capacity = int(vehicle_maximum_travel_distance) if vehicle_maximum_travel_distance else 100
    fuel_consumption_rate = float(vehicle_speed) if vehicle_speed else 1.0

    locations = []
    demands = []
    charging_stations = []
    charging_station_names = {}
    num_vehicles = None
    vehicle_capacity = None

    # ── Leer todo el archivo ────────────────────────────────────────────────
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    # ── Procesar línea por línea ────────────────────────────────────────────
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1

        # Ignorar vacías y comentarios
        if not line or line.startswith('#'):
            continue

        # Split para parsear CSV manualmente
        fields = [f.strip() for f in line.split(',')]

        # ── PROBLEMA: "5,200" ───────────────────────────────────────────────
        if num_vehicles is None and len(fields) == 2:
            try:
                num_vehicles = int(fields[0])
                vehicle_capacity = int(fields[1])
                continue
            except ValueError:
                pass

        # ── DEPÓSITO: "DEPOSITO,46.5,-72.0" ────────────────────────────────
        if fields[0] == 'DEPOSITO' and len(fields) >= 3:
            lat, lon = float(fields[1]), float(fields[2])
            locations.append((lat, lon))
            demands.append(0)
            continue

        # ── CLIENTES: "C_001,46.124,-73.456,18" ────────────────────────────
        if fields[0].startswith('C_') and len(fields) >= 4:
            try:
                lat = float(fields[1])
                lon = float(fields[2])
                demand = int(fields[3])
                locations.append((lat, lon))
                demands.append(demand)
                continue
            except ValueError:
                pass

        # ── ESTACIONES: "82842,Nissan Sorel-Tracy,46.024,-73.154,..." ──────
        if len(fields) >= 4:
            try:
                # Intenta parsear como estación de carga
                station_id = int(fields[0])
                name = fields[1]
                lat = float(fields[2])
                lon = float(fields[3])

                node_index = len(locations)
                locations.append((lat, lon))
                demands.append(0)
                charging_stations.append(node_index)
                charging_station_names[node_index] = name
                continue
            except (ValueError, IndexError):
                pass

    # ── Calcular matriz de distancias ───────────────────────────────────────
    num_locations = len(locations)
    distance_matrix = [[0] * num_locations for _ in range(num_locations)]
    for i in range(num_locations):
        for j in range(num_locations):
            if i != j:
                distance_matrix[i][j] = calculate_distance(
                    locations[i], locations[j], distance_type, integer
                )

    return {
        "num_vehicles": num_vehicles,
        "vehicle_capacity": vehicle_capacity,
        "vehicle_capacities": [vehicle_capacity] * num_vehicles,
        "locations": locations,
        "num_locations": num_locations,
        "demands": demands,
        "depot": 0,
        "distance_matrix": distance_matrix,
        "fuel_capacity": fuel_capacity,
        "fuel_consumption_rate": fuel_consumption_rate,
        "charging_stations": charging_stations,
        "charging_station_names": charging_station_names,
    }
