"""
Caracteristicas (variables explicativas) de una corrida.

Prefijos de columna:
  f_  caracteristicas de la INSTANCIA (calculadas de los datos: tamaño, demanda, geometria)
  s_  parametros del ESCENARIO (bateria, consumo)
  p_  parametros del ALGORITMO
"""


def instance_features(data):
    """data: diccionario que devuelve instance.import_data.read_file_evrp (con la matriz de distancias)."""
    dm = data["distance_matrix"]
    stations = list(data["charging_stations"])
    n = data["num_locations"]
    depot = data["depot"]
    clients = [c for c in range(n) if c != depot and c not in set(stations)]

    depot_d = [dm[depot][c] for c in clients]
    nn_d = [min(dm[c][o] for o in clients if o != c) for c in clients]
    st_d = [min(dm[c][s] for s in stations) for c in clients] if stations else [0.0]
    total_demand = sum(data["demands"][c] for c in clients)
    fleet_capacity = data["num_vehicles"] * data["vehicle_capacity"]

    return {
        "f_n_clients": len(clients),
        "f_n_stations": len(stations),
        "f_n_vehicles": data["num_vehicles"],
        "f_vehicle_capacity": data["vehicle_capacity"],
        "f_demand_fill": round(total_demand / fleet_capacity, 4),      # demanda total / capacidad de la flota
        "f_depot_dist_mean": round(sum(depot_d) / len(depot_d), 4),
        "f_depot_dist_max": round(max(depot_d), 4),
        "f_nn_dist_mean": round(sum(nn_d) / len(nn_d), 4),             # cercania entre clientes
        "f_station_dist_mean": round(sum(st_d) / len(st_d), 4),        # distancia cliente - estacion mas cercana
        "f_station_density": round(len(stations) / len(clients), 4),
    }


def scenario_features(battery, consumption, data):
    """Batería, consumo y el alcance resultante relativo a la geometria de la instancia."""
    rng_km = battery / consumption
    dm = data["distance_matrix"]
    depot = data["depot"]
    far = max(dm[depot][c] for c in range(data["num_locations"]) if c != depot)
    return {
        "s_battery": battery,
        "s_consumption": consumption,
        "s_range_ratio": round(rng_km / far, 4),   # alcance / cliente mas lejano: <1 = ningun cliente alcanzable sin recargar
    }
