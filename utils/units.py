"""
units.py -- unidad entera interna de OR-Tools y de ACO.

OR-Tools solo admite costos enteros. Si la distancia (km, decimal) se trunca con int(), cada arco pierde hasta 1 km
(0,5 km en promedio) y un desvio corto a una estacion cuesta 0. Por eso el modelo trabaja en METROS enteros:

    costo de arco  = round(km * DIST_SCALE)
    consumo de arco = round(km * tasa * DIST_SCALE)     (tasa en unidades de bateria por km)
    capacidad de bateria = unidades * DIST_SCALE

Todo lo que se escribe en los archivos de solucion se convierte de vuelta: distancias en km y bateria en unidades,
con decimales, sin redondear a enteros. Cambiar DIST_SCALE (p. ej. 100 = centi-km) cambia la resolucion del modelo.
"""
DIST_SCALE = 1000


def cost_int(km):
    return int(round(km * DIST_SCALE))


def energy_int(km, rate):
    return int(round(km * rate * DIST_SCALE))


def cap_int(units):
    return int(round(units * DIST_SCALE))


def to_km(value):
    return value / DIST_SCALE


def to_units(value):
    return value / DIST_SCALE
