import numpy as np


def generar_puntos(punto_inicio, punto_fin, espacio):
    x1, y1 = punto_inicio
    x2, y2 = punto_fin

    distancia = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    num_puntos = int(distancia / espacio) + 1  # +1 para incluir el punto final

    x_puntos = np.linspace(x1, x2, num_puntos)
    y_puntos = np.linspace(y1, y2, num_puntos)

    return list(x_puntos), list(y_puntos)


# Ejemplo de uso
punto_inicio = (0, 0)
punto_fin = (10, 10)
espacio = 1.0

puntos_x, puntos_y = generar_puntos(punto_inicio, punto_fin, espacio)
print(puntos_x)
print(puntos_y)