from collections import deque
import math
import numpy as np
from filterpy.kalman import KalmanFilter


def moving_average_position(history, window_size=7):
    if len(history) == 0:
        return None
    avg_x = sum(pos[0] for pos in history) / len(history)
    avg_y = sum(pos[1] for pos in history) / len(history)
    return (avg_x, avg_y)


# Classe para o Filtro de Kalman
class KalmanFilter2D:
    def __init__(self):
        # Inicializa o filtro de Kalman
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0., 0., 0., 0.])  # Posição inicial [x, y, vx, vy]
        self.kf.F = np.array([[1., 0., 1., 0.],  # Matriz de transição
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])
        self.kf.H = np.array([[1., 0., 0., 0.],  # Matriz de observação
                              [0., 1., 0., 0.]])
        self.kf.P *= 1000.  # Covariância inicial
        self.kf.R = np.array([[10., 0.], [0., 10.]])  # Covariância do ruído de medição
        self.kf.Q = np.array([[0.5, 0., 0., 0.],    # Covariância do processo
                              [0., 0.5, 0., 0.],
                              [0., 0., 0.5, 0.],
                              [0., 0., 0., 0.5]])

    def update(self, position):
        # Atualiza o estado do filtro de Kalman com a nova posição
        self.kf.predict()
        self.kf.update(np.array(position))  # Converte a posição para array NumPy
        return self.kf.x[:2]  # Retorna a posição suavizada (x, y)
