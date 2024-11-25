import sslclient
import time
import numpy as np
from filterpy.kalman import KalmanFilter
import math


c = sslclient.client(ip='224.5.23.2', port=10006)  
c.connect()


class KalmanFilter2D:
    def __init__(self):
        # Inicializa o filtro de Kalman
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.x = np.array([0., 0., 0., 0.])  # Estado inicial [x, y, vx, vy]
        self.kf.F = np.array([[1., 0., 1., 0.],  # Matriz de transição
                              [0., 1., 0., 1.],
                              [0., 0., 1., 0.],
                              [0., 0., 0., 1.]])
        self.kf.H = np.array([[1., 0., 0., 0.],  # Matriz de observação
                              [0., 1., 0., 0.]])
        self.kf.P *= 1000.  # Covariância inicial
        self.kf.R = np.array([[5., 0.], [0., 5.]])  # Covariância do ruído de medição
        self.kf.Q = np.array([[0.1, 0., 0., 0.],    # Covariância do processo
                              [0., 0.1, 0., 0.],
                              [0., 0., 0.1, 0.],
                              [0., 0., 0., 0.1]])

    def update(self, position):
        self.kf.predict()
        self.kf.update(np.array(position))
        return self.kf.x[:2]  


kalman_filter = KalmanFilter2D()


last_position = None
last_time = None

while True:
    
    data = c.receive()
    current_time = time.time()
    
    if data.HasField('detection'):
        detection = data.detection

        
        ball_position = (detection.balls[0].x, detection.balls[0].y) if detection.balls else None

        if ball_position:
            
            smoothed_position = kalman_filter.update(ball_position)

            if last_position:
                
                distance = math.sqrt((smoothed_position[0] - last_position[0])**2 +
                                     (smoothed_position[1] - last_position[1])**2)
                
                
                time_elapsed = current_time - last_time
                
                
                if time_elapsed > 0:
                    velocity = distance / time_elapsed
                    print(f"Ball Velocity: {velocity:.2f} units/sec")
                else:
                    print("Time elapsed is zero, unable to calculate velocity.")
            
            
            last_position = smoothed_position
            last_time = current_time
