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

    def predict(self):
        self.kf.predict()

    def update(self, position):
        self.kf.update(np.array(position))
        return self.kf.x[:2] 


kalman_filter_ball = KalmanFilter2D()
kalman_filters_robots = [KalmanFilter2D() for _ in range(12)]  


last_ball_position = None
last_ball_time = None
last_robot_positions = [None] * 12
last_robot_times = [None] * 12

while True:

    data = c.receive()
    current_time = time.time()
    
    if data.HasField('detection'):
        detection = data.detection


        ball_position = (detection.balls[0].x, detection.balls[0].y) if detection.balls else None

        if ball_position:

            kalman_filter_ball.predict()
            smoothed_ball_position = kalman_filter_ball.update(ball_position)

            if last_ball_position:

                ball_distance = math.sqrt((smoothed_ball_position[0] - last_ball_position[0])**2 +
                                          (smoothed_ball_position[1] - last_ball_position[1])**2)
                

                ball_time_elapsed = current_time - last_ball_time
                

                if ball_time_elapsed > 0:
                    ball_velocity = ball_distance / ball_time_elapsed
                    print(f"Ball Velocity: {ball_velocity:.2f} units/sec")
            

            last_ball_position = smoothed_ball_position
            last_ball_time = current_time
        

        robots = detection.robots_blue + detection.robots_yellow  

        for i, robot in enumerate(robots):
            robot_position = (robot.x, robot.y)


            kalman_filters_robots[i].predict()
            smoothed_robot_position = kalman_filters_robots[i].update(robot_position)

            if last_robot_positions[i]:

                robot_distance = math.sqrt((smoothed_robot_position[0] - last_robot_positions[i][0])**2 +
                                           (smoothed_robot_position[1] - last_robot_positions[i][1])**2)
                

                robot_time_elapsed = current_time - last_robot_times[i]
                

                if robot_time_elapsed > 0:
                    robot_velocity = robot_distance / robot_time_elapsed
                    print(f"Robot {i} Velocity: {robot_velocity:.2f} units/sec")
            

            last_robot_positions[i] = smoothed_robot_position
            last_robot_times[i] = current_time
