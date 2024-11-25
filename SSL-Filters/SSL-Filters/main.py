import sslclient
import time
import csv
from collections import deque
from filters import moving_average_position, KalmanFilter2D

# Configurações do cliente SSL
c = sslclient.client(ip='224.5.23.2', port=10006)
c.connect()

# Configurações da média móvel
ball_position_history = deque(maxlen=5)

# Inicializa o filtro de Kalman
kalman_filter = KalmanFilter2D()

# Abrir arquivo CSV para registrar os dados
with open('ball_positions.csv', mode='w') as file:
    writer = csv.writer(file)
    writer.writerow(['time', 'raw_x', 'raw_y', 'moving_avg_x', 'moving_avg_y', 'kalman_x', 'kalman_y'])

    while True:
        # Receber dados do cliente
        data = c.receive()
        current_time = time.time()
        
        if data.HasField('detection'):
            detection = data.detection

            # Acessa a posição da bola
            ball_position = (detection.balls[0].x, detection.balls[0].y) if detection.balls else None

            if ball_position:
                # Suavização pela média móvel
                ball_position_history.append(ball_position)
                moving_avg_position = moving_average_position(ball_position_history)
                
                # Suavização pelo filtro de Kalman
                kalman_position = kalman_filter.update(ball_position)

                # Registrar dados no CSV
                writer.writerow([
                    current_time, 
                    ball_position[0], ball_position[1],  # Posições brutas
                    moving_avg_position[0], moving_avg_position[1],  # Posição pela média móvel
                    kalman_position[0], kalman_position[1]  # Posição pelo filtro de Kalman
                ])

                print(f"Raw: {ball_position}, Moving Avg: {moving_avg_position}, Kalman: {kalman_position}")