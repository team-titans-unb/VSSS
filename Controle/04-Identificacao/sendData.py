import socket
import time
import csv

# wifi: Titans
# pass: Titans2024

PORT = 80  

def send_data(speed1, speed2, direction1, direction2, IP):
    combined_value = (speed1 << 24) | (speed2 << 16) | (direction1 << 8) | direction2
    retry_delay = 2  # Tempo de espera (em segundos) entre tentativas

    while True:  # Loop infinito para tentativas indefinidas
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print("Trying to connect to ESP32...")
                s.connect((IP, PORT))
                s.sendall(combined_value.to_bytes(4, byteorder='little'))  # Envia o valor como 4 bytes
                print("Data sent successfully.")
                return  # Sai da função após o envio bem-sucedido
        except ConnectionRefusedError:
            print(f"Connection refused, retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)  # Aguarda antes de tentar novamente

def receive_data(IP):
    """Recebe os dados da ESP32 em dois frames separados."""
    retry_delay = 2  # Tempo de espera (em segundos) entre tentativas

    while True:  # Loop infinito para tentativas indefinidas
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print("Trying to connect to ESP32 to receive data...")
                s.connect((IP, PORT))
                
                # Recebe o primeiro frame (4 bytes para a velocidade da roda 1)
                frame1 = s.recv(4)
                if len(frame1) < 4:
                    print("Incomplete frame 1 received, retrying...")
                    continue
                wheel_speed_1 = int.from_bytes(frame1, byteorder='little', signed=True)

                # Recebe o segundo frame (4 bytes para a velocidade da roda 2)
                frame2 = s.recv(4)
                if len(frame2) < 4:
                    print("Incomplete frame 2 received, retrying...")
                    continue
                wheel_speed_2 = int.from_bytes(frame2, byteorder='little', signed=True)

                print(f"Received data from ESP32: Wheel 1 Speed = {wheel_speed_1}, Wheel 2 Speed = {wheel_speed_2}")
                return wheel_speed_1, wheel_speed_2
        except ConnectionRefusedError:
            print(f"Connection refused, retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)  # Aguarda antes de tentar novamente

def control_and_log_pwm(ip, pwm_values, log_filename="motor_data.csv", duration=5):
    """
    Envia valores de PWM para as rodas do robô, lê os valores de RPM correspondentes durante um período
    e salva os dados em um arquivo CSV.

    :param ip: IP da ESP32.
    :param pwm_values: Lista de pares de valores de PWM (pwm1, pwm2) para as rodas.
    :param log_filename: Nome do arquivo CSV onde os dados serão registrados.
    :param duration: Duração (em segundos) para registrar os dados para cada par de valores de PWM.
    """
    print(f"Starting system identification. Logging data to {log_filename}")
    
    # Abre o arquivo CSV para escrita
    with open(log_filename, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Escreve o cabeçalho do arquivo CSV
        csv_writer.writerow(["PWM_Right", "PWM_Left", "RPM_Right", "RPM_Left"])
        
        for pwm1, pwm2 in pwm_values:
            try:
                print(f"Sending PWM values: Wheel 1 = {pwm1}, Wheel 2 = {pwm2}")
                # Envia os valores de PWM para o robô
                send_data(pwm1, pwm2, 0 if pwm1 >= 0 else 1, 0 if pwm2 >= 0 else 1, ip)
                
                # Começa a registrar os dados por 5 segundos
                start_time = time.time()
                while time.time() - start_time < duration:
                    # Lê os valores de RPM retornados pelos encoders
                    rpm1, rpm2 = receive_data(ip)
                    
                    # Salva os valores no arquivo CSV com timestamp
                    timestamp = time.time()
                    csv_writer.writerow([pwm1, pwm2, rpm1, rpm2])
                    print(f"Logged data: PWM1={pwm1}, PWM2={pwm2}, RPM1={rpm1}, RPM2={rpm2}")
                    
                    # Aguarda um curto intervalo para evitar excesso de chamadas
                    time.sleep(0.01)
                # time.sleep(5)
            except Exception as e:
                print(f"Error during data logging: {e}")
                continue

    print(f"System identification complete. Data logged to {log_filename}")