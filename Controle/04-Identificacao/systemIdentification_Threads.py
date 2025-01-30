import socket
import struct
import time
import csv
import serial
import wrapper_pb2 as wr
from collections import deque
from threading import Thread
import math

# Configuração da comunicação
ROBOT_IP = "192.168.0.103"  # IP do robô
ROBOT_PORT = 80  # Porta do robô
SERIAL_PORT = "/dev/ttyUSB0"  # Porta serial do robô (ex: COM3 ou /dev/ttyUSB0)
SERIAL_BAUDRATE = 19200  # Baudrate da porta serial
VISION_IP = "224.5.23.2"  # IP do sistema de visão
VISION_PORT = 10011  # Porta do sistema de visão
ROBOT_ID = 4  # ID do robô azul a ser controlado

PWM_STOP = (0, 0)  # PWM para parar o robô

def send_pwm(pwm_left, pwm_right, robot_ip, robot_port):
    """
    Envia valores de PWM para o robô via socket.

    Parameters:
    pwm_left (int): Valor de PWM para a roda esquerda.
    pwm_right (int): Valor de PWM para a roda direita.
    robot_ip (str): Endereço IP do robô.
    robot_port (int): Porta TCP do robô.

    Returns:
    None
    """
    combined_value = (pwm_left << 16) | pwm_right
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((robot_ip, robot_port))
            s.sendall(combined_value.to_bytes(4, byteorder='little'))
            print(f"Sent PWM: Left={pwm_left}, Right={pwm_right}")
    except Exception as e:
        print(f"Error sending PWM: {e}")

def read_rpm(serial_port):
    """
    Lê os valores de RPM enviados pelo robô via porta serial.

    Parameters:
    serial_port (serial.Serial): Objeto da porta serial configurada.

    Returns:
    tuple: (rpm_right, rpm_left) valores de RPM das rodas direita e esquerda.
    """
    try:
        line = serial_port.readline().decode('utf-8').strip()
        if "RPM R:" in line and "RPM L:" in line:
            parts = line.split("|")
            rpm_right = float(parts[0].split(":")[1].strip())
            rpm_left = float(parts[1].split(":")[1].strip())
            return rpm_right, rpm_left
    except Exception as e:
        print(f"Error reading RPM: {e}")
    return None, None

def receive_vision_frame(vision_sock):
    """
    Recebe um frame de dados do sistema de visão via multicast UDP.

    Parameters:
    vision_sock (socket.socket): Socket configurado para o sistema de visão.

    Returns:
    SSL_WrapperPacket: Objeto protobuf contendo os dados do frame.
    """
    data, _ = vision_sock.recvfrom(1024)
    return wr.SSL_WrapperPacket().FromString(data)

def filter_robot_data(frame, robot_id, team="blue"):
    """
    Filtra os dados de um robô específico no frame do sistema de visão.

    Parameters:
    frame (SSL_WrapperPacket): Frame recebido do sistema de visão.
    robot_id (int): ID do robô a ser filtrado.
    team (str): Time do robô ('blue' ou 'yellow').

    Returns:
    SSL_DetectionRobot: Dados do robô filtrado, ou None se não encontrado.
    """
    robots = frame.detection.robots_blue if team == "blue" else frame.detection.robots_yellow
    for robot in robots:
        if robot.robot_id == robot_id:
            return robot
    return None

def calculate_displacement_and_velocity(previous, current, delta_time):
    """
    Calcula o deslocamento e a velocidade do robô com base nas posições anteriores e atuais.
    """
    displacement_x = (current["x"] - previous["x"]) / 1000  # Convertendo mm para metros
    displacement_y = (current["y"] - previous["y"]) / 1000  # Convertendo mm para metros
    displacement_total = math.sqrt(displacement_x**2 + displacement_y**2)

    velocity_x = displacement_x / delta_time if delta_time > 0 else 0
    velocity_y = displacement_y / delta_time if delta_time > 0 else 0
    velocity_total = displacement_total / delta_time if delta_time > 0 else 0

    return displacement_x, displacement_y, displacement_total, velocity_x, velocity_y, velocity_total


# def main(PWM_INITIAL = (100, 100), duration = 10, CSV_FILENAME = "robot_system_identification.csv"):
#     # Inicializa o socket para o sistema de visão
#     vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
#     vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
#     vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
#                            struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
#     vision_sock.bind((VISION_IP, VISION_PORT))

#     # Buffers para armazenar os dados
#     vision_data = deque()  # Armazena os dados do sistema de visão
#     serial_data = []  # Armazena os dados da porta serial
#     pwm_data = PWM_INITIAL  # PWM enviado (será salvo no final)

#     # Tempo inicial e duração do experimento
#     start_time = time.time()

#     # Envia o PWM inicial para o robô
#     send_pwm(PWM_INITIAL[0], PWM_INITIAL[1], ROBOT_IP, ROBOT_PORT)

#     # Inicializa a comunicação serial
#     serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=10)
#     # time.sleep(2)  # Tempo para estabilizar a serial

#     # Thread de leitura da visão
#     def read_vision():
#         while time.time() - start_time < duration:
#             frame = receive_vision_frame(vision_sock)
#             robot_data = filter_robot_data(frame, ROBOT_ID, team="blue")
#             if robot_data:
#                 vision_data.append({
#                     "timestamp": time.time(),
#                     "x": robot_data.x,
#                     "y": robot_data.y,
#                     "orientation": robot_data.orientation
#                 })

#     # Thread de leitura da serial
#     def read_serial():
#         while time.time() - start_time < duration:
#             rpm_left, rpm_right = read_rpm(serial_port)
#             if ((rpm_left != 0 and rpm_left is not None) and (rpm_right != 0 and rpm_right is not None)):
#                 # print(f"{rpm_left}, {rpm_right}")
#                 serial_data.append({
#                     "timestamp": time.time(),
#                     "rpm_left": rpm_left,
#                     "rpm_right": rpm_right
#                 })
#             # time.sleep(0.001)  # Serial a cada 100ms

#     # Executa as threads
#     vision_thread = Thread(target=read_vision)
#     serial_thread = Thread(target=read_serial)

#     vision_thread.start()
#     serial_thread.start()

#     vision_thread.join()
#     serial_thread.join()

#     # Envia o comando para parar o robô
#     send_pwm(PWM_STOP[0], PWM_STOP[1], ROBOT_IP, ROBOT_PORT)

#     # Sincroniza os dados e salva no CSV
#     with open(CSV_FILENAME, mode='w', newline='') as csvfile:
#         csv_writer = csv.writer(csvfile)
#         csv_writer.writerow(["Timestamp", "PWM_Left", "PWM_Right", "RPM_Left", "RPM_Right", "X", "Y", "Orientation"])

#         rpm_index = 0  # Índice para navegar nos dados da serial

#         for vision_entry in vision_data:
#             # Avança no serial_data até encontrar um valor de RPM válido
#             while rpm_index < len(serial_data) and (serial_data[rpm_index]["rpm_left"] == 0 and serial_data[rpm_index]["rpm_right"] == 0):
#                 rpm_index += 1

#             # Se não houver mais valores válidos de RPM, interrompe o loop
#             if rpm_index >= len(serial_data):
#                 break

#             # Associa o dado de visão ao primeiro RPM válido encontrado
#             serial_entry = serial_data[rpm_index]
#             csv_writer.writerow([
#                 serial_entry["timestamp"],
#                 pwm_data[0],  # PWM_Left
#                 pwm_data[1],  # PWM_Right
#                 serial_entry["rpm_left"],
#                 serial_entry["rpm_right"],
#                 vision_entry["x"],
#                 vision_entry["y"],
#                 vision_entry["orientation"]
#             ])

#             # Avança para o próximo dado de RPM
#             rpm_index += 1

#     print(f"Data saved to {CSV_FILENAME}")

def main(PWM_INITIAL = (100, 100), duration = 10, CSV_FILENAME = "robot_system_identification.csv"):
    # Inicializa o socket para o sistema de visão
    vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
    vision_sock.bind((VISION_IP, VISION_PORT))

    # Buffers para armazenar os dados
    vision_data = deque()  # Armazena os dados do sistema de visão
    serial_data = []  # Armazena os dados da porta serial
    pwm_data = PWM_INITIAL  # PWM enviado (será salvo no final)

    # Tempo inicial e duração do experimento
    start_time = time.time()

    # Envia o PWM inicial para o robô
    send_pwm(PWM_INITIAL[0], PWM_INITIAL[1], ROBOT_IP, ROBOT_PORT)

    # Inicializa a comunicação serial
    serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=10)

    # Thread de leitura da visão
    def read_vision():
        while time.time() - start_time < duration:
            frame = receive_vision_frame(vision_sock)
            robot_data = filter_robot_data(frame, ROBOT_ID, team="blue")
            if robot_data:
                vision_data.append({
                    "timestamp": time.time(),
                    "x": robot_data.x,
                    "y": robot_data.y,
                    "orientation": robot_data.orientation
                })

    # Thread de leitura da serial
    def read_serial():
        while time.time() - start_time < duration:
            rpm_left, rpm_right = read_rpm(serial_port)
            if ((rpm_left != 0 and rpm_left is not None) and (rpm_right != 0 and rpm_right is not None)):
                serial_data.append({
                    "timestamp": time.time(),
                    "rpm_left": rpm_left,
                    "rpm_right": rpm_right
                })

    # Executa as threads
    vision_thread = Thread(target=read_vision)
    serial_thread = Thread(target=read_serial)

    vision_thread.start()
    serial_thread.start()

    vision_thread.join()
    serial_thread.join()

    # Envia o comando para parar o robô
    send_pwm(PWM_STOP[0], PWM_STOP[1], ROBOT_IP, ROBOT_PORT)

    # Sincroniza os dados e salva no CSV
    with open(CSV_FILENAME, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Timestamp", "PWM_Left", "PWM_Right", "RPM_Left", "RPM_Right", "X", "Y", "Orientation", "Displacement_X", "Displacement_Y", "Displacement_Total", "Velocity_X", "Velocity_Y", "Velocity_Total"])

        rpm_index = 0  # Índice para navegar nos dados da serial
        previous_vision = None

        for vision_entry in vision_data:
            # Avança no serial_data até encontrar um valor de RPM válido
            while rpm_index < len(serial_data) and (serial_data[rpm_index]["rpm_left"] == 0 and serial_data[rpm_index]["rpm_right"] == 0):
                rpm_index += 1

            # Se não houver mais valores válidos de RPM, interrompe o loop
            if rpm_index >= len(serial_data):
                break

            # Associa o dado de visão ao primeiro RPM válido encontrado
            serial_entry = serial_data[rpm_index]
            
            # Calcula deslocamento e velocidade
            if previous_vision:
                delta_time = vision_entry["timestamp"] - previous_vision["timestamp"]
                displacement_x, displacement_y, displacement_total, velocity_x, velocity_y, velocity_total = calculate_displacement_and_velocity(previous_vision, vision_entry, delta_time)
            else:
                displacement_x = displacement_y = displacement_total = velocity_x = velocity_y = velocity_total = 0

            previous_vision = vision_entry
            
            csv_writer.writerow([
                serial_entry["timestamp"],
                pwm_data[0],  # PWM_Left
                pwm_data[1],  # PWM_Right
                serial_entry["rpm_left"],
                serial_entry["rpm_right"],
                vision_entry["x"] / 1000,  # Convertendo mm para metros
                vision_entry["y"] / 1000,  # Convertendo mm para metros
                vision_entry["orientation"],
                displacement_x,
                displacement_y,
                displacement_total,
                velocity_x,
                velocity_y,
                velocity_total
            ])

            # Avança para o próximo dado de RPM
            rpm_index += 1

    print(f"Data saved to {CSV_FILENAME}")

if __name__ == "__main__":
    PWMR = 225
    PWML = 220

    CSV_FILENAME = f"R{PWMR}L{PWML}.csv"
    PWM_INITIAL = (PWMR, PWML)  # PWM inicial (direita, esquerda)
    duration = 3  # Duração do experimento em segundos

    main(PWM_INITIAL, duration, CSV_FILENAME)