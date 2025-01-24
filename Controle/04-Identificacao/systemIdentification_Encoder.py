import socket
import struct
import time
import csv
import serial
import wrapper_pb2 as wr

# Configuração da comunicação
ROBOT_IP = "192.168.4.1"  # IP do robô
ROBOT_PORT = 80  # Porta do robô
SERIAL_PORT = "COM3"  # Porta serial do robô (ex: COM3 ou /dev/ttyUSB0)
SERIAL_BAUDRATE = 19200  # Baudrate da porta serial
VISION_IP = "224.5.23.2"  # IP do sistema de visão
VISION_PORT = 10011  # Porta do sistema de visão
ROBOT_ID = 4  # ID do robô azul a ser controlado

# Configuração do arquivo de log
CSV_FILENAME = "robot_system_identification.csv"

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
    # Combina os valores de PWM em um único valor de 32 bits
    combined_value = (pwm_left << 16) | pwm_right
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((robot_ip, robot_port))
        # Envia os 4 bytes para o robô no formato little-endian
        s.sendall(combined_value.to_bytes(4, byteorder='little'))

def read_rpm(serial_port):
    """
    Lê os valores de RPM enviados pelo robô via porta serial.

    Parameters:
    serial_port (serial.Serial): Objeto da porta serial configurada.

    Returns:
    tuple: (rpm_right, rpm_left) valores de RPM das rodas direita e esquerda.
    """
    try:
        # Lê uma linha da porta serial e decodifica
        line = serial_port.readline().decode('utf-8').strip()
        if "RPM R:" in line and "RPM L:" in line:
            # Extrai os valores de RPM das rodas direita e esquerda
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

def main():
    """
    Realiza o experimento de identificação de sistemas do robô, integrando:
    1. Envio de valores de PWM para as rodas.
    2. Leitura de valores de RPM retornados pelo robô.
    3. Cálculo de distância percorrida e velocidade média via sistema de visão.
    4. Registro dos dados em um arquivo CSV.

    Returns:
    None
    """
    # Inicializa a comunicação serial
    serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    time.sleep(2)  # Tempo para inicializar a serial

    # Inicializa o socket para o sistema de visão
    vision_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vision_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    vision_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                           struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
    vision_sock.bind((VISION_IP, VISION_PORT))

    # Arquivo CSV para salvar os dados
    with open(CSV_FILENAME, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        # Cabeçalho do arquivo CSV
        csv_writer.writerow(["Timestamp", "PWM_Left", "PWM_Right", "RPM_Left", "RPM_Right",
                             "Distance_X", "Distance_Y", "Distance_Total",
                             "Velocity_X", "Velocity_Y", "Velocity_Total"])

        # Valores de PWM para as rodas
        pwm_left = 100
        pwm_right = 100

        # Envia os comandos iniciais
        print("Sending PWM values to robot...")
        send_pwm(pwm_left, pwm_right, ROBOT_IP, ROBOT_PORT)

        # Tempo inicial
        start_time = time.time()
        duration = 5  # Segundos
        initial_position = None

        while time.time() - start_time < duration:
            # Recebe dados da visão
            frame = receive_vision_frame(vision_sock)
            robot_data = filter_robot_data(frame, ROBOT_ID, team="blue")

            if robot_data:
                if initial_position is None:
                    initial_position = (robot_data.x, robot_data.y)
                current_position = (robot_data.x, robot_data.y)
                distance_x = current_position[0] - initial_position[0]
                distance_y = current_position[1] - initial_position[1]
                distance_total = (distance_x**2 + distance_y**2) ** 0.5

                # Leitura de RPM
                rpm_left, rpm_right = read_rpm(serial_port)

                # Tempo atual
                timestamp = time.time() - start_time

                # Velocidades médias
                velocity_x = distance_x / timestamp
                velocity_y = distance_y / timestamp
                velocity_total = distance_total / timestamp

                # Salva os dados no CSV
                csv_writer.writerow([timestamp, pwm_left, pwm_right,
                                     rpm_left, rpm_right,
                                     distance_x, distance_y, distance_total,
                                     velocity_x, velocity_y, velocity_total])

        # Para o robô
        print("Stopping robot...")
        send_pwm(0, 0, ROBOT_IP, ROBOT_PORT)

    print(f"Experiment completed. Data saved to {CSV_FILENAME}")

if __name__ == "__main__":
    main()
