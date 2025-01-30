import socket
import serial
import time
import csv

# Configuração de comunicação com o robô
ROBOT_IP = "192.168.0.103"  # Endereço IP do robô
ROBOT_PORT = 80  # Porta do robô
SERIAL_PORT = "/dev/ttyUSB0"  # Porta serial conectada ao ESP32
SERIAL_BAUDRATE = 19200  # Baudrate da porta serial

# Configuração do arquivo CSV
CSV_FILENAME = "pwm_serial_data.csv"

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
    combined_value = (pwm_left << 16) | pwm_right  # Combina os valores de PWM em 32 bits
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((robot_ip, robot_port))
            s.sendall(combined_value.to_bytes(4, byteorder='little'))
            print(f"Sent PWM: Left={pwm_left}, Right={pwm_right}")
    except Exception as e:
        print(f"Error sending PWM: {e}")

def read_serial_data(serial_port):
    """
    Lê dados da porta serial.

    Parameters:
    serial_port (serial.Serial): Objeto da porta serial configurada.

    Returns:
    str: Dados brutos recebidos da porta serial.
    """
    try:
        # Lê uma linha da porta serial e decodifica
        line = serial_port.readline().decode('utf-8').strip()
        print(f"Received from serial: {line}")  # Exibe os dados recebidos
        return line
    except Exception as e:
        print(f"Error reading from serial: {e}")
        return None

def main():
    # Inicializa a comunicação serial
    try:
        serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
        time.sleep(2)  # Tempo para estabilizar a conexão serial
        print(f"Serial port {SERIAL_PORT} opened successfully.")
    except Exception as e:
        print(f"Failed to open serial port: {e}")
        return

    # Configuração inicial do arquivo CSV
    with open(CSV_FILENAME, mode='w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["Timestamp", "PWM_Left", "PWM_Right", "Serial_Data"])

        # Envio de PWM e leitura da serial
        try:
            for pwm_left, pwm_right in [(100, 100), (200, 200), (150, 150), (0, 0)]:
                # Envia os valores de PWM para o robô
                send_pwm(pwm_left, pwm_right, ROBOT_IP, ROBOT_PORT)

                # Lê a resposta da porta serial
                start_time = time.time()
                while time.time() - start_time < 5:  # Aguarda até 5 segundos de resposta
                    serial_data = read_serial_data(serial_port)
                    if serial_data:
                        # Salva os dados no CSV
                        csv_writer.writerow([time.time(), pwm_left, pwm_right, serial_data])

                # Aguarda 2 segundos antes de enviar o próximo comando
                time.sleep(2)

        except KeyboardInterrupt:
            print("Interrupted by user.")
        except Exception as e:
            print(f"Error during execution: {e}")

    print(f"Data saved to {CSV_FILENAME}.")
    serial_port.close()

if __name__ == "__main__":
    main()