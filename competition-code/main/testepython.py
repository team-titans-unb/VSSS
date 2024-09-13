import socket
import time 
import random

ESP32_IP = '192.168.179.89' 
PORT = 80  

def send_speed_and_direction(speed, direction):
    combined_value = (speed << 8) | direction
    max_retries = 5  # Número máximo de tentativas
    retry_delay = 2  # Tempo de espera (em segundos) entre tentativas

    for attempt in range(max_retries):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"Attempt {attempt+1}/{max_retries} to connect...")
                s.connect((ESP32_IP, PORT))
                s.sendall(combined_value.to_bytes(2, byteorder='little'))  # Envia o valor como 2 bytes
                print("Data sent successfully.")
                return  # Sai da função se a conexão for bem-sucedida
        except ConnectionRefusedError:
            print(f"Connection refused, retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)  # Aguarda antes de tentar novamente

    print("Failed to connect to ESP32 after multiple attempts.")


while 1:
    time.sleep()
    val = random.randint(0,255)
    direction = random.randint(0,1)
    send_speed_and_direction(val, direction)
    print("pwm:", val)
    print("pwm:", direction)
    

