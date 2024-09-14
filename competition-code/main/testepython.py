import socket
import time
import random

ESP32_IP = '192.168.179.89' 
PORT = 80  

def send_speed_and_direction(speed1, speed2, direction1, direction2):
    combined_value = (speed1 << 24) | (speed2 << 16) | (direction1 << 8) | direction2
    retry_delay = 2  # Tempo de espera (em segundos) entre tentativas

    while True:  # Loop infinito para tentativas indefinidas
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print("Trying to connect to ESP32...")
                s.connect((ESP32_IP, PORT))
                s.sendall(combined_value.to_bytes(4, byteorder='little'))  # Envia o valor como 4 bytes
                print("Data sent successfully.")
                return  # Sai da função após o envio bem-sucedido
        except ConnectionRefusedError:
            print(f"Connection refused, retrying in {retry_delay} seconds...")
            time.sleep(retry_delay)  # Aguarda antes de tentar novamente


def test():
    cont = 0
    while True:
        cont += 1 
        print(f"Attempt: {cont}")
        
        # Gera valores aleatórios para PWM e direção
        val1 = random.randint(0, 255)
        val2 = random.randint(0, 255)
        direction1 = random.randint(0, 1)
        direction2 = random.randint(0, 1)
        
        print("pwm1:", hex(val1))
        print("direction1:", hex(direction1))
        print("pwm2:", hex(val2))
        print("direction2:", hex(direction2))
        
        send_speed_and_direction(val1, val2, direction1, direction2)
        time.sleep(0.1)  # Pausa antes de gerar novos valores e enviar novamente

if __name__ == "__main__":
    test()