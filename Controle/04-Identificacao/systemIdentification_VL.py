from sendData import *
import time

# Apenas um exemplo de como usar em um código qualquer
# Vão precisar saber os ips da esp antes, pra isso temos duas opções:
# 1 - Conecte a ESP à entrada USB do seu computador e abra o monitor serial do arduino IDE
# lá a ESP irá imprimir seu endereço IP
# 2 - Se conectar a ESP a uma rede roteada pelo teu celular ou notebook, é possivel ver o 
# endereço de IP na lista de aparelhos conectados nas configurações da rede.

#goleiro_ip = '192.168.0.110'
goleiro_ip = '192.168.0.111' #robo azul-+
# goleiro_ip = '192.168.137.70'
#send_data(Direito, Esquero, direção, direção, IP)

send_data(120, 120, 1, 1, goleiro_ip) # é aqui que vocês devem controlar o pwm.
time.sleep(2)
send_data(255, 255, 1, 1, goleiro_ip) 
time.sleep(1)
# wheel_speed_1, wheel_speed_2 = receive_data(goleiro_ip)
# print(f"Velocidade das rodas: {wheel_speed_1}, {wheel_speed_2}")
time.sleep(5)
##--------- isso aqui abaixo só faz o robô voltar e parar
# send_data(150, 150, 1, 1, goleiro_ip) # faz o robô voltar para perto do ponto de partida
# time.sleep(5)
send_data(0, 0, 1, 1, goleiro_ip) # faz o robô parar
