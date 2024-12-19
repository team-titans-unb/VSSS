from sendData import send_speed_and_direction
import time

# Apenas um exemplo de como usar em um código qualquer
# Vão precisar saber os ips da esp antes, pra isso temos duas opções:
# 1 - Conecte a ESP à entrada USB do seu computador e abra o monitor serial do arduino IDE
# lá a ESP irá imprimir seu endereço IP
# 2 - Se conectar a ESP a uma rede roteada pelo teu celular ou notebook, é possivel ver o 
# endereço de IP na lista de aparelhos conectados nas configurações da rede.

goleiro_ip = '192.168.0.103' # Este ip aqui nao irá funcionar, é apenas um exemplo
#send_speed_and_direction(Direito, Esquero, direção, direção, IP)

send_speed_and_direction(255, 255, 0, 0, goleiro_ip) # é aqui que vocês devem controlar o pwm.
time.sleep(5)

##--------- isso aqui abaixo só faz o robô voltar e parar
send_speed_and_direction(255, 255, 1, 1, goleiro_ip) # faz o robô voltar para perto do ponto de partida
time.sleep(2.5)
send_speed_and_direction(0, 0, 1, 1, goleiro_ip) # faz o robô parar
