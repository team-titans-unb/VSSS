from sendData import send_speed_and_direction

# Apenas um exemplo de como usar em um código qualquer
# Vão precisar saber os ips da esp antes :/, mas a gente tem que ver 
# como deixar os ips fixos
goleiro_ip = '100.168.30.100' # Este ip aqui nao irá funcionar, é apenas um exemplo
robo2 = ''
send_speed_and_direction(255, 255, 1, 1, goleiro_ip)
send_speed_and_direction(255, 255, 0, 0, robo2)

