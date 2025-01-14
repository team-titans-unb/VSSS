from sendData import control_and_log_pwm

# IP da ESP32
IP = "192.168.0.103"
# IP = "192.168.137.83"

# Lista de valores de PWM para identificação de sistemas
# pwm_values = [
#     (50, 50),
#     (90, 90),
#     (150, 150),
#     (200, 200),
#     (250, 250),
#     (0, 0),
# ]
#              r    l
pwm_values = [(150, 100)]

# Realiza o controle e registra os dados no arquivo CSV
control_and_log_pwm(IP, pwm_values, log_filename="motor_identification.csv", duration=20)
control_and_log_pwm(IP, [(0,0)], log_filename="motor_identification.csv", duration=1)
