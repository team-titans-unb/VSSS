from sendData import control_and_log_pwm

# IP da ESP32
IP = "192.168.137.115"

# Lista de valores de PWM para identificação de sistemas
pwm_values = [
    (50, 50),
    (90, 90),
    (150, 150),
    (200, 200),
    (250, 250),
    (0, 0),
]

# Realiza o controle e registra os dados no arquivo CSV
control_and_log_pwm(IP, pwm_values, log_filename="motor_identification.csv", duration=5)
