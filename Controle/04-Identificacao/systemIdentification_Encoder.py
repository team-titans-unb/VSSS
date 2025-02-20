import socket
import time
import csv
import serial

# Configuração da comunicação
ROBOT_IP = "192.168.0.103"
ROBOT_PORT = 80
SERIAL_PORT = "COM3"
SERIAL_BAUDRATE = 19200

def send_pwm(pwm_left, pwm_right, robot_ip, robot_port):
    direction1 = 1 if pwm_left >= 0 else 0
    direction2 = 1 if pwm_right >= 0 else 0 
    combined_value = (pwm_left << 24) | (pwm_right << 16) | (direction1 << 8) | direction2
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((robot_ip, robot_port))
        s.sendall(combined_value.to_bytes(4, byteorder='little'))

def read_rpm(serial_port):
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

def append_average_to_csv(filename):
    with open(filename, mode='r', newline='') as csvfile:
        rows = list(csv.reader(csvfile))
        if len(rows) > 1:  # Verifica se há dados além do cabeçalho
            data = rows[1:][-20:]  # Pega as últimas 20 linhas
            columns = list(zip(*data))
            averages = [sum(map(float, col)) / len(col) for col in columns]
            
    with open(filename, mode='a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(averages)

def main(pwm_right=100, pwm_left=100, duration=20, CSV_FILENAME="identification.csv"):
    serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1)
    # time.sleep(2)
    
    with open(CSV_FILENAME, mode='a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["PWM_Right", "PWM_Left", "RPM_Right", "RPM_Left"])

        print("Sending PWM values to robot...")
        send_pwm(pwm_right, pwm_left, ROBOT_IP, ROBOT_PORT)
        start_time = time.time()

        while time.time() - start_time < duration:
            rpm_right, rpm_left = read_rpm(serial_port)
            timestamp = time.time() - start_time
            csv_writer.writerow([pwm_right, pwm_left, rpm_right, rpm_left])

        print("Stopping robot...")
        send_pwm(0, 0, ROBOT_IP, ROBOT_PORT)
    
    append_average_to_csv(CSV_FILENAME)
    print(f"Experiment completed. Data saved to {CSV_FILENAME}")

if __name__ == "__main__":
    PWM_STEP = 5
    PWM_MAX = 255
    PWM_INI = 75
    Max_iter = int((PWM_MAX - PWM_INI)/PWM_STEP)
    CSV_FILENAME = "Rampa.csv"
    with open(CSV_FILENAME, mode='a', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(["PWM_Right", "PWM_Left", "RPM_Right", "RPM_Left"])
        serial_port = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=1) 
        for i in range(Max_iter+1):

            pwm_left = PWM_INI + i * PWM_STEP
            pwm_right = PWM_INI + i * PWM_STEP

            print("Sending PWM values to robot...")
            send_pwm(pwm_right, pwm_left, ROBOT_IP, ROBOT_PORT)
            start_time = time.time()

            while time.time() - start_time < 5:
                rpm_right, rpm_left = read_rpm(serial_port)
                timestamp = time.time() - start_time
                if rpm_right != 0: 
                    csv_writer.writerow([pwm_right, pwm_left, rpm_right, rpm_left])

        print("Stopping robot...")
        send_pwm(0, 0, ROBOT_IP, ROBOT_PORT)
        
        # append_average_to_csv(CSV_FILENAME)
        print(f"Experiment completed. Data saved to {CSV_FILENAME}")
