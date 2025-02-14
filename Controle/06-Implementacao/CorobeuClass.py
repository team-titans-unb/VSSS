import socket
import time
import math
import struct
import signal
import matplotlib.pyplot as plt
import wrapper_pb2 as wr
from DrawField import draw_field, plot_robot_path

def init_vision_socket(VISION_IP="224.5.23.2", VISION_PORT=10015):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    struct.pack("=4sl", socket.inet_aton(VISION_IP), socket.INADDR_ANY))
    sock.bind((VISION_IP, VISION_PORT))
    return sock

class Corobeu:
    def __init__(self, robot_ip, robot_port, robot_id, vision_sock, kp, ki, kd):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot_id = robot_id
        self.vision_sock = vision_sock
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.v_max = 255
        self.v_min = 0
        self.v_linear = 340
        self.phi = 0
        
        self.last_speed_time = time.time()
        self.time_log = []
        self.set_point_log = []
        self.output_log = []
        
        # Listas para armazenar as coordenadas do robô
        self.x_log = []
        self.y_log = []
        
        signal.signal(signal.SIGINT, self.plot_response)
        signal.signal(signal.SIGTERM, self.plot_response)
    
    def get_position(self):
        data, _ = self.vision_sock.recvfrom(1024)
        frame = wr.SSL_WrapperPacket().FromString(data)
        for robot in frame.detection.robots_blue:
            if robot.robot_id == self.robot_id:
                return robot.x / 1000, robot.y / 1000, robot.orientation
        return None, None, None
    
    def speed_control(self, U, omega):
        limiar_min = 68
        sat_omega = abs(((limiar_min*6.4) - (2*U))/7.5)
        omega = max(min(omega, sat_omega), -sat_omega)
        vr = (2 * U + omega * 7.5) / (2 * 3.2)
        vl = (2 * U - omega * 7.5) / (2 * 3.2)
        print(f"Prev: {vr}, {vl}")
        vr = max(min(vr, self.v_max), self.v_min)
        vl = max(min(vl, self.v_max), self.v_min)
        
        return int(vl), int(vr)
    
    def send_speed(self, speed_left, speed_right):
        direction_left = 1 if speed_left >= 0 else 0
        direction_right = 1 if speed_right >= 0 else 0
        combined_value = (abs(speed_left) << 24) | (abs(speed_right) << 16) | (direction_left << 8) | direction_right
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(combined_value.to_bytes(4, byteorder='little'))
                print("data sent")
        except Exception as e:
            print(f"Erro ao enviar dados: {e}")
    
    def follow_path(self, pathX, pathY, End_position):
        deltaT = 0.7
        a = 1
        interror_phi = Integral_part_phi = fant_phi = 0
        phi_obs = 0
        
        while a == 1:
            # x, y, _ = self.get_position()
            x, y, phi_obs = self.get_position()
            if x is None or y is None:
                continue
            
            # Armazenar as coordenadas do robô
            self.x_log.append(x)
            self.y_log.append(y)

            phid = math.atan2((pathY - y), (pathX - x))
            error_phi = phid - phi_obs
            omega, fant_phi, interror_phi, Integral_part_phi = self.pid_controller(self.kp, self.ki, self.kd, deltaT, error_phi, interror_phi, fant_phi, Integral_part_phi)
            
            # phi_obs = phi_obs + omega * deltaT

            error_distance = math.sqrt((pathY - y)**2 + (pathX - x)**2)
            error_distance_global = math.sqrt((End_position[1] - y) ** 2 + (End_position[0] - x) ** 2)
            
            U = self.v_linear
            
            current_time = time.time()
            if current_time - self.last_speed_time >= 0.7:
                vl, vr = self.speed_control(U, omega)
                self.send_speed(vl, vr)
                self.last_speed_time = current_time
                print(f"X: {x}, Y: {y}, Phi: {phi_obs}")
                print(f"Erro: {error_phi}, Omega: {omega}, Phid: {phid}")
                print(f"Vl: {vl}, Vr: {vr}")

                self.time_log.append(current_time)
                self.set_point_log.append(phid)
                self.output_log.append(omega)
            
            if error_distance <= 0.1:
                a = 0
            if error_distance_global <= 0.07:
                self.send_speed(0, 0)
                a = 0
    
    def pid_controller(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
        Integral_saturation = 5
        raizes = math.sqrt(kd), math.sqrt(kp), math.sqrt(ki)
        Filter_e = 1 / (max(raizes) * 10)   
        unomenosalfaana = math.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror += error
        f = unomenosalfaana * fant + alfaana * error
        deerror = (f - fant) / deltaT if fant != 0 else f / deltaT
        Integral_part = min(max(Integral_part + ki * interror * deltaT, -Integral_saturation), Integral_saturation)
        PID = kp * error + Integral_part + deerror * kd
        return PID, f, interror, Integral_part
    
    def plot_response(self, signum=None, frame=None):
        self.send_speed(0, 0)
        
        # Plotar o caminho percorrido pelo robô
        if self.x_log and self.y_log:
            errorX = 0 - self.x_log[-1]
            errorY = 0 - self.y_log[-1] 
            fileName = f"P1A270-x:{errorX}-y:{errorY}-E3"
            plot_robot_path([], [], self.x_log, self.y_log, fileName)
            print(fileName)

        plt.figure()
        plt.plot(self.time_log, self.set_point_log, label='Set Point', linestyle='--')
        plt.plot(self.time_log, self.output_log, label='PID Output')
        plt.xlabel('Time (s)')
        plt.ylabel('Response')
        plt.title('System Response vs. Set Point')
        plt.legend()
        plt.grid()
        plt.show()
        exit(0)

if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = "192.168.0.103"
    ROBOT_PORT = 80
    ROBOT_ID = 4

    Kp = 25         #20
    Ki = 5          #5
    Kd = 3

    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ROBOT_ID, vision_sock, Kp, Ki, Kd)
    crb01.follow_path(0.7, 0, (0.7, 0))
    crb01.plot_response()