import socket
import time
import math
import struct
import wrapper_pb2 as wr

def init_vision_socket(VISION_IP = "224.5.23.2", VISION_PORT = 10015):
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
        self.v_min = -255
        self.v_linear = 50
        self.phi = 0
    
    def get_position(self):
        data, _ = self.vision_sock.recvfrom(1024)
        frame = wr.SSL_WrapperPacket().FromString(data)
        print(frame)
        for robot in frame.detection.robots_blue:
            if robot.robot_id == self.robot_id:
                return robot.x / 1000, robot.y / 1000, robot.orientation
        return None, None, None
    
    def speed_control(self, U, omega):
        vd = (2 * U + omega * 7.5) / (2 * 3.2)
        vl = (2 * U - omega * 7.5) / (2 * 3.2)

        # if omega >= 1 or omega <= -1:
        #     Max_Speed = 0.01
        #     Min_Speed = -0.01
        # else:
        #     Max_Speed = self.v_max
        #     Min_Speed = self.v_min

        vd = max(min(vd, self.v_max), self.v_min)
        vl = max(min(vl, self.v_max), self.v_min)

        return int(vl), int(vd)
    
    def send_speed(self, speed_left, speed_right):
        direction_left = 1 if speed_left > 0 else -1
        direction_right = 1 if speed_right > 0 else -1 
        combined_value = (abs(speed_left) << 24) | (abs(speed_right) << 16) | (direction_left << 8) | direction_right
        # print(f"enviando: {combined_value}")
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(combined_value.to_bytes(4, byteorder='little'))
                print("data sent")
                return
        except Exception as e:
            print(f"Erro ao enviar dados: {e}")
    
    def follow_path(self, pathX, pathY, End_position):
        #--------------------------------------------
        deltaT = 0.033
        #---------------------------------------------
        a = 1
        interror_phi = Integral_part_phi = fant_phi = 0
        
        while a == 1:
            x, y, phi_obs = self.get_position()
            print(f"x: {x}, y: {y}, phi: {phi_obs}")
            if x is None or y is None:
                # print("bah gurizada")
                continue
            
            phid = math.atan2((pathY - y), (pathX - x))
            error_phi = phid - phi_obs
            omega, fant_phi, interror_phi, Integral_part_phi = self.pid_controller(self.kp, self.ki, self.kd, deltaT, error_phi, interror_phi, fant_phi, Integral_part_phi)
            error_distance = math.sqrt((pathY - y)**2 + (pathX - x)**2)
            error_distance_global = math.sqrt((End_position[1] - y) ** 2 + (End_position[0] - x) ** 2)
            
            U = self.v_linear
            print(f"Erro Phi: {error_phi}, omega: {omega}")            
            vl, vd = self.speed_control(U, omega)
            print(f"velocidades: L: {vl}, R: {vd}")
            self.send_speed(vl, vd)
            
            if error_distance <= 0.02 or error_distance_global <= 0.03:
                self.send_speed(0, 0)
                a = 0
    
    def pid_controller(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
        Integral_saturation = 10
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

if __name__ == "__main__":
    VISION_IP = "224.5.23.2"
    VISION_PORT = 10015
    ROBOT_IP = "192.168.0.107"  # IP do robô
    ROBOT_PORT = 80  # Porta do robô
    ROBOT_ID = 4  # ID do robô azul a ser controlado

    Kp = 150
    Ki = 75
    Kd = 10

    vision_sock = init_vision_socket(VISION_IP, VISION_PORT)
    crb01 = Corobeu(ROBOT_IP, ROBOT_PORT, ROBOT_ID, vision_sock, Kp, Ki, Kd)
    crb01.follow_path(0.45, 0.45, [0.45, 0.45])