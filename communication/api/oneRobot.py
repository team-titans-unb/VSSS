import math
import sim
import matplotlib.pyplot as plt
import numpy as np
import math as mat
import csv
import socket
import time

class Corobeu():

    def __init__(self):
        self.Time_sample_print = []
        self.SP = []
        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.phi_obs = 0
        self.v = 0.75
        self.posError = []
        # self.positions = []

    def connect_CRB(self, port):
        """""
        Function used to communicate with CoppeliaSim
            arg :
                - Port used to CoppeliaSim (same CoppeliaSim)
                
            out : 
                - Communication CoppeliaSim
        """""
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("no se pudo conectar")

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', 
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'motorL01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'motorR01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        return clientID, robot, MotorE, MotorD
    
    def Speed_CRB(self, U, omega, error_distance):
        vd = (2 * U + omega * 7.5) / 2 * 1.6
        vl = (2 * U - omega * 7.5) / 2 * 1.6
        a = 1
        if vd >= 3:
            vd = 3
        if vd <= 0:
            vd = 0

        if vl >= 3:
            vl = 3
        if vl <= 0:
            vl = 0

        if error_distance <= 0.005:
            a = 0
            vl = 0
            vd = 0
        return vl, vd, a
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
        Integral_saturation = 10
        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        PID = kp * error + Integral_part + deerror * kd
        #print("----PID-----")
        # print(PID)
        return PID, f, interror, Integral_part

    def Robot_CRB(self, x, kpi, kii, kdi, deltaT):
        (clientID, robot, motorE, motorD) = self.connect_CRB(19999)
        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0
        # alfa = 10
        # offset_speed = 0.5

        if (sim.simxGetConnectionId(clientID) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################  
                while cont0 < 5:
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    cont0 = cont0 + 1
                # print(positiona)
                if positiona == [0, 0, 0]:
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                else:
                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    # print(f'positions = {positiona}')
                    phid = math.atan2(x[1] - positiona[1], x[0] - positiona[0])

                    error_phi = phid - self.phi

                    if error_phi <= 5:
                        a = 0

                    error_distance = math.sqrt((x[1] - positiona[1])**2 + (x[0] - positiona[0])**2)
                    self.posError.append(error_distance)
                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                            error_phi, interror_phi,
                                                                                            fant_phi, Integral_part_phi)

                    if Number_Iterations >= 20:
                        # # k = self.v*(1-math.exp(-alfa*(abs(error_distance))**2))/(abs(error_distance))
                        # U = k*abs(error_distance)*error_distance + offset_speed
                       # U = self.v
                       a = 0
                   # else:
                        #U = (self.v/20) * Number_Iterations
                    U = self.v
                    self.phi = self.phi + omega * deltaT
                    # print(error_phi)
                    vl, vd, a = self.Speed_CRB(U, omega, error_distance)

                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)
                    print("------")
                    print(vl)
                    print(vd)
                    Number_Iterations = Number_Iterations + 1
                    Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out.append(positiona[1])
        self.x_out.append(positiona[0])


def send_and_receive_esp32(x, y, z, esp32_ip, esp32_port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((esp32_ip, esp32_port))
        data = f"{x},{y},{z}\n"
        s.sendall(data.encode())
        
        # Receive modified data from ESP32
        data = s.recv(1024).decode()
        x_mod, y_mod, z = map(float, data.strip().split(','))
        return x_mod, y_mod, z

if __name__ == "__main__":

    esp32_ip = '192.168.106.238'  # Substitua pelo IP real da ESP32
    esp32_port = 80  

    crb01 = Corobeu()
    kpi = 1.3
    kii = 0.001
    kdi = 5
    deltaT = 0.05
    
    xideal = []
    yideal = []
    x_new  = []
    y_new = []
   
    
    positions = './posicoesAdquiridas4.csv'
    with open(positions, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=';')
        for row in reader:
            for value__str in row:
                values = value__str.strip('[]').split(',')
                xideal.append(float(values[0]))
                yideal.append(float(values[1]))
                z = float(values[2])
                # Imprime na tela os 2 dados modificados
                # (f"Before values: x_mod={float(values[0])}, y_mod={float(values[1])}, z={z}")
                #x_mod = crb01.x_out[]
                #y_mod = crb01.y_out[]
                #x_mod, y_mod, z = send_and_receive_esp32(float(values[0]), float(values[1]), float(z), esp32_ip, esp32_port)
                #x_new.append(x_mod)
                #y_new.append(y_mod)
                #print(f"Modified values: x_mod={x_mod}, y_mod={y_mod}, z={z}")


    plt.scatter(xideal, yideal, color='blue', marker='x')
    plt.grid(True)
    plt.xlim(0, 1.7)
    plt.ylim(0, 1.3)
    plt.show()
        

    for path in range(len(xideal)):
        pos = [xideal[path], yideal[path]]
        #crb01.Robot_CRB(pos, kpi, kii, kdi, deltaT)
        #if(input("Continuar: [y]/[n]?: ") == "y" or "Y"):
        
        print("Enviando posição inicial")
        crb01.Robot_CRB(pos, kpi, kii, kdi, deltaT) # inicial
        x, y, z = send_and_receive_esp32(crb01.x_out[0], crb01.y_out[0], 0, esp32_ip, esp32_port)
        print("Enviando nova posição para o Coppellia")
        print(f"Valores anteriores: x - > {xideal[path]} e y -> {yideal[path]}")
        print(f"Valores modificados: x: {x} e y: {y}")
        time.sleep(10)
        crb01.Robot_CRB([x, y], kpi, kii, kdi, deltaT)
        print("Programa terminado")
        #break


    #posInicial = [xideal[0], yideal[0]]
    #crb01.Robot_CRB(posInicial, kpi,kii,kdi,deltaT)
    #enviar para esp crb01.x_out e crb.yout
    #recebe da esp  posesp = xnovo ynovo
    #crb01.Robot_CRB(posesp, kpi, kii, kdi, deltaT)
    
    #x0 = [y_new[-1], y_new[-1]]   #pegar x e y da esp e por aqui
    #crb01.Robot_CRB(x0, kpi, kii, kdi, deltaT)

    plt.scatter(crb01.x_out, crb01.y_out, color='red', marker='o')
    plt.grid(True)
    plt.xlim(0, 1.7)
    plt.ylim(0, 1.3)
    plt.show()