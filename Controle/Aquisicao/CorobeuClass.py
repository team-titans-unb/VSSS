import math
import sim
import matplotlib.pyplot as plt
import numpy as np
import math as mat
import AlgorithmAStart

class Corobeu():

    def __init__(self, port, name, motor_E, motor_D):
        self.clientID, self.robot, self.motorE, self.motorD = self.connect_CRB(port, name, motor_E, motor_D)
        self.Time_sample_print = []
        self.SP = []
        self.y_out = 1
        self.x_out = 1
        self.phi = 0
        self.phi_obs = 0
        self.v = 5

    def connect_CRB(self, port, name, motor_E, motor_D):
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

        returnCode, robot = sim.simxGetObjectHandle(clientID, name, 
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, motor_E,
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, motor_D,
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

        if error_distance <= 0.1:
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
            Integral_part = kii * interror * deltaT

        PID = kpi * error + Integral_part + deerror * kdi
        # print(PID)
        return PID, f, interror, Integral_part

    def Robot_CRB(self, x, kpi, kii, kdi, deltaT):
        #clientID, robot, motorE, motorD = self.connect_CRB(19999, robot, motor_e, motor_d)
        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        alfa = 10
        offset_speed = 0.5


        if (sim.simxGetConnectionId(self.clientID) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################
                s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)

                phid = math.atan2(x[1] - positiona[1], x[0] - positiona[0])

                error_phi = phid - self.phi

                error_distance = math.sqrt((x[1] - positiona[1])**2 + (x[0] - positiona[0])**2)
                omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                           error_phi, interror_phi,
                                                                                           fant_phi, Integral_part_phi)

                if Number_Iterations >= 20:

                    k = self.v*(1-math.exp(-alfa*(abs(error_distance))**2))/(abs(error_distance))
                    U = k*abs(error_distance)*error_distance + offset_speed

                else:
                    U = (self.v/20) * Number_Iterations

                # print(error_phi)
                vl, vd, a = self.Speed_CRB(U, omega, error_distance)

                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vd, sim.simx_opmode_blocking)
                print("------")
                print(vl)
                print(vd)
                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out = positiona[1]
        self.x_out = positiona[0]

if __name__ == "__main__":

    crb01 = Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    # crb02 = Corobeu(19999, 'robot02#0', 'motorL02#0', 'motorR02#0')
    # crb03 = Corobeu(19999, 'robot03#1', 'motorL03#1', 'motorR03#1')
    kpi = 0.001
    kii = 0.0
    kdi = 0.0
    deltaT = 0.05
    # yideal = [2]
    # xideal = [2]
    # square_value_x = [[2, 3], [5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    # square_value_y = [[2, 3], [0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    square_value_x = []#, [0, 5.5]]
    square_value_y = []#, [0, 0.5]]
    x_init = [0.35, 0.102, 0.30]#, 4]
    y_init = [0.65, 1.106, 1.525]#, 1]
    #### formacion rombo ####
    # x_end = [3, 2, 3, 4]
    # y_end = [4, 3, 2, 3]
    ###### formacion triangulo ####
    x_end = [0.028, 1, 0.5]#, 4]
    y_end = [1.107, 1, 0.5]#, 3]
    ###### formacion fila ####
    # x_end = [3, 3, 3, 3]
    # y_end = [4, 3, 2, 4.5]

    # for i in range(len(x_end)):
    pp0 = AlgorithmAStart.Trajectory_Generation()
    # pp1 = AlgorithmAStart.Trajectory_Generation()
    # pp2 = AlgorithmAStart.Trajectory_Generation()

    x0 = x_init[0]
    y0 = y_init[0]
    xend0 = x_end[0]
    yend0= y_end[0]

    x1 = x_init[1]
    y1 = y_init[1]
    xend1 = x_end[1]
    yend1 = y_end[1]

    x2 = x_init[2]
    y2 = y_init[2]
    xend2 = x_end[2]
    yend2 = y_end[2]

    pp0.AStar(x0, y0, xend0, yend0, square_value_x, square_value_y)
    xideal0 = pp0.xtrajectory_send
    yideal0 = pp0.ytrajectory_send

    # pp1.AStar(x1, y1, xend1, yend1, square_value_x, square_value_y)
    # xideal1 = pp1.xtrajectory_send
    # yideal1 = pp1.ytrajectory_send

    # pp2.AStar(x2, y2, xend2, yend2, square_value_x, square_value_y)
    # xideal2 = pp2.xtrajectory_send
    # yideal2 = pp2.ytrajectory_send

    for path in range(len(xideal0)):
        x0 = [xideal0[path], yideal0[path]]
        # x1 = [xideal1[path], yideal1[path]]
        # x2 = [xideal2[path], yideal2[path]]
        # if i == 0:
        crb01.Robot_CRB(x0, kpi, kii, kdi, deltaT)
        # # elif i == 1:
        # crb02.Robot_CRB(x1, kpi, kii, kdi, deltaT)
        # # else:
        # crb03.Robot_CRB(x2, kpi, kii, kdi, deltaT)

