import math
import sim
import numpy as np
import math as mat

class Corobeu():

    def __init__(self, port, name, motor_E, motor_D):
        self.clientID, self.robot, self.motorE, self.motorD = self.connect_CRB(port, name, motor_E, motor_D)
        self.Time_sample_print = []
        self.SP = []
        self.yOut = []
        self.xOut = []
        self.phi = 0
        self.phi_obs = 0
        self.v = 8
        self.instPosition = [0, 0]
        self.posError = []

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
    
    def Speed_CRB(self, U, omega, error_distance_relative, error_distance_global):
        vd = (2 * U + omega * 7.5) / 2 * 1.6
        vl = (2 * U - omega * 7.5) / 2 * 1.6
        a = 1
        if vd >= 8:
            vd = 8
        if vd <= -8:
            vd = -8

        if vl >= 8:
            vl = 8
        if vl <= -8:
            vl = -8

        if error_distance_relative <= 0.05:
            a = 0
        else:
            pass

        ### Arrive the end point posisition ###

        if error_distance_global <= 0.02:
            a = 0
            vl = 0
            vd = 0
        else:
            pass

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
        return PID, f, interror, Integral_part

    def Follow_Path(self, pathX, pathY, End_position):
        #----------------Constantes
        kp = 2
        ki = 0.05
        kd = 7
        deltaT = 0.01
        #---------------------------
        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0

        if (sim.simxGetConnectionId(self.clientID) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################  
                while cont0 < 5:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    cont0 = cont0 + 1
                # print(positiona)
                if positiona == [0, 0, 0]:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                else:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    # print(f'positions = {positiona}')
                    phid = math.atan2((pathY - positiona[1]), (pathX - positiona[0]))

                    error_phi = phid - self.phi
                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kp, ki, kd, deltaT,
                                                                                            error_phi, interror_phi,
                                                                                            fant_phi, Integral_part_phi)

                    error_distance = math.sqrt((pathY - positiona[1])**2 + (pathX - positiona[0])**2)
                    self.posError.append(error_distance)
                    
                    #### Error distance next point and the global is calculate ###
                    
                    error_distance_global = math.sqrt(
                        (End_position[1] - positiona[1]) ** 2 + (End_position[0] - positiona[0]) ** 2)

                    #### speed profiles ####

                    U = self.v

                    ### calculate new phi robot integrating the angular speed ###

                    self.phi = self.phi + omega * deltaT

                    #### Calculate the right and left speed values ###
                    # #### Check if the angular error is within the threshold ###
                    # if abs(error_phi) > 0.3:
                    #     # Rotate in place to correct orientation
                    #     vl = -omega if omega < 0 else omega  # Rotate in place
                    #     vd = omega if omega < 0 else -omega  # Rotate in place
                    # else:
                    #     # Move towards the waypoint
                    #     vl, vd, a = self.Speed_CRB(U, omega, error_distance, error_distance_global)

                    vl, vd, a = self.Speed_CRB(U, omega, error_distance, error_distance_global)

                    #### Send the speed values ####

                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vd, sim.simx_opmode_blocking)
                    print("------")
                    print(vl)
                    print(vd)
                    ### Save values to plot ###

                    Number_Iterations = Number_Iterations + 1
                    Time_Sample.append(Number_Iterations * deltaT)

                    self.yOut.append(positiona[1])
                    self.xOut.append(positiona[0])

                # print(Number_Iterations)

        # self.yOut.append(positiona[1])
        # self.xOut.append(positiona[0])

    def Get_Position(self):
        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        while positiona == [0, 0, 0]:
            s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
        self.instPosition[0] = positiona[0]
        self.instPosition[1] = positiona[1] 
        return positiona[0], positiona[1]
        

        # if (error_phi > 0.1):
                    #     U = 0
                    # elif (error_phi < -0.1):
                    #     U = 0 
                    # else:
                    #     U = self.v