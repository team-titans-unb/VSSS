"""
ConnectionCP.py

This code connect from Python3 to Coppeliasim

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
    EVA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    Jan 23, 2023
"""

import math
import sim
import matplotlib.pyplot as plt
import numpy as np
import math as mat
import AlgorithmAStart

class EVA_robot():

    def __init__(self):

        self.Infrared_Sensor = []
        self.Time_sample_print = []
        self.SP = []
        self.y_out = 1
        self.x_out = 1
        self.phi = 0
        self.phi_obs = 0
        self.v = 5

    def connect_EVA_0(self, port):
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

            # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
            returnCode, Motori = sim.simxGetObjectHandle(clientID, 'motori_EVA_0',
                                                         sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
            returnCode, Motord = sim.simxGetObjectHandle(clientID, 'motord_EVA_0',
                                                         sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
            returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'EVA_0',
                                                         sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
            # %%% Obtendos os objetos dos sensores do rob� %%%
            returnCode, sensord1 = sim.simxGetObjectHandle(clientID, 'sensor1_EVA_0', sim.simx_opmode_oneshot_wait)
            returnCode, sensord2 = sim.simxGetObjectHandle(clientID, 'sensor2_EVA_0', sim.simx_opmode_oneshot_wait)
            returnCode, sensord3 = sim.simxGetObjectHandle(clientID, 'sensor3_EVA_0', sim.simx_opmode_oneshot_wait)
            returnCode, sensord4 = sim.simxGetObjectHandle(clientID, 'sensor4_EVA_0', sim.simx_opmode_oneshot_wait)

            return clientID, Motori, Motord, cuerpo, sensord1, sensord2, sensord3, sensord4

    def connect_EVA_1(self, port):
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

        # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
        returnCode, Motori = sim.simxGetObjectHandle(clientID, 'motori_EVA_0#0',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, Motord = sim.simxGetObjectHandle(clientID, 'motord_EVA_0#0',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'EVA_0#0',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        # %%% Obtendos os objetos dos sensores do rob� %%%
        returnCode, sensord1 = sim.simxGetObjectHandle(clientID, 'sensor1_EVA_0#0', sim.simx_opmode_oneshot_wait)
        returnCode, sensord2 = sim.simxGetObjectHandle(clientID, 'sensor2_EVA_0#0', sim.simx_opmode_oneshot_wait)
        returnCode, sensord3 = sim.simxGetObjectHandle(clientID, 'sensor3_EVA_0#0', sim.simx_opmode_oneshot_wait)
        returnCode, sensord4 = sim.simxGetObjectHandle(clientID, 'sensor4_EVA_0#0', sim.simx_opmode_oneshot_wait)

        return clientID, Motori, Motord, cuerpo, sensord1, sensord2, sensord3, sensord4

    def connect_EVA_2(self, port):
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

        # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
        returnCode, Motori = sim.simxGetObjectHandle(clientID, 'motori_EVA_0#1',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, Motord = sim.simxGetObjectHandle(clientID, 'motord_EVA_0#1',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'EVA_0#1',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        # %%% Obtendos os objetos dos sensores do rob� %%%
        returnCode, sensord1 = sim.simxGetObjectHandle(clientID, 'sensor1_EVA_0#1', sim.simx_opmode_oneshot_wait)
        returnCode, sensord2 = sim.simxGetObjectHandle(clientID, 'sensor2_EVA_0#1', sim.simx_opmode_oneshot_wait)
        returnCode, sensord3 = sim.simxGetObjectHandle(clientID, 'sensor3_EVA_0#1', sim.simx_opmode_oneshot_wait)
        returnCode, sensord4 = sim.simxGetObjectHandle(clientID, 'sensor4_EVA_0#1', sim.simx_opmode_oneshot_wait)

        return clientID, Motori, Motord, cuerpo, sensord1, sensord2, sensord3, sensord4

    def connect_EVA_3(self, port):
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

        # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
        returnCode, Motori = sim.simxGetObjectHandle(clientID, 'motori_EVA_0#2',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, Motord = sim.simxGetObjectHandle(clientID, 'motord_EVA_0#2',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'EVA_0#2',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        # %%% Obtendos os objetos dos sensores do rob� %%%
        returnCode, sensord1 = sim.simxGetObjectHandle(clientID, 'sensor1_EVA_0#2', sim.simx_opmode_oneshot_wait)
        returnCode, sensord2 = sim.simxGetObjectHandle(clientID, 'sensor2_EVA_0#2', sim.simx_opmode_oneshot_wait)
        returnCode, sensord3 = sim.simxGetObjectHandle(clientID, 'sensor3_EVA_0#2', sim.simx_opmode_oneshot_wait)
        returnCode, sensord4 = sim.simxGetObjectHandle(clientID, 'sensor4_EVA_0#2', sim.simx_opmode_oneshot_wait)

        return clientID, Motori, Motord, cuerpo, sensord1, sensord2, sensord3, sensord4

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
        return PID, f, interror, Integral_part

    def Object_EVA(self, clientID, cuerpo, sensord1, sensord2, sensord3, sensord4, Critical_Value):
        s, positiona = sim.simxGetObjectPosition(clientID, cuerpo, -1, sim.simx_opmode_streaming)
        # %%% Obtendo os valores de distancia de cada objeto %%%

        rtd1, sd1, sdd1, _, _ = sim.simxReadProximitySensor(clientID, sensord1, sim.simx_opmode_oneshot_wait)
        rtd2, sd2, sdd2, _, _ = sim.simxReadProximitySensor(clientID, sensord2, sim.simx_opmode_oneshot_wait)
        rtd3, sd3, sdd3, _, _ = sim.simxReadProximitySensor(clientID, sensord3, sim.simx_opmode_oneshot_wait)
        rtd4, sd4, sdd4, _, _ = sim.simxReadProximitySensor(clientID, sensord4, sim.simx_opmode_oneshot_wait)

        if sdd1[2] <= 0.01:
            value_sensor_1 = 0.9
        else:
            value_sensor_1 = sdd1[2]

        if sdd2[2] <= 0.01:
            value_sensor_2 = 0.9
        else:
            value_sensor_2 = sdd2[2]

        if sdd3[2] <= 0.01:
            value_sensor_3 = 0.9
        else:
            value_sensor_3 = sdd3[2]

        if sdd4[2] <= 0.01:
            value_sensor_4 = 0.9
        else:
            value_sensor_4 = sdd4[2]

        if value_sensor_1 <= Critical_Value:

            x_object_1 = 0.07 + value_sensor_1 * math.sin(30) + positiona[0]
            y_object_1 = 0.07 + value_sensor_1 * math.cos(30) + positiona[1]
            phi_object_1 = math.atan2(y_object_1 - positiona[1], x_object_1 - positiona[0])
            state_sensor_1 = 1
        else:
            phi_object_1 = 0
            state_sensor_1 = 0

        if value_sensor_2 <= Critical_Value:

            x_object_2 = 0.07 + value_sensor_2 * math.sin(60) + positiona[0]
            y_object_2 = 0.07 + value_sensor_2 * math.cos(60) + positiona[1]
            phi_object_2 = math.atan2(y_object_2 - positiona[1], x_object_2 - positiona[0])
            state_sensor_2 = 1
        else:
            phi_object_2 = 0
            state_sensor_2 = 0

        if value_sensor_3 <= Critical_Value:

            x_object_3 = 0.07 + value_sensor_3 * math.sin(-60) + positiona[0]
            y_object_3 = 0.07 + value_sensor_3 * math.cos(-60) + positiona[1]
            phi_object_3 = math.atan2(y_object_3 - positiona[1], x_object_3 - positiona[0])
            state_sensor_3 = 1
        else:
            phi_object_3 = 0
            state_sensor_3 = 0

        if value_sensor_4 <= Critical_Value:

            x_object_4 = 0.07 + value_sensor_4 * math.sin(-30) + positiona[0]
            y_object_4 = 0.07 + value_sensor_4 * math.cos(-30) + positiona[1]
            phi_object_4 = math.atan2(y_object_4 - positiona[1], x_object_4 - positiona[0])
            state_sensor_4 = 1
        else:
            phi_object_4 = 0
            state_sensor_4 = 0
        return positiona, state_sensor_1, state_sensor_2, state_sensor_3, state_sensor_4, phi_object_1, phi_object_2, phi_object_3, phi_object_4

    def Speed_EVA(self, U, omega, error_distance):

        vd = (2 * U + omega * 12) / 2 * 4.2
        vl = (2 * U - omega * 12) / 2 * 4.2
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

    def EVA_0(self, x, kpi, kii, kdi, deltaT):


        (clientID_EVA_0, Motori_EVA_0, Motord_EVA_0, cuerpo_EVA_0, sensord1_EVA_0, sensord2_EVA_0, sensord3_EVA_0,
         sensord4_EVA_0) = self.connect_EVA_0(19998)
        a = 1
        Critical_Value = 0.3
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        alfa = 10
        offset_speed = 0.5


        if (sim.simxGetConnectionId(clientID_EVA_0) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################
                (positiona, state_sensor_1, state_sensor_2, state_sensor_3, state_sensor_4, phi_object_1, phi_object_2,
                 phi_object_3, phi_object_4) = self.Object_EVA(clientID_EVA_0, cuerpo_EVA_0, sensord1_EVA_0,
                                                               sensord2_EVA_0, sensord3_EVA_0, sensord4_EVA_0,
                                                               Critical_Value)
                state = state_sensor_1 + state_sensor_2 + state_sensor_3 + state_sensor_4

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

                if state == 0:
                    self.phi = self.phi + omega*deltaT
                elif state >= 1:
                    self.phi = (phi_object_1 + phi_object_2 + phi_object_3 + phi_object_4)/4 + math.pi/2
                    #self.phi = 0

                print(error_phi)
                vl, vd, a = self.Speed_EVA(U, omega, error_distance)
                ####################################################################3

                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motord_EVA_0, vd, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motori_EVA_0, vl, sim.simx_opmode_blocking)
                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out = positiona[1]
        self.x_out = positiona[0]

    def EVA_1(self, x, kpi, kii, kdi, deltaT):


        (clientID_EVA_0, Motori_EVA_0, Motord_EVA_0, cuerpo_EVA_0, sensord1_EVA_0, sensord2_EVA_0, sensord3_EVA_0,
         sensord4_EVA_0) = self.connect_EVA_1(19999)
        a = 1
        Critical_Value = 0.3
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        alfa = 10
        offset_speed = 0.5


        if (sim.simxGetConnectionId(clientID_EVA_0) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################
                (positiona, state_sensor_1, state_sensor_2, state_sensor_3, state_sensor_4, phi_object_1, phi_object_2,
                 phi_object_3, phi_object_4) = self.Object_EVA(clientID_EVA_0, cuerpo_EVA_0, sensord1_EVA_0,
                                                               sensord2_EVA_0, sensord3_EVA_0, sensord4_EVA_0,
                                                               Critical_Value)
                state = state_sensor_1 + state_sensor_2 + state_sensor_3 + state_sensor_4

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

                if state == 0:
                    self.phi = self.phi + omega*deltaT
                elif state >= 1:
                    self.phi = (phi_object_1 + phi_object_2 + phi_object_3 + phi_object_4)/4 + math.pi/2
                    #self.phi = 0

                print(error_distance)
                vl, vd, a = self.Speed_EVA(U, omega, error_distance)
                ####################################################################3

                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motord_EVA_0, vd, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motori_EVA_0, vl, sim.simx_opmode_blocking)
                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out = positiona[1]
        self.x_out = positiona[0]


    def EVA_2(self, x, kpi, kii, kdi, deltaT):


        (clientID_EVA_0, Motori_EVA_0, Motord_EVA_0, cuerpo_EVA_0, sensord1_EVA_0, sensord2_EVA_0, sensord3_EVA_0,
         sensord4_EVA_0) = self.connect_EVA_2(19996)
        a = 1
        Critical_Value = 0.3
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        alfa = 10
        offset_speed = 0.5


        if (sim.simxGetConnectionId(clientID_EVA_0) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################
                (positiona, state_sensor_1, state_sensor_2, state_sensor_3, state_sensor_4, phi_object_1, phi_object_2,
                 phi_object_3, phi_object_4) = self.Object_EVA(clientID_EVA_0, cuerpo_EVA_0, sensord1_EVA_0,
                                                               sensord2_EVA_0, sensord3_EVA_0, sensord4_EVA_0,
                                                               Critical_Value)
                state = state_sensor_1 + state_sensor_2 + state_sensor_3 + state_sensor_4

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

                if state == 0:
                    self.phi = self.phi + omega*deltaT
                elif state >= 1:
                    self.phi = (phi_object_1 + phi_object_2 + phi_object_3 + phi_object_4)/4 + math.pi/2
                    #self.phi = 0

                print(error_distance)
                vl, vd, a = self.Speed_EVA(U, omega, error_distance)
                ####################################################################3

                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motord_EVA_0, vd, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motori_EVA_0, vl, sim.simx_opmode_blocking)
                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out = positiona[1]
        self.x_out = positiona[0]

    def EVA_3(self, x, kpi, kii, kdi, deltaT):


        (clientID_EVA_0, Motori_EVA_0, Motord_EVA_0, cuerpo_EVA_0, sensord1_EVA_0, sensord2_EVA_0, sensord3_EVA_0,
         sensord4_EVA_0) = self.connect_EVA_3(19995)
        a = 1
        Critical_Value = 0.3
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        alfa = 10
        offset_speed = 0.5


        if (sim.simxGetConnectionId(clientID_EVA_0) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################
                (positiona, state_sensor_1, state_sensor_2, state_sensor_3, state_sensor_4, phi_object_1, phi_object_2,
                 phi_object_3, phi_object_4) = self.Object_EVA(clientID_EVA_0, cuerpo_EVA_0, sensord1_EVA_0,
                                                               sensord2_EVA_0, sensord3_EVA_0, sensord4_EVA_0,
                                                               Critical_Value)
                state = state_sensor_1 + state_sensor_2 + state_sensor_3 + state_sensor_4

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

                if state == 0:
                    self.phi = self.phi + omega*deltaT
                elif state >= 1:
                    self.phi = (phi_object_1 + phi_object_2 + phi_object_3 + phi_object_4)/4 + math.pi/2
                    #self.phi = 0

                print(error_distance)
                vl, vd, a = self.Speed_EVA(U, omega, error_distance)
                ####################################################################3

                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motord_EVA_0, vd, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motori_EVA_0, vl, sim.simx_opmode_blocking)
                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                # print(Number_Iterations)

        self.y_out = positiona[1]
        self.x_out = positiona[0]

if __name__ == "__main__":

    ctf = EVA_robot()
    kpi = 20
    kii = 1
    kdi = 0.072
    deltaT = 0.05
    # yideal = [2]
    # xideal = [2]
    # square_value_x = [[2, 3], [5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    # square_value_y = [[2, 3], [0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    square_value_x = [[5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    square_value_y = [[0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    x_init = [1, 2, 3, 4]
    y_init = [1, 1, 1, 1]
    #### formacion rombo ####
    # x_end = [3, 2, 3, 4]
    # y_end = [4, 3, 2, 3]
    ###### formacion triangulo ####
    x_end = [3, 2, 3, 4]
    y_end = [4, 3, 3, 3]
    ###### formacion fila ####
    # x_end = [3, 3, 3, 3]
    # y_end = [4, 3, 2, 4.5]

    for i in range(len(x_end)):
        pp = AlgorithmAStart.Trajectory_Generation()
        x0 = x_init[i]
        y0 = y_init[i]
        xend = x_end[i]
        yend = y_end[i]

        pp.AStar(x0, y0, xend, yend, square_value_x, square_value_y)
        xideal = pp.xtrajectory_send
        yideal = pp.ytrajectory_send

        for path in range(len(xideal)):
            x = [xideal[path], yideal[path]]
            if i == 0:
                ctf.EVA_0(x, kpi, kii, kdi, deltaT)
            elif i == 1:
                ctf.EVA_1(x, kpi, kii, kdi, deltaT)
            elif i == 2:
                ctf.EVA_2(x, kpi, kii, kdi, deltaT)
            else:
                ctf.EVA_3(x, kpi, kii, kdi, deltaT)

