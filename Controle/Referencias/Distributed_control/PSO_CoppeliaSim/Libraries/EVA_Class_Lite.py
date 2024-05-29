""""
EVA_Class_Lite.py

This code present the conexion between the python program and the EVA robot used within CoppeliaSim software. This code
have a class called EVA_Robot, and within class have five functions. the first function connect_EVA make the conexion
with the EVA robot within the coppeliasim simultor; the second function called PID_Controller_phi is the PID controller
used in the objective go to goal controller; the third function called Speed_EVA calculate the right speed wheel and 
the left speed wheel based on the cinematic cofiguration EVA robot; Finally the EVA function is the principal EVA 
controller that receives the position data and send the speed data for the simulator.

Author:
    Mario Andres Pastrana Triana (mario.pastrana@ieee.org)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)

Version:
    0.0.1 (beta)

Release Date:
    MAY 20, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)

"""""

import math
import Libraries.Bioinspired_Algorithm as PSO_Algorithm
import Libraries.sim as sim
import numpy as np
import math as mat

class EVA_robot():

    """"
    This code have a class called EVA_Robot, and within class have five functions. the first function connect_EVA make 
    the conexion with the EVA robot within the coppeliasim simultor; the second function called PID_Controller_phi is 
    the PID controller used in the objective go to goal controller; the third function called Speed_EVA calculate the 
    right speed wheel and the left speed wheel based on the cinematic cofiguration EVA robot; Finally the EVA function
     is the principal EVA controller that receives the position data and send the speed data for the simulator.
    """""

    def __init__(self):

        """
            Initializations global variables
            self.phi        = Global phi angle
            self.v          = Linear speed 
        """""

        self.phi = 0
        self.v = 1

    def connect_EVA(self, EVA):
        """""
        Function used to communicate with CoppeliaSim
        
            Arguments :
            
                EVA (Integer) = Name the robot to use

            Return : 
            
                clientID (Integer)   = ID of communication
                Motori   (Integer)   = Handle left motor
                Motord   (Integer)   = Handle right motor
                cuerpo   (Integer)   = Handle robot
        """""

        ### Select the EVA robot ###

        if EVA == 0:

            Left_Motor = 'motori_EVA_0'
            Rigth_Motor = 'motord_EVA_0'
            Chasis = 'EVA_0'
            port = 19998

        elif EVA == 1:

            Left_Motor = 'motori_EVA_0#0'
            Rigth_Motor = 'motord_EVA_0#0'
            Chasis = 'EVA_0#0'
            port = 19999

        elif EVA == 2:

            Left_Motor = 'motori_EVA_0#1'
            Rigth_Motor = 'motord_EVA_0#1'
            Chasis = 'EVA_0#1'
            port = 19996

        elif EVA == 3:

            Left_Motor = 'motori_EVA_0#2'
            Rigth_Motor = 'motord_EVA_0#2'
            Chasis = 'EVA_0#2'
            port = 19995

        ### Make the conexion with the coppeliaSim simulator ###

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        # if clientID == 0:
        #    print("Connect to", port)
        #else:
        #    print("Can not Connect to")


        ### Get the EVA robot object and the motors ###

        returnCode, Motori = sim.simxGetObjectHandle(clientID, Left_Motor,
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, Motord = sim.simxGetObjectHandle(clientID, Rigth_Motor,
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, cuerpo = sim.simxGetObjectHandle(clientID, Chasis,
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)

        ### Send the handlers and the ID client ###

        return clientID, Motori, Motord, cuerpo

    def PID_Controller_phi(self, error, interror, fant, Integral_part):
        """""
        PID_Controller_phi fucntion is the PID controller used in the objective go to goal controller

            Arguments :

                error           (Float) = Error of orientation
                interror        (Float) = Integral error
                fant            (Float) = value F previous used in the PID
                Integral_part   (Float) = Integral part

            Return : 

                PID             (Float) = PID value
                f               (Float) = current f value
                interror        (Float) = Integral error
                Integral_part   (Float) = Integral part
        """""

        ### Init PID constants ###

        kp = 2
        ki = 0.5
        kd = 3.5
        deltaT = 0.05
        Integral_saturation = 10

        ### Get the filter PID value used on the derivative part ###

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error

        ### Calculate the derivative error ###

        if fant == 0:
            deerror = (f / deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### PID equation ###

        PID = kp * error + Integral_part + deerror * kd

        ### Send the PID values ###

        return PID, f, interror, Integral_part

    def Speed_EVA(self, U, omega, error_distance_relative, error_distance_global):
        """""
        PID_Controller_phi fucntion is the PID controller used in the objective go to goal controller

            Arguments :

                U                       (Float) = Linear speed
                omega                   (Float) = Angular speed
                error_distance_relative (Float) = Distance error between the robot and the next point
                error_distance_global   (Float) = Distance error between the robot and the end point

            Return : 

                vl  = Left Value
                vd  = Rigth Value
                a   = value to end simulation with coppeliaSim
        """""

        ### Calculate the right and left speed ###

        vd = (2 * U + omega * 12) / 2 * 4.2
        vl = (2 * U - omega * 12) / 2 * 4.2
        a = 1

        ### Saturation the right speed ###

        if vd >= 3:
            vd = 3
        elif vd <= 0:
            vd = 0

        ### Saturation the Left speed ###

        if vl >= 3:
            vl = 3
        elif vl <= 0:
            vl = 0

        ### Arrive the next point posisition ###

        if error_distance_relative <= 0.1:
            a = 0
        else:
            pass

        ### Arrive the end point posisition ###

        if error_distance_global <= 0.1:
            a = 0
            vl = 0
            vd = 0
        else:
            pass

        ### Return the right and left speed ###

        return vl, vd, a

    def EVA(self, x, End_position, EVA_Number):
        """""
        PID_Controller_phi fucntion is the PID controller used in the objective go to goal controller

            Arguments :

                x               (Float) = The next point (X and Y axis)
                End_position    (Float) = The end point (X and Y axis)
                EVA_Number      (Float) = Number of the EVA robot

            Return : 

                none
        """""

        ### Get the handler and the EVA ID ###

        clientID_EVA_0, Motori_EVA_0, Motord_EVA_0, cuerpo_EVA_0 = self.connect_EVA(EVA_Number)

        ### Init configurations variables ###

        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        deltaT = 0.05

        ### Begin the communication with coppeliaSim simulator ###

        if (sim.simxGetConnectionId(clientID_EVA_0) != -1):

            ### While condition of simulation is active ###

            while (a == 1):

                #### Get the position robot ####

                s, positiona = sim.simxGetObjectPosition(clientID_EVA_0, cuerpo_EVA_0, -1, sim.simx_opmode_streaming)

                #### Phi Goal calculate ####

                phid = math.atan2(x[1] - positiona[1], x[0] - positiona[0])

                #### PID controller Phi ####

                error_phi = phid - self.phi
                omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(error_phi, interror_phi,
                                                                                           fant_phi, Integral_part_phi)

                #### Error distance next point and the global is calculate ###

                error_distance = math.sqrt((x[1] - positiona[1]) ** 2 + (x[0] - positiona[0]) ** 2)
                error_distance_global = math.sqrt(
                    (End_position[1] - positiona[1]) ** 2 + (End_position[0] - positiona[0]) ** 2)

                #### speed profiles ####

                if Number_Iterations < 20:
                    U = (self.v / 20) * Number_Iterations
                elif Number_Iterations >= 20 and error_distance_global >= 1:
                    U = self.v
                elif error_distance_global < 1:
                    U = error_distance_global + 0.5

                ### calculate new phi robot integrating the angular speed ###

                self.phi = self.phi + omega * deltaT

                #### Calculate the right and left speed values ###

                vl, vd, a = self.Speed_EVA(U, omega, error_distance, error_distance_global)

                #### Send the speed values ####

                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motord_EVA_0, vd, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID_EVA_0, Motori_EVA_0, vl, sim.simx_opmode_blocking)

                ### Save values to plot ###

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)


if __name__ == "__main__":

    ctf = EVA_robot()
    Bioinspired_object_path = PSO_Algorithm.PSO()
    EVA = 0

    if EVA == 0:
        x_init = 1
        y_init = 1
        x_end = 1
        y_end = 2.5

    elif EVA == 1:
        x_init = 2
        y_init = 1
        x_end = 2
        y_end = 2

    elif EVA == 2:
        x_init = 3
        y_init = 1
        x_end = 2
        y_end = 3

    elif EVA == 3:
        x_init = 4
        y_init = 1
        x_end = 3
        y_end = 2.5

    delta_x = 0
    delta_y = 0.5
    """""
    for Number_delta in range(3):

        xideal, yideal = ctf.Plot_plath_planing(x_init, y_init, x_end, y_end)

        for path in range(len(xideal)):
            x = [xideal[path], yideal[path]]
            End_position = [xideal[-1], yideal[-1]]
            ctf.EVA(x, End_position, EVA)
    """""
    x = [1, 1]
    End_position = [1, 1]
    ctf.EVA(x, End_position, EVA)
    x_init = x_end
    y_init = y_end
    x_end = x_init + delta_x
    y_end = y_init + delta_y


