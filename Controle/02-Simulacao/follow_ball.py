""""
follow_ball.py

This class present the PID controller implementation on the mobile robot follower ball, this class present five functions
the first function called init, make the initialization of the variables, the connect_CRB present the connection with
the coppeliaSim, Speed_CRB calculated the wheels speeds based on the robot topology (in this case is a differential robot),
PID_Controller_phi implement the PID based on the phi error (see video of the georgia tech course in this link:
https://www.youtube.com/watch?v=Lgy92yXiyqQ&list=PLp8ijpvp8iCvFDYdcXqqYU5Ibl_aOqwjr&index=14) finally, Robot_CRB is the 
principal function in this class.


Author:

    Mario Andres  Pastrana Triana (mario.pastrana@ieee.org)
    Matheus de Sousa Luiz (luiz.matheus@aluno.unb.br)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)
    

Release Date:
    JUN 19, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's get started =)
"""""


import math
import sim
import numpy as np
import math as mat
import time
import subprocess
from float2bin import float2bin
from bin2float import bin2float

class Corobeu():
    """"
    Corobeu class is the principal class to controller the EVA positions robots.
        __init___               = Function to inicializate the global variables
        connect_CRB             = Present the connection with the coppeliaSim
        Speed_CRB               = Calculated the wheels speeds based on the robot topology (in this case is a differential robot)
        PID_Controller_phi      = Implement the PID based on the phi error
        Robot_CRB               = The principal function in this class.
    """""

    def __init__(self):

        """"
            Initializations global variables
            self.y_out    (float list) = Out position robot in the y axis
            self.x_out    (Float list) = Out position robot in the x axis
            self.phi      (Float)   = phi robot value
            self.v_max    (Integer) = max speed for the robot
            self.v_min    (Integer) = min speed for the robot
            self.posError (Float list) = phi error
            self.sel_position (Integer) = 0 goleiro, 1 atacante
        """""

        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max = 15
        self.v_min = -15
        self.Exponente = 5  # Bits para el exponente en la conversión binaria
        self.Mantiza = 5  # Bits para la mantisa en la conversión binaria
        self.sumEM = self.Exponente + self.Mantiza  # Suma de bits para exponente y mantisa
        self.v_linear = 6
        self.posError = []
        self.ideal_goleiro_x = -0.6
        self.sel_position = 1

    def connect_CRB(self, port):
        """""
        Function used to communicate with CoppeliaSim
            argument :
                Port (Integer) = used to CoppeliaSim (same CoppeliaSim)
                
            outputs : 
                clientID (Integer)  = Client number
                robot    (Integer)  = objecto robot
                MotorE   (Integer)  = Object motor left
                MotorD   (Integer)  = Object motor right
                ball     (Integer)  = Object ball on the scene
        """""

        ### Connect to coppeliaSim ###

        sim.simxFinish(-1)
        
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)
            

        ### Return the objects ###

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'robot01', 
                                                    sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'motorL01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'motorR01',
                                                     sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)
               
        return clientID, robot, MotorE, MotorD, ball
    
    def Speed_CRB(self, U, omega, error_distance, signed):

        """""
        Function used to calculate the speed for each wheel based on the topology robot
            argument :
                U              (Integer) = Max linear speed
                omega          (Float)   = Angular speed
                error_distance (Float)   = Error between the robot and the final point

            outputs : 
                vl (Float)   = Left speed
                vd (Float)   = right speed
                a  (Integer) = condition to simulate
        """""
        L = 6.5 # Distance between the wheels
        R = 1.6  # Wheel radio

        ### Calculate the speed based on the topology robot ###

        vd = ((2 * (U * signed) + omega * L) / (2 * R))
        vl = ((2 * (U * signed) - omega * L) / (2 * R))
        a = 1
        print(f'vl === {vl} vd == {vd}')
        print(f' omega == {omega}')
        ### the first time #####
        
        Max_Speed = self.v_max
        Min_Speed = self.v_min

        # Max_Speed = self.v_max
        # Min_Speed = self.v_min

        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        ### When arrive to the goal ###

        if error_distance <= 0.1:
            a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl, vd, a
    
    def PID_Hardware(self, kp, ki, kd, ideal, sensor_value, deltaT, unomenosalfaana, alfaana, Number_Iterations):
        """
        Implementa un controlador PID en hardware.
        
        Args:
            kp (float): Ganancia proporcional.
            ki (float): Ganancia integral.
            kd (float): Ganancia derivativa.
            ideal (float): Valor deseado (Set Point).
            sensor_value (float): Valor actual del sensor.
            deltaT (float): Tiempo de muestreo.
            unomenosalfaana (float): Parámetro del filtro.
            alfaana (float): Parámetro del filtro.
            Number_Iterations (int): Número de iteraciones.
        
        Returns:
            float: Valor de salida del PID.
        """
        uno_sobre_delta_t = 1 / deltaT  # Calcula el inverso del tiempo de muestreo

        # Conversión de valores a binario
        kp_bin = float2bin(self.Exponente, self.Mantiza, kp)
        ki_bin = float2bin(self.Exponente, self.Mantiza, ki)
        kd_bin = float2bin(self.Exponente, self.Mantiza, kd)
        ideal_bin = float2bin(self.Exponente, self.Mantiza, ideal)
        sensor_value_bin = float2bin(self.Exponente, self.Mantiza, sensor_value)
        deltaT_bin = float2bin(self.Exponente, self.Mantiza, deltaT)
        unomenosalfaana_bin = float2bin(self.Exponente, self.Mantiza, unomenosalfaana)
        alfaana_bin = float2bin(self.Exponente, self.Mantiza, alfaana)
        uno_sobre_delta_t_bin = float2bin(self.Exponente, self.Mantiza, uno_sobre_delta_t)

        # Conversión de binario a enteros
        kp_int = int(kp_bin, 2)
        ki_int = int(ki_bin, 2)
        kd_int = int(kd_bin, 2)
        ideal_int = int(ideal_bin, 2)
        sensor_value_int = int(sensor_value_bin, 2)
        deltaT_int = int(deltaT_bin, 2)
        unomenosalfaana_int = int(unomenosalfaana_bin, 2)
        alfaana_int = int(alfaana_bin, 2)
        uno_sobre_delta_t_int = int(uno_sobre_delta_t_bin, 2)

        # Reset del sistema si es la primera iteración
        if Number_Iterations == 2:
            reseti = 1
            app(f'x_write_reg 10 {reseti}')
            reseti = 0
            app(f'x_write_reg 10 {reseti}')

        # Envío de valores al hardware
        app(f'x_write_reg 4 {kp_int}')
        app(f'x_write_reg 5 {ki_int}')
        app(f'x_write_reg 6 {kd_int}')
        app(f'x_write_reg 1 {ideal_int}')
        app(f'x_write_reg 2 {sensor_value_int}')
        app(f'x_write_reg 3 {deltaT_int}')
        app(f'x_write_reg 7 {unomenosalfaana_int}')
        app(f'x_write_reg 8 {alfaana_int}')
        app(f'x_write_reg 9 {uno_sobre_delta_t_int}')

        # Inicio del proceso PID
        starti = 1
        app(f'x_write_reg 0 {starti}')
        outputb = app(f'x_read_reg 0')
        readyb = app(f'x_read_reg 1')
        starti = 0
        app(f'x_write_reg 0 {starti}')

        # Conversión de la salida binaria a float
        outputbin = (bin(outputb.data[1])[2:])
        while len(outputbin) <= self.sumEM:
            outputbin = '0' + outputbin

        output_PID = bin2float(self.Exponente, self.Mantiza, outputbin)
        # print(f'Valor del PID: {output_PID}')
        return output_PID

    def Robot_CRB(self, x, deltaT):

        """""
        Principal function to simulate the follower ball robot
            argument :
                kpi              (Float) = Proportional constant used on the PID controller
                kii              (Float) = Integral constant used on the PID controller
                kdi              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time

            outputs : 
               none
        """""
        kpi = x[0]
        kii = x[1]
        kdi = x[2]
        ### Get the objects within coppeliaSim using the connect_CRB function ###

        (clientID, robot, motorE, motorD, ball) = self.connect_CRB(19999)

        ### Criterio to simulation ###
        # Constantes del PID
        raizes = np.roots([kdi, kpi, kii])
        acumulate_error = 0
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        omega_ant = 0
        a = 1
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0

        ### Init the principal values ###

        Number_Iterations = 0
        Time_Sample = []

        ### Make the communication with coppeliaSim ###

        if (sim.simxGetConnectionId(clientID) != -1):

            ### Criterio to simulation ###

            while (a == 1):

                ### important to get valid values and init the phi value ###

                if Number_Iterations <= 1:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                else:

                    s, positiona = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(clientID, ball, -1, sim.simx_opmode_streaming)
                    # s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # s, angle_ball = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - 1.47
                    # Obtener la orientación del robot relativa al sistema de coordenadas global
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    
                    phi_robot = orientation[2]
                    self.phi = phi_robot
                    if returnCode == sim.simx_return_ok:
                        print(f"Orientación del robot: {phi_robot}")
                    else:
                        print("Error al obtener la orientación del robot")

                    # print(f'Angle robot ==> {angle_robot}')

                    ### Calculate the phid (see georgia tech course) ###
                    # Se é atacante, signed é 1, se é goleiro, signed pode mudar
                    if self.sel_position == 1:
                        phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])
                        signed = 1
                    elif self.sel_position == 0:
                        phid = math.atan2(ballPos[1] - positiona[1], (self.ideal_goleiro_x) - positiona[0])
                        if ballPos[1] > positiona[1]:
                            signed = 1
                        else:
                            signed = -1
                    ### Phi error to send the PID controller
                    phid = phid + 1.5708 # sum 90º
                    if phid > 3.15:
                        phid = phid - 6.2832
                        
                    # Calcula la diferencia entre el ángulo actual y el anterior
                    diferencia_phid = phid - angulo_anterior_phid
                    diferencia_phi  = self.phi - angulo_anterior_phi_robot
                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phid > math.pi:
                        phid -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phid < -math.pi:
                        phid += 2 * math.pi
                    
                    # Si la diferencia es mayor que π, ajusta restando 2π
                    if diferencia_phi > math.pi:
                        self.phi -= 2 * math.pi
                    # Si la diferencia es menor que -π, ajusta sumando 2π
                    elif diferencia_phi < -math.pi:
                        self.phi += 2 * math.pi
                    
                    # Actualiza el ángulo anterior
                    angulo_anterior_phid = phid
                    angulo_anterior_phi_robot = self.phi
                    
    

                    print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid-self.phi}' )

                    acumulate_error = acumulate_error + abs(phid-self.phi)
                    omega = self.PID_Hardware(kpi, kii, kdi, phid, self.phi, deltaT, unomenosalfaana, alfaana, Number_Iterations)

                    ### Calculate the distance error, the robot stop when arrive the ball ###
                    if self.sel_position == 1:
                        error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + (ballPos[0] - positiona[0]) ** 2)
                    elif self.sel_position == 0:
                        error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + ((self.ideal_goleiro_x) - positiona[0]) ** 2)1
                Time_Sample.append(Number_Iterations * deltaT)
                # time.sleep(0.5)
        ### Save the robot position ###ç
        # Detener la simulación
        self.y_out.append(positiona[1])
        self.x_out.append(positiona[0])

if __name__ == "__main__":

    crb01 = Corobeu()
    # kpi = 0.3
    # kii = 0.005
    # kdi = 0.0005
    ##### PSO constant
    kpi = 0.2386
    kii = 0.0860
    kdi = 0.0112
    x = [kpi, kii, kdi]
    deltaT = 0.05
    crb01.Robot_CRB(x, deltaT)