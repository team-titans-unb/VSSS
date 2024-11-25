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
    Let's go started =)
"""""


import math
import sslclient
# import sim
import numpy as np
# import time
import socket
import time
import random

# wifi: Titans
# pass: Titans2024

PORT = 80  

class Corobeu():
    """"
    Corobeu class is the principal class to controller the EVA positions robots.
        __init___               = Function to inicializate the global variables
        connect_CRB             = Present the connection with the coppeliaSim
        Speed_CRB               = Calculated the wheels speeds based on the robot topology (in this case is a differential robot)
        PID_Controller_phi      = Implement the PID based on the phi error
        Robot_CRB               = The principal function in this class.
    """""

    def __init__(self, IP):

        """"
            Initializations global variables
            self.y_out    (float list) = Out position robot in the y axis
            self.x_out    (Float list) = Out position robot in the x axis
            self.phi      (Float)   = phi robot value
            self.v_max    (Integer) = max speed for the robot
            self.v_min    (Integer) = min speed for the robot
            self.posError (Float list) = phi error
        """""
        self.vision = sslclient.client(ip='224.5.23.2', port=10006)
        self.vision.connect()

        self.IP = IP
        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max = 20
        self.v_min = -20
        self.v_linear = 16
        self.posError = []
        self.positions = []
        self.rv = []
        self.lv = []
        self.phid = []
        self.phiRobot = []
        self.errorPhi = []
        self.ballPosition = []
    
        # def save_to_dataframe(self, filename='robot_data.csv'):
        #     """ Saves the lv, rv, and positions lists into a DataFrame and stores it as a CSV file. """
        #     data = {
        #         'Left Wheel Speed': self.lv,
        #         'Right Wheel Speed': self.rv,
        #         'Position X': [pos[0] for pos in self.positions],
        #         'Position Y': [pos[1] for pos in self.positions],
        #         'Robot phi': self.phiRobot,
        #         'Error phi': self.errorPhi,
        #         'Ball Position X': [pos[0] for pos in self.ballPosition],
        #         'Ball Position Y': [pos[1] for pos in self.ballPosition],
        #         'Desired Phi': self.phid
        #     }
        #     df = pd.DataFrame(data)
        #     df.to_csv(filename, index=False)
        #     print("Data saved to 'robot_data.csv'")

    def Speed_CRB(self, U, omega, error_distance, Number_Iterations):

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

        ### Calculate the speed based on the topology robot ###

        vd = (2 * U + omega * 7.5) / (2 * 3.2)
        vl = (2 * U - omega * 7.5) / (2 * 3.2)
        a = 1
        # print(f'vl === {vl} vd == {vd}')
        # print(f' omega == {omega}')
        ### the first time #####

        if omega >= 1 or omega <= -1:
            Max_Speed = 0.01
            Min_Speed = -0.01
        else:
            Max_Speed = self.v_max
            Min_Speed = self.v_min

        vd_PWM = (255*vd)/20
        vl_PWM = (255*vl)/20
        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd_PWM = 255
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl_PWM = 255
        if vl <= Min_Speed:
            vl = Min_Speed

        ### When arrive to the goal ###
        
        if error_distance <= 0.07:
            a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl_PWM, vd_PWM, a
    
    def send_speed_and_direction(self, speed1, speed2, direction1, direction2):
        combined_value = (speed1 << 24) | (speed2 << 16) | (direction1 << 8) | direction2
        retry_delay = 2  # Tempo de espera (em segundos) entre tentativas

        while True:  # Loop infinito para tentativas indefinidas
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    print("Trying to connect to ESP32...")
                    s.connect((self.IP, PORT))
                    s.sendall(combined_value.to_bytes(4, byteorder='little'))  # Envia o valor como 4 bytes
                    print("Data sent successfully.")
                    return  # Sai da função após o envio bem-sucedido
            except ConnectionRefusedError:
                print(f"Connection refused, retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)  # Aguarda antes de tentar novamente
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        """""
        Function used to calculate the omega value (output PID) used on the speed function
            argument :
                kp              (Float) = Proportional constant used on the PID controller
                ki              (Float) = Integral constant used on the PID controller
                kd              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time
                error           (Float) = Phi error between the robot and the goal point
                interror        (Float) = Error Integral
                fant            (Float  = Used on the derivative part
                Integral_part   (Float) = Integral part

            outputs : 
               PID              (Float) = Omega value used on the speed function
               f                (Float) = f value use on the derivative part
               interror         (Float) = Error Integral
               Integral_part    (Float) = Integral part
        """""

        ### Max value of saturation in the integral part ###

        Integral_saturation = 10

        ### Find the filter e ####

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        # print(f'mayor == {mayor}')
        Filter_e = 1 / (mayor * 10)

        ### Calculate the derivative part ###

        unomenosalfaana = math.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### Calculate the omega value (PID output) ###

        PID = kp * error + Integral_part + deerror * kd

        ### Return the principal values ###
        # print(f'Integral_part == {Integral_part}')
        return PID, f, interror, Integral_part

    def Robot_CRB(self, kpi, kii, kdi, deltaT):

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

        ### Criterio to simulation ###

        a = 1

        ### Init the principal values ###

        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0
        positiona = []

        ### Make the communication with coppeliaSim ###

        ### Criterio to simulation ###

        while (a == 1):

            data = self.vision.receive()
            if data.HasField('detection'):

                detection = data.detection()
                ballPos = (detection.balls[0].x, detection.balls[0].y) if detection.balls else None
                positiona = (detection.robots_blue[0].x, detection.robots_blue[0].y)
                # print(f'Angle robot ==> {angle_robot}')

                ### Calculate the phid (see georgia tech course) ###
                
                phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])
                ### Phi error to send the PID controller

                error_phi = phid - self.phi # plotar isso
                # error_phi_degree = math.degrees(error_phi)
                # print(f'error degree == > {error_phi_degree}, error ==> {error_phi}' )
                omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                            error_phi, interror_phi,
                                                                                            fant_phi,
                                                                                            Integral_part_phi)

                ### Calculate the distance error, the robot stop when arrive the ball ###

                error_distance = math.sqrt((ballPos[1] - positiona[1]) ** 2 + (ballPos[0] - positiona[0]) ** 2)

                ### Acumulative distance error ###

                self.posError.append(error_distance)

                ### Implement the PID controller ###

                ### Update the phi robot value ###

                self.phi = self.phi + omega * deltaT

                ### Calculate the speed right and left based on the topology robot ###

                ve, vd, a = self.Speed_CRB(self.v_linear, omega, error_distance, Number_Iterations)
                self.lv.append(ve)
                self.rv.append(vd)
                self.positions.append([positiona[0], positiona[1]])
                self.phid.append(phid)
                self.phiRobot.append(self.phi)
                self.errorPhi.append(error_phi)
                self.ballPosition.append(ballPos)
                ### Send the speed values to coppeliasim simulato ###

                self.send_speed_and_direction(ve, vd, 1, 1)

                ### update the time simulation and the simulation iteration
            Number_Iterations = Number_Iterations + 1
            print(Number_Iterations)
            Time_Sample.append(Number_Iterations * deltaT)
            # time.sleep(0.5)
        ### Save the robot position ###
        # self.save_to_dataframe('robot_data.csv')
        self.y_out.append(positiona[1])
        self.x_out.append(positiona[0])

if __name__ == "__main__":

    crb01 = Corobeu() #Iniciar com o IP aqui
    kpi = 20
    kii = 1
    kdi = 0.001
    deltaT = 0.05 # 50 ms
    crb01.Robot_CRB(kpi, kii, kdi, deltaT)
    print(crb01.lv)
    print(crb01.rv)
    print(crb01.positions)