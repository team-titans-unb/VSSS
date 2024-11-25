import math
import sim
import numpy as np
import ANNClass as ann
import pandas as pd
import Bioinspired as bia
import matplotlib.pyplot as plt
from DrawField import plot_robot_path
from followBall import Corobeu


class LFDmetodology:
    def __init__(self, port, destiny=None, nSamples=50, filename='wheelSpeeds'):
        if destiny is None:
            destiny = [0, 0]
        self.phi = []
        self.posError = []
        self.lv = []
        self.rv = []
        self.positions = []
        self.phid = []
        self.phiRobot = []
        self.errorPhi = []
        self.ballPosition = []
        (self.clientID, self.robot, self.motorE, self.motorD, self.ball) = self.connect_CRB(port)
        self.v_max = 20
        self.v_min = -20
        self.v_linear = 16
        self.acqBehavior = []
        self.acqPosit = []
        self.acqAngle = []
        self.acqInputs = []
        self.acqErrorPhi = []
        self.acqDistance = []
        self.destiny = destiny
        self.objX = []
        self.objY = []
        self.nSamples = nSamples
        self.filename = filename
        self.ys = []
        self.fitVector = []
        self.imtSpeed = []
        self.imtError = []
        self.imtPosit = []

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

        if error_distance <= 0.07:
            a = 0
            vl = 0
            vd = 0

        ### Return values ###

        return vl, vd, a

    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
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
            deerror = (f / deltaT)
        else:
            deerror = (float((f - fant) / deltaT))

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

        ### Get the objects within coppeliaSim using the connect_CRB function ###

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

        if sim.simxGetConnectionId(self.clientID) != -1:

            ### Criterio to simulation ###

            while a == 1:

                ### important to get valid values and init the phi value ###

                if Number_Iterations <= 1:

                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - math.pi
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
                else:

                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    s, ballPos = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_streaming)
                    # s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # s, angle_ball = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    # self.phi = angle_robot[2] - math.pi

                    # print(f'Angle robot ==> {angle_robot}')

                    ### Calculate the phid (see georgia tech course) ###

                    phid = math.atan2(ballPos[1] - positiona[1], ballPos[0] - positiona[0])
                    ### Phi error to send the PID controller

                    error_phi = phid - self.phi  # plotar isso
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

                    vl, vd, a = self.Speed_CRB(self.v_linear, omega, error_distance, Number_Iterations)
                    self.lv.append(vl)
                    self.rv.append(vd)
                    self.positions.append([positiona[0], positiona[1]])
                    self.phid.append(phid)
                    self.phiRobot.append(self.phi)
                    self.errorPhi.append(error_phi)
                    self.ballPosition.append(ballPos)
                    ### Send the speed values to coppeliasim simulato ###

                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vd, sim.simx_opmode_blocking)

                    ### update the time simulation and the simulation iteration
                Number_Iterations = Number_Iterations + 1
                print(Number_Iterations)
                Time_Sample.append(Number_Iterations * deltaT)
                # time.sleep(0.5)
        ### Save the robot position ###
        # self.save_to_dataframe('robot_data.csv')
        # self.y_out.append(positiona[1])
        # self.x_out.append(positiona[0])

    def dataAcquisition(self):
        kpi = 20
        kii = 1
        kdi = 0.001
        deltaT = 0.05  # 50 ms
        self.Robot_CRB(kpi, kii, kdi, deltaT)
        # self.crb01.save_to_dataframe('Training_Referee.csv')

        for i in range(len(self.lv)):
            sr = self.rv[i]
            sl = self.lv[i]
            if 0.9*sr > sl:
                self.acqBehavior.append(2)
            elif 0.9*sl > sr:
                self.acqBehavior.append(3)
            else:
                self.acqBehavior.append(1)
        self.nSamples = len(self.lv)
        print("Fim da Aquisição")

    def training(self):
        a = 0

        while a == 0:
            print("Treinando")
            trainObj = bia.Bioinspired_algorithms()
            self.fitVector, self.ys = trainObj.PSO(self.nSamples, self.errorPhi, self.acqBehavior, self.filename)

            print("Validação")
            ubehavior = []
            val = ann.Referee(self.nSamples)
            # errorL = 0
            # errorR = 0
            for i in range(self.nSamples):
                input_vector = self.errorPhi[i]
                weights = self.ys[:5]  # First 5 elements are weights (for 3 layers)
                biases = self.ys[5:]  # Remaining elements are biases

                ubehavior.append(val.topology1(input_vector, weights, biases))
                # errorL = (speedL[i] - self.acqSpeed[0][i]) ** 2 + errorL

            # print(f'error L : {errorL / self.nSamples}')
            # print(f'error R : {errorR / self.nSamples}')
            # # plt.figure()
            # # plt.plot(((speedL - self.acqSpeed[0]) ** 2), label='Erro de treinamento da roda direita')
            # # plt.plot(((speedR - self.acqSpeed[1]) ** 2), label='Erro de treinamento da roda esquerda')
            # vd = [vel for vel in speedL]
            # ve = [vel for vel in speedR]
            # plt.figure()
            # # plt.plot(self.acqSpeed[0], label='Velocidades roda direita')
            # # plt.plot(self.acqSpeed[1], label='Velocidades roda esquerda')
            # # plt.plot(vd, label='Velocidades calculadas da roda direita')
            # # plt.plot(ve, label='Velocidades calculadas da roda esquerda')
            # # plt.legend()
            # # plt.xlabel("Iteração")
            # # plt.ylabel("Velocidade da Roda")
            # # plt.title("Comparação entre as Velocidades Medidas e Calculadas")
            # plt.plot(self.acqSpeed[0], label='Right wheel speed')
            # plt.plot(self.acqSpeed[1], label='Left wheel speed')
            # plt.plot(vd, label='Calculated right wheel speed')
            # plt.plot(ve, label='Calculated left wheel speed')
            # plt.legend()
            # plt.xlabel("Iteration")
            # plt.ylabel("Wheel speed")
            # plt.title("Comparison between Acquired and Calculated Speeds")
            # name = self.filename + '_Speeds.pdf'
            # plt.savefig(name)
            # plt.show()
            resposta = input("Ficou bom? (s/n)").strip().lower()
            if resposta == 's':
                a = 1

    def imitation(self):
        # clientID = self.connect(19995)
        spd = ann.ArtificialNeuralNetwork(self.nSamples)

        weightsUBehavior1 = [0.39894832971087163, -2.105905809136049, 0.31736796205179807, -2.105179399172271]
        weightsUBehavior2 = [-0.024973293083546694, -2.870969075689935, 0.04775503893057847, -2.3766928990983884]
        weightsUBehavior3 = [0.06438700509254595, -2.4500444769455267, 0.03583502267106433, -2.69537854438019]

        ref = ann.Referee(self.nSamples, weightsUBehavior1, weightsUBehavior2, weightsUBehavior3)

        a = 1
        i = 0
        positiona = []
        ballpos = []
        omega = []
        phid = []
        Vd = []
        Ve = []

        if sim.simxGetConnectionId(self.clientID) != -1:
            print("Iniciando Imitação")

            sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_oneshot)

            # s, positiona[0] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
            # s, ballpos[0] = sim.simxGetObjectPosition(clientID, bola, -1, sim.simx_opmode_streaming)
            while a == 1:
                print(i)
                positiona.append([])
                ballpos.append([])
                # phid.append(0)

                s, positiona[i] = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_buffer)
                s, ballpos[i] = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_buffer)
                while positiona[i] == [0, 0, 0] or ballpos[i] == [0, 0, 0]:
                    s, positiona[i] = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    s, ballpos[i] = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_buffer)
                print(f'Ball Pos = {ballpos}')
                y_atual = positiona[i][1]
                x_atual = positiona[i][0]

                s, angle = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_buffer)
                omega.append(angle[2] - 1.5708)
                phi_atual = omega[i]
                phid = math.atan2(ballpos[i][1] - y_atual, ballpos[i][0] - x_atual)
                error_phi = phid - phi_atual

                error_distance = math.sqrt(((ballpos[i][1] - y_atual) ** 2) + ((ballpos[i][0] - x_atual) ** 2))

                if error_distance <= 0.03:
                    a = 0

                self.imtError.append(error_distance)

                # REFEREE
                weights = self.ys[:5]  # First 5 elements are weights (for 3 layers)
                biases = self.ys[5:]  # Remaining elements are biases

                out_ref = ref.topology1(error_phi, weights, biases)
                print(out_ref)
                slp_weights, ub = ref.select_behavior(out_ref)

                weightsL = slp_weights[0]
                biasL = slp_weights[1]
                weightsR = slp_weights[2]
                biasR = slp_weights[3]

                speedL = spd.neuron(error_phi, weightsL, biasL)
                speedR = spd.neuron(error_phi, weightsR, biasR)

                Vd.append(speedL / 0.032)
                Ve.append(speedR / 0.032)
                # Vd.append(speedL)
                # Ve.append(speedR)

                # if Vd[i] > 2:
                #     Vd[i] = 2
                # if Ve[i] > 2:
                #     Ve[i] = 2

                # print([Vd[i], Ve[i]])
                print(f'Micro Behaviour - {ub+1}')
                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, Ve[i], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0.95 * Vd[i], sim.simx_opmode_blocking)
                i = i + 1
                if i > 5 * self.nSamples:
                    a = 0
            imtX = [coord[0] for coord in positiona]
            imtY = [coord[1] for coord in positiona]
            balX = [coord[0] for coord in ballpos]
            balY = [coord[1] for coord in ballpos]
            self.imtPosit = [imtX, imtY]
            sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_oneshot)
            plot_robot_path(balX, balY, imtX, imtY, self.filename)
            # plt.figure()
            # plt.plot(phid)
            # plt.show()

    def calculate_errors(self):
        acqX, acqY = self.acqPosit
        imtX, imtY = self.imtPosit

        num_points = len(imtX)
        original_indices = np.arange(0, len(acqX))
        target_indices = np.linspace(0, len(acqX) - 1, num_points)

        interpolated_acqX = np.interp(target_indices, original_indices, acqX)
        interpolated_acqY = np.interp(target_indices, original_indices, acqY)

        squared_diff_x = [(ix - ax) ** 2 for ix, ax in zip(imtX, interpolated_acqX)]
        squared_diff_y = [(iy - ay) ** 2 for iy, ay in zip(imtY, interpolated_acqY)]

        mse = np.mean(squared_diff_x + squared_diff_y)
        mae = np.mean([math.sqrt(sx + sy) for sx, sy in zip(squared_diff_x, squared_diff_y)])
        max_error = np.max([math.sqrt(sx + sy) for sx, sy in zip(squared_diff_x, squared_diff_y)])

        planned_path_length = np.sum(
            [math.sqrt((imtX[i + 1] - imtX[i]) ** 2 + (imtY[i + 1] - imtY[i]) ** 2) for i in range(num_points - 1)])
        executed_path_length = np.sum([math.sqrt((interpolated_acqX[i + 1] - interpolated_acqX[i]) ** 2 + (
                    interpolated_acqY[i + 1] - interpolated_acqY[i]) ** 2) for i in range(num_points - 1)])
        path_length_deviation = abs(planned_path_length - executed_path_length) / planned_path_length
        name = self.filename + '.txt'
        with open(name, 'w') as file:
            file.write(f'W_B = [{self.ys[0]}, {self.ys[1]}, {self.ys[2]}, {self.ys[3]}]\n\n')
            file.write(f'Erro Absoluto Medio (MAE): {mae} m\n')
            file.write(f'Erro Quadratico Medio (MSE): {mse} m²\n')
            file.write(f'Erro Maximo: {max_error} m\n')
            file.write(f'Desvio Medio ao Longo do Caminho: {path_length_deviation * 100:.2f}%\n')

        print(f'Erro Absoluto Médio (MAE): {mae} m')
        print(f'Erro Quadrático Médio (MSE): {mse} m²')
        print(f'Erro Máximo: {max_error} m')
        print(f'Desvio Médio ao Longo do Caminho: {path_length_deviation * 100:.2f}%')

        plot_robot_path(interpolated_acqX, interpolated_acqY, imtX, imtY, self.filename)

    def stopBot(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_oneshot)


if __name__ == "__main__":
    finalPos = [-0.5, 0]
    cen = 'T06'
    filename = f'{cen}_LfD_{finalPos[0]}_{finalPos[1]}'

    crb = LFDmetodology(port=19995, destiny=finalPos, nSamples=50, filename=filename)
    try:
        crb.dataAcquisition()
        print(crb.acqBehavior)
        #
        # spdD = crb.acqSpeed[0]
        # spdE = crb.acqSpeed[1]
        # plt.figure()
        # # plt.plot(spdD, label='Velocidades roda direita')
        # # plt.plot(spdE, label='Velocidades roda esquerda')
        # plt.plot(spdD, label='Right wheel speed')
        # plt.plot(spdE, label='Left wheel speed')
        # plt.legend()
        # plt.show()
        #
        # simulate = True
        # while simulate:
        #
        crb.training()
        #
        crb.imitation()
        #
        #     crb.calculate_errors()
        #
        #     resposta = input("Ficou bom? (s/n)").strip().lower()
        #     if resposta == 's':
        #         simulate = False
        #
        # crb.calculate_errors()
        #
        # print(f'W_B = [{crb.ys[0]}, {crb.ys[1]}, {crb.ys[2]}, {crb.ys[3]}]')  # , {crb.ys[4]}, {crb.ys[5]}]')

    except Exception as e:
        crb.stopBot()