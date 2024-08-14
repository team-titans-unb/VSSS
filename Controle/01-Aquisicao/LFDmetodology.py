import math
import sim
import numpy as np
from ANNClass import ArtificialNeuralNetwork
import pandas as pd
import Bioinspired as bia
import matplotlib.pyplot as plt
from DrawField import plot_robot_path

class LFDmetodology():
    def __init__(self, destiny=[0, 0], nSamples = 50, filename='wheelSpeeds'):
        self.acqSpeed = []
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

    def connect(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("Não foi possível conectar")
        return clientID

    def dataAquisition(self):
        clientID = self.connect(19999)
        a = 1
        i = 0
        positions = []
        omega = []
        mdg = []
        meg = []

        conectionID = sim.simxGetConnectionId(clientID)
        if (sim.simxGetConnectionId(clientID) != -1):
            print("Iniciando a Aquisição")
            while (a == 1):
                print(i)
                if i == self.nSamples:
                    a = 2
                else:
                    a = 1
                _, motorE = sim.simxGetObjectHandle(clientID, 'motorL01', sim.simx_opmode_blocking)
                _, motorD = sim.simxGetObjectHandle(clientID, 'motorR01', sim.simx_opmode_blocking)
                _, corpo = sim.simxGetObjectHandle(clientID, 'robot01', sim.simx_opmode_blocking)

                positions.append([])
                _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                while positions[i] == [0, 0, 0]:
                    _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                    _, angles = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                    _, _, md = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_blocking)
                    _, _, me = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_blocking)
                _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                _, angles = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                _, me, _ = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_oneshot)
                _, md, _ = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_oneshot)

                omega.append(angles[2] - 1.5708)
                # print(omega[i])
                phid = math.atan2((self.destiny[1] - positions[i][1]), (self.destiny[0] - positions[i][0]))
                error_phi = phid - omega[i]
                self.acqErrorPhi.append(error_phi)

                mdg.append((md[1] ** 2 + md[0] ** 2) ** 0.5)
                meg.append((me[1] ** 2 + me[0] ** 2) ** 0.5)

                self.objX.append(self.destiny[0])
                self.objY.append(self.destiny[1])

                error_distance = math.sqrt((self.destiny[1] - positions[i][1]) ** 2 + (self.destiny[0] - positions[i][0]) ** 2)
                self.acqDistance.append(error_distance)
                # print(error_phi)
                # print(error_distance)
                i = i + 1
            self.nSamples = i
            Vd = mdg
            Ve = meg
            self.acqSpeed = [Vd, Ve]
            acqX = [coord[0] for coord in positions]
            acqY = [coord[1] for coord in positions]
            self.acqPosit = [acqX, acqY]
            self.acqAngle = omega
            
            # phid = [math.atan2(self.destiny[1] - y, self.destiny[0] - x) for x, y in zip(acqX, acqY)]
            # self.acqInputs = [omega, phid]
            # self.acqInputs = [self.acqErrorPhi, self.acqDistance]
            self.acqInputs = self.acqErrorPhi
            dataframe = [acqX, acqY, omega, self.objX, self.objY, self.acqErrorPhi, Vd, Ve]
            index = ['PosX', 'PosY', 'Omega', 'DesX', 'DesY', 'Error Phi', 'VelD', 'VelE']
            df = pd.DataFrame(dataframe, index=index)
            name = self.filename + "Acquired_data.csv"
            df.to_csv(name)

            print("Fim da Aquisição")

    def training(self):
        a = 0

        while a == 0:
            print("Treinando")
            trainObj = bia.Bioinspired_algorithms()
            self.fitVector, self.ys = trainObj.PSO(self.nSamples, self.acqInputs, self.acqSpeed, self.filename)

            print("Validação")
            speedL = []
            speedR = []
            val = ArtificialNeuralNetwork(self.nSamples)
            errorL = 0
            errorR = 0
            for i in range(self.nSamples):
                input_vector = []
                speedL.append([])
                speedR.append([])
                # for j in range(2):
                #     input_vector.append(self.acqInputs[j][i])
                input_vector = self.acqInputs[i]
                weightsL = self.ys[0]
                biasL = self.ys[1]
                weightsR = self.ys[2]
                biasR = self.ys[3]

                speedL[i] = val.neuron(input_vector, weightsL, biasL)
                errorL = (speedL[i] - self.acqSpeed[0][i]) ** 2 + errorL
                speedR[i] = val.neuron(input_vector, weightsR, biasR)
                errorR = (speedR[i] - self.acqSpeed[1][i]) ** 2 + errorR

            print(f'error L : {errorL / self.nSamples}')
            print(f'error R : {errorR / self.nSamples}')
            # plt.figure()
            # plt.plot(((speedL - self.acqSpeed[0]) ** 2), label='Erro de treinamento da roda direita')
            # plt.plot(((speedR - self.acqSpeed[1]) ** 2), label='Erro de treinamento da roda esquerda')
            vd = [vel for vel in speedL]
            ve = [vel for vel in speedR]
            plt.figure()
            plt.plot(self.acqSpeed[0], label='Velocidades roda direita')
            plt.plot(self.acqSpeed[1], label='Velocidades roda esquerda')
            plt.plot(vd, label='Velocidades calculadas da roda direita')
            plt.plot(ve, label='Velocidades calculadas da roda esquerda')
            plt.legend()
            plt.xlabel("Iteração")
            plt.ylabel("Velocidade da Roda")
            plt.title("Comparação entre as Velocidades Medidas e Calculadas")
            name = self.filename + '_Speeds.pdf'
            plt.savefig(name)
            plt.show()
            resposta = input("Ficou bom? (s/n)").strip().lower()
            if resposta == 's':
                a = 1

    def imitation(self):
        clientID = self.connect(19999)
        spd = ArtificialNeuralNetwork(self.nSamples)
        a = 1
        i = 0
        positiona = []
        omega = []
        phid = []
        Vd = []
        Ve = []
        pathX = self.destiny[0]
        pathY = self.destiny[1]

        if (sim.simxGetConnectionId(clientID) != -1):
            print("Iniciando Imitação")

            _, motorE = sim.simxGetObjectHandle(clientID, 'motorL01', sim.simx_opmode_blocking)
            _, motorD = sim.simxGetObjectHandle(clientID, 'motorR01', sim.simx_opmode_blocking)
            _, corpo = sim.simxGetObjectHandle(clientID, 'robot01', sim.simx_opmode_blocking)

            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

            while (a == 1):
                print(i)
                positiona.append([])
                # phid.append(0)

                s, positiona[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                while positiona[i] == [0, 0, 0]:
                    s, positiona[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)

                y_atual = positiona[i][1]
                x_atual = positiona[i][0]

                s, angle = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                omega.append(angle[2] - 1.5708)
                phi_atual = omega[i]
                phid = math.atan2(pathY - y_atual, pathX - x_atual)
                error_phi = phid - phi_atual

                error_distance = math.sqrt(((pathY - y_atual) ** 2) + ((pathX - x_atual) ** 2))

                if error_distance <= 0.03:
                    a = 0

                self.imtError.append(error_distance)

                weightsL = self.ys[0]
                biasL = self.ys[1]
                weightsR = self.ys[2]
                biasR = self.ys[3]

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

                print([Vd[i], Ve[i]])
                sim.simxSetJointTargetVelocity(clientID, motorE, Ve[i], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID, motorD, 0.95*Vd[i], sim.simx_opmode_blocking)
                i = i + 1
                if i > 5*self.nSamples:
                    a = 0
            imtX = [coord[0] for coord in positiona]
            imtY = [coord[1] for coord in positiona]
            self.imtPosit = [imtX, imtY]
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
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

        planned_path_length = np.sum([math.sqrt((imtX[i + 1] - imtX[i]) ** 2 + (imtY[i + 1] - imtY[i]) ** 2) for i in range(num_points - 1)])
        executed_path_length = np.sum([math.sqrt((interpolated_acqX[i + 1] - interpolated_acqX[i]) ** 2 + (interpolated_acqY[i + 1] - interpolated_acqY[i]) ** 2) for i in range(num_points - 1)])
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


if __name__ == "__main__":

    finalPos = [0.7, 0.3]
    cen = 'T01'
    filename = f'{cen}_LfD_{finalPos[0]}_{finalPos[1]}'

    crb = LFDmetodology(destiny=finalPos, nSamples=50, filename=filename)
    crb.dataAquisition()

    spdD = crb.acqSpeed[0]
    spdE = crb.acqSpeed[1]
    plt.figure()
    plt.plot(spdD, label='Velocidades roda direita')
    plt.plot(spdE, label='Velocidades roda esquerda')
    plt.legend()
    plt.show()

    simulate = True
    while(simulate):

        crb.training()

        crb.imitation()

        crb.calculate_errors()

        resposta = input("Ficou bom? (s/n)").strip().lower()
        if resposta == 's':
            simulate = False

    crb.calculate_errors()

    print(f'W_B = [{crb.ys[0]}, {crb.ys[1]}, {crb.ys[2]}, {crb.ys[3]}]')#, {crb.ys[4]}, {crb.ys[5]}]')