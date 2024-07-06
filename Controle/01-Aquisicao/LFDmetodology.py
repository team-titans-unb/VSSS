import math
import sim
import numpy as np
from ANNClass import *
import pandas as pd
import scipy
import Bioinspired as bia
import matplotlib.pyplot as plt
import os
from DrawField import plot_robot_path

class LFDmetodology():
    def __init__(self, destiny = [0,0]):
        self.acqSpeed = []
        self.acqPosit = []
        self.acqAngle = []
        self.acqInputs = []
        self.destiny = destiny
        self.objX = []
        self.objY = []
        self.nSamples = 20
        self.ys = []
        self.fitVector = []
        self.imtSpeed = []
        self.imtError = []
        self.imtPosit = []

    def connect(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("conectado a ", port)
        else:
            print("n foi possivel conectar")
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
        if(sim.simxGetConnectionId(clientID) != -1):
            print("Iniciando a Aquisicao")
            while(a == 1):
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
                while positions[i] == [0,0,0]:
                    _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                    _, angles = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                    _, _, md = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_blocking)
                    _, _, me = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_blocking)
                _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                _, angles = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                _, me, _ = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_oneshot)
                _, md, _ = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_oneshot)
                omega.append(angles[2])
                mdg.append((md[1]**2 + md[0]**2) ** 1/2)
                meg.append((me[1]**2 + me[0]**2) ** 1/2)
                # mdg.append(md[1])
                # meg.append(me[1])
                self.objX.append(self.destiny[0])
                self.objY.append(self.destiny[1])
                i = i+1
            
            # Vd = scipy.signal.medfilt(mdg, kernel_size=3)
            # Ve = scipy.signal.medfilt(meg, kernel_size=3)
            Vd = mdg
            Ve = meg
            self.acqSpeed = [Vd, Ve]
            acqX = [coord[0] for coord in positions]
            acqY = [coord[1] for coord in positions]
            self.acqPosit = [acqX, acqY]
            self.acqAngle = omega
            self.acqInputs = [acqX, acqY, omega, self.objX, self.objY]
            dataframe = [acqX, acqY, omega, self.objX, self.objY, Vd, Ve]
            index = ['PosX', 'PosY', 'Omega', 'DesX', 'DesY', 'VelD', 'VelE']
            df = pd.DataFrame(dataframe, index=index)
            df.to_csv("Acquired_data.csv")

            print("Fim da Aquisicao")

    def training(self):
        a = 0
        
        while a==0:
            print("Treinando")
            trainObj = bia.Bioinspired_algorithms()
            self.fitVector, self.ys = trainObj.PSO(self.nSamples, self.acqInputs, self.acqSpeed)
            
            print("Validacao")
            # test = []
            speedL = []
            speedR = []
            val = ArtificialNeuralNetwork(self.nSamples)
            errorL = 0
            errorR = 0
            for i in range(self.nSamples):
                input_vector = []
                # test.append([])
                speedL.append([])
                speedR.append([])
                for j in range(5):
                    input_vector.append(self.acqInputs[j][i])
                # test[i] = val.mlp432(input_vector, self.ys[:38], self.ys[38:])
                # errorL = (test[i][0] - self.acqSpeed[0][i])**2 + errorL
                # errorR = (test[i][1] - self.acqSpeed[1][i])**2 + errorR
                weightsL = self.ys[:5]
                biasL = self.ys[5]
                weightsR = self.ys[6:11]
                biasR = self.ys[11]

                speedL[i] = val.neuron(input_vector, weightsL, biasL)
                errorL = (speedL[i] - self.acqSpeed[0][i])**2 + errorL
                speedR[i] = val.neuron(input_vector, weightsR, biasR)
                errorR = (speedR[i] - self.acqSpeed[1][i])**2 + errorR

            print(errorL/self.nSamples)
            print(errorR/self.nSamples)
            plt.figure()
            plt.plot(self.acqSpeed[0], label='Velocidades roda direita')
            plt.plot(self.acqSpeed[1], label='Velocidades roda esquerda')
            # vd = [vel[0] for vel in test]
            # ve = [vel[1] for vel in test]
            vd = [vel for vel in speedL]
            ve = [vel for vel in speedR]
            plt.figure()
            plt.plot(self.acqSpeed[0], label='Velocidades roda direita')
            plt.plot(self.acqSpeed[1], label='Velocidades roda esquerda')
            plt.plot(vd, marker='o', label='Velocidades calculadas da roda direita')
            plt.plot(ve, marker='x', label='Velocidades calculadas da roda esquerda')
            plt.legend()
            plt.ylim(-0.001, 0.01)
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
        omega =[]
        Vd = []
        Ve = []
        pathX = self.destiny[0]
        pathY = self.destiny[1]
      
        if (sim.simxGetConnectionId(clientID) != -1):
            print("Iniciando Imitacao")
            
            _, motorE = sim.simxGetObjectHandle(clientID, 'motorL01', sim.simx_opmode_blocking)
            _, motorD = sim.simxGetObjectHandle(clientID, 'motorR01', sim.simx_opmode_blocking)
            _, corpo = sim.simxGetObjectHandle(clientID, 'robot01', sim.simx_opmode_blocking)

            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

            while (a == 1):
                print(i)
                positiona.append([])
                
                s, positiona[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                while positiona[i] == [0, 0, 0]:
                    s, positiona[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming) 
                
                y_atual = positiona[i][1]
                x_atual = positiona[i][0]
                
                s, angle = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                omega.append(angle[2])
                phi_atual = omega[i]

                error_distance = math.sqrt((pathY - y_atual)**2 + (pathX - x_atual)**2)

                if error_distance <= 0.05:
                    a = 0

                self.imtError.append(error_distance)
            
                # Speeds = spd.mlp432([x_atual, y_atual, phi_atual, pathX, pathY], self.ys[:38], self.ys[38:])
                # Vd.append(Speeds[0]/0.008)
                # Ve.append(Speeds[1]/0.008)

                weightsL = self.ys[:5]
                biasL = self.ys[5]
                weightsR = self.ys[6:11]
                biasR = self.ys[11]

                speedL = spd.neuron([x_atual, y_atual, phi_atual, pathX, pathY], weightsL, biasL)
                speedR = spd.neuron([x_atual, y_atual, phi_atual, pathX, pathY], weightsR, biasR)

                Vd.append(speedL/0.008)
                Ve.append(speedR/0.008)
                
                if Vd[i] > 0.6:
                    Vd[i] = 0.6
                else:
                    pass
                if Ve[i] > 0.6:
                    Ve[i] = 0.6
                else:
                    pass
                
                print([Vd[i], Ve[i]])
                sim.simxSetJointTargetVelocity(clientID, motorE, Ve[i], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID, motorD, Vd[i], sim.simx_opmode_blocking)
                i = i + 1
            imtX = [coord[0] for coord in positiona]
            imtY = [coord[1] for coord in positiona]
            self.imtPosit = [imtX, imtY]
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

    def calculate_errors(self):
        # Interpolate acqPosit to match imtPosit size
        acqX, acqY = self.acqPosit
        imtX, imtY = self.imtPosit

        num_points = len(imtX)
        original_indices = np.arange(0, len(acqX))
        target_indices = np.linspace(0, len(acqX) - 1, num_points)

        interpolated_acqX = np.interp(target_indices, original_indices, acqX)
        interpolated_acqY = np.interp(target_indices, original_indices, acqY)

        # Calculate errors
        squared_diff_x = [(ix - ax) ** 2 for ix, ax in zip(imtX, interpolated_acqX)]
        squared_diff_y = [(iy - ay) ** 2 for iy, ay in zip(imtY, interpolated_acqY)]
        
        mean_squared_diff = np.mean(squared_diff_x + squared_diff_y)
        mse = np.sqrt(mean_squared_diff)
        
        absolute_error = np.mean([math.sqrt(sx + sy) for sx, sy in zip(squared_diff_x, squared_diff_y)])

        print(f'Absolute Error: {absolute_error}')
        print(f'Root Mean Square Error (RMSE): {mse}')
        
        plot_robot_path(interpolated_acqX, interpolated_acqY, imtX, imtY)


if __name__ == "__main__":
    crb = LFDmetodology(destiny = [0.1, 0])
    crb.dataAquisition()

    spdD = crb.acqSpeed[0]
    spdE = crb.acqSpeed[1]
    plt.figure()
    plt.plot(spdD, label='Velocidades roda direita')
    plt.plot(spdE, label='Velocidades roda esquerda')
    plt.legend()
    # plt.ylim(-1, 1)
    plt.show()

    crb.training()

    crb.imitation()

    crb.calculate_errors()

    print(crb.ys)


# --------ys------- Pesos Girar
# [-4.04069268  1.38744497 -8.99670856 -2.18461956  9.21339113 -2.81178177
#   0.19637485  9.65459987 -0.40882053  9.85188924 -8.71060887  2.70752698
#  -9.11125389  6.35079494  0.42378771 -8.92498143 -9.45973626 -1.63194986
#   1.45867756  9.5526229  -5.24294222  9.76293572 -8.99004702 -5.86192286
#  -9.64085376 -9.29808582 -7.89428016  8.14154     4.54506868 -0.31774471
#  -9.36154671  9.2979766  -5.39858819 -1.4530112   2.82265982 -9.56153776
#   8.26326626  0.67453113 -3.79252777  7.94580785  2.79350686  6.96919699
#   4.67280037 -6.01191198  2.2566084  -8.06245391 -6.27043557]

# --------ys------- Pesos Diagonal 7060
# [-4.20425524  1.53444182 -1.67308289 -3.49985773 -9.27890491 -9.83023784
#  -5.98523535 -1.1642898  -9.47581686 -9.29663792  9.38431104  8.35461558
#  -9.56177909  7.33893066 -6.16889889 -9.77096521 -8.07213175  9.40863963
#  -9.41158085  9.00963664 -9.34768925 -6.93329079  1.20393585 -9.60185536
#   6.02611604 -6.39896894  3.53965985 -3.76426846  9.47240405  9.59747516
#   3.89011426  8.91652154 -9.1221537   8.44598929  4.17915639 -1.89990752
#   9.55796972  1.47824176 -6.03950006  5.01836632 -5.97217837 -8.98736598
#  -1.64457459  0.41973824  2.61157785 -9.50386485 -6.74143624]