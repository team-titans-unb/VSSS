import math
import sim
import numpy as np
from ANNClass import *
import pandas as pd
import scipy
import Bioinspired as bia
import matplotlib.pyplot as plt
import os

class LFDmetodology():
    def __init__(self, destiny = [0,0]):
        self.acqSpeed = []
        self.acqPosit = []
        self.acqAngle = []
        self.acqInputs = []
        self.destiny = destiny
        self.objX = []
        self.objY = []
        self.nSamples = 50
        self.ys = []
        self.fitVector = []
        self.imtSpeed = []
        self.imtError = []

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
                    _, md, _ = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_blocking)
                    _, me, _ = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_blocking)
                _, positions[i] = sim.simxGetObjectPosition(clientID, corpo, -1, sim.simx_opmode_streaming)
                _, angles = sim.simxGetObjectOrientation(clientID, corpo, -1, sim.simx_opmode_blocking)
                _, me, _ = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_blocking)
                _, md, _ = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_blocking)
                omega.append(angles[2])
                mdg.append((md[1]**2 + md[0]**2) ** 1/2)
                meg.append((me[1]**2 + me[0]**2) ** 1/2)
                # mdg.append(md[2])
                # meg.append(me[2])
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
        print("Treinando")
        trainObj = bia.Bioinspired_algorithms()
        self.fitVector, self.ys = trainObj.PSO(self.acqInputs, self.acqSpeed)

    def imitation(self):
        clientID = self.connect(19999)
        spd = ArtificialNeuralNetwork(50)
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

                if error_distance <= 0.02:
                    a = 0

                self.imtError.append(error_distance)
            
                Speeds = spd.mlp432([x_atual, y_atual, phi_atual, pathX, pathY], self.ys[:38], self.ys[38:])
                Vd.append(Speeds[0]/0.008)
                Ve.append(Speeds[1]/0.008)
                print([Vd[i], Ve[i]])
                sim.simxSetJointTargetVelocity(clientID, motorE, Ve[i]/0.008, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(clientID, motorD, Vd[i]/0.008, sim.simx_opmode_blocking)
                i = i + 1
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)


if __name__ == "__main__":
    crb = LFDmetodology(destiny = [-0.35, 0])
    crb.dataAquisition()

    spdD = crb.acqSpeed[0]
    spdE = crb.acqSpeed[1]
    plt.figure()
    plt.plot(spdD, label='Velocidades roda direita')
    plt.plot(spdE, label='Velocidades roda esquerda')
    plt.legend()
    plt.ylim(-1, 1)
    plt.show()
    plt.figure()
    plt.plot(crb.acqPosit[0], crb.acqPosit[1], label='positions')
    plt.legend()
    plt.xlim(-0.9, 0.9)
    plt.ylim(-0.7, 0.7)
    plt.show()
    plt.plot(crb.acqAngle, label='anglus')
    plt.legend()
    plt.ylim(-3.2, 3.2)
    plt.show()

    crb.training()

    crb.imitation()

# --------ys------- Pesos Girar
# [-4.04069268  1.38744497 -8.99670856 -2.18461956  9.21339113 -2.81178177
#   0.19637485  9.65459987 -0.40882053  9.85188924 -8.71060887  2.70752698
#  -9.11125389  6.35079494  0.42378771 -8.92498143 -9.45973626 -1.63194986
#   1.45867756  9.5526229  -5.24294222  9.76293572 -8.99004702 -5.86192286
#  -9.64085376 -9.29808582 -7.89428016  8.14154     4.54506868 -0.31774471
#  -9.36154671  9.2979766  -5.39858819 -1.4530112   2.82265982 -9.56153776
#   8.26326626  0.67453113 -3.79252777  7.94580785  2.79350686  6.96919699
#   4.67280037 -6.01191198  2.2566084  -8.06245391 -6.27043557]

