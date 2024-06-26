import math
import sim
import numpy as np
import math as mat
import ANNClass
import pandas as pd
import scipy
import Bioinspired as bia
import matplotlib.pyplot as plt
import os

class LFDmetodology():
    def __init__(self) -> None:
        self.acqSpeed = []
        self.acqPosit = []
        self.acqAngle = []
        self.acqInputs = []
        self.objX = []
        self.objY = []
        self.nSamples = 50
        self.ys = []
        self.fitVector = []
        self.imtSpeed = []

    def connect(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("conectado a ", port)
        else:
            print("n foi possivel conectar")
        return clientID
    
    def dataAquisition(self, destiny):
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
                _, _, md = sim.simxGetObjectVelocity(clientID, motorD, sim.simx_opmode_blocking)
                _, _, me = sim.simxGetObjectVelocity(clientID, motorE, sim.simx_opmode_blocking)
                omega.append(angles[2])
                # mdg.append((md[1]**2 + md[0]**2) ** 1/2)
                # meg.append((me[1]**2 + me[0]**2) ** 1/2)
                mdg.append(md[2])
                meg.append(me[2])
                self.objX.append(destiny[0])
                self.objY.append(destiny[1])
                i = i+1
            
            Vd = scipy.signal.medfilt(mdg, kernel_size=3)
            Ve = scipy.signal.medfilt(meg, kernel_size=3)
            # Vd = mdg
            # Ve = meg
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

    def training(self, destiny):
        print("Treinando")
        trainObj = bia.Bioinspired_algorithms()
        self.fitVector, self.ys = trainObj.PSO()

if __name__ == "__main__":
    crb = LFDmetodology()
    crb.dataAquisition([0.2, 0.6])
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



