import sim
import numpy as np
import pandas as pd
import scipy
import matplotlib.pyplot as plt
import CorobeuClass
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient

Xd = 0 #-0.85 0.85
Yd = 0.2 #-0.65 0.65

class Simulation():

    def __init__(self):
        self.velocidades = []
        self.numeroAmostras = 50
        self.posicaoPassada = 6
        self.rWheelSpeed = []
        self.lWheelSpeed = []
        self.angle = []
        self.yOut = []
        self.xOut = []
        self.desiredX = []
        self.desiredY = []

    def connect(self, port):
        # client = RemoteAPIClient()
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("Nao foi possivel conectar")
        return clientID
      
    def dataAdquicition(self):
        clientID1 = self.connect(19999) 
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras

        if ((sim.simxGetConnectionId(clientID1) != -1)):# & (sim.simxGetConnectionId(clientID2) != -1) & (sim.simxGetConnectionId(clientID3) != -1)):
            print("Simulando")
            while (a == 1):
                print(i)
                # self.rWheelSpeed.append([])
                # self.lWheelSpeed.append([])
                # self.angle.append([])
                # self.yOut.append([])
                # self.xOut.append([])

                if i == self.numeroAmostras:
                    a = 2
                else:
                    a = 1

                # Obtendos os objetos dos motores e a posi��o do rob� 
                returnCode, motorD = sim.simxGetObjectHandle(clientID1, 'motorR01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, motorE = sim.simxGetObjectHandle(clientID1, 'motorL01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, robot = sim.simxGetObjectHandle(clientID1, 'robot01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                                
                # Obtendo os valores de distancia de cada objeto %%%
                s, positiona = sim.simxGetObjectPosition(clientID1, robot, -1, sim.simx_opmode_streaming)
                while positiona == [0, 0, 0]:
                    s, positiona = sim.simxGetObjectPosition(clientID1, robot, -1, sim.simx_opmode_streaming) 
                self.xOut.append(positiona[0])
                self.yOut.append(positiona[1])
                s, angle = sim.simxGetObjectOrientation(clientID1, robot, -1, sim.simx_opmode_blocking)
                self.angle.append(angle[2])
                s, __, lw = sim.simxGetObjectVelocity(clientID1, motorE, sim.simx_opmode_blocking)
                s, __, rw = sim.simxGetObjectVelocity(clientID1, motorD, sim.simx_opmode_blocking)
                self.rWheelSpeed.append(rw[2])
                self.lWheelSpeed.append(lw[2])
                self.desiredX.append(Xd)
                self.desiredY.append(Yd)
                i = i + 1
            print('Fim da Aquisição')
            dfr01 = [self.xOut, self.yOut, self.angle, self.rWheelSpeed, self.lWheelSpeed, self.desiredX, self.desiredY]
            ir01 = ['X', 'Y', 'Gama', 'Vr', 'Vl', 'Xd', 'Yd']
            dfd = pd.DataFrame(dfr01, index=ir01)
            dfd.to_csv("dados23.csv", sep=';')
            print("Arquivo Gerado")

if __name__ == "__main__":
    robot = Simulation()
    robot.dataAdquicition()
    plt.plot(robot.xOut, robot.yOut, color='red', marker='o')
    plt.grid(True)
    plt.xlim(-0.90, 0.9)
    plt.ylim(-0.7, 0.7)
    plt.show()
   