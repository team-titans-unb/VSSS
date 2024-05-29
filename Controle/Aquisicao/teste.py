#import sim
# import numpy as np
# import pandas as pd
# import scipy
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class Corobeu():
    def __init__(self, robotHandle, lMotorHandle, rMotorHandle, clientID):
        self.sampleNum = 50
        self.positions = []      # 
        self.rWheelSpeed = []
        self.lWheelSpeed = []
        self.linVelocity = []
        self.angVelocity = []
        self.robotHandle = robotHandle
        self.lMotorHandle = lMotorHandle
        self.rMotorHandle = rMotorHandle
        self.clientID = clientID
    
    def acquisition(self):
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras

        print(f'adquirindo velocidades do {self.robotHandle}')
        while (a == 1):
            print(i)
            self.rWheelSpeed.append([])
            self.lWheelSpeed.append([])
            self.positions.append([])

            if i == self.sampleNum:
                a = 2
            else:
                a = 1
            # %%% Obtendos os objetos dos motores e a posicao do robo %%%
            returnCode, robot = sim.getObject(self.clientID, self.robotHandle, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
            returnCode, lMotor = sim.getObject(self.clientID, self.lMotorHandle, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
            returnCode, rMotor = sim.getObject(self.clientID, self.rMotorHandle, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
            
            # returnCode, field = sim.simxGetObjectHandle(clientID1, 'Campo', sim.simx_opmode_blocking)
            # %%% Obtendo os valores de distancia de cada objeto %%%
            s, self.positions[i] = sim.simxGetObjectPosition(self.clientID, robot, sim.sim_handle_parent, sim.simx_opmode_streaming)
            
            s, self.lWheelSpeed[i], __ = sim.simxGetObjectVelocity(self.clientID, lMotor, sim.simx_opmode_blocking)
            s, self.rWheelSpeed[i], __ = sim.simxGetObjectVelocity(self.clientID, rMotor, sim.simx_opmode_blocking)
            
            i = i + 1
        print("Fim das aquisicoes")

    def Imitation(self):
        # a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras
        print(f'Imitando velocidades do {self.robotHandle}')
        #while (a == 1):
        for i in range(self.sampleNum):
            # if i == self.sampleNum:
            #     a = 2
            # else:
            #     a = 1
            print(self.lWheelSpeed[i])
            print(self.rWheelSpeed[i])
            sim.simxSetJointTargetVelocity(self.clientID, self.lMotorHandle, self.lWheelSpeed[i], sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.rMotorHandle, self.rWheelSpeed[i], sim.simx_opmode_blocking)


if __name__ == "__main__":
    client = RemoteAPIClient(port=19995)
    sim = client.require('sim')
    robot1 = sim.getObject('/robot01')
    motorE = sim.getObject('/motorL01')

    sim.startSimulation()
    while sim.getSimulationTime() < 1:
        print('A')
        sim.getJointTargetVelocity(motorE)
    sim.stopSimulation()