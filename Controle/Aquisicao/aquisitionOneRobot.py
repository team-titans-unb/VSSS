import sim
# import numpy as np
import pandas as pd
# import scipy
# import matplotlib.pyplot as plt
# from coppeliasim_zmqremoteapi_client import RemoteAPIClient

class Simulation():

    def __init__(self):
        self.velocidades = []
        self.numeroAmostras = 50
        self.posicaoPassada = 6
    
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
        clientID1 = self.connect(19995) 
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras

        rob01 = []      # 
        r01vd = []
        r01ve = []
        r01vv = []
        r01wv = []
        r01wd = []
        r01we = []
        posBall = []    #

        if ((sim.simxGetConnectionId(clientID1) != -1)):# & (sim.simxGetConnectionId(clientID2) != -1) & (sim.simxGetConnectionId(clientID3) != -1)):
            print("Simulando")
            while (a == 1):
                print(i)
                rob01.append([])
                r01vd.append([])
                r01ve.append([])
                r01vv.append([])
                r01wv.append([])
                r01wd.append([])
                r01we.append([])
                posBall.append([])

                if i == self.numeroAmostras:
                    a = 2
                else:
                    a = 1
                # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
                returnCode, mdr01 = sim.simxGetObjectHandle(clientID1, 'motorR01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mer01 = sim.simxGetObjectHandle(clientID1, 'motorL01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, robo01 = sim.simxGetObjectHandle(clientID1, 'robot01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, ball = sim.simxGetObjectHandle(clientID1, 'ball', sim.simx_opmode_blocking)
                
                # %%% Obtendo os valores de distancia de cada objeto %%%
                s, rob01[i] = sim.simxGetObjectPosition(clientID1, robo01, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, posBall[i] = sim.simxGetObjectPosition(clientID1, ball, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, r01vd[i], r01wd[i] = sim.simxGetObjectVelocity(clientID1, mdr01, sim.simx_opmode_blocking)
                s, r01ve[i], r01we[i] = sim.simxGetObjectVelocity(clientID1, mer01, sim.simx_opmode_blocking)
                s, r01vv[i], r01wv[i] = sim.simxGetObjectVelocity(clientID1, robo01, sim.simx_opmode_streaming)
                
                i = i + 1
            print("Fim das aquisicoes")
            dfPositions = [rob01]#, posBall]#, adv01, adv02, adv03]
            iPositions = ['Position R1']#, 'Ball Position']
            dfd = pd.DataFrame(data=dfPositions)
            dfd.to_csv("posicoesAdquiridas.csv", sep=';', index=False)
            print("Arquivo pos Gerado")

            # print(rob01[1][0])

            dfr01 = [rob01, r01vd, r01wd, r01ve, r01we, r01vv, r01wv]
            ir01 = ['Position R1', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfr01, index=ir01)
            dfd.to_csv("robo01.csv", sep=';')
            print("Arquivo r1 Gerado")

if __name__ == "__main__":
    robot = Simulation()
    robot.dataAdquicition()
   