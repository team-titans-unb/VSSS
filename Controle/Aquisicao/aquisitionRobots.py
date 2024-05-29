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
        clientID1 = self.connect(19995) #conectar ao campo p pegar todas as posicoes
        # clientID2 = self.connect(19998)
        # clientID3 = self.connect(19989)
        # clientID4 = self.connect(19996)
        # clientID5 = self.connect(19995)
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras

        rob01 = []      # 
        r01vd = []
        r01ve = []
        r01vv = []
        r01wv = []
        r01wd = []
        r01we = []

        rob02 = []      #
        r02vd = []
        r02ve = []
        r02vv = []
        r02wv = []
        r02wd = []
        r02we = []

        rob03 = []      #
        r03vd = []
        r03ve = []
        r03vv = []
        r03wv = []
        r03wd = []
        r03we = []
        
        adv01 = []      #
        e01vd = []
        e01ve = []
        e01vv = []
        e01wv = []
        e01wd = []
        e01we = []

        adv02 = []      #
        e02vd = []
        e02ve = []
        e02vv = []
        e02wv = []
        e02wd = []
        e02we = []

        adv03 = []      #
        e03vd = []
        e03ve = []
        e03vv = []
        e03wv = []
        e03wd = []
        e03we = []

        posBall = []    #

        if ((sim.simxGetConnectionId(clientID1) != -1)):# & (sim.simxGetConnectionId(clientID2) != -1) & (sim.simxGetConnectionId(clientID3) != -1)):
            print("Simulando")
            while (a == 1):
                print(i)
                rob01.append([])
                rob02.append([])
                rob03.append([])
                
                adv01.append([])
                adv02.append([]) 
                adv03.append([])

                r01vd.append([])
                r01ve.append([])
                r02vd.append([])
                r02ve.append([])
                r03vd.append([])
                r03ve.append([])
                e01vd.append([])
                e01ve.append([])
                e02vd.append([])
                e02ve.append([])
                e03vd.append([])
                e03ve.append([])
                r01vv.append([])
                r01wv.append([])
                r02vv.append([])
                r02wv.append([])
                r03vv.append([])
                r03wv.append([])
                e01vv.append([])
                e01wv.append([])
                e02vv.append([])
                e02wv.append([])
                e03vv.append([])
                e03wv.append([])
                r01wd.append([])
                r01we.append([])
                r02wd.append([])
                r02we.append([])
                r03wd.append([])
                r03we.append([])
                e01wd.append([])
                e01we.append([])
                e02wd.append([])
                e02we.append([])
                e03wd.append([])
                e03we.append([])
                posBall.append([])

                if i == self.numeroAmostras:
                    a = 2
                else:
                    a = 1
                # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
                returnCode, mdr01 = sim.simxGetObjectHandle(clientID1, 'motorR01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mer01 = sim.simxGetObjectHandle(clientID1, 'motorL01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, mdr02 = sim.simxGetObjectHandle(clientID1, 'motorR02#0', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mer02 = sim.simxGetObjectHandle(clientID1, 'motorL02#0', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, mdr03 = sim.simxGetObjectHandle(clientID1, 'motorR03#1', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mer03 = sim.simxGetObjectHandle(clientID1, 'motorL03#1', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, mde01 = sim.simxGetObjectHandle(clientID1, 'motorR04#2', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mee01 = sim.simxGetObjectHandle(clientID1, 'motorL04#2', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, mde02 = sim.simxGetObjectHandle(clientID1, 'motorR05#3', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mee02 = sim.simxGetObjectHandle(clientID1, 'motorL05#3', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, mde03 = sim.simxGetObjectHandle(clientID1, 'motorR06#4', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, mee03 = sim.simxGetObjectHandle(clientID1, 'motorL06#4', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                
                returnCode, robo01 = sim.simxGetObjectHandle(clientID1, 'robot01', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, robo02 = sim.simxGetObjectHandle(clientID1, 'robot02#0', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, robo03 = sim.simxGetObjectHandle(clientID1, 'robot03#1', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, ball = sim.simxGetObjectHandle(clientID1, 'ball', sim.simx_opmode_blocking)
                returnCode, enemy01 = sim.simxGetObjectHandle(clientID1, 'enemy01#2', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, enemy02 = sim.simxGetObjectHandle(clientID1, 'enemy02#3', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                returnCode, enemy03 = sim.simxGetObjectHandle(clientID1, 'enemy03#4', sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
               
                # returnCode, field = sim.simxGetObjectHandle(clientID1, 'Campo', sim.simx_opmode_blocking)
                # %%% Obtendo os valores de distancia de cada objeto %%%
                s, rob01[i] = sim.simxGetObjectPosition(clientID1, robo01, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, rob02[i] = sim.simxGetObjectPosition(clientID1, robo02, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, rob03[i] = sim.simxGetObjectPosition(clientID1, robo03, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, posBall[i] = sim.simxGetObjectPosition(clientID1, ball, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, adv01[i] = sim.simxGetObjectPosition(clientID1, enemy01, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, adv02[i] = sim.simxGetObjectPosition(clientID1, enemy02, sim.sim_handle_parent, sim.simx_opmode_streaming)
                s, adv03[i] = sim.simxGetObjectPosition(clientID1, enemy03, sim.sim_handle_parent, sim.simx_opmode_streaming)
                
                s, r01vd[i], r01wd[i] = sim.simxGetObjectVelocity(clientID1, mdr01, sim.simx_opmode_blocking)
                s, r01ve[i], r01we[i] = sim.simxGetObjectVelocity(clientID1, mer01, sim.simx_opmode_blocking)
                s, r02vd[i], r02wd[i] = sim.simxGetObjectVelocity(clientID1, mdr02, sim.simx_opmode_blocking)
                s, r02ve[i], r02we[i] = sim.simxGetObjectVelocity(clientID1, mer02, sim.simx_opmode_blocking)
                s, r03vd[i], r03wd[i] = sim.simxGetObjectVelocity(clientID1, mdr03, sim.simx_opmode_blocking)
                s, r03ve[i], r03we[i] = sim.simxGetObjectVelocity(clientID1, mer03, sim.simx_opmode_blocking)
                s, e01vd[i], e01wd[i] = sim.simxGetObjectVelocity(clientID1, mde01, sim.simx_opmode_blocking)
                s, e01ve[i], e01we[i] = sim.simxGetObjectVelocity(clientID1, mee01, sim.simx_opmode_blocking)
                s, e02vd[i], e02wd[i] = sim.simxGetObjectVelocity(clientID1, mde02, sim.simx_opmode_blocking)
                s, e02ve[i], e02we[i] = sim.simxGetObjectVelocity(clientID1, mee02, sim.simx_opmode_blocking)
                s, e03vd[i], e03wd[i] = sim.simxGetObjectVelocity(clientID1, mde03, sim.simx_opmode_blocking)
                s, e03ve[i], e03we[i] = sim.simxGetObjectVelocity(clientID1, mee03, sim.simx_opmode_blocking)

                s, r01vv[i], r01wv[i] = sim.simxGetObjectVelocity(clientID1, robo01, sim.simx_opmode_streaming)
                s, r02vv[i], r02wv[i] = sim.simxGetObjectVelocity(clientID1, robo02, sim.simx_opmode_streaming)
                s, r03vv[i], r03wv[i] = sim.simxGetObjectVelocity(clientID1, robo03, sim.simx_opmode_streaming)
                s, e01vv[i], e01wv[i] = sim.simxGetObjectVelocity(clientID1, enemy01, sim.simx_opmode_streaming)
                s, e02vv[i], e02wv[i] = sim.simxGetObjectVelocity(clientID1, enemy02, sim.simx_opmode_streaming)
                s, e03vv[i], e03wv[i] = sim.simxGetObjectVelocity(clientID1, enemy03, sim.simx_opmode_streaming)
                
                i = i + 1
            print("Fim das aquisicoes")
            dfPositions = [rob01, rob02, rob03, posBall, adv01, adv02, adv03]
            iPositions = ['Position R1', 'Position R2', 'Position R3', 'Ball Position', 'Position E1', 'Position E2', 'Position E3']
            dfd = pd.DataFrame(dfPositions, index=iPositions)
            dfd.to_csv("posicoesAtaque1.csv")
            print("Arquivo pos Gerado")

            dfr01 = [rob01, r01vd, r01wd, r01ve, r01we, r01vv, r01wv]
            ir01 = ['Position R1', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfr01, index=ir01)
            dfd.to_csv("robo01Ataque1.csv")
            print("Arquivo r1 Gerado")

            dfr02 = [rob02, r02vd, r02wd, r02ve, r02we, r02vv, r02wv]
            ir02 = ['Position R2', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfr02, index=ir02)
            dfd.to_csv("robo02Ataque1.csv")
            print("Arquivo r2 Gerado")

            dfr03 = [rob03, r03vd, r03wd, r03ve, r03we, r03vv, r03wv]
            ir03 = ['Position R3', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfr03, index=ir03)
            dfd.to_csv("robo03Ataque1.csv")
            print("Arquivo r3 Gerado")

            dfe01 = [adv01, e01vd, e01wd, e01ve, e01we, e01vv, e01wv]
            ie01 = ['Position E1', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfe01, index=ie01)
            dfd.to_csv("advr01Ataque1.csv")
            print("Arquivo e1 Gerado")

            dfe02 = [adv02, e02vd, e02wd, e02ve, e02we, e02vv, e02wv]
            ie02 = ['Position E2', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfe02, index=ie02)
            dfd.to_csv("advr02Ataque1.csv")
            print("Arquivo e2 Gerado")

            dfe03 = [adv03, e03vd, e03wd, e03ve, e03we, e03vv, e03wv]
            ie03 = ['Position E3', 'vd', 'wd', 've', 'we', 'v linear', 'v angular']
            dfd = pd.DataFrame(dfe03, index=ie03)
            dfd.to_csv("advr03Ataque1.csv")
            print("Arquivo e3 Gerado")

if __name__ == "__main__":
    robot = Simulation()
    robot.dataAdquicition()
   