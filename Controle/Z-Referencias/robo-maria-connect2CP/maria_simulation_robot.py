"""
ConnectionCP.py

This code connect from Python3 to Coppeliasim

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
            MARIA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    Jan 23, 2023
"""
from bioinspired.bioinspired_algorithms import *
from coppeliaSim import sim
import pandas as pd
import scipy
from neural_network import *
class LFDmethodology():

    def __init__(self):
        self.rvelocidadf = []
        self.rsensorf = []
        self.numero_muestras = 10
        self.ys = []
        self.fitVector = []
        self.valuerecRI = 0
        self.valuerecLI = 0
        self.past_position = 6

    def connect(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("no se pudo conectar")
        return clientID

    def dataAdquicition(self):

        clientID = self.connect(19999)
        a = 1
        i = 0
        position = []
        sdd1 = []
        rsensord1_current = []
        rsensord1_past = []
        rsensord2_current = []
        rsensord2_past = []
        rsensord3_current = []
        rsensord3_past = []
        rsensord4_current = []
        rsensord4_past = []
        rsensord5_current = []
        rsensord5_past = []
        rsensord6_current = []
        rsensord6_past = []
        mdg = []
        mig = []

        if (sim.simxGetConnectionId(clientID) != -1):
            print("Connect")
            while (a == 1):
                print(i)
                position.append([])
                if i == self.numero_muestras:
                    a = 2
                else:
                    a = 1
                # Obtendos os objetos dos motores e a posi��o do rob� %%%
                returnCode, Motori = sim.simxGetObjectHandle(clientID, 'EngineL',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, Motord = sim.simxGetObjectHandle(clientID, 'EngineR',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'Chasis6',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                # Obtendos os objetos dos sensores do rob� %%%
                returnCode, sensord1_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor4',
                                                                    sim.simx_opmode_oneshot_wait)
                returnCode, sensord2_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor5',
                                                                    sim.simx_opmode_oneshot_wait)
                returnCode, sensord3_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor6',
                                                                    sim.simx_opmode_oneshot_wait)
                returnCode, sensord4_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor7',
                                                                    sim.simx_opmode_oneshot_wait)
                returnCode, sensord5_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor8',
                                                                    sim.simx_opmode_oneshot_wait)
                returnCode, sensord6_past = sim.simxGetObjectHandle(clientID, 'Proximity_sensor9',
                                                                    sim.simx_opmode_oneshot_wait)
                # Obtendo os valores de distancia de cada objeto %%%

                rtd1, sd1, sdd1, _, _ = sim.simxReadProximitySensor(clientID, sensord1_past,
                                                                    sim.simx_opmode_oneshot_wait)
                rtd2, sd2, sdd2, _, _ = sim.simxReadProximitySensor(clientID, sensord2_past,
                                                                    sim.simx_opmode_oneshot_wait)
                rtd3, sd3, sdd3, _, _ = sim.simxReadProximitySensor(clientID, sensord3_past,
                                                                    sim.simx_opmode_oneshot_wait)
                rtd4, sd4, sdd4, _, _ = sim.simxReadProximitySensor(clientID, sensord4_past,
                                                                    sim.simx_opmode_oneshot_wait)
                rti1, sd5, sdd5, _, _ = sim.simxReadProximitySensor(clientID, sensord5_past,
                                                                    sim.simx_opmode_oneshot_wait)
                rti1, sd6, sdd6, _, _ = sim.simxReadProximitySensor(clientID, sensord6_past,
                                                                    sim.simx_opmode_oneshot_wait)
                s, position[i] = sim.simxGetObjectPosition(clientID, cuerpo, -1, sim.simx_opmode_streaming)
                rsensord1_past.append(sdd1[2])
                if rsensord1_past[i] < 0.05 or rsensord1_past[i] > 0.9:
                    rsensord1_past[i] = 0.9
                rsensord2_past.append(sdd2[2])
                if rsensord2_past[i] < 0.05 or rsensord2_past[i] > 0.9:
                    rsensord2_past[i] = 0.9
                rsensord3_past.append(sdd3[2])
                if rsensord3_past[i] < 0.05 or rsensord3_past[i] > 0.9:
                    rsensord3_past[i] = 0.9
                rsensord4_past.append(sdd4[2])
                if rsensord4_past[i] < 0.05 or rsensord4_past[i] > 0.9:
                    rsensord4_past[i] = 0.9
                rsensord5_past.append(sdd5[2])
                if rsensord5_past[i] < 0.05 or rsensord5_past[i] > 0.9:
                    rsensord5_past[i] = 0.9
                rsensord6_past.append(sdd6[2])
                if rsensord6_past[i] < 0.05 or rsensord6_past[i] > 0.9:
                    rsensord6_past[i] = 0.9
                if i >= self.past_position:
                    rsensord1_current.insert(0, rsensord1_past[i])
                    rsensord2_current.insert(0, rsensord2_past[i])
                    rsensord3_current.insert(0, rsensord3_past[i])
                    rsensord4_current.insert(0, rsensord4_past[i])
                    rsensord5_current.insert(0, rsensord5_past[i])
                    rsensord6_current.insert(0, rsensord6_past[i])

                # Obten��o das velocidades do rob� em cm/s 
                returnCode, md, _ = sim.simxGetObjectVelocity(clientID, Motord,
                                                              sim.simx_opmode_blocking)  # Obtendo a velocidade da roda direita
                returnCode, mi, _ = sim.simxGetObjectVelocity(clientID, Motori,
                                                              sim.simx_opmode_blocking)  # Obtendo a velocidade da roda esquerda
                mdg.append((md[1] ** 2 + md[0] ** 2) ** 1 / 2)
                mig.append((mi[1] ** 2 + mi[0] ** 2) ** 1 / 2)
                i = i + 1

            self.rsensorf = [rsensord1_past, rsensord1_current, rsensord2_past, rsensord2_current, rsensord3_past,
                             rsensord3_current,
                             rsensord4_past, rsensord4_current, rsensord5_past, rsensord5_current, rsensord6_past,
                             rsensord6_current]
            Vd = scipy.signal.medfilt(mdg, kernel_size=3)
            Vi = scipy.signal.medfilt(mig, kernel_size=3)
            self.rvelocidadf = [Vd, Vi]
            dframedata = [rsensord1_past, rsensord1_current, rsensord2_past, rsensord2_current, rsensord3_past,
                          rsensord3_current,
                          rsensord4_past, rsensord4_current, rsensord5_past, rsensord5_current, rsensord6_past,
                          rsensord6_current, Vd, Vi]
            index = ['rsensord1_past', 'rsensord1_current', 'rsensord2_past', 'rsensord2_current', 'rsensord3_past',
                     'rsensord3_current',
                     'rsensord4_past', 'rsensord4_current', 'rsensord5_past', 'rsensord5_current', 'rsensord6_past',
                     'rsensord6_current', 'Vd', 'Vi']
            dfd = pd.DataFrame(dframedata, index=index)
            dfd.to_csv("DatatoTraining.csv")

            print("stop")
            # Satura��o dos sensores, Nota: Aqui n�o tem necesita de normalizar, devido a que a distancia esta normalizada desde V-REP %%%
    
    def training(self):
        print("Training")
        training_object = Bioinspired_algorithms()
        training_object_PSO = training_object.PSO(self.rsensorf, self.rvelocidadf,)
        self.ys = training_object.out_pso
    
    def imitation(self):
        clientID = self.connect(19999)
        a = 1
        i = 0
        past_position = 6
        positiona = []
        rsensord1 = []
        rsensord2 = []
        rsensord3 = []
        rsensord4 = []
        rsensord5 = []
        rsensord6 = []
        Vd = []
        Vi = []

        if (sim.simxGetConnectionId(clientID) != -1):
            print("Connect")

            while (a == 1):
                print(i)
                positiona.append([])
                if i == self.numero_muestras * 5:
                    a = 2
                else:
                    a = 1
                # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
                returnCode, Motori = sim.simxGetObjectHandle(clientID, 'EngineL',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, Motord = sim.simxGetObjectHandle(clientID, 'EngineR',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'Chasis6',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                # %%% Obtendos os objetos dos sensores do rob� %%%
                returnCode, sensord1 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor4',
                                                               sim.simx_opmode_oneshot_wait)
                returnCode, sensord2 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor5',
                                                               sim.simx_opmode_oneshot_wait)
                returnCode, sensord3 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor6',
                                                               sim.simx_opmode_oneshot_wait)
                returnCode, sensord4 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor7',
                                                               sim.simx_opmode_oneshot_wait)
                returnCode, sensord5 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor8',
                                                               sim.simx_opmode_oneshot_wait)
                returnCode, sensord6 = sim.simxGetObjectHandle(clientID, 'Proximity_sensor9',
                                                               sim.simx_opmode_oneshot_wait)
                # %%% Obtendo os valores de distancia de cada objeto %%%

                rtd1, sd1, sdd1, _, _ = sim.simxReadProximitySensor(clientID, sensord1, sim.simx_opmode_oneshot_wait)
                rtd2, sd2, sdd2, _, _ = sim.simxReadProximitySensor(clientID, sensord2, sim.simx_opmode_oneshot_wait)
                rtd3, sd3, sdd3, _, _ = sim.simxReadProximitySensor(clientID, sensord3, sim.simx_opmode_oneshot_wait)
                rtd4, sd4, sdd4, _, _ = sim.simxReadProximitySensor(clientID, sensord4, sim.simx_opmode_oneshot_wait)
                rti1, sd5, sdd5, _, _ = sim.simxReadProximitySensor(clientID, sensord5, sim.simx_opmode_oneshot_wait)
                rti1, sd6, sdd6, _, _ = sim.simxReadProximitySensor(clientID, sensord6, sim.simx_opmode_oneshot_wait)
                s, positiona[i] = sim.simxGetObjectPosition(clientID, cuerpo, -1, sim.simx_opmode_streaming)
                rsensord1.append(sdd1[2])
                if rsensord1[i] < 0.05 or rsensord1[i] > 0.9:
                    rsensord1[i] = 0.9
                rsensord2.append(sdd2[2])
                if rsensord2[i] < 0.05 or rsensord2[i] > 0.9:
                    rsensord2[i] = 0.9
                rsensord3.append(sdd3[2])
                if rsensord3[i] < 0.05 or rsensord3[i] > 0.9:
                    rsensord3[i] = 0.9
                rsensord4.append(sdd4[2])
                if rsensord4[i] < 0.05 or rsensord4[i] > 0.9:
                    rsensord4[i] = 0.9
                rsensord5.append(sdd5[2])
                if rsensord5[i] < 0.05 or rsensord5[i] > 0.9:
                    rsensord5[i] = 0.9
                rsensord6.append(sdd6[2])
                if rsensord6[i] < 0.05 or rsensord6[i] > 0.9:
                    rsensord6[i] = 0.9
                if i >= self.past_position:
                    rsensor = [rsensord1[i - self.past_position], rsensord1[i], rsensord2[i - self.past_position],
                               rsensord2[i], rsensord3[i - self.past_position], rsensord3[i],
                               rsensord4[i - self.past_position], rsensord4[i], rsensord5[i - self.past_position],
                               rsensord5[i], rsensord6[i - self.past_position], rsensord6[i]]

                    # self.NN(self.ys, rsensor, self.rvelocidadf, self.numero_muestras, 2, i)
                    object_neural_network = Neural_network_sim()
                    object_neural_network.neural_network_input_realimentation(0, rsensor, self.rvelocidadf, 3, self.ys)
                    self.valuerecRI = object_neural_network.output_neuron_sr
                    self.valuerecLI = object_neural_network.output_neuron_sl
                    Vd.append(self.valuerecRI)
                    Vi.append(self.valuerecLI)
                    sim.simxSetJointTargetVelocity(clientID, Motord, ((self.valuerecRI * -1) / 0.002275),
                                                   sim.simx_opmode_blocking);
                    sim.simxSetJointTargetVelocity(clientID, Motori, ((self.valuerecLI * -1) / 0.002275),
                                                   sim.simx_opmode_blocking);
                i = i + 1

            dframedata = [rsensord1[i - self.past_position], rsensord1[i], rsensord2[i - self.past_position],
                          rsensord2[i], rsensord3[i - self.past_position], rsensord3[i],
                          rsensord4[i - self.past_position], rsensord4[i], rsensord5[i - self.past_position],
                          rsensord5[i], rsensord6[i - self.past_position], rsensord6[i], Vd, Vi]
            index = ['rsensord1_past', 'rsensord1_current', 'rsensord2_past', 'rsensord2_current',
                             'rsensord3_past', 'rsensord3_current',
                             'rsensord4_past', 'rsensord4_current', 'rsensord5_past', 'rsensord5_current',
                             'rsensord6_past', 'rsensord6_current', 'Vd', 'Vi']
            dfd = pd.DataFrame(dframedata, index=index)
            dfd.to_csv("DatatoImplementation.csv")

            
if __name__ == "__main__":
    lfdobject = LFDmethodology()
    lfdobject.dataAdquicition()
    lfdobject.training()
    #WB1 = [-0.2234, 0.9476, -1.7260, 0.9076, -1.9599, -0.6658, 0.2809, -0.6176,
    #       -1.0969, -1.3535, -1.4101, 1.0278,-1.7179, -1.4717, -0.7869, 0.9914] # micro comportamento 1

    #WB2 = [-1.5685, -1.2373, -1.0293, 0.4559, -0.7168, 1.6572, -1.5633, -0.9664,
    #       -0.1796, 1.3985, 1.4419, -1.3624, -1.7463, -1.5169, -1.1115, 1.3636] # micro comportamento 2

    #WB3 = [-1.1134, -0.3398, 0.8823, -1.8398, -1.5859, -1.3597, -1.0778, -1.4451,
    #       -1.8505, -1.3614, -1.6705, 1.2315, -1.5976, -1.4825, 1.5041, 0.3849] # micro comportamento 3

    lfdobject.imitation()