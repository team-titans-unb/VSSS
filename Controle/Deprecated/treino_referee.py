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
        self.rsensor_past= []
        self.numero_muestras = 100
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
                # %%% Obtendos os objetos dos motores e a posi��o do rob� %%%
                returnCode, Motori = sim.simxGetObjectHandle(clientID, 'EngineL',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
                returnCode, Motord = sim.simxGetObjectHandle(clientID, 'EngineR',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
                returnCode, cuerpo = sim.simxGetObjectHandle(clientID, 'Chasis6',
                                                             sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx" de v - rep(Robot Movil)
                # %%% Obtendos os objetos dos sensores do rob� %%%
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
                # %%% Obtendo os valores de distancia de cada objeto %%%

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
                    rsensord1_current.append(rsensord1_past[i])
                    rsensord2_current.append(rsensord2_past[i])
                    rsensord3_current.append(rsensord3_past[i])
                    rsensord4_current.append(rsensord4_past[i])
                    rsensord5_current.append(rsensord5_past[i])
                    rsensord6_current.append(rsensord6_past[i])

                # %%% Obten��o das velocidades do rob� em cm/s %%%
                returnCode, md, _ = sim.simxGetObjectVelocity(clientID, Motord,
                                                              sim.simx_opmode_blocking);  # Obtendo a velocidade da roda direita
                returnCode, mi, _ = sim.simxGetObjectVelocity(clientID, Motori,
                                                              sim.simx_opmode_blocking);  # Obtendo a velocidade da roda esquerda
                mdg.append((md[1] ** 2 + md[0] ** 2) ** 1 / 2);
                mig.append((mi[1] ** 2 + mi[0] ** 2) ** 1 / 2);
                i = i + 1

            self.rsensorf = [rsensord1_past, rsensord1_current, rsensord2_past, rsensord2_current, rsensord3_past,
                             rsensord3_current,
                             rsensord4_past, rsensord4_current, rsensord5_past, rsensord5_current, rsensord6_past,
                             rsensord6_current]
            self.rsensor_past = [rsensord1_current,  rsensord2_current, rsensord3_current,
                                 rsensord4_current,  rsensord5_current, rsensord6_current]
                                 
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
            # %%% Satura��o dos sensores, Nota: Aqui n�o tem necesita de normalizar, devido a que a distancia esta normalizada desde V-REP %%%
    
    def training(self):
        Ractivation=0.55;               # Valor de ativação do padrão giro anti horario
        Lactivation=0.55;               # Valor de ativação do padrão giro horario
        Mediabehavior1=0.25;            # Valor para ativar o micro comportamento 1
        Mediabehavior2=0.5;             # Valor para ativar o micro comportamento 2
        Mediabehavior3=0;               # Valor para ativar o micro comportamento 3
        select_MB = []
        
        for i in range(self.numero_muestras-self.past_position):            
            if(self.rsensor_past[0][i]>=Ractivation):
                select_MB.append(Mediabehavior2)       # Coloca o valor do micro-comportamento 2
            elif(self.rsensor_past[3][i]>=Lactivation):
                select_MB.append(Mediabehavior3)       # Coloca o valor do micro-comportamento 3
            else: 
                select_MB.append(Mediabehavior1)      # Coloca o valor do micro-comportamento 1
   
        print("Training")
        training_object = Bioinspired_algorithms()
        training_object_PSO = training_object.PSO(self.rsensor_past, select_MB)
        self.ys = training_object.out_pso
        dframedata = [self.ys]
        index = ['Pesos Bias']
        dfd = pd.DataFrame(dframedata, index=index)
        dfd.to_csv("PesosPSOC4.csv")
        plt.plot(select_MB)
        plt.title("Padrão")
        plt.show()
    
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
        ImRange= 0.03
        Mediabehavior1=0.25;            # Valor para ativar o micro comportamento 1
        Mediabehavior2=0.5;             # Valor para ativar o micro comportamento 2
        Mediabehavior3=0;               # Valor para ativar o micro comportamento 3
        Vd = []
        Vi = []
        

        if (sim.simxGetConnectionId(clientID) != -1):
            print("Connect")

            while (a == 1):
                print(i)
                positiona.append([])
                if i == self.numero_muestras * 5:
                    a = 1
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
                    self.rsensor_past = [rsensord1[i], rsensord2[i], rsensord3[i], rsensord4[i],rsensord5[i], rsensord6[i]]
                  
                               
                    # self.NN(self.ys, rsensor, self.rvelocidadf, self.numero_muestras, 2, i)

                    #pesos_referee = [-1.59946835, -1.62621924, 1.2501294, -1.69509674, -1.2952991, 0.02427323, 1.2084605]

                    object_neural_network = Neural_network_sim()
                    wbref = [1.0156, -0.8691, 0.2829, 1.7362, -3.6098, 2.3651, -0.7229] #Pesos do Luis
                    #object_neural_network.neural_network_refeere_canonical(0, self.rsensor_past, 0, 3, self.ys)
                    object_neural_network.neural_network_refeere_canonical(0, self.rsensor_past, 0, 3, wbref)
                    P = object_neural_network.output_neuron_referee  
                    print(P)     
                    if (P>Mediabehavior1+ImRange):   # Para girar sentido horário 
                        wb=[-0.201666543, -0.00102102440, -1.72135195, 0.521864450,  -0.628955414, -1.76192865,
                            -1.39792839, -0.756522830,  -1.70138918, -1.19876450, -1.23600897,  1.40690117,
                            -0.324786760, -1.08446599, -1.23305015, -1.66405544, -1.77150849, -0.295850683,
                            1.39579233,  0.955660392,  1.35393433, -1.37641468,  1.24229009, -0.913604308,
                            -0.583340884, -1.18304332]
                            
                    elif (P<Mediabehavior1-ImRange):# Para girar em sentido antihorario
                        wb=[1.28700433,  0.66427092, -1.01892827,  0.29806468, -1.39776929, -1.61012329,
                           -1.25869676, -1.11305619, -0.43045905, -1.40927805,  0.47519327,  1.43111298,
                            1.30334435, -1.24757295, -1.38216318, -1.22442325,  0.10699092, -0.50191784,
                            1.05044407, -1.64234608, -1.89512091,  1.57437579, -0.14609219, -1.51601655,
                           -0.22981181, -1.50801128]
                    else:       # Para avançar no medio de um corredor         
                        wb=[-0.70695835, -0.75090366, -1.48005501,  1.31657005, -1.43600622, -1.66684853,
                            -1.72043672, -0.6208094,   0.44611131, -1.42414918, -1.49819962,  0.23899789,
                            0.99980421,  0.7681298,  -1.58846966, -1.43046761, -1.56485114,  0.79751878,
                            -1.0034032,  -1.51129699, -1.20877726,  1.62819792, -0.7594599,  -1.04839773,
                            0.68300147, -0.8326035]
                    object_neural_network.neural_network_input_realimentation(0, rsensor, self.rvelocidadf, 3, wb)
                    self.valuerecRI = object_neural_network.output_neuron_sr
                    self.valuerecLI = object_neural_network.output_neuron_sl
                    Vd.append(self.valuerecRI)
                    Vi.append(self.valuerecLI)
                    sim.simxSetJointTargetVelocity(clientID, Motord, ((self.valuerecRI * -1) / 0.02275),
                                                   sim.simx_opmode_blocking);
                    sim.simxSetJointTargetVelocity(clientID, Motori, ((self.valuerecLI * -1) / 0.02275),
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
    #lfdobject.dataAdquicition()
    #lfdobject.training()
    lfdobject.imitation()