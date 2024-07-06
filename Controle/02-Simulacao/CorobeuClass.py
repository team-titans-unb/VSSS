import math
import sim
import numpy as np
import math as mat
import ANNClass as ann

class Corobeu():
    """
    Classe para controlar o robô Corobeu simulado no CoppeliaSim.
    
    Atributos:
        clientID: ID da conexão com o CoppeliaSim.
        robot: Handle do objeto robô no CoppeliaSim.
        motorE: Handle do motor esquerdo no CoppeliaSim.
        motorD: Handle do motor direito no CoppeliaSim.
        yOut: Lista para armazenar a posição Y do robô.
        xOut: Lista para armazenar a posição X do robô.
        phi: Ângulo atual do robô.
        phi_obs: Ângulo observado do robô.
        v: Velocidade linear do robô.
        instPosition: Posição instantânea do robô.
        posError: Lista para armazenar erros de posição.
    """

    def __init__(self, port, name, motor_E, motor_D):
        """
        Inicializa a classe Corobeu e estabelece a conexão com o CoppeliaSim.
        
        Parâmetros:
            port (int): Porta usada para conectar ao CoppeliaSim.
            name (str): Nome do objeto robô no CoppeliaSim.
            motor_E (str): Nome do motor esquerdo no CoppeliaSim.
            motor_D (str): Nome do motor direito no CoppeliaSim.
        """
        self.clientID, self.robot, self.motorE, self.motorD = self.connect_CRB(port, name, motor_E, motor_D)
        self.yOut = []
        self.xOut = []
        self.phi = 0
        self.phi_obs = 0
        self.v = 8
        self.instPosition = [0, 0]
        self.posError = []

    def connect_CRB(self, port, name, motor_E, motor_D):
        """
        Função usada para comunicar-se com o CoppeliaSim.
        
        Parâmetros:
            port (int): Porta usada para conectar ao CoppeliaSim.
            name (str): Nome do objeto robô no CoppeliaSim.
            motor_E (str): Nome do motor esquerdo no CoppeliaSim.
            motor_D (str): Nome do motor direito no CoppeliaSim.
            
        Retorna:
            tuple: Contém o ID da conexão, o handle do robô, e os handles dos motores esquerdo e direito.
        """
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("no se pudo conectar")

        returnCode, robot = sim.simxGetObjectHandle(clientID, name, 
                                                    sim.simx_opmode_blocking)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, motor_E,
                                                     sim.simx_opmode_blocking)
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, motor_D,
                                                     sim.simx_opmode_blocking)
        return clientID, robot, MotorE, MotorD
    
    def Speed_CRB(self, U, omega, error_distance_relative, error_distance_global):
        """
        Calcula e retorna as velocidades dos motores baseadas na velocidade linear U e angular omega.
        Possui condições de paradas, onde quando o robô chegar no ponto final da trajetória, sua velocidade
        é zerada, mas ao chegar em posições intermediárias apenas muda o próximo ponto desejado.
        
        Speed_CRB(self, U, omega, error_distance_relative, error_distance_global)
        Parâmetros:
            U (float): Velocidade linear do robô.
            omega (float): Velocidade angular do robô.
            error_distance_relative (float): Erro de distância relativo ao ponto atual.
            error_distance_global (float): Erro de distância global ao ponto final.
            
        Retorna:
            tuple: Contém as velocidades dos motores esquerdo e direito, e o estado 'a'.
        """
        vd = (2 * U + omega * 7.5) / 2 * 1.6
        vl = (2 * U - omega * 7.5) / 2 * 1.6
        a = 1
        if vd >= 8:
            vd = 8
        if vd <= -8:
            vd = -8

        if vl >= 8:
            vl = 8
        if vl <= -8:
            vl = -8

        if error_distance_relative <= 0.05:
            a = 0
        else:
            pass

        ### Arrive the end point posisition ###

        if error_distance_global <= 0.02:
            a = 0
            vl = 0
            vd = 0
        else:
            pass

        return vl, vd, a
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):
        """
        Controlador PID para o ângulo de orientação do robô.
        
        PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part)
        Parâmetros:
            kp (float): Constante proporcional do controlador PID.
            ki (float): Constante integral do controlador PID.
            kd (float): Constante derivativa do controlador PID.
            deltaT (float): Intervalo de tempo entre as iterações.
            error (float): Erro atual.
            interror (float): Erro integral acumulado.
            fant (float): Erro filtrado anterior.
            Integral_part (float): Parte integral do controlador.
            
        Retorna:
            tuple: Contém o valor PID, erro filtrado, erro integral acumulado e parte integral.
        """
        Integral_saturation = 10
        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        PID = kp * error + Integral_part + deerror * kd
        return PID, f, interror, Integral_part

    def Follow_Path(self, pathX, pathY, End_position):
        """
        Faz o robô seguir um caminho especificado por coordenadas.
        
        Follow_Path(self, pathX, pathY, End_position)
        Parâmetros:
            pathX (float): Coordenada X do próximo ponto do caminho.
            pathY (float): Coordenada Y do próximo ponto do caminho.
            End_position (list): Coordenadas X e Y do ponto final.
        """
        #----------------Constantes
        kp = 20
        ki = 10
        kd = 0.01
        deltaT = 0.01
        #---------------------------
        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0

        if (sim.simxGetConnectionId(self.clientID) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################  
                while cont0 < 3:
                    if cont0 <= 1:
                        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                        # s, ballPos = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_streaming)
                        sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
                        sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
                    else:
                        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                        
                        phid = math.atan2(pathY - positiona[1], pathX - positiona[0])

                        error_phi = phid - self.phi

                        error_distance = math.sqrt((pathY - positiona[1])**2 + (pathX - positiona[0])**2)
                        self.posError.append(error_distance)
                        omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kp, ki, kd, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)
                        self.phi = self.phi + omega * deltaT
                    cont0 = cont0 + 1
            
                s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                # print(f'positions = {positiona}')
                phid = math.atan2((pathY - positiona[1]), (pathX - positiona[0]))

                error_phi = phid - self.phi
                omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kp, ki, kd, deltaT,
                                                                                        error_phi, interror_phi,
                                                                                        fant_phi, Integral_part_phi)

                self.phi = self.phi + omega * deltaT
                error_distance = math.sqrt((pathY - positiona[1])**2 + (pathX - positiona[0])**2)
                self.posError.append(error_distance)
                
                #### Error distance next point and the global is calculate ###
                
                error_distance_global = math.sqrt(
                    (End_position[1] - positiona[1]) ** 2 + (End_position[0] - positiona[0]) ** 2)

                #### speed profiles ####

                U = self.v

                ### calculate new phi robot integrating the angular speed ###

                self.phi = self.phi + omega * deltaT

                #### Calculate the right and left speed values ###
                # #### Check if the angular error is within the threshold ###
                # if abs(error_phi) > 0.3:
                #     # Rotate in place to correct orientation
                #     vl = -omega if omega < 0 else omega  # Rotate in place
                #     vd = omega if omega < 0 else -omega  # Rotate in place
                # else:
                #     # Move towards the waypoint
                #     vl, vd, a = self.Speed_CRB(U, omega, error_distance, error_distance_global)

                vl, vd, a = self.Speed_CRB(U, omega, error_distance, error_distance_global)

                #### Send the speed values ####

                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, vl, sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, vd, sim.simx_opmode_blocking)
                print("------")
                print(vl)
                print(vd)
                ### Save values to plot ###

                Number_Iterations = Number_Iterations + 1
                Time_Sample.append(Number_Iterations * deltaT)

                self.yOut.append(positiona[1])
                self.xOut.append(positiona[0])

                # print(Number_Iterations)

        # self.yOut.append(positiona[1])
        # self.xOut.append(positiona[0])

    def Micro_Behaviors(self, pathX, pathY, End_position):

        # W_B = [-5.01741035,  7.3015568  , 3.74523337 , 8.27068089 ,-6.46508538 , -6.55563277,     #Pesos Reto pra Cima
        #         -2.5995025,   3.52918937,  7.89099878,  9.9663086 ,  7.30504368,  -3.38570642,
        #         -2.91147369,  2.52139964 , 9.08486877 ,-8.76833578 , 8.871959   , -9.46906964,
        #         -4.77931204,  9.20620595 , 0.32505323 ,-0.94674904 , 8.99310604 ,  8.23423603,
        #         9.59171953, -8.62683762 , 5.00605245 ,-4.75819819 ,-8.77414539 , -4.30479759,
        #         9.65273876, -6.82077998 ,-9.63130215 , 9.08246552 ,-7.74973591 , -9.65986285,
        #         -0.95544853,  9.58712966 ,-6.1828829  , 9.29140431 , 9.61270472 , -5.29675332,
        #         -5.47583421,  2.7313302  ,-1.70503075 , 3.42999507 , 1.53887427]

        # W_B = [ -1.86365051,  -2.27538616,   6.08390142,  -2.89045463, -10.33631516,      #Reto foi bem bom
        #         4.23766188,  -3.6323266 ,  -8.39896023,  -7.41898823, -19.77095548,
        #         -18.87217549,  -7.29754205, -18.69174596,   2.19989256,  -9.08973375,
        #         -4.60074503,  -0.0540541 ,  19.49996712,  13.53207901, -19.42878875,
        #         19.50870004,   4.26586958,  19.33815447,   2.92876751,  18.28259787,
        #         -8.51414805,  13.32417725,  -1.052109,     3.45686041,   7.74818339,
        #         -7.25987084, -19.28379967,  -6.81791706,  -6.52121758,  -3.50204675,
        #         13.31306306,  -3.64198127,   3.62987612, -11.2970507 ,   0.14632333,
        #         -19.84346989,  18.26240223,  -1.48079776,  19.54425496,  19.69625827,
        #         11.63623866, -18.64303549]

        # W_B = [ 14.64694488,  -6.36927042,   8.83947545,  11.0817705 ,  -1.2565759,
        #         1.65786299 , 19.62009915 , -6.22183817 , 18.32423052 ,  8.82994826,
        #         6.39168664 , 19.10587157 , -7.66432091 , -8.7218808  ,-12.79064504,
        #         8.49103209,   3.2408461 ,   5.25851189,  14.87636276,  13.2031121,
        #         -12.62490383 , 10.65594076 , 15.64438447 ,-19.16514464 ,-19.79652639,
        #         -6.72330271 ,-18.17978538 ,-18.19416844 , 19.18990476 , -3.78296666,
        #         8.10118872 , 19.21243479 , 15.6507282  , 18.98905988 , -6.46169403,
        #         -3.87170752 , -3.49922818 ,-12.12767604 , -2.59257815 ,  6.95873413,
        #         -3.05777665 , -0.56502055 , -5.57639711 ,  7.45578264 , 19.06629418,
        #         1.15370097 ,  6.78967286]

        #Pesos Diagonal pra Cima Direita
        # W_B = [-4.20425524,  1.53444182, -1.67308289, -3.49985773, -9.27890491, -9.83023784, 
        #        -5.98523535, -1.1642898, -9.47581686, -9.29663792, 9.38431104, 8.35461558, 
        #        -9.56177909,  7.33893066, -6.16889889, -9.77096521, -8.07213175,  9.40863963, 
        #        -9.41158085,  9.00963664, -9.34768925, -6.93329079,  1.20393585, -9.60185536, 
        #        6.02611604, -6.39896894,  3.53965985, -3.76426846,  9.47240405,  9.59747516, 
        #        3.89011426,  8.91652154, -9.1221537,  8.44598929,  4.17915639, -1.89990752, 
        #        9.55796972,  1.47824176, -6.03950006,  5.01836632, -5.97217837, -8.98736598, 
        #        -1.64457459,  0.41973824,  2.61157785, -9.50386485, -6.74143624] 
        # 
        # W_B = [ -3.09125238,  19.06461025,  12.94923895,   1.32806065,  -7.81177275,  ##Girar anti horario
        #         -1.81736459,  10.90374796,   7.60350425,   7.7943093 , -19.83404311,
        #         -19.33966384,  -4.73083948,  19.31987355,  13.83133106,  15.73233468,
        #         10.5465348 ,   5.09809345,  -4.33676239, -13.61119497,   5.35244127,
        #         10.53185117,  19.43809151,   7.68432645,  11.2719665 , -10.22375025,
        #         -0.25521828,  -1.99032175, -19.64993012, -13.11036811,   7.22708726,
        #         11.43607538,  13.31618564,  -5.75672113, -12.39645725,  19.45972846,
        #         19.56272515, -13.00702501, -10.5543915 ,  10.59773146,  -6.12563933,
        #         -3.55754586,  -7.74802239,   1.84314958, -11.24032938,  18.56532455,
        #         -18.94459553, -14.66383038]

        # W_B = [ -3.1654063  ,-14.38891641 ,-19.53232111 , -6.33754766 ,-18.11301306,
        #         -15.42210623 ,-14.59073205 ,-17.81224609 , -7.73953553 , 12.42176586,
        #         1.84162476,  -8.29874614, -18.44165367, -19.67210887,  -7.2813423,
        #         1.5378415  ,-13.27015256 ,  3.24792883 ,  7.67935525 , -3.16901713,
        #         3.182569   ,-18.39494675 , 14.87986947 , -5.47909145 , 18.91676971,
        #         16.47990909 , 17.47967776 , -3.64791028 ,-13.98051803 ,  6.72762095,
        #         16.08275076 ,  1.1684511  ,  2.48575246 ,-12.89698845 ,  0.23108102,
        #         11.32640569,  13.49590063, -15.4288431 ,  -9.90018168,  -1.8101304,
        #         -4.64097379 , 16.63740793 ,-14.17029855 , -7.00719827 ,-10.14806961,
        #         -7.61567641 , -7.7236279 ]

        # W_B = [ -4.28362628 ,  3.51094317 ,-18.25879615 ,-10.55613911 , -5.86769267,
        #         -11.70756215 ,-19.77326136 ,-12.86909982 ,  1.96644351 ,  0.08948276,
        #         -19.33146187 ,  6.00808939 ,-19.41625442 ,  6.33603909 ,  2.44299711,
        #         16.05746432 , -1.23705704 , 11.80724927 ,-19.47696316 , -5.90345155,
        #         -9.10001472 ,-19.47039327 , 18.06699354 , -2.32946513 , -6.82313076,
        #         17.16109016 ,  4.66634275 ,-12.79813213 ,-11.14402352 , 19.66043376,
        #         -17.26132668 ,-10.21497724 ,  7.11686033 ,-19.25273488 ,-19.85957031,
        #         11.68034421,  -1.21759677,   2.27537687,  -9.87100434,   3.8822601,
        #         0.95269188 ,  3.72958495 ,  8.40862371 ,  1.29584702 , 13.42773378,
        #         6.29969787, -19.55497635]

        W_B = [  7.03990996,   4.69196667, -11.3473865,   -9.24670135,  -18.19009364,
                 14.56323475,  7.73202405, -18.75541291,  -2.8263516,   -6.5242303,
                 3.19543633,   0.77009031]

        
        spd = ann.ArtificialNeuralNetwork(50)
        a = 1
        i = 0
        positiona = []
        omega =[]
        Vd = []
        Ve = []
      
        if (sim.simxGetConnectionId(self.clientID) != -1):
            print("Iniciando Imitacao")

            sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)

            while (a == 1):
                print(i)
                positiona.append([])
                
                s, positiona[i] = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                while positiona[i] == [0, 0, 0]:
                    s, positiona[i] = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
                
                y_atual = positiona[i][1]
                x_atual = positiona[i][0]
                
                s, angle = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
                omega.append(angle[2])
                phi_atual = omega[i]

                print("Posicoes")
                print([x_atual, y_atual, phi_atual])

                error_distance = math.sqrt((pathY - y_atual)**2 + (pathX - x_atual)**2)
                self.posError.append(error_distance)
                
                #### Error distance next point and the global is calculate ###
                
                error_distance_global = math.sqrt(
                    (End_position[1] - y_atual) ** 2 + (End_position[0] - x_atual) ** 2)

                if error_distance <= 0.02:
                    a = 0

                if error_distance_global <= 0.02:
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
                    a = 0
            
                # Speeds = spd.mlp432([x_atual, y_atual, phi_atual, pathX, pathY], W_B[:38], W_B[38:])
                # Vd.append(Speeds[0]/0.008)
                # Ve.append(Speeds[1]/0.008)
                # Vd.append(Speeds[0])
                # Ve.append(Speeds[1])

                # if Vd[i] > 0.6:
                #     Vd[i] = 0.6
                # else:
                #     pass
                # if Ve[i] > 0.6:
                #     Ve[i] = 0.6
                # else:
                #     pass

                weightsL = W_B[:5]
                biasL = W_B[5]
                weightsR = W_B[6:11]
                biasR = W_B[11]

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

                print("Velocidades")
                print([Vd[i], Ve[i]])
                
                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, Ve[i], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, Vd[i], sim.simx_opmode_blocking)

                self.yOut.append(y_atual)
                self.xOut.append(x_atual)

                i = i + 1


    def Get_Position(self):
        """
        Obtém a posição atual do robô no simulador. Salva as posições no
        atributo 'instPosition' do objeto e retorna estes valores em um tupla
        
        Get_Position(self)
        Retorna:
            tuple: Contém as coordenadas X e Y da posição atual do robô.
        """
        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        while positiona == [0, 0, 0]:
            s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
        self.instPosition[0] = positiona[0]
        self.instPosition[1] = positiona[1] 
        return positiona[0], positiona[1]
    
    def Stop_bot(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
        sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)

if __name__ == "__main__":
    crb01 = Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    crb01.Micro_Behaviors(0, 0, [0, 0])