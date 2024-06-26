import math
import sim
import numpy as np
import math as mat
import ANNClass

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
        kp = 2
        ki = 0.01
        kd = 7.9
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
                while cont0 < 5:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    cont0 = cont0 + 1
                # print(positiona)
                if positiona == [0, 0, 0]:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                else:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    # print(f'positions = {positiona}')
                    phid = math.atan2((pathY - positiona[1]), (pathX - positiona[0]))

                    error_phi = phid - self.phi
                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kp, ki, kd, deltaT,
                                                                                            error_phi, interror_phi,
                                                                                            fant_phi, Integral_part_phi)

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
        spd = ANNClass.ArtificialNeuralNetwork(50)
        a = 1

        W_B = [9.500083770798813,-7.048627638470341,4.565488868373927,-5.15839098004682,-7.11988327448397,
               -7.6933471362981045,-0.14967111968939162,1.0420280553245673,0.8761360270602279,-3.866038265961346,
               0.5931729498128842,9.821738780052467,0.6563196857398859,-5.040705533657092,9.421486950658625,
               9.667540551394271,3.6015678012753103,-0.5384525569128837,-9.751039010975445,7.897070448108166,
               6.84392299931182,9.496435129583695,9.299146157514846,-4.187356898904916,9.40934922346995,
               -1.2164707154317387,2.7673767687534525,-9.450181373743986,5.094346503769871,-9.018196837298047,
               -8.38212908212964,-8.735346088346722,-9.223047036412842,-3.5364913998666907,9.462482203953911,
               4.503284243322762,-6.517604944062597,-6.164227873409657,5.197536016784519,-4.458222483103241,
               -9.642454038016586,-4.223866851757718,8.82594815865603,2.6554039166930044,-9.261887586253152,
               -9.163007567236807,-5.402219031661269]
        
        if (sim.simxGetConnectionId(self.clientID) != -1):

            print("Connect")
            while (a == 1):

                s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                while positiona == [0, 0, 0]:
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
                y_atual = positiona[1]
                x_atual = positiona[0]
                
                s, angle = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
                phi_atual = angle[2]

                error_distance = math.sqrt((pathY - positiona[1])**2 + (pathX - positiona[0])**2)

                if error_distance <= 0.02:
                    a = 0

                self.posError.append(error_distance)
            
                wheelSpeeds = spd.mlp432([x_atual, y_atual, phi_atual, pathX, pathY], W_B[:38], W_B[38:])
                print(wheelSpeeds)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 10000 * wheelSpeeds[1], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 1000 * wheelSpeeds[0], sim.simx_opmode_blocking)


        

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
    
if __name__ == "__main__":
    crb01 = Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    crb01.Micro_Behaviors(0, 0.2, [0.4, 0.2])