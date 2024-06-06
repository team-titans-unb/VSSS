import sim
import matplotlib.pyplot as plt
import numpy as np
import math as math
# import AlgorithmAStart

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
        self.rWheelSpeed = []
        self.lWheelSpeed = []
        self.angle = []

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
        unomenosalfaana = math.exp(-(deltaT / Filter_e))
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
        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        while positiona == [0, 0, 0]:
            s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
        self.yOut.append(positiona[1])
        self.xOut.append(positiona[0])

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
    
    def Aquire_Data(self):
        s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
        while positiona == [0, 0, 0]:
            s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming) 
        self.instPosition[0] = positiona[0]
        self.instPosition[1] = positiona[1] 
        self.xOut.append(positiona[0])
        self.yOut.append(positiona[1])
        s, angle = sim.simxGetObjectOrientation(self.clientID, self.robot, -1, sim.simx_opmode_blocking)
        self.angle.append(angle[2])
        s, __, lw = sim.simxGetObjectVelocity(self.clientID, self.motorE, sim.simx_opmode_blocking)
        s, __, rw = sim.simxGetObjectVelocity(self.clientID, self.motorD, sim.simx_opmode_blocking)
        self.rWheelSpeed.append(rw[2])
        self.lWheelSpeed.append(lw[2])
        return [positiona[0], positiona[1]], [lw[2], rw[2]], angle[2]

if __name__ == "__main__":

    crb01 = Corobeu(19999, 'robot01', 'motorL01', 'motorR01')
    # # crb02 = Corobeu(19999, 'robot02#0', 'motorL02#0', 'motorR02#0')
    # # crb03 = Corobeu(19999, 'robot03#1', 'motorL03#1', 'motorR03#1')
    # kpi = 0.001
    # kii = 0.0
    # kdi = 0.0
    # deltaT = 0.05
    # # yideal = [2]
    # # xideal = [2]
    # # square_value_x = [[2, 3], [5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    # # square_value_y = [[2, 3], [0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    # square_value_x = []#, [0, 5.5]]
    # square_value_y = []#, [0, 0.5]]
    # x_init = [0.35, 0.102, 0.30]#, 4]
    # y_init = [0.65, 1.106, 1.525]#, 1]
    # #### formacion rombo ####
    # # x_end = [3, 2, 3, 4]
    # # y_end = [4, 3, 2, 3]
    # ###### formacion triangulo ####
    # x_end = [0.028, 1, 0.5]#, 4]
    # y_end = [1.107, 1, 0.5]#, 3]
    # ###### formacion fila ####
    # # x_end = [3, 3, 3, 3]
    # # y_end = [4, 3, 2, 4.5]

    # # for i in range(len(x_end)):
    # # pp0 = AlgorithmAStart.Trajectory_Generation()
    # # pp1 = AlgorithmAStart.Trajectory_Generation()
    # # pp2 = AlgorithmAStart.Trajectory_Generation()

    # x0 = x_init[0]
    # y0 = y_init[0]
    # xend0 = x_end[0]
    # yend0= y_end[0]

    # x1 = x_init[1]
    # y1 = y_init[1]
    # xend1 = x_end[1]
    # yend1 = y_end[1]

    # x2 = x_init[2]
    # y2 = y_init[2]
    # xend2 = x_end[2]
    # yend2 = y_end[2]

    # pp0.AStar(x0, y0, xend0, yend0, square_value_x, square_value_y)
    # xideal0 = pp0.xtrajectory_send
    # yideal0 = pp0.ytrajectory_send

    # pp1.AStar(x1, y1, xend1, yend1, square_value_x, square_value_y)
    # xideal1 = pp1.xtrajectory_send
    # yideal1 = pp1.ytrajectory_send

    # pp2.AStar(x2, y2, xend2, yend2, square_value_x, square_value_y)
    # xideal2 = pp2.xtrajectory_send
    # yideal2 = pp2.ytrajectory_send

    # for path in range(len(xideal0)):
    #     x0 = [xideal0[path], yideal0[path]]
    #     # x1 = [xideal1[path], yideal1[path]]
    #     # x2 = [xideal2[path], yideal2[path]]
    #     # if i == 0:
    #     crb01.Robot_CRB(x0, kpi, kii, kdi, deltaT)
    #     # # elif i == 1:
    #     # crb02.Robot_CRB(x1, kpi, kii, kdi, deltaT)
    #     # # else:
    #     # crb03.Robot_CRB(x2, kpi, kii, kdi, deltaT)

