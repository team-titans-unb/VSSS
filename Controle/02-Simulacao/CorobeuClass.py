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
        self.v_max = 8
        self.v_min = -8
        self.v_linear = 5
        self.instPosition = [0, 0]
        self.posError = []
        self.mae = 0
        self.mse = 0
        self.max_error = 0
        self.path_length_deviation = 0

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
        vd = (2 * U + omega * 7.5) / (2 * 3.2)
        vl = (2 * U - omega * 7.5) / (2 * 3.2)
        a = 1
        # print(f'omega={omega}')

        if omega >= 1 or omega <= -1:
            Max_Speed = 0.01
            Min_Speed = -0.01
        else:
            Max_Speed = self.v_max
            Min_Speed = self.v_min

        ### Saturation speed upper ###

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        ### Saturation speed Lower ###

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        if error_distance_relative <= 0.02:
            a = 0
        else:
            pass

        ### Arrive the end point posisition ###

        if error_distance_global <= 0.03:
            a = 0
            vl = 0
            vd = 0
        else:
            pass

        return vl, vd, a
    
    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        """""
        Function used to calculate the omega value (output PID) used on the speed function
            argument :
                kp              (Float) = Proportional constant used on the PID controller
                ki              (Float) = Integral constant used on the PID controller
                kd              (Float) = Derivative constant used on the PID controller
                deltaT          (Float) = Sample time
                error           (Float) = Phi error between the robot and the goal point
                interror        (Float) = Error Integral
                fant            (Float  = Used on the derivative part
                Integral_part   (Float) = Integral part

            outputs : 
               PID              (Float) = Omega value used on the speed function
               f                (Float) = f value use on the derivative part
               interror         (Float) = Error Integral
               Integral_part    (Float) = Integral part
        """""

        ### Max value of saturation in the integral part ###

        Integral_saturation = 10

        ### Find the filter e ####

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        # print(f'mayor == {mayor}')
        Filter_e = 1 / (mayor * 10)

        ### Calculate the derivative part ###

        unomenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - unomenosalfaana
        interror = interror + error
        f = unomenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f/deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        ### Calculate the integral part ###

        if Integral_part > Integral_saturation:
            Integral_part = Integral_saturation
        elif Integral_part < -Integral_saturation:
            Integral_part = -Integral_saturation
        else:
            Integral_part = ki * interror * deltaT

        ### Calculate the omega value (PID output) ###

        PID = kp * error + Integral_part + deerror * kd

        ### Return the principal values ###
        # print(f'Integral_part == {Integral_part}')
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
        ki = 1
        kd = 0.001
        deltaT = 0.05
        #---------------------------
        a = 1
        Number_Iterations = 0
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0
        offset_speed = 2
        alfa = 10

        if (sim.simxGetConnectionId(self.clientID) != -1):

            print("Connect")
            while (a == 1):

                ################Go to Goal ######################  
                s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                
                while positiona == [0, 0, 0]:
                
                    s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                    self.phi = 0
                    # s, ballPos = sim.simxGetObjectPosition(self.clientID, self.ball, -1, sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
            
                # s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
                
                # if cont0 == 1:
                    
                
                # cont0 = cont0 + 1
        
                # s, positiona = sim.simxGetObjectPosition(self.clientID, self.robot, -1, sim.simx_opmode_streaming)
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
                
                self.phi = self.phi + omega * deltaT

                U = self.v_linear
                # if Number_Iterations >= 100:

                #     k = self.v_linear*(1-math.exp(-alfa*(abs(error_distance))**2))/(abs(error_distance))
                #     U = k*abs(error_distance)*error_distance + offset_speed

                # else:
                #     U = (self.v_linear/(100 - Number_Iterations)) * Number_Iterations

                #### Calculate the right and left speed values ###
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

    def Micro_Behaviors(self, pathX, pathY, End_position):

        W_B = [0.39894832971087163, -2.105905809136049, 0.31736796205179807, -2.105179399172271] # Reto
        # W_B = [-0.024973293083546694, -2.870969075689935, 0.04775503893057847, -2.3766928990983884] # Anti-Horario
        # W_B = [0.06438700509254595, -2.4500444769455267, 0.03583502267106433, -2.69537854438019] # Horario
        
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
                phi_atual = omega[i] - 1.5708

                error_distance = math.sqrt((pathY - y_atual)**2 + (pathX - x_atual)**2)
                self.posError.append(error_distance)
                # phid = math.atan2(pathY - y_atual, pathX - x_atual) - phi_atual
                phid = math.atan2(pathY - y_atual, pathX - x_atual)
                error_phi = phid - phi_atual
                
                #### Error distance next point and the global is calculate ###
                
                error_distance_global = math.sqrt(
                    (End_position[1] - y_atual) ** 2 + (End_position[0] - x_atual) ** 2)

                if error_distance <= 0.04:
                    a = 0

                # if x_atual > End_position[0]:
                #     a = 0

                if error_distance_global <= 0.02:
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0, sim.simx_opmode_blocking)
                    a = 0

                weightsL = W_B[0]
                biasL = W_B[1]
                weightsR = W_B[2]
                biasR = W_B[3]

                speedL = spd.neuron(error_phi, weightsL, biasL)
                speedR = spd.neuron(error_phi, weightsR, biasR)

                Vd.append(speedL/0.032)
                Ve.append(speedR/0.032)
                
                # if Vd[i] > 8:
                #     Vd[i] = 8
                # else:
                #     pass
                # if Ve[i] > 8:
                #     Ve[i] = 8
                # else:
                #     pass

                print("Velocidades")
                print([Vd[i], Ve[i]])
                
                sim.simxSetJointTargetVelocity(self.clientID, self.motorE, Ve[i], sim.simx_opmode_blocking)
                sim.simxSetJointTargetVelocity(self.clientID, self.motorD, 0.95*Vd[i], sim.simx_opmode_blocking)

                self.yOut.append(y_atual)
                self.xOut.append(x_atual)

                # if x_atual < End_position[0]:
                #     a = 0

                i = i + 1

    def calculate_errors(self, pathX, pathY, filename='errors.txt'):
        numPoints = len(self.xOut)
        originalIndices = np.arange(0, len(pathX))
        targetIndices = np.linspace(0, len(pathX) - 1, numPoints)

        interpolated_pathX = np.interp(targetIndices, originalIndices, pathX)
        interpolated_pathY = np.interp(targetIndices, originalIndices, pathY)

        squared_diff_x = [(ix - ax) ** 2 for ix, ax in zip(interpolated_pathX, self.xOut)]
        squared_diff_y = [(iy - ay) ** 2 for iy, ay in zip(interpolated_pathY, self.yOut)]
        
        mse = np.mean(squared_diff_x + squared_diff_y)
        mae = np.mean([math.sqrt(sx + sy) for sx, sy in zip(squared_diff_x, squared_diff_y)])
        max_error = np.max([math.sqrt(sx + sy) for sx, sy in zip(squared_diff_x, squared_diff_y)])

        planned_path_length = np.sum([math.sqrt((interpolated_pathX[i + 1] - interpolated_pathX[i]) ** 2 + (interpolated_pathY[i + 1] - interpolated_pathY[i]) ** 2) for i in range(numPoints - 1)])
        executed_path_length = np.sum([math.sqrt((self.xOut[i + 1] - self.xOut[i]) ** 2 + (self.yOut[i + 1] - self.yOut[i]) ** 2) for i in range(numPoints - 1)])
        path_length_deviation = abs(planned_path_length - executed_path_length) / planned_path_length

        name = filename + '.txt'
        with open(name, 'w') as file:
            file.write(f'Erro Absoluto Medio (MAE): {mae} m\n')
            file.write(f'Erro Quadratico Medio (MSE): {mse} m2\n')
            file.write(f'Erro Maximo: {max_error} m\n')
            file.write(f'Desvio Medio ao Longo do Caminho: {path_length_deviation * 100:.2f}%\n')

        print(f'Erro Absoluto Médio (MAE): {mae} m')
        print(f'Erro Quadrático Médio (MSE): {mse} m²')
        print(f'Erro Máximo: {max_error} m')
        print(f'Desvio Médio ao Longo do Caminho: {path_length_deviation * 100:.2f}%')

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
    crb01.Follow_Path(0.5, 0, [0.5, 0])