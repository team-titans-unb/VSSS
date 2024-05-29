import sim
import matplotlib.pyplot as plt
import threading
import time

class Corobeu():
    def __init__(self, robotHandle):
        self.sampleNum = 50
        self.positions = []      # 
        self.rWheelSpeed = []
        self.lWheelSpeed = []
        self.linVelocity = []
        self.angVelocity = []
        self.robotHandle = robotHandle
        self.clientID = 1
        self.imitation_thread = None
        self.stop_event = threading.Event()

    def Connect(self, port):
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado a", port)
        else:
            print("Não foi possivel conectar")
        return clientID
    
    def Acquisition(self, leftWheel, rightWheel):
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras
        self.clientID = self.Connect(19995)

        print(f'adquirindo velocidades do {self.robotHandle}')
        #  Obtendos os objetos dos motores e a posicao do robo %%%
        returnCode, robot = sim.simxGetObjectHandle(self.clientID, self.robotHandle, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        returnCode, lMotor = sim.simxGetObjectHandle(self.clientID, leftWheel, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, rMotor = sim.simxGetObjectHandle(self.clientID, rightWheel, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
            
        while (a == 1):
            # print(i)
            self.positions.append([])

            if i == self.sampleNum:
                a = 2
            else:
                a = 1
            #  Obtendo os valores de distancia de cada objeto %%%
            s, self.positions[i] = sim.simxGetObjectPosition(self.clientID, robot, sim.sim_handle_parent, sim.simx_opmode_streaming)
            
            s, __, rw = sim.simxGetObjectVelocity(self.clientID, lMotor, sim.simx_opmode_blocking)
            s, __, lw = sim.simxGetObjectVelocity(self.clientID, rMotor, sim.simx_opmode_blocking)
            self.rWheelSpeed.append(rw[2])
            self.lWheelSpeed.append(lw[2])
            i = i + 1
        print("Fim das aquisicoes")

    def Imitation(self, leftMotor, rightMotor):
        self.clientID = self.Connect(19995)
        #  Obtendos os objetos dos motores e a posicao do robo %%%
        returnCode, lMotor = sim.simxGetObjectHandle(self.clientID, leftMotor, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_rightMotor" de v - rep(motor derecho)
        returnCode, rMotor = sim.simxGetObjectHandle(self.clientID, rightMotor, sim.simx_opmode_blocking)  # Obteniendo el objeto "Pioneer_p3dx_leftmotor" de v-rep (motor izquierdo)
        a = 1           # apenas para controle do while 
        i = 0           # controle do numero de amostras
        print(f'Imitando velocidades do {self.robotHandle}')
        while (a == 1):
            # print(i)
            if i == self.sampleNum:
                a = 2
            else:
                a = 1        
            # print(f'e: {self.lWheelSpeed[i]}; r: {self.rWheelSpeed[i]}')

            sim.simxSetJointTargetVelocity(self.clientID, lMotor, self.rWheelSpeed[i], sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(self.clientID, rMotor, self.lWheelSpeed[i], sim.simx_opmode_blocking)
            i = i + 1
        print('Fim da Imitação')

    def start_imitation(self, leftMotor, rightMotor):
        self.stop_imitation()  # Ensure any existing imitation is stopped
        self.stop_event.clear()  # Clear the stop event to allow the new imitation to run
        self.imitation_thread = threading.Thread(target=self.Imitation, args=(leftMotor, rightMotor))
        self.imitation_thread.start()

    def stop_imitation(self):
        if self.imitation_thread and self.imitation_thread.is_alive():
            self.stop_event.set()  # Signal the imitation thread to stop
            self.imitation_thread.join()  # Wait for the imitation thread to finish

def plotPositions(positions):
    x = [pos[0] for pos in positions]
    y = [pos[1] for pos in positions]
    plt.scatter(x, y, color='blue', marker='x')
    plt.grid(True)
    plt.xlim(0, 1.7)
    plt.ylim(0, 1.3)
    plt.show()

#%%%Testando threads %%%
if __name__ == "__main__":
    robots = []
    robotNames = ['robot01', 'robot02#0', 'robot03#1', 'enemy01#2', 'enemy02#3', 'enemy03#4']
    leftWheels = ['rodaEsquerda', 'rodaEsquerda#0', 'rodaEsquerda#1', 'rodaEsquerda#2', 'rodaEsquerda#3', 'rodaEsquerda#4']
    rightWheels = ['rodaDireita', 'rodaDireita#0', 'rodaDireita#1', 'rodaDireita#2', 'rodaDireita#3', 'rodaDireita#4']
    leftMotors = ['motorL01', 'motorL02#0', 'motorL03#1', 'motorL04#2', 'motorL05#3', 'motorL06#4']
    rightMotors = ['motorR01', 'motorR02#0', 'motorR03#1', 'motorR04#2', 'motorR05#3', 'motorR06#4']

    # Create robot instances
    for i in range(6):
        robots.append(Corobeu(robotHandle=robotNames[i]))

    # Sequentially acquire and imitate
    for i in range(6):
        # Acquire data from the current robot
        robots[i].Acquisition(leftWheel=leftWheels[i], rightWheel=rightWheels[i])
        
        # Plot positions after acquisition
        plotPositions(robots[i].positions)
        
        # Start imitation for all previously acquired robots
        for j in range(i):
            robots[j].start_imitation(leftMotor=leftMotors[j], rightMotor=rightMotors[j])

    print("All tasks completed.")



# if __name__ == "__main__":
#     crb1 = Corobeu(robotHandle = 'robot01')#, clientID = clientID)
#     crb1.acquisition(leftWheel = 'rodaEsquerda', rightWheel = 'rodaDireita')
#     plotPositions(crb1.positions)
#     crb1.Imitation(leftMotor = 'motorL01', rightMotor = 'motorR01')
    

