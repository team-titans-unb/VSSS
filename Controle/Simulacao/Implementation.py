import CorobeuClass as crb
import Histogram_Neighbor as hist
import matplotlib.pyplot as plt
import math
import sim
import csv

robot = 'robot01'
leftMotor = 'motorL01'
rightMotor = 'motorR01'
port = 19999
finalPos = [0, 0]

obstacleX = [0.5, 0.57]
obstacleY = [0.7, 0.77]

try:
    pp = hist.Histograma_Neighbor
    crb01 = crb.Corobeu(port, robot, leftMotor, rightMotor)
    xInit, yInit = crb01.Get_Position()
    xEnd = finalPos[0]
    yEnd = finalPos[1]
    #best_neighbor(x_init_aux, y_init_aux, x_end, y_end, square_value_x, square_value_y, AU):
    pathX, pathY = pp.best_neighbor(pp, xInit, yInit, xEnd, yEnd, obstacleX, obstacleY, 'PSO')

    x = [pos for pos in pathX]
    y = [pos for pos in pathY]
    plt.plot(x, y, color='blue', marker='x')
    plt.grid(True)
    plt.xlim(-0.90, 0.9)
    plt.ylim(-0.7, 0.7)
    plt.show()

    for pos in range(len(pathX)):
        #def Follow_Path(self, pathX, pathY, End_position):
        crb01.Follow_Path(pathX[pos], pathY[pos],[pathX[-1],pathY[-1]])

finally:
    x = [pos for pos in crb01.xOut]
    y = [pos for pos in crb01.yOut]
    # plt.scatter(x, y, color='blue', marker='x')
    # plt.grid(True)
    # plt.xlim(0, 1.7)
    # plt.ylim(0, 1.3)
    # plt.show()
#except KeyboardInterrupt: