import CorobeuClass as crb
import Histogram_Neighbor as hist
import matplotlib.pyplot as plt
from DrawField import draw_field
from DrawField import plot_robot_path
import math
import sim
import csv

robot = 'robot01'
leftMotor = 'motorL01'
rightMotor = 'motorR01'
port = 19999
finalPos = [0.5, 0.5]

obstacleX = [0.5, 0.57]
obstacleY = [0.7, 0.77]

try:
    pp = hist.Histograma_Neighbor
    crb01 = crb.Corobeu(port, robot, leftMotor, rightMotor)
    xInit, yInit = crb01.Get_Position()
    xEnd = finalPos[0]
    yEnd = finalPos[1]
    pathX, pathY = pp.best_neighbor(pp, xInit, yInit, xEnd, yEnd, obstacleX, obstacleY, 'PSO')

    x = [pos for pos in pathX]
    y = [pos for pos in pathY]
    plt.plot(x, y, color='blue', marker='x')
    plt.grid(True)
    plt.xlim(-0.90, 0.9)
    plt.ylim(-0.7, 0.7)
    plt.show()

    for pos in range(len(pathX)):
        crb01.Follow_Path(pathX[pos], pathY[pos],[pathX[-1],pathY[-1]])

finally:
    plot_robot_path(pathX, pathY, crb01.xOut, crb01.yOut)
    # draw_field()
    # x = [pos for pos in pathX]
    # y = [pos for pos in pathY]
    # plt.plot(x, y, color='red', marker='o')
    # x = [pos for pos in crb01.xOut]
    # y = [pos for pos in crb01.yOut]
    # plt.plot(x, y, color='blue')
    # plt.grid(True)
    # plt.xlim(-0.90, 0.9)
    # plt.ylim(-0.7, 0.7)
    # plt.show()
#except KeyboardInterrupt: