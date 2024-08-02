import CorobeuClass as crb
import Histogram_Neighbor as hist
import matplotlib.pyplot as plt
from DrawField import draw_field
from DrawField import plot_robot_path
import math
import numpy as np
import sim
import csv

robot = 'robot01'
leftMotor = 'motorL01'
rightMotor = 'motorR01'
port = 19999
finalPos = [0.6, -0.3]

obstacleX = [-1.2, 1.2]
obstacleY = [-1.2, 1.2]

try:
    pp = hist.Histograma_Neighbor
    crb01 = crb.Corobeu(port, robot, leftMotor, rightMotor)
    xInit, yInit = crb01.Get_Position()
    xEnd = finalPos[0]
    yEnd = finalPos[1]
    print("Definindo trajetoria...")
    pathX, pathY = pp.best_neighbor(pp, xInit, yInit, xEnd, yEnd, obstacleX, obstacleY, 'PSO')

    x = [pos for pos in pathX]
    y = [pos for pos in pathY]
    plt.plot(x, y, color='blue', marker='x')
    plt.grid(True)
    plt.xlim(-0.90, 0.9)
    plt.ylim(-0.7, 0.7)
    plt.show()

    print("Seguindo trajetoria...")
    for pos in range(len(pathX)):
        crb01.Micro_Behaviors(pathX[pos], pathY[pos],[pathX[-1],pathY[-1]])
    # crb01.Micro_Behaviors(finalPos[0],finalPos[1],finalPos)

finally:
    crb01.Stop_bot()

    crb01.calculate_errors(pathX, pathY, 'c01_00N23_0023.txt')

    plot_robot_path(pathX, pathY, crb01.xOut, crb01.yOut)