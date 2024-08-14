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
finalPos = [0, 0.5]
cen = 'c02'
filename = f'{cen}_BBC_{finalPos[0]}_{finalPos[1]}'

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
    for pos in range(len(pathX)-2):
        crb01.Micro_Behaviors(pathX[pos+2], pathY[pos+2],[pathX[-1],pathY[-1]])
    # crb01.Micro_Behaviors(finalPos[0], finalPos[1], [pathX[-1],pathY[-1]])

finally:
    crb01.Stop_bot()

    crb01.calculate_errors(pathX, pathY, filename)

    plot_robot_path(pathX, pathY, crb01.xOut, crb01.yOut, filename)