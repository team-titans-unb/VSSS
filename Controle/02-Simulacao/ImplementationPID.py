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
finalPos = [0.5, 0]

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
        crb01.Follow_Path(pathX[pos+2], pathY[pos+2],[pathX[-1],pathY[-1]])

finally:
    crb01.Stop_bot()

    crb01.calculate_errors(pathX, pathY, 'c01_PID_N4000_5000.txt')
    
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