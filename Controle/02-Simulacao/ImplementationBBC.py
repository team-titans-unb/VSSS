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
finalPos = [0, 0.6]

obstacleX = [0.5, 0.57]
obstacleY = [0.7, 0.77]

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

finally:
    crb01.Stop_bot()

    numPoints = len(crb01.xOut)
    originalIndices = np.arange(0, len(pathX))
    targetIndices = np.linspace(0, len(pathX)-1, numPoints)
    intpolX = np.interp(targetIndices, originalIndices, pathX)
    intpolY = np.interp(targetIndices, originalIndices, pathY)

    minLength = min(len(intpolX), len(crb01.xOut))
    intpolX = intpolX[:minLength]
    intpolY = intpolY[:minLength]
    xOut = crb01.xOut[:minLength]
    yOut = crb01.yOut[:minLength]
    squared_diff_x = [(px - ex) ** 2 for px, ex in zip(intpolX, xOut)]
    squared_diff_y = [(py - ey) ** 2 for py, ey in zip(intpolY, yOut)]
    mean_squared_diff = np.mean(squared_diff_x + squared_diff_y)
    rmse = np.sqrt(mean_squared_diff)
    print(rmse)

    finalPosX, finalPosY = crb01.Get_Position()
    absoluteError = np.sqrt(finalPos[0]**2 + finalPos[1]**2) - np.sqrt(finalPosX**2 + finalPosY**2)
    print(absoluteError)
    
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