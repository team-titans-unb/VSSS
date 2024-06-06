"""
Bioinspired.py

This code have a several bioisnpired algorithms

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
    EVA/MARIA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    MAY 25, 2023
"""
import math

import numpy as np
import matplotlib.pyplot as plt
import math as mt
from random import *
import EVAClass

class Bioinspired:

    def __init__(self):

        self.fitVector = []
        self.ys = []
        self.interror = 0
        self.fant = 0
        self.position_y = []
        self.position_x = []


    def PSO(self, S, N, maxIter, w0, wf, c1, c2, vMax, vIni, xMax, xMin, x_init, y_init, xend, yend, objects_x, objects_y):

        '''
            Unpack the configuration parameters from key arguments.
        '''
        fitVector = []

        '''
            PSO Initializations
        '''
        w, dw = w0, (wf - w0) / maxIter
        x = xMin + (xMax - xMin) * np.random.rand(S, N)
        y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
        fInd, k = 1e10 * np.ones(S), 1
        bestFit = 1000
        self.position_y.append(y_init)
        self.position_x.append(x_init)
        lock_G4 = 0
        '''
            PSO Main Loop
        '''
        print("PSO")

        while bestFit >= 10.1:

            # print(k)
            '''
                Loop to find the best individual particle
            '''
            for i in range(S):

                xnew = x_init + x[i, 0]
                ynew = y_init + x[i, 1]

                for i in range(len(objects_x)):
                    if ((xnew >= min(objects_x[i]) and xnew <= max(objects_x[i])) and (
                            ynew >= min(objects_y[i]) and ynew <= max(objects_y[i]))):
                        G = 1000
                        lock_G4 = 1
                    else:
                        if lock_G4 == 0:
                            G = 10
                        else:
                            pass

                lock_G4 = 0
                H_new = math.sqrt((xnew - xend) ** 2 + (ynew - yend) ** 2)

                fx = G + H_new

                #print(f'Error == {fx}')

                # print(fx)
                # print("___________________")
                if fx < fInd[i]:
                    y[i, :] = x[i, :]
                    x_init = xnew
                    y_init = ynew
                    print(f'x_init == {x_init}')
                    print(f'y_init == {y_init}')
                    print(f'Error == {fx}')
                    self.position_y.append(y_init)
                    self.position_x.append(x_init)
                    fInd[i] = fx

            '''
                Find the best overall particle from the swarm
            '''
            bestFit = min(fInd)
            self.fitVector.append(bestFit)
            p = np.where(fInd == bestFit)[0][0]
            self.ys = y[p, :]
            # print(self.ys)
            # print(bestFit)

            '''
                Particles' speed update using inertia factor.
            '''
            for j in range(N):
                for i in range(S):
                    u1, u2 = np.random.rand(), np.random.rand()
                    v[i, j] = w * v[i, j] + c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.ys[j] - x[i, j])
                    x[i, j] += v[i, j]

                    if x[i, j] > xMax: x[i, j] = xMax - np.random.rand() * (xMax - xMin)
                    if x[i, j] < xMin: x[i, j] = xMin + np.random.rand() * (xMax - xMin)
                    if v[i, j] > vMax: v[i, j] = vMax - np.random.rand() * (vMax - vIni)
                    if v[i, j] < 0: v[i, j] = vIni

            k += 1
            w += dw

        return self.position_x, self.position_y

if __name__ == "__main__":
    ctf = Bioinspired()
    S = 14
    N = 2
    maxIter = 350
    w0 = 0.9
    wf = 0.1
    c1 = 2.05
    c2 = 2.05
    vMax = 1
    vIni = vMax / 10
    xMax = 0.5
    xMin = -0.5
    x_init = 1
    y_init = 1
    xend = 5
    yend = 4
    objects_x = [[2, 3], [5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    objects_y = [[2, 3], [0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    x_position, y_position = ctf.PSO(S, N, maxIter, w0, wf, c1, c2, vMax, vIni, xMax, xMin, x_init, y_init, xend, yend, objects_x, objects_y)
    # Coordenadas del cuadrado
    object_x1 = [2, 3, 3, 2, 2]
    object_y1 = [2, 2, 3, 3, 2]

    object_x2 = [4, 5, 5, 4, 4]
    object_y2 = [1, 1, 4, 4, 1]

    object_x3 = [5.5, 6, 6, 5.5, 5.5]
    object_y3 = [0, 0, 5.5, 5.5, 0]

    object_x4 = [0.5, 6, 6, 0.5, 0.5]
    object_y4 = [5, 5, 5.5, 5.5, 5]

    object_x5 = [0, 0.5, 0.5, 0, 0]
    object_y5 = [0, 0, 5.5, 5.5, 0]

    object_x6 = [0, 5.5, 5.5, 0, 0]
    object_y6 = [0, 0, 0.5, 0.5, 0]

    # fig = plt.figure()
    # fig.clf()
    # ax = fig.subplots(1, 1)

    plt.plot(object_x1, object_y1, label='Object_1', color='blue')
    # plt.plot(object_x2, object_y2, label='Object_2', color='blue')
    plt.plot(object_x3, object_y3, label='Wall_right', color='red')
    plt.plot(object_x4, object_y4, label='Wall_up', color='red')
    plt.plot(object_x5, object_y5, label='Wall_left', color='red')
    plt.plot(object_x6, object_y6, label='Wall_down', color='red')
    plt.plot(x_position, y_position, color='black')
    plt.grid(True)
    plt.show()

