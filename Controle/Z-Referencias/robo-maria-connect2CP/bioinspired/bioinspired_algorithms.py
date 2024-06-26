"""bioinspired_algorithms.py.

This class crete a several bioinspired algorithms to training neural network in
MARIA robots


Attributes:
        out_pso : out PSO values
        fit_vector : best fitness value vector

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
            MARIA PROJECT - Universidade de Brasília

    -Felipe Sidrone (colocar teus dados de contato)
    -Mateus TITAN (colocar teus dados de contato)

Release Date:
    Jul 26, 2023

"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from neural_network.neural_network_sim import *
class Bioinspired_algorithms:
    """This class crete a several bioinspired algorithms to training neural network in
        MARIA robots

        Attributes:
            out_pso : out PSO values
            fit_vector : best fitness value vector
        """

    def __init__(self):
        self.out_pso = []
        self.fit_vector = []

    def PSO(self, inputs_value, outputs_value):

        """PSO.

            implement the PSO algorithm to find the best weights value in training MARIA neural networks

            Args:
                inputs_value (float): Are the inputs neural network value (generally distance)
                outputs_value (float) : Are the outputs neural network  value (generally speed)

            """
        S = 15
        N = 26
        maxIter = 100
        w0 = 0.9
        wf = 0.1
        c1 = 2.05
        c2 = 2.05
        vMax = 10
        vIni = vMax / 10
        xMax = 2
        xMin = -2

        #    PSO Initializations

        w, dw = w0, (wf - w0) / maxIter
        x = xMin + (xMax - xMin) * np.random.rand(S, N)
        y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
        fInd, k = 1e10 * np.ones(S), 1


        #   PSO Main Loop

        print("PSO")
        while k <= maxIter:
            print(k)

            #    Loop to find the best individual particle

            for i in range(S):
                object_neural_network_sim = Neural_network_sim()
                fx = object_neural_network_sim.neural_network_input_realimentation(x[i, :], inputs_value, outputs_value, 2, 0)
                # print(fx)
                # print("___________________")
                if fx < fInd[i]:
                    y[i, :] = x[i, :]
                    fInd[i] = fx


            #    Find the best overall particle from the swarm
            bestFit = min(fInd)
            self.fit_vector.append(bestFit)
            p = np.where(fInd == bestFit)[0][0]
            self.out_pso = y[p, :]
            # print(self.ys)
            # print(bestFit)


            #   Particles' speed update using inertia factor.

            for j in range(N):
                for i in range(S):
                    u1, u2 = np.random.rand(), np.random.rand()
                    v[i, j] = w * v[i, j] + c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.out_pso[j] - x[i, j])
                    v[i, j] = min(vMax, max(-vMax, v[i, j]))

                    x[i, j] += v[i, j]
                    if x[i, j] > xMax: x[i, j] = xMax - np.random.rand()
                    if x[i, j] < xMin: x[i, j] = xMin + np.random.rand()

            k += 1
            w += dw
        print("--------ys-------")
        print(self.out_pso)
        print(self.fit_vector)
        print("--------ys-------")
        plt.plot(self.fit_vector)
        plt.title("PSO")
        plt.show()
        dframedata = [self.fit_vector[-1],self.out_pso]
        index = ['Best fit', 'Pesos Bias']
        dfd = pd.DataFrame(dframedata, index=index)
        dfd.to_csv("PesosPSOC3.csv")