# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# import ANNClass as ann

# class BioinspiredAlgorithms:
#     """This class implements several bioinspired algorithms to train neural networks in MARIA robots.

#     Attributes:
#         out_pso : list
#             Output PSO values
#         fit_vector : list
#             Best fitness value vector
#     """

#     def __init__(self):
#         self.out_pso = []
#         self.fit_vector = []

#     def PSO(self, inputs_value, outputs_value, S=15, N=26, maxIter=100, w0=0.9, wf=0.1, c1=2.05, c2=2.05, vMax=10, xMax=2, xMin=-2):
#         """PSO algorithm to find the best weights for training MARIA neural networks.

#         Args:
#             inputs_value (np.ndarray): Neural network input values (generally distance).
#             outputs_value (np.ndarray): Neural network output values (generally speed).
#             S (int): Number of particles.
#             N (int): Dimension of the problem.
#             maxIter (int): Maximum number of iterations.
#             w0 (float): Initial inertia weight.
#             wf (float): Final inertia weight.
#             c1 (float): Cognitive component coefficient.
#             c2 (float): Social component coefficient.
#             vMax (float): Maximum velocity.
#             xMax (float): Upper bound for positions.
#             xMin (float): Lower bound for positions.
#         """

#         # PSO Initializations
#         w, dw = w0, (wf - w0) / maxIter
#         x = xMin + (xMax - xMin) * np.random.rand(S, N)
#         y, v = 1e10 * np.ones_like(x), np.zeros_like(x)
#         fInd = 1e10 * np.ones(S)
#         k = 1

#         object_neural_network_sim = ann.ArtificialNeuralNetwork(nSamples=inputs_value.shape[1])

#         # PSO Main Loop
#         while k <= maxIter:
#             for i in range(S):
#                 fx = object_neural_network_sim.mse(inputs_value, x[i, :], outputs_value)
#                 if fx < fInd[i]:
#                     y[i, :] = x[i, :]
#                     fInd[i] = fx

#             # Find the best overall particle from the swarm
#             bestFit = np.min(fInd)
#             self.fit_vector.append(bestFit)
#             p = np.argmin(fInd)
#             self.out_pso = y[p, :]

#             # Particles' speed and position update
#             r1, r2 = np.random.rand(S, N), np.random.rand(S, N)
#             v = w * v + c1 * r1 * (y - x) + c2 * r2 * (self.out_pso - x)
#             v = np.clip(v, -vMax, vMax)
#             x += v
#             x = np.clip(x, xMin, xMax)

#             k += 1
#             w += dw

#         self._plot_results()
#         self._save_results()

#     def _plot_results(self):
#         """Plot the fitness vector."""
#         plt.plot(self.fit_vector)
#         plt.title("PSO")
#         plt.xlabel("Iteration")
#         plt.ylabel("Best Fitness")
#         plt.show()

#     def _save_results(self):
#         """Save the results to a CSV file."""
#         dframedata = [self.fit_vector[-1], self.out_pso]
#         index = ['Best fit', 'Weights and Biases']
#         dfd = pd.DataFrame(dframedata, index=index)
#         dfd.to_csv("PesosPSOC3.csv")

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import ANNClass as ann

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

    def PSO(self, nSample, inputs_value, outputs_value):

        """PSO.

            implement the PSO algorithm to find the best weights value in training MARIA neural networks

            Args:
                inputs_value (float): Are the inputs neural network value (generally distance)
                outputs_value (float) : Are the outputs neural network  value (generally speed)

            """
        S = 20
        # N = 47
        N = 12
        maxIter = 100
        w0 = 0.9
        wf = 0.1
        c1 = 2.05
        c2 = 2.05
        vMax = 10
        vIni = vMax / 10
        xMax = 20
        xMin = -20

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
                object_neural_network_sim = ann.ArtificialNeuralNetwork(nSample)
                fx = object_neural_network_sim.mse_slp(inputs_value, x[i, :] , outputs_value)
                print(fx)
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
        # dframedata = [self.fit_vector[-1],self.out_pso]
        # index = ['Best fit', 'Pesos Bias']
        # dfd = pd.DataFrame(dframedata, index=index)
        # dfd.to_csv("PesosPSO_23.csv")
        return self.fit_vector, self.out_pso
