import numpy as np
from scipy.special import expit

class ArtificialNeuralNetwork:
    
    def __init__(self, nSamples):
        self.outputVr = 0
        self.outputVl = 0
        self.nSamples = nSamples

    def sigmoid(self, x):
        return expit(x)
    
    def neuron(self, inputs, weights, bias):
        # sumation = 0
        # for i in range(len(weights)):
        #     sumation += inputs[i] * weights[i]
        # sumation += bias
        sumation = (inputs * weights) + bias
        return self.sigmoid(sumation)
    
    def mlp432(self, inputs, weights, biases):
        fstW = [weights[i:i + 5] for i in range(0, 20, 5)]
        fstB = biases[:4]
        sndW = [weights[i:i + 4] for i in range(20, 32, 4)]
        sndB = biases[4:7]
        trdW = [weights[i:i + 3] for i in range(32, 38, 3)]
        trdB = biases[7:9]

        fstLayer = [0] * 4
        sndLayer = [0] * 3
        speeds = [0] * 2
        # First layer
        for i in range(4):
            fstLayer[i] = self.neuron(inputs, fstW[i], fstB[i])

        for i in range(3):
            sndLayer[i] = self.neuron(fstLayer, sndW[i], sndB[i])

        for i in range(2):
            speeds[i] = self.neuron(sndLayer, trdW[i], trdB[i])

        return speeds
    
    def mse(self, input, weights_biases, desired):
        errorL = 0
        errorR = 0
        weights = weights_biases[:38]
        biases = weights_biases[38:]
        for i in range(self.nSamples):
            input_vector = []
            for j in range(5):
                input_vector.append(input[j][i])
            wspeed = self.mlp432(input_vector, weights, biases)
            errorL = (wspeed[0] - desired[0][i])**2 + errorL
            errorR = (wspeed[1] - desired[1][i])**2 + errorR
        MSEL = errorL / self.nSamples
        MSER = errorR / self.nSamples
        mse = MSEL + MSER
        return mse
    
    def mse_slp(self, inputs, weights_biases, desired):
        errorL = 0
        errorR = 0
        # Extrair pesos e bias para as duas SLPs
        weightsL = weights_biases[0]
        biasL = weights_biases[1]
        weightsR = weights_biases[2]
        biasR = weights_biases[3]
        
        for i in range(self.nSamples):
            input_vector = []
            # for j in range(len(inputs)):
                # input_vector.append(inputs[j][i])
            input_vector.append(inputs[i])
            wspeedL = self.neuron(input_vector[0], weightsL, biasL)
            wspeedR = self.neuron(input_vector[0], weightsR, biasR)
            errorL += (wspeedL - desired[0][i])**2
            errorR += (wspeedR - desired[1][i])**2
        MSEL = errorL / self.nSamples
        MSER = errorR / self.nSamples
        mse = (MSEL + MSER) * 1000
        return mse