import numpy as np
from scipy.special import expit

class ArtificialNeuralNetwork:
    
    def __init__(self, nSamples=50, vmax=8):
        self.outputVr = 0
        self.outputVl = 0
        self.nSamples = nSamples
        self.vmax = vmax

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
        # Reshape weights and biases for two inputs per neuron
        fstW = [weights[i:i + 2] for i in range(0, 8, 2)]
        fstB = biases[:4]
        sndW = [weights[i:i + 2] for i in range(8, 14, 2)]
        sndB = biases[4:7]
        trdW = [weights[i:i + 2] for i in range(14, 18, 2)]
        trdB = biases[7:9]

        fstLayer = [0] * 4
        sndLayer = [0] * 3
        speeds = [0] * 2

        # First layer
        for i in range(4):
            fstLayer[i] = self.neuron(inputs[:2], fstW[i], fstB[i])

        # Second layer
        for i in range(3):
            sndLayer[i] = self.neuron(fstLayer[:2], sndW[i], sndB[i])

        # Output layer
        for i in range(2):
            speeds[i] = self.neuron(sndLayer[:2], trdW[i], trdB[i])

        return speeds
    
    def mlp22(self, inputs, weights, biases):
        # Reshape weights and biases for two inputs per neuron
        # Considering a simple architecture with one hidden layer
        # 2 inputs -> 2 neurons in the hidden layer -> 2 outputs
        
        # First layer weights and biases (2 neurons, 2 inputs each)
        fstW = [weights[i:i + 2] for i in range(0, 4, 2)]
        fstB = biases[:2]
        
        # Output layer weights and biases (2 neurons, 2 inputs each)
        sndW = [weights[i:i + 2] for i in range(4, 8, 2)]
        sndB = biases[2:4]

        fstLayer = [0] * 2
        speeds = [0] * 2

        # First layer
        for i in range(2):
            fstLayer[i] = self.neuron(inputs, fstW[i], fstB[i])

        # Output layer
        for i in range(2):
            speeds[i] = self.neuron(fstLayer, sndW[i], sndB[i])

        return speeds
    
    def mse_mlp22(self, inputs, weights_biases, desired):
        errorL = 0
        errorR = 0
        
        # A nova rede tem 8 pesos (2 entradas * 2 neurônios na camada oculta + 2 entradas * 2 neurônios na camada de saída)
        # e 4 biases (2 para a camada oculta + 2 para a camada de saída)
        weights = weights_biases[:8]
        biases = weights_biases[8:]

        for i in range(self.nSamples):
            input_vector = []
            for j in range(2):
                input_vector.append(inputs[j][i])
            
            wspeed = self.mlp22(input_vector, weights, biases)
            
            errorL += (wspeed[0] - desired[0][i])**2
            errorR += (wspeed[1] - desired[1][i])**2

        MSEL = errorL / self.nSamples
        MSER = errorR / self.nSamples
        mse = MSEL + MSER
        return mse

    def mse(self, inputs, weights_biases, desired):
        errorL = 0
        errorR = 0
        weights = weights_biases[:18]
        biases = weights_biases[18:]
        for i in range(self.nSamples):
            input_vector = []
            for j in range(2):
                input_vector.append(inputs[j][i])
            wspeed = self.mlp432(input_vector, weights, biases)
            errorL += (wspeed[0] - desired[0][i])**2
            errorR += (wspeed[1] - desired[1][i])**2
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