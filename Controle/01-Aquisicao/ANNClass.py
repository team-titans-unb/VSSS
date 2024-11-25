import numpy as np
from scipy.special import expit
from scipy.stats import burr12


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
        input_vector = []
        for i in range(self.nSamples):

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

class Referee:

    def __init__(self, nSamples=50, behavior1=[0,0,0,0], behavior2=[0,0,0,0], behavior3=[0,0,0,0]):
        self.nSamples = nSamples
        self.behaviorWeights = {
            0: behavior1,
            1: behavior2,
            2: behavior3
        }

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))

    def neuron(self, inputs, weights, bias):
        sumation = (inputs * weights) + bias
        return self.sigmoid(sumation)

    def topology1(self, inputs, weights, biases):
        w1 = weights[0]
        b1 = biases[0]
        w2 = weights[1]
        b2 = biases[1]
        w3 = weights[2:5]
        b3 = biases[2:5]
        outlayer = [0] * 3

        fstlayer = self.neuron(inputs, w1, b1)
        sndlayer = self.neuron(fstlayer, w2, b2)

        for i in range(3):
            outlayer[i] = self.neuron(sndlayer, w3[i], b3[i])

        return outlayer


    def mse_top1(self, inputs, weights_biases, desired_outputs):
        weights = weights_biases[:5]  # First 5 elements are weights (for 3 layers)
        biases = weights_biases[5:]  # Remaining elements are biases

        mse_total = 0

        for i in range(len(inputs)):
            actual_output = self.topology1(inputs[i], weights, biases)  # Get the actual output from topology1
            target_output = [0, 0, 0]  # Initialize target output

            # Set the target output based on the desired output (1, 2, or 3)
            if desired_outputs[i] == 1:
                target_output = [1, 0, 0]
            elif desired_outputs[i] == 2:
                target_output = [0, 1, 0]
            elif desired_outputs[i] == 3:
                target_output = [0, 0, 1]

            # Calculate MSE for the current input using the provided logic
            mse_total += (target_output[0] - actual_output[0]) ** 2
            mse_total += (target_output[1] - actual_output[1]) ** 2
            mse_total += (target_output[2] - actual_output[2]) ** 2

        # Average the MSE over all samples
        mse_total /= len(inputs)

        return mse_total

    def select_behavior(self, ann_outputs):
        behavior_index = np.argmax(ann_outputs)
        return self.behaviorWeights[behavior_index], behavior_index

