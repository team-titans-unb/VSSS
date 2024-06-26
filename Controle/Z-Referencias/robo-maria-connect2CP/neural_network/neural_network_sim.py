# -*- coding: utf-8 -*-
"""neural_networks_sim.py.

This class crete a several neural network topologies to test in simultion, all the topologies are
implemented in coppeliasim with a MARIA robot (two neuron perceptron).


Attributes:
        number_sample : Is the sample number to simulate in coppeliasim
        output_neuron_sl : Is the output neuron sl
        output_neuron_sr : Is the output neuron sr
        output_neuron_referee : Is the output neuron perceptron

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
            MARIA PROJECT - Universidade de Brasília

    -Felipe Sidrone (colocar teus dados de contato)
    -Mateus TITAN (colocar teus dados de contato)

Release Date:
    Jul 26, 2023

"""


from neural_network.activation_functions import *

class Neural_network_sim:

    """Neural_network_sim is a class to create several topologies to use
    in a MARIA robot


    Attributes:
        number_sample (int): number of sample used in teaching state
        output_neuron_sl (float): output neuron sl (speed sl)
        output_neuron_sr (float): output neuron sr (speed sr)
        output_neuron_referee (float): outpur neuron referee (range 0 - 1)

    """
    def __init__(self):
        self.number_sample = 10
        self.output_neuron_sl = 0
        self.output_neuron_sr = 0
        self.output_neuron_referee = 0

    def neural_network_input_realimentation(self, x, rsensor, rspeed, ELFD, weights_imitation):
        """Example function with types documented in the docstring.

        `PEP 484`_ type annotations are supported. If attribute, parameter, and
        return types are annotated according to `PEP 484`_, they do not need to be
        included in the docstring:

        Args:
            param1 (int): The first parameter.
            param2 (str): The second parameter.

        Returns:
            bool: The return value. True for success, False otherwise.

        .. _PEP 484:
            https://www.python.org/dev/peps/pep-0484/

        """
        sumErrorR = 0
        sumErrorL = 0
        wr = []
        wl = []
        ntSr = 0
        ntSl = 0
        Sr = []
        Sl = []
        nnSr = 0
        nnSl = 0
        object_funcition_activation = Activations_Functions()
        if ELFD == 2:

            for i in range(12):
                wr.append(x[i])
                wl.append(x[i + 12])
            br = x[24]
            bl = x[25]

            for j in range(self.number_sample - 6):

                ntSr = 0
                ntSl = 0

                for i in range(12):
                    ntSr = ntSr + wr[i] * rsensor[i][j]
                    ntSl = ntSl + wl[i] * rsensor[i][j]

                object_funcition_activation.sigmoid(ntSr + br)
                Sr = object_funcition_activation.value_sigmoide
                object_funcition_activation.sigmoid(ntSl + bl)
                Sl = object_funcition_activation.value_sigmoide

                sumErrorR = (Sr - rspeed[0][j]) ** 2 + sumErrorR
                sumErrorL = (Sl - rspeed[1][j]) ** 2 + sumErrorL

            MSER = sumErrorR / self.number_sample
            MSEL = sumErrorL / self.number_sample
            mse = MSER + MSEL
            # print("MSE")
            # print(mse)
            return mse

        elif ELFD == 3:
            for i in range(12):
                nnSr = nnSr + weights_imitation[i] * rsensor[i]
                nnSl = nnSl + weights_imitation[i + 12] * rsensor[i]
            object_funcition_activation.sigmoid(nnSr + weights_imitation[24])
            self.output_neuron_sr = object_funcition_activation.value_sigmoide
            object_funcition_activation.sigmoid(nnSl + weights_imitation[25])
            self.output_neuron_sl = object_funcition_activation.value_sigmoide


    def neural_network_canonical(self, x, inputs_value, outputs_value, select_state, weights_imitation, percent_training = 100):
        """neural_network_canonical.

        Create a neural network canonical to work MARIA robot

        Args:
            select_state (int): is the state from LfD(Learning from demonstration), this value can be
            2 (training state) or 3 (imitation state)
            x (float): Is a list with PSO weigths in training state
            inputs_value (float): Are the inputs neural network value (generally distance)
            outputs_value (float) : Are the outputs neural network  value (generally speed)
            weights_imitation (float) : Are the weight to imitate state
            percent_training : Is the percento to training neural network

        Returns:

            Training : mean square error perceptron sl + perceptron sr
            Imitation : output perceptron sl and percepton sr

        """

        sum_error_perceptron_sr = 0
        sum_error_perceptron_sl = 0
        weight_neuron_perceptron_sr = []
        weight_neuron_perceptron_sl = []
        sum_perceptron_sr = 0
        sum_perceptron_sl = 0
        object_activation_function = Activations_Functions


        if select_state == 2:

            for i in range(len(inputs_value)):
                weight_neuron_perceptron_sr.append(x[i])
                weight_neuron_perceptron_sl.append(x[i + len(inputs_value)])
            bias_neuron_sr = x[-2]
            bias_neuron_sl = x[-1]

            for j in range(round(self.numero_muestras*percent_training)):

                sum_perceptron_sr = 0
                sum_perceptron_sl = 0

                for i in range(len(inputs_value)):
                    sum_perceptron_sr = sum_perceptron_sr + weight_neuron_perceptron_sr[i] * inputs_value[i][j]
                    sum_perceptron_sl = sum_perceptron_sl + weight_neuron_perceptron_sl[i] * inputs_value[i][j]

                object_activation_function.sigmoid(sum_perceptron_sr + bias_neuron_sr)
                sum_perceptron_sr_sigmoid = object_activation_function.value_sigmoide
                object_activation_function.sigmoid(sum_perceptron_sr + bias_neuron_sl)
                sum_perceptron_sl_sigmoid = object_activation_function.value_sigmoide

                sum_error_perceptron_sr = (sum_perceptron_sr_sigmoid - outputs_value[0][j]) ** 2 + sum_error_perceptron_sr
                sum_error_perceptron_sl = (sum_perceptron_sl_sigmoid - outputs_value[1][j]) ** 2 + sum_error_perceptron_sl

            mean_square_error_perceptron_sr = sum_error_perceptron_sr / round(self.numero_muestras*percent_training)
            mean_square_error_perceptron_sl = sum_error_perceptron_sl / round(self.numero_muestras*percent_training)
            mean_square_error = mean_square_error_perceptron_sr + mean_square_error_perceptron_sl
            return mean_square_error

        elif select_state == 3:

            for i in range(len(inputs_value)):
                sum_perceptron_sr = sum_perceptron_sr + weights_imitation[i] * inputs_value[i]
                sum_perceptron_sl = sum_perceptron_sl + weights_imitation[i + len(inputs_value)] * inputs_value[i]

            object_activation_function.sigmoid(sum_perceptron_sr + weights_imitation[-2])
            self.output_neuron_sr = object_activation_function.value_sigmoide
            object_activation_function.sigmoid(sum_perceptron_sl + weights_imitation[-1])
            self.output_neuron_sl = object_activation_function.value_sigmoide
    def neural_network_refeere_canonical(self, x, inputs_value, outputs_value, select_state, weights_imitation, percent_training):
        """neural_network_canonical.

        Create a neural network canonical to work MARIA robot

        Args:
            select_state (int): is the state from LfD(Learning from demonstration), this value can be
            2 (training state) or 3 (imitation state)
            x (float): Is a list with PSO weigths in training state
            inputs_value (float): Are the inputs neural network value (generally distance)
            outputs_value (float) : Are the outputs neural network  value (generally speed)
            weights_imitation (float) : Are the weight to imitate state
            percent_training (float) : Is the percent of training

        Returns:

            Training : mean square error perceptron sl + perceptron sr
            Imitation : output perceptron sl and percepton sr

        """

        sum_error_perceptron_referee = 0
        weight_neuron_perceptron_referee = []
        sum_perceptron_referee = 0
        object_activation_function = Activations_Functions


        if select_state == 2:

            for i in range(len(inputs_value)):
                weight_neuron_perceptron_referee.append(x[i])

            bias_neuron_referee = x[-1]

            for j in range(round(self.numero_muestras*percent_training)):

                sum_perceptron_referee = 0

                for i in range(len(inputs_value)):
                    sum_perceptron_referee = sum_perceptron_referee + weight_neuron_perceptron_referee[i] * inputs_value[i][j]

                object_activation_function.sigmoid(sum_perceptron_referee + bias_neuron_referee)
                sum_perceptron_referee_sigmoid = object_activation_function.value_sigmoide

                sum_error_perceptron_referee = (sum_perceptron_referee_sigmoid - outputs_value[0][j]) ** 2 + sum_error_perceptron_referee

            mean_square_error_perceptron_referee = sum_error_perceptron_referee/round(self.number_sample*percent_training)
            mean_square_error = mean_square_error_perceptron_referee
            return mean_square_error

        elif select_state == 3:

            for i in range(len(inputs_value)):
                sum_perceptron_referee = sum_perceptron_referee + weights_imitation[i] * inputs_value[i]

            object_activation_function.sigmoid(sum_perceptron_referee + weights_imitation[-1])
            self.output_neuron_referee = object_activation_function.value_sigmoide

