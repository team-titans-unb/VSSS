# -*- coding: utf-8 -*-
"""activation_functions.py.

This class crete a several activations functions to use in neural network topologies,
all the topologies are implemented in coppeliasim with a MARIA robot (two neuron perceptron).


Attributes:
        value_tanh : function Value to tanh
        value_sigomide : function Value to sigmoide
        value_linear : function value to linear

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
            MARIA PROJECT - Universidade de Brasília

    -Felipe Sidrone (colocar teus dados de contato)
    -Mateus TITAN (colocar teus dados de contato)

Release Date:
    Jul 26, 2023

"""

from math import e

class Activations_Functions:
    """Activations_Functions is a class to create several activation function to use
    in neural networks topologies in a MARIA robot

        Attributes:
            value_tanh (int): function tanh value
            value_sigmoide (float): function sigmoide value
            value_linear (float): function linear value
        """
    def __init__(self):
        self.value_tanh = 0
        self.value_sigmoide = 0
        self.value_linear = 0
    def exp(self, x):
        """exp is a function to calculate the exp value.

        Args:
            x (float): value to exp to calculate

        Returns:
            return the exp(x)

        """
        return e**x

    def sigmoid(self, s):
        """sigmoide is a function to calculate the sigmoide value.

            Args:
                s (float): value to calculate the sigmoide function

            Returns:
                return the sigmoide(s)

            """
        self.value_sigmoide = 1/(1+self.exp(-s))

    def tanh(self, t):
        """tanh is a function to calculate the tanh value.

        Args:
            t (float): value to calculate the tanh function

        Returns:
            return the tanh(t)

        """
        self.value_tanh = (self.exp(t)-self.exp(-t))/(self.exp(t)+self.exp(-t))

    def linear(self, x):
        """sigmoide is a function to calculate the linear value.

        Args:
            x (float): value to calculate the linear function

        Returns:
            return the linear(x)

        """
        if x < -1:
            self.value_linear = -1
        elif x > 1:
            self.value_linear = 1
        else:
            self.value_linear = x