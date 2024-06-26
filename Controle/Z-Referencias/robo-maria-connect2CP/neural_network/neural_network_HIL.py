# -*- coding: utf-8 -*-
"""neural_networks_HIL.py.

This class crete a several neural network topologies to test in HIL, all the topologies are
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

from fluctuating_point.float2bin import float2bin
from fluctuating_point.bin2float import bin2float

class Neural_Network_HIL:
    """Neural_network_HIL is a class to create several topologies to use
        in a MARIA robot with HIL


        Attributes:
            number_sample (int): number of sample used in teaching state
            output_neuron_sl (float): output neuron sl (speed sl)
            output_neuron_sr (float): output neuron sr (speed sr)
            output_neuron_referee (float): outpur neuron referee (range 0 - 1)

        """
    def __init__(self):
        self.number_sample = 100
        self.output_neuron_sl = 0
        self.output_neuron_sr = 0
        self.output_neuron_referee = 0
    def neural_network_canonical(self, x, inputs_value, outputs_value, select_state, i_imita, percent_training):
        """neural_network_canonical.

        Create a neural network canonical to work MARIA robot

        Args:
            select_state (int): is the state from LfD(Learning from demonstration), this value can be
            2 (training state) or 3 (imitation state)
            x (float): Is a list with PSO weigths in training state
            inputs_value (float): Are the inputs neural network value (generally distance)
            outputs_value (float) : Are the outputs neural network  value (generally speed)
            percent_training : Is the percento to training neural network

        Returns:

            Training : mean square error perceptron sl + perceptron sr
            Imitation : output perceptron sl and percepton sr

        """
        Exponente = 5
        Mantiza = 10
        sumEM = Exponente + Mantiza
        valuennR = 2
        valuennL = 2
        valuerecR = []
        valuerecL = []
        sumErrorR = 0
        sumErrorL = 0

        br = float2bin(Exponente, Mantiza, x[12])
        bl = float2bin(Exponente, Mantiza, x[13])
        w1r = float2bin(Exponente, Mantiza, x[0])
        w2r = float2bin(Exponente, Mantiza, x[1])
        w3r = float2bin(Exponente, Mantiza, x[2])
        w4r = float2bin(Exponente, Mantiza, x[3])
        w5r = float2bin(Exponente, Mantiza, x[4])
        w6r = float2bin(Exponente, Mantiza, x[5])
        w7r = float2bin(Exponente, Mantiza, x[11])
        w1l = float2bin(Exponente, Mantiza, x[6])
        w2l = float2bin(Exponente, Mantiza, x[7])
        w3l = float2bin(Exponente, Mantiza, x[8])
        w4l = float2bin(Exponente, Mantiza, x[9])
        w5l = float2bin(Exponente, Mantiza, x[10])
        w6l = float2bin(Exponente, Mantiza, x[14])
        w7l = float2bin(Exponente, Mantiza, x[15])

        bri = int(br, 2)
        w1ri = int(w1r, 2)
        w2ri = int(w2r, 2)
        w3ri = int(w3r, 2)
        w4ri = int(w4r, 2)
        w5ri = int(w5r, 2)
        w6ri = int(w6r, 2)
        w7ri = int(w7r, 2)
        bli = int(bl, 2)
        w1li = int(w1l, 2)
        w2li = int(w2l, 2)
        w3li = int(w3l, 2)
        w4li = int(w4l, 2)
        w5li = int(w5l, 2)
        w6li = int(w6l, 2)
        w7li = int(w7l, 2)
        # Comblock 0
        app(f'select_comblock 0')
        app(f'x_write_reg 9 {w1ri}')
        app(f'x_write_reg 10 {w2ri}')
        app(f'x_write_reg 11 {w3ri}')
        app(f'x_write_reg 12 {w4ri}')
        app(f'x_write_reg 13 {w5ri}')
        app(f'x_write_reg 14 {w6ri}')
        # Mudar para o Comblock 1
        app(f'select_comblock 1')
        app(f'x_write_reg 0 {bli}')
        app(f'x_write_reg 1 {w1li}')
        app(f'x_write_reg 2 {w2li}')
        app(f'x_write_reg 3 {w3li}')
        app(f'x_write_reg 4 {w4li}')
        app(f'x_write_reg 5 {w5li}')
        app(f'x_write_reg 6 {w6li}')
        app(f'x_write_reg 10 {w7li}')
        app(f'x_write_reg 8 {w7ri}')
        app(f'x_write_reg 7 {bri}')
        # Seleccionando o comblock 0
        app(f'select_comblock 0')
        #############################
        if select_state == 2:
            for i in range(round(self.number_sample*percent_training)):
                reseti = 1
                app(f'x_write_reg 0 {reseti}')
                reseti = 0
                app(f'x_write_reg 0 {reseti}')
                valueS1b = float2bin(Exponente, Mantiza, inputs_value[0][i])
                valueS2b = float2bin(Exponente, Mantiza, inputs_value[1][i])
                valueS3b = float2bin(Exponente, Mantiza, inputs_value[2][i])
                valueS4b = float2bin(Exponente, Mantiza, inputs_value[3][i])
                valueS5b = float2bin(Exponente, Mantiza, inputs_value[4][i])
                valueS6b = float2bin(Exponente, Mantiza, inputs_value[5][i])
                if i <= 0:
                    valueS7rb = float2bin(Exponente, Mantiza, 0)
                    valueS7lb = float2bin(Exponente, Mantiza, 0)
                    valueS7lb = int(valueS7lb, 2)
                    valueS7rb = int(valueS7rb, 2)
                else:
                    valueS7rb = valuennR
                    valueS7lb = valuennL

                valueS1i = int(valueS1b, 2)
                valueS2i = int(valueS2b, 2)
                valueS3i = int(valueS3b, 2)
                valueS4i = int(valueS4b, 2)
                valueS5i = int(valueS5b, 2)
                valueS6i = int(valueS6b, 2)
                app(f'x_write_reg 2 {valueS1i}')
                app(f'x_write_reg 3 {valueS2i}')
                app(f'x_write_reg 4 {valueS3i}')
                app(f'x_write_reg 5 {valueS4i}')
                app(f'x_write_reg 6 {valueS5i}')
                app(f'x_write_reg 7 {valueS6i}')
                app(f'x_write_reg 7 {valueS6i}')
                app(f'x_write_reg 15 {valueS7lb}')
                app(f'x_write_reg 8 {valueS7rb}')
                starti = 1
                app(f'x_write_reg 1 {starti}')
                # time.sleep(1)
                valuennR = app(f'x_read_reg 1')
                valuennL = app(f'x_read_reg 2')
                starti = 0
                app(f'x_write_reg 1 {starti}')
                valuennpbinR = (bin(valuennR.data[1])[2:])
                valuennpbinL = (bin(valuennL.data[1])[2:])
                while len(valuennpbinR) <= sumEM:
                    valuennpbinR = '0' + valuennpbinR
                while len(valuennpbinL) <= sumEM:
                    valuennpbinL = '0' + valuennpbinL
                valuerecR.append(bin2float(Exponente, Mantiza, valuennpbinR))
                valuerecL.append(bin2float(Exponente, Mantiza, valuennpbinL))
                # print("-----------------")
                sumErrorR = sumErrorR + ((valuerecR[i] - outputs_value[0][i]) * (valuerecR[i] - outputs_value[0][i]))
                sumErrorL = sumErrorL + ((valuerecL[i] - outputs_value[1][i]) * (valuerecL[i] - outputs_value[1][i]))

            MSER = sumErrorR / round(self.number_sample*percent_training)
            MSEL = sumErrorL / round(self.number_sample*percent_training)
            mse = MSER + MSEL
            # print("MSE")
            # print(mse)
            return mse

        elif select_state == 3:
            reseti = 1
            app(f'x_write_reg 0 {reseti}')
            reseti = 0
            app(f'x_write_reg 0 {reseti}')
            valueS1b = float2bin(Exponente, Mantiza, inputs_value[0])
            valueS2b = float2bin(Exponente, Mantiza, inputs_value[1])
            valueS3b = float2bin(Exponente, Mantiza, inputs_value[2])
            valueS4b = float2bin(Exponente, Mantiza, inputs_value[3])
            valueS5b = float2bin(Exponente, Mantiza, inputs_value[4])
            valueS6b = float2bin(Exponente, Mantiza, inputs_value[5])
            if i_imita <= 0:
                valueS7rb = float2bin(Exponente, Mantiza, 0)
                valueS7lb = float2bin(Exponente, Mantiza, 0)
                valueS7lb = int(valueS7lb, 2)
                valueS7rb = int(valueS7rb, 2)
            else:
                valueS7rb = self.output_neuron_sr
                valueS7lb = self.output_neuron_sl
                valueS7rb = float2bin(Exponente, Mantiza, valueS7rb)
                valueS7lb = float2bin(Exponente, Mantiza, valueS7lb)
                valueS7lb = int(valueS7lb, 2)
                valueS7rb = int(valueS7rb, 2)
        starti = 1
        valueS1i = int(valueS1b, 2)
        valueS2i = int(valueS2b, 2)
        valueS3i = int(valueS3b, 2)
        valueS4i = int(valueS4b, 2)
        valueS5i = int(valueS5b, 2)
        valueS6i = int(valueS6b, 2)
        app(f'x_write_reg 2 {valueS1i}')
        app(f'x_write_reg 3 {valueS2i}')
        app(f'x_write_reg 4 {valueS3i}')
        app(f'x_write_reg 5 {valueS4i}')
        app(f'x_write_reg 6 {valueS5i}')
        app(f'x_write_reg 7 {valueS6i}')
        app(f'x_write_reg 15 {valueS7lb}')
        app(f'x_write_reg 8 {valueS7rb}')
        starti = 1
        app(f'x_write_reg 1 {starti}')
        # time.sleep(1)
        valuennR = app(f'x_read_reg 1')
        valuennL = app(f'x_read_reg 2')
        starti = 0
        app(f'x_write_reg 1 {starti}')
        valuennpbinR = (bin(valuennR.data[1])[2:])
        valuennpbinL = (bin(valuennL.data[1])[2:])
        while len(valuennpbinR) <= sumEM:
            valuennpbinR = '0' + valuennpbinR
        while len(valuennpbinL) <= sumEM:
            valuennpbinL = '0' + valuennpbinL
        self.output_neuron_sr = bin2float(Exponente, Mantiza, valuennpbinR)
        self.output_neuron_sl = bin2float(Exponente, Mantiza, valuennpbinL)
        # print("-----------------")

