"""
AlgorithmA-start.py

This code implement A* algorithm using CoppeliaSim

Author:
    Mario Pastrana (mariopastrana403@gmail.com)
    EVA/MARIA PROJECT - Universidade de Brasília

Version:
    0.0.1 (beta)

Release Date:
    Dez 03, 2023
"""
import math
import matplotlib.pyplot as plt
import numpy as np

class Trajectory_Generation():

    def __init__(self):
        self.xtrajectory = []
        self.xtrajectory_send = []
        self.ytrajectory = []
        self.ytrajectory_send = []
        self.GlobalMin = []
        self.delta_space_init = 0.5
        self.delta_space_finish = 0.5
        self.delta_space = 0
    def AStar(self, x_init, y_init, xend, yend, objects_x, objects_y):

        H = math.sqrt((x_init-xend)**2 + (y_init-yend)**2)
        max_iter = 200
        x0_old = []
        y0_old = []
        xnew = [1, 2, 3, 4, 5, 6, 7]
        ynew= [1, 2, 3, 4, 5, 6, 7]
        G = [1, 2, 3, 4, 5, 6, 7]
        F = [1, 2, 3, 4, 5, 6, 7]
        H_new = [1, 2, 3, 4, 5, 6, 7]
        iteration = 0
        min_f_value = []
        lock_G0 = 0
        lock_G1 = 0
        lock_G2 = 0
        lock_G3 = 0
        lock_G4 = 0
        lock_G5 = 0
        lock_G6 = 0
        lock_it0 = 0
        min_f_value_ant = 10000
        H_ant = 1000
        count_no_best = 0
        count_pos_init = 0
        w = (self.delta_space_finish - self.delta_space_init) / max_iter
        self.delta_space = self.delta_space_init
        x0 = x_init
        y0 = y_init
        primer_iteracion = True

        """""
        Comienzo del bucle principal, sí el valor de la distancia euclidiana es mayor a 0.1 es ejecutado.
        """""
        while H >= 0.1:

            self.xtrajectory.append(x0)
            self.ytrajectory.append(y0)

            ############### posibilidad ++ ##########
            xnew[0] = x0 + self.delta_space
            ynew[0] = y0 + self.delta_space
            """""
            Con el siguiente for se observa si la trayectoria entra o no a un objeto
            """""
            for i in range(len(objects_x)):
                if ((xnew[0] >= min(objects_x[i]) and xnew[0] <= max(objects_x[i])) and  (ynew[0] >= min(objects_y[i]) and ynew[0] <= max(objects_y[i]))):
                    G[0] = 1000
                    lock_G0 = 1
                else:
                    if lock_G0 == 0:
                        G[0] = 10
                    else:
                        pass
            lock_G0 = 0
            H_new[0] = math.sqrt((xnew[0]-xend)**2 + (ynew[0]-yend)**2)
            F[0] = G[0] + H_new[0]

            ############### posiblidad 0+ ##########
            xnew[1] = x0
            ynew[1] = y0 + self.delta_space
            for i in range(len(objects_x)):
                if ((xnew[1]>= min(objects_x[i]) and xnew[1] <= max(objects_x[i])) and (ynew[1]>= min(objects_y[i]) and ynew[1] <= max(objects_y[i]))):
                    G[1] = 1000
                    lock_G1 = 1
                else:
                    if lock_G1 == 0:
                        G[1] = 10
                    else:
                        pass

            lock_G1 = 0
            H_new[1] = math.sqrt((xnew[1] - xend) ** 2 + (ynew[1] - yend) ** 2)
            F[1] = G[1] + H_new[1]

            ############### posibilidad +0 ##########
            xnew[2] = x0 + self.delta_space
            ynew[2] = y0

            for i in range(len(objects_x)):
                if ((xnew[2]>= min(objects_x[i]) and xnew[2] <= max(objects_x[i])) and (ynew[2]>= min(objects_y[i]) and ynew[2] <= max(objects_y[i]))):
                    G[2] = 1000
                    lock_G2 = 1
                else:
                    if lock_G2 == 0:
                        G[2] = 10
                    else:
                        pass

            lock_G2 = 0
            H_new[2] = math.sqrt((xnew[2] - xend) ** 2 + (ynew[2] - yend) ** 2)
            F[2] = G[2] + H_new[2]

            ############### posibilidad +-##########
            xnew[3] = x0 + self.delta_space
            ynew[3] = y0 - self.delta_space

            for i in range(len(objects_x)):
                if ((xnew[3] >= min(objects_x[i]) and xnew[3] <= max(objects_x[i])) and (
                        ynew[3] >= min(objects_y[i]) and ynew[3] <= max(objects_y[i]))):
                    G[3] = 1000
                    lock_G3 = 1
                else:
                    if lock_G3 == 0:
                        G[3] = 10
                    else:
                        pass

            lock_G3 = 0
            H_new[3] = math.sqrt((xnew[3] - xend) ** 2 + (ynew[3] - yend) ** 2)
            F[3] = G[3] + H_new[3]

            ############### posibilidad -+##########
            xnew[4] = x0 - self.delta_space
            ynew[4] = y0 + self.delta_space

            for i in range(len(objects_x)):
                if ((xnew[4] >= min(objects_x[i]) and xnew[4] <= max(objects_x[i])) and (
                        ynew[4] >= min(objects_y[i]) and ynew[4] <= max(objects_y[i]))):
                    G[4] = 1000
                    lock_G4 = 1
                else:
                    if lock_G4 == 0:
                        G[4] = 10
                    else:
                        pass

            lock_G4 = 0
            H_new[4] = math.sqrt((xnew[4] - xend) ** 2 + (ynew[4] - yend) ** 2)
            F[4] = G[4] + H_new[4]

            ############### posibilidad -0##########
            xnew[5] = x0 - self.delta_space
            ynew[5] = y0

            for i in range(len(objects_x)):
                if ((xnew[5] >= min(objects_x[i]) and xnew[5] <= max(objects_x[i])) and (
                        ynew[5] >= min(objects_y[i]) and ynew[5] <= max(objects_y[i]))):
                    G[5] = 1000
                    lock_G5 = 1
                else:
                    if lock_G5 == 0:
                        G[5] = 10
                    else:
                        pass

            lock_G5 = 0
            H_new[5] = math.sqrt((xnew[5] - xend) ** 2 + (ynew[5] - yend) ** 2)
            F[5] = G[5] + H_new[5]

            ############### posibilidad -- ##########
            xnew[6] = x0 - self.delta_space
            ynew[6] = y0 - self.delta_space

            for i in range(len(objects_x)):
                if ((xnew[6] >= min(objects_x[i]) and xnew[6] <= max(objects_x[i])) and (
                        ynew[6] >= min(objects_y[i]) and ynew[6] <= max(objects_y[i]))):
                    G[6] = 1000
                    lock_G6 = 1
                else:
                    if lock_G6 == 0:
                        G[6] = 10
                    else:
                        pass

            lock_G6 = 0
            H_new[6] = math.sqrt((xnew[6] - xend) ** 2 + (ynew[6] - yend) ** 2)
            F[6] = G[6] + H_new[6]
            #########################################

            min_f_value.append(min(F))
            min_indice = F.index(min_f_value[iteration])
            x0 = xnew[min_indice]
            y0 = ynew[min_indice]
            H = H_new[min_indice]

            if primer_iteracion:
                F0_save, index_min = self.ordenar_vector_con_indices_menor_a_mayor(F)
                for k in range(len(xnew)):
                    x0_old.append(xnew[k])
                    y0_old.append(ynew[k])
                primer_iteracion = False


            iteration = iteration + 1
            # print(y0_old)

            if min_f_value[-1] < min_f_value_ant:
                min_f_value_ant = min_f_value[-1]

            else:
                count_no_best = count_no_best + 1
                if count_no_best == 10:
                    count_pos_init = count_pos_init + 1
                    if count_pos_init >= len(index_min):
                        print("No found optimal")
                        break
                    else:

                        if H_ant >= min_f_value_ant:

                            self.xtrajectory_send = self.xtrajectory
                            self.ytrajectory_send = self.ytrajectory
                            plt.plot(self.xtrajectory, self.ytrajectory)
                            plt.show()
                            H_ant = min_f_value_ant

                        self.xtrajectory= [x_init]
                        self.ytrajectory = [y_init]
                        x0 = x0_old[index_min[count_pos_init]]
                        y0 = y0_old[index_min[count_pos_init]]
                        count_no_best = 0
                        min_f_value_ant = 1000
                        # print(index_min[count_pos_init])

                        # print(f'y_value {y0}')
                        # print(f'x_value {x0}')
            # print(H)
            self.delta_space = self.delta_space + w
            #print(self.delta_space)

        self.xtrajectory_send = self.xtrajectory
        self.ytrajectory_send = self.ytrajectory
        self.xtrajectory_send.append(x0)
        self.ytrajectory_send.append(y0)
        self.GlobalMin_send = min_f_value


    def ordenar_vector_con_indices_menor_a_mayor(self, vector):
        # Enumerar el vector para obtener pares (índice, valor)
        indices_y_valores = list(enumerate(vector))

        # Ordenar los pares por el valor en orden ascendente
        indices_y_valores_ordenados = sorted(indices_y_valores, key=lambda x: x[1])

        # Extraer los índices ordenados
        indices_ordenados = [index for index, _ in indices_y_valores_ordenados]

        # Crear el vector ordenado
        vector_ordenado = [vector[index] for index in indices_ordenados]

        return vector_ordenado, indices_ordenados




if __name__ == "__main__":

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

    #fig = plt.figure()
    # fig.clf()
    #ax = fig.subplots(1, 1)

    # plt.plot(object_x1, object_y1, label='Object_1', color='blue')
    # plt.plot(object_x2, object_y2, label='Object_2', color='blue')
    plt.plot(object_x3, object_y3, label='Wall_right', color='red')
    plt.plot(object_x4, object_y4, label='Wall_up', color='red')
    plt.plot(object_x5, object_y5, label='Wall_left', color='red')
    plt.plot(object_x6, object_y6, label='Wall_down', color='red')
    #ax.set_xlabel('x')
    #ax.set_ylabel('y')

    # plt.legend()
    # fig.tight_layout()
    # plt.show()
    x_init = [1, 2, 3, 4]
    y_init = [1, 1, 1, 1]
    x_end = [3, 2, 4, 4]
    y_end = [4, 3, 4, 3]
    # square_value_x = [[2, 3], [5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    # square_value_y = [[2, 3], [0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]
    square_value_x = [[5.5, 6], [0.5, 6], [0, 0.5], [0, 5.5]]
    square_value_y = [[0, 5.5], [5, 5.5], [0, 5.5], [0, 0.5]]

    for i in range(len(x_init)):
        ctf = Trajectory_Generation()

        x0 = x_init[i]
        y0 = y_init[i]
        xend = x_end[i]
        yend = y_end[i]

        ctf.AStar(x0, y0, xend, yend, square_value_x, square_value_y)
        xtrajectory = ctf.xtrajectory_send
        ytrajectory = ctf.ytrajectory_send
        plt.plot(xtrajectory, ytrajectory, color='black')

    plt.grid(True)
    plt.show()
