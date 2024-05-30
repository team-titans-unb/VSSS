""""
Histogram_Neighborn.py

This code present three functions that refined the path planning solution using histogram and the best path between the 
neighborns. The best_value_bioinspired_histograma function is used to find the histogram in a area, the 
best_neighbor function find the best way using the neighborns points and the Path planning for each follower robot.

Author:
    Mario Andres Pastrana Triana (mario.pastrana@ieee.org)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)

Version:
    0.0.1 (beta)

Release Date:
    MAY 17, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)

"""""

import math
import Bioinspired_Algorithm as Bioinspired_Algorithm
import Scenery as Scenery

class Histograma_Neighbor:

    """"
    The Histograma_Neighbor class is used to refined the path planning find in the PSO algorithm; This class have two 
    function, the best_value_bioinspired_histograma is used to find the histogram in a specific space, best_neighbor
    function find the refined path planing PSO and Path planning for each follower robot
    """""

    def best_value_bioinspired_histograma(self, x_position, y_position):

        """"
        The best_value_bioinspired_histograma is used to find the histogram in a specific space
        
        Arguments:
            
            x_position (Float List)        = Position path planning in the X axis
            y_position (Float List)        = Position path planning in the Y axis
            
        Init Variables:
        
            min_x_value                 = Minimun value in the X axis
            min_y_value                 = inimun value in the Y axis
            delta_x                     = Space for find the histogram in X axis
            delta_y                     = Space for find the histogram in Y axis
            count_history_x             = Number of points in the search space in the X axis
            count_history_y             = Number of points in the search space in the Y axis
            cons_x_value                = sum of delta_x
            cons_y_value                = sum of delta_y
            histograma_x                = Frequency in each search space in the X axis
            histograma_y                = Frequency in each search space in the Y axis
            plot_histograma_value_x     = Variable to plot the histogram in the X axis
            plot_histograma_value_y     = Variable to plot the histogram in the Y axis
            
        Return:
            
            histograma_x (Float List) = List with the histogram in the X axis
            histograma_y (Float List) = List with the histogram in the Y axis
            
        """""

        ### Init variables ###

        min_x_value = min(x_position)
        min_y_value = min(y_position)
        delta_x = 0.1
        delta_y = 0.1
        count_history_x = 0
        count_history_y = 0
        cons_x_value = delta_x
        cons_y_value = delta_y
        histograma_x = []
        histograma_y = []
        plot_histograma_value_x = [min_x_value]
        plot_histograma_value_y = [min_y_value]

        ### Read al the X positions ###

        for k in x_position:

            ### If the position in X axis is <= that search space ###

            if k <= min_x_value + cons_x_value:

                count_history_x = count_history_x + 1

            else:

                histograma_x.append(count_history_x)
                plot_histograma_value_x.append(min_x_value + cons_x_value)
                cons_x_value = cons_x_value + delta_x
                count_history_x = 1

        ### Read al the Y positions ###

        for k in y_position:

            ### If the position in X axis is <= that search space ###

            if k <= min_y_value + cons_y_value:

                count_history_y = count_history_y + 1

            else:

                histograma_y.append(count_history_y)
                plot_histograma_value_y.append(min_y_value + cons_y_value)
                cons_y_value = cons_y_value + delta_y
                count_history_y = 1

        ### Update histogram values ###

        histograma_x.append(count_history_x)
        plot_histograma_value_x.append(min_x_value + cons_x_value)
        histograma_y.append(count_history_y)
        plot_histograma_value_y.append(min_y_value + cons_y_value)

        ### Return two list with histogram values ###

        return histograma_x, histograma_y

    # def best_neighbor(self, x_init_aux, y_init_aux, x_end, y_end, square_value_x, square_value_y, k_value, AU):
    def best_neighbor(self, x_init_aux, y_init_aux, x_end, y_end, square_value_x, square_value_y, AU):

        """"
        The best_neighbor find the refined path planing PSO.

        Arguments:

            x_init_aux     (Float) = Init position path planning in the X axis
            y_init_aux     (Float) = Init position path planning in the Y axis
            x_end          (Float) = Final position path planning in the X axis
            y_end          (float) = Final position path planning in the Y axis
            square_value_x (List Float) = List with all object position in X axis
            square_value_y (List Float) = List with all object position in Y axis
            k_value        (Integer) = current robot.

        Init Variables:

            PSO_ALgorithm_object    = PSO object
            criterio_stop           = Variable to stop this function
            cadjanela               = Lock janela
            janela                  = Windows to optimize the path planing using neighbor
            max_janela              = Maximum windows to optimize the path planning using neighbor
            min_janela              = Minimum windows to optimize the path planning using neighbor
            umbral_histograma       = Threshold Histogram

        Return:

            best_neig_x_send (Float List) = Best path planning points in the X axis
            best_neig_y_send (Float List) = Best path planning points in the Y axis

        """""

        ### Init variables ###

        PSO_ALgorithm_object = Bioinspired_Algorithm.PSO()
        MFO_Algorithm_Object = Bioinspired_Algorithm.MFO()
        criterio_stop = 1
        cadjanela = 0
        janela = 2
        max_janela = 2
        min_janela = 1
        umbral_histograma = 10

        ### Loop to find the best path planning ###

        while criterio_stop == 1:

            ### Get the first path planning using the bioinspired algorithms ###

            # if AU == 'PSO':

            #     x_position, y_position = PSO_ALgorithm_object.PSO_PP(x_init_aux[-1], y_init_aux[-1], x_end[k_value], y_end[k_value], square_value_x, square_value_y)

            # elif AU == 'MFO':

            #     x_position, y_position = MFO_Algorithm_Object.MFO_PP(x_init_aux[-1], y_init_aux[-1], x_end[k_value],
            #                                                      y_end[k_value], square_value_x, square_value_y)

            if AU == 'PSO':

                x_position, y_position = PSO_ALgorithm_object.PSO_PP(x_init_aux, y_init_aux, x_end, y_end,
                                                                      square_value_x, square_value_y)

            elif AU == 'MFO':

                x_position, y_position = MFO_Algorithm_Object.MFO_PP(x_init_aux, y_init_aux, x_end, y_end, 
                                                                     square_value_x, square_value_y)

            ### Get the init and end path ###

            end_y = y_position[-1]
            end_x = x_position[-1]
            init_y = y_position[0]
            init_x = x_position[0]
            fitnees_function = 1
            best_neig_x = [end_x]
            best_neig_y = [end_y]
            const_janela = 0
            cont_optimization = 0

            ### Find the best path start in the end and end in the init ####

            while fitnees_function >= 0.1:

                best_neig = 1000

                ### For each value in the windows ###

                for k in range(janela):

                    ### calculate the distance between the current point and the neighborns

                    distance_neigh = math.sqrt((end_x - x_position[-k - 2 + const_janela]) ** 2 + (
                                end_y - y_position[-k - 2 + const_janela]) ** 2)

                    ### The minimum distance is stored ###

                    if distance_neigh < best_neig:
                        end_x_next = x_position[-k - 2 + const_janela]
                        end_y_next = y_position[-k - 2 + const_janela]
                        best_neig = distance_neigh

                ### Update the constant values ###

                fitnees_function = math.sqrt((init_x - end_x_next) ** 2 + (init_y - end_y_next) ** 2)
                const_janela = x_position.index(end_x_next)
                best_neig_x.append(end_x_next)
                best_neig_y.append(end_y_next)
                end_x = end_x_next
                end_y = end_y_next
                cont_optimization = cont_optimization + 1

                ### Guarantor the end of While ###

                if cont_optimization >= len(x_position) + max_janela:
                    break

            ### Update the optimum path ###

            best_neig_x.append(init_x)
            best_neig_y.append(init_y)

            ### Check the number of points in certain area (Histogram) ###

            hist_x, hist_y = self.best_value_bioinspired_histograma(self, best_neig_x, best_neig_y)

            ### If the Histogram value is more thath Threshold ###

            if max(hist_x) >= umbral_histograma or max(hist_y) >= umbral_histograma:

                ### Not stop ###

                criterio_stop = 1

                ### largest windows ###

                if janela <= max_janela and cadjanela == 0:

                    janela = janela + 1

                    if janela == max_janela:

                        cadjanela = 1

                ### Lowest windows ###

                elif cadjanela == 1:

                    janela = janela - 1

                    if janela == min_janela:

                        cadjanela = 0

            ### If the Histogram value is lower thath Threshold ###

            else:

                criterio_stop = 0

            ### Get the sub optimal path in X and Y axis ###

            best_neig_x_send = best_neig_x[::-1]
            best_neig_y_send = best_neig_y[::-1]

            ### Send the suboptimal path ###

        return best_neig_x_send, best_neig_y_send

    def Path_followers(self, Historic_particles_x, Historic_particles_y, path_leader_extended_x, path_leader_extended_y,
                       x_end_inspection, y_end_inspection, square_value_x, square_value_y, x_init_aux, y_init_aux,
                       x_init, y_init, AU):

        """"
        The Path planning for each follower robot

        Arguments:

            Historic_particles_x    (Matriz Float) = Matrix with all position for each robot in the X axis
            Historic_particles_y    (Matriz Float) = Matrix with all position for each robot in the Y axis
            path_leader_extended_x  (List Float)   = List with all position for the leader robot in the X axis
            path_leader_extended_y  (List Float)   = List with all position for the leader robot in the Y axis
            x_end_inspection        (List Float)   = List with all end positions ponts for each robot in X axis
            y_end_inspection        (List Float)   = List with all end positions ponts for each robot in Y axis
            square_value_x          (Matriz Float) = All the objects in the scenario in X axis
            square_value_y          (Matriz Float) = All the objects in the scenario in Y axis
            x_init_aux              (List float)   = List with initial position for each robot in the X axis
            y_init_aux              (List float)   = List with initial position for each robot in the Y axis
            x_init                  (List float)   = List with initial position for each robot in the X axis
            y_init                  (List float)   = List with initial position for each robot in the Y axis

        Return:

            Historic_particles_x    (Matriz Float) = Matrix with all position for each robot in the X axis
            Historic_particles_y    (Matriz Float) = Matrix with all position for each robot in the Y axis
            path_leader_extended_x  (List Float)   = List with all position for the leader robot in the X axis
            path_leader_extended_y  (List Float)   = List with all position for the leader robot in the Y axis
            x_init_aux              (List float)   = List with initial position for each robot in the X axis
            y_init_aux              (List float)   = List with initial position for each robot in the Y axis
            x_init                  (List float)   = List with initial position for each robot in the X axis
            y_init                  (List float)   = List with initial position for each robot in the Y axis

        """""

        ### Create the Scenary object ###

        object_SC = Scenery.Scenary()

        ### Read each history robot ###

        for robots_number in range(len(Historic_particles_x)):

            ### Use the final point as init point for each robot ###

            x_init_robot = [Historic_particles_x[robots_number][-1]]
            y_init_robot = [Historic_particles_y[robots_number][-1]]

            ### Get the new path planning for each robot this PP is for arrive to the machine ###

            x_best_neighbor_robot, y_best_neighbor_robot = self.best_neighbor(x_init_robot, y_init_robot, x_end_inspection[robots_number], y_end_inspection[robots_number], square_value_x, square_value_y, 0, AU)
            path_robot_extended_x, path_robot_extended_y = object_SC.generar_puntos(x_best_neighbor_robot, y_best_neighbor_robot, espacio=0.03)

            ### Update the variables ###

            Historic_particles_x[robots_number].extend(path_robot_extended_x)
            Historic_particles_y[robots_number].extend(path_robot_extended_y)
            x_init[robots_number] = Historic_particles_x[robots_number][-1]
            y_init[robots_number] = Historic_particles_y[robots_number][-1]

            ### When read the leader robot ###

            if robots_number == len(Historic_particles_x) - 1:

                path_leader_extended_x.extend(path_robot_extended_x)
                path_leader_extended_y.extend(path_robot_extended_y)
                x_init_aux[-1] = Historic_particles_x[robots_number][-1]
                y_init_aux[-1] = Historic_particles_y[robots_number][-1]

            ### Send the news values for each variable ###

        return Historic_particles_x, Historic_particles_y, path_leader_extended_x, path_leader_extended_y, x_init_aux, y_init_aux, x_init, y_init