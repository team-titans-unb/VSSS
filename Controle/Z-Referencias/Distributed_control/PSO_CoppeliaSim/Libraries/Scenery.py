""""
Scenery.py

This code present the scenarios used on the EVA-MARIA project, now just a scenary is implemented called 
scenario_test_factory function and the generar_puntos function is used to obtain more points between line. This code
present an animation in circles for each robot and scenary.

Author:
    Mario Andres Pastrana Triana (mario.pastrana@ieee.org)
    EVA/MARIA PROJECT - University of Brasilia-(FGA)

Version:
    0.0.1 (beta)

Release Date:
    MAY 20, 2024

Finally comment:
    Querido lector por ahora, la correcta implementación de este código lo sabe Mario, Dios, la Virgen Maria y los santos
    esperemos que futuramente Mario continue sabiendo como ejecutar el código o inclusive que más personas se unan para
    que el conocimiento aqui depositado se pueda expandir de la mejor manera.
    Let's go started =)

"""""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class Scenary:

    """"
    The Scenary class implement the scenarios used on the EVA-MARIA project. This class just only one scenary called
    scenario_test_factory and the generar_puntos that create more artificial point to simulated the robots.
    """""

    def scenario_test_factory(self, plot_scenario = 0, best_neighbor = [], Historic_particles_x = [[]], Historic_particles_y = [[]], active_animation = 0):

        """"
        The best_value_bioinspired_histograma is used to find the histogram in a specific space

        Arguments:

            plot_scenario       (Integer)       = active the plot
            best_neighbor       (Float List)    = Best path find for the leader robot
            Historic_particles_x(Float Matriz)  = Matriz with all position of the robots in the X axis 
            Historic_particles_y(Float Matriz)  = Matriz with all position of the robots in the Y axis
            active_animation    (Integer)       = Active the animation

        Return:

            square_value_x (Float Matriz) = Objects present on the scenary (X axis)
            square_value_y (Float Matriz) = Objects present on the scenary (Y axis)

        """""

        ### First object on the scenary ###

        min_object_1_x = 3
        max_object_1_x = 6
        min_object_1_y = 0.5
        max_object_1_y = 3

        ### Second object on the scenary ###

        min_object_2_x = 2
        max_object_2_x = 8
        min_object_2_y = 5
        max_object_2_y = 6

        ### Third object on the scenary ###

        min_object_3_x = 3
        max_object_3_x = 6
        min_object_3_y = 8
        max_object_3_y = 10

        ### Right Wall ###

        min_rigth_object_x = 10
        max_rigth_object_x = 10.5
        min_rigth_object_y = 0
        max_rigth_object_y = 10

        ### Top Wall ###

        min_up_object_x = 0
        max_up_object_x = 10.5
        min_up_object_y = 10
        max_up_object_y = 10.5

        ### Left Wall ###

        min_left_object_x = -0.5
        max_left_object_x = 0
        min_left_object_y = 0
        max_left_object_y = 10.5

        ### Bottom Wall ###

        min_down_object_x = 0
        max_down_object_x = 10.5
        min_down_object_y = 0
        max_down_object_y = 0.5

        ### Machine 1 ###

        min_machine_1_x = 7
        max_machine_1_x = 7.5
        min_machine_1_y = 2.5
        max_machine_1_y = 3

        ### Machine 2 ###

        min_machine_2_x = 9
        max_machine_2_x = 9.5
        min_machine_2_y = 2.5
        max_machine_2_y = 3

        ### Machine 3 ###

        min_machine_3_x = 8
        max_machine_3_x = 8.5
        min_machine_3_y = 1
        max_machine_3_y = 1.5

        ### Machine 4 ###

        min_machine_4_x = 7
        max_machine_4_x = 7.5
        min_machine_4_y = 9
        max_machine_4_y = 9.5

        ### Machine 5 ###

        min_machine_5_x = 8
        max_machine_5_x = 8.5
        min_machine_5_y = 9
        max_machine_5_y = 9.5

        ### Machine 6 ###

        min_machine_6_x = 9
        max_machine_6_x = 9.5
        min_machine_6_y = 9
        max_machine_6_y = 9.5

        ### Machine 7 ###

        min_machine_7_x = 2
        max_machine_7_x = 2.5
        min_machine_7_y = 9
        max_machine_7_y = 9.5

        ### Machine 8 ###

        min_machine_8_x = 0.5
        max_machine_8_x = 1
        min_machine_8_y = 9
        max_machine_8_y = 9.5

        ### Machine 9 ###

        min_machine_9_x = 0.5
        max_machine_9_x = 1
        min_machine_9_y = 7
        max_machine_9_y = 7.5

        ### First object on the scenary in a list ###

        object_x1 = [min_object_1_x, max_object_1_x, max_object_1_x, min_object_1_x, min_object_1_x]
        object_y1 = [min_object_1_y, min_object_1_y, max_object_1_y, max_object_1_y, min_object_1_y]

        ### Second object on the scenary in a list ###

        object_x2 = [min_object_2_x, max_object_2_x, max_object_2_x, min_object_2_x, min_object_2_x]
        object_y2 = [min_object_2_y, min_object_2_y, max_object_2_y, max_object_2_y, min_object_2_y]

        ### Third object on the scenary in a list ###

        object_x7 = [min_object_3_x, max_object_3_x, max_object_3_x, min_object_3_x, min_object_3_x]
        object_y7 = [min_object_3_y, min_object_3_y, max_object_3_y, max_object_3_y, min_object_3_y]

        ### Machine 1 object on the scenario in a list ###

        object_machine_1_x = [min_machine_1_x, max_machine_1_x, max_machine_1_x, min_machine_1_x, min_machine_1_x]
        object_machine_1_y = [min_machine_1_y, min_machine_1_y, max_machine_1_y, max_machine_1_y, min_machine_1_y]

        ### Machine 2 object on the scenario in a list ###

        object_machine_2_x = [min_machine_2_x, max_machine_2_x, max_machine_2_x, min_machine_2_x, min_machine_2_x]
        object_machine_2_y = [min_machine_2_y, min_machine_2_y, max_machine_2_y, max_machine_2_y, min_machine_2_y]

        ### Machine 3 object on the scenario in a list ###

        object_machine_3_x = [min_machine_3_x, max_machine_3_x, max_machine_3_x, min_machine_3_x, min_machine_3_x]
        object_machine_3_y = [min_machine_3_y, min_machine_3_y, max_machine_3_y, max_machine_3_y, min_machine_3_y]

        ### Machine 4 object on the scenario in a list ###

        object_machine_4_x = [min_machine_4_x, max_machine_4_x, max_machine_4_x, min_machine_4_x, min_machine_4_x]
        object_machine_4_y = [min_machine_4_y, min_machine_4_y, max_machine_4_y, max_machine_4_y, min_machine_4_y]

        ### Machine 5 object on the scenario in a list ###

        object_machine_5_x = [min_machine_5_x, max_machine_5_x, max_machine_5_x, min_machine_5_x, min_machine_5_x]
        object_machine_5_y = [min_machine_5_y, min_machine_5_y, max_machine_5_y, max_machine_5_y, min_machine_5_y]

        ### Machine 6 object on the scenario in a list ###

        object_machine_6_x = [min_machine_6_x, max_machine_6_x, max_machine_6_x, min_machine_6_x, min_machine_6_x]
        object_machine_6_y = [min_machine_6_y, min_machine_6_y, max_machine_6_y, max_machine_6_y, min_machine_6_y]

        ### Machine 7 object on the scenario in a list ###

        object_machine_7_x = [min_machine_7_x, max_machine_7_x, max_machine_7_x, min_machine_7_x, min_machine_7_x]
        object_machine_7_y = [min_machine_7_y, min_machine_7_y, max_machine_7_y, max_machine_7_y, min_machine_7_y]

        ### Machine 8 object on the scenario in a list ###

        object_machine_8_x = [min_machine_8_x, max_machine_8_x, max_machine_8_x, min_machine_8_x, min_machine_8_x]
        object_machine_8_y = [min_machine_8_y, min_machine_8_y, max_machine_8_y, max_machine_8_y, min_machine_8_y]

        ### Machine 9 object on the scenario in a list ###

        object_machine_9_x = [min_machine_9_x, max_machine_9_x, max_machine_9_x, min_machine_9_x, min_machine_9_x]
        object_machine_9_y = [min_machine_9_y, min_machine_9_y, max_machine_9_y, max_machine_9_y, min_machine_9_y]

        ### Right Wall in a list ###

        object_x3 = [min_rigth_object_x, max_rigth_object_x, max_rigth_object_x, min_rigth_object_x, min_rigth_object_x]
        object_y3 = [min_rigth_object_y, min_rigth_object_y, max_rigth_object_y, max_rigth_object_y, min_rigth_object_y]

        ### Top Wall in a list ###

        object_x4 = [min_up_object_x, max_up_object_x, max_up_object_x, min_up_object_x, min_up_object_x]
        object_y4 = [min_up_object_y, min_up_object_y, max_up_object_y, max_up_object_y, min_up_object_y]

        ### Left Wall in a list ###

        object_x5 = [min_left_object_x, max_left_object_x, max_left_object_x, min_left_object_x, min_left_object_x]
        object_y5 = [min_left_object_y, min_left_object_y, max_left_object_y, max_left_object_y, min_left_object_y]

        ### Bottom Wall in a list ###

        object_x6 = [min_down_object_x, max_down_object_x, max_down_object_x, min_down_object_x, min_down_object_x]
        object_y6 = [min_down_object_y, min_down_object_y, max_down_object_y, max_down_object_y, min_down_object_y]

        ### Create a matriz with all the objects present on the scenary ###

        square_value_x = [[min_object_1_x, max_object_1_x], [min_object_2_x, max_object_2_x], [min_rigth_object_x, max_rigth_object_x],
                          [min_up_object_x, max_up_object_x], [min_left_object_x, max_left_object_x],
                          [min_down_object_x, max_down_object_x], [min_object_3_x, max_object_3_x], [min_machine_1_x, max_machine_1_x],
                          [min_machine_2_x, max_machine_2_x], [min_machine_3_x, max_machine_3_x], [min_machine_4_x, max_machine_4_x],
                          [min_machine_5_x, max_machine_5_x], [min_machine_6_x, max_machine_6_x], [min_machine_7_x, max_machine_7_x],
                          [min_machine_8_x, max_machine_8_x], [min_machine_9_x, max_machine_9_x]]
        square_value_y = [[min_object_1_y, max_object_1_y], [min_object_2_y, max_object_2_y], [min_rigth_object_y, max_rigth_object_y],
                          [min_up_object_y, max_up_object_y], [min_left_object_y, max_left_object_y],
                          [min_down_object_y, max_down_object_y], [min_object_3_y, max_object_3_y], [min_machine_1_y, max_machine_1_y],
                          [min_machine_2_y, max_machine_2_y], [min_machine_3_y, max_machine_3_y], [min_machine_4_y, max_machine_4_y],
                          [min_machine_5_y, max_machine_5_y], [min_machine_6_y, max_machine_6_y], [min_machine_7_y, max_machine_7_y],
                          [min_machine_8_y, max_machine_8_y], [min_machine_9_y, max_machine_9_y]]

        ### If the plot_scenario is active ###

        if plot_scenario == 1:

            ### create the figure and the robots (circles) ###

            fig, ax = plt.subplots()
            circle_leader, = ax.plot([], [], 'bo', markersize=5)  # 'bo' para un círculo azul
            circle_follower_1, = ax.plot([], [], 'ro', markersize=5)  # 'bo' para un círculo rojo
            circle_follower_2, = ax.plot([], [], 'ro', markersize=5)  # 'bo' para un círculo rojo

            ### function to init the robots ###

            def init():

                circle_leader.set_data([], [])
                circle_follower_1.set_data([], [])
                circle_follower_2.set_data([], [])

                return circle_leader, circle_follower_1, circle_follower_2,

            ### function to update the robots positions ###

            def update(frame):

                # Calcular la nueva posición del círculo
                # print(frame)

                if frame >= len(Historic_particles_x[2]):

                    x = Historic_particles_x[2][-1]
                    y = Historic_particles_y[2][-1]

                else:

                    x = Historic_particles_x[2][frame]
                    y = Historic_particles_y[2][frame]

                if frame >= len(Historic_particles_x[0]):

                    x_follower_1 = Historic_particles_x[0][-1]
                    y_follower_1 = Historic_particles_y[0][-1]

                else:
                    x_follower_1 = Historic_particles_x[0][frame]
                    y_follower_1 = Historic_particles_y[0][frame]

                if frame >= len(Historic_particles_x[1]):

                    x_follower_2 = Historic_particles_x[1][-1]
                    y_follower_2 = Historic_particles_y[1][-1]

                else:

                    x_follower_2 = Historic_particles_x[1][frame]
                    y_follower_2 = Historic_particles_y[1][frame]

                circle_leader.set_data(x, y)
                circle_follower_1.set_data(x_follower_1, y_follower_1)
                circle_follower_2.set_data(x_follower_2, y_follower_2)

                return circle_leader, circle_follower_1, circle_follower_2,

            ### Plot the walls and object on the scenario ###

            ax.plot(object_x1, object_y1, label='Object_1', color='blue')
            ax.plot(object_x2, object_y2, label='Object_2', color='blue')
            ax.plot(object_x7, object_y7, label='Object_3', color='blue')
            ax.plot(object_x3, object_y3, label='Wall_right', color='red')
            ax.plot(object_x4, object_y4, label='Wall_up', color='red')
            ax.plot(object_x5, object_y5, label='Wall_left', color='red')
            ax.plot(object_x6, object_y6, label='Wall_down', color='red')
            ax.plot(object_machine_1_x, object_machine_1_y, label='machine_1', color='cyan')
            ax.plot(object_machine_2_x, object_machine_2_y, label='machine_2', color='cyan')
            ax.plot(object_machine_3_x, object_machine_3_y, label='machine_3', color='cyan')
            ax.plot(object_machine_4_x, object_machine_4_y, label='machine_4', color='cyan')
            ax.plot(object_machine_5_x, object_machine_5_y, label='machine_5', color='cyan')
            ax.plot(object_machine_6_x, object_machine_6_y, label='machine_6', color='cyan')
            ax.plot(object_machine_7_x, object_machine_7_y, label='machine_7', color='cyan')
            ax.plot(object_machine_8_x, object_machine_8_y, label='machine_8', color='cyan')
            ax.plot(object_machine_9_x, object_machine_9_y, label='machine_9', color='cyan')
            ax.plot(best_neighbor[0], best_neighbor[1], color='green')
            plt.grid(True)

            ### If the animation option is active ###

            if active_animation == 1:

                ani = animation.FuncAnimation(fig, update, frames=np.arange(0, 3600, 1), init_func=init, blit=True)

                ### Plot the figure ###

                plt.show()

            else:

                pass



        ### Return the

        return square_value_x, square_value_y

    def generar_puntos(self, puntos_x, puntos_y, espacio):
        """"
        Function used to obtain more points between the lines

        Arguments:

            puntos_x    (Float List)= List with X points
            puntos_y    (Float List)= List with Y points
            espacio     (Float)     = space between the new artificials points

        Return:

            x_puntos_send (Float List) = New list with more points in X axis
            y_puntos_send (Float List) = New list with more points in Y axis

        """""

        x_puntos_send = []
        y_puntos_send = []

        ### Read the initial points in X and Y axis ###

        for j in range(len(puntos_x) - 1):

            x1 = puntos_x[j]
            x2 = puntos_x[j+1]
            y1 = puntos_y[j]
            y2 = puntos_y[j+1]

            ### Calculate the more artificial points ###

            distancia = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            num_puntos = int(distancia / espacio) + 1  # +1 para incluir el punto final

            ### Update the new list with the artificials points ###

            x_puntos = np.linspace(x1, x2, num_puntos)
            y_puntos = np.linspace(y1, y2, num_puntos)
            x_puntos_send.extend(x_puntos)
            y_puntos_send.extend(y_puntos)

        ### Send the new list with the artificials points ###

        return x_puntos_send, y_puntos_send