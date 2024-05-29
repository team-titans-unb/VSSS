import Libraries.Bioinspired_Algorithm as PA
import Libraries.Swarm_Controller as SW
import Libraries.Scenery as sc
import Libraries.Histogram_Neighbor as hn
import time
import numpy as np


if __name__ == "__main__":

    ctf = PA.PSO()
    sc_object = sc.Scenary()
    object_swarm_controller = SW.Swarm_Controller()
    object_scenary = sc.Scenary()
    object_hn = hn.Histograma_Neighbor()
    AU_PP = 'PSO'
    AU_swarm = 'PSO'
    x_init = [1, 1.5, 2]
    x_init_aux = [2]
    y_init = [1.5, 1.5, 1.5]
    y_init_aux = [1.5]
    inspection_activate = 1
    type_formation_base_data = ['Triangle', 'Line_x', 'Line_y', 'Line_x-', 'Line_y-']
    count_inspection = 0
    x_end = [[6], [9], [3], [1]] # End swarm X axis
    y_end = [[4], [6], [7], [4]] # End swarm Y axis
    # x_end = [[9]] # End swarm X axis
    # y_end = [[7]] # End swarm Y axis
    x_end_inspection = [[[7.2]], [[9.2]], [[8.2]]]
    y_end_inspection = [[[3.5]], [[3.5]], [[2]]]
    square_value_x, square_value_y = object_scenary.scenario_test_factory()

    for k in range(len(x_end)):

        final_x_values = []
        final_y_values = []
        inicio = time.time()
        x_best_neighbor, y_best_neighbor = object_hn.best_neighbor(x_init_aux, y_init_aux, x_end, y_end, square_value_x,
                                                             square_value_y, k, AU_PP)
        fin = time.time()
        duracion = fin - inicio
        print("Tiempo de ejecución del lider:", duracion, "segundos")
        final_x_values = final_x_values + x_best_neighbor
        final_y_values = final_y_values + y_best_neighbor
        square_value_x, square_value_y = sc_object.scenario_test_factory(1, [final_x_values, final_y_values], active_animation=0)

        path_leader = [final_x_values, final_y_values]
        path_leader_extended_x, path_leader_extended_y = sc_object.generar_puntos(final_x_values, final_y_values, espacio=0.03)
        path_leader_extended = [path_leader_extended_x, path_leader_extended_y]
        Historic_particles_x, Historic_particles_y, point_color_hist = object_swarm_controller.Swarm_positions(x_init, y_init,
                                                                                                               path_leader_extended, type_formation_base_data, AU_swarm)

        if inspection_activate == 1:

            if count_inspection == 0:

                x_end_inspection = [[[7.2]], [[8.2]], [[9.2]]]
                y_end_inspection = [[[3.5]], [[2]], [[3.5]]]

            elif count_inspection == 1:

                x_end_inspection = [[[7.2]], [[8.2]], [[9.2]]]
                y_end_inspection = [[[8.5]], [[8.5]], [[8.5]]]

            elif count_inspection == 2:

                x_end_inspection = [[[2]], [[0.7]], [[1.5]]]
                y_end_inspection = [[[8.5]], [[8.5]], [[7.5]]]

            elif count_inspection == 3:

                x_end_inspection = [[[1]], [[1.5]], [[2]]]
                y_end_inspection = [[[1.5]], [[1.5]], [[1.5]]]

            inicio = time.time()
            Historic_particles_x, Historic_particles_y, path_leader_extended_x, path_leader_extended_y, x_init_aux, y_init_aux, x_init, y_init = object_hn.Path_followers(Historic_particles_x, Historic_particles_y, path_leader_extended_x, path_leader_extended_y,x_end_inspection, y_end_inspection, square_value_x, square_value_y, x_init_aux, y_init_aux, x_init, y_init, AU_PP)
            fin = time.time()
            duracion = fin - inicio
            print("Tiempo de ejecución de los seguidores:", duracion, "segundos")
            count_inspection = count_inspection + 1

        square_value_x, square_value_y = sc_object.scenario_test_factory(1, [path_leader_extended_x, path_leader_extended_y], Historic_particles_x, Historic_particles_y, active_animation=1)
    #object_swarm_controller.Simulation_EVA(final_x_values, final_y_values, 0)
