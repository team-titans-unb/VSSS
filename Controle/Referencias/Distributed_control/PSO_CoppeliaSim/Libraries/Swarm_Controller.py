""""
Swarm_controller.py

This code present four functions to swarm controller robot EVA/MARIA project, the first function called Swarm_Controller_Classic
showed a overview of clasic swarm controller see the Georgia Tech course; the second function called Swarm_Controller_PSO
present other way to controller swarm using the bioisnpired algorithm Particle Swarm Optimization (PSO); subsequently Swarm_positions function
is the upper controller to Swarm_Controller_PSO function and finally, Simulation_EVA function implement the controller offline on the EVA robot
within CoppeliaSim.

Author:
    Mario Andres  Pastrana Triana (mario.pastrana@ieee.org)
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

""""
math library is python property
EVA_Class_Lite is a library to execute the EVA robot within the CoppeliaSim
PSO_Algorithm is a library with the PSO algorithms to create the path and for each EVA robot
"""""

import math
import Libraries.EVA_Class_Lite as EVA_Class_Lite
import Libraries.Bioinspired_Algorithm as Bioinspired_Algorithm


class Swarm_Controller:

    """""
    Swarm_controller class is the principal class to controller the EVA positions robots.
        __init___ =                 function to inicializate the global variables
        Swarm_Controller_Classic =  showed a overview of classic swarm controller see the Georgia Tech course
        Swarm_Controller_PSO =      present other way to controller swarm using the bioisnpired algorithm Particle Swarm Optimization (PSO)
        Swarm_positions function =  is the upper controller to Swarm_Controller_PSO function
        Simulation_EVA =            implement the controller offline on the EVA robot within CoppeliaSim.
    """""

    def __init__(self):

        """"
            Initializations global variables
            self.C = constant C value used on the Swarm_controller_Classic more information see the Georgia tech course
            self.W = constant W value used on the Swarm_controller_Classic more information see the Georgia tech course
        """""

        self.C = 0.004
        self.W = 0.06

    def Swarm_Controller_Classic(self, x_position_robot, y_position_robot, k, type_formation):
        """""
        Swarm_Controller_Classic =  showed a overview of classic swarm controller see the Georgia Tech course, today
        this function only can execute two formation, Line and Rendezvouse.
        
        Arguments: 
            x_position_robot (List float)   = X axis position of the all robots
            y_position_robot (List float)   = Y axis position of the all robots
            k                (Integer)      = current robot
            type_formation   (string)       = type of the formation
            
        Init Varibles :
        
            sum_weigths_x       = sum of the weigths this is, o W value multiplied by the distances [m] neighbor to robot [k]
                                and finally, multiplied by the error between axe X neighbor[m] to robot [k]
            sum_weigths_y       = sum of the weigths this is, o W value multiplied by the distances [m] neighbor to robot [k]
                                and finally, multiplied by the error between axe Y neighbor[m] to robot [k]
            upper_limit         = max error acepted within the swarm formation
            lower_limit         = min error acepted within the swarm formation
            Limit_Swarm         = threshold to indicate if the robot belongs to the swarm 
            condition_neighbor  = selecto two differents behaviors for the swarm if 0 applied the formation
                                if 1 not applied formation
            neighbor            = determine all the neighbors in the swarm
            sum_neighbor        = Variable to read all the possibles neighbors
            
        Return :
            pix_new (Float) = New X position for the current robot
            piy_new (Float) = New Y position for the current robot
            
        """""

        sum_weigths_x = 0
        sum_weigths_y = 0
        upper_limit = 20
        lower_limit = 10
        Limit_Swarm = 50
        condition_neighbor = 0
        neighbor = 0
        sum_neighbor = 0

        ### Select the type formation in this case Rendezvouse ###

        if type_formation == 'Rendezvouse':

            ### All posibles neighbors are read in this for ###

            for m in range(len(x_position_robot)):

                ### Distance between the robot [k] and yours neighborn [m] ##

                distance_Ne = (math.sqrt((x_position_robot[m] - x_position_robot[k]) ** 2 + (y_position_robot[m] - y_position_robot[k]) ** 2))

                ### Selecto two differents W, when the robots are far the W is highs, when the robots are very near ###
                ### the W value is lower ###

                if distance_Ne > upper_limit:
                    W = self.W
                elif distance_Ne <= lower_limit:
                    W = -0.6
                else:
                    W = 0

                ### When the robot belogn to swarm the robot color is blue ###

                point_color = 'B'

                ### implement the weight equation for x and y axis more information see the Georgia Tech course ###

                sum_weigths_x = sum_weigths_x + W * distance_Ne * (x_position_robot[m] - x_position_robot[k])
                sum_weigths_y = sum_weigths_y + W * distance_Ne * (y_position_robot[m] - y_position_robot[k])

                ### implement the movement equation for x and y axis more information see the Georgia Tech course ###

                complement_x = self.C * (sum_weigths_x)
                complement_y = self.C * (sum_weigths_y)

        ### Select the type formation in this case Line ###

        elif (type_formation == 'Line'):

            ### In this case we have a leader, the leader is indepent to swarm, for this reason K is not applied in the
            ## latest position value

            if k < len(x_position_robot)-1:

                ### This while is applied only the followers robots ###

                while sum_neighbor <= len(x_position_robot)-1:

                    ### Distance between the robot [k] and your neighborn [k + neighbor + 1] just one neighborn ###

                    distance_Ne = (math.sqrt((x_position_robot[k + neighbor + 1] - x_position_robot[k]) ** 2 + (
                        y_position_robot[k + neighbor + 1] - y_position_robot[k]) ** 2))

                    ### if the neigborn belong the swarm ###

                    if distance_Ne > Limit_Swarm:
                        condition_neighbor = 0
                    else:
                        condition_neighbor = 1
                        neighbor_final = neighbor
                        break

                    ### update neighborn ###

                    neighbor = neighbor + 1
                    sum_neighbor = k + neighbor + 1

                ### if the robot not belong the swarm is a independent robot and is the red color ###

                if condition_neighbor == 0:

                    complement_x = 0.4
                    complement_y = -0.3
                    point_color = 'R'

                ### if the robot belong the swarm ###

                else:

                    ### Distance between the robot [k] and your neighborn [k + neighbor + 1] just one neighborn ###

                    distance_Ne = (math.sqrt((x_position_robot[k + neighbor_final + 1] - x_position_robot[k]) ** 2 + (
                            y_position_robot[k + neighbor_final + 1] - y_position_robot[k]) ** 2))

                    ### Selecto two differents W, when the robots are far the W is highs, when the robots are very near ###
                    ### the W value is lower ###

                    if distance_Ne > upper_limit and distance_Ne <= Limit_Swarm:
                        W = self.W

                    elif distance_Ne <= lower_limit:
                        W = -4
                    else:
                        W = 0

                    ### implement the weight equation for x and y axis more information see the Georgia Tech course ###

                    sum_weigths_x = sum_weigths_x + W * distance_Ne * (x_position_robot[k + neighbor_final + 1] - x_position_robot[k])
                    sum_weigths_y = sum_weigths_y + W * distance_Ne * (y_position_robot[k + neighbor_final + 1] - y_position_robot[k])

                    ### implement the movement equation for x and y axis more information see the Georgia Tech course ###

                    complement_x = self.C * (sum_weigths_x)
                    complement_y = self.C * (sum_weigths_y)
                    point_color = 'B'

            ### For the leader ###

            else:

                sum_weigths_x = 0
                sum_weigths_y = 0
                complement_x = self.C * (sum_weigths_x)
                complement_y = self.C * (sum_weigths_y)
                point_color = 'B'

        ### update the x and y positions robots ###

        pix_new = x_position_robot[k] + complement_x
        piy_new = y_position_robot[k] + complement_y

        ### send the news x and y positions robots ###

        return pix_new, piy_new, point_color

    def Swarm_Controller_Bioinspired(self, x_position_robot, y_position_robot, k, type_formation, AU_swarm):

        """""
        Swarm_Controller_PSO =  present other way to controller swarm using the bioisnpired algorithm Particle Swarm Optimization (PSO), today
        this function only can execute three formation, Line, triangle and square.
        
        Arguments: 
            
            x_position_robot (List float)   = X axis position of the all robots
            y_position_robot (List float)   = Y axis position of the all robots
            k                (Integer)      = current robot
            type_formation   (string)       = type of the formation
        
        Init Varibles :
        
            object_PSO_Controller   = PSO object with the bioinspired optimization functions
            value_x_line            = ideal X distance to neighbor 
            value_y_line            = ideal Y distance to neighbor 
            optimum_x_aux           = all the ideal distance values of the neighbors in the X axis
            optimum_y_aux           = all the ideal distance values of the neighbors in the X axis
            Limit_Swarm         = threshold to indicate if the robot belongs to the swarm 
            sum_neighbor            = variable to read all the followers robot
            neighbor                = determine all the neighbors in the swarm
            sum_neighbor            = Variable to read all the possibles neighbors
            
        Return :
        
            pix_new (Float) = New X position for the current robot
            piy_new (Float)= New Y position for the current robot
            point_color (Char) = robot colot 
        """""

        object_PSO_Controller = Bioinspired_Algorithm.PSO()
        object_MFO_Controller = Bioinspired_Algorithm.MFO()
        value_x_line = 0.3
        value_y_line = 0.3
        optimum_x_aux = []
        optimum_y_aux = []
        Limit_Swarm = 2
        sum_neighbor = 0
        neighbor = 0
        condition_neighbor = 0

        ### This code is execute just the followers robots ###

        if k < len(x_position_robot) - 1:

            ### Read all the neighbors robots ###

            while sum_neighbor <= len(x_position_robot) - 1:

                ### Distance between the robot [k] and your neighborn [k + neighbor + 1] just one neighborn ###

                distance_Ne = (math.sqrt((x_position_robot[k + neighbor + 1] - x_position_robot[k]) ** 2 + (
                        y_position_robot[k + neighbor + 1] - y_position_robot[k]) ** 2))

                ### if the neigborn belong the swarm ###

                if distance_Ne > Limit_Swarm:
                    condition_neighbor = 0
                else:
                    condition_neighbor = 1
                    neighbor_final = neighbor
                    break

                ### update neighborn ###

                neighbor = neighbor + 1
                sum_neighbor = k + neighbor + 1

        ### if the robot not belong the swarm is a independent robot and is the red color ###

        if condition_neighbor == 0:
            complement_x = 0.2
            complement_y = -0.15
            point_color = 'R'

        ### if the robot belong the swarm ###

        else:

            ### Select type formation in this case square ###

            if type_formation == "Square":
                if k == 0:
                    optimum_x = -value_x_line
                    optimum_y = value_y_line
                elif k == 1:
                    optimum_x = value_x_line
                    optimum_y = value_y_line
                elif k == 2:
                    optimum_x = -value_x_line
                    optimum_y = value_y_line
                elif k == len(x_position_robot)-1:
                    optimum_x = 0
                    optimum_y = 0

            ### Select type formation in this case triangle ###

            elif type_formation == "Triangle":
                if k == 0:
                    optimum_x = value_x_line
                    optimum_y = -value_y_line
                elif k == 1:
                    optimum_x = value_x_line
                    optimum_y = value_y_line

                elif k == len(x_position_robot)-1:
                    optimum_x = 0
                    optimum_y = 0

            ### Select type formation in this case Line ###

            elif type_formation == "Line_x":

                for j in range(len(x_position_robot)- 1):
                    optimum_x_aux.append(value_x_line)
                    optimum_y_aux.append(0)
                optimum_y_aux.append(0)
                optimum_x_aux.append(0)
                optimum_x = optimum_x_aux[k]
                optimum_y = optimum_y_aux[k]

            elif type_formation == "Line_x-":

                for j in range(len(x_position_robot) - 1):
                    optimum_x_aux.append(-value_x_line)
                    optimum_y_aux.append(0)
                optimum_y_aux.append(0)
                optimum_x_aux.append(0)
                optimum_x = optimum_x_aux[k]
                optimum_y = optimum_y_aux[k]

            elif type_formation == "Line_y":

                for j in range(len(x_position_robot)- 1):
                    optimum_x_aux.append(0)
                    optimum_y_aux.append(value_y_line)
                optimum_y_aux.append(0)
                optimum_x_aux.append(0)
                optimum_x = optimum_x_aux[k]
                optimum_y = optimum_y_aux[k]

            elif type_formation == "Line_y-":

                for j in range(len(x_position_robot)- 1):
                    optimum_x_aux.append(0)
                    optimum_y_aux.append(-value_y_line)
                optimum_y_aux.append(0)
                optimum_x_aux.append(0)
                optimum_x = optimum_x_aux[k]
                optimum_y = optimum_y_aux[k]

            ### Find the best movement to x and y axis using the differents bioinspired algorithms ###

            if AU_swarm == 'PSO':

                complement_x = object_PSO_Controller.PSO_Controller(x_position_robot, optimum_x, k, neighbor_final)
                complement_y = object_PSO_Controller.PSO_Controller(y_position_robot, optimum_y, k, neighbor_final)

            elif AU_swarm == 'MFO':

                complement_x = object_MFO_Controller.MFO_Controller(x_position_robot, optimum_x, k, neighbor_final)
                complement_y = object_MFO_Controller.MFO_Controller(y_position_robot, optimum_y, k, neighbor_final)

            point_color = 'B'

        ### update the x and y positions robots ###

        pix_new = x_position_robot[k] + complement_x
        piy_new = y_position_robot[k] + complement_y

        ### send the news x and y positions robots ###

        return pix_new, piy_new, point_color

    def Swarm_positions(self, x_position_robot, y_position_robot, path_leader, type_formation_base_data, AU_swarm):
        """""
        Swarm_positions =  is the upper controller to Swarm_Controller_PSO function.
        
        Arguments: 
            
            x_position_robot (List float)           = X axis position of the all robots
            y_position_robot (List float)           = Y axis position of the all robots
            path_leader      (List float)           = Path used in the leader robot
            type_formation_base_data   (string)     = type of the formation
            
        Init variables:

            EVA                     = EVA robot to use
            Historic_particles_x    = List with all robot position on the X axis
            Historic_particles_y    = List with all robot position on the Y axis
            point_color_hist        = List with all robot color blue belong the swarm, red not belong swarm
            stop                    = Iteration to wait the swarm formation
            Point_numbers           = number of iteration used on the simulation
            
        return:
        
            Historic_particles_x (Float list)   = Positions in X axis for all robots
            Historic_particles_y (Float list)   = Positions in Y axis for all robots
            point_color_hist     (Float chart)  = Color positions for all robots
            
        """""

        Historic_particles_x = []
        Historic_particles_y = []
        point_color_hist = []
        stop = 20
        Point_numbers = len(path_leader[0])

        ### Creat a matrix using all the robots ###

        for k in range(len(x_position_robot)):
            Historic_particles_x.append([])
            Historic_particles_y.append([])
            point_color_hist.append([])

        type_formation = type_formation_base_data[0]

        ### For that use all the point numbers plus stop ###

        for k in range(Point_numbers + stop):

            ### The leader only begin to move after k >= stop ###

            if k >= stop:
                x_position_robot[-1] = path_leader[0][k-stop]
                y_position_robot[-1] = path_leader[1][k-stop]

            ### Read all the robots ###

            for m in range(len(x_position_robot)):

                ### based on the current position (x,y) get the news position in x, y axis

                x_new, y_new, point_color = self.Swarm_Controller_Bioinspired(x_position_robot,
                                                                                         y_position_robot, m,
                                                                                         type_formation, AU_swarm)

                ### Change the formation ###

                if (x_new > 0.2 and x_new <= 3) and (y_new >0.2 and y_new <= 5):
                    type_formation = type_formation_base_data[0]

                elif (x_new > 3 and x_new <= 6) and (y_new >0.2 and y_new <= 5):
                    type_formation = type_formation_base_data[1]

                elif (x_new > 6 and x_new <= 9)and (y_new >0.2 and y_new <= 5):
                    type_formation = type_formation_base_data[1]

                elif (x_new > 6 and x_new <= 9)and (y_new >5 and y_new <= 6):
                    type_formation = type_formation_base_data[2]

                elif (x_new > 6 and x_new <= 9)and (y_new >6 and y_new <= 10):
                    type_formation = type_formation_base_data[3]

                elif (x_new > 3 and x_new <= 6) and (y_new >6 and y_new <= 10):
                    type_formation = type_formation_base_data[3]

                if (x_new > 0.2 and x_new <= 3) and (y_new >6 and y_new <= 10):
                    type_formation = type_formation_base_data[4]

                ### save the follower robots values x and y ###

                if m < len(x_position_robot) - 1:
                    x_position_robot[m] = round(x_new, 4)
                    y_position_robot[m] = round(y_new, 4)
                    Historic_particles_x[m].append(x_position_robot[m])
                    Historic_particles_y[m].append(y_position_robot[m])
                    point_color_hist[m].append(point_color)

                ### save the leader robot values x and y ###

                elif m == len(x_position_robot) - 1:
                    Historic_particles_x[m].append(x_position_robot[-1])
                    Historic_particles_y[m].append(y_position_robot[-1])

        ### Returns all the positions for each robot on the x and y axis ###

        return Historic_particles_x, Historic_particles_y, point_color_hist

    def Simulation_EVA(self, Position_x, Position_y, EVA):

        """""
        Simulation_EVA = Implement the controller offline on the EVA robot within CoppeliaSim. 
        important Not test yet
        
        Arguments : 
        
            Position_x (Float List)  = Matriz with all robots positions in the X axis
            Position_y (Float List)  = Matriz with all robots positions in the Y axis
            EVA        (Integer)     = EVA robot
            
        Return:
            
            none
        
        """""

        EVA_Object = EVA_Class_Lite.EVA_robot()

        for i in range(len(Position_x)):
            Position = [Position_x[i], Position_y[i]]
            Position_End = [Position_x[-1], Position_y[-1]]
            EVA_Object.EVA(Position, Position_End, EVA)



