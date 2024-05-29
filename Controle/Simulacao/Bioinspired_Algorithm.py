""""
Bioinspired_Algorithm.py

This code have a several bioisnpired algorithms_class (today only the PSO and MFO) to create of optimum path planing and 
swarm position controller. This file have two class with six functions, the PSO_PP and MFO_PP that created the suboptimal path 
planning for the leader robot; The PSO and MFO controller create the best step in the X or Y axis, this function is executed
two times for get the best value in X and Y axis; Finally, function_swarm_controller_line is the fitness function used
in the PSO_Controller and MFO_Controller Functions

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
import numpy as np

class PSO:

    """""
    Class PSO have tree differents functions the first function is PSO_PP that obtain the best path planing for the
    EVA-MARIA robot; the second funtion is PSO_Controller, this function obtain the best position to keep the ideal
    position; Finally, the function_swarm_controller_line and fitness_function_PP are the fitness function to 
    PSO_Controller and the PSO_PP respectivelly.
    """""

    def __init__(self):

        """
            Initializations global variables
            self.fitVector  = Is a vector with best position 
            self.ys         = Best solution of the particles
            self.position_y = Is a list with the best position in the PSO_PP lagorithm in the Y axis
            self.position_x = Is a list with the best position in the PSO_PP lagorithm in the X axis
            self.Tolerance  = Tolerance factor between the objects in meters
        """""

        self.fitVector = []
        self.ys = []
        self.position_y = []
        self.position_x = []
        self.Tolerance = 0.02

    def PSO_PP(self, x_init, y_init, xend, yend, objects_x, objects_y):

        """"
        The PSO_PP that obtain the best path planing for the EVA-MARIA robot
        
        Arguments:
            
            x_init (Integer)        = Init position of the robot leader in the X axis
            y_init (Integer)        = Init position of the robot leader in the Y axis
            xend   (Integer)        = End position of the robot leader in the X axis
            yedn   (Integer)        = End position of the robot leader in the Y axis
            objects_x (Float list)  = List with all the objects positions in the X axis
            objects_y (Float list)  = List with all the objects positions in the Y axis
            
        Init Variables:
        
            S           = Number of particles
            N           = Number of dimensions
            maxIter     = Number of iterations
            w0          = Inertial Factor init
            wf          = Inertial Factor final
            c1          = cognitive component
            c2          = social component
            vMax        = Maximum speed of the particles
            vInit       = Init speed of the particles
            xMax        = Init search space top
            xMin        = Init search space bottom
            H_new       = Distance between the robot and the end point
            
        Return:
            
            self.position_x (Float List) = Best path in the X axis
            self.position_y (Float List) = Best path in the Y axis
            
        """""

        ### Init PSO variables ###

        S = 10
        N = 2
        maxIter = 100
        w0 = 0.9
        wf = 0.001
        c1 = 2.05
        c2 = 2.05
        vMax = 0.01
        vIni = vMax / 10
        xMax = 0.1
        xMin = -0.1
        w = w0
        dw = (wf - w0) / maxIter
        H_new = 100
        self.position_y.append(y_init)
        self.position_x.append(x_init)

        ### While the distance betwen the robot and the end point is more that 0.2 meters ###

        while H_new >= 0.02:

            ### Init parameers used in the PSO algorithm ###


            x = xMin + (xMax - xMin) * np.random.rand(S, N)
            y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
            bestFit = 10000
            fInd, k = bestFit * np.ones(S), 1
            lock_G4 = 0
            count_PSO = 0

            ### Main PSO loop ###

            while count_PSO <= maxIter:


                ### Loop to find the best individual particle ###

                for i in range(S):

                    fx, lock_G4, H_new = self.fitness_function_PP(x[i,:], x_init, y_init, objects_x, objects_y, xend, yend, lock_G4)

                    if fx < fInd[i]:
                        y[i, :] = x[i, :]
                        fInd[i] = fx

                ### Save the best values ###

                bestFit = min(fInd)
                self.fitVector.append(bestFit)
                p = np.where(fInd == bestFit)[0][0]
                self.ys = y[p, :]

                ### Particles' speed update using inertia factor. ###

                for j in range(N):
                    for i in range(S):
                        u1, u2 = np.random.rand(), np.random.rand()
                        v[i, j] = w * v[i, j] + c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.ys[j] - x[i, j])
                        x[i, j] += v[i, j]

                        if x[i, j] > xMax: x[i, j] = xMax - np.random.rand() * (xMax - xMin)
                        if x[i, j] < xMin: x[i, j] = xMin + np.random.rand() * (xMax - xMin)
                        if v[i, j] > vMax: v[i, j] = vMax - np.random.rand() * (vMax - vIni)
                        if v[i, j] < 0: v[i, j] = vIni

                ### Update several variables in the PSO algorithm ###

                count_PSO = count_PSO + 1
                w += dw
                # print(count_PSO)

            x_init = self.ys[0] + x_init
            y_init = self.ys[1] + y_init
            self.position_y.append(y_init)
            self.position_x.append(x_init)
            # print(Max_experiment)

        ### Return the best position on the axis Y and X ###

        return self.position_x, self.position_y

    def PSO_Controller(self, position_robot, ideal_distance, actual_robot, neighbor_final):

        """"
         PSO_Controller, this function obtain the best position to keep the ideal position
        
        Arguments:
            
            position_robot (Float)        = Current position robot in the X or Y axis
            ideal_distance (Float)        = ideal position robot in the X or Y axis
            actual_robot   (Integer)      = current robot
            neighbor_final (Integer)      = Neighbot of the current robot
            
        Init Variables:
        
            S           = Number of particles
            N           = Number of dimensions
            maxIter     = Number of iterations
            w0          = Inertial Factor init
            wf          = Inertial Factor final
            c1          = cognitive component
            c2          = social component
            vMax        = Maximum speed of the particles
            vInit       = Init speed of the particles
            xMax        = Init search space top
            xMin        = Init search space bottom
            
        Return:
            
            self.ys[0] (Float) = Best postion in the X or Y axis
            
        """""

        S = 5
        N = 1
        maxIter = 50
        w0 = 0.9
        wf = 0.1
        c1 = 2.05
        c2 = 2.05
        vMax = 0.01
        vIni = vMax / 10
        xMax = 0.1
        xMin = -0.1

        ### PSO Initializations ###

        w, dw = w0, (wf - w0) / maxIter
        x = xMin + (xMax - xMin) * np.random.rand(S, N)
        y, v = 1e10 * np.ones((S, N)), vIni * np.ones((S, N))
        fInd, k = 1e10 * np.ones(S), 1

        ### PSO Main Loop ###

        while k <= maxIter:


            ### Loop to find the best individual particle ###

            for i in range(S):

                fx = self.function_swarm_controller_line(position_robot, ideal_distance, actual_robot, x[i,:], neighbor_final)

                if fx < fInd[i]:
                    y[i, :] = x[i, :]
                    fInd[i] = fx


            ### Find the best overall particle from the swarm ###

            bestFit = min(fInd)
            self.fitVector.append(bestFit)
            p = np.where(fInd == bestFit)[0][0]
            self.ys = y[p, :]


            ### Particles' speed update using inertia factor. ###

            for j in range(N):
                for i in range(S):
                    u1, u2 = np.random.rand(), np.random.rand()
                    v[i, j] = w * v[i, j] + c1 * u1 * (y[i, j] - x[i, j]) + c2 * u2 * (self.ys[j] - x[i, j])
                    x[i, j] += v[i, j]

                    if x[i, j] > xMax: x[i, j] = xMax - (np.random.rand() * (xMax - xMin))
                    if x[i, j] < xMin: x[i, j] = xMin + (np.random.rand() * (xMax - xMin))
                    if v[i, j] > vMax: v[i, j] = vMax - (np.random.rand() * vMax)
                    if v[i, j] < -vMax: v[i, j] = -vMax + (np.random.rand() * vMax)

            ### Update several variables in the PSO algorithm ###

            k += 1
            w += dw

        ### Return the best position in the X or Y axis

        return self.ys[0]

    def function_swarm_controller_line(self, robots, ideal, k, x, neighbor_final):

        """"
        The function_swarm_controller_line is the fitness function to PSO_Controller.

        Arguments:

            robots          (Integer)        = Position of the current robot in the X or Y axis
            ideal           (Float)          = End ideal position for the current robot in the X or Y axis
            k               (Integer)        = Current robot
            x               (Float list)     = Posible new position in the X or Y axis
            neighbor_final  (Float list)     = Neighbor to the current robot

        Return:

            f (Float) = Error between the posible new position and the ideal distance

        """""

        if k+1 == len(robots):
            pass
        else:
            ideal_distance_x = robots[k + neighbor_final + 1] - ideal
            distance_now_x = (robots[k] + x[0])
            sum_distances = (ideal_distance_x - distance_now_x)

        f = abs(sum_distances)

        return f

    def fitness_function_PP(self, x, x_init, y_init, objects_x, objects_y, xend, yend, lock_G4):

        """"
        The ffitness_function_PP is the fitness function to PSO_PP.

        Arguments:

            x                   (Float)        = movement in the X and Y axis positions
            x_init              (Float)        = Init position of the robot in the X axis
            y_init              (Float)        = Init position of the robot in the Y axis
            objects_x           (Float List)   = List with all the position in the X axis of the objects
            objects_y           (Float List)   = List with all the position in the Y axis of the objects
            xend                (Float)        = End point of the robot in the X axis
            yend                (Float)        = End point of the robot in the Y axis
            exploration_weights (Integer)      = Variable of exploration in the PSO algorithm
            lock_G4             (Integer)      = Lock for the PSO algorithm

        Return:

            f (Float)      = fitness function value
            lock_G4 (Integer) = Lock in the PSO algorithm 
            H_new (Float)  = Distance between the robot and the end point

        """""

        ### Psoible new position ###

        xnew = x_init + x[0]
        ynew = y_init + x[1]

        ### Read all the objects in the enviroment ####

        # for k in range(len(objects_x)):

        #     ### If the posible new position is within the objects in the enviroment ###

        #     if ((xnew >= min(objects_x[k]) - self.Tolerance and xnew <= max(objects_x[k]) + self.Tolerance) and (
        #             ynew >= min(objects_y[k]) - self.Tolerance and ynew <= max(objects_y[k]) + self.Tolerance)):
        #         G = 10000
        #         lock_G4 = 1

        #     ### If the posible new position is not within the objects in the enviroment ###

        #     else:
        #         if lock_G4 == 0:
        #             G = 10
        #         else:
        #             pass

        ### If the posible new position is within the objects in the enviroment ###

        if ((xnew >= min(objects_x) - self.Tolerance and xnew <= max(objects_x) + self.Tolerance) and (
                ynew >= min(objects_y) - self.Tolerance and ynew <= max(objects_y) + self.Tolerance)):
            G = 10000
            lock_G4 = 1

        ### If the posible new position is not within the objects in the enviroment ###

        else:
            if lock_G4 == 0:
                G = 10
            else:
                pass

        ### Calculate the fitness function f ###

        lock_G4 = 0
        distance = math.sqrt((xnew - xend) ** 2 + (ynew - yend) ** 2)
        H_new = (((distance * 3.1416) / 2) + distance)
        # f = G + H_new - 10 + exploration_weights
        f = G + H_new - 10

        ### Return fitness function and the new distance between the robot and the end point ###

        return f, lock_G4, H_new

class MFO:
    """""
    Class MFO have tree differents functions the first function is MFO_PP that obtain the best path planing for the
    EVA-MARIA robot; the second funtion is MFO_Controller, this function obtain the best position to keep the ideal
    position; Finally, the function_swarm_controller_line and fitness_function_PP are the fitness function to 
    PSO_Controller and the MFO_PP respectivelly.
    """""

    def __init__(self):
        """
            Initializations global variables
            self.fitVector  = Is a vector with best position 
            self.ys         = Best solution of the particles
            self.position_y = Is a list with the best position in the PSO_PP lagorithm in the Y axis
            self.position_x = Is a list with the best position in the PSO_PP lagorithm in the X axis
            self.Tolerance  = Tolerance factor between the objects in meters
        """""

        self.fitVector = []
        self.ys = []
        self.position_y = []
        self.position_x = []
        self.Tolerance = 0.3

    def initialization(self, SearchAgents_no, dim, ub, lb):

        Boundary_no = 1  # number of boundaries

        # If the boundaries of all variables are equal and user enters a single number for both ub and lb
        if Boundary_no == 1:
            X = np.random.rand(SearchAgents_no, dim) * (ub - lb) + lb

        # If each variable has a different lb and ub
        if Boundary_no > 1:
            X = np.zeros((SearchAgents_no, dim))
            for i in range(dim):
                ub_i = ub[i]
                lb_i = lb[i]
                X[:, i] = np.random.rand(SearchAgents_no) * (ub_i - lb_i) + lb_i

        return X

    def MFO_PP(self, x_init, y_init, xend, yend, objects_x, objects_y):

        """"
       The MFO_PP that obtain the best path planing for the EVA-MARIA robot

       Arguments:

           x_init (Integer)        = Init position of the robot leader in the X axis
           y_init (Integer)        = Init position of the robot leader in the Y axis
           xend   (Integer)        = End position of the robot leader in the X axis
           yedn   (Integer)        = End position of the robot leader in the Y axis
           objects_x (Float list)  = List with all the objects positions in the X axis
           objects_y (Float list)  = List with all the objects positions in the Y axis

       Init Variables:

            N               = Number of particles
            dim             = Number of dimensions
            Max_iteration   = Number of iterations
            lb              = Init search space top
            ub              = Init search space bottom
            H_new       = Distance between the robot and the end point

       Return:

           self.position_x (Float List) = Best path in the X axis
           self.position_y (Float List) = Best path in the Y axis

       """""

        ### Init MFO variables ###

        N = 25
        dim = 2
        Max_iteration = 10
        ub = 0.1
        lb = -0.1
        H_new = 100
        self.position_y.append(y_init)
        self.position_x.append(x_init)

        while H_new >= 0.02:

            Moth_pos = self.initialization(N, dim, ub, lb)
            Convergence_curve = np.zeros(Max_iteration)
            Iteration = 1
            lock_G4 = 0
            Moth_fitness = np.zeros(Moth_pos.shape[0])

            ### Main MFO loop ###

            while Iteration <= Max_iteration:

                # Number of flames Eq. (3.14) in the paper
                Flame_no = round(N - Iteration * ((N - 1) / Max_iteration))

                for i in range(Moth_pos.shape[0]):
                    # Check if moths go out of the search space and bring it back
                    Flag4ub = Moth_pos[i, :] > ub
                    Flag4lb = Moth_pos[i, :] < lb
                    Moth_pos[i, :] = (
                            Moth_pos[i, :] * (~(Flag4ub + Flag4lb)) + ub * Flag4ub + lb * Flag4lb
                    )

                    Moth_fitness[i], lock_G4, H_new = self.fitness_function_PP(Moth_pos[i, :], x_init, y_init, objects_x, objects_y, xend,
                                                                  yend, lock_G4)

                if Iteration == 1:
                    # Sort the first population of moths
                    fitness_sorted, I = np.sort(Moth_fitness), np.argsort(Moth_fitness)
                    sorted_population = Moth_pos[I, :]

                    # Update the flames
                    best_flames = sorted_population
                    best_flame_fitness = fitness_sorted
                else:
                    # Sort the moths
                    double_population = np.concatenate((previous_population, best_flames), axis=0)
                    double_fitness = np.concatenate((previous_fitness, best_flame_fitness))
                    I = np.argsort(double_fitness)
                    double_sorted_population = double_population[I, :]

                    fitness_sorted = double_fitness[I[:N]]
                    sorted_population = double_sorted_population[:N, :]

                    # Update the flames
                    best_flames = sorted_population
                    best_flame_fitness = fitness_sorted

                # Update the position best flame obtained so far
                Best_flame_score = fitness_sorted[0]
                Best_flame_pos = sorted_population[0, :]

                previous_population = Moth_pos
                previous_fitness = Moth_fitness

                # A linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
                a = -1 + Iteration * ((-1) / Max_iteration)

                for i in range(Moth_pos.shape[0]):
                    for j in range(Moth_pos.shape[1]):
                        if i <= Flame_no:  # Update the position of the moth with respect to its corresponsing flame
                            # D in Eq. (3.13)
                            distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                            b = 1
                            t = (a - 1) * np.random.rand() + 1

                            # Eq. (3.12)
                            Moth_pos[i, j] = (
                                    distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                    + sorted_population[i, j]
                            )
                        if i > Flame_no:  # Update the position of the moth with respct to one flame
                            # Eq. (3.13)
                            distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                            b = 1
                            t = (a - 1) * np.random.rand() + 1

                            # Eq. (3.12)
                            Moth_pos[i, j] = (
                                    distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                    + sorted_population[Flame_no, j]
                            )

                Convergence_curve[Iteration - 1] = Best_flame_score

                Iteration += 1
                # print(f"Iteration ==> {Iteration}")
                self.ys = Best_flame_pos
                self.fitVector = Convergence_curve

            x_init = self.ys[0] + x_init
            y_init = self.ys[1] + y_init
            self.position_y.append(y_init)
            self.position_x.append(x_init)
            # print(Max_experiment)

            ### Return the best position on the axis Y and X ###

        return self.position_x, self.position_y

    def MFO_Controller(self, position_robot, ideal_distance, actual_robot, neighbor_final):
        """"
        MFO_Controller, this function obtain the best position to keep the ideal position
        
        Arguments:
        
            position_robot (Float)        = Current position robot in the X or Y axis
            ideal_distance (Float)        = ideal position robot in the X or Y axis
            actual_robot   (Integer)      = current robot
            neighbor_final (Integer)      = Neighbot of the current robot
        
        Init Variables:
        
            N               = Number of particles
            dim             = Number of dimensions
            Max_iteration   = Number of iterations
            lb              = Init search space top
            ub              = Init search space bottom
        
        Return:
        
            self.ys[0] (Float) = Best postion in the X or Y axis
        
        """""

        ### Init MFO variables ###

        N = 5
        dim = 1
        Max_iteration = 50
        ub = 0.1
        lb = -0.1
        Moth_pos = self.initialization(N, dim, ub, lb)
        Convergence_curve = np.zeros(Max_iteration)
        Iteration = 1
        Moth_fitness = np.zeros(Moth_pos.shape[0])

        ### Main MFO loop ###

        while Iteration <= Max_iteration:

            # Number of flames Eq. (3.14) in the paper
            Flame_no = round(N - Iteration * ((N - 1) / Max_iteration))

            for i in range(Moth_pos.shape[0]):
                # Check if moths go out of the search space and bring it back
                Flag4ub = Moth_pos[i, :] > ub
                Flag4lb = Moth_pos[i, :] < lb
                Moth_pos[i, :] = (
                        Moth_pos[i, :] * (~(Flag4ub + Flag4lb)) + ub * Flag4ub + lb * Flag4lb
                )


                # Moth_fitness[i] = SISOFunct.EVA_Dynamic_PID(Moth_pos[i, :], time_simulation, Sample_time, sp)
                Moth_fitness[i] = self.function_swarm_controller_line(position_robot, ideal_distance, actual_robot, Moth_pos[i, :], neighbor_final)

            if Iteration == 1:
                # Sort the first population of moths
                fitness_sorted, I = np.sort(Moth_fitness), np.argsort(Moth_fitness)
                sorted_population = Moth_pos[I, :]

                # Update the flames
                best_flames = sorted_population
                best_flame_fitness = fitness_sorted
            else:
                # Sort the moths
                double_population = np.concatenate((previous_population, best_flames), axis=0)
                double_fitness = np.concatenate((previous_fitness, best_flame_fitness))
                I = np.argsort(double_fitness)
                double_sorted_population = double_population[I, :]

                fitness_sorted = double_fitness[I[:N]]
                sorted_population = double_sorted_population[:N, :]

                # Update the flames
                best_flames = sorted_population
                best_flame_fitness = fitness_sorted

            # Update the position best flame obtained so far
            Best_flame_score = fitness_sorted[0]
            Best_flame_pos = sorted_population[0, :]

            previous_population = Moth_pos
            previous_fitness = Moth_fitness

            # A linearly dicreases from -1 to -2 to calculate t in Eq. (3.12)
            a = -1 + Iteration * ((-1) / Max_iteration)

            for i in range(Moth_pos.shape[0]):
                for j in range(Moth_pos.shape[1]):
                    if i <= Flame_no:  # Update the position of the moth with respect to its corresponsing flame
                        # D in Eq. (3.13)
                        distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                        b = 1
                        t = (a - 1) * np.random.rand() + 1

                        # Eq. (3.12)
                        Moth_pos[i, j] = (
                                distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                + sorted_population[i, j]
                        )
                    if i > Flame_no:  # Update the position of the moth with respct to one flame
                        # Eq. (3.13)
                        distance_to_flame = abs(sorted_population[i, j] - Moth_pos[i, j])
                        b = 1
                        t = (a - 1) * np.random.rand() + 1

                        # Eq. (3.12)
                        Moth_pos[i, j] = (
                                distance_to_flame * np.exp(b * t) * np.cos(t * 2 * np.pi)
                                + sorted_population[Flame_no, j]
                        )

            Convergence_curve[Iteration - 1] = Best_flame_score

            Iteration += 1
            # print(f"Iteration ==> {Iteration}")
            self.ys = Best_flame_pos
            self.fitVector = Convergence_curve

        return self.ys[0]

    def fitness_function_PP(self, x, x_init, y_init, objects_x, objects_y, xend, yend, lock_G4):

        """"
        The ffitness_function_PP is the fitness function to PSO_PP.

        Arguments:

            x                   (Float)        = movement in the X and Y axis positions
            x_init              (Float)        = Init position of the robot in the X axis
            y_init              (Float)        = Init position of the robot in the Y axis
            objects_x           (Float List)   = List with all the position in the X axis of the objects
            objects_y           (Float List)   = List with all the position in the Y axis of the objects
            xend                (Float)        = End point of the robot in the X axis
            yend                (Float)        = End point of the robot in the Y axis
            lock_G4             (Integer)      = Lock for the PSO algorithm

        Return:

            f (Float)      = fitness function value
            lock_G4 (Integer) = Lock in the PSO algorithm 
            H_new (Float)  = Distance between the robot and the end point

        """""

        ### Psoible new position ###

        xnew = x_init + x[0]
        ynew = y_init + x[1]

        ### Read all the objects in the enviroment ####

        for k in range(len(objects_x)):

            ### If the posible new position is within the objects in the enviroment ###

            if ((xnew >= min(objects_x[k]) - self.Tolerance and xnew <= max(objects_x[k]) + self.Tolerance) and (
                    ynew >= min(objects_y[k]) - self.Tolerance and ynew <= max(objects_y[k]) + self.Tolerance)):
                G = 10000
                lock_G4 = 1

            ### If the posible new position is not within the objects in the enviroment ###

            else:
                if lock_G4 == 0:
                    G = 10
                else:
                    pass

        ### Calculate the fitness function f ###

        lock_G4 = 0
        distance = math.sqrt((xnew - xend) ** 2 + (ynew - yend) ** 2)
        H_new = (((distance * 3.1416) / 2) + distance)
        f = G + H_new - 10

        ### Return fitness function and the new distance between the robot and the end point ###

        return f, lock_G4, H_new

    def function_swarm_controller_line(self, robots, ideal, k, x, neighbor_final):

        """"
        The function_swarm_controller_line is the fitness function to PSO_Controller.

        Arguments:

            robots          (Integer)        = Position of the current robot in the X or Y axis
            ideal           (Float)          = End ideal position for the current robot in the X or Y axis
            k               (Integer)        = Current robot
            x               (Float list)     = Posible new position in the X or Y axis
            neighbor_final  (Float list)     = Neighbor to the current robot

        Return:

            f (Float) = Error between the posible new position and the ideal distance

        """""

        if k+1 == len(robots):
            pass
        else:
            ideal_distance_x = robots[k + neighbor_final + 1] - ideal
            distance_now_x = (robots[k] + x[0])
            sum_distances = (ideal_distance_x - distance_now_x)

        f = abs(sum_distances)

        return f