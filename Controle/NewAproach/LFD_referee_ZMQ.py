import math
import numpy as np
import ANNClass as ann
import pandas as pd
import Bioinspired as bia
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import keyboard as kbd
from DrawField import plot_robot_path



class LFD:
    def __init__(self, desired_pos=None, filename='wheelSpeeds'):
        if desired_pos is None:
            desired_pos = [0.6,0.6]
        self.desired_pos = desired_pos
        self.v_max = 20
        self.v_min = -20
        self.v_linear = 0
        self.omega = 0
        self.iterations_num = 0
        self.phi = 0
        
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

    def StartSimulation(self):
        self.sim.startSimulation()
        print("============Simulation Started============")

    def StopSimulation(self):
        self.sim.stopSimulation()
        print("============Simulation Stopped============")


    def SpeedCRB(self, U, omega):
        vd = (2 * U + omega * 7.5) / 3.2
        ve = (2 * U - omega * 7.5) / 3.2

        vd = max(min(vd, self.v_max), self.v_min)
        ve = max(min(ve, self.v_max), self.v_min)

        return ve, vd

    def Demonstration(self):
        Time_Sample = []
        interror_phi = 0
        Integral_part_phi = 0
        fant_phi = 0
        cont0 = 0
        robot_pos = []
        simulation = True
        state = 0
        ubehavior = []
        self.desired_pos = [-0.5, 0]

        self.sim.startSimulation()
        robot = self.sim.getObject('/robot01')
        motorE = self.sim.getObject('/motorL01')
        motorD = self.sim.getObject('/motorR01')
        ball = self.sim.getObject('/ball')

        print("====================Demonstração====================")
        print("Control the robot using arrow keys")
        while simulation:

            robot_pos = self.sim.getObjectPosition(robot, self.sim.handle_world)
            robot_ang = self.sim.getObjectOrientation(robot, self.sim.handle_world)
            self.phi = robot_ang[2]

            if state == 0:
                self.desired_pos = [-0.5, 0]
            if state == 1:
                self.desired_pos = [0, 0]
            if state == 2:
                self.desired_pos = [0.5, 0]
            if state == 3:
                self.desired_pos = [0.5, -0.5]
            if state == 4:
                simulation = False

            phid = math.atan2(self.desired_pos[1] - robot_pos[1], self.desired_pos[0] - robot_pos[0])

            error_phi = phid - self.phi
            
            if kbd.is_pressed('up'):
                self.v_linear = min(self.v_linear + 0.5, self.v_max)
                ubehavior.append(0)
            if kbd.is_pressed('down'):
                self.v_linear = max(self.v_linear - 0.5, self.v_min)
                ubehavior.append(3)
            if kbd.is_pressed('left'):
                self.omega = min(self.omega + 0.1, 5.0)  # Max angular velocity
                ubehavior.append(2)
            if kbd.is_pressed('right'):
                self.omega = max(self.omega - 0.1, -5.0)  # Min angular velocity
                ubehavior.append(1)

            error_distance = math.sqrt((self.desired_pos[1] - robot_pos[1]) ** 2 + (self.desired_pos[0] - self.desired_pos[0]) ** 2)


            ve, vd = self.SpeedCRB(self.v_linear, self.omega)
            self.sim.setJointTargetVelocity(motorE, ve)
            self.sim.setJointTargetVelocity(motorD, vd)

