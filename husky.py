#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez
@Time: February 2024
"""
from robots.robot import Robot
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
import matplotlib.pyplot as plt
import yaml


class HuskyRobot(Robot):
    def __init__ (self, simulation):
        Robot.__init__(self, simulation=simulation)

    def start (self, base_name='/HUSKY'):
        robotbase = self.simulation.sim.getObject(base_name)
        wheelRL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRLW')
        wheelFL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFLW')
        wheelRR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRRW')
        wheelFR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFRW')
        wheeljoints = []
        wheeljoints.append(wheelRL)
        wheeljoints.append(wheelFL)
        wheeljoints.append(wheelRR)
        wheeljoints.append(wheelFR)
        self.joints = wheeljoints
        self.width = 0.555
        self.wheel_radius = 0.165
        # self.Vmax = 1
        # self.Wmax = np.pi / 2
        # self.Vmin=0.1
        # self.amax=0
        # self.alphamax=0
        self.x = []
        self.y = []
        self.V = []
        self.W = []
        self.omega = []

    def chackmax (self,V, w):
        vmax=self.Vmax
        wmax=self.Wmax
        vmin=self.Vmin
        if V > vmax:
            V = vmax
        elif V < vmin:
            V = vmin
        if w > wmax:
            w = wmax
        elif w < -wmax:
            w = -wmax
        return V, w

    def calcVW (self):
        [wl, nada, wr, nada2] = self.get_joint_speeds()
        V = (self.wheel_radius * (wr + wl)) / 2

        W = (self.wheel_radius * (wr - wl)) / self.width
        return V, W

    def move (self, v, w):
        r = self.wheel_radius
        b = self.width
        wl = (v - w * (b / 2)) / r
        wr = (v + w * (b / 2)) / r

        self.simulation.sim.setJointTargetVelocity(self.joints[0], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[2], wr)
        self.simulation.sim.setJointTargetVelocity(self.joints[3], wr)


    def getParams(self):
        print('hola')
        # Getting data from config.yaml
        try:
            with open(r'C:/Users/User/Desktop/tfg/config/prueba.yaml') as file:
                param_list = yaml.load(file, Loader=yaml.FullLoader)
                print(param_list)
        except:
            print("YAML loading error!...")
        try:
            self.Vmax = param_list.get('Vmax')
            self.Wmax = param_list.get('Wmax')
            self.Vmin = param_list.get('Vmin')
            self.amax = param_list.get('amax')
            self.alphamax = param_list.get('alphamax')
            print(f'{self.Vmax=},{self.Wmax=},{self.Vmin=},{self.amax=},{self.alphamax=}')

        except:
            print("Error getting params from config.YAML!...")
