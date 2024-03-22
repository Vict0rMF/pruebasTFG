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
        # self.Vmax=0
        # self.Wmax=0
        # self.Vmin=0
        # self.amax=0
        # self.alphamax=0
        self.Vmax = 0.5
        self.Wmax = np.pi / 4
        self.Vmin = 0.1
        self.amax = 0.1
        self.alphamax = 0.2
        self.x = []
        self.y = []
        self.V = 0
        self.W = 0
        self.omega = []
        self.time = []
        self.simulation.sim.setObjectFloatParam(self.joints[0], self.simulation.sim.jointfloatparam_maxaccel, self.amax)
        self.simulation.sim.setObjectFloatParam(self.joints[1], self.simulation.sim.jointfloatparam_maxaccel, self.amax)
        self.simulation.sim.setObjectFloatParam(self.joints[2], self.simulation.sim.jointfloatparam_maxaccel, self.amax)
        self.simulation.sim.setObjectFloatParam(self.joints[3], self.simulation.sim.jointfloatparam_maxaccel, self.amax)

    def calcVW (self):
        [wl, nada, wr, nada2] = self.get_joint_speeds()
        V = (self.wheel_radius * (wr + wl)) / 2

        W = (self.wheel_radius * (wr - wl)) / self.width
        return V, W

    def move (self, v, w):
        v, w = self.accelcontol(v, w)
        self.V = v
        self.W = w
        r = self.wheel_radius
        b = self.width
        wl = (v - w * (b / 2)) / r
        wr = (v + w * (b / 2)) / r

        self.simulation.sim.setJointTargetVelocity(self.joints[0], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], wl)
        self.simulation.sim.setJointTargetVelocity(self.joints[2], wr)
        self.simulation.sim.setJointTargetVelocity(self.joints[3], wr)

    def getParams (self):
        print('hola')
        # Getting data from config.yaml
        try:
            with open(r'C:/Users/User/pruebasTFG/config/Husky_Config.yaml') as file:
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
            self.Vmax = float(self.Vmax)
            self.Wmax = float(self.Wmax)
            self.Vmin = float(self.Vmin)
            self.amax = float(self.amax)
            self.alphamax = float(self.alphamax)

        except:
            print("Error getting params from config.YAML!...")

    def checkmax (self, V, w):

        Vout = np.clip(V, self.Vmin, self.Vmax)
        wout = np.clip(w, -self.Wmax, self.Wmax)
        return Vout, wout

    def accelcontol (self, Vcom, Wcom):
        delta_time = 0.05
        a = (Vcom - self.V) / delta_time
        ar = np.clip(a, -self.amax, self.amax)
        v = delta_time * ar + self.V
        alpha = (Wcom - self.W) / delta_time
        alphar = np.clip(alpha, -self.alphamax, self.alphamax)
        w = delta_time * alphar + self.W
        print(alpha)
        return v, w

   #  Añadir a robots.objects
   # def getVelocity(self):
   #      linear_velocity = []
   #      angular_velocity = []
   #      linear_velocity,angular_velocity = self.simulation.sim.getObjectVelocity(self.handle)
   #      return linear_velocity,angular_velocity