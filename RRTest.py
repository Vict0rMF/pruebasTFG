#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
"""
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot
import numpy as np
from robots.objects import CoppeliaObject
from keyframe.keyframe import KeyFrame

def simulate():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')

    base= CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')

    # get lidar data
    data = lidar.get_laser_data()
    print('Received Laser Data')
    try:
        print(data.shape)
        print(data.dtype)
    except:
        print('Unknown data type')

    pcl = KeyFrame()
    pcl.from_points(data)
    pcl.save_pointcloud('Pointcloud/pruebaCoppelia3.pcd')

    simulation.stop()


if __name__ == "__main__":
    simulate()