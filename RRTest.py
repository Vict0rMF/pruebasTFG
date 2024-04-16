#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
"""
from robots.ouster import Ouster
from robots.simulation import Simulation
from husky import HuskyRobot
import numpy as np
from robots.objects import CoppeliaObject
from keyframe.keyframe import KeyFrame
from run_rrt_planner_pc import connect_to_goal
from husky_control import gotoDef2
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from husky_traj import interpTraj
from husky_traj import SplinTraj
import matplotlib.pyplot as plt

def simulate ():
    # Start simulation

    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # robot.getParams()
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')

    base = CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')
    get_lidar_data(lidar,base)
    dest = [-15.05, 57.05, 0]  # dist=59
    inicio = [0, 0, 0]
    arrx = np.linspace(0, stop=-15.05, num=15)
    arry = np.linspace(0, 57.05, 15)
    goal = []
    # for i in range(len(arrx) - 1):
    #     goal.append([arrx[i + 1], arry[i + 1], 0])
    traj = [[0.0,0.0,0.243],[-3, 11.1, 0.243], [8.725, 14.00, 0.243], [2.5, 25.25, 0.243]]
    newtraj = interpTraj(traj, 3)
    goal = SplinTraj(newtraj, traj)

    for cord in goal:
        plt.figure(7)
        plt.title('Nodos de la trayectoria', fontsize=20)
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')
        plt.scatter(cord[0], cord[1])
    plt.show()

    for i in range(len(goal)):
        data_lidar = get_lidar_data(lidar, base)
        li_traj, valid = connect_to_goal(data_lidar, goal[i], base.get_position())
        if not valid:
            continue
        # for j in range(len(li_traj)):
        #     li_traj[2] = 0.243
        gotoDef2(robot=robot, puntos=li_traj, base=base, velocity=0.3, CSR=True, smoothing=True)
    data2 = get_lidar_data(lidar, base)

    simulation.stop()


def get_lidar_data (lidar, base):
    # get lidar data

    data = lidar.get_laser_data()

    print('Received Laser Data')
    try:
        print(data.shape)
        print(data.dtype)
    except:
        print('Unknown data type')
    print(data)
    pcl = KeyFrame()
    pcl.from_points(data)
    pcl.downsample(voxel_size=0.05)
    T_base = base.get_transform()
    T2 = HomogeneousMatrix(T_base.pos(), T_base.euler()[0])
    mrot = RotationMatrix([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    meuler = mrot.euler()[0]
    T3 = HomogeneousMatrix([0, 0, 0.682], meuler)
    tx = T2 * T3
    # T = np.array([[0, 0, 1, 0],
    #               [1, 0, 0, 0],
    #               [0, 1, 0, 0.925],
    #               [0, 0, 0, 1]])
    # T = np.array([[1, 0, 0, 0],
    #               [0, 1, 0, 0],
    #               [0, 0, 1, 0.925],
    #               [0, 0, 0, 1]])

    # pcl.draw_cloud_plt()
    pcl.transform(tx.toarray())
    # pcl.draw_cloud()
    pcl.draw_cloud_plt()
    return np.asarray(pcl.pointcloud.points)

def simulate2():
    # Start simulation

    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # robot.getParams()
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')

    base = CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')
    get_lidar_data(lidar, base)
    goal = [[2.525, 3.2, 0.243], [-0.025, 7.325, 0.243]]

    for i in range(len(goal)):
        data_lidar = get_lidar_data(lidar, base)
        li_traj, valid = connect_to_goal(data_lidar, goal[i], base.get_position())
        if not valid:
            continue
        for j in range(len(li_traj)):
            li_traj[2] = 0.243
        gotoDef2(robot=robot, puntos=li_traj, base=base, velocity=0.3, CSR=True, smoothing=True)
    data2 = get_lidar_data(lidar, base)

    simulation.stop()


if __name__ == "__main__":
    simulate()

