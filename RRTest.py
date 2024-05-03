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
from husky_control import CoordsToLocal
from husky_control import CoordsToGlobal
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from husky_traj import interpTraj
from husky_traj import SplinTraj
import matplotlib.pyplot as plt
import time
import json


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
    vs = 0.1
    # get_lidar_data(lidar, base)
    dest = [-15.05, 57.05, 0]  # dist=59
    inicio = [0, 0, 0]
    arrx = np.linspace(0, stop=-15.05, num=15)
    arry = np.linspace(0, 57.05, 15)
    goal = []
    # for i in range(len(arrx) - 1):
    #     goal.append([arrx[i + 1], arry[i + 1], 0])
    traj = [[0.0, 0.0, 0.243], [-3, 11.1, 0.243], [8.725, 14.00, 0.243], [2.5, 25.25, 0.243]]
    newtraj = interpTraj(traj, 3)
    goal = SplinTraj(newtraj, traj)

    for cord in goal:
        plt.figure(7)
        plt.title('Nodos de la trayectoria', fontsize=20)
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')
        plt.scatter(cord[0], cord[1])
    plt.show()
    tiempos = []
    for i in range(len(goal)):
        data_lidar = get_lidar_data(lidar, base, vs)
        start = time.process_time()
        li_traj, valid, _, _ = connect_to_goal(data_lidar, goal[i], base.get_position(), vs)
        end = time.process_time()
        tiempos.append(end - start)
        if not valid:
            continue
        # for j in range(len(li_traj)):
        #     li_traj[2] = 0.243
        gotoDef2(robot=robot, puntos=li_traj, base=base, velocity=0.3, CSR=True, smoothing=True)
    # data2 = get_lidar_data(lidar, base)
    print("Tiempo de proceso:", tiempos)

    simulation.stop()


def SimulLocal ():
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
    vs = 0.3
    # get_lidar_data(lidar, base)

    traj = [[0.0, 0.0, 0.243], [-3, 11.1, 0.243], [8.725, 14.00, 0.243], [2.5, 25.25, 0.243]]
    newtraj = interpTraj(traj, 3)
    goal = SplinTraj(newtraj, traj)

    for cord in goal:
        plt.figure(7)
        plt.title('Nodos de la trayectoria', fontsize=20)
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')
        plt.scatter(cord[0], cord[1])
    plt.show()
    tiempos = []
    for i in range(len(goal)):
        data_lidar = get_lidar_data(lidar, base, vs)
        obj = CoordsToLocal(base, goal[i])
        start = time.process_time()
        li_traj, valid, _, _ = connect_to_goal(data_lidar, obj, base.get_position(), vs)
        end = time.process_time()
        glb_traj = []
        for punto in li_traj:
            glb_traj.append(CoordsToGlobal(base, punto))
        tiempos.append(end - start)
        if not valid:
            continue
        # for j in range(len(li_traj)):
        #     li_traj[2] = 0.243
        gotoDef2(robot=robot, puntos=glb_traj, base=base, velocity=0.3, CSR=True, smoothing=True)
    # data2 = get_lidar_data(lidar, base)
    print("Tiempo de proceso:", tiempos)

    simulation.stop()


def get_lidar_data (lidar, base, vs):
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
    pcl.downsample(voxel_size=vs)
    T_base = base.get_transform()
    # T2 = HomogeneousMatrix(T_base.pos(), T_base.euler()[0])
    T2 = HomogeneousMatrix([0, 0, 0], T_base.euler()[0])
    mrot = RotationMatrix([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    meuler = mrot.euler()[0]
    # T3 = HomogeneousMatrix([0, 0, 0.682], meuler)
    T3 = HomogeneousMatrix([0, 0, 0.925], meuler)
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


def simulate2 ():
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
    # get_lidar_data(lidar, base)
    goal = [[-2.5, 9, 0.243]]
    totalTiempos = []
    for ise in range(1, 6, 1):
        tiempos = []
        it = []
        g_r = []
        for idx in range(10):
            vs = ise / 10
            print(vs)
            data_lidar = get_lidar_data(lidar, base, vs)
            start = time.time()
            li_traj, valid, iteraciones, reached = connect_to_goal(data_lidar, goal[0], base.get_position(), vs)
            end = time.time()
            if not reached:
                print('Goal not reached')
                g_r.append(False)
                continue
            tiempos.append(end - start)
            it.append(iteraciones)
            g_r.append(True)
        if tiempos == []:
            for j in range(10):
                tiempos.append(0)
                it.append(0)

        totalTiempos.append([tiempos, it, g_r])
        print("Tiempo de proceso:", tiempos)
    try:
        with open('C:/Users/User/pruebasTFG/config/DatosVS3.json', 'w') as file:
            json.dump(totalTiempos, file)
    except:
        print("JSON loading error!...")

    try:
        with open('C:/Users/User/pruebasTFG/config/DatosVS3.json', 'r') as file:
            data = json.load(file)
    except:
        print("JSON reading error!...")

    for i in range(5):
        media_tiempo = np.mean(data[i][0])
        max_tiempo = max(data[i][0])
        min_tiempo = min(data[i][0])
        media_iter = np.mean(data[i][1])
        max_iter = max(data[i][1])
        min_iter = min(data[i][1])
        aciertos = np.mean(data[i][2])
        # print(totalTiempos[i])
        print(f'Voxel_size={(i + 1) / 10}')
        print(f'Media de tiempo:{media_tiempo:.3f}, Media de iteraciones:{media_iter}, Goal reached:{aciertos * 100}%')
        print(f'Tiempo Max:{max_tiempo:.3f}, Min:{min_tiempo:.3f}; Iteraciones Max:{max_iter}, Min:{min_iter}')

    simulation.stop()


def plot_Data ():
    tiempo = [[], [], [], [], []]
    iteraciones = [[], [], [], [], []]
    g_r = [[], [], [], [], []]
    global_data = [tiempo, iteraciones, g_r]
    voxels = ['0,1', '0,2', '0,3', '0,4', '0,5']

    def BarComparison (j, mdata):
        if j == 0:
            aux = 'Time'
            aux2 = 'Time(s)'
        else:
            aux = 'Iterations'
            aux2 = aux

        colors = ['r', 'g', 'b']
        my_data = [[np.mean(i[j]) for i in mdata[:5]], [max(i[j]) for i in mdata[:5]], [min(i[j]) for i in mdata[:5]]]
        X_axis = np.arange(len(voxels))
        plt.bar(X_axis - 0.2, my_data[0], label='Mean', color=colors[0], width=0.2)
        plt.bar(X_axis, my_data[1], label='Max', color=colors[1], width=0.2)
        plt.bar(X_axis + 0.2, my_data[2], label='Min', color=colors[2], width=0.2)
        plt.xticks(X_axis, voxels)
        plt.legend()
        plt.ylabel(aux2)
        plt.xlabel('Voxel size')
        plt.title(aux + ' performance', fontsize='xx-large')

    for idx in range(1, 4, 1):
        try:
            with open('C:/Users/User/pruebasTFG/config/DatosVS' + str(idx) + '.json', 'r') as file:
                data = json.load(file)
        except:
            print("JSON reading error!...")

        auxlist = []
        j = 0
        for k in data[:5]:
            tiempo[j] = tiempo[j] + k[0]
            iteraciones[j] = iteraciones[j] + k[1]
            g_r[j] = g_r[j] + k[2]
            j += 1
        print('a', tiempo)
        plt.figure(1)
        plt.bar(voxels, [sum(i[2]) * 10 for i in data[:5]])
        plt.ylabel('Goal reached (%)')
        plt.xlabel('Voxel size')
        plt.title('Goal reached', fontsize='xx-large')

        plt.figure(2)
        BarComparison(0, data)
        plt.figure(3)
        BarComparison(1, data)

        plt.show()
    global_data = [tiempo, iteraciones, g_r]

    plt.figure(6)
    plt.bar(voxels, [sum(i) * 100 / len(i) for i in g_r[:5]])
    plt.ylabel('Goal reached (%)')
    plt.xlabel('Voxel size')
    plt.title('Goal reached', fontsize='xx-large')
    plt.figure(4)
    X_axis = np.arange(len(voxels))
    plt.bar(X_axis - 0.2, [np.mean(i) for i in tiempo[:5]], label='Mean', color='r', width=0.2)
    plt.bar(X_axis, [max(i) for i in tiempo[:5]], label='Max', color='g', width=0.2)
    plt.bar(X_axis + 0.2, [min(i) for i in tiempo[:5]], label='Min', color='b', width=0.2)
    plt.xticks(X_axis, voxels)
    plt.legend()
    plt.ylabel('Tiempo(s)')
    plt.xlabel('Voxel size')
    plt.title('Time' + ' performance', fontsize='xx-large')

    plt.figure(5)
    X_axis = np.arange(len(voxels))
    plt.bar(X_axis - 0.2, [np.mean(i) for i in iteraciones[:5]], label='Mean', color='r', width=0.2)
    plt.bar(X_axis, [max(i) for i in iteraciones[:5]], label='Max', color='g', width=0.2)
    plt.bar(X_axis + 0.2, [min(i) for i in iteraciones[:5]], label='Min', color='b', width=0.2)
    plt.xticks(X_axis, voxels)
    plt.legend()
    plt.ylabel('Iterations')
    plt.xlabel('Voxel size')
    plt.title('Iterations' + ' performance', fontsize='xx-large')
    plt.show()
    mt = np.mean(tiempo[2])
    mi = np.mean(iteraciones[2])
    ma = sum(g_r[2]) * 100 / len(g_r[2])
    print(f'Media de tiempo para voxel size=0.3: {mt:.3f}s')
    print(f'Media de iteraciones para voxel size=0.3:    {int(mi)} ')
    print(f'Porcentaje de aciertos para voxel size=0.3:   {ma}%')


if __name__ == "__main__":
    # plot_Data()
    # simulate2()
    SimulLocal()
