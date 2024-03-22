from robots.simulation import Simulation
from husky import HuskyRobot
from robots.objects import CoppeliaObject
import numpy as np
import matplotlib.pyplot as plt
from husky_control import goto5
from husky_control import gotoDef2
from practicals.kinematics.husky_control import sigue_puntos
from husky_traj import convertCoordinates
from husky_traj import interpTraj
from husky_traj import interpSpline
from husky_traj import SplinTraj
from main import getTrajNodes
from husky_traj import test


def empezar ():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    print('va')
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # robot.getParams()
    base = CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')
    # test(robot,base)
    coord0 = [38.275401, -0.686178]
    # destination_point = [38.276374, -0.685745]  # original
    # destination_point = [38.2754855, -0.6857143] #primero
    destination_point = [38.2763131, -0.6866021] #curv
    # destination_point = [38.277182, -0.6851315]
    data = getTrajNodes(coord0, destination_point)
    print('data', data)
    traj = []
    trajx = []
    trajy = []
    for i in range(len(data)):
        x, y = convertCoordinates(coord0, data[i])
        z = 0.243
        trajx.append(x)
        trajy.append(y)
        traj.append([x, y, z])

    newtraj = interpTraj(traj, 1)
    newtraj = SplinTraj(newtraj, traj)
    # newtraj = interpSpline(traj)
    for cord in newtraj:
        plt.figure(7)
        plt.title('Nodos de la trayectoria', fontsize=20)
        plt.xlabel('x(m)')
        plt.ylabel('y(m)')
        plt.scatter(cord[0], cord[1])

    plt.show()
    plt.plot(trajx, trajy, label='Desired trajectory', linewidth=8)
    del newtraj[0]

    gotoDef2(robot=robot, puntos=newtraj, base=base, velocity=0.3, CSR=True, smoothing=True)

    plt.figure(1)
    plt.legend(loc='upper left')

    plt.show()
    simulation.stop()


if __name__ == "__main__":
    empezar()
