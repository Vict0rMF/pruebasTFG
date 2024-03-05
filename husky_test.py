
from robots.simulation import Simulation
from robots.husky import HuskyRobot
from robots.objects import CoppeliaObject
import numpy as np
import matplotlib.pyplot as plt
from practicals.kinematics.husky_control import goto
from practicals.kinematics.husky_control import sigue_puntos
from practicals.kinematics.husky_traj import convertCoordinates
from practicals.kinematics.husky_traj import interpTraj
from main import getTrajNodes



def empezar ():
    # Start simulation
    simulation = Simulation()
    simulation.start()
    print('va')
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    robot.getParams()
    base = CoppeliaObject(simulation=simulation)
    base.start(name='/CentroHusky')
    coord0 = [38.275401, -0.686178]
    destination_point = [38.276374, -0.685745]

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

    newtraj=interpTraj(traj,4)
    # newtraj = traj
    for cord in newtraj:
        plt.figure(7)
        plt.scatter(cord[0], cord[1])

    plt.show()
    print('traj', traj)

    plt.plot(trajx, trajy, label='Desired trajectory')

    # print(lista)
    # # robot.sigue_puntos(robot,base,lista)
    del newtraj[0]

    goto(robot=robot,puntos=newtraj,base=base)

    plt.figure(1)
    # plt.plot(trayectx,trayecty, label='Trayectoria deseada')
    plt.legend(loc='upper left')

    plt.show()
    robot.wait(500)
    simulation.stop()


if __name__ == "__main__":
    empezar()
