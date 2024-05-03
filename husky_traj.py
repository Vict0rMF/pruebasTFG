import numpy as np
import matplotlib.pyplot as plt
import utm
import scipy.interpolate as spi


def convertCoordinates (coord0, coord2):
    base = utm.from_latlon(coord0[0], coord0[1])
    dest = utm.from_latlon(coord2[1], coord2[0])
    x = dest[0] - base[0]
    y = dest[1] - base[1]
    print(x, '   ', y)
    return [x, y]


def interpTraj (list, sep):
    newlist = []
    for i in range(len(list) - 1):
        print(f'punto{i}--{i + 1}')
        dist = np.sqrt((list[i][0] - list[i + 1][0]) ** 2 + (list[i][1] - list[i + 1][1]) ** 2)
        print('distancia', dist)
        if dist < 2 * sep:
            print('nada')
            newlist.append(list[i])
        else:
            step = round(dist / sep) + 1
            print('step', step)
            x_p = [list[i][0], list[i + 1][0]]
            y_p = [list[i][1], list[i + 1][1]]
            x_interp = np.linspace(x_p[0], x_p[1], step)
            y_linear = spi.interp1d(x_p, y_p, kind='linear')
            y_interp = y_linear(x_interp)
            j = 0
            for j in range(len(x_interp)):
                x = x_interp[j]
                y = y_interp[j]
                z = 0.243
                newlist.append([x, y, z])
            del newlist[-1]
    newlist.append(list[len(list) - 1])
    return newlist


def interpSpline (list):
    num=5
    newlist = []
    print('------------------')
    xlist = []
    ylist = []
    for i in range(len(list)):
        xlist.append(list[i][0])
        ylist.append(list[i][1])

    nose, *rest = spi.splprep([xlist, ylist], k=2)
    # print(nose)
    x_interp = np.linspace(0, 1, num)
    salida = spi.splev(x_interp, nose)
    # print(salida)
    for j in range(len(x_interp)):
        x = salida[0][j]
        y = salida[1][j]
        # plt.scatter(x,y)
        z = 0.243
        newlist.append([x, y, z])
    # plt.show()

    return newlist


def SplinTraj (lista, nodos):
    puntos=1
    for nodo in nodos[1:-1]:
        index = lista.index(nodo)
        print(index)
        # del lista[index]
        auxlist = []
        for i in range(-puntos, puntos+1, 1):
            auxlist.append([lista[index + i][0], lista[index + i][1], lista[index + i][2]])
        newlist = interpSpline(auxlist)
        del lista[index - puntos:index + puntos+1]
        # for k in range(len(newlist)):
        #     plt.scatter(newlist[k][0], newlist[k][1])
        # plt.show()
        for j in range(-puntos, len(newlist) - puntos, 1):
            lista.insert(index + j, [newlist[puntos + j][0], newlist[puntos + j][1], newlist[puntos + j][2]])
    del lista[0]
    return lista


def test (robot, base):
    time = 0
    j = 0
    listV = []
    listW = []
    for j in range(85):
        vel, ang = base.getVelocity()
        vel = np.linalg.norm(vel)
        ang = ang[2]
        listV.append(vel)
        listW.append(ang)
        robot.time.append(time)
        time += 0.05
        robot.move(0.3, 0.0)
        robot.wait()

    j = 0
    for j in range(250):
        vel, ang = base.getVelocity()
        vel = np.linalg.norm(vel)
        ang = ang[2]
        listV.append(vel)
        listW.append(ang)
        robot.time.append(time)
        time += 0.05
        robot.move(0.3, 0.8)
        robot.wait()

    j = 0
    # for j in range(300):
    #     vel, ang = base.getVelocity()
    #     vel = np.linalg.norm(vel)
    #     ang = ang[2]
    #     listV.append(vel)
    #     listW.append(ang)
    #     robot.time.append(time)
    #     time += 0.05
    #     robot.move(0.3, -0.8)
    #     robot.wait()
    plt.figure(3)
    plt.plot(robot.time, listV)
    plt.title('Velocidad lineal', fontsize=20)
    plt.ylabel('V(m/s)')
    plt.xlabel('time(s)')
    plt.figure(4)
    plt.plot(robot.time, listW)
    plt.title('Velocidad angular', fontsize=20)
    plt.ylabel('w(rad/s)')
    plt.xlabel('time(s)')
    plt.show()
