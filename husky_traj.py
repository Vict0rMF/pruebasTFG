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

    newlist.append(list[len(list) - 1])
    return newlist


def Circular ():
    lista = []
    an = 0
    r = 5
    while an < np.pi * 2:
        lista.append(np.array([r * np.cos(an), r * np.sin(an), 0.243]))
        an += np.pi / 16
    return lista


def zigzag ():
    lista = []
    an = 0
    r = 1
    lon = 2
    while an < np.pi * 2:
        lista.append(np.array([lon, r * np.cos(an), 0.2433]))
        an += np.pi / 4
        lon += 1
    return lista


def recta ():
    lista = []
    lon = 0
    while lon < 20:
        lista.append(np.array([0, lon, 0.2556]))
        lon += 1
    return lista

    # fig, axs = plt.subplots(2, 1)
    # for i in range(1, 6):
    #     robot.move(i / 10, 0)
    #     j = 0
    #     listV = []
    #     listW = []
    #     for j in range(150):
    #         [wl, nada, wr, nada2] = robot.get_joint_speeds()
    #         Va, Wa = robot.calcVW()
    #         listV.append(Va)
    #         listW.append(Wa)
    #         robot.wait()
    #     robot.move(0, 0)
    #     j = 0
    #     for j in range(150):
    #         robot.wait()
    #     axs[0].plot(listV)
    #     axs[1].plot(listW)
