import numpy as np
import matplotlib.pyplot as plt
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix


def mov_orientacion (robot, base, objetivo):
    vel_ang = 0.25
    T_base = base.get_transform()
    orientacion = T_base.euler()[0]
    final = orientacion.abg[2] + objetivo
    print('ver aqui')
    print(final)
    while True:
        T_base = base.get_transform()
        orientacion = T_base.euler()[0]
        print('-------------')
        print(orientacion.abg[2])
        e = np.linalg.norm(final - orientacion.abg[2])
        print(e)
        robot.V.append(0.0)

        if e < 0.01:
            robot.move(v=0, w=0)
            break
        if objetivo > np.pi:
            robot.move(v=0, w=-vel_ang)
            robot.W.append(-vel_ang)
        elif np.pi > objetivo >= 0:
            robot.move(v=0, w=vel_ang)
            robot.W.append(vel_ang)
        elif -np.pi < objetivo < 0:
            robot.move(v=0, w=-vel_ang)
            robot.W.append(-vel_ang)
        else:
            robot.move(v=0, w=vel_ang)
            robot.W.append(vel_ang)
        robot.wait()


def mov_posicion (robot, base, objetivo):
    vel = 0.3
    while True:
        robot.move(v=vel, w=0)
        robot.V.append(vel)
        robot.W.append(0)
        T_base = base.get_transform()
        posicion = T_base.pos()
        robot.x.append(posicion[0])
        robot.y.append(posicion[1])
        print('posac')
        print(posicion)

        e = np.linalg.norm(objetivo - posicion)
        print(e)
        if e < 0.3:
            robot.move(v=0, w=0)
            break
        robot.wait()


def sigue_puntos (robot, base, puntos):
    print(len(puntos))
    for i in range(len(puntos)):
        print(puntos[i])
        T_base = base.get_transform()
        Tfinal = HomogeneousMatrix((puntos[i]), Euler([0, 0, 0]))
        Tinicio = HomogeneousMatrix(T_base.pos(), Euler([0, 0, 0]))
        tx = Tinicio.inv() * Tfinal
        print('---------------------Tfinal---------------------------')
        print(Tfinal)
        print('---------------------Tinicio---------------------------')
        print(Tinicio)
        print('---------------------TX--------------------------')
        print(tx)
        print('-------------------------------------------------------')
        print(np.arctan2(tx.pos()[1], tx.pos()[0]))
        print(T_base.euler()[0].abg[2])
        angulo = np.arctan2(tx.pos()[1], tx.pos()[0]) - T_base.euler()[0].abg[2]
        print('angulo a girar')
        print(angulo)
        mov_orientacion(robot, base, angulo)
        mov_posicion(robot, base=base, objetivo=puntos[i])

    plt.figure(1)
    plt.title('Posición Husky', fontsize=30)
    plt.xlabel('x(m)')
    plt.xlabel('y(m)')
    plt.plot(robot.x, robot.y, label='Trayectoria del Husky')
    plt.figure(2)
    plt.title('Velocidad lineal', fontsize=30)
    plt.xlabel('V(m/s)')
    plt.plot(robot.V)
    plt.figure(3)
    plt.title('Velocidad angular', fontsize=30)
    plt.xlabel('w(rad/s)')
    plt.plot(robot.W)


def goto (robot, base, puntos):
    k1 = 0.04
    h = 0.1
    for punto in puntos:
        xdes = punto[0]
        ydes = punto[1]
        vdes = 0.1
        while (np.linalg.norm(punto - base.get_transform().pos())) > 0.7:
            print('error', (np.linalg.norm(punto - base.get_transform().pos())))
            T_base = base.get_transform()
            om = T_base.euler()[0].abg[2]
            pos = T_base.pos()
            robot.x.append(pos[0])
            robot.y.append(pos[1])

            jacob = np.array([(np.cos(om), -h * np.sin(om)), (np.sin(om), h * np.cos(om))])
            ux = - k1 * (pos[0] - xdes)
            uy = - k1 * (pos[1] - ydes)
            u = np.array([(ux), (uy)])
            [V, w] = np.linalg.inv(jacob) @ u
            V = V + vdes
            V, w = robot.chackmax(V,w)
            robot.V.append(V)
            robot.W.append(w)
            print('V', V, 'w', w)
            robot.move(V, w)
            robot.wait()
    plt.figure(1)
    plt.title('Posición Husky', fontsize=30)
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    plt.plot(robot.x, robot.y, label='Trayectoria del Husky')
    plt.figure(2)
    plt.title('Velocidad lineal', fontsize=30)
    plt.ylabel('V(m/s)')
    plt.plot(robot.V)
    plt.figure(3)
    plt.title('Velocidad angular', fontsize=30)
    plt.ylabel('w(rad/s)')
    plt.plot(robot.W)
