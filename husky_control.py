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
        print('posac')
        print(posicion)

        e = np.linalg.norm(posicion - objetivo)
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
            V, w = robot.chackmax(V, w)
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


def checkturn (i, puntos, base):
    T_base = base.get_transform()
    pos = T_base.pos()
    if (i == 0 or i == len(puntos) - 1 or i == 1 or i == 2):
        return False
    preang = np.arctan2(pos[1] - puntos[i - 1][1], pos[0] - puntos[i - 1][0])
    print(f'{preang=}')
    nextang = np.arctan2(puntos[i + 1][1] - pos[1], puntos[i + 1][0] - pos[0])
    print(f'{nextang=}')
    dif = nextang - preang
    print(f'{dif=}')

    preang2 = np.arctan2(puntos[i - 1][1] - puntos[i - 2][1], puntos[i - 1][0] - puntos[i - 2][0])

    nextang2 = np.arctan2(pos[1] - puntos[i - 1][1], pos[0] - puntos[i - 1][0])

    dif2 = nextang2 - preang2
    print(f'{preang2=}')
    print(f'{nextang2=}')
    print(f'{dif2=}')
    if dif > np.pi:
        dif = dif - np.pi
    elif dif < -np.pi:
        dif = dif + np.pi
    if dif2 > np.pi:
        dif2 = dif2 - np.pi
    elif dif2 < -np.pi:
        dif2 = dif2 + np.pi

    if np.pi / 3 < abs(dif) < np.pi - np.pi / 3:
        aux = True
    else:
        aux = False
    if np.pi / 3 < abs(dif2) < np.pi - np.pi / 3:
        aux2 = True
    else:
        aux2 = False

    if aux == True or aux2 == True:
        return True
    else:
        return False


def goto5 (robot, base, puntos):
    time = 0
    K = 0.2
    Vdes = 0.3
    i = 0
    for punto in puntos:
        print(punto, puntos[i])
        if checkturn(i, puntos, base):
            Vcom = Vdes / 2
        else:
            Vcom = Vdes
        # Vcom=Vdes
        xdes = punto[0]
        ydes = punto[1]
        i += 1
        while (np.linalg.norm(punto - base.get_transform().pos())) > 0.7:
            print('error', (np.linalg.norm(punto - base.get_transform().pos())))

            T_base = base.get_transform()
            om = T_base.euler()[0].abg[2]
            pos = T_base.pos()

            if i == len(puntos) - 1: i = 0
            if (np.linalg.norm(puntos[i + 1] - pos)) < (np.linalg.norm(punto - pos)):
                print('---------------------------------------------------------------------------')
                break
            robot.x.append(pos[0])
            robot.y.append(pos[1])
            vel, ang = base.getVelocity()
            vel = np.linalg.norm(vel)
            ang = ang[2]

            omdes = np.arctan2(ydes - pos[1], xdes - pos[0])

            V = Vcom
            eom = omdes - om
            if abs(eom) > np.pi / 2 and np.linalg.norm(punto - base.get_transform().pos()) < 2:
                break
            print(f'{omdes=},{om=},{eom=}')
            w = K * eom
            print(f'{w=}')
            V, w = robot.checkmax(V, w)
            robot.V.append(vel)
            robot.time.append(time)
            time += 0.05
            robot.W.append(ang)
            # print(f'{vel=}, {ang=}')
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
    plt.xlabel('time(s)')
    long = len(robot.V)
    plt.plot(robot.time, robot.V)

    print(long)
    # plt.xticks(robot.V,[f'{x*0.05}s' for x in range(0,long,600)])setxticks
    plt.figure(3)
    plt.title('Velocidad angular', fontsize=30)
    plt.ylabel('w(rad/s)')
    plt.xlabel('time(s)')
    plt.plot(robot.time, robot.W)


def gotoDef (robot, base, puntos, velocity, CSR, smoothing):
    time = 0
    K = 2
    lv = []
    lw = []
    Vdes = velocity
    i = 0
    for punto in puntos:
        print(punto, puntos[i])
        if CSR:
            if checkturn(i, puntos, base):
                Vcom = Vdes / 2
            else:
                Vcom = Vdes
        else:
            Vcom = Vdes
        xdes = punto[0]
        ydes = punto[1]
        i += 1
        while (np.linalg.norm(punto - base.get_transform().pos())) > 0.7:
            print('error', (np.linalg.norm(punto - base.get_transform().pos())))

            T_base = base.get_transform()
            om = T_base.euler()[0].abg[2]
            pos = T_base.pos()

            robot.x.append(pos[0])
            robot.y.append(pos[1])
            vel, ang = base.getVelocity()
            vel = np.linalg.norm(vel)
            ang = ang[2]

            omdes = np.arctan2(ydes - pos[1], xdes - pos[0])

            V = Vcom
            eom = omdes - om
            if smoothing:
                if i == len(puntos) - 1: i = 0
                if (np.linalg.norm(puntos[i + 1] - pos)) < (np.linalg.norm(punto - pos)):
                    print('---------------------------------------------------------------------------')
                    break
                if abs(eom) > np.pi / 2 and np.linalg.norm(punto - base.get_transform().pos()) < 2:
                    break
            print(f'{omdes=},{om=},{eom=}')
            w = K * eom
            print(f'{w=}')
            V, w = robot.checkmax(V, w)
            lv.append(vel)
            robot.time.append(time)
            time += 0.05
            lw.append(ang)
            # print(f'{vel=}, {ang=}')
            robot.move(V, w)
            robot.wait()
    plt.figure(1)
    plt.title('Posición Husky', fontsize=20)
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    plt.plot(robot.x, robot.y, label='Trayectoria del Husky')
    plt.figure(2)
    plt.title('Velocidad lineal', fontsize=20)
    plt.ylabel('V(m/s)')
    plt.xlabel('time(s)')
    plt.plot(robot.time, lv)
    plt.figure(3)
    plt.title('Velocidad angular', fontsize=30)
    plt.ylabel('w(rad/s)')
    plt.xlabel('time(s)')
    plt.plot(robot.time, lw)


def gotoDef2 (robot, base, puntos, velocity, CSR, smoothing):

    Kv = 2
    Kw = 0.7 #0.4
    Vdes = velocity
    i = 0
    for punto in puntos:
        print(punto, puntos[i])
        xdes = punto[0]
        ydes = punto[1]
        i += 1
        while (np.linalg.norm(punto - base.get_transform().pos())) > 0.5:
            print('error', (np.linalg.norm(punto - base.get_transform().pos())))

            T_base = base.get_transform()
            om = T_base.euler()[0].abg[2]
            pos = T_base.pos()

            robot.x.append(pos[0])
            robot.y.append(pos[1])
            vel, ang = base.getVelocity()
            vel = np.linalg.norm(vel)
            ang = ang[2]

            omdes = np.arctan2(ydes - pos[1], xdes - pos[0])

            eom = omdes - om
            if smoothing:
                if i == len(puntos) - 1: i = 0
                if (np.linalg.norm(puntos[i + 1] - pos)) < (np.linalg.norm(punto - pos)):
                    print('---------------------------------------------------------------------------')
                    break
                if abs(eom) > np.pi / 2.5 and np.linalg.norm(punto - base.get_transform().pos()) < 2:
                    break
            print(f'{omdes=},{om=},{eom=}')
            w = Kv * eom
            print(f'{w=}')
            w = np.clip(w, -robot.Wmax, robot.Wmax)
            wcont = abs(w / robot.Wmax)
            V = Vdes * (1 - Kw * wcont)
            V, w = robot.checkmax(V, w)
            robot.lv.append(vel)
            robot.time.append(robot.seconds)
            robot.seconds += 0.05
            robot.lw.append(ang)
            # print(f'{vel=}, {ang=}')
            robot.move(V, w)
            robot.wait()
    plt.figure(1)
    plt.title('Posición Husky', fontsize=20)
    plt.xlabel('x(m)')
    plt.ylabel('y(m)')
    plt.plot(robot.x, robot.y, label='Trayectoria del Husky')
    plt.figure(2)
    plt.title('Velocidad lineal', fontsize=20)
    plt.ylabel('V(m/s)')
    plt.xlabel('time(s)')
    plt.plot(robot.time, robot.lv)
    plt.figure(3)
    plt.title('Velocidad angular', fontsize=20)
    plt.ylabel('w(rad/s)')
    plt.xlabel('time(s)')
    plt.plot(robot.time, robot.lw)
