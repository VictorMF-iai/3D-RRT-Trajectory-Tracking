import numpy as np
import utm
import scipy.interpolate as spi
from artelib.homogeneousmatrix import HomogeneousMatrix


def convertCoordinates (coord0, coord2):
    """

    :param coord0: Lat/long coordinates
    :param coord2: Lat/long coordinates
    :return: cartesian coordinates
    """
    base = utm.from_latlon(coord0[0], coord0[1])
    dest = utm.from_latlon(coord2[1], coord2[0])
    x = dest[0] - base[0]
    y = dest[1] - base[1]
    print(x, '   ', y)
    return [x, y]


def interpTraj (list, sep):
    """

    :param list:List of path nodes
    :param sep: Distance desired between points
    :return: Interpolated path
    """
    zh = 0.243  # Centro husky
    newlist = []
    for i in range(len(list) - 1):
        dist = np.sqrt((list[i][0] - list[i + 1][0]) ** 2 + (list[i][1] - list[i + 1][1]) ** 2)
        if dist < 2 * sep:
            newlist.append(list[i])
        else:
            step = round(dist / sep) + 1
            x_p = [list[i][0], list[i + 1][0]]
            y_p = [list[i][1], list[i + 1][1]]
            x_interp = np.linspace(x_p[0], x_p[1], step)
            y_linear = spi.interp1d(x_p, y_p, kind='linear')
            y_interp = y_linear(x_interp)
            for j in range(len(x_interp)):
                x = x_interp[j]
                y = y_interp[j]
                z = zh
                newlist.append([x, y, z])
            del newlist[-1]
    newlist.append(list[len(list) - 1])
    return newlist


def interpSpline (list):
    zh = 0.243  # Centro husky
    num = 5  # Number of points in the interpolation
    newlist = []
    xlist = []
    ylist = []
    for i in range(len(list)):
        xlist.append(list[i][0])
        ylist.append(list[i][1])

    nose, *rest = spi.splprep([xlist, ylist], k=2)

    x_interp = np.linspace(0, 1, num)
    salida = spi.splev(x_interp, nose)

    for j in range(len(x_interp)):
        x = salida[0][j]
        y = salida[1][j]
        z = zh
        newlist.append([x, y, z])
    return newlist


def SplinTraj (lista, nodos):
    puntos = 1
    for nodo in nodos[1:-1]:
        index = lista.index(nodo)
        auxlist = []
        for i in range(-puntos, puntos + 1, 1):
            auxlist.append([lista[index + i][0], lista[index + i][1], lista[index + i][2]])
        newlist = interpSpline(auxlist)
        del lista[index - puntos:index + puntos + 1]
        for j in range(-puntos, len(newlist) - puntos, 1):
            lista.insert(index + j, [newlist[puntos + j][0], newlist[puntos + j][1], newlist[puntos + j][2]])
    del lista[0]
    return lista


def CoordsToLocal (base, goal):
    """
    Transform from Global coordinates to Local coordinates
    """
    T_base = base.get_transform()
    T1 = HomogeneousMatrix(goal, T_base.euler()[0])
    T2 = HomogeneousMatrix(T_base.pos(), T_base.euler()[0])
    tx = T1 * T2.inv()
    posicion = tx.pos()
    return posicion


def CoordsToGlobal (base, goal):
    """
    Transform from Local coordinates to Global coordinates
    """
    T_base = base.get_transform()
    T1 = HomogeneousMatrix(goal, T_base.euler()[0])
    T2 = HomogeneousMatrix(T_base.pos(), [0, 0, 0])
    tx = T2 * T1
    posicion = tx.pos()
    return posicion
