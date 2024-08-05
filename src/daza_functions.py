# -*- coding: utf-8 -*-
import numpy as np

def gaussian2_a2_focussed(mean_x, mean_y, rotation, variance_front, variance_right, variance_left, variance_rear):
    limit = 2
    x, y = np.meshgrid(np.arange(mean_x - limit, mean_x + limit, 0.04), np.arange(mean_y - limit, mean_y + limit, 0.04))
    alpha = np.arctan2(y - mean_y, x - mean_x) - rotation + np.pi / 2
    alpha = np.where(alpha > np.pi, alpha - 2 * np.pi, alpha)
    alpha = np.where(alpha < -np.pi, alpha + 2 * np.pi, alpha)
    variance = np.where(alpha <= 0, variance_rear, variance_front)
    variance_sides = np.where((alpha >= np.pi / 2) | (alpha <= -np.pi / 2), variance_left, variance_right)
    a = (np.cos(rotation) ** 2) / (2 * variance) + (np.sin(rotation) ** 2) / (2 * variance_sides)
    b = np.sin(2 * rotation) / (4 * variance) - np.sin(2 * rotation) / (4 * variance_sides)
    c = (np.sin(rotation) ** 2) / (2 * variance) + (np.cos(rotation) ** 2) / (2 * variance_sides)
    f = np.exp(-(a * (x - mean_x) ** 2 + 2 * b * (x - mean_x) * (y - mean_y) + c * (y - mean_y) ** 2))
    return x, y, f

def ordenar_puntos(xcm, ycm, x, y):
    """
    Ordena los puntos (x, y) en función de sus ángulos respecto a un punto central (xcm, ycm).
    Args:
    xcm (float): Coordenada x del punto central.
    ycm (float): Coordenada y del punto central.
    x (array_like): Vector de coordenadas x de los puntos.
    y (array_like): Vector de coordenadas y de los puntos.
    Returns:
    tuple: Dos arrays con las coordenadas x e y ordenadas.
    """
    # Verificar que xcm e ycm son escalares
    if not np.isscalar(xcm) or not np.isscalar(ycm):
        raise ValueError("xcm e ycm deben ser valores escalares.")
    # Comprobar que los vectores x e y tienen el mismo tamaño
    if len(x) != len(y):
        raise ValueError("Los vectores x e y deben tener el mismo tamaño.")
    # Calcular los ángulos en radianes entre cada punto (x, y) y el punto central (xcm, ycm)
    ang_ordenar = np.rad2deg(np.arctan2(y - ycm, x - xcm))
    # Ajustar los ángulos para que estén en el rango [0, 360)
    angulos_ajustados = np.mod(ang_ordenar, 360)
    # Ordenar los ángulos ajustados y obtener los índices de ordenamiento
    indice_orden = np.argsort(angulos_ajustados)
    # Utilizar los índices de ordenamiento para reordenar los vectores x e y
    x_ord = x[indice_orden]
    y_ord = y[indice_orden]
    return x_ord, y_ord

def dis_ang(x, y, xc, yc):
    """
    Calcula las distancias euclidianas y los ángulos en grados de cada punto (x, y) respecto a un punto central (xc, yc)
    Args:
    x (array_like): Vector de coordenadas x de los puntos.
    y (array_like): Vector de coordenadas y de los puntos.
    xc (float): Coordenada x del punto central.
    yc (float): Coordenada y del punto central.
    Returns:
    tuple: Dos arrays con las distancias y los ángulos en grados.
    """
    # Verificar que xc e yc son escalares
    if not np.isscalar(xc) or not np.isscalar(yc):
        raise ValueError('xc e yc deben ser valores escalares.')
    # Verificar que los vectores x e y tienen el mismo tamaño
    if len(x) != len(y):
        raise ValueError('Los vectores x e y deben tener el mismo tamaño.')
    # Calcular las distancias euclidianas entre cada punto (x, y) y el punto central (xc, yc)
    distancias = np.sqrt((x - xc)**2 + (y - yc)**2)
    # Calcular los ángulos en radianes entre cada punto (x, y) y el punto central (xc, yc)
    angulos = np.arctan2(y - yc, x - xc)
    # Convertir los ángulos de radianes a grados y ajustarlos para que estén en el rango [0, 360)
    angulos_grados = np.mod(np.degrees(angulos), 360)
    return distancias, angulos_grados

def result(x, y):
    # Verificar que x e y tienen dos elementos
    if len(x) != 2 or len(y) != 2:
        raise ValueError('Los vectores x e y deben tener dos elementos.')
    # Crear los vectores
    v1 = np.array([x[0], y[0]])
    v2 = np.array([x[1], y[1]])
    # Calcular la resultante sumando los vectores
    resultante = v1 + v2
    return resultante

def orientacion_vec(x, y, cmx, cmy):
    # Se determina el desplazamiento necesario para mover el centro de masa al origen
    xmove = -cmx
    ymove = -cmy
    # Se mueve todo al origen
    x = x + xmove
    y = y + ymove
    if len(x) == 2:
        # Cuando hay dos individuos, la dirección es el primer individuo
        resultan = np.array([x[0], y[0]])
    else:
        # Ordenar los puntos alrededor del origen
        x_ord, y_ord = ordenar_puntos(0, 0, x, y)
        # Se halla la dirección resultante sumando los vectores
        resultan = np.array([x_ord[0], y_ord[0]])
        for i in range(1, len(x)):
            resultan = result([resultan[0], x_ord[i]], [resultan[1], y_ord[i]])
    # Caso en el que la resultante sea cero
    if resultan[0] == 0 and resultan[1] == 0:
        resultan = np.array([x_ord[0], y_ord[0]])
    # Determinar el ángulo
    an = np.arctan2(resultan[1], resultan[0])
    ang = np.mod(np.rad2deg(an), 360)
    return ang