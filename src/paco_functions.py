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

def entre_personas(minima, ang, dis, x_ord, y_ord):
    # Verificar que los vectores de entrada tienen el mismo tamaño
    if not (len(ang) == len(dis) == len(x_ord) == len(y_ord)):
        raise ValueError('Todos los vectores de entrada deben tener el mismo tamaño.')
    
    # Determinar las distancias angulares entre personas
    distancias = separacion(ang)
    eliminar = -1  # Indicador de persona a eliminar

    # Encontrar la persona a eliminar basada en la distancia mínima
    for i in range(len(distancias)):
        if distancias[i] <= minima:  # Filtro de distancia
            if i == len(x_ord) - 1:  # Caso en el que se trabaja con la última persona
                if dis[i] < dis[0]:
                    eliminar = i
                else:
                    eliminar = 0
            else:  # Los demás casos
                if dis[i] < dis[i + 1]:
                    eliminar = i
                else:
                    eliminar = i + 1
    
    # Modificar los vectores x e y, eliminando la persona seleccionada
    if eliminar == -1:
        # Si no se elimina ninguna persona, devolver los vectores originales
        xmod = x_ord
        ymod = y_ord
    else:
        # Inicializar vectores modificados
        xmod = np.delete(x_ord, eliminar)
        ymod = np.delete(y_ord, eliminar)
    
    return xmod, ymod

def separacion(ang):
    # Verificar que el vector ang no esté vacío
    if len(ang) == 0:
        raise ValueError('El vector de ángulos no puede estar vacío.')
    
    # Verificar que el vector ang tenga al menos dos elementos
    if len(ang) < 2:
        raise ValueError('El vector de ángulos debe contener al menos dos elementos.')
    
    # Agregar el primer ángulo más 360 al final del vector
    ang = np.append(ang, 360 + ang[0])
    
    # Calcular las distancias angulares entre personas
    distancias = np.diff(ang)
    
    return distancias

def aumentar_02(maximo, x_ord, y_ord, xcm, ycm):
    if len(x_ord) != len(y_ord):
        raise ValueError('Los vectores x_ord e y_ord deben tener el mismo tamaño.')

    dis1, ang1 = dis_ang(x_ord, y_ord, xcm, ycm)
    nuevas_distancias = separacion(ang1)

    cambios = []
    for i in range(len(nuevas_distancias)):
        if nuevas_distancias[i] > maximo:
            cambios.append(i)

    x_mas = []
    y_mas = []
    for i in cambios:
        if i == len(nuevas_distancias) - 1:
            angulo = ang1[i] + (nuevas_distancias[i] / 2)
            Lnew = dis1[i] / 4 + dis1[0] / 4
        else:
            angulo = ang1[i] + (nuevas_distancias[i] / 2)
            Lnew = dis1[i] / 4 + dis1[i + 1] / 4

        xnew = np.cos(np.deg2rad(angulo)) * Lnew + xcm
        ynew = np.sin(np.deg2rad(angulo)) * Lnew + ycm
        x_mas.append(xnew)
        y_mas.append(ynew)

    if len(cambios) != 0:
        x_aum = np.concatenate((x_ord, x_mas))
        y_aum = np.concatenate((y_ord, y_mas))
        x_aaum, y_aaum = ordenar_puntos(xcm, ycm, x_aum, y_aum)
    else:
        x_aaum = x_ord
        y_aaum = y_ord

    return x_aaum, y_aaum

def ordenamiento(ang, referencia, x, y):
    # Verificar que los vectores de entrada tienen el mismo tamaño
    if len(ang) != len(x) or len(x) != len(y):
        raise ValueError('Los vectores ang, x, y deben tener el mismo tamaño.')
    
    # Encontrar el índice de la referencia
    if referencia not in ang:
        raise ValueError('La referencia no se encuentra en el vector de ángulos.')
    
    indice = np.where(ang == referencia)[0][0]
    
    # Reordenar los ángulos y las coordenadas según el índice de referencia
    nuevo_ang = np.concatenate((ang[indice:], ang[:indice]))
    x_nuv = np.concatenate((x[indice:], x[:indice]))
    y_nuv = np.concatenate((y[indice:], y[:indice]))
    
    # Inicializar el vector de ángulos ajustados
    nuevo = np.zeros(len(nuevo_ang))
    
    # Ajustar los ángulos considerando la referencia como cero grados
    nuevo = (nuevo_ang - nuevo_ang[0]) % 360
    
    return nuevo_ang, nuevo, x_nuv, y_nuv
import numpy as np

def rotar_gaussiana(x, y, z, angulo, cmx, cmy):
    # Verificar que las matrices de entrada tienen el mismo tamaño
    if not (x.shape == y.shape == z.shape):
        raise ValueError('Las matrices x, y, z deben tener el mismo tamaño.')

    # Verificar que el ángulo es un valor numérico
    if not isinstance(angulo, (int, float)):
        raise ValueError('El ángulo debe ser un valor numérico.')

    # Verificar que cmx y cmy son valores numéricos
    if not (isinstance(cmx, (int, float)) and isinstance(cmy, (int, float))):
        raise ValueError('Los valores de cmx y cmy deben ser numéricos.')

    # Determinamos la traslación en los ejes
    xmove = -cmx
    ymove = -cmy

    # Ángulo de rotación en radianes
    angulo = np.deg2rad(angulo)

    # Crear la matriz de transformación homogénea para la rotación alrededor del eje Z
    matriz_rotacion = np.array([
        [np.cos(angulo), -np.sin(angulo), 0],
        [np.sin(angulo), np.cos(angulo), 0],
        [0, 0, 1]
    ])

    # Convertir las matrices x, y, z en vectores
    xx = x.ravel()
    yy = y.ravel()
    zz = z.ravel()

    # Trasladar la gaussiana al origen de coordenadas
    xx += xmove
    yy += ymove

    # Combinar los vectores en una matriz de puntos
    puntos = np.vstack((xx, yy, zz)).T

    # Aplicar la matriz de transformación homogénea (Rotación en Z) a los puntos en 3D
    puntos_rot = np.dot(puntos, matriz_rotacion.T)

    # Separar los puntos rotados en vectores x, y, z
    Xrot = puntos_rot[:, 0]
    Yrot = puntos_rot[:, 1]
    Zrot = puntos_rot[:, 2]

    # Regresar la gaussiana a su posición original
    Xrot -= xmove
    Yrot -= ymove

    # Reshape para obtener matrices 2D
    xrot = Xrot.reshape(x.shape)
    yrot = Yrot.reshape(y.shape)
    zrot = Zrot.reshape(z.shape)

    return xrot, yrot, zrot
