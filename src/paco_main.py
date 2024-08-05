#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
from paco_functions import *

def group_positions_callback(data):
    global groups_positions
    #rospy.loginfo("Groups positions callback triggered.")
    groups_positions = data.data

def create_restricted_zone():
    rospy.init_node('restricted_zone_node', anonymous=True)
    rospy.Subscriber('group_positions', Float32MultiArray, group_positions_callback)
    pub = rospy.Publisher('/restricted_map', PointCloud, queue_size=10)
    rate = rospy.Rate(12)  # 5hz
    f_threshold = 0.8

    while not rospy.is_shutdown():
        if groups_positions is not None:
            #rospy.loginfo("Groups positions received: {}".format(groups_positions))
            n = len(groups_positions)
            if n % 2 != 0:
                rospy.logwarn("The length of groups_positions is odd, cannot divide equally.")
                continue

            # Dividir la data en x e y
            mid = n // 2
            x = np.array(groups_positions[:mid])
            y = np.array(groups_positions[mid:])
            #print(x)
            #print(y)
            restricted_zone = PointCloud()
            restricted_zone.header.stamp = rospy.Time.now()
            restricted_zone.header.frame_id = "map"
            # Datos de ejemplo
            #x = np.array([1, -1, -1, 1, 0.4, 2], dtype=float)
            #y = np.array([1, 1, 2, -1, 2, 1.4], dtype=float)
            # determinar el centro del grupo
            xcm = np.divide((np.max(x) + np.min(x)), 2)
            ycm = np.divide((np.max(y) + np.min(y)), 2)
            # Ordenar puntos
            x_ord, y_ord = ordenar_puntos(xcm, ycm, x, y)
            # determinar las distancias y sus angulos cn respecto al eje x.
            dis, ang = dis_ang(x_ord, y_ord, xcm, ycm)
            # determinar la orientacion del grupo
            ang_vec = orientacion_vec(x_ord, y_ord, xcm, ycm)
            # SE APLICA EL PRIMER FILTRO DE ELIMINAR PERSONAS
            for _ in range(len(x)):
                dis, ang = dis_ang(x_ord, y_ord, xcm, ycm)
                x_mod, y_mod = entre_personas(15, ang, dis, x_ord, y_ord)
                if len(x_mod) == len(x_ord):
                    break
                else:
                    x_ord = x_mod
                    y_ord = y_mod
            # Se aplica el filtro de aumentar personas xaum y yaum
            x_aum, y_aum = aumentar_02 (70,x_mod,y_mod,xcm,ycm)
            # Se halla la priemra persona cercana a la orientacion
            orientacion = 1000
            for i in range(len(ang)):
                if ang_vec < ang[i]:
                    orientacion = ang[i]  # Primer ángulo, para rotar todo
                    break
                # En el caso de que no hay ángulos mayores a la dirección vectorial, escogerá el primer ángulo desde 0°
                if orientacion == 1000:
                    orientacion = ang[0]
            # Se determina el angulo de rotacion
            rotacion = -orientacion
            dis, ang = dis_ang (x_aum,y_aum,xcm,ycm)
            # determinacion de la nueva distribucion de vectores
            ang_new, ang, x_new, y_new = ordenamiento(ang, orientacion, x_aum, y_aum)
            # Determinacion de las distancias y sus angulos con respecto al eje x, de cada punto al centro del grupo
            dis, ang_sum = dis_ang (x_new,y_new,xcm,ycm)
            # DETERMINACION DE LAS VARIANZAS MADRE
            min_sig = 0.5  # Valor mínimo de las varianzas
            # Calcular sigma_x y sigma_y
            sigma_x = np.abs(dis) + min_sig
            sigma_y = np.abs(dis) + min_sig
            # Añadir el sigma inicial al final de todos los sigmas
            sigma_x = np.append(sigma_x, sigma_x[0])
            sigma_y = np.append(sigma_y, sigma_y[0])
            # Determinar las distancias para hallar los sigmas intermedios
            distan = np.zeros(len(ang))
            for i in range(len(ang)):
                if i == len(ang) - 1:
                    distan[i] = 360 - ang[i] + ang[0]
                else:
                    distan[i] = ang[i + 1] - ang[i]
            # Generar un valor de cero al comienzo del arreglo para manejar mejor los índices
            distancias = np.insert(distan, 0, 0)
            #DETERMINAR LOS SIGMA HIJOS, CON UN RECORRIDO DE 0 A 360°
            j = 1  # índice ajustado para Python (0-based index)
            k = 0  # índice ajustado para Python (0-based index)
            cont = 0
            angulo = 0
            delta_ang = 1
            # Inicializar arrays para los sigma
            sigma_xx = np.zeros(360)
            sigma_yy = np.zeros(360)
            
            for i in range(360):
                if i > distancias[j] + angulo:
                    angulo = distancias[j] + angulo
                    j += 1
                    k += 1
                    cont = 0
                t1 = distancias[j] - cont
                t2 = cont
                cont += delta_ang
                sigma_xx[i] = ((t1 / distancias[j]) * sigma_x[k]) + ((t2 / distancias[j]) * sigma_x[k + 1])
                sigma_yy[i] = ((t1 / distancias[j]) * sigma_y[k]) + ((t2 / distancias[j]) * sigma_y[k + 1])
            # GENERACION DE LA MALLA PARA LA GAUSSIANA
            lado = 2.5; #maximo valor de cada lado de la grafica
            paso = 0.04; #Paso de la malla, entre punto a punto
            xpos = abs(max(x))+lado
            xneg = abs(min(x))-lado
            ypos = abs(max(y))+lado
            yneg = abs(min(y))-lado
            # gaussiana
            # Se genera la malla en la que se determinará cada punto de la gaussiana
            xx, yy = np.meshgrid(np.arange(xneg, xpos + paso, paso), np.arange(yneg, ypos + paso, paso))

            # Creación de las matrices de varianzas
            tam = yy.shape
            varianzax = np.zeros(tam)
            varianzay = np.zeros(tam)
            
            for i in range(tam[0]):
                for j in range(tam[1]):
                    theta = np.arctan2(yy[i, j] - ycm, xx[i, j] - xcm)
                    theta = np.rad2deg(theta) % 360
                    alpha = int(round(theta))  # redondeando se asigna un valor al ángulo

                    if alpha >= 360:
                        alpha = 360
                    elif alpha <= 1:
                        alpha = 1

                    varianzax[i, j] = sigma_xx[alpha - 1]
                    varianzay[i, j] = sigma_yy[alpha - 1]

            # Se crea cada punto zz de la función gaussiana
            zz = np.exp(-(xx - xcm)**2 / (2 * varianzax**2) - (yy - ycm)**2 / (2 * varianzay**2))

            # Se rota el desfase de la gaussiana
            xrot, yrot, zrot = rotar_gaussiana(xx, yy, zz, rotacion, xcm, ycm)
            print("distancias:", distancias)
            # Resultado final
           

            # Llamar a la función
            #x, y, f = gaussian2_a2_focussed(xcm, ycm, rotation, variance_front, variance_right, variance_left, variance_rear)

            #rospy.loginfo("Gaussian function output: x={}, y={}, f={}".format(x, y, f))

            # Crear los puntos del PointCloud para los valores de f > f_threshold
            for i in range(xrot.shape[0]):
                for j in range(xrot.shape[1]):
                    if zrot[i, j] > f_threshold:
                        restricted_zone.points.append(Point32(xrot[i, j], yrot[i, j], 0))

            rospy.loginfo("Publishing restricted zone with {} points.".format(len(restricted_zone.points)))
            pub.publish(restricted_zone)
        rate.sleep()

if __name__ == '__main__':
    groups_positions = None
    
    try:       
        create_restricted_zone()
    except rospy.ROSInterruptException:    
        pass


# -2.660 0.332
    #-1.88 1.516