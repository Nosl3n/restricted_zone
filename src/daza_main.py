#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np
from daza_functions import *

def group_positions_callback(data):
    global groups_positions
    groups_positions = data.data

def create_restricted_zone():
    rospy.init_node('restricted_zone_node', anonymous=True)
    rospy.Subscriber('group_positions', Float32MultiArray, group_positions_callback)
    pub = rospy.Publisher('/restricted_map', PointCloud, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        if groups_positions is not None and len(groups_positions) > 0:
            n = len(groups_positions)
            if n % 2 != 0:
                rospy.logwarn("The length of groups_positions is odd, cannot divide equally.")
                continue

            mid = n // 2
            x = np.array(groups_positions[:mid])
            y = np.array(groups_positions[mid:])
            restricted_zone = PointCloud()
            restricted_zone.header.stamp = rospy.Time.now()
            restricted_zone.header.frame_id = "map"

            xcm = np.divide((np.max(x) + np.min(x)), 2)
            ycm = np.divide((np.max(y) + np.min(y)), 2)
            x_ord, y_ord = ordenar_puntos(xcm, ycm, x, y)
            dis, ang = dis_ang(x_ord, y_ord, xcm, ycm)
            md = np.max(dis)
            ang_vec = orientacion_vec(x_ord, y_ord, xcm, ycm)
            rotation = np.deg2rad(ang_vec)

            # Parámetros de la función gaussiana
            variance_front = md + 1 * md
            variance_right = md + 0.5 * md
            variance_left = md + 0.5 * md
            variance_rear = md + 0.1 * md

            # Llamar a la función
            x, y, f = gaussian2_a2_focussed(xcm, ycm, rotation, variance_front, variance_right, variance_left, variance_rear)

            f_threshold = 0.6
            clearing_zone = PointCloud()
            clearing_zone.header.stamp = rospy.Time.now()
            clearing_zone.header.frame_id = "map"

            for i in range(x.shape[0]):
                for j in range(x.shape[1]):
                    if f[i, j] > f_threshold:
                        restricted_zone.points.append(Point32(x[i, j], y[i, j], 0))
                        # También agregamos el mismo punto en la nube de puntos para limpiez
                        clearing_zone.points.append(Point32(x[i, j], y[i, j], 0))

            rospy.loginfo("Publishing restricted zone with {} points.".format(len(restricted_zone.points)))
            pub.publish(restricted_zone)  # Publicar la nube de puntos generada

            rospy.sleep(0.1)  # Esperar un breve momento para asegurar que la publicación se procese

            # Publicar la nube de puntos de limpieza (con costo 0)
            pub.publish(clearing_zone)  # Publica la nube para limpiar los puntos anteriores
        else:
            # Si no hay datos disponibles, simplemente publica una nube vacía
            empty_cloud = PointCloud()
            empty_cloud.header.frame_id = "map"
            pub.publish(empty_cloud)
            rospy.loginfo("No group positions available, clearing restricted zone.")

        rate.sleep()  # Esperar el tiempo restante para mantener la frecuencia de 10Hz

if __name__ == '__main__':
    groups_positions = None
    
    try:       
        create_restricted_zone()
    except rospy.ROSInterruptException:    
        pass
