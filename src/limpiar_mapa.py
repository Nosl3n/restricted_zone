#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point32
import numpy as np

# Variables globales para almacenar el mapa estático
static_map = None

def map_callback(map_msg):
    global static_map
    static_map = map_msg

def pointcloud_callback(pointcloud_msg):
    global static_map
    if static_map is None:
        rospy.logwarn("Static map not yet received.")
        return

    # Crear un nuevo PointCloud para publicar los puntos filtrados
    filtered_pointcloud = PointCloud()
    filtered_pointcloud.header = pointcloud_msg.header

    # Convertir el mapa estático a una matriz para facilitar la verificación
    map_width = static_map.info.width
    map_height = static_map.info.height
    map_data = np.array(static_map.data).reshape((map_height, map_width))
    map_resolution = static_map.info.resolution
    map_origin_x = static_map.info.origin.position.x
    map_origin_y = static_map.info.origin.position.y

    for point in pointcloud_msg.points:
        # Convertir las coordenadas del punto al sistema de coordenadas del mapa
        map_x = int((point.x - map_origin_x) / map_resolution)
        map_y = int((point.y - map_origin_y) / map_resolution)

        # Verificar si el punto está dentro de los límites del mapa
        if 0 <= map_x < map_width and 0 <= map_y < map_height:
            # Verificar si el punto está en una celda ocupada del mapa estático
            if map_data[map_y, map_x] == -1:  # Celda desconocida
                filtered_pointcloud.points.append(point)
            elif map_data[map_y, map_x] == 0:  # Celda libre
                filtered_pointcloud.points.append(point)

    pub.publish(filtered_pointcloud)
    rospy.loginfo("Published filtered PointCloud with {} points.".format(len(filtered_pointcloud.points)))

if __name__ == '__main__':
    rospy.init_node('clean_pointclouds_node', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.Subscriber('/restricted_map', PointCloud, pointcloud_callback)
    pub = rospy.Publisher('/filtered_restricted_map', PointCloud, queue_size=10)

    rospy.loginfo("Nodo limpiador de PointClouds iniciado.")
    rospy.spin()
