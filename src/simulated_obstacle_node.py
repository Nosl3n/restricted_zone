#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np

def gaussian2_a2_focussed(mean_x, mean_y, rotation, variance_front, variance_right, variance_left, variance_rear):
    limit = 2
    x, y = np.meshgrid(np.arange(mean_x - limit, mean_x + limit, 0.01), np.arange(mean_y - limit, mean_y + limit, 0.01))
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

def person_position_callback(data):
    global person_position
    person_position = data.data

def create_restricted_zone():
    rospy.init_node('restricted_zone_node', anonymous=True)
    rospy.Subscriber('person_position', Float32MultiArray, person_position_callback)
    pub = rospy.Publisher('/restricted_map', PointCloud, queue_size=10)
    rate = rospy.Rate(5)  # 5hz

    f_threshold = 0.8

    while not rospy.is_shutdown():
        if person_position is not None:
            restricted_zone = PointCloud()
            restricted_zone.header.stamp = rospy.Time.now()
            restricted_zone.header.frame_id = "map"

            # Obtener la posición y calcular la función gaussiana
            cx, cy = person_position
            rotation = 3.1415/2  # Define la rotación según sea necesario
            variance_front = 0.5
            variance_right = 0.4
            variance_left = 0.4
            variance_rear = 0.1

            x, y, f = gaussian2_a2_focussed(cx, cy, rotation, variance_front, variance_right, variance_left, variance_rear)

            # Crear los puntos del PointCloud para los valores de f > f_threshold
            for i in range(x.shape[0]):
                for j in range(x.shape[1]):
                    if f[i, j] > f_threshold:
                        restricted_zone.points.append(Point32(x[i, j], y[i, j], 0))

            pub.publish(restricted_zone)
        rate.sleep()

if __name__ == '__main__':
    person_position = None

    try:
        create_restricted_zone()
    except rospy.ROSInterruptException:
        pass
