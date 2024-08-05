#!/usr/bin/env python
# -*- coding: utf-8 -*-n
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy as np

def person_position_callback(data):
    global person_position
    person_position = data.data

def create_restricted_zone():
    rospy.init_node('restricted_zone_node', anonymous=True)
    rospy.Subscriber('person_position', Float32MultiArray, person_position_callback)
    pub = rospy.Publisher('/restricted_map', PointCloud, queue_size=10)
    rate = rospy.Rate(5) # 1hz

    while not rospy.is_shutdown():
        if person_position is not None:
            restricted_zone = PointCloud()
            restricted_zone.header.stamp = rospy.Time.now()
            restricted_zone.header.frame_id = "map"

            # Create a circular restricted zone
            cx, cy = person_position
            radius = 0.2
            resolution = 0.1  # Points every 10 cm

            for angle in np.arange(0, 2 * np.pi, resolution):
                x = cx + radius * np.cos(angle)
                y = cy + radius * np.sin(angle)
                restricted_zone.points.append(Point32(x, y, 0))

            pub.publish(restricted_zone)
        rate.sleep()

if __name__ == '__main__':
    person_position = None

    try:
        create_restricted_zone()
    except rospy.ROSInterruptException:
        pass