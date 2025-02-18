#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

def callback(data):
    coords = data.data
    x_coords = coords[:len(coords)//2]
    y_coords = coords[len(coords)//2:]
    marker_array = MarkerArray()

    for i in range(len(x_coords)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "grid_node"
        marker.id = i
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = x_coords[i]
        marker.pose.position.y = y_coords[i]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Diameter of 50 cm
        marker.scale.y = 0.2
        marker.scale.z = 0.01  # Very thin cylinder to represent a circle
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = rospy.Duration(0.5)

        marker_array.markers.append(marker)

    marker_pub.publish(marker_array)

def listener():
    rospy.init_node('grid_node', anonymous=True)
    rospy.Subscriber("group_positions", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
    listener()
