#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Indica que este script debe ser ejecutado con el intérprete de Python del entorno ROS

import rospy
# Importa rospy, la biblioteca de ROS para Python
import numpy as np

from std_msgs.msg import Float32MultiArray
# Importa el tipo de mensaje Float32MultiArray desde std_msgs

def publish_positions():
    pub = rospy.Publisher('group_positions', Float32MultiArray, queue_size=10)
    # Crea un publicador en el tópico 'group_positions' que enviará mensajes de tipo Float32MultiArray
    rospy.init_node('position_group_publisher', anonymous=True)
    # Inicializa el nodo de ROS llamado 'position_group_publisher'
    rate = rospy.Rate(1)  # 1Hz, ajusta la frecuencia según sea necesario
    # Establece la frecuencia de publicación a 1 Hz 
    X = np.array([1, -1, -1, 1, 0.4, 2], dtype=float)
    Y = np.array([1, 1, 2, -1, 2, 1.4], dtype=float)
    # Define las coordenadas X e Y

    while not rospy.is_shutdown():
        # Bucle que continuará ejecutándose mientras ROS no haya sido cerrado
        if len(X) != len(Y):
            rospy.logerr("X and Y vectors must have the same number of elements.")
            break
        positions = Float32MultiArray()
        
        positions.data.extend(X)
        positions.data.extend(Y)
        # Crea un nuevo mensaje Float32MultiArray y asigna las coordenadas x, y al mensaje

        rospy.loginfo("Publishing group positions: {}".format(positions.data))
        # Publica en el registro de ROS el mensaje que será enviado usando el método format
        pub.publish(positions)
        # Publica el mensaje en el tópico 'group_positions'
        rate.sleep()
        # Duerme durante el tiempo necesario para mantener la frecuencia de 1 Hz

if __name__ == '__main__':
    try:
        publish_positions()
        # Llama a la función publish_positions cuando el script es ejecutado
    except rospy.ROSInterruptException:
        pass
        # Captura y maneja cualquier excepción de ROS que pueda ocurrir durante la ejecución
