#!/usr/bin/env python
# -*- coding: utf-8 -*-n
# Indica que este script debe ser ejecutado con el intérprete de Python del entorno ROS

import rospy
# Importa rospy, la biblioteca de ROS para Python

from std_msgs.msg import Float32MultiArray
# Importa el tipo de mensaje Float32MultiArray desde std_msgs

def publish_position():
    pub = rospy.Publisher('person_position', Float32MultiArray, queue_size=10)
    # Crea un publicador en el tópico 'person_position' que enviará mensajes de tipo Float32MultiArray
    rospy.init_node('position_publisher', anonymous=True)
    # Inicializa el nodo de ROS llamado 'position_publisher'
    rate = rospy.Rate(5) # 10hz
    # Establece la frecuencia de publicación a 10 Hz
    y=0
    while not rospy.is_shutdown():
        # Bucle que continuará ejecutándose mientras ROS no haya sido cerrado
        position = Float32MultiArray()
        if y < -5:
            y=0
        # Crea un nuevo mensaje Float32MultiArray
        position.data = [-1.5, y]
        y=y-0.1
        # Asigna las coordenadas x, y al mensaje
        rospy.loginfo("Publishing position: {}".format(position.data))
        # Publica en el registro de ROS el mensaje que será enviado usando el método format
        pub.publish(position)
        # Publica el mensaje en el tópico 'person_position'
        rate.sleep()
        # Duerme durante el tiempo necesario para mantener la frecuencia de 10 Hz

if __name__ == '__main__':
    try:
        publish_position()
        # Llama a la función publish_position cuando el script es ejecutado
    except rospy.ROSInterruptException:
        pass
        # Captura y maneja cualquier excepción de ROS que pueda ocurrir durante la ejecución

