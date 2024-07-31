#!/usr/bin/env python
# -*- coding: utf-8 -*-n
# Indica que este script debe ser ejecutado con el intérprete de Python del entorno ROS

import rospy
# Importa rospy, la biblioteca de ROS para Python

from std_msgs.msg import Float32MultiArray
# Importa el tipo de mensaje Float32MultiArray desde std_msgs

def callback(data):
    rospy.loginfo("Received position: {}".format(data.data))
    # Función de callback que se llama cada vez que se recibe un mensaje en el tópico 'person_position'
    # Publica en el registro de ROS los datos del mensaje recibido usando el método format

def listen_position():
    rospy.init_node('position_listener', anonymous=True)
    # Inicializa el nodo de ROS llamado 'position_listener'
    rospy.Subscriber('person_position', Float32MultiArray, callback)
    # Crea un suscriptor al tópico 'person_position' que ejecutará la función callback cuando reciba un mensaje
    rospy.spin()
    # Mantiene el nodo en ejecución hasta que ROS sea cerrado

if __name__ == '__main__':
    try:
        listen_position()
        # Llama a la función listen_position cuando el script es ejecutado
    except rospy.ROSInterruptException:
        pass
        # Captura y maneja cualquier excepción de ROS que pueda ocurrir durante la ejecución
