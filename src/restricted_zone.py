ó
ªfc           @   s   d  d l  Z  d  d l m Z d  d l m Z d  d l m Z d  d l Z d   Z	 d   Z
 e d k r d a y e
   Wq e  j k
 r q Xn  d S(   iÿÿÿÿN(   t   Float32MultiArray(   t
   PointCloud(   t   Point32c         C   s   |  j  a d  S(   N(   t   datat   person_position(   R   (    (    sD   /home/shado/pepper_sim_ws/src/restricted_zone/src/restricted_zone.pyt   person_position_callback	   s    c    
      C   s=  t  j d d t t  j d t t  t  j d t d d }  t  j d  } xé t  j	   s8t
 d  k	 r+t   } t  j j   | j _ d | j _ t
 \ } } d	 } d
 } xn t j d d t j |  D]P } | | t j |  } | | t j |  }	 | j j t | |	 d   qÇ W|  j |  n  | j   qP Wd  S(   Nt   restricted_zone_nodet	   anonymousR   s   /restricted_mapt
   queue_sizei
   i   t   mapgÉ?g¹?i    i   (   t   rospyt	   init_nodet   Truet
   SubscriberR    R   t	   PublisherR   t   Ratet   is_shutdownR   t   Nonet   Timet   nowt   headert   stampt   frame_idt   npt   aranget   pit   cost   sint   pointst   appendR   t   publisht   sleep(
   t   pubt   ratet   restricted_zonet   cxt   cyt   radiust
   resolutiont   anglet   xt   y(    (    sD   /home/shado/pepper_sim_ws/src/restricted_zone/src/restricted_zone.pyt   create_restricted_zone   s$    	# t   __main__(   R
   t   std_msgs.msgR    t   sensor_msgs.msgR   t   geometry_msgs.msgR   t   numpyR   R   R*   t   __name__R   R   t   ROSInterruptException(    (    (    sD   /home/shado/pepper_sim_ws/src/restricted_zone/src/restricted_zone.pyt   <module>   s   		