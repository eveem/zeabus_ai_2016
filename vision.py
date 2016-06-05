#!/usr/bin/env python
from S_Factor import *
import numpy as np
from zeabus_vision_srv.srv import Boom2_Srv
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from sensor_msgs.msg import CompressedImage
import rospy
from operator import itemgetter
import os
import math
import copy
import rospkg

class Vision():

    _instance = None
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(Singleton, cls).__new__(cls, *args, **kwargs)
        return cls._instance

    def __init__ (self):

    def find_object (self, object, req):

        return value

if __name__=='__main__':
    print 'Vision'
    # rospy.init_node('ai_control', anonymous=True) #comment if run main.py
    vision=Vision()
