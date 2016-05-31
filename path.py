import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class PathMission (object):

    def __init__ (self):
        print "Now do path"
        # subscribe vision
        self.aicontrol = AIControl()
        self.found = False

    
