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

    def run (self):
        #while not found
            #move to find
        #while not center
            #move to find center
        #set your auv head

if __name__ == '__main__':
    path_mission = PathMission()
    #command
    path_mission.run()
    print "finish path"
