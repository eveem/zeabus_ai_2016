import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class BinnMission (object):

    def __init__ (self):
        print "Now do bin"
        # subscribe vision
        self.aicontrol = AIControl()

    def without_cover (self):
        #while not found
            #move to find
        #while not center
            #move to center
        #drop 2 times

    def with_cover (self):
        #while not found
            #move to find
        #while not center
            #move to center
        #grap
        #move up
        #left
        #drop cover
        #right
        #drop 2 times

if __name__ == '__main__':
    binn_mission = BinnMission()
    #command
    binn_mission.without_cover()
    print "finish bin"
