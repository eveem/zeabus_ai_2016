import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class NavigateMission (object):

    def __init__ (self):
        print "Now do Navigation Channel"
        # subscribe vision
        self.aicontrol = AIControl()

    def run (self):
        #while not found bottom border of navigate
            #move down to find
        #move up to set position
        #play style
        #stop

if __name__ == '__main__':
    navigate_mission = NavigationMission()
    navigate_mission.run()
    print "finish Navigation Channel"
