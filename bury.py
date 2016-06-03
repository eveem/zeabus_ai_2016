import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class BuryMission (object):

    def __init__ (self):
        print "Now do Bury Treassure"
        # subscribe vision
        self.aicontrol = AIControl()

    def find_cone (self):

    def grap (self):

    def run (self):
        for i in xrange(2):
            #find_cone
            #grap
            #drive to table
            #drop
            #float

if __name__ == '__main__':
    bury_mission = BuryMission()
    #command
    bury_mission.without_cover()
    print "finish bin"
