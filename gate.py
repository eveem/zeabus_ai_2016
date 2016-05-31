import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class GateMission (object):

    def __init__ (self):
        print "Now do gate"
        # subscribe vision
        self.aicontrol = AIControl()

    def run_with_vision (self):

    def run_without_vision (self):
        self.aicontrol.drive ([1,0,0,0,0,0])
        rospy.sleep(20) #change time to wait
        print "forward complete"

if __name__ == '__main__':
    gate_mission = GateMission()
    #command
    gate_mission.run_without_vision()
    print "finish gate"
