import rospy
import math
from gate import GateMission
from path import BouyMission
from AIControl import AIControl

from modbus_ascii_ros.msg import Switch

done = False

def function (switch):
    if switch.auto_switch == True:
        print "Do AI"
        start()
        gate()

def start (self):
    aicontrol = AIControl()
    aicontrol.drive ([0,0,-2,0,0,0])

if __name__ == '__main__':
    rospy.init_node('main_ai')
