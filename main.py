#!/usr/bin/env python

import rospy
import math
# from gate import GateMission
# from path import PathMission
from bouy import BouyMission
# from navigate import NavigateMission
# from binn import BinnMission
from AIControl import AIControl
# from modbus_ascii_ros.msg import Switch

# done = False
#
# def function (switch):
#     if switch.auto_switch == True:
#         print "Do AI"
#         start()
#         gate()
#         '''
#         path()
#         bouy()
#         path()
#         navigate()
#         pinger()
#         binn()
#         sett()
#         pinger()
#         bury()
#         '''

def start ():
    aicontrol = AIControl()
    aicontrol.drive_z(-1.5)

if __name__ == '__main__':
    aicontrol = AIControl()

    rospy.init_node('main_ai')
    # print "init node complete"
    # start()
    # gate_mission = GateMission()
    # gate_mission.run_without_vision()
    # print 'fin gate in main'
    # path_mission = PathMission()
    # path_mission.run()
    # print 'fin path1 in main'

    # # in future will use mission vision
    # aicontrol.drive([1,0,0,0,0,0])
    # rospy.sleep(0.5)
    # aicontrol.stop()
    # rospy.sleep(0.5)
    bouy_mission = BouyMission()
    bouy_mission.red_then_green()
    print 'fin bouy in main'
    # path_mission = PathMission()
    # path_mission.run()
    # print 'fin path2 in main'
    # binn_mission = BinnMission()
    # binn_mission.run(0)
    # print 'fin binn in main'
    # # navigate_mission = NavigateMission()
    # navigate_mission.run()
    # print 'fin navChannel in main'
