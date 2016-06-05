import rospy
import math
# from gate import GateMission
from path import PathMission
# from bouy import BouyMission
# from navigate import NavigateMission
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
    rospy.init_node('main_ai')
    print "init node complete"
    # start()
    # gate_mission = GateMission()
    # gate_mission.run_without_vision()
    path_mission = PathMission()
    path_mission.run()
