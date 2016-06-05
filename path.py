import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class PathMission (object):

    def __init__ (self):
        print "Now do path"
        ## subscribe vision
        srv_name = 'Vision_Service2'
        rospy.wait_for_service(srv_name)
        self.srv = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()

    def run (self):
        while not shutdown:
            if find_object != [-999,-999,-999,-999] :
                object_center = find_object ('path', 'orange') #return list [x,y,area,angle]
                point[0] = aicontrol.adjust (object_center[0]/100, -0.4, -0.1, 0.1, 0.4)
                point[1] = aicontrol.adjust (object_center[1]/100, -0.4, -0.1, 0.1, 0.4)
                if object_center[2] > 400000 :
                    while !(aicontrol.is_center(point, -50, 50, -50, 50)):
                        aicontrol.drive ()
                    aicontrol.stop ()
                    print 'Can go to next mission'
                else :
                    aicontrol.drive ()
            else :

if __name__ == '__main__':
    path_mission = PathMission()
    #command
    path_mission.run()
    print "finish path"
