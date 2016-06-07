import rospy
import math
from std_msgs.msg import String
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class NavigateMission (object):

    def __init__ (self):
        print "Now do Navigation Channel"
        ## subscribe vision
        srv_name = 'Vision_Service1'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('portal')
        ## self.target = ??

    def run (self):
        print 'run in navigate'
        count = 50
        object_data = Boom_Msg()
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            object_data = self.detect(self.object,'''String(self.target[i])''')
            object_data = object_data.data
            if object_data.appear :
                if self.aicontrol.is_center([object_data.x,object_data.y],-40,40,-40,40):
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    count -= 0.5
                vy = self.aicontrol.adjust (object_data.x/100, -0.5, -0.2, 0.2, 0.5)
                vz = self.aicontrol.adjust (object_data.y/100, -0.5, -0.2, 0.2, 0.5)
                self.aicontrol.drive([1,vy,vz,0,0,0])
            else :
                count -= 1
            rospy.sleep (0.25)

        print 'see portal'
        self.aicontrol.drive_z (1)
        print 'drive style'
        ## style
        ## stop

if __name__ == '__main__':
    navigate_mission = NavigationMission()
    navigate_mission.run()
    print "finish Navigation Channel"
