import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist,TwistStamped,Pose,PoseStamped
from zeabus_vision_srv.srv import Boom_Srv
from zeabus_vision_srv.msg import Boom_Msg
from AIControl import AIControl

class BouyMission (object):

    def __init__ (self):
        print "Now do bouy"
        ## subscribe vision
        srv_name = 'Vision_Service1'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String('bouy')
        self.target = ['green','red']
        self.first_point = Pose()
        self.point = Pose()

    def red_then_green (self):
        self.first_point = self.aicontrol.get_pose()

        for i in xrange(2):
            print 'will hit ' + self.target[i]
            count = 50
            max_area = 0

            while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
                object_data = self.detect(self.object,String(self.target[i]))
                object_data = object_data.data

                if object_data.appear :
                    if object_data.value > max_area : ### 1 ###
                        max_area = object_data.value

                    if object_data.value > 2000 : ### near ###
                        print 'near'
                        vx = 0.5/object_data.value
                        vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.25, -0.1, 0.1, 0.25)
                        vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.30, -0.1, 0.1, 0.30)
                        bc = 80
                    else : ### far ###
                        print 'far'
                        vx = 0.5/object_data.value
                        vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.35, -0.1, 0.1, 0.35)
                        vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.40, -0.1, 0.1, 0.40)
                        bc = 100

                    if self.aicontrol.is_center([object_data.x,object_data.y],-bc,bc,-bc,bc) :
                        self.aicontrol.drive ([vx,0,0,0,0,0])
                        print 'go to bouy'
                    else :
                        self.aicontrol.drive ([0,vy,vz,0,0,0])
                        print 'set to center'

                    rospy.sleep (0.25)

                else :
                    if max_area > const_area :
                        print 'hit'
                        break
                    count -= 1
            ### end while ###

            self.aicontrol.stop ()
            print 'stop state after hit bouy'

            print 'backward'
            self.aicontrol.drive ([-1,0,0,0,0,0])
            rospy.sleep (2)

            print 'go to set point'
            self.aicontrol.goto (self.first_point.position.x,self.first_point.position.y,self.first_point.position.z,1)
            print 'finish ' + self.target[i]
            ### end for ###

        print 'finish 2 bouy'

    def yellow_bouy (self):
        print 'do yellow bouy'
            #while not found
                #move to find
            #while not center
                #move to center
            #up
            #45 down
            #forward
            #45 up
            #backward
            #forward
        print 'finish yellow bouy'

if __name__ == '__main__':
    bouy_mission = BouyMission()
    #command
    bouy_mission.red_then_green()
    bouy_mission.yellow_bouy()
    print "finish bouy"
