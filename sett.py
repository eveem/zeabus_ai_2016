import rospy
import math
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class SettMission (object):

    def __init__ (self):
        print "Now do Set Course"
        ## subscribe vision
        srv_name = 'Vision_Service1'
        rospy.wait_for_service(srv_name)
        print 'service starts'
        self.detect = rospy.ServiceProxy(srv_name, Boom_Srv)
        ## old vision
        self.aicontrol = AIControl()
        self.object = String("sett")
        self.size = ['small','big']
        self.done = 0

    def find_fire (self, character, block_size):
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count) :
            object_data = self.detect(self.object,String(character),String(block_size))   #### should return cover || uncover and should ask about size
            object_data = object_data.data

            if object_data.appear :
                vx = (1/object_data.value)*5000
                vy = self.aicontrol.adjust ((object_data.y/100)/object_data.value, -0.25, -0.1, 0.1, 0.25)
                vz = self.aicontrol.adjust ((object_data.x/100)/object_data.value, -0.30, -0.1, 0.1, 0.30)

                if self.aicontrol.is_center([object_data.x,object_data.y],-50,50,-50,50) :
                    self.aicontrol.drive ([vx,0,0,0,0,0])
                    print 'center of %c '%character + '%s'%block_size
                    break
                else :
                    self.aicontrol.drive ([0,vy,vz,0,0,0])
                    print 'set to center'
                rospy.sleep (0.25)
            else :
                count -= 1

        if count != 0:
            if object_data.cover == 1:
                # grap
                self.aicontrol.drive([0,0.25,0,0,0,0])
                rospy.sleep(0.25)
                # drop
                self.aicontrol.drive([0,-0.25,0,0,0,0])
                rospy.sleep(0.25)
            # fire
            print 'fire to set complete %c'%character + '%s'%block_size
            self.done += 1
        else:
            print 'fail %c'%character + '%s'%block_size

    def run (self, search):
        for i in xrange(2):
            for j in xrange(2):
                find_fire (search[i],self.size[j])
                if self.done == 1:
                    print 'finish %c'%search[i]
                    break
            if self.done == 2:
                print 'finish set course'
                break

if __name__ == '__main__':
    sett_mission = SettMission()
    #command
    sett_mission.run(['N','W'])
    print "finish bin"
