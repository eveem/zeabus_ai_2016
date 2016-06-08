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
        ## self.object = String ('bury')
        self.req = ['red','green']

    def find_cone (self, n):
        print 'will find ' + self.target[i] + 'cone'
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            object_data = self.detect(self.object,String(self.target[n]))
            object_data = object_data.data

            if object_data.appear :
                if self.aicontrol.is_center([object_data.x,object_data.y],-80,80,-80,80):
                    print 'Center'
                    break
                else :
                    print 'Not Center'
                    count -= 0.5

                vx = self.aicontrol.adjust (object_data.x/100, -0.4, -0.2, 0.2, 0.4)
                vy = self.aicontrol.adjust (object_data.y/100, -0.4, -0.2, 0.2, 0.4)
                self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

    def fine_table (self, x, n):
        print 'will find table'
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            table_data = self.detect(String('table'),'''String(self.target[n])''')
            table_data = table_data.data

            if table_data.appear :
                if self.aicontrol.is_center([table_data.x,table_data.y],-50,50,-50,50):
                    print 'Center table'
                    if x == 1:
                        find_xmark (n)
                    break
                else :
                    print 'Not Center table'
                    count -= 0.5

                vx = self.aicontrol.adjust (table_data.x/100, -0.4, -0.2, 0.2, 0.4)
                vy = self.aicontrol.adjust (table_data.y/100, -0.4, -0.2, 0.2, 0.4)
                self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

    def find_xmark (self, n):
        print 'will find xmark'
        count = 50
        while not rospy.is_shutdown() and not self.aicontrol.is_fail(count):
            xmark_data = self.detect(String('xmark'),String(self.target[n]))
            xmark_data = xmark_data.data

            if xmark_data.appear :
                if self.aicontrol.is_center([xmark_data.x,xmark_data.y],-50,50,-50,50):
                    print 'Center table'
                    break
                else :
                    print 'Not Center table'
                    count -= 0.5

                vx = self.aicontrol.adjust (xmark_data.x/100, -0.4, -0.2, 0.2, 0.4)
                vy = self.aicontrol.adjust (xmark_data.y/100, -0.4, -0.2, 0.2, 0.4)
                self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

    def run (self, x):
        for i in xrange(2):
            find_cone (i)
            print 'drive z to cone ' + self.target[i]
            self.aicontrol.drive_z() ## drive to cone
            ## grap
            self.aicontrol.drive_z (0)
            find_table (x, i)
            ## drop

if __name__ == '__main__':
    bury_mission = BuryMission()
    #command
    bury_mission.run(1) ## on mark = 1 , on table = 0
    print "finish bin"
