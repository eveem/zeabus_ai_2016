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
            cone_data = self.detect(String('cone'),String(self.target[n]))
            cone_data = cone_data.data

            if cone_data.appear :
                if self.aicontrol.is_center([cone_data.x,cone_data.y],-80,80,-80,80):
                    print 'Cone is center'
                    if cone_data.value > 2000 : ### near ###
                        break
                    else :
                        self.aicontrol.drive_z (-1.4) ### depth of cone ###
                else :
                    print 'Cone not center'
                    vx = self.aicontrol.adjust ((cone_data.x/100)/cone_data.value, -0.4, -0.2, 0.2, 0.4)
                    vy = self.aicontrol.adjust ((cone_data.y/100)/cone_data.value, -0.4, -0.2, 0.2, 0.4)
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
                    if cone_data.value > 2000 : ### near ###
                        if x == 1:
                            find_xmark (n)
                        break
                    else :
                        self.aicontrol.drive_z ('''number..''') ### depth of cone ###
                else :
                    print 'Not Center table'
                    vx = self.aicontrol.adjust ((table_data.x/100)/table_data.value, -0.4, -0.2, 0.2, 0.4)
                    vy = self.aicontrol.adjust ((table_data.y/100)/table_data.value, -0.4, -0.2, 0.2, 0.4)
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
                    if xmark_data.value > 2000 : ### near ###
                        break
                    else :
                        self.aicontrol.drive_z ('''number..''') ### depth of table ###
                else :
                    print 'Not Center x_mark'
                    vx = self.aicontrol.adjust ((xmark_data.x/100)/xmark_data.value, -0.4, -0.2, 0.2, 0.4)
                    vy = self.aicontrol.adjust ((xmark_data.y/100)/xmark_data.value, -0.4, -0.2, 0.2, 0.4)
                    self.aicontrol.drive([vx,vy,0,0,0,0])
            else :
                count -= 1
            rospy.sleep(0.25)

    def run (self, x):
        for i in xrange(2):
            find_cone (i)                   ### drive to cone ready to grap ###
            ## grap
            self.aicontrol.drive_z (0)      ### float to surface ###
            self.aicontrol.drive_z (0.1)    ### drive to find table ###
            find_table (x, i)
            ## drop
            ### listen pinger to goto cone again ###

if __name__ == '__main__':
    bury_mission = BuryMission()
    #command
    bury_mission.run(1) ## on mark = 1 , on table = 0
    print "finish bin"
