import rospy
from std_msgs.msg import String
''' vision
from zeabus_vision_srv.srv import ____
from zeabus_vision_srv.msg import ____
'''
from AIControl import AIControl

class BouyMission (object):

    def __init__ (self):
        print "Now do bouy"
        # subscribe vision
        self.aicontrol = AIControl()
        self.target = ('red','green')

    def red_then_green (self):
        for i in xrange(2):
            print 'will hit' + self.target[i]
            #while aicontrol.find_object('bouy', self.target[i], 'bouy')
                #move to find
            #while not center
                #move to center
            #move to hit
            #backward
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
