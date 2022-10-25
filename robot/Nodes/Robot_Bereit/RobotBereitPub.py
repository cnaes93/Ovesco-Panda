#!/usr/bin/env python3
 # license removed for brevity
import rospy
from std_msgs.msg import Bool

def Roboter_Zustand(A):
    pub = rospy.Publisher('/RobotBereit', Bool, queue_size=10)
    #rospy.init_node('RobotBereitschaft', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    
    if A is True:
        RobotBereit = True 
        #rospy.loginfo(RobotBereit)
    #    pub.publish(RobotBereit)
    #    rate.sleep()
    else:
        RobotBereit = False

    #rospy.Timer(rospy.Duration(10))
    
    pub.publish(RobotBereit)
    rate.sleep()   
        


if __name__ == '__main__':
    try:
        Roboter_Zustand( A = True )
        #rospy.Timer(rospy.Duration(1/100), RB(A=True))
    except rospy.ROSInterruptException:
        pass
