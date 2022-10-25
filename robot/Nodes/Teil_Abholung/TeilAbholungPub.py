#!/usr/bin/env python3
 # license removed for brevity
import rospy
from std_msgs.msg import Bool

def Abholung_Zustand(A):
    pub = rospy.Publisher('/TeilAbholbereit', Bool, queue_size=10)
    #rospy.init_node('TeilAbholung', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if A is True:
        TeilAbholbereit = True 
        #rospy.loginfo(RobotBereit)
        

    else:
        TeilAbholbereit = False
        #pub.publish(TeilAbholbereit)
        #rate.sleep()
        
    pub.publish(TeilAbholbereit)
    rate.sleep()


if __name__ == '__main__':
    try:
        Abholung_Zustand(A=True)
    except rospy.ROSInterruptException:
        pass
