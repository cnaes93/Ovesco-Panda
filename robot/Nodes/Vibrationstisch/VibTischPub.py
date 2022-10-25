#!/usr/bin/env python3
 # license removed for brevity
import rospy
from std_msgs.msg import Bool

def Tisch_Zustand(A):
    pub = rospy.Publisher('/VibTisch', Bool, queue_size=10)
    #rospy.init_node('Vibration', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if A is True:
        VibTisch = True 
        #rospy.loginfo(RobotBereit)
       
    else:
        VibTisch = False
        
    pub.publish(VibTisch)
    rate.sleep()   


if __name__ == '__main__':
    try:
        Tisch_Zustand(A=True)
    except rospy.ROSInterruptException:
        pass
