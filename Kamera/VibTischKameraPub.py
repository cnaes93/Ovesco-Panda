#!/usr/bin/env python3
 # license removed for brevity
import rospy
from std_msgs.msg import Bool

def Tisch_Kamera_Zustand(A,B=5):
    pub = rospy.Publisher('/VibTischKamera', Bool, queue_size=10)
    #rospy.init_node('Vibration', anonymous=True)
    rate = rospy.Rate(B)

    if A is True:
        VibTisch = True 
        #rospy.loginfo(RobotBereit)
       
    else:
        VibTisch = False
        
    pub.publish(VibTisch)
    rate.sleep()   


if __name__ == '__main__':
    try:
        Tisch_Kamera_Zustand(A=True,B=5)
    except rospy.ROSInterruptException:
        pass
