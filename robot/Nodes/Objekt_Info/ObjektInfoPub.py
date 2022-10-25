#!/usr/bin/env python3
 # license removed for brevity
from tkinter import RADIOBUTTON
import rospy
from geometry_msgs.msg import Vector3

def Objekt_Info_Publisher(A):
    #rospy.init_node("ortsVorgabe")
    rospy.loginfo("ortsVorgabe gestartet")
    pub = rospy.Publisher("/lageTeil", Vector3, queue_size=10)
   # rospy.init_node('RobotBereitschaft', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    
    if A is True:
        msg = Vector3(0.5, 0.1, 0.6)
        pub.publish(msg)
        #rospy.loginfo(RobotBereit)
    #    pub.publish(RobotBereit)
    #    rate.sleep()
    

    rate.sleep()   
        


if __name__ == '__main__':
    try:
        Objekt_Info_Publisher( A = True )
        #rospy.Timer(rospy.Duration(1/100), RB(A=True))
    except rospy.ROSInterruptException:
        pass
