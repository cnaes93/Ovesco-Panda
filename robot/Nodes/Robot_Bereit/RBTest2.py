#!/usr/bin/env python3
 # license removed for brevity
import rospy
from std_msgs.msg import Bool
from RobotBereitPub import RB
from RobotBereitSub import RBSub


def Aktion():
    for x in range (1,40):
        if x < 20:
            A=True
            RB(A)
            
        elif x >= 20 :
            A=False
            RB(A)
            #RBSub()


if __name__ == '__main__':
    try:
        Aktion()
    except rospy.ROSInterruptException:
        pass