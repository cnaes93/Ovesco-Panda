#!/usr/bin/env python3
 # license removed for brevity
import rospy
from RobotBereitPub import RB

for x in range (1,10):
    A=True
    RB(A)
    
else:
    A=False


if __name__ == '__main__':
    try:
        RB(A=True)
    except rospy.ROSInterruptException:
        pass