#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def callback(data):
    global B
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    B=data.data
    print(B)
  
def RBSub():
    rospy.init_node('RobotBereitschaf_Subscriber', anonymous=True)

    rospy.Subscriber("RobotBereit", Bool, callback)
    #print(B)


    rospy.spin()

if __name__ == '__main__':
    RBSub()