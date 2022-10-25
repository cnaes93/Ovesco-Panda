#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3 

global msg_test

def callback_definiere_ziel(msg): 
   # x0 = msg.x 
    #y0 = msg.y 
   # z0 = msg.z
   # print(msg)
    #print(type(msg))
    #print(type(x0))    
    #print (x0,y0,z0)
    #print(type(x0,y0,z0))
    msg_test = msg
    print('in callback function')
    return(msg_test)
    #print(type(msg.x))

    

def SubCam():
    
    sub = rospy.Subscriber("/lageTeil", Vector3, callback_definiere_ziel)
    print('returned from callback_definiere_ziel')
    #rospy.spin()
    #return msg_test
    
    


if __name__ == '__main__':
    SubCam()