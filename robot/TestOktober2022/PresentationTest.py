#! /usr/bin/env python3
from geometry_msgs.msg import Vector3 
import rospy
from math import pi    # fabs, cos, sqrt
# import schon definierte funktionen im Move_Sina_Gripper
from sina_main import *
from KameraSub import SubCam
# Um die Nodes in andere Directory zu importieren, sollen wir erst sys importieren und dann den Pfad dazu eingeben.
import sys
sys.path.insert (1, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Robot_Bereit')
from RobotBereitPub import Roboter_Zustand
sys.path.insert (2, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Teil_Abholung')
from TeilAbholungPub import Abholung_Zustand
##############################################
## Werte für definieren
#A = 2.0 * pi
A = -pi/4      #Winkel der TCP vor Eingreifen
B = +pi/4	#Winkel der TCP 	
x1 = 0.3       # x Position 
y1 = 0.3	# y Postion
z1 = 0.17	# z + Abstand
z2 = 0.43	# z Position
z3 = 0.6	# Aufheben: Bewegung in z achse
x4_0 = 0.25
y4_0 = 0.3
z4_0 = 0.5 
x4 = 0.3	#neue Position
y4 = 0.4
z4 = 0.5

y5 = -0.16

##############################################
RobotBereit = True
AbholBereit = True

def callback_Position_Subscriber(msg):
    global x0,y0,z0, RobotBereit, AbholBereit
    x0 = msg.y * 0.968083333333333 + 0.3210240291
    y0 = msg.x * (-1.42586530764292) + 0.747973112888269
    z0 = msg.z + 0.25
    
    go_home()
    go_home_joints()
    RobotBereit = False
    open_gripper()
    print('Position wurde empfangen')
    AbholBereit = False
    go_to_position(x0, y0, z0, v=0.7, a=0.4)


    go_to_position(x0, y0, z1, v=0.7, a=0.4)
    #relative_cartesian_path(z=-0.2)
    #relative_cartesian_path(z=-0.07)

    #go_to_position(x0, y0, z2, v=0.7, a=0.4)
    print('Roboter ist an der Objekt-Position')
    #close_gripper()
    grasp_client(0.005)

    go_to_position(x0, y0, z2, v=0.7, a=0.4)
    #relative_cartesian_path(z=0.07)
    go_to_position(x4_0, y4_0, z4_0, v=0.7, a=0.4)

    go_to_pose_goal(x4_0, y4_0, z4_0, v=0.7, a=0.4,W=1)

    go_to_pose_goal(x4_0, y4, z4_0, v=0.7, a=0.4,W=1)

    
    go_to_joint_state(A)
    go_to_joint_state(B)
    go_to_pose_goal(x4_0, y4_0, z4_0, v=0.7, a=0.4,W=1)

    #go_home_joints()

    #go_to_pose_goal(x4, y5, z2, v=0.7, a=0.4)

    
    
    
    print('Roboter ist an der Ziel-Position')
    go_home_joints()
    open_gripper()
    RobotBereit = True
    AbholBereit = True
    #return (start)
    print (RobotBereit)
    
    rospy.sleep(2)

def main():
    global RobotBereit
    while True:
        print('Roboter läuft...!!!')
        # Publish a topic: /RoboterBereit   --> True / False
        Roboter_Zustand(RobotBereit)        # publiziert ständig, dass der Roboter bereit ist oder nicht (True / False)
        # Publish a topic: /TeilAbholBereit --> True / False
        Abholung_Zustand(AbholBereit)       # publiziert ständig, dass das Objekt abgohlt ist oder nicht (True / False)

if __name__ == '__main__':
    
    rospy.init_node('Test', anonymous=True)
    #Objektinfo aus der Kamera-Node abboniert -> Roboter fährt zu der Position
    sub = rospy.Subscriber("/lageTeil", Vector3, callback_Position_Subscriber)
    main()
   