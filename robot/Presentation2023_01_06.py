#! /usr/bin/env python3
from geometry_msgs.msg import Vector3 
from std_msgs.msg import Bool
import rospy
from math import pi    # fabs, cos, sqrt
# import schon definierte funktionen im Move_Sina_Gripper
from sina_main_presi import *
# Um die Nodes in andere Directory zu importieren, sollen wir erst sys importieren und dann den Pfad dazu eingeben.
import sys
sys.path.insert (1, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Robot_Bereit')
from RobotBereitPub import Roboter_Zustand
sys.path.insert (2, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Teil_Abholung')
from TeilAbholungPub import Abholung_Zustand
sys.path.insert (3, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Dispenser')
from DispenserPub import Dispenser_Zustand
from tf.transformations import quaternion_from_euler
import math
import numpy as np
##############################################
## Konstante Werte  definieren
#A = 2.0 * pi
A   = -pi/8     # Winkel der TCP zum drehen
B   = +pi/8 	    # Winkel der TCP zum drehen
x1  = 0.15       # x Position 
y1  = 0.5	    # y Postion
y2  = 0.48
z1  = 0.13  # z + Abstand
z2  = 0.33	    # z Position
z3  = 0.2	    # Aufheben: Bewegung in z achse
x4_0= 0.25
y4_0= 0.3
z4_0= 0.5 
x4  = 0.2	    # neue Position
y4  = 0.35
z5  = 0.2
z4  = 0.5
y5  = -0.16
zeit= 250        # wie lange sollte der Roboter-Node auf die Nachrit von der Kamera-Node warten.
##############################################
# Konstante Werte zur Berechnung der Bilinear-Interpolation-Gleichung zur Kalibrierung des Roboters

# Postion der 4 Punkte auf der Platte bezüglich der Roboter-Ursprung (Istwert)
X_Ist_1 = 0.22065       # X vom Punkt 1 in RViz
Y_Ist_1 = 0.13756       # Y vom Punkt 1 in RViz
X_Ist_2 = 0.56137       # X vom Punkt 2 in RViz
Y_Ist_2 = 0.1314        # Y vom Punkt 2 in RViz
X_Ist_3 = 0.22805       # X vom Punkt 3 in RViz
Y_Ist_3 = 0.53549       # Y vom Punkt 3 in RViz
X_Ist_4 = 0.56846       # X vom Punkt 4 in RViz
Y_Ist_4 = 0.530882      # Y vom Punkt 4 in RViz

# Postion der 4 Punkte auf der Platte bezüglich der Roboter-Ursprung (Sollwert)
X_SOLL_1 = 0.220        # X vom Punkt 1 in SolidWorks
Y_SOLL_1 = 0.14         # Y vom Punkt 1 in SolidWorks
X_SOLL_2 = 0.56         # X vom Punkt 2 in SolidWorks
Y_SOLL_2 = 0.14         # Y vom Punkt 2 in SolidWorks
X_SOLL_3 = 0.22         # X vom Punkt 3 in SolidWorks
Y_SOLL_3 = 0.54         # Y vom Punkt 3 in SolidWorks
X_SOLL_4 = 0.56         # X vom Punkt 4 in SolidWorks
Y_SOLL_4 = 0.54         # Y vom Punkt 4 in SolidWorks
# Postion der 4 Punkte auf der Platte aus der Kamera
X_CAM_1 = 0.270750677083        # X vom Punkt 1 in Kamera
Y_CAM_1 = 0.04171875        # Y vom Punkt 1 in Kamera
X_CAM_2 = 0.391686979167         # X vom Punkt 2 in Kamera
Y_CAM_2 = 0.0425         # Y vom Punkt 2 in Kamera
X_CAM_3 = 0.2078239583         # X vom Punkt 3 in Kamera
Y_CAM_3 = 0.36046875         # Y vom Punkt 3 in Kamera
X_CAM_4 = 0.39052395833         # X vom Punkt 4 in Kamera
Y_CAM_4 = 0.3596875         # Y vom Punkt 4 in Kamera

# Abwichung (Error) zwischen ISt-, und Sollwert in X-Richtung
Q_X_1 = X_SOLL_1 - X_Ist_1 # Error vom Punkt 1 in X-Richtung
Q_X_2 = X_SOLL_2 - X_Ist_2  # Error vom Punkt 2 in X-Richtung
Q_X_3 = X_SOLL_3 - X_Ist_3  # Error vom Punkt 3 in X-Richtung
Q_X_4 = X_SOLL_4 - X_Ist_4  # Error vom Punkt 4 in X-Richtung

# Abwichung (Error) zwischen ISt-, und Sollwert in Y-Richtung
Q_Y_1 = Y_SOLL_1 - Y_Ist_1     # Error vom Punkt 1 in Y-Richtung
Q_Y_2 = Y_SOLL_2 - Y_Ist_2     # Error vom Punkt 2 in Y-Richtung
Q_Y_3 = Y_SOLL_3 - Y_Ist_3   # Error vom Punkt 3 in Y-Richtung
Q_Y_4 = Y_SOLL_4 - Y_Ist_4     # Error vom Punkt 4 in Y-Richtung

# Abwichung (Error) zwischen ISt-, und Sollwert in X-Richtung
# E_X_1 =  X_Ist_1 - X_CAM_1 # Error vom Punkt 1 in X-Richtung
# E_X_2 =  X_Ist_2 - X_CAM_2 # Error vom Punkt 2 in X-Richtung
# E_X_3 =  X_Ist_3 - X_CAM_3  # Error vom Punkt 3 in X-Richtung
# E_X_4 =  X_Ist_4 - X_CAM_4  # Error vom Punkt 4 in X-Richtung

# # Abwichung (Error) zwischen ISt-, und Sollwert in Y-Richtung
# E_Y_1 = Y_Ist_1 - Y_CAM_1   # Error vom Punkt 1 in Y-Richtung
# E_Y_2 = Y_Ist_2 - Y_CAM_2     # Error vom Punkt 2 in Y-Richtung
# E_Y_3 = Y_Ist_3 - Y_CAM_3   # Error vom Punkt 3 in Y-Richtung
# E_Y_4 = Y_Ist_4 - Y_CAM_4     # Error vom Punkt 4 in Y-Richtung

# Abwichung (Error) zwischen ISt-, und Sollwert in X-Richtung
E_X_1 = -0.01      # Error vom Punkt 1 in X-Richtung
E_X_2 = 0.0025           # Error vom Punkt 2 in X-Richtung
E_X_3 = 0.0008       # Error vom Punkt 3 in X-Richtung
E_X_4 = 0.0115      # Error vom Punkt 4 in X-Richtung

# Abwichung (Error) zwischen ISt-, und Sollwert in Y-Richtung
E_Y_1 = 0.0009      # Error vom Punkt 1 in Y-Richtung
E_Y_2 = -0.0029      # Error vom Punkt 2 in Y-Richtung
E_Y_3 = 0.005       # Error vom Punkt 3 in Y-Richtung
E_Y_4 = -0.0030     # Error vom Punkt 4 in Y-Richtung
 

##############################################
RobotBereit = True
AbholBereit = True
Dispenser = False

def callback_Position_Subscriber(msg):
        global RobotBereit, AbholBereit, Dispenser


        # Publish a topic: /RoboterBereit   --> True / False
        Roboter_Zustand(RobotBereit)        # publiziert ständig, dass der Roboter bereit ist oder nicht (True / False)
        # Publish a topic: /TeilAbholBereit --> True / False
        Abholung_Zustand(AbholBereit)       # publiziert ständig, dass der Greifer frei ist oder nicht (True / False)
        Dispenser_Zustand(Dispenser)




def main():
    global RobotBereit, AbholBereit, Dispenser
    while True:
        print('Roboter läuft...!!!')
        #addCollisionObjects()
        msg = rospy.wait_for_message("/lageTeil", Vector3, timeout=zeit)       
        #######################################################################################################
        # Transformation zw. Kamera-, und Roboter-Koordinatensystem
        # x0 = msg.x * 1.94841793335877  - 0.144461772082637 
        # y0 = msg.y * (1.313444207879) + 0.249837381024666
        # x0_K =  (msg.y * 1.17232860067279 + 0.221199472843068) 
        # y0_K= (msg.x * (-2.08011195043537) + 0.217292574197138 ) 
        x0_K =  (msg.x * 1.81287133447629 - 0.148708096619466) 
        y0_K= (msg.y * (1.27084815724816) + 0.075388953316953)
        #x0_K =  msg.x
        #y0_K= msg.y 
        z0 = 0.20
        print('X, Y Ohne Interpolation:',x0_K,y0_K)
        WinkelKamera = msg.z - 45 #- 90     # Null Richtung bei echtem Roboter ist im 45 grad definiert.
        if WinkelKamera < -135:          # WinkelKamera: Bereich vom Winkel der Nase aus der Kamera-Node ist von 0 bis 360° -->Verteilen in: [-180, 0] und [0, 180]
            WinkelKamera= -(360 + WinkelKamera +45)
        else:
            WinkelKamera = WinkelKamera
        #######################################################################################################
    ###########################################################################################################
    # Berechnung der Bilinear-Interpolation-Gleichung zur Kalibrierung des Roboters
        # X_Matrix = np.array([X_SOLL_4 - x0_K , x0_K - X_SOLL_1])
        # Y_Matrix = np.array([[Y_SOLL_4 - y0_K], [y0_K - Y_SOLL_1]])
        # E_Matrix_X = np.array([[Q_X_1 , Q_X_2], [Q_X_3 , Q_X_4]])
        # E_Matrix_Y = np.array([[Q_Y_1 , Q_Y_2], [Q_Y_3 , Q_Y_4]])
        # F_x = (1/((X_SOLL_4 - X_SOLL_1)*(Y_SOLL_4 - Y_SOLL_1))) * (X_Matrix.dot(E_Matrix_X.dot(Y_Matrix)))
        # F_y = (1/((X_SOLL_4 - X_SOLL_1)*(Y_SOLL_4 - Y_SOLL_1))) * (X_Matrix.dot(E_Matrix_Y.dot(Y_Matrix)))
        X_Matrix = np.array([X_Ist_4 - x0_K , x0_K - X_Ist_1])
        Y_Matrix = np.array([[Y_Ist_4 - y0_K], [y0_K - Y_Ist_1]])
        E_Matrix_X = np.array([[E_X_1 , E_X_3], [E_X_2 , E_X_4]])
        E_Matrix_Y = np.array([[E_Y_1 , E_Y_3], [E_Y_2 , E_Y_4]])
        F_x = (1/((X_Ist_4 - X_Ist_1)*(Y_Ist_4 - Y_Ist_1))) * (X_Matrix.dot(E_Matrix_X.dot(Y_Matrix)))
        F_y = (1/((X_Ist_4 - X_Ist_1)*(Y_Ist_4 - Y_Ist_1))) * (X_Matrix.dot(E_Matrix_Y.dot(Y_Matrix)))
        print('Error in X und Y Richtung:',F_x,F_y)
        x0  = float(x0_K + F_x)
        y0  = float(y0_K + F_y)
    ############################################################################################################
        print('Kalibrierte Position:',x0,y0)
        quotientLoeschen = y0/x0# 
        #print(quotientLoeschen)# 
        Theta = math.atan(quotientLoeschen)
        #print(Theta)
        #print(x0,y0,WinkelKamera)
        go_home()
        go_home_joints()
        RobotBereit = False
        AbholBereit = False
        
        open_gripper()
        print('Position wurde empfangen')
        GoJoint(Theta)
        go_to_position(x0, y0, z4, v=0.1, a=0.08)
        quad = quaternion_from_euler(math.radians(WinkelKamera),math.radians(180),math.radians(0))

        go_to_pose_goal(x0, y0, z3, v=0.05, a=0.01, W=quad[0], Roll=quad[1],Pitch= quad[2], Yaw=quad[3])

        #go_to_position(x0, y0, z3, v=0.1, a=0.1)
        go_to_position(x0, y0, z3-0.024, v=0.05, a=0.005)

        print('Roboter ist an der Objekt-Position')

        grasp_client(0.003)

        AbholBereit = False

        go_to_position(x0, y0, z3+0.08, v=0.05, a=0.005)

        #go_to_position(x4, y4, z2, v=0.1, a=0.03)


        
        #go_to_joint_state(0)
        #Dispenser = True
        #go_to_joint_state(math.radians(-165))
        #go_to_position(x4, y4+0.15, z2+0.2, v=0.2, a=0.2)
        #Dispenser = True
        #go_to_joint_state(math.radians(165))

        #go_to_joint_state(A)
        #Dispenser = False
        #rospy.sleep(1)

        #go_to_position(x0, y0, z2, v=0.2, a=0.08)
        open_gripper()
    
        print('Roboter ist an der Ziel-Position')
        go_home()
        go_home_joints()
        
        RobotBereit = True
        AbholBereit = True
    
        print (RobotBereit)
    
        rospy.sleep(2)
    

if __name__ == '__main__':
    
    rospy.init_node('Roboter_Node', anonymous=True)
    #Objektinfo aus der Kamera-Node abboniert -> Roboter fährt zu der Position
    sub = rospy.Subscriber("/VibTisch", Bool, callback_Position_Subscriber)
    
    main()