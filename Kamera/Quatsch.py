#!/usr/bin/env python3
#extra needed files: cnn weights, tools.py, testphoto
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
import tools # ha uyens programm
import cv2
import numpy as np
#import control_rob
import time
import torch
import os
from IPython.display import Image, clear_output, display
import sys
sys.path.insert (1, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Robot_Bereit')
from RobotBereitPub import Roboter_Zustand
sys.path.insert (2, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Teil_Abholung')
from TeilAbholungPub import Abholung_Zustand
sys.path.insert (3, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Vibrationstisch')
from VibTischPub import Tisch_Zustand
from VibTischKameraPub import Tisch_Kamera_Zustand
from std_msgs.msg import Bool
from yolov5 import detect
#test run Yolov5 detection
#1. 'Automatisierung_ObjectDetection_HaUyen'Ordner  in yolov5 Ordner kopieren
#2. 'Rechts' Ordner  in yolov5 Ordner kopieren
#Pfad zu den Dateien relativ definieren

dirname = os.path.dirname(__file__)
relative_path_1 = os.path.join(dirname, 'Automatisierung_ObjectDetection_HaUyen','proto_ObjectDetection_HaUyen','1280','1280_8_300_M','8_300_M.pt')
relative_path_2 = os.path.join(dirname,'rechts','DSC_0376.JPG')
weights = relative_path_1
img_test = relative_path_2
detect.run(source= img_test, weights= weights, imgsz=1280, conf_thres=0.50, save_txt=True, save_conf=True, classes=[2,3])

path1 = os.path.join(dirname,'runs' , 'detect' )
i=len(os.listdir(path1))

RobotBereit = True
AbholBereit = True
TischBereit = True

def callback_AbholBereit_Subscriber(msg):
    global AbholBereit
    #if msg.data is True:
    AbholBereit = msg.data
    return (AbholBereit)

def callback_RobotBereit_Subscriber(msg):
    global RobotBereit
    RobotBereit =msg.data
    return (RobotBereit)

def callback_Tisch_Subscriber(msg):
    global TischBereit
    Tisch_Kamera_Zustand(TischBereit,20)

         
def main():
    global i, TischBereit, AbholBereit, RobotBereit
    rospy.loginfo(" Kamera Objekterkennung gestartet")
    pub  = rospy.Publisher("/lageTeil", Vector3, queue_size=10)
    rate = rospy.Rate(1)
    while True:
        msg  = rospy.wait_for_message("/VibTisch", Bool, 10)
        TischBereit = msg.data
        print(TischBereit)
        if TischBereit and AbholBereit and RobotBereit: 
            i += 1
            try:
                print('ok')
                tools.take_photo()
                print('Saved to photo.jpg')

            except Exception as err:
        # Errors will be thrown if the user does not have a webcam or if they do not
        # grant the page permission to access it.            
                print(str(err))
                
    # object detection with chosen weights -> label index 2: nose, label index 3: component
            photo = os.path.join(dirname,'photo.jpg')
    #photo = '/home/sina/pycharm/yolov5/photo.jpg'
            detect.run(source=photo, weights=weights, imgsz=1280, conf_thres=0.5, save_txt=True, save_conf=True,
                    classes=[2, 3], line_thickness=1)

    # try to load text file with labels
            time.sleep(1)
            try:
                Foto_i = os.path.join(dirname,'runs/detect/exp%d/labels/photo.txt'  % i )
                teil = np.loadtxt(Foto_i)
            except IOError:
        # error will be thrown if there is no correct component -> no file to open
                print('Kein richtiges Teil entdeckt')
                TischBereit = False
                print(TischBereit) 
                #for iii in range (1,10):
                #    TischBereit = False
                 #   Tisch_Kamera_Zustand(TischBereit,20)
                continue
                #Tisch_Zustand(TischBereit) ##############################################################################

            while np.any(teil):

                print('Next component')

        # detecting component with highest confidence
                max_conf, chosen, teile_rest = tools.choose_teil(teil, 3)

        # try to detect matching nose
                try:
                    chosen_nase, end_result, nase_rest = tools.nase_teil(teil, chosen, 2)
                except Exception as e:
            # error will be thrown if there is no matching nose & returns to take_photo
                    print(str(e))
                    #TischBereit = False
                    for i in range (1,10):
                        TischBereit = False
                        Tisch_Kamera_Zustand(TischBereit)
                    #Tisch_Zustand(TischBereit) ##########################################################################
                    break

        # computes angle of rotation
                winkel = tools.position_winkel(end_result)

        # remaining components
                teil = np.concatenate((nase_rest, teile_rest), 0)
                print('Chosen:')
                print('Component coordinates:', end_result[0, 1], ',', end_result[0, 2], ',', end_result[0, 3], ',',
                    end_result[0, 4], 'mit Confidence:', end_result[0, 5])
                print('Nose coordinates:', end_result[1, 1], ',', end_result[1, 2], ',', end_result[1, 3], ',',
                    end_result[1, 4], 'mit Confidence:', end_result[1, 5])
                print('Angle of rotation:', winkel)
                chos_i = os.path.join(dirname,'runs/detect/exp%d/photo.jpg' % i)
# Zeig das Foto                #tools.plot_chosen(chos_i, end_result, 640, 480)

                x,y,z = tools.trans_coor(chosen,640,480,400,203)
                print(x,y,winkel)
                counter =0
                while counter <10:
                    msg = Vector3(x, y, winkel)
                    pub.publish(msg)
                    rate.sleep()
                    counter +=1
        
                print('New Photo')
                print(TischBereit)
                print(AbholBereit)
                print(RobotBereit)
        


if __name__ == '__main__':
    rospy.init_node('QKam_Test', anonymous=True)
    #Objektinfo aus der Kamera-Node abboniert -> Roboter f??hrt zu der Position
    sub = rospy.Subscriber("/VibTisch", Bool, callback_Tisch_Subscriber)
    sub = rospy.Subscriber("/TeilAbholbereit", Bool, callback_AbholBereit_Subscriber)
    sub = rospy.Subscriber("/RobotBereit", Bool, callback_RobotBereit_Subscriber)
    main()
   