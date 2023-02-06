#!/usr/bin/env python3
#from picamera2 import Picamera2
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Quaternion
from picamera2 import Picamera2, Preview
from pyzbar.pyzbar import decode
import time
from  tools_Sina import trans_coor
##############################################################################################################
# In Raspberry Pi
# In Raspberry Pi muss Picamera 2 verwendet werden, um ein Foto zu schießen
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(main={"size": (2560, 1920)}, lores={"size": (2560,1920 )}, display="lores")
camera_config = picam2.create_still_configuration(main={"size": (2560, 1920)})
picam2.configure(camera_config)

def take_photo():
    picam2.start()
    time.sleep(2)
    picam2.capture_file("CamCalib.jpg")
    picam2.stop()
##############################################################################################################
# In Laptop
# In Laptop kann cv2 Library verwendet werden, um ein Foto zu schießen
#def take_photo():
#    global cam
#    cam = cv2.VideoCapture(0)
#    cam.set(3,1920) #width=640
#    cam.set(4,1080) #height=480
#    if cam.isOpened():
#        _,frame = cam.read()
#        cam.release()
#        if _ and frame is not None:
#            cv2.imwrite('CamCalib.jpg', frame)


def main():
    global x1_Kamera,y1_Kamera, x2_Kamera, y2_Kamera
    take_photo()
    img = cv2.imread('CamCalib.jpg', 0)
    img_origin = cv2.imread('CamCalib.jpg')
    # cap = cv2.VideoCapture(0)
    # cap.set(3,1920) #width=640
    # cap.set(4,1080) #height=480
    for barcode in decode(img_origin):
        #print(barcode.date)
        myData = barcode.data.decode('utf-8')
        print(int(myData))
        rect_pts = barcode.rect
        if myData:
            pts = np.array([barcode.polygon], np.int32)
            #print(pts)
            point=pts[0].mean(0)
            x ,y = point
            x1= int(x)
            y1 =int(y)
            print (x,y)
            cv2.polylines(img_origin, [pts], True, (255, 0 ,0), 3)
            cv2.circle(img_origin, (x1,y1), radius=10, color=(0, 255, 0), thickness=-1)
            #return(x1_Kamera,y1_Kamera, x2_Kamera, y2_Kamera)
            #if int(myData) == 1:
            if myData == "QR_Code_1":
               # x1_Kamera = y1
               # y1_Kamera = x1
                #print(x1_Kamera,y1_Kamera)
                chosen1=[x1,y1]
                x1_Kamera,y1_Kamera,z=trans_coor(chosen1,2560,1920,400,203)
                print(x1_Kamera,y1_Kamera)
            #elif int(myData) == 2:
            elif myData == "QR_Code_2":
               # x2_Kamera = y1
               
               # y2_Kamera = x1
                chosen2=[x1,y1]
                x2_Kamera,y2_Kamera,z=trans_coor(chosen2,2560,1920,400,203)
                print(x2_Kamera,y2_Kamera)





    cv2.imshow('Result',img_origin)
    cv2.waitKey(6000)
     #######################################################################################
    # Berechnung der Steigerung- sowie Konstante Werte zur Kamera Kalibrierung 
    #print(x1_Kamera,y1_Kamera,x2_Kamera,y2_Kamera)

    # x1_Roboter = 500
    # y1_Roboter = 130
    # x2_Roboter = 250
    # y2_Roboter = 450
    x1_Roboter = 0.44528
    y1_Roboter = -0.256
    x2_Roboter = 0.12277
    y2_Roboter = -0.50058

    Steigung_x = (x2_Roboter - x1_Roboter) / (y2_Kamera - y1_Kamera)
    Steigung_y = (y2_Roboter - y1_Roboter) / (x2_Kamera - x1_Kamera)

    Konstant_x = x2_Roboter - Steigung_x * y2_Kamera
    Konstant_y = y2_Roboter - Steigung_y * x2_Kamera
    #######################################################################################
    pub = rospy.Publisher("/Kalibrierung", Quaternion, queue_size=10)
    rospy.init_node('Kamera_Kalibrierung', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = Quaternion(Steigung_x, Steigung_y, Konstant_x, Konstant_y)
    print(msg)
    for i in (1,200):
        pub.publish(msg)



 

if __name__ == '__main__':
    main()
