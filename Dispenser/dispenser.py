#! /usr/bin/env python3  
from std_msgs.msg import Bool
import rospy
#import sys # Um die Nodes in andere Directory zu importieren, sollen wir erst sys importieren und dann den Pfad dazu eingeben.
#sys.path.insert (1, '/home/sina/catkin_ws/src/panda_controllers/scripts/Nodes/Dispenser')
#from DispenserPub import Dispenser_Zustand
import RPi.GPIO as GPIO
import time



Dispenser = False
ledPin =11   #pin 11 von Raspberry Pi
def setup():
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(ledPin, GPIO.OUT)
        GPIO.output(ledPin, GPIO.LOW)
        print (' start ')
def loop():
    while True:
        GPIO.output(ledPin, GPIO.HIGH)
        #time.sleep(2)
        GPIO.output(ledPin, GPIO.LOW)
        print ('end')
        time.sleep(1)

def destroy():
        GPIO.cleanup()


def callback_Dispenser_Subscriber(msg):
    global Dispenser
    Dispenser = msg.data
    print(Dispenser)

    if Dispenser:
    #####################################################################################################
        # Hier muss die Befehle zum Steueren des Vibrationstischs geschrieben werden ...
        print ('Dispenser ist jetzt eingeschaltet!')
        #while True:
        GPIO.output(ledPin, GPIO.HIGH)


    else:
        print ('Dispenser ist jetzt eingeschaltet!')
        GPIO.output(ledPin, GPIO.LOW)


        

    #####################################################################################################
    rospy.spin()  

#def main():
 #   global Dispenser
 #   while True:
 #       #TischBereit=rospy.wait_for_message("/VibTisch", Bool,10)
 #           # Publish a topic: /TischBereit   --> True / False
 #       Tisch_Zustand(Dispenser)        # publiziert ständig, dass der Tisch bereit ist oder nicht (True / False)


if __name__ == '__main__':
    rospy.init_node('Dispenser_Node', anonymous=True)
    # Topic "Dispenser" aus der Roboter-Node abboniert -
    sub = rospy.Subscriber("/Dispenser", Bool, callback_Dispenser_Subscriber)
   # main()
