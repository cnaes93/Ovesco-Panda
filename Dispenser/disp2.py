def main():
    #1
    rospy.init_node('Dispenser_Node', anonymous=True)
    #rospy.Subscriber("/Dispenser", Bool, callback_Dispenser_Subscriber)
    #rospy.spin()
    #2
    setup()
    while True:
        print('....')
        Dispenser = rospy.wait_for_message("/Dispenser", Bool, 30)
        Dispenser = Dispenser.data
        #print(Dispenser)
        if Dispenser:
            print ('Dispenser startet..!') 
            GPIO.output(ledPin, GPIO.HIGH)
        else:
            print ('Dispenser ausgeschaltet!')
            GPIO.output(ledPin, GPIO.LOW)



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        destroy()
