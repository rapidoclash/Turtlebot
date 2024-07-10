#!/usr/bin/env python3

import RPI.GPIO as GPIO

from time import sleep

import rospy

from std_msgs.msg import Int32

from turtlebot3_led.srv import choisirCouleur, choisirCouleurResponse

GPIO. setmode (GPIO.BOARD) 
GPIO.setwarnings (False)

red = 15
green = 13
blue = 11

GPIO.setup(red, GPIO.OUT) 
GPIO.setup(blue, GPIO.OUT) 
GPIO.setup(green, GPIO.OUT)

def turnOff():
     GPIO.output (red, 0) 
     GPIO.output (green, 0) 
     GPIO.output (blue, 0)

def greenLED():
     GPIO.output (red, 0)
     GPIO.output (green, 1) 
     GPIO.output (blue, 0)

def redLED():
     GPIO.output (red, 1)
     GPIO.output (green, 0) 
     GPIO.output (blue, 0)

def blueLED():
     GPIO.output (red, 0) 
     GPIO.output (green, 0) 
     GPIO.output (blue, 1)

def violetLED():
     GPIO.output (red, 1) 
     GPIO.output (green, 0) 
     GPIO.output (blue, 1)

def cyanLED():
     GPIO.output (red, 0) 
     GPIO.output(green, 1) 
     GPIO.output (blue, 1)

def total():
     GPIO.output (red, 1) 
     GPIO.output (green, 1) 
     GPIO.output (blue, 1)

def chooseColor(req): 
    print ("Returning [%s, %s, %s] "%(req.red, req.green, req.blue)) 
    if int(req.red) == 0:
        if int (req.green) == 0: 
            if int(req.blue) == 0:
                print ("eteindre")
                turnOff()
                return 0
            else:
                print ("blue light")
                blueLED() 
                return 1 
        else:
            if int (req.blue) == 0: 
                    print ("green light") 
                    greenLED() 
                    return 2 
            else:
                    print ("cyan light")
                    cyanLED()
                    return 3 
    elif (int(req.red) ==1):
        if int(req.green) == 0:
            if int (req.blue) == 0:
                print ("red light")
                redLED()
                return 4
            else:
                print ("violet light")
                violetLED()
                return 5
        else:
            if int (req.blue) == 1:
                print ("total light")
                total()
                return 6 
            else:
                print ("cette couleur n existe pas")
                turnOff() 
                return 100

    print ("Erreur de syntaxe: ecrire rosservice call /ChooseColor 0 1 1") 
    return 100

def chooseColor_server():
    rospy. init_node('chooseColor_server')
    s=rospy.Service('chooseColor', choisirCouleur, chooseColor)
    print ("Ready to choose color")
    rospy.spin()

if __name__=="main":
    chooseColor_server() 


