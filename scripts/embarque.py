#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys, tty, termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
#import cv2.cv as cv
import math


maxSpeed = 0.3          # Vitesse maximale
mediumSpeed = 0.2           # Vitesse moyennement limitée
slowSpeed = 0.1             # Vitesse très limitée
emergencyStopDistance = 0.5 # Distance à laquelle le robot s'arrete net
StopDistance = 0.51         # Distance à laquelle le robot freine gentiment pour s'arreter
MediumDistance = 1          # Distance au delà de laquelle le robo va à mediumSpeed
FreedomDistance = 1.5       # Distance au delà de laquelle le robot va à maxSpeed
joystickMultiplier = maxSpeed / 1.5
rotationDistance = 1.5

pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist, queue_size=10)
global currSpeed
currSpeed= 0
global currSpeedRad
currSpeedRad= 0
global lastDirection
lastDirection = "droite"
global chgDirCounter
chgDirCounter = 0
global rotation
rotation = False
global autonomousMode
autonomousMode = True
global movementOn
movementOn = True
global randomOn
randomOn = True
global joyTwist
joyTwist = Twist()


def callback(msg):
    global currSpeed
    global pub
    global lastDirection
    global rotation
    global chgDirCounter
    global currSpeedRad
    global autonomousMode
    global joyTwist
    global randomOn

    bridge = CvBridge()
    twist = Twist()

    if not movementOn:
        return

    # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        # The depth image is a single-channel float32 image
        depth_image = bridge.imgmsg_to_cv2(msg, "32FC1")
    except CvBridgeError, e:
        rospy.logerr( str(e) )


    # depth_image.height = height of the matrix
    # depth_image.width = width of the matrix
    # depth_image[x,y] = the float value in m of a point place a a height x and width y

    # Get min dist
    #print np.asarray(depth_image).shape
    zone = np.concatenate((np.asarray(depth_image[100:280]), np.asarray(depth_image[280:290])), axis= 0)
    (minVal,maxVal,minLoc,maxLoc) = cv2.minMaxLoc(zone)
    rospy.loginfo("Loc : "+str(minLoc)+"Val : "+str(minVal))


    # Partie Rotation
    targetRotation = 0
    if maxVal < 1.6:
        rospy.loginfo("demi tour")
        chgDirCounter = 50
        rotation = True
    elif chgDirCounter > 50:
        rospy.loginfo("demi tour (counter)")
        rotation = True
    elif minLoc[0] < 320 and minVal < rotationDistance :
        rospy.loginfo("Je dois tourner a droite")
        if lastDirection == "gauche":
            chgDirCounter = chgDirCounter + 10
            lastDirection = "droite"
        targetRotation = -0.5
    elif minLoc[0] > 320 and minVal < rotationDistance :
        rospy.loginfo("Je dois tourner a gauche")
        if lastDirection == "droite":
            chgDirCounter = chgDirCounter + 10
            lastDirection = "gauche"
        targetRotation = 0.5
    else:
        targetRotation = 0
        if randomOn:
            targetRotation = targetRotation + 2 * np.random.randn()

    # Partie translation
    targetSpeed = 0
    if minVal < emergencyStopDistance:
        targetSpeed = float("nan")
        rospy.loginfo("Emergency stop!!!")
    elif minVal < StopDistance:
        rospy.loginfo("Stop")
        targetSpeed = 0
    elif minVal < MediumDistance:
        targetSpeed = slowSpeed
        chgDirCounter = chgDirCounter - 2
    elif minVal < FreedomDistance:
        targetSpeed = mediumSpeed
        chgDirCounter = chgDirCounter - 3
    else:
        targetSpeed = maxSpeed
        chgDirCounter = chgDirCounter - 4

    # Gestion demi tour
    if chgDirCounter <= 0:
        chgDirCounter = 0
        rospy.loginfo("Counter : "+str(chgDirCounter))
    if rotation == True:
        targetRotation = -0.6
        targetSpeed = 0
        chgDirCounter = chgDirCounter - 0.5
        if chgDirCounter <= 0:
            rotation = False


    if autonomousMode == False:
        targetSpeed = joystickMultiplier * joyTwist.linear.x
        targetRotation = joyTwist.angular.z
        rospy.logwarn(str(joyTwist.linear.x) )
    else:
        targetRotation = joyTwist.angular.z + targetRotation

    acceleration(targetSpeed)
    accelerationRadiale(targetRotation)

    twist.linear.x = currSpeed
    twist.angular.z = currSpeedRad
    pub.publish(twist)

def acceleration(targetSpeed):
    global currSpeed
    accelerationRate = 0.02  #Fonction Smousse
    if math.isnan(targetSpeed):
        currSpeed = 0
        rospy.logerr("target speed is NaN")
        return
    if targetSpeed>maxSpeed:
        targetSpeed = maxSpeed
    if -targetSpeed > maxSpeed:
        targetSpeed = -maxSpeed

    if currSpeed > targetSpeed + accelerationRate:
        currSpeed = currSpeed - accelerationRate
    elif currSpeed > targetSpeed:
        currSpeed = targetSpeed
    elif currSpeed < targetSpeed - accelerationRate:
        currSpeed = currSpeed + accelerationRate
    elif currSpeed < targetSpeed:
        currSpeed = targetSpeed

def accelerationRadiale(targetSpeed):
    global currSpeedRad

    if currSpeedRad > targetSpeed:
        currSpeedRad = currSpeedRad - 0.1
    elif currSpeedRad < targetSpeed:
        currSpeedRad = currSpeedRad + 0.1

def joystickCallback(msg):
    global autonomousMode
    global joyTwist
    joyTwist = msg
    if msg.linear.y == -1:
        autonomousMode = False
    if msg.linear.y == 1:
        autonomousMode = True

def callbackSwitchMovement(msg):
    global movementOn
    movementOn = msg.data

def listener():
    rospy.loginfo("lanching nav node")
    rospy.init_node('nav', anonymous=True)
    rospy.Subscriber("/camera/depth/image", Image, callback)
    rospy.Subscriber("/turtleshow/command", Twist, joystickCallback)
    rospy.Subscriber("/turtleshow/movement_on", Bool, callbackSwitchMovement)
    rospy.spin()

if __name__ == '__main__':
    listener()
