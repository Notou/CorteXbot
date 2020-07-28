#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import sys, tty, termios
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import math


maxSpeed = 1          # Vitesse maximale
mediumSpeed = 0.5           # Vitesse moyennement limitée
slowSpeed = 0.1             # Vitesse très limitée
emergencyStopDistance = 0.5 # Distance à laquelle le robot s'arrete net
StopDistance = 0.55         # Distance à laquelle le robot freine gentiment pour s'arreter
MediumDistance = 1          # Distance au delà de laquelle le robo va à mediumSpeed
FreedomDistance = 1.5       # Distance au delà de laquelle le robot va à maxSpeed
joystickMultiplier = maxSpeed / 1.5
rotationDistance = 1.5
randomCoefficient = 6

accelerationRate = 0.02
accelerationRateRad = 0.2

class Walker():
    def __init__(self, ):
        self.pub = rospy.Publisher("/cmd_vel_mux/input/teleop",Twist, queue_size=10)

        self.currSpeed= 0
        self.currSpeedRad= 0
        self.chgDirCounter = 0
        self.uTurn = False
        self.autonomousMode = True
        self.movementOn = False
        self.TakeOff = False
        self.randomOn = False
	self.lastRot = 2

        self.lastDirection = "droite"

        self.joyTwist = Twist()

    def callback(self, msg):
        if not self.movementOn:
            return

        bridge = CvBridge()
        twist = Twist()


        #*---- Get closest obstacle information from depth image ----*
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
        (minDistance,maxDistance,minLoc,maxLoc) = cv2.minMaxLoc(np.asarray(depth_image[100:290]))
        rospy.loginfo("Closest obstacle localisation : "+str(minLoc)+"Distance : "+str(minDistance))


        # Partie rotation
        targetRotation = 0
        if maxDistance < 1.6:           #If the field of view is full of obstacles
            rospy.loginfo("demi tour")
            self.chgDirCounter = 50
            self.uTurn = True
        elif self.chgDirCounter > 50:    #If there is too much back and forth rotation without going forward, it means that we are stuck and we need to turn
            rospy.loginfo("demi tour (counter)")
            self.uTurn = True
        elif minLoc[0] < 320 and minDistance < rotationDistance :
            rospy.loginfo("Je dois tourner a droite")
            if self.lastDirection == "gauche":
                self.chgDirCounter = self.chgDirCounter + 10
                self.lastDirection = "droite"
            targetRotation = -1
        elif minLoc[0] > 320 and minDistance < rotationDistance :
            rospy.loginfo("Je dois tourner a gauche")
            if self.lastDirection == "droite":
                self.chgDirCounter = self.chgDirCounter + 10
                self.lastDirection = "gauche"
            targetRotation = 1
        else:
            targetRotation = 0
            if self.randomOn:               #We turn randomly when there is no obstacle to avoid
		if np.random.randint(20) < 1: #One chance over 20 to change direction
			targetRotation = -1 * self.lastRot
			self.lastRot = targetRotation
		else:
			targetRotation = self.lastRot
		print("Random turn: ", targetRotation)

        # Partie translation
        targetSpeed = 0
        if minDistance < emergencyStopDistance:
            targetSpeed = float("nan")
            rospy.loginfo("Emergency stop!!!")
        elif minDistance < StopDistance:
            rospy.loginfo("Stop")
            targetSpeed = 0
        elif minDistance < MediumDistance:
            targetSpeed = slowSpeed
            self.chgDirCounter = self.chgDirCounter - 2
        elif minDistance < FreedomDistance:
            targetSpeed = mediumSpeed
            self.chgDirCounter = self.chgDirCounter - 3
        else:
            targetSpeed = maxSpeed
            self.chgDirCounter = self.chgDirCounter - 4

        # Gestion demi tour
        if self.chgDirCounter <= 0:
            self.chgDirCounter = 0
            rospy.loginfo("Counter : "+str(self.chgDirCounter))
        if self.uTurn == True:
            targetRotation = -0.8
            targetSpeed = 0
            self.chgDirCounter = self.chgDirCounter - 0.5
            if self.chgDirCounter <= 0:
                self.uTurn = False


        if self.autonomousMode == False:
            targetSpeed = joystickMultiplier * self.joyTwist.linear.x
            targetRotation = self.joyTwist.angular.z
            rospy.logwarn(str(self.joyTwist.linear.x) )
        else:
            targetRotation = self.joyTwist.angular.z + targetRotation

        self.acceleration(targetSpeed)
        self.accelerationRadiale(targetRotation)

        twist.linear.x = self.currSpeed
        twist.angular.z = self.currSpeedRad
	print(twist.angular.z)
        self.pub.publish(twist)

    def acceleration(self, targetSpeed):

          #Fonction Smousse
        if math.isnan(targetSpeed):
            self.currSpeed = 0
            rospy.logerr("target speed is NaN")
            return
        if targetSpeed>maxSpeed:
            targetSpeed = maxSpeed
        if -targetSpeed > maxSpeed:
            targetSpeed = -maxSpeed

        if self.currSpeed > targetSpeed + accelerationRate:
            self.currSpeed = self.currSpeed - accelerationRate
        elif self.currSpeed > targetSpeed:
            self.currSpeed = targetSpeed
        elif self.currSpeed < targetSpeed - accelerationRate:
            self.currSpeed = self.currSpeed + accelerationRate
        elif self.currSpeed < targetSpeed:
            self.currSpeed = targetSpeed

    def accelerationRadiale(self, targetSpeed):
	print("target ", targetSpeed, " curr ", self.currSpeedRad)
        if self.currSpeedRad > targetSpeed:
            self.currSpeedRad = self.currSpeedRad - accelerationRateRad
        elif self.currSpeedRad < targetSpeed:
            self.currSpeedRad = self.currSpeedRad + accelerationRateRad

    def joystickCallback(self, msg):
        self.joyTwist = msg
        if msg.linear.y == -1:
            self.autonomousMode = False
        if msg.linear.y == 1:
            self.autonomousMode = True

    def callbackSwitchMovement(self, req):
        self.movementOn = not self.movementOn

    def callbackSwitchTakeoff(self, req):
        self.TakeOff = True
        return TriggerResponse(True,'In progress')

    def GotToBaseCallback(self, req):
        soundThread = threading.Thread(target=self.GoToBase)
        soundThread.daemon = True
        soundThread.start()

    def GoToBase(self):
        call(["roslaunch", "kobuki_auto_docking", "activate.launch", "--screen"])

    def listener(self):
        rospy.loginfo("lanching nav node")
        rospy.init_node('nav', anonymous=True)
        rospy.Subscriber("/camera/depth/image", Image, self.callback)
        rospy.Subscriber("/cortexbot/command", Twist, self.joystickCallback)
        rospy.Service('/cortexbot/movement_on', Trigger, self.callbackSwitchMovement)
        rospy.Service('/cortexbot/take_off', Trigger, self.callbackSwitchTakeoff)
        rospy.Service('/cortexbot/land', Trigger, self.GotToBaseCallback)
        rospy.spin()

if __name__ == '__main__':
    navigator = Walker()
    navigator.listener()
