#!/usr/bin/env python
# This Python file uses the following encoding: utf-8
import rospy
from geometry_msgs.msg import Point
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import BatteryState
import sys, tty, termios
from subprocess import call, check_output
import re

aggregated_charge = Point()
pubCharge = rospy.Publisher("/cortexbot/robot_charge_level", Point, queue_size=10)

def callback(msg):
    if len(msg.status) == 0:
        return
    if msg.status[0].name == 'mobile_base_nodelet_manager: Battery':
        aggregated_charge.x =  float(msg.status[0].values[1].value)
        pubCharge.publish(aggregated_charge)
        if is_battery_empty():
            emergency_return()

def callback_laptop(msg):
    aggregated_charge.y = msg.percentage / 100
    pubCharge.publish(aggregated_charge)
    if is_battery_empty():
        emergency_return()

def is_battery_empty():
    return ((aggregated_charge.x < 0.2 and aggregated_charge.x > 0) or (aggregated_charge.y < 0.2 and aggregated_charge.y > 0))

def emergency_return():
    pass

def listener():
    global aggregated_charge
    rospy.loginfo("lanching monitor node")
    rospy.init_node('batteryMonitor', anonymous=True)
    rospy.Subscriber("/diagnostics", DiagnosticArray, callback)
    rospy.Subscriber("/laptop_charge", BatteryState, callback_laptop)
    rospy.spin()

if __name__ == '__main__':
    listener()
