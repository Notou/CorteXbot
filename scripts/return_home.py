#!/usr/bin/env python

import roslib; roslib.load_manifest('kobuki_auto_docking')
import rospy
import sys
import os
import actionlib

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction, AutoDockingGoal
from actionlib_msgs.msg import GoalStatus

def movebase_client():
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = -0.3
	goal.target_pose.pose.position.y = 0
	goal.target_pose.pose.orientation.x = 0
	goal.target_pose.pose.orientation.y = 0
	goal.target_pose.pose.orientation.z = 0
	goal.target_pose.pose.orientation.w = 1
	
	client.send_goal(goal)
	wait = client.wait_for_result()

	if not wait:
		rospy.logerr("Server not available!")
		rospy.signal_shutdown("Server not available!")
	else:
		return client.get_result()


def doneCb(status, result):
  if 0: print('')
  elif status == GoalStatus.PENDING   : state='PENDING'
  elif status == GoalStatus.ACTIVE    : state='ACTIVE'
  elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
  elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
  elif status == GoalStatus.ABORTED   : state='ABORTED'
  elif status == GoalStatus.REJECTED  : state='REJECTED'
  elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
  elif status == GoalStatus.RECALLING : state='RECALLING'
  elif status == GoalStatus.RECALLED  : state='RECALLED'
  elif status == GoalStatus.LOST      : state='LOST'
  # Print state of action server
  print('Result - [ActionServer: ' + state + ']: ' + result.text)

def activeCb():
  if 0: print('Action server went active.')

def feedbackCb(feedback):
  # Print state of dock_drive module (or node.)
  print('Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text)

def dock_drive_client():
  # add timeout setting
  client = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
  while not client.wait_for_server(rospy.Duration(5.0)):
    if rospy.is_shutdown(): return
    print('Action server is not connected yet. still waiting...')

  goal = AutoDockingGoal()
  client.send_goal(goal, doneCb, activeCb, feedbackCb)
  print('Goal: Sent.')
  rospy.on_shutdown(client.cancel_goal)
  client.wait_for_result()

  #print '    - status:', client.get_goal_status_text()
  return client.get_result()


if __name__ == "__main__":
	try:
		rospy.init_node("movebase_client_py", anonymous=True)
		result = movebase_client()
		if result:
			rospy.loginfo("Reached goal!")
		dock_drive_client()
	except rospy.ROSInterruptException:
		rospy.loginfo("Interrupt, navigation finnished.")

