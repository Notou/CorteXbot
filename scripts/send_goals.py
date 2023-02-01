#!/usr/bin/env python

import rospy
import sys
import os
import actionlib
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def reach_goal(posX, posY):
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()
	
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = posX
	goal.target_pose.pose.position.y = posY

	goal.target_pose.pose.orientation.w = 1


	client.send_goal(goal)
	wait = client.wait_for_result()

	if not wait:
		rospy.logerr("Server not available!")
		rospy.signal_shutdown("Server not available!")
	else:
		return client.get_result()


if __name__ == "__main__":
	try:
		rospy.init_node("goals_client_py", anonymous=True)
		listXY=[]
		for i in range((len(sys.argv)-1)/2):
			listXY.append([float(sys.argv[i*2+1]),float(sys.argv[i*2+2])])
		
		for i in range(len(listXY)):
			result = reach_goal(listXY[i][0], listXY[i][1])
			if result:
				rospy.loginfo("Reached goal number "+str(i+1))
		rospy.loginfo("Final goal reached! Returning to base.")
		#os.system("python return_home.py")
	except rospy.ROSInterruptException:
		rospy.loginfo("Interrupt, navigation finished.")

