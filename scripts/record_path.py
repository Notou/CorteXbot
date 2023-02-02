#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
import numpy as np

import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from nav_msgs.msg import Odometry, Path


class PathRecorder():
    """
    Record incoming odometry messages to build up a travelled path over time
    """

    def __init__(self):
        self.pub_path = rospy.Publisher("/cortexbot/travelled_path", Path, queue_size=10)

        self.travelled_path = Path()
        self.dist_resolution = 0.1 # Min travel distance to record from previous pose (to reduce spamming)
        self.travelled_path.header.frame_id = "odom"

    
    def odom_callback(self, msg):
        """
        Called when an odometry message is recieved

        Parameters
        ----------
        msg : Odometry
        """

        stamped_pose = geometry_msgs.msg.PoseStamped()
        stamped_pose.pose = msg.pose.pose
        stamped_pose.header = msg.header
        # rospy.loginfo(stamped_pose.header)

        if len(self.travelled_path.poses) > 0:
            dist_from_previous = np.sqrt(np.square(stamped_pose.pose.position.x - self.travelled_path.poses[-1].pose.position.x) + np.square(stamped_pose.pose.position.y - self.travelled_path.poses[-1].pose.position.y))
            if dist_from_previous > self.dist_resolution:
                self.travelled_path.poses.append(stamped_pose)
        else:
            self.travelled_path.poses.append(stamped_pose)

        self.pub_path.publish(self.travelled_path)

    def listener(self):
        rospy.loginfo("lanching path recorder node")
        rospy.init_node('path_rec', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        rospy.spin()

if __name__ == '__main__':
    recorder = PathRecorder()
    recorder.listener()