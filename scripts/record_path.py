#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
import numpy as np

import tf2_ros
import tf2_geometry_msgs
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
        self.travelled_path.header.frame_id = "map"



    def odom_callback(self, msg):
        """
        Called when an odometry message is recieved

        Parameters
        ----------
        msg : Odometry
        """

        try:
            trans = self.tfBuffer.lookup_transform("map", 'odom', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            trans = tf2_ros.TransformStamped()
            rospy.loginfo_throttle_identical(1.0, "Lookup error, defaulting to 0 transform")
        

        stamped_pose = geometry_msgs.msg.PoseStamped()
        stamped_pose.pose = msg.pose.pose
        stamped_pose.header = msg.header
        stamped_pose = tf2_geometry_msgs.do_transform_pose(stamped_pose, trans)

        # rospy.loginfo(stamped_pose.header)

        if len(self.travelled_path.poses) > 0:
            dist_from_previous = np.sqrt(np.square(stamped_pose.pose.position.x - self.travelled_path.poses[-1].pose.position.x) + np.square(
                stamped_pose.pose.position.y - self.travelled_path.poses[-1].pose.position.y))
            if dist_from_previous > self.dist_resolution:
                self.travelled_path.poses.append(stamped_pose)
        else:
            self.travelled_path.poses.append(stamped_pose)

        self.pub_path.publish(self.travelled_path)


    def listen(self):
        rospy.loginfo("lanching path recorder node")
        rospy.init_node('path_rec', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.spin()

if __name__ == '__main__':
    recorder = PathRecorder()
    recorder.listen()