#!/usr/bin/env python
# This Python file uses the following encoding: utf-8

import rospy
import numpy as np

import tf2_ros
import tf2_geometry_msgs
import tf.transformations
import geometry_msgs.msg
from nav_msgs.msg import Odometry, Path
from slam_toolbox_msgs.srv import SaveMap, SaveMapRequest, SaveMapResponse

class PathRecorder():
    """
    Record incoming odometry messages to build up a travelled path over time
    """

    def __init__(self):
        self.pub_path = rospy.Publisher("/cortexbot/travelled_path", Path, queue_size=10)

        self.travelled_path = Path()
        self.dist_resolution = 0.1 # Min travel distance to record from previous pose (to reduce spamming)
        self.travelled_path.header.frame_id = "map"

        self.map = None
        self.record_folder = ""



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

    def quat_obj_to_array(self, obj):
        """
        Pass from Quaternion object from a ros message (geometry_msgs/Quaternion)
        To a numpy array

        Parameters
        ----------
        obj : geometry_msgs/Quaternion

        Returns
        -------
        np.array
        """
        return np.array([obj.x, obj.y, obj.z, obj.w])

    def save_path_to_file(self, file_name: str, format: str='csv'):
        """
        Save the internal travelled path to a file

        Parameters
        ----------
        file_name : str
            Base file name (with path)
        format : str
            Saving format. Only csv is available for now
        """

        if format == 'csv':
            with open(file_name+"_travelled.csv", "wt") as csv_file:
                print("seq_id, t_sec, t_nsec, x, y, rot")
                for stamped_pose in self.travelled_path.poses:
                    print(stamped_pose.header.seq, stamped_pose.header.stamp.sec, stamped_pose.header.stamp.nsec, sep=", ", end=", ", file=csv_file)    # Stamp
                    print(stamped_pose.pose.position.x, stamped_pose.pose.position.y, sep=", ", end=", ", file=csv_file)                                # Position
                    print(tf.transformations.euler_from_quaternion(self.quat_obj_to_array(stamped_pose.pose.orientation))[-1], file=csv_file)           # Orientation


    def callback_save_to_file(self, req:SaveMapRequest):
        map_name = req.name
        # Will make use of the slam_toolbox service to save map
        try:
            resp1 = self.save_map_proxy(map_name)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # Then save path.
        self.save_path_to_file(map_name)

        return resp1
        


    def listen(self):
        rospy.loginfo("lanching path recorder node")
        rospy.init_node('path_rec', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Service("/cortexbot/save_to_file", SaveMap, self.callback_save_to_file)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.save_map_proxy = rospy.ServiceProxy('/slam_toolbox/save_map', SaveMap)
        rospy.spin()

if __name__ == '__main__':
    recorder = PathRecorder()
    recorder.listen()