#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        while not rospy.is_shutdown():
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "base_link"
            #t.header.frame_id = "base_footprint"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "laser_link"
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.5

            q = quaternion_from_euler(0, 0, 3.14159)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
