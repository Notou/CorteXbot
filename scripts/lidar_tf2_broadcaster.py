#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    # if len(sys.argv) < 8:
    #     rospy.logerr('Invalid number of parameters\nusage: '
    #                  './static_turtle_tf2_broadcaster.py '
    #                  'child_frame_name x y z roll pitch yaw')
    #     sys.exit(0)
    # else:
    #     if sys.argv[1] == 'world':
    #         rospy.logerr('Your static turtle name cannot be "world"')
    #         sys.exit(0)

    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_link"
    static_transformStamped.child_frame_id = "laser_link"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    quat = quaternion_from_euler(0, 0, 3.14159)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()