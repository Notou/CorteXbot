#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

# class FixedTFBroadcaster:

#     def __init__(self):
#         self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

#         while not rospy.is_shutdown():
#             # Run this loop at about 10Hz
#             rospy.sleep(0.1)

#             t = geometry_msgs.msg.TransformStamped()
#             t.header.frame_id = "base_link"
#             #t.header.frame_id = "base_footprint"
#             t.header.stamp = rospy.Time.now()
#             t.child_frame_id = "laser_link"
#             t.transform.translation.x = 0.0
#             t.transform.translation.y = 0.0
#             t.transform.translation.z = 0.0

#             q = quaternion_from_euler(0, 0, 3.14159)
#             t.transform.rotation.x = q[0]
#             t.transform.rotation.y = q[1]
#             t.transform.rotation.z = q[2]
#             t.transform.rotation.w = q[3]

#             tfm = tf2_msgs.msg.TFMessage([t])
#             self.pub_tf.publish(tfm)

# if __name__ == '__main__':
#     proiudsdf
#     rospy.init_node('fixed_tf2_broadcaster')
#     tfb = FixedTFBroadcaster()

#     rospy.spin()

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
    static_transformStamped.transform.translation.z = 0.5

    quat = quaternion_from_euler(0, 0, 3.14159)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()