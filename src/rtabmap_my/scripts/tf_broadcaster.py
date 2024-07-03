#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast_transforms():
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)  # 10 Hz

    while not rospy.is_shutdown():
        t = TransformStamped()
        
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        br.sendTransform((t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
                         (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w),
                         t.header.stamp, t.child_frame_id, t.header.frame_id)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    try:
        broadcast_transforms()
    except rospy.ROSInterruptException:
        pass
