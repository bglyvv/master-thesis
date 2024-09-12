#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PointStamped

def print_camera_location():
    rospy.init_node('camera_location_printer', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/world', '/left_camera_link', rospy.Time(0))
            (trans2, rot2) = listener.lookupTransform('/world', '/right_camera_link', rospy.Time(0))

            rospy.loginfo(f"Left Camera Position: x={trans[0]}, y={trans[1]}, z={trans[2]}")
            rospy.loginfo(f"Right Camera Position: x={trans2[0]}, y={trans2[1]}, z={trans2[2]}")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

if __name__ == '__main__':
    try:
        print_camera_location()
    except rospy.ROSInterruptException:
        pass