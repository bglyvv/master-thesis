#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range

def callback(data):
    # Define the threshold range
    min_range = 0.01
    max_range = 2.0

    # Check if the measured range is within the specified range
    if min_range < data.range < max_range:
        rospy.loginfo("Object detected within range: %f meters", data.range)
    else:
        rospy.loginfo("No object within range")

def ir_sensor_listener():
    rospy.init_node('ir_sensor_listener', anonymous=True)
    rospy.Subscriber('/sensor/ir_front', Range, callback)
    rospy.spin()

if __name__ == '__main__':
    ir_sensor_listener()
