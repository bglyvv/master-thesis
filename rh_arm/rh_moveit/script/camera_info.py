#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import CameraInfo

def camera_info_callback(data):
    # Extract the camera matrix
    camera_matrix = data.K
    fx = camera_matrix[0]  # Focal length in x direction
    fy = camera_matrix[4]  # Focal length in y direction
    cx = camera_matrix[2]  # Principal point x-coordinate
    cy = camera_matrix[5]  # Principal point y-coordinate

    print("-----------------------------")
    print(data.header.frame_id)
    rospy.loginfo(f"Focal length (fx): {fx}")
    rospy.loginfo(f"Focal length (fy): {fy}")
    rospy.loginfo(f"Principal point (cx, cy): ({cx}, {cy})")
    print("-----------------------------")


def main():
    rospy.init_node('camera_info_listener', anonymous=True)
    rospy.Subscriber("/left_arm_camera/left_camera/camera_info", CameraInfo, camera_info_callback)
    rospy.Subscriber("/right_arm_camera/right_camera/camera_info", CameraInfo, camera_info_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
