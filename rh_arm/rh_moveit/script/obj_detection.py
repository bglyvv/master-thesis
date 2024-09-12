#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from std_msgs.msg import String

class ObjectDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/arm_camera/camera/image_raw", Image, self.image_callback)
        
        # Camera parameters
        self.focal_length = 528.4337  # in pixels
        self.real_object_width = 100.0  # Real object width in cm
        self.camera_center_x = 320.5  
        self.camera_center_y = 240.5 

        # Initialize MoveIt!
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "arm"
        self.gripper_group_name = "gripper"
        self.arm_group = MoveGroupCommander(self.group_name)
        self.gripper_group = MoveGroupCommander(self.gripper_group_name)

        rospy.loginfo(f"Arm group end effector link: {self.arm_group.get_end_effector_link()}")

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Process the image to detect objects
        self.detect_objects(cv_image)

    def detect_objects(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Apply morphological operations to close gaps in edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Find contours in the edges
        contours, _ = cv2.findContours(closed_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate the area of the contour
            area = cv2.contourArea(contour)
            if area > 500:  # Filter out small contours
                # Get the bounding box coordinates
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / h
                if 0.5 < aspect_ratio < 2.0:  # Filter based on aspect ratio
                    # Calculate the distance to the object using the known width of the object
                    distance = (self.focal_length * self.real_object_width) / w
                    rospy.loginfo(f"Distance to object: {distance} cm")

                    # Calculate the object's position in world coordinates
                    obj_x = (x + w/2 - self.camera_center_x) * distance / self.focal_length
                    obj_y = (y + h/2 - self.camera_center_y) * distance / self.focal_length
                    obj_z = distance / 100.0  # Convert to meters

                    rospy.loginfo(f"Object position: x={obj_x}, y={obj_y}, z={obj_z}")

                    # Move the arm to the detected object
                    self.move_to_object(obj_x, obj_y, obj_z)

                    # Draw a rectangle around the detected object
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Optionally, draw the contour
                    cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
                    # Annotate the image with the distance
                    cv2.putText(image, f"{distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the processed image
        cv2.imshow("Object Detection", image)
        cv2.waitKey(1)

    def move_to_object(self, x, y, z):
        pose_goal = Pose()

        # Set the position to the detected object's position
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        # Offset to account for the gripper length to avoid collision
        gripper_length_offset = 0.1
        pose_goal.position.z += gripper_length_offset

        # Maintain the current orientation of the end effector
        current_pose = self.arm_group.get_current_pose().pose
        pose_goal.orientation = current_pose.orientation

        rospy.loginfo(f"Pose goal: {pose_goal}")

        # Set the pose target
        self.arm_group.set_pose_target(pose_goal)

        # Plan to the new pose
        plan = self.arm_group.go(wait=True)

        # Ensure that there is no residual movement
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        self.control_gripper(open=False)

    def control_gripper(self, open=True):
        if open:
            # Open the gripper
            self.gripper_group.set_named_target("open")
        else:
            # Close the gripper
            self.gripper_group.set_named_target("close")
        
        plan = self.gripper_group.go(wait=True)
        self.gripper_group.stop()

if __name__ == '__main__':
    try:
        od = ObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
    roscpp_shutdown()
