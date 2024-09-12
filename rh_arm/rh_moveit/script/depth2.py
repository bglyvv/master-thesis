#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class StereoObjectDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('stereo_object_detector', anonymous=True)

        self.bridge = CvBridge()
        self.left_image_sub = rospy.Subscriber("left_arm_camera/left_camera/image_raw", Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber("right_arm_camera/right_camera/image_raw", Image, self.right_image_callback)
        
        self.camera_matrix_left = np.array([[528.4337, 0, 320.5], [0, 528.4337, 240.5], [0, 0, 1]], dtype=np.float32)
        self.camera_matrix_right = np.array([[528.4337, 0, 320.5], [0, 528.4337, 240.5], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs_left = np.zeros((5, 1))
        self.dist_coeffs_right = np.zeros((5, 1)) 
        
        self.T_left = np.array([0.5, 0, 0], dtype=np.float32)
        self.R_left = quaternion_from_euler(0, 0.523599, 2.0944)

        self.T_right = np.array([-0.5, 0, 0], dtype=np.float32)
        self.R_right = quaternion_from_euler(0, 0.523599, 1.047198)

        self.T = self.T_right - self.T_left

        self.Q = np.array([[1, 0, 0, -self.camera_matrix_left[0, 2]],
                           [0, 1, 0, -self.camera_matrix_left[1, 2]],
                           [0, 0, 0, self.camera_matrix_left[0, 0]],
                           [0, 0, -1/self.T[0], 0]], dtype=np.float32)

        self.left_image = None
        self.right_image = None

    def left_image_callback(self, data):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def right_image_callback(self, data):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        if self.left_image is not None and self.right_image is not None:
            self.process_stereo_images()

    def process_stereo_images(self):
        # Convert images to grayscale
        gray_left = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)

        # Compute disparity map
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(gray_left, gray_right)

        # Normalize the disparity map for visualization
        disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

        # Display disparity map
        cv2.imshow("Disparity Map", disparity_normalized)
        cv2.waitKey(1)

        # Detect object in the left image
        obj_x, obj_y = self.detect_object(gray_left)

        if obj_x is not None and obj_y is not None:
            # Get disparity value at the object's location
            disparity_value = disparity[obj_y, obj_x]

            # Reproject image to 3D
            points_3D = cv2.reprojectImageTo3D(disparity, self.Q)
            obj_3D = points_3D[obj_y, obj_x]

            x, y, z = obj_3D

            rospy.loginfo(f"Object 3D position: x={x}, y={y}, z={z}")

    def detect_object(self, image):
        # Simple object detection using contours (replace with your detection method)
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed_edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(closed_edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                return x + w // 2, y + h // 2
        return None, None

if __name__ == '__main__':
    try:
        sod = StereoObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
