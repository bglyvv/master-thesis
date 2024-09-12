#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class DepthImageProcessor:
    def __init__(self):
        # Initialize the ROS node
        self.node_name = "depth_image_processor"
        rospy.init_node(self.node_name, anonymous=True)
        
        # Depth image topic
        self.depth_topic = "/depth_camera/depth/depth_image_raw" 
        self.bridge = CvBridge()
        
        # Subscribe to depth image topic
        self.depth_subscriber = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.loginfo(f"{self.node_name} started, listening to {self.depth_topic}")

    def depth_callback(self, data):
        try:
            # Convert the ROS image to a depth image using CV Bridge
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        depth_image = np.nan_to_num(depth_image, nan=5.0)

        # Normalize the depth image for better visualization
        depth_image_normalized = cv2.normalize(depth_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        depth_image_normalized = np.array(depth_image_normalized, dtype=np.uint8)

        # Apply a threshold to isolate objects at certain distances
        _, thresh = cv2.threshold(depth_image_normalized, 60, 255, cv2.THRESH_BINARY_INV)

        # Find contours from the thresholded image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:  # Minimum contour area to consider
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(depth_image_normalized, (x, y), (x+w, y+h), (0, 255, 0), 2)

                # Compute the centroid of the contour
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    distance = depth_image[cy, cx]

                    if distance > 0 and distance < 5:  # Checking valid range
                        rospy.loginfo(f"Object detected at distance: {distance} meters")
                    else:
                        rospy.loginfo("Object detected but out of valid range")
        
        # Display the depth image with the object highlighted
        cv2.imshow("Depth Image", depth_image_normalized)
        cv2.waitKey(1)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down the depth image processor node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    processor = DepthImageProcessor()
    processor.run()

