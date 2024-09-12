#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from gpd_ros.msg import GraspConfigList
import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class GraspDetector:
    def __init__(self):
        rospy.init_node('grasp_detector', anonymous=True)

        # Subscribe to point cloud and grasp topics
        self.point_cloud_sub = rospy.Subscriber('/depth_camera/depth/image_raw', PointCloud2, self.point_cloud_callback)
        self.grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, self.grasp_callback)
        
        self.point_cloud = None
        self.grasps = []

    def point_cloud_callback(self, data):
        # Convert point cloud message to PCL data
        self.point_cloud = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))

    def grasp_callback(self, data):
        self.grasps = data.grasps
        rospy.loginfo(f"Received {len(self.grasps)} grasp configurations")

        # Visualize grasp points
        self.visualize_grasps()

    def visualize_grasps(self):
        if self.point_cloud is None or len(self.grasps) == 0:
            rospy.logwarn("No point cloud data or grasps available for visualization.")
            return
        
        # Create a blank image for visualization
        import matplotlib.pyplot as plt
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, projection='3d')

        # Plot the point cloud
        pc_np = np.array(self.point_cloud)
        ax.scatter(pc_np[:, 0], pc_np[:, 1], pc_np[:, 2], c='blue', marker='o')

        # Plot the grasp points
        for grasp in self.grasps:
            position = grasp.position
            ax.scatter(position.x, position.y, position.z, c='red', marker='^')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

if __name__ == '__main__':
    try:
        detector = GraspDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
