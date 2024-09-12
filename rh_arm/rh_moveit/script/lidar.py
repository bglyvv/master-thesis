#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LiDARObjectDetector:
    def __init__(self):
        rospy.init_node('lidar_object_detector', anonymous=True)
        self.laser_sub = rospy.Subscriber("/lidar/scan", LaserScan, self.laser_callback)

        # Define scanner position offset
        self.scanner_position = np.array([0.03, 0.0, 0.035])

    def laser_callback(self, data):
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))

        # Filter out invalid range values (NaNs and inf)
        valid_indices = np.isfinite(ranges)
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]

        if len(ranges) == 0:
            rospy.logwarn("No valid laser scan data received")
            return

        # Convert polar coordinates to Cartesian coordinates
        x = (ranges * np.cos(angles))+0.02
        y = ranges * np.sin(angles)

        points = np.vstack((x, y)).T

        clusters = self.euclidean_clustering(points, 0.1)

        # Process each cluster to find objects
        for i, cluster in enumerate(clusters):
            if len(cluster) > 5:  # Filter out small clusters
                centroid = np.mean(cluster, axis=0)
                # Add the scanner position offset to the centroid
                world_centroid = centroid + self.scanner_position[:2]
                rospy.loginfo(f"Object {i} detected at x={world_centroid[0]:.2f}, y={world_centroid[1]:.2f}, z={self.scanner_position[2]:.2f}")

    def euclidean_clustering(self, points, tolerance):
        clusters = []
        visited = set()

        def neighbors(point_idx):
            point = points[point_idx]
            distances = np.linalg.norm(points - point, axis=1)
            return set(np.where(distances < tolerance)[0])

        for idx in range(len(points)):
            if idx not in visited:
                cluster = set([idx])
                to_visit = set([idx])
                while to_visit:
                    current_idx = to_visit.pop()
                    visited.add(current_idx)
                    new_neighbors = neighbors(current_idx)
                    to_visit.update(new_neighbors - visited)
                    cluster.update(new_neighbors)
                clusters.append(points[list(cluster)])
        return clusters

if __name__ == '__main__':
    try:
        detector = LiDARObjectDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
