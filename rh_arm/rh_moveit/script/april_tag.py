#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import apriltag
import numpy as np
import tf
from geometry_msgs.msg import Pose
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander, roscpp_initialize, roscpp_shutdown

class AprilTagDetectorROS:
    def __init__(self):
        rospy.init_node('april_tag_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/left_arm_camera/left_camera/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/left_arm_camera/left_camera/camera_info", CameraInfo, self.camera_info_callback)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame_id = 0

        self.tf_listener = tf.TransformListener()
        self.camera_pos = None
        self.camera_orientation = None

        # Initialize AprilTag detector
        try:
            self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
            rospy.loginfo("AprilTag detector initialized successfully.")
        except Exception as e:
            rospy.logerr(f"Failed to initialize AprilTag detector: {e}")
            self.detector = None

        # Initialize MoveIt!
        roscpp_initialize(sys.argv)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")

        rospy.loginfo(f"Arm group end effector link: {self.arm_group.get_end_effector_link()}")

    def camera_info_callback(self, data):
        self.camera_matrix = np.array(data.K).reshape((3, 3))
        self.dist_coeffs = np.array(data.D)
        rospy.loginfo("Camera info received.")

    def image_callback(self, data):
        if self.detector is None:
            rospy.logerr("AprilTag detector is not initialized.")
            return

        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logerr("Camera info not yet received.")
            return

        try:
            rospy.loginfo(f"Processing frame ID: {self.frame_id}")
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Get the camera position and orientation dynamically
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/world', '/left_camera_link', rospy.Time(0))
                self.camera_pos = np.array(trans)
                self.camera_orientation = np.array(rot)

                # Print the camera position and orientation
                rospy.loginfo(f"Camera position: {self.camera_pos}")
                rospy.loginfo(f"Camera orientation (quaternion): {self.camera_orientation}")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TF lookup failed")
                return

            detections = self.detector.detect(gray)
            rospy.loginfo(f"Number of detections: {len(detections)}")

            for detection in detections:
                self.draw_detection(frame, detection)
                self.print_tag_info(detection)
                self.calculate_tag_pose(detection)

            cv2.imshow('AprilTag Detection', frame)
            cv2.waitKey(1)

            self.frame_id += 1

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")

    def draw_detection(self, frame, detection):
        for idx in range(len(detection.corners)):
            cv2.line(frame, tuple(detection.corners[idx-1, :].astype(int)),
                     tuple(detection.corners[idx, :].astype(int)),
                     (0, 255, 0), 2)

    def print_tag_info(self, detection):
        tag_family = detection.tag_family.decode('utf-8')
        tag_id = detection.tag_id
        center = detection.center
        corners = detection.corners
        rospy.loginfo(f"Detected tag {tag_family} with ID {tag_id} at center {center}")
        rospy.loginfo(f"Corners: {corners}")

    def calculate_tag_pose(self, detection):
        tag_size = 0.2  # Size of the AprilTag in meters
        object_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
        ])

        image_points = detection.corners

        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.dist_coeffs)

        if ret:
            x, y, z = tvec.flatten()
            rospy.loginfo(f"Translation vector (camera frame): x={x}, y={y}, z={z}")
            rospy.loginfo(f"Rotation vector: {rvec.flatten()}")
            
            # Transform to world coordinates
            tag_pos_world = self.transform_to_world(rvec, tvec)
            rospy.loginfo(f"Tag position in world frame: x={tag_pos_world[0]}, y={tag_pos_world[1]}, z={tag_pos_world[2]}")
            
            # Move the arm to the detected object
            self.move_to_object(tag_pos_world[0], tag_pos_world[1], tag_pos_world[2])
        else:
            rospy.logerr("Failed to calculate pose")

    def transform_to_world(self, rvec, tvec):
        if self.camera_pos is None or self.camera_orientation is None:
            rospy.logerr("Camera position or orientation is not available")
            return np.array([0, 0, 0])

        # Convert rotation vector to rotation matrix
        R_cam_to_tag, _ = cv2.Rodrigues(rvec)
        

        T_cam_to_tag = np.eye(4)
        T_cam_to_tag[:3, :3] = R_cam_to_tag
        T_cam_to_tag[:3, 3] = tvec.flatten()

        R_world_to_cam = tf.transformations.quaternion_matrix(self.camera_orientation)[:3, :3]
        
        T_world_to_cam = np.eye(4)
        T_world_to_cam[:3, :3] = R_world_to_cam
        T_world_to_cam[:3, 3] = self.camera_pos

        # Compute the final transformation matrix from the tag to the world
        T_world_to_tag = np.dot(T_world_to_cam, T_cam_to_tag)

        # Extract the translation part which is the position of the tag in the world frame
        tag_pos_world = T_world_to_tag[:3, 3]

        return tag_pos_world

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
        detector = AprilTagDetectorROS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
    roscpp_shutdown()
