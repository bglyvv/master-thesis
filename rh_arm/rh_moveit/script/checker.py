#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Range

class RotateAndDetect:
    def __init__(self):
        rospy.init_node('rotate_and_detect', anonymous=True)
        
        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object
        self.robot = moveit_commander.RobotCommander()
        
        # Instantiate a PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Instantiate a MoveGroupCommander object for the arm
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        
        # Instantiate a MoveGroupCommander object for the gripper
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Initial positions and steps
        self.rate = rospy.Rate(10)  # 10 Hz
        self.min_angle = -math.radians(166)
        self.max_angle = math.radians(166)
        self.step = math.radians(1)  # step size in radians
        self.current_angle = self.min_angle
        self.direction = 1  # 1 for increasing, -1 for decreasing
        self.object_detected = False
        self.object_position = None
        self.object_range = None

        # Subscriber for the IR sensor
        rospy.Subscriber('/sensor/ir_front', Range, self.sensor_callback)

        # Publisher for the gripper
        self.gripper_pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

    def sensor_callback(self, data):
        if data.range < data.max_range:
            self.object_detected = True
            self.object_position = self.current_angle  # Save the current angle as the object's position
            self.object_range = data.range
            rospy.loginfo(f"Object detected at range: {data.range} meters, angle: {math.degrees(self.object_position)} degrees")

    def rotate_joint(self):
        while not rospy.is_shutdown() and not self.object_detected:
            joint_goal = self.arm_group.get_current_joint_values()
            joint_goal[0] = self.current_angle  # Only panda_joint1 moves

            self.arm_group.go(joint_goal, wait=True)
            rospy.loginfo(f"Current angle: {math.degrees(self.current_angle)} degrees")

            self.current_angle += self.direction * self.step

            # Reverse direction if limits are reached
            if self.current_angle > self.max_angle:
                self.current_angle = self.max_angle
                self.direction = -1
            elif self.current_angle < self.min_angle:
                self.current_angle = self.min_angle
                self.direction = 1

            self.rate.sleep()

        if self.object_detected:
            rospy.loginfo("Stopping movement due to object detection.")
            self.move_to_object()

    def move_to_object(self):
        rospy.loginfo("Moving to object position.")

        # Get the current pose of the end effector
        current_pose = self.arm_group.get_current_pose().pose

        # Update the pose based on the detected object's position
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = current_pose.position.x + self.object_range * math.cos(self.object_position)
        target_pose.position.y = current_pose.position.y + self.object_range * math.sin(self.object_position)
        target_pose.position.z = current_pose.position.z - 0.1 
        target_pose.orientation = current_pose.orientation

        # Set the target pose for the arm group
        self.arm_group.set_pose_target(target_pose)

        # Plan and execute the motion
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        rospy.sleep(2)

        rospy.loginfo("Closing the gripper to grasp the object.")
        self.control_gripper([0.04, 0.04]) 
        rospy.sleep(2)

        rospy.loginfo("Moving the object to the place position.")
        self.move_to_target(0.5, 0.5, 0.1)

    def control_gripper(self, positions):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ['rh_p12_rn', 'rh_l1']

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(1)

        traj.points.append(point)
        self.gripper_pub.publish(traj)

    def move_to_target(self, x, y, z):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.w = 1.0

        self.arm_group.set_pose_target(pose_goal)
        plan = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

        rospy.sleep(2)

        rospy.loginfo("Releasing the gripper to place the object.")
        self.control_gripper([0.0, 0.0])  # Open the gripper to release the object

if __name__ == '__main__':
    try:
        detector = RotateAndDetect()
        detector.rotate_joint()  # Start rotating and detecting
    except rospy.ROSInterruptException:
        pass
