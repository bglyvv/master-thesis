#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import math

class ArmReach:
    def __init__(self):
        rospy.init_node('arm_reach', anonymous=True)
        
        # Initialize the moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Instantiate a RobotCommander object
        self.robot = moveit_commander.RobotCommander()
        
        # Instantiate a PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Instantiate a MoveGroupCommander object for the arm
        self.arm_group = moveit_commander.MoveGroupCommander("arm")

    def find_reach(self):
        # Get the current pose of the end effector
        initial_pose = self.arm_group.get_current_pose().pose

        # Try different positions to find the maximum and minimum reach
        max_reach = 0.0
        min_reach = float('inf')

        for angle in range(0, 360, 1):  # Try different angles
            rad = math.radians(angle)
            for dist in range(0, 200, 1):  # Try different distances (in cm)
                x = dist / 100.0 * math.cos(rad)
                y = dist / 100.0 * math.sin(rad)
                z = initial_pose.position.z  # Keep the same height
                
                # Create a target pose
                target_pose = geometry_msgs.msg.Pose()
                target_pose.position.x = x
                target_pose.position.y = y
                target_pose.position.z = z
                target_pose.orientation = initial_pose.orientation  # Keep the same orientation

                self.arm_group.set_pose_target(target_pose)
                
                # Plan to the new state
                plan = self.arm_group.plan()
                
                # Check if the plan is valid
                if plan:
                    distance = math.sqrt(x**2 + y**2)
                    if distance > max_reach:
                        max_reach = distance
                    if distance < min_reach:
                        min_reach = distance

        rospy.loginfo(f"Maximum reach: {max_reach} meters")
        rospy.loginfo(f"Minimum reach: {min_reach} meters")

if __name__ == '__main__':
    try:
        arm_reach = ArmReach()
        arm_reach.find_reach()
    except rospy.ROSInterruptException:
        pass
