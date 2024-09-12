#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped
rospy.init_node('move_group_python_interface', anonymous=True)

# Initialize the moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

def calculate_magnitude(x, y, z):
    magnitude = math.sqrt(x**2 + y**2 + z**2)
    return magnitude


# Create a publisher for the gripper controller command
pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

# Give some time to establish the connection
rospy.sleep(1)
data = rospy.wait_for_message('/right_force_sensor', WrenchStamped, timeout=5)
print("==========================================================")
rospy.loginfo("Data before closing gripper")
rospy.loginfo("Force x: {}, y: {}, z: {}, Mag: {}".format(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, calculate_magnitude(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z)))
rospy.loginfo("Torque x: {}, y: {}, z: {}, Mag: {}".format(data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z, calculate_magnitude(data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z)))
print("==========================================================")

# Create a JointTrajectory message
traj = JointTrajectory()
traj.header.seq = 0
traj.header.stamp = rospy.Time.now()
traj.header.frame_id = ''
traj.joint_names = ['rh_p12_rn', 'rh_l1']

# Create a JointTrajectoryPoint message
point = JointTrajectoryPoint()
point.positions = [0.64, 0.64]
point.velocities = [0, 0]
point.accelerations = [0, 0]
point.effort = [0, 0]
point.time_from_start = rospy.Duration(secs=10, nsecs=10)

# Add the point to the trajectory
traj.points.append(point)

# Publish the message
pub.publish(traj)

rospy.loginfo("Published gripper command")
rospy.sleep(4.9)
data = rospy.wait_for_message('/right_force_sensor', WrenchStamped, timeout=5)

print("==========================================================")
rospy.loginfo("Data After closing gripper")
rospy.loginfo("Force x: {}, y: {}, z: {}, Mag: {}".format(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, calculate_magnitude(data.wrench.force.x, data.wrench.force.y, data.wrench.force.z)))
rospy.loginfo("Torque x: {}, y: {}, z: {}, Mag: {}".format(data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z, calculate_magnitude(data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z)))
print("==========================================================")






