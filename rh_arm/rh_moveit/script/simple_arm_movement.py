#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospkg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
rospy.init_node('move_group_python_interface', anonymous=True)

# Initialize the moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object for the arm
group_name = "arm" 
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get information for debugging
planning_frame = move_group.get_planning_frame()

# Load and execute the predefined group state
predefined_state_name = 'pick2'
move_group.set_named_target(predefined_state_name)

# Plan to the new state
plan = move_group.go(wait=True)

move_group.stop()

# Clear pose targets
move_group.clear_pose_targets()

rospy.sleep(5)




from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose

# Initialize the ROS node
# rospy.init_node('spawn_model_node')

# Create a ServiceProxy to call the spawn_model service
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

# Define the model's name
model_name = 'box1'

# Load the SDF model file
# model_xml = open('/home/bacha20/model_editor_models/arm_clynder/model.sdf', 'r').read()
rospack = rospkg.RosPack()
package_path = rospack.get_path('rh_moveit')

# Construct the full path to the SDF model file
model_sdf_path = package_path + '/model/unit_box/model.sdf'

# Load the SDF model file
with open(model_sdf_path, 'r') as model_file:
    model_xml = model_file.read()


# Define the robot namespace and initial pose
robot_namespace = '/foo'
initial_pose = Pose()
initial_pose.position.x = 0.628706 
initial_pose.position.y = 0.0214454   
initial_pose.position.z = 0.03515  
# initial_pose.position.x = 0.626379
# initial_pose.position.y = 0.022767  
# initial_pose.position.z = 0.039142 
# initial_pose.position.z = 0.132106
# Set orientation (quaternion representation)
# initial_pose.orientation.x = 1.629220
# initial_pose.orientation.y = 0.012144
# initial_pose.orientation.z = -0.666762
# initial_pose.orientation.w = 1.0  # Default orientation (no rotation)

# Set the reference frame
reference_frame = 'world'

# Call the spawn_model service to spawn the model
try:
    spawn_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
    rospy.loginfo(f"Spawned model '{model_name}' with initial pose and orientation.")
except rospy.ServiceException as e:
    rospy.logerr(f"Service call to spawn model failed: {e}")


















rospy.sleep(5)

# Create a publisher for the gripper controller command
pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

rospy.sleep(1)

# Create a JointTrajectory message
traj = JointTrajectory()
traj.header.seq = 0
traj.header.stamp = rospy.Time.now()
traj.header.frame_id = ''
traj.joint_names = ['rh_p12_rn', 'rh_l1']

# Create a JointTrajectoryPoint message
point = JointTrajectoryPoint()
point.positions = [1, 1]
point.velocities = [0, 0]
point.accelerations = [0, 0]
point.effort = [0, 0]
point.time_from_start = rospy.Duration(secs=10, nsecs=10)

# Add the point to the trajectory
traj.points.append(point)

# Publish the message
pub.publish(traj)

rospy.loginfo("Published gripper command")






rospy.sleep(15)



moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object for the arm
group_name = "arm" 
move_group = moveit_commander.MoveGroupCommander(group_name)

# Get information for debugging
planning_frame = move_group.get_planning_frame()

# Load and execute the predefined group state
predefined_state_name = 'place' 
move_group.set_named_target(predefined_state_name)

# Plan to the new state
plan = move_group.go(wait=True)

move_group.stop()

# Clear pose targets
move_group.clear_pose_targets()

rospy.sleep(5)





# rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# joint_names:
# - 'rh_p12_rn'
# - 'rh_l1'
# points:
# - positions: [1,1]
#   velocities: [0,0]
#   accelerations: [0, 0]
#   effort: [0,0]
#   time_from_start: {secs: 10, nsecs: 10}" 
