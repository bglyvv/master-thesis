#!/usr/bin/env python3

from gazebo_msgs.srv import SpawnModel
import rospy
from geometry_msgs.msg import Pose
import rospkg

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


# Set the reference frame
reference_frame = 'world'

# Call the spawn_model service to spawn the model
try:
    spawn_model_client(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
    rospy.loginfo(f"Spawned model '{model_name}' with initial pose and orientation.")
except rospy.ServiceException as e:
    rospy.logerr(f"Service call to spawn model failed: {e}")
