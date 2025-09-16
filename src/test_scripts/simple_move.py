#-------------------------
# Import Modules
#-------------------------
import rospy
import numpy as np
import sys
import copy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from moveit_msgs.msg import Constraints, TrajectoryConstraints, PositionConstraint, BoundingVolume
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene

#-------------------------
# Initialize ROS & MoveIt!
#-------------------------
# Initialize moveit_commander
moveit_commander.roscpp_initialize(sys.argv)

# Initialize ROS node
rospy.init_node("ur5e_motion_node", anonymous=True)

# Initialize grid world
grid_world = grid_world()

# Instantiate RobotCommander (interface to robot)
robot = moveit_commander.RobotCommander()

# Get the first MoveGroup available
group_name = robot.get_group_names()[0]
UR5e_move_group = moveit_commander.MoveGroupCommander(group_name)

# PlanningSceneInterface for adding objects
scene = moveit_commander.PlanningSceneInterface()

# Clear any previous constraints
UR5e_move_group.clear_path_constraints()

# add work table to the scene
scene_w_table = add_table2scene(robot, scene)

#-------------------------
# UR5e_move_group move_group plan parameters
#-------------------------

# pos_tolerance = 0.85/8 # meters
pos_tolerance = np.sqrt( 3*(0.125**2) ) # meters
# orientation_tolerance = 0.1 # radians
# joint_tolerance = 0.1 # radians
planning_time = 25 # seconds
planning_attempts = 10

UR5e_move_group.set_planning_time(planning_time)
UR5e_move_group.set_goal_position_tolerance(pos_tolerance)       # 1 tolerance in XYZ
# UR5e_move_group.set_goal_orientation_tolerance(0.05)    # ~2.8 degrees tolerance in radians
UR5e_move_group.set_num_planning_attempts(planning_attempts)  # try 10 times
# UR5e_move_group.set_joint_tolerance(0.01)

#-------------------------
# Move Robot to a target pose
#-------------------------
# Define a simple reachable pose for UR5e

# Get current pose
current_pose = UR5e_move_group.get_current_pose().pose

# for debugging
# current_x = current_pose.position.x
# current_y = current_pose.position.y
# current_z = current_pose.position.z

# get current grid state 
current_grid_state = grid_world.rect_pose_r2grid_state(pose_r=current_pose)

# possible actions 
# action = [1, 0, 0]  # move in +x direction
# action = [0, 1, 0]  # move in +y direction
# action = [0, 0, 1]  # move in +z direction
action = [-1, 0, 0]  # move in -x direction
# action = [0, -1, 0]  # move in -y direction
# action = [0, 0, -1]  # move in -z direction

# get target grid state
# target_grid_state = grid_world.grid_iso_step(action=action, current_grid_state=current_grid_state)
target_grid_state = [1, 1 , 3] # directly above origin

# get target pose from target grid state
target_pose       = grid_world.grid_state2rect_pose_r(grid_state=target_grid_state)
    
# Define target pose
# target_pose = Pose()
# target_pose.position.x = -0.2    
# target_pose.position.y = -0.2   
# target_pose.position.z = 0.4  
# target_pose.orientation.x = 0.0
# target_pose.orientation.y = 0.0
# target_pose.orientation.z = 0.0
# target_pose.orientation.w = -0.0  # neutral orientation

# Set the target pose
UR5e_move_group.set_start_state_to_current_state()
UR5e_move_group.set_pose_target(target_pose, end_effector_link = "tool0")

# check tool of interest
# tool = UR5e_move_group.get_end_effector_link()

# Plan to the pose
success_bool, trajectory, time, error_code = UR5e_move_group.plan()

# Execute only if plan succeeded and has points
if success_bool and len(trajectory.joint_trajectory.points) > 0:
    UR5e_move_group.execute(trajectory, wait=True)
    print("Motion complete!")
else:
    print("No valid plan found. Check constraints or target pose.")

#-------------------------
# Convert plan to cartesian trajectory
#-------------------------

# obtain cartesian trajectory and grid trajectory
cartesian_trajectory, grid_trajectory = grid_world.plan2tool_traj(trajectory, robot=robot)

#-------------------------
# Clean up
#-------------------------
UR5e_move_group.clear_pose_targets()
UR5e_move_group.clear_path_constraints()
moveit_commander.roscpp_shutdown()
