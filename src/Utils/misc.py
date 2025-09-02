#-------------------------
# Import Modules
#-------------------------
import rospy
import sys
import copy
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from moveit_msgs.msg import Constraints, TrajectoryConstraints, PositionConstraint, BoundingVolume
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from typing import Optional, Union, Sequence

def init_scene_with_table(table_center: Optional[Union[Sequence[float], np.ndarray]] = None,
                            table_dims: Optional[tuple] = None) -> tuple:
    '''
    Function to initialize the planning scene with a tabl
    '''
  
    #-------------------------
    # Initialize ROS & MoveIt!
    #-------------------------
    # Initialize moveit_commander
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize ROS node
    rospy.init_node("ur5e_motion_node", anonymous=True)

    # Instantiate RobotCommander (interface to robot)
    robot = moveit_commander.RobotCommander()

    # Get the first MoveGroup available
    group_name = robot.get_group_names()[0]
    manipulator = moveit_commander.MoveGroupCommander(group_name)

    # PlanningSceneInterface for adding objects
    scene = moveit_commander.PlanningSceneInterface()

    # Clear any previous constraints
    manipulator.clear_path_constraints()

    # Add a table to the scene
    #-------------------------
    table_pose = PoseStamped()
    table_pose.header.frame_id = robot.get_planning_frame()
    if table_center is None:
        # Position the table (convert inches to meters)
        table_pose.pose.position.x = -(36/2 - 3.5)/39.37
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = (-2)/(2*39.37)
    else:
        table_pose.pose.position.x = table_center[0]
        table_pose.pose.position.y = table_center[1]
        table_pose.pose.position.z = table_center[2]
    
    if table_dims is None:
        # Table size in meters (width x length x height)
        table_size = (36/39.37, 60/39.37, 2/39.37)
    else:
        table_size = table_dims
   

    # Add the table to the scene
    scene.add_box(name="Table", pose=table_pose, size=table_size)
    
    return robot, manipulator, scene
    