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
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningScene

def add_table2scene( robot: RobotCommander,
                          scene: PlanningScene,
                          table_center: Optional[Union[Sequence[float], np.ndarray]] = None,
                            table_dims: Optional[tuple] = None) -> PlanningScene:
    '''
<<<<<<< HEAD
    Function to initialize the planning scene with a table
    Returns:
    robot: RobotCommander - represents robot arm state
    UR5e_move_group: MoveGroupCommander - represents robot arm planner
    scene: PlanningSceneInterface - object housing items in environment
=======
    Function to initialize the planning scene with a tabl
>>>>>>> parent of 1b63faf... modifiedgrid world class
    '''
    #-------------------------
    # Add a table to the scene
    #-------------------------
    
    # initialize table pose
    table_pose = PoseStamped()
    
    # set reference frame
    table_pose.header.frame_id = robot.get_planning_frame()

    # set table pose
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
    
    return scene
    
def add_elbow_constraints(manipulator: MoveGroupCommander, robot: RobotCommander) -> MoveGroupCommander:
    '''
    Adds elbow constraint to manipulator object to ensure Ur5e does not hit the table with any of it's arms
    Inputs:
    manipulator: MoveGroupCommander - UR5e planning object
    Robot: RobotCommander - object for UR5e states and information
    '''
    
    ws_box = SolidPrimitive()
    ws_box.type = SolidPrimitive.BOX
    ws_box.dimensions = [60*2/39.37, 60*2/39.37, 70/39.37]  # width, length, height

    # Pose of the workspace box
    ws_pose = Pose()
    ws_pose.position.x = -(20/2 - 3.5)/39.37
    ws_pose.position.y = 0.0
    ws_pose.position.z = (70+2)/(2*39.37)

    # Bounding volume
    ws_region = BoundingVolume()
    ws_region.primitives = [ws_box]
    ws_region.primitive_poses = [ws_pose]

    # Position constraint message
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = robot.get_planning_frame()
    position_constraint.link_name = "forearm_link"  # elbow link
    position_constraint.target_point_offset = Vector3(0.0, 0.0, -(2)/(2*39.37))
    position_constraint.constraint_region = ws_region
    position_constraint.weight = 1.0

    # Full constraints
    ws_constraint = Constraints()
    ws_constraint.position_constraints = [position_constraint]

    # Trajectory constraints
    ws_traj_constraint = TrajectoryConstraints()
    ws_traj_constraint.constraints = [ws_constraint]

    # increase solving time 
    manipulator.set_planning_time(15.0)

    # Apply constraints to the manipulator
    manipulator.set_path_constraints(ws_constraint)
    return manipulator

# # define start state based on grid size array
# self.start_state = [1,1,3] # easy to get to

# # define goal state based on grid size array
# self.goal_state = [-1,-1,1] # hard to get to


# # map discrete action → xyz translation
# self.action_map = {
#                     0: [1, 0, 0],   # +x
#                     1: [-1, 0, 0],  # -x
#                     2: [0, 1, 0],   # +y
#                     3: [0, -1, 0],  # -y
#                     4: [0, 0, 1],   # +z
#                     5: [0, 0, -1],  # -z
#                     }
        
# done = False
# total_reward = 0.0
# # action_plan = [
# #     [0, 0, -1],  # Move down in Z
# #     [0, 0, -1],  # Move down in Z
# #     [-1, 0, 0],  # Move left in X
# #     [-1, 0, 0],  # Move left in X
# #     [0, -1, 0],  # Move back in Y
# #     [0, -1, 0],  # Move back in Y
# # ]

# action_plan = [1, 1,
#                5, 5,
#                3, 3]

# for action in action_plan:
#     print(f"Planned Action: {env.action_map[int(action)]}")
#     obs, reward, terminated, truncated, info = env.step(action)
#     total_reward += reward
#     done = terminated or truncated
#     print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")
#     if done:
#         print("Reached terminal state.")
#         break
    
# # while not done:
# #     action = env.action_space.sample()  # random action
# #     print(f"Sampled Action: {env.action_map[int(action)]}")
# #     obs, reward, terminated, truncated, info = env.step(action)
# #     total_reward += reward
# #     done = terminated or truncated
# #     print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")

# print(f"Episode finished. Total Reward: {total_reward}")
# env.close()
