import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import time 
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor, create_UR5e_traj_DFA
from typing import Optional, Sequence   

gazebo_bool = False

# environment with DFA monitor
ur5e_DFA, potentials = create_UR5e_xyz_DFA()
dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# hard settings
standard_delta = 0.10
scale = 1
true_delta = scale * standard_delta
start_state = [3,3,6]
goal_state = [-3, -3, 3]
grid_size_array = [true_delta*1, true_delta*1, true_delta*1]


# moveit planner parameters
# plan_time = 10.0 # seconds
plan_time = 3.0 # seconds
pos_tolerance = 0.05 # meters

start_state = [3, 4, 6]

trajectory = [ 
              start_state,
              [-1, -1, 4],
              [1, 1, 4], 
              [5, 1, 6]
                ]

# environment with trajectory DFA monitor
ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory=trajectory)

dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)


# hard settings

goal_state = [-3, -3, 3]
grid_size_array = [0.10, 0.10, 0.10]

# easy settings
# start_state = [-2,-2,2]
# goal_state = [-2, 1, 2]
# grid_size_array = [0.25, 0.25, 0.25 ]


env = UR5eGridEnvwDFA(start_states=start_state, 
                    goal_state=goal_state, 
                    grid_size_array=grid_size_array, 
                    DFA_monitor=dfa_monitor, 
                    DFA_alphabet_dict=alphabet_dict, 
                    pos_tolerance=pos_tolerance, 
                    plan_time=plan_time)

# visualize waypoints in gazebo if necessary
if gazebo_bool:
    env.initialize_waypoint_markers()
else:
    # pass
    time.sleep(10)

# safe configs
q_14 = [0.0, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  
q_1 = [np.pi/4,  -np.pi/2,   np.pi/2,   0.0,  np.pi/2, 0.0]
q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0 ] 
q_3 = [-3*np.pi/4,  -np.pi/2,   np.pi/2,   np.pi,    np.pi/2,   0.0]
q_4 = [-np.pi/4,  -np.pi/2,   np.pi/2,   np.pi,    np.pi/2,   0.0]

env.joint_angle_move(q_14)  

# reset environment and get initial observation
obs, info = env.reset()
print(f"Initial Observation: {obs}")

# trajectory = [
#               q,
#               [0, 2, 4],
#               q,
#               [-1, -1, 4]
#                       ]

# trajectory = [
#               [0, 2, 4],
#               [-1, -1, 4]
#                       ]


# trajectory = [
#               [3, 3, 7],
#               [1, -3, 7],
#                 [0, -2, 5],
#                 [0, -2, 4],
#                 [0, -1, 4],
#                       ]



# trajectory = [
#               [3, 3, 7],
#               [3, 3, 7],
#               [3, 0, 5],
#               [2, 0, 4],
#               [1, 0, 4],
#               [3, 0, 6], 
#               [0, -1, 6],
#               [0, -1, 4], 
#               [-2, -2, 4]
#                       ]
              

# trajectory = [[2, 2, 10]]


# easier trajectory avoiding robot base
# trajectory = [
#               [3, 3, 7],
#               [3, 0, 7], 
#               [2, 0, 5],
#               [1, 0, 5], 
#               [-2, -2, 4], 
#                       ]
              
# trajectory_follow = [ 
#               [4, 4, 5],
#               [1, 1, 4],
#               [4, 4, 5],
#               [-4, -4, 5],
#               [-1, -1, 4]
#                 ]

# trajectory_follow = [ 
#                      q,
#               [-1, -1, 4],
#               [1, 1, 4]
#                 ]

# bool to indicate whether to use dependable trajectory
dependable_bool = False

# plot indicator
plot_bool = False    

if dependable_bool:
    sleep_time = 0.05
    trajectory_follow = [ q_1, 
                     [-1, -1, 4], 
                     q_4,
                     [1, 1, 4], 
                     q_14
                ]
else:
    # non dependable trajectory 
    # does NOT take conservative actions to avoid failed transitions
    sleep_time = 8.0 
    trajectory_follow = [
                        [-1, -1, 4], 
                        [1, 1, 4]
                         ]
    time.sleep( sleep_time )


# no action space is used
for state in trajectory_follow:
    success_bool = False
    while not success_bool:
        # first determine if the state is a joint configuration or a grid state
        if len(state) == 3:
            # grid state
            success_grid_bool, _, _, _ = env.grid_move(state, plot_bool=plot_bool)
            if env.current_grid_state == state:
                success_bool = True
                time.sleep(sleep_time)
        elif len(state) == 6:
            # joint configuration
            success_joint_bool = env.joint_angle_move(state)
            if success_joint_bool:
                success_bool = True
                time.sleep(sleep_time)
        else:
            raise ValueError("State must be either a grid state of length 3 or a joint configuration of length 6.")


env.joint_angle_move(q_14)
if gazebo_bool:
    env.delete_all_markers()