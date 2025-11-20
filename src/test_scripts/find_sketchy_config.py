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
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor
from typing import Optional, Sequence   

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

# easy settings
# start_state = [-2,-2,2]
# goal_state = [-2, 1, 2]
# grid_size_array = [0.25, 0.25, 0.25 ]


env = UR5eGridEnvwDFA(start_states=start_state, 
                    goal_state=goal_state, 
                    grid_size_array=grid_size_array, 
                    DFA_monitor=dfa_monitor, 
                    pos_tolerance=pos_tolerance, 
                    plan_time=plan_time)

# simple long plan
# trajectory = [
#               [-2, 2, 4], 
#               [-2, 2, 2], 
#               [-2, 2, 4], 
#               [-2, 0, 4], 
#               [-2, 0, 2], 
#               [-2, 0, 4], 
#               [-2, -2, 4], 
#               [-2, -2, 2], 
#               [-2, -2, 4], 
#               ]

# hard plan over robot base
# q = [0.0, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  
# trajectory = [
#               q,
#               [0, 2, 4],
#               q,
#               [-1, -1, 4]
#                       ]

trajectory = [
              [0, 2, 4],
              [-1, -1, 4]
                      ]

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
              

obs, info = env.reset()
print(f"Initial Observation: {obs}")


# env.joint_angle_move(q)

# no action space is used
for state in trajectory:
    success_bool = False
    while not success_bool:
        # first determine if the state is a joint configuration or a grid state
        if len(state) == 3:
            # grid state
            success_grid_bool, _, _, _ = env.grid_move(state)
            if env.current_grid_state == state:
                success_bool = True
                time.sleep(0.5)
        elif len(state) == 6:
            # joint configuration
            success_joint_bool = env.joint_angle_move(state)
            if success_joint_bool:
                success_bool = True
                time.sleep(0.5)
        else:
            raise ValueError("State must be either a grid state of length 3 or a joint configuration of length 6.")
        
        