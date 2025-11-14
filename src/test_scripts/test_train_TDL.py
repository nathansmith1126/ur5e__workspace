import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA, UR5e_TQ_agent
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor
from typing import Optional, Sequence 

# environment with DFA monitor
ur5e_DFA, potentials = create_UR5e_xyz_DFA()
dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# hard settings
# start_state = [2,2,5]
# goal_state = [-3, -3, 3]
# grid_size_array = [0.10, 0.10, 0.10]

# easy settings
start_state = [2,2,3]
goal_state = [1, 1, 2]
grid_size_array = [0.25, 0.25, 0.25 ]


env = UR5eGridEnvwDFA(start_states=start_state, 
                      goal_states=goal_state, 
                      grid_size_array=grid_size_array, 
                      DFA_monitor=dfa_monitor)

# intiialize TQ-learning agent

# agent parameters
max_steps = int(1e3)
q_agent = UR5e_TQ_agent(UR5e_env=env, 
                        max_steps=max_steps)

for step in range(max_steps):
    # start a new episode
    obs, info = env.reset()
    done = False
    
    while not done:
        # select action with epsilon-greedy policy
        action = q_agent.get_action(obs)
        
        # take action in environment
        next_obs, reward, terminated, truncated, info = env.step(action)
        
        # check if episode ended
        if terminated or truncated:
            done = True
        
        # update Q-table, errors and step counter
        q_agent.update_q_table(obs=obs,
                               action=action, 
                               reward=reward,
                               done=done,
                               next_obs=next_obs
                               )
        
        # decay epsilon
        q_agent.decay_epsilon()
        
        # update observation
        obs = next_obs