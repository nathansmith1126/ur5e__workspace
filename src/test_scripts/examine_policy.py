import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import os, pickle 
import numpy as np
import rospy
import moveit_commander
import numpy as np
from collections import deque
import os, pickle
from src.Utils.gym_envs import UR5eGridEnvwDFA, UR5e_TQ_agent, env_info2UR5e_grid_env
from src.Utils.AUTOMATA.auto_funcs import DFAMonitor, create_UR5e_traj_DFA
from typing import Optional, Sequence 

# file_name = "q_table_20251/219_113456.pkl" # complete q-table for pick and place but no env/agent parameter info
# file_name = "q_table_20251219_143256.pkl"   # incomplete q-table for pick and place with env/agent parameter info
# file_name = "q_table_20251220_205846.pkl" # complete q-table for pick and place with env/agent params
# file_name = "q_table_20251221_155900.pkl" # complete for fetch and return with one state
file_name = "q_table_20251222_143433.pkl" # complete for fetch and return with multiple start states
# file_name = "q_table_20251230_112103.pkl" # complete for flexible start, fixed finish with multiple middle states

path = os.path.join(os.getcwd(), "src", "RL_models", file_name)

with open(path, "rb") as f:
    policy_data = pickle.load(f)
    
q_table = policy_data['q']
rewards = policy_data['episode_returns']
episode_lengths = policy_data['episode_lengths']
env_info   = policy_data['env_info']
TQL_params = policy_data['TQL_agent_params']


# script to examine saved policy q-tables
test_policy_bool = True 

if test_policy_bool:
    env = UR5eGridEnvwDFA(**env_info)
    q_agent = UR5e_TQ_agent(UR5e_env=env, **TQL_params)

    # start new episode
    obs, info = env.reset()
    done = False
    ep_ret = 0.0
    ep_len = 0
    ep_td_sum = 0.0

    # episode loop
    while not done:
        # discretize observation
        obs_discrete = q_agent.discrete_obs(obs)
        
        # map obs to tuple for q-table indexing
        obs_key = tuple(np.asarray(obs_discrete, dtype=int))

        # select action according to loaded q-table
        action = np.argmax(q_table[obs_key])
        
        # step in environment
        print(f"Current action is {env.action_map[action]}")
        next_obs, reward, terminated, truncated, info = env.step(action)
        print(f"current reward is {reward}")
        done = bool(terminated or truncated)

        ep_ret += float(reward)
        ep_len += 1
        obs = next_obs


# plot rewards over episodes and episode lengths

plt.figure(1)
plt.plot(rewards)
plt.xlabel('Episode')
plt.ylabel('Total Reward')
plt.title('Rewards Over Episodes')

plt.figure(2)
plt.plot(episode_lengths)
plt.xlabel('Episode')
plt.ylabel('Episode Length')
plt.title('Episode Lengths Over Episodes')

plt.show()

# supplementary to generate agents and environments

# hard pick and place trajectory
# trajectory = [
#                 [-1, -1, 4], 
#                 [1, 1, 5]
#                         ]

# # fetch and return
# # trajectory = [ 
# #               [3, 3, 6], 
# #               [-2, -1, 4], 
# #               [3, 3, 6]
# #               ]


# # environment with trajectory DFA monitor
# ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory=trajectory)

# dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# start_state = [4,3,6]

# goal_state = [-3, -3, 3]
# grid_size_array = [0.10, 0.10, 0.10]
# auto_reward_scaler = 1.0

# # safe configs
# q_12 = [np.pi/2,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# q_23 = [np.pi,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# q_34 = [-np.pi/2,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# q_41 = [0.0,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  

# # q_1 = [np.pi/4,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# # q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0] 
# # q_3 = [-3*np.pi/4,-np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# # q_4 = [-np.pi/4,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]

# # safe_configs = [q_12, q_23, q_34, q_41, q_1, q_2, q_3, q_4]
# safe_configs = [q_12, q_23, q_34, q_41]

# env = UR5eGridEnvwDFA(start_states=start_state, 
#                     goal_state=goal_state, 
#                     grid_size_array=grid_size_array, 
#                     DFA_monitor=dfa_monitor, 
#                     DFA_alphabet_dict=alphabet_dict, 
#                     safe_configs=safe_configs, 
#                     auto_reward_scaler=auto_reward_scaler)

#  # progress tracking variables
#     PRINT_EVERY        = 5            # episodes
#     MA_WINDOW          = 100           # moving-average window
#     ep_returns         = deque(maxlen=MA_WINDOW)
#     ep_lengths         = deque(maxlen=MA_WINDOW)
#     temp_diff_errors   = deque(maxlen=MA_WINDOW)  # optional (see below)
#     angle_size         = np.pi/4  # discretization angle size for each joint used in Q-table

#     # intiialize TQ-learning agent
#     q_agent = UR5e_TQ_agent(UR5e_env=env, 
#                             max_steps=max_steps, 
#                             learning_rate=learning_rate, 
#                             angle_size=angle_size)

