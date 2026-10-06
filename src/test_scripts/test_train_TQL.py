import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
import numpy as np
from collections import deque
import os, pickle, time
from datetime import datetime 
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA, UR5e_TQ_agent
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor, create_UR5e_traj_DFA
from typing import Optional, Sequence 

# environment with xyz DFA monitor
# ur5e_DFA, potentials = create_UR5e_xyz_DFA()
# dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# hard trajectory
trajectory = [
                [4, 4, 6], 
                [-1, -1, 4], 
                [1, 1, 5]
                        ]

# environment with trajectory DFA monitor
ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory=trajectory)

dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

# hard settings
# start_state = [[3,3,6], 
#                [4, 4, 4], 
#                [-4, -4, 4],
#                [-2, 2, 7], 
#                [3, -3, 6]
#                 ] 

start_state = [4,5,6]

goal_state = [-3, -3, 3]
grid_size_array = [0.10, 0.10, 0.10]
auto_reward_scaler = 0.5

# safe configs
q_12 = [np.pi/2,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
q_23 = [np.pi,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
q_34 = [-np.pi/2,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
q_41 = [0.0,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  

# q_1 = [np.pi/4,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0] 
# q_3 = [-3*np.pi/4,-np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
# q_4 = [-np.pi/4,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]

# safe_configs = [q_12, q_23, q_34, q_41, q_1, q_2, q_3, q_4]
safe_configs = [q_12, q_23, q_34, q_41]

env = UR5eGridEnvwDFA(start_states=start_state, 
                    goal_state=goal_state, 
                    grid_size_array=grid_size_array, 
                    DFA_monitor=dfa_monitor, 
                    DFA_alphabet_dict=alphabet_dict, 
                    safe_configs=safe_configs)
# easy settings
# start_state = [-2,-2,2]
# goal_state = [-2, 1, 2]
# grid_size_array = [0.25, 0.25, 0.25 ]

# hard settings
# start_state = [2,2,5]
# goal_state = [-3, -3, 3]
# grid_size_array = [0.10, 0.10, 0.10]

# easy settings
# start_state = [2,2,3]
# goal_state = [1, 1, 2]
# grid_size_array = [0.25, 0.25, 0.25 ]


# env = UR5eGridEnvwDFA(start_states=start_state, 
#                       goal_states=goal_state, 
#                       grid_size_array=grid_size_array, 
#                       DFA_monitor=dfa_monitor)



# learning parameters
max_steps          = int(4e3)
learning_rate      = 0.1

# progress tracking variables
PRINT_EVERY        = 5            # episodes
MA_WINDOW          = 100           # moving-average window
ep_returns         = deque(maxlen=MA_WINDOW)
ep_lengths         = deque(maxlen=MA_WINDOW)
temp_diff_errors   = deque(maxlen=MA_WINDOW)  # optional (see below)
angle_size         = np.pi/4  # discretization angle size for each joint used in Q-table

# intiialize TQ-learning agent
q_agent = UR5e_TQ_agent(UR5e_env=env, 
                        max_steps=max_steps, 
                        learning_rate=learning_rate, 
                        angle_size=angle_size)

episode    = 0
step_count = 0
t0         = time.time()

# training loop
while step_count < max_steps:
    # start new episode
    obs, info = env.reset()
    done = False
    ep_ret = 0.0
    ep_len = 0
    ep_td_sum = 0.0

    # episode loop
    while not done:
        action = q_agent.get_action(obs)
        print(f"Current action is {env.action_map[action]}")
        next_obs, reward, terminated, truncated, info = env.step(action)
        print(f"current reward is {reward}")
        done = bool(terminated or truncated)

        # --- Q update (optionally capture TD error if your agent returns it) ---
        temp_diff = q_agent.update_q_table(
            obs=obs, action=action, reward=reward, done=terminated, next_obs=next_obs
        )
        # If update_q_table returns None, just ignore 'td'
        if temp_diff is not None:
            ep_td_sum += abs(float(temp_diff))

        q_agent.decay_epsilon()
        ep_ret += float(reward)
        ep_len += 1
        obs = next_obs

    # end of episode
    # track episode number and step count
    episode += 1 
    step_count = q_agent.step_counter
    
    # track rewards and length
    ep_returns.append(ep_ret)
    ep_lengths.append(ep_len)
    if ep_td_sum > 0:
        temp_diff_errors.append(ep_td_sum / max(1, ep_len))

    # periodic print
    if episode % PRINT_EVERY == 0:
        eps = getattr(q_agent, "epsilon", None)
        ma_ret = np.mean(ep_returns) if ep_returns else np.nan
        ma_len = np.mean(ep_lengths) if ep_lengths else np.nan
        ma_temp_diff  = np.mean(temp_diff_errors) if temp_diff_errors else np.nan
        elapsed = time.time() - t0
        print(
            f"ep={episode:5d}  "
            f"ret={ep_ret:8.3f}  len={ep_len:4d}  "
            f"eps={eps:.3f}" if eps is not None else f"ep={episode:5d}  ret={ep_ret:8.3f}  len={ep_len:4d}  "
            + f"  MA_RET={ma_ret:8.3f}  MA_LEN={ma_len:5.1f}  MA_TD={ma_temp_diff:8.4f}  "
            f"time={elapsed:6.1f}s"
        )

# SAVE
run_time = time.time() - t0
# target folder
out_dir = os.path.expanduser("~/ur5e_ws/src/RL_models")
os.makedirs(out_dir, exist_ok=True)

# timestamp: YYYYMMDD_HHMMSS
ts = datetime.now().strftime("%Y%m%d_%H%M%S")
out_path = os.path.join(out_dir, f"q_table_{ts}.pkl")

q_agent.save_q_table(path=out_path, time_elapsed=run_time)

# OLD CODE 

# for step in range(max_steps):
#     # start a new episode
#     obs, info = env.reset()
#     done = False
    
#     while not done:
#         # select action with epsilon-greedy policy
#         action = q_agent.get_action(obs)
        
#         # take action in environment
#         next_obs, reward, terminated, truncated, info = env.step(action)
        
#         # check if episode ended
#         if terminated or truncated:
#             done = True
        
#         # update Q-table, errors and step counter
#         temp_diff = q_agent.update_q_table(obs=obs,
#                                action=action, 
#                                reward=reward,
#                                done=done,
#                                next_obs=next_obs
#                                )
        
#         # decay epsilon
#         q_agent.decay_epsilon()
        
#         # update observation
#         obs = next_obs
        
        
