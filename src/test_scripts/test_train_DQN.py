import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor, create_UR5e_traj_DFA
from typing import Optional, Sequence   
from stable_baselines3 import DQN
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnNoModelImprovement

DFA_bool = True 
TRAJ_bool = True 

if DFA_bool:
    if TRAJ_bool:
        # # easy trajectory
        # trajectory = [
        #                 [3, 3, 4], 
        #                 [4, 5, 5], 
        #                 [5, -5, 5]
        #                         ]

        # # environment with trajectory DFA monitor
        # ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory=trajectory)

        # dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

        # # hard settings
        # start_state = [3,3,6]
        # goal_state = [-3, -3, 3]
        # grid_size_array = [0.10, 0.10, 0.10]
        # auto_reward_scaler = 0.15

        # env = UR5eGridEnvwDFA(start_states=start_state, 
        #                     goal_state=goal_state, 
        #                     grid_size_array=grid_size_array, 
        #                     DFA_monitor=dfa_monitor, 
        #                     DFA_alphabet_dict=alphabet_dict, 
        #                     auto_reward_scaler=auto_reward_scaler)
        
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

    else:
        # dfa info for temporal task
        ur5e_DFA, potentials = create_UR5e_xyz_DFA()
        gamma = 0.93
        dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials, gamma=gamma)
        
        # hard settings
        # start_state = [2,2,5]
        # goal_state = [-3, -3, 3]
        # grid_size_array = [0.10, 0.10, 0.10]

        # easy settings
        start_state = [2,2,3]
        goal_state = [1, 1, 2]
        grid_size_array = [0.25, 0.25, 0.25 ]
            
        # rewards and penalties
        completion_reward = 1.0
        failed_trans_penalty = 0.25
        efficiency_penalty = 0.10
        
        # max episode length
        max_timesteps = 80
        env = UR5eGridEnvwDFA(start_states=start_state, 
                            goal_states=goal_state, 
                            grid_size_array=grid_size_array, 
                            DFA_monitor=dfa_monitor,
                            max_episode_steps=max_timesteps, 
                            completion_reward=completion_reward,
                            failed_trans_penalty=failed_trans_penalty, 
                            efficiency_penalty=efficiency_penalty,
                            auto_reward_scaler=3.0)
else:
    # vanilla grid world environment
    completion_reward = 10.0
    failed_trans_penalty = 0.1
    efficiency_penalty = 0.01
    closer2goal_reward = 0.01
    closest2goal_reward = 1.0

    env = UR5eGridEnv(completion_reward=completion_reward, 
                    failed_trans_penalty=failed_trans_penalty, 
                    efficiency_penalty=efficiency_penalty,
                    closer2goal_reward=closer2goal_reward, 
                    closest2goal_reward=closest2goal_reward)

# check environment
# check_env(env)

# -------------------------------
# Hyperparameters for Training
# -------------------------------
learning_rate = 1e-2
buffer_size = 100_000
learning_starts = 500
batch_size = 64
train_freq = 1
gradient_steps = 1
target_update_interval = 100
gamma = env.DFA_monitor.gamma

# exploration/exploitation
exploration_initial_eps = 1.0
exploration_final_eps = 0.15
exploration_fraction = 0.40

net_arch = [64, 64]
seed = 0
verbose = 1

# -------------------------------
# Model definition
# -------------------------------
policy_kwargs = dict(net_arch=net_arch)

# model = DQN(
#     "MlpPolicy",
#     env,
#     learning_rate=learning_rate,
#     gamma=gamma,
#     buffer_size=buffer_size,
#     learning_starts=learning_starts,
#     batch_size=batch_size,
#     train_freq=train_freq,
#     gradient_steps=gradient_steps,
#     target_update_interval=target_update_interval,
#     exploration_initial_eps=exploration_initial_eps,
#     exploration_final_eps=exploration_final_eps,
#     exploration_fraction=exploration_fraction,
#     policy_kwargs=policy_kwargs,
#     verbose=verbose,
#     seed=seed,
# )

# # -------------------------------
# # Create a callback method to check for improvements
# # -------------------------------


# stop_cb = StopTrainingOnNoModelImprovement(
#     max_no_improvement_evals=5,   # patience: stop after 5 evals without improvement
#     min_evals=5,                  # wait at least 5 evals before checking
#     verbose=1,
# )

# eval_cb = EvalCallback(
#     env,
#     eval_freq=1_000,              # run eval every N training steps
#     best_model_save_path="./logs/best",
#     log_path="./logs",
#     deterministic=True,
#     render=False,
#     callback_after_eval=stop_cb,  # wires the patience logic
# )

# # -------------------------------
# # Training
# # -------------------------------
# total_timesteps = 10_000
# model.learn(total_timesteps=total_timesteps, callback=eval_cb)
# model.save("UR5e_test_DQN_0.1")

# del model # remove to demonstrate saving and loading

# model = DQN.load("UR5e_test_DQN")
model = DQN.load("UR5e_test_DQN_0.1")

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(int( action ))
    if terminated or truncated:
        obs, info = env.reset()