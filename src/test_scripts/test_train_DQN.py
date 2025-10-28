import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA
from src.Utils.AUTOMATA.auto_funcs import create_UR5e_xyz_DFA, DFAMonitor
from typing import Optional, Sequence   
from stable_baselines3 import DQN
from stable_baselines3.common.env_checker import check_env

DFA_bool = True 

if DFA_bool:
    ur5e_DFA, potentials = create_UR5e_xyz_DFA()
    gamma = 0.93
    dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials, gamma=gamma)

    completion_reward = 1.0
    failed_trans_penalty = 0.25
    efficiency_penalty = 0.10
    env = UR5eGridEnvwDFA(DFA_monitor=dfa_monitor, 
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
check_env(env)

# -------------------------------
# Hyperparameters
# -------------------------------
learning_rate = 1e-3
buffer_size = 10_000
learning_starts = 1_000
batch_size = 64
train_freq = 1
gradient_steps = 1
target_update_interval = 500

exploration_initial_eps = 1.0
exploration_final_eps = 0.10
exploration_fraction = 0.20

net_arch = [64, 64]
seed = 0
verbose = 1

# -------------------------------
# Model definition
# -------------------------------
policy_kwargs = dict(net_arch=net_arch)

model = DQN(
    "MlpPolicy",
    env,
    learning_rate=learning_rate,
    gamma=gamma,
    buffer_size=buffer_size,
    learning_starts=learning_starts,
    batch_size=batch_size,
    train_freq=train_freq,
    gradient_steps=gradient_steps,
    target_update_interval=target_update_interval,
    exploration_initial_eps=exploration_initial_eps,
    exploration_final_eps=exploration_final_eps,
    exploration_fraction=exploration_fraction,
    policy_kwargs=policy_kwargs,
    verbose=verbose,
    seed=seed,
)

# -------------------------------
# Training
# -------------------------------
# total_timesteps = 100_000
# model.learn(total_timesteps=total_timesteps)
# model.save("UR5e_test_DQN")

del model # remove to demonstrate saving and loading

# model = DQN.load("UR5e_test_DQN")
model = DQN.load("dqn_ur5e")

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()