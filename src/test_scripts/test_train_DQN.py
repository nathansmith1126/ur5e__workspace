import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.gym_envs import UR5eGridEnv
from typing import Optional, Sequence   
from stable_baselines3 import DQN
from stable_baselines3.common.env_checker import check_env


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

# initialize 
model = DQN("MlpPolicy", env, verbose=1)

total_timesteps = 1_000

# learn
model.learn(total_timesteps=total_timesteps, log_interval=4)
model.save("UR5e_test_DQN")

del model # remove to demonstrate saving and loading

model = DQN.load("UR5e_test_DQN")

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()