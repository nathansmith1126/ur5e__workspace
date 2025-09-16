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

env = UR5eGridEnv()

# check environment
check_env(env)

# initialize 
model = DQN("MlpPolicy", env, verbose=1)

# learn
model.learn(total_timesteps=10_000, log_interval=4)
model.save("UR5e_test_DQN")

del model # remove to demonstrate saving and loading

model = DQN.load("UR5e_test_DQN")

obs, info = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()