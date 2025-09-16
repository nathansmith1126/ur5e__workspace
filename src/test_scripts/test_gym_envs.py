import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.gym_envs import UR5eGridEnv
from typing import Optional, Sequence   
        

# simple test of environment
env = UR5eGridEnv()
obs, info = env.reset()
print(f"Initial Observation: {obs}")


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
        
done = False
total_reward = 0.0
# action_plan = [
#     [0, 0, -1],  # Move down in Z
#     [0, 0, -1],  # Move down in Z
#     [-1, 0, 0],  # Move left in X
#     [-1, 0, 0],  # Move left in X
#     [0, -1, 0],  # Move back in Y
#     [0, -1, 0],  # Move back in Y
# ]

action_plan = [1, 1,
               5, 5,
               3, 3]

for action in action_plan:
    print(f"Planned Action: {env.action_map[int(action)]}")
    obs, reward, terminated, truncated, info = env.step(action)
    total_reward += reward
    done = terminated or truncated
    print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")
    if done:
        print("Reached terminal state.")
        break
    
# while not done:
#     action = env.action_space.sample()  # random action
#     print(f"Sampled Action: {env.action_map[int(action)]}")
#     obs, reward, terminated, truncated, info = env.step(action)
#     total_reward += reward
#     done = terminated or truncated
#     print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")

print(f"Episode finished. Total Reward: {total_reward}")
env.close()
