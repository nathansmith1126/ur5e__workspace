import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.gym_envs import UR5eGridEnv, UR5eGridEnvwDFA
from src.Utils.AUTOMATA.auto_funcs import create_flex_start_fixed_finish_DFA, create_fetch_return_DFA, create_UR5e_xyz_DFA, DFAMonitor, create_UR5e_traj_DFA
from typing import Optional, Sequence   

# booleaan for which environment to test
DFA_bool = True
traj_DFA_bool = False
fetch_return_bool = False
xyz_bool = False
flexible_start_bool = True



if DFA_bool:
    if traj_DFA_bool:
       # easy trajectory
        trajectory = [
                        [3, 4, 6], 
                        [-1, 1, 5], 
                        [1, -1, 5]
                                ]

        # environment with trajectory DFA monitor
        ur5e_DFA, potentials, alphabet_dict = create_UR5e_traj_DFA(trajectory=trajectory)

        dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)

        
        # hard settings
        start_state = [4, 4, 6]
        goal_state = [-3, -3, 3]
        grid_size_array = [0.10, 0.10, 0.10]

        # easy settings
        # start_state = [-2,-2,2]
        # goal_state = [-2, 1, 2]
        # grid_size_array = [0.25, 0.25, 0.25 ]

        # safe configs
        q_12 = [np.pi/2,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_23 = [np.pi,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_34 = [-np.pi/2,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_41 = [0.0,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  
        
        q_1 = [np.pi/4,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0] 
        q_3 = [-3*np.pi/4,-np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_4 = [-np.pi/4,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]

        safe_configs = [q_12, q_23, q_34, q_41, q_1, q_2, q_3, q_4]

        env = UR5eGridEnvwDFA(start_states=start_state, 
                            goal_state=goal_state, 
                            grid_size_array=grid_size_array, 
                            DFA_monitor=dfa_monitor, 
                            DFA_alphabet_dict=alphabet_dict, 
                            safe_configs=safe_configs)
        
        # visualize waypoints in gazebo
        env.initialize_waypoint_markers()
    elif fetch_return_bool:   
        # environment with fetch and return DFA monitor
        fetch_return_DFA_obj, potentials, alphabet_dict = create_fetch_return_DFA() 

        start_states = [
                        [4, 4, 6],
                        [-4, -4, 6],
                         [4, -3, 6], 
                         [3, 4, 6], 
                         [-3, -4, 6],
                         [4, 3, 6],
                         [-4, 4, 6]
                         ]
        dfa_monitor = DFAMonitor(fetch_return_DFA_obj, potential_dict=potentials)
        
        # safe configs
        q_12 = [np.pi/2,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_23 = [np.pi,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_34 = [-np.pi/2,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_41 = [0.0,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  
        
        q_1 = [np.pi/4,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0] 
        q_3 = [-3*np.pi/4,-np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_4 = [-np.pi/4,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]

        safe_configs = [q_12, q_23, q_34, q_41, q_1, q_2, q_3, q_4]
        
        grid_size_array = [0.10, 0.10, 0.10]
        
        goal_state = [-3, -3, 3]
        
        env = UR5eGridEnvwDFA(start_states=start_states,
                            goal_state=goal_state, 
                            grid_size_array=grid_size_array, 
                            DFA_monitor=dfa_monitor, 
                            DFA_alphabet_dict=alphabet_dict, 
                            safe_configs=safe_configs,
                            fetch_return_bool=True)
        
    elif flexible_start_bool:    
        start_state = [-4, -3, 5]
        
        middle_states = [
                            [3, 4, 6], 
                            [-1, 1, 5], 
                            [1, -1, 5],
                            [-3, 3, 6],
                            [2, -2, 5],
                            [-4, 2, 6]
                        ]
        final_state = [ -3, -3, 5 ]
        flex_start_DFA_obj, potentials, alphabet_dict = create_flex_start_fixed_finish_DFA(middle_states=middle_states, 
                                                                            goal_state=final_state)
    
        dfa_monitor = DFAMonitor(flex_start_DFA_obj, potential_dict=potentials)
        
        # environment with flexible start fixed finish DFA monitor
        # safe configs
        q_12 = [np.pi/2,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_23 = [np.pi,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_34 = [-np.pi/2,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_41 = [0.0,      -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]  
        
        q_1 = [np.pi/4,   -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_2 = [3*np.pi/4, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0] 
        q_3 = [-3*np.pi/4,-np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
        q_4 = [-np.pi/4,  -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]

        safe_configs = [q_12, q_23, q_34, q_41, q_1, q_2, q_3, q_4]
        
        grid_size_array = [0.10, 0.10, 0.10]
        
        goal_state = [-3, -3, 3]
        
        env = UR5eGridEnvwDFA(start_states=start_state,
                            goal_state=goal_state, 
                            grid_size_array=grid_size_array, 
                            DFA_monitor=dfa_monitor, 
                            DFA_alphabet_dict=alphabet_dict, 
                            safe_configs=safe_configs,
                            fetch_return_bool=False)
        

    elif xyz_bool:
        # original dfa test 
        # environment with DFA monitor
        ur5e_DFA, potentials = create_UR5e_xyz_DFA()
        dfa_monitor = DFAMonitor(ur5e_DFA, potential_dict=potentials)
        
        # hard settings
        start_state = [2,2,5]
        goal_state = [-3, -3, 3]
        grid_size_array = [0.15, 0.15, 0.15]

        # easy settings
        # start_state = [-2,-2,2]
        # goal_state = [-2, 1, 2]
        # grid_size_array = [0.25, 0.25, 0.25 ]


        env = UR5eGridEnvwDFA(start_states=start_state, 
                            goal_state=goal_state, 
                            grid_size_array=grid_size_array, 
                            DFA_monitor=dfa_monitor)
else:
    env = UR5eGridEnv()


        
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



# for a trajectory of n states and m safe configs
# action_map[0 to n-1] = trajectory states
# action_map[n to n+m-1] = safe configs
action_plan = [6, 4, 10, 6, 1, 3] 
num_reps = 1

success_reps = 0
for rep in np.arange(num_reps):
    obs, info = env.reset()
    print(f"Initial Observation: {obs}")
    for action in action_plan:
        success_exec_bool = False
        while not success_exec_bool:
            print(f"Planned Action: {env.action_map[int(action)]}")
            obs, reward, terminated, truncated, info = env.step(action)
            success_exec_bool = info["success_exec_bool"]
            total_reward += reward
            done = terminated or truncated
            print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")
            if done:
                if terminated:
                    print("Reached terminal state.")
                    success_reps += 1
                break
        
print(f"{success_reps} successful reps ")      

env.delete_all_markers()

    # print(f"Episode finished. Total Reward: {total_reward}")
    # env.close()

# action plans for grid size of 0.25m
# action_plan = [3, 3,
#                1, 1,
#                5, 5]

# action_plan = [3, 3,
#                1, 1,
#                0, 0,
#                1, 1,
#                5, 5]

# action plan for grid size of 0.1m 
# action_plan = [1, 1, 1, 1, 1,
#                3, 3, 3, 3, 3,
#                5, 5]

# action_plan = [3, 3, 3, 3, 3,
#                1, 1, 1, 1, 1,
#                5, 5]

# action plan for easy settings with grid size of 0.25m
# action_plan = [1, 1, 1, 1,
#                3,
#                5]

# action_plan = [5, 5, 5, 5]

# # trajectory that ignores actions
# trajectory = [
#               [1, 1, 1], 
#               [2, 2, 2]
#               ]

# Default start and goal states
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