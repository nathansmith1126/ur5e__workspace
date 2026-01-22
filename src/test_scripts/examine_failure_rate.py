import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime 
import scipy.stats
from scipy.stats import t 
from collections import deque
import os, pickle 
import rospy
import moveit_commander
from collections import deque
import os, pickle
from src.Utils.gym_envs import UR5eGridEnvwDFA, UR5e_TQ_agent, env_info2UR5e_grid_env

# check statistical performance of a saved policy q-table

# stats_file = "stats_20260107_173049.pkl"
# stats_path = os.path.join(os.getcwd(), "src", "fail_rate_results", stats_file)
# with open(stats_path, "rb") as f:
#     stats_data = pickle.load(f)

# model_info_name = "q_table_20251/219_113456.pkl"# complete q-table for pick and place but no env/agent parameter info
# model_info_name = "q_table_20251219_143256.pkl" # incomplete q-table for pick and place with env/agent parameter info
# model_info_name = "q_table_20251220_205846.pkl" # complete q-table for pick and place with env/agent params
# model_info_name = "q_table_20251221_155900.pkl" # complete for fetch and return with one state
model_info_name = "q_table_20251222_143433.pkl" # complete for fetch and return with multiple start states
# model_info_name = "q_table_20251230_112103.pkl" # complete for flexible start, fixed finish with multiple middle states

# number of attempts the agent will make to complete the task
num_episodes = 20

# RL policy boolean
use_rl_policy = False  

# path of RL model and env data
path = os.path.join(os.getcwd(), "src", "RL_models", model_info_name)

with open(path, "rb") as f:
    policy_data = pickle.load(f)

# extract data from saved policy    
q_table = policy_data['q']
rewards = policy_data['episode_returns']
episode_lengths = policy_data['episode_lengths']
env_info   = policy_data['env_info']
TQL_params = policy_data['TQL_agent_params']

# initialize enviornment and RL agent
env = UR5eGridEnvwDFA(**env_info)
q_agent = UR5e_TQ_agent(UR5e_env=env, **TQL_params)

# number of failures for each episode
ep_failures_list = []

# rewwards and lengths for each episode
ep_rewards_list = []
ep_lengths_list = []

if use_rl_policy:
    # sanalyze selected saved policy
    for episode in range(num_episodes):
        print(f"----- Starting attempt {episode+1} -----")
        # start new episode
        obs, info = env.reset()
        done = False
        ep_ret = 0.0
        ep_len = 0
        ep_td_sum = 0.0
        num_failures = 0
        # episode loop
        while not done:
            # discretize observation
            obs_discrete = q_agent.discrete_obs(obs)
            
            # map obs to tuple for q-table indexing
            obs_key = tuple(np.asarray(obs_discrete, dtype=int))

            # if this observation is previously unseen, move to safe configuration
            if obs_key in q_table:
                # we have seen this observation before        
                # select action according to loaded q-table
                action = np.argmax(q_table[obs_key])
            else:
                # unseen observation, move to safe configuration
                print("Unseen observation encountered, moving to random safe configuration.")
                action = np.random.choice( list( env.action_map_safe.keys() ) )
            
            # step in environment
            print(f"Current action is {env.action_map[action]}")
            next_obs, reward, terminated, truncated, info = env.step(action)
            print(f"current reward is {reward}")
            
            # check if execution was successful 
            success_exec_bool = info.get("success_exec_bool")
            if not success_exec_bool:
                num_failures += 1
                print(f"Execution failure encountered. Total failures so far: {num_failures}")

            ep_ret += float(reward)
            ep_len += 1
            obs = next_obs
            
            done = bool(terminated or truncated)
            
            if done:
                print("Episode finished.")
                ep_failures_list.append(num_failures)
                ep_rewards_list.append(ep_ret)
                ep_lengths_list.append(ep_len)
                print(f"Total Reward: {ep_ret}, Episode Length: {ep_len}, Number of Failures: {num_failures}")
                break
else:
    # no model specified so we will analyze basic RRT-Connect instead 
    # for a trajectory of n states and m safe configs
    # action_map[0 to n-1] = trajectory states
    # action_map[n to n+m-1] = safe configs
    # task_name = "pick_place"
    task_name = "fetch_return"
    # task_name = "flex_start"
    for episode in range(num_episodes):
        print(f"----- Starting attempt {episode+1} -----")
        # start new episode
        obs, info = env.reset()
        done = False
        ep_ret = 0.0
        ep_len = 0
        ep_td_sum = 0.0
        num_failures = 0
        
        if task_name == "flex_start":
            middle_action = np.random.randint(0, 5)
            action_plan = [middle_action, 6]
            
        if task_name == "fetch_return":
            action_plan = [0, 1]
            
        if task_name == "pick_place":
            action_plan = [0, 1]
        

        for action in action_plan:
            success_exec_bool = False
            while not success_exec_bool:
                print(f"Planned Action: {env.action_map[int(action)]}")
                obs, reward, terminated, truncated, info = env.step(action)
                success_exec_bool = info["success_exec_bool"]
                print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")
                
                 # check if execution was successful 
                success_exec_bool = info.get("success_exec_bool")
                if not success_exec_bool:
                    num_failures += 1
                    print(f"Execution failure encountered. Total failures so far: {num_failures}")
                
                ep_ret += float(reward)
                ep_len += 1
                
                done = bool(terminated or truncated)
                
                if done:
                    print("Episode finished.")
                    ep_failures_list.append(num_failures)
                    ep_rewards_list.append(ep_ret)
                    ep_lengths_list.append(ep_len)
                    print(f"Total Reward: {ep_ret}, Episode Length: {ep_len}, Number of Failures: {num_failures}")
                    break
    

    
    
           
# statistical analysis of results
# reward results
mean_reward = np.mean(ep_rewards_list)
std_reward = np.std(ep_rewards_list)

# length results
mean_length = np.mean(ep_lengths_list)
std_length = np.std(ep_lengths_list)

# failure results
mean_failures = np.mean(ep_failures_list)
std_failures = np.std(ep_failures_list)

# confidence interval parameters with a T-distribution
confidence_level = 0.95
alpha = 1 - confidence_level
df = num_episodes - 1
t_crit = t.ppf(1 - alpha/2, df)

# compute confidence intervals
reward_margin_of_error = t_crit * (std_reward / np.sqrt(num_episodes))
length_margin_of_error = t_crit * (std_length / np.sqrt(num_episodes))
failures_margin_of_error = t_crit * (std_failures / np.sqrt(num_episodes))

# boolean to indicate we are saving results
save_results_bool = True
if save_results_bool:
    results_summary = {
        "mean_reward": mean_reward,
        "std_reward": std_reward,
        "reward_confidence_interval": (mean_reward - reward_margin_of_error, mean_reward + reward_margin_of_error),
        "mean_length": mean_length,
        "std_length": std_length,
        "length_confidence_interval": (mean_length - length_margin_of_error, mean_length + length_margin_of_error),
        "mean_failures": mean_failures,
        "std_failures": std_failures,
        "failures_confidence_interval": (mean_failures - failures_margin_of_error, mean_failures + failures_margin_of_error),
        "num_episodes": num_episodes,
        "ep_rewards_list": ep_rewards_list,
        "ep_lengths_list": ep_lengths_list,
        "ep_failures_list": ep_failures_list
    }
    
    # target folder
    out_dir = os.path.expanduser("~/ur5e_ws/src/fail_rate_results")
    os.makedirs(out_dir, exist_ok=True)

    # timestamp: YYYYMMDD_HHMMSS
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    if use_rl_policy:
        results_path = os.path.join(out_dir, f"stats_{ts}.pkl")
    else:
        results_path = os.path.join(out_dir, f"stats_rrt_{task_name}_{ts}.pkl")

    with open(results_path, "wb") as f:
        pickle.dump(results_summary, f)
    print(f"Results summary saved to {results_path}")


# summarize results
print("----- Summary of Results -----")
for i in range(num_episodes):
    print(f"Attempt {i+1}: Total Reward = {ep_rewards_list[i]}, Episode Length = {ep_lengths_list[i]}, Number of Failures = {ep_failures_list[i]}")