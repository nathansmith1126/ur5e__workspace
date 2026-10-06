from cProfile import label
import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import inspect 
import os, pickle 
import moveit_commander
import tf.transformations as T
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel, DeleteModel
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene, add_gripper2scene
from src.Utils.AUTOMATA.auto_funcs import DFAMonitor
from typing import Optional, Sequence, List, Tuple, Dict, Union
from collections import defaultdict
from datetime import datetime 

class UR5eGridEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, max_episode_steps: Optional[int] = 50, 
                 failed_trans_penalty: Optional[float] = 0.1, 
                 closer2goal_reward: Optional[float] = 0.01,
                 closest2goal_reward: Optional[float] = 0.1, 
                 completion_reward: Optional[float] = 1.0, 
                 efficiency_penalty: Optional[float] = 0.01):
        super().__init__()
        
        # Initialize moveit planning node
        moveit_commander.roscpp_initialize([])
        
        # initialize this script as a ROS node
        rospy.init_node("ur5e_grid_env", anonymous=True)

        # Initialize custom grid world for UR5e
        self.grid_world = grid_world()
        
        # Initialize MoveIt interfaces
        self.robot = moveit_commander.RobotCommander() # interface to the robot
        self.group_name = self.robot.get_group_names()[0] # first move group is the manipulator
        self.UR5e_move_group = moveit_commander.MoveGroupCommander(self.group_name) # interface to the ur5e planner
        self.scene = moveit_commander.PlanningSceneInterface() # interface to the world surrounding the robot
        
        # Clear any previous constraints
        self.UR5e_move_group.clear_path_constraints()
        
        # add work table to the scene
        add_table2scene(self.robot, self.scene)

        # Action space: 6 discrete moves
        self.action_space = spaces.Discrete(6)
        
        # map discrete action → xyz translation
        self.action_map = {
                            0: [1, 0, 0],   # +x
                            1: [-1, 0, 0],  # -x
                            2: [0, 1, 0],   # +y
                            3: [0, -1, 0],  # -y
                            4: [0, 0, 1],   # +z
                            5: [0, 0, -1],  # -z
                            }
      
        # used to determine observation space limits, 7 is overestimation of 2pi to account for joint angles 
        max_grid_index = max( (np.ceil( self.grid_world.arm_radius / self.grid_world.min_thickness ), 7.0 ) )
        
        # number of joint angles
        n_joints = len(self.UR5e_move_group.get_active_joints())
        
        # diemension of grid space
        grid_dim = 3
        
        # dimesnsion of end effector position
        eef_dim = 3
        
        # observation space dimension is sum of grid dim, eef dim and n_joints
        obs_space_dimension = grid_dim + eef_dim + n_joints
        
        # observation space dependent on grid world
        self.observation_space = spaces.Box(
            low=-max_grid_index, high=max_grid_index, shape=(obs_space_dimension,), dtype=np.float32
        )

        self.max_episode_steps = max_episode_steps
        
        # initialize env trackers
        self.current_step = 0
        self.current_grid_state = None
        
        # initialize rewards and penalties
        self.failed_trans_penalty = failed_trans_penalty # penalty for failed UR5e transition
        self.closer2goal_reward = closer2goal_reward # small reward for getting closer to goal
        self.completion_reward = completion_reward # reward for reaching goal
        self.efficiency_penalty = efficiency_penalty # small penalty for each step taken to promote efficiency
        self.closest2goal_reward = closest2goal_reward
        
        # initialize previous distance and grid state
        self.prev_distance2goal = None
        self.prev_grid_state = None
        self.min_distance = None
        self.distance2goal = None
        
    def grid_state2obs(self, state):
        return np.array(state, dtype=np.float32)
    
    def update_grid_state(self):
        '''
        Query current end-effector pose and convert to grid state from move_group
        '''
        
        # End-effector link name (defaults to tool0 or whatever you set)
        eef_link = self.UR5e_move_group.get_end_effector_link()

        # Query the current pose
        pose_stamped = self.UR5e_move_group.get_current_pose(eef_link)
        current_pose = pose_stamped.pose   # geometry_msgs/Pose

        # convert to grid state
        current_grid_state = self.grid_world.rect_pose_r2grid_state(current_pose)
        self.current_grid_state = current_grid_state
        
        # update distance to goal
        self.distance2goal = self.distance(self.current_grid_state, self.grid_world.goal_state)
        
    def move2start_state(self):
        '''
        Move UR5e to start state defined in grid world
        '''
        
        # initialize success flag
        self.reset_success_bool = False
        
        start_pose = self.grid_world.grid_state2rect_pose_r(self.grid_world.start_state)

        # set target pose in move group
        self.UR5e_move_group.set_pose_target(start_pose, end_effector_link="tool0")
        
        # look for plan
        success_bool, trajectory, _, _ = self.UR5e_move_group.plan()
        
        # execute plan if found
        if success_bool and len(trajectory.joint_trajectory.points) > 0:
            # execute plan
            self.UR5e_move_group.execute(trajectory, wait=True)
            
            # update current grid state
            self.update_grid_state()
            
            # check if execution was success
            if self.current_grid_state == self.grid_world.start_state:
                self.reset_success_bool = True
                print("Moved to start state successfully using one-shot approach")
            else:
                print("Still not at start state, retrying...")
                
        else:
            print("Failed to move to start state using oneshot RRT, switching methods") 
            # keep trying to move to start state using step-by-step approach
            while not self.reset_success_bool:
                self.move2start_state_slow()
                if self.current_grid_state == self.grid_world.start_state:
                    self.reset_success_bool = True
                    print("Moved to start state successfully using step-by-step approach")
                else:
                    print("Still not at start state, retrying...")
                
        self.UR5e_move_group.clear_pose_targets()
        
    def move2start_state_slow(self):
        '''
        Method to move the UR5e to the start state defined in the grid world
        using a slow, step-by-step approach to ensure feasibility of each step
        '''
                
        # update grid state
        self.update_grid_state()
        
        # get start state from grid world
        start_state = self.grid_world.start_state
        
        # calculate difference between current and start state
        delta_state_array = np.array(start_state) - np.array(self.current_grid_state)
        
        # iterate through each dimension (x,y,z)
        dir_index_array = np.arange(3)
        
        # randomize order of directions to take steps in to avoid bias
        np.random.shuffle(dir_index_array) 
        
        for dir_index in dir_index_array: # iterate through x,y,z
            
            # clear prior targets
            self.UR5e_move_group.clear_pose_targets()
            
            # reset flag flips if a transition fails and forces us to restart motion planning
            reset_slow_move = False
            
            # get number of steps to take in this dimension
            num_steps = abs(delta_state_array[dir_index])
            
            # determine step direction (+1 or -1)
            step_direction = int(np.sign(delta_state_array[dir_index])) # +1 or -1
            
            for step in range(num_steps):
                # create action to take one step in this dimension
                action = [0, 0, 0]
                action[dir_index] = step_direction
                
                # get next grid state
                next_grid_state = self.grid_world.grid_iso_step(action=action, current_grid_state=self.current_grid_state)
                
                # get corresponding pose in ur5e frame
                target_pose = self.grid_world.grid_state2rect_pose_r(grid_state=next_grid_state)
                
                # plan to target pose
                self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
                
                # look for plan
                success_bool, trajectory, _, _ = self.UR5e_move_group.plan()

                # execute plan if found
                if success_bool and len(trajectory.joint_trajectory.points) > 0:
                    # execute plan
                    self.UR5e_move_group.execute(trajectory, wait=True)
                    
                    # update current grid state
                    self.update_grid_state()
                else:
                    # Retrying slow reset
                    print("Failed to transition")
                    reset_slow_move = True 
                    break
            
            if reset_slow_move:
                # break if we failed a transition and need to replan from current state
                break        

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        
        # move to start state
        self.move2start_state()
        
        # update min distance to goal as initial distance since we just started
        self.min_distance = self.distance2goal.copy()
        
        # get initial observation
        obs = self.grid_state2obs(self.current_grid_state)
        return obs, {}
    
    def distance(self, state1, state2)-> float:
        '''
        Compute taxi-cab distance between two grid states
        
        Args:
            state1 (list): first grid state
            state2 (list): second grid state    
            
        Returns:
            distance (float): taxi-cab distance between two grid states
        '''
        tc_distance_array = np.abs(np.array(state1) - np.array(state2))
        distance = np.sum(tc_distance_array)
        return distance

    def step(self, action):
        # update previous distance to goal
        self.prev_grid_state = self.current_grid_state.copy()

        # update previous distance to goal
        self.prev_distance2goal = self.distance(self.current_grid_state, self.grid_world.goal_state)

        # increment step count
        self.current_step += 1

        # initialize reward with small penalty for each step to promote efficiency
        reward = -self.efficiency_penalty 
        
        # map action to xyz translation
        action_list = self.action_map[int(action)]

        # compute target grid state
        target_grid_state = self.grid_world.grid_iso_step(action_list, self.current_grid_state)
        target_pose = self.grid_world.grid_state2rect_pose_r(target_grid_state)

        # plan to target pose
        self.UR5e_move_group.set_start_state_to_current_state()
        self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
        success, plan, _, _ = self.UR5e_move_group.plan()

        # initialize reward and termination flags
        terminated = False
        truncated = False

        # execute if plan is successful
        if success and len(plan.joint_trajectory.points) > 0:
            # execute plan
            self.UR5e_move_group.execute(plan, wait=True)

            # update current grid stateF
            self.update_grid_state() 
            
            # investigate if gloser to goal, further from goal or completed
            if self.current_grid_state == self.grid_world.goal_state:
                
                # add completion reward
                reward += self.completion_reward
                terminated = True
                print("Reached Goal")
            else:
                # check if we got closer
                if self.prev_distance2goal < self.distance2goal:
                    # got closer 
                    print("Got closer")
                    reward += self.closer2goal_reward
                    # check if this is the closest we have been to goal
                    if self.distance2goal < self.min_distance:                    
                        # add reward for getting closest to goal thus far
                        reward += self.closest2goal_reward
                        print("closest we have ever been!")
                        
                        # update min distance 
                        self.min_distance = self.distance2goal.copy()
                else:
                    # successful move but did not get closer
                    pass
        else:
            # subtract penalty for failure to execute
            reward -= self.failed_trans_penalty

        if self.current_step >= self.max_episode_steps:
            truncated = True
            print("Truncation - too many steps")
            
        print(f"current grid state after step is {self.current_grid_state}")

        obs = self.grid_state2obs(self.current_grid_state)
        info = {}
        return obs, reward, terminated, truncated, info

    def close(self):
        # shutdown moveit commander
        self.UR5e_move_group.clear_pose_targets()
        self.UR5e_move_group.clear_path_constraints()
        moveit_commander.roscpp_shutdown()
        
class UR5eGridEnvwDFA(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, 
                 start_states: Union[List[List[int]], List[int]], 
                 goal_state: List[int], 
                 grid_size_array: List[float],
                 DFA_monitor: DFAMonitor, 
                 DFA_alphabet_dict: Dict[int, str] = None,
                 max_episode_steps: Optional[int] = 50, 
                 failed_trans_penalty: Optional[float] = 0.1, 
                 closer2goal_reward: Optional[float] = 0.01,
                 closest2goal_reward: Optional[float] = 0.1, 
                 completion_reward: Optional[float] = 1.0, 
                 auto_reward_scaler: Optional[float] = 4.0,
                 efficiency_penalty: Optional[float] = 0.01, 
                 pos_tolerance: Optional[float] = 0.05,
                 plan_time: Optional[float] = 3.0,
                 gazebo_bool: Optional[bool] = True, 
                 safe_configs: Optional[List[List[float]]] = None, 
                 fetch_return_bool: Optional[bool] = False):
        super().__init__()
        
        # Initialize moveit planning node
        moveit_commander.roscpp_initialize([])
        
        # initialize this script as a ROS node
        rospy.init_node("ur5e_grid_env", anonymous=True)
        
        # bool to indicate if we are running in gazebo
        self.gazebo_bool = gazebo_bool
        
        # check if we are working with the tech and return objective for a range of initial states
        self.fetch_return_bool = fetch_return_bool 
        
        # bool to indicate if we are using a series of radial safe states instead of a grid
        self.safe_configs = safe_configs
        
        # Initialize custom grid world parameters
        self.start_states = start_states
        self.goal_state = goal_state
        self.grid_size_array = grid_size_array
        
        # initialize DFA monitor for temporal rewards 
        self.DFA_monitor = DFA_monitor
        
        # optional alphabet dictionary to map grid labels to DFA letters (IF NECESSARY)
        self.DFA_alphabet_dict = DFA_alphabet_dict
        
        # Generate grid world
        self.generate_grid_world()
        
        # Initialize MoveIt interfaces
        self.robot = moveit_commander.RobotCommander() # interface to the robot
        self.group_name = self.robot.get_group_names()[0] # first move group is the manipulator
        self.UR5e_move_group = moveit_commander.MoveGroupCommander(self.group_name) # interface to the ur5e planner
        self.scene = moveit_commander.PlanningSceneInterface() # interface to the world surrounding the robot
        
        # Clear any previous constraints
        self.UR5e_move_group.clear_path_constraints()
        
        # add work table to the scene
        add_table2scene(self.robot, self.scene)
        
        # add gripper to end effector
        add_gripper2scene(move_group=self.UR5e_move_group, 
                          scene=self.scene)
        
        # add position tolerances for moveit
        self.pos_tolerance = pos_tolerance
        self.UR5e_move_group.set_goal_position_tolerance(pos_tolerance) # 5mm
        
        # add time allowance for planning
        self.plan_time = plan_time
        self.UR5e_move_group.set_planning_time(plan_time) #seconds

        # initialize info dictionary that is returned at each bym env step
        # It will house planning time and error codes from MoveIt
        
        self.info_dict = {"plan_time": None, 
                          "error_code": None,
                          "success_plan_bool": None, 
                          "success_exec_bool": None}
        
        # initialize list of waypoint names for gazebo visualization
        self.waypoint_names = []
      
        # used to determine observation space limits, 7 is overestimation of 2pi to account for joint angles 
        max_grid_index = max( (np.ceil( 2*self.grid_world.arm_radius / self.grid_world.min_thickness ), 7.0 ) )
        
        # number of joint angles
        n_joints = len(self.UR5e_move_group.get_active_joints())
        
        # diemension of grid space
        grid_dim = 3
        
        # dimesnsion of end effector position
        # eef_dim = 3
        
        # number of DFA states
        DFA_states = self.DFA_monitor.num_states
        
        # observation space dimension is sum of grid dim, eef dim, DFA state size and n_joints
        obs_space_dimension = grid_dim + DFA_states + n_joints
        
        # observation space dependent on grid world
        self.observation_space = spaces.Box(
            low=-max_grid_index, high=max_grid_index, shape=(obs_space_dimension,), dtype=np.float32
        )

        # Generate action map and initialize action maps
        self.action_map_safe = {}
        self.action_map_traj = {}
        self.generate_action_map()

        self.max_episode_steps = max_episode_steps
        
        # initialize env trackers
        self.current_step = 0
        self.current_grid_state = None
        self.current_obs = None
        
        # initialize rewards and penalties
        self.failed_trans_penalty = failed_trans_penalty # penalty for failed UR5e transition
        self.closer2goal_reward   = closer2goal_reward # small reward for getting closer to goal
        self.completion_reward    = completion_reward # reward for reaching goal
        self.efficiency_penalty   = efficiency_penalty # small penalty for each step taken to promote efficiency
        self.closest2goal_reward  = closest2goal_reward
        self.auto_reward_scaler   = auto_reward_scaler # scale DFA potential difference reward
        
        # initialize previous distance and grid state
        self.prev_distance2goal = None
        self.prev_grid_state = None
        self.min_distance = None
        self.distance2goal = None
        
    def get_env_info(self):
        '''
        Method to return current dictionary containing environment initialization information
        associated with goal states, rewards, dfa etc
        Returns:
            env_info_dict (dict): dictionary containing environment information
        '''
        env_info_dict = {
            "start_states": self.start_states,
            "goal_state": self.goal_state,
            "grid_size_array": self.grid_size_array,
            "DFA_monitor": self.DFA_monitor,
            "DFA_alphabet_dict": self.DFA_alphabet_dict,
            "max_episode_steps": self.max_episode_steps,
            "safe_configs": self.safe_configs,
            "max_episode_steps": self.max_episode_steps,
            "failed_trans_penalty": self.failed_trans_penalty,
            "closer2goal_reward": self.closer2goal_reward,
            "closest2goal_reward": self.closest2goal_reward,
            "completion_reward": self.completion_reward,
            "auto_reward_scaler": self.auto_reward_scaler,
            "efficiency_penalty": self.efficiency_penalty,  
            "pos_tolerance": self.pos_tolerance,
            "plan_time": self.plan_time, 
            "fetch_return_bool": self.fetch_return_bool
        }
        
            # def __init__(self, 
            #      start_states: Union[List[List[int]], List[int]], 
            #      goal_state: List[int], 
            #      grid_size_array: List[float],
            #      DFA_monitor: DFAMonitor, 
            #      DFA_alphabet_dict: Dict[int, str] = None,
            #      max_episode_steps: Optional[int] = 50, 
            #      failed_trans_penalty: Optional[float] = 0.1, 
            #      closer2goal_reward: Optional[float] = 0.01,
            #      closest2goal_reward: Optional[float] = 0.1, 
            #      completion_reward: Optional[float] = 1.0, 
            #      auto_reward_scaler: Optional[float] = 4.0,
            #      efficiency_penalty: Optional[float] = 0.01, 
            #      pos_tolerance: Optional[float] = 0.05,
            #      plan_time: Optional[float] = 3.0,
            #      gazebo_bool: Optional[bool] = True, 
            #      safe_configs: Optional[List[List[float]]] = None):
            
        return env_info_dict 
        
    def generate_action_map(self):
        '''
        Generate action map based on type of DFA provided
        '''    
        
        if self.DFA_alphabet_dict:
            if self.safe_configs is not None:
                # only configs of interest are those in the trajectory + safe configs
                self.num_actions = len(self.DFA_alphabet_dict) + len(self.safe_configs)
                
                # 1 integer action per trajectory config + safe config
                self.action_space = spaces.Discrete(self.num_actions)
                
                # map discrete action → desired trajectory configs
                self.action_map_traj = {i: config for i, config in enumerate(self.DFA_alphabet_dict.values())}
                
                # map discrete action → safe configs
                self.action_map_safe = {i + len(self.DFA_alphabet_dict): config for i, config in enumerate(self.safe_configs)}
                
                # combine all action mappings
                self.action_map = {**self.action_map_traj, **self.action_map_safe}
            else:
                # configs of interest are grid world, safe config and trajectory configs
                # indicates we are dealing with a DFA that involves moving through a fixed set of configurations
                # Action space: 6 discrete moves + num_configs DFA letters + movement to "safe" configuration
                self.num_actions = len(self.DFA_alphabet_dict) + 6 + 1
                self.action_space = spaces.Discrete(self.num_actions)

                # initialize action map with 6 movement directions in cartesian space
                action_cart_map = {
                        0: [1, 0, 0],   # +x
                        1: [-1, 0, 0],  # -x
                        2: [0, 1, 0],   # +y
                        3: [0, -1, 0],  # -y
                        4: [0, 0, 1],   # +z
                        5: [0, 0, -1],  # -z
                        }
                
                
                num_configs = len(self.DFA_alphabet_dict)
                
                # map discrete action → desired trajectory configs 
                # eg we can try to reach any state in the trajectory from arbitrary grid state
                traj_dict = {int( i + 6 ): self.DFA_alphabet_dict[f"reach_{i}"] for i in range(num_configs) }
                safe_traj_dict = {int(self.num_actions-1): "safe_config" }
                
                # combine all action mappings
                self.action_map = {**action_cart_map, **traj_dict, **safe_traj_dict}
        else:
            # labeling dictionary to map grid labels to DFA letters
            '''
            NOTE:
            Technically, the automaton alphabet is equal to the label of indicators, but 
            we use the current grid state label and prior grid state label to reduce 
            the automaton alphabet from 8 letters to 4 letters for easier implementation. 
            '''
            self.labeling_dict = {0:"delta_x", 1:"delta_y", 2:"delta_z"}
            
            # Action space: 6 discrete moves
            self.action_space = spaces.Discrete(6)
            
            # map discrete action → xyz translation
            self.action_map = {
                                0: [1, 0, 0],   # +x
                                1: [-1, 0, 0],  # -x
                                2: [0, 1, 0],   # +y
                                3: [0, -1, 0],  # -y
                                4: [0, 0, 1],   # +z
                                5: [0, 0, -1],  # -z
                                }
        
    def generate_grid_world(self):
        '''
        Generate grid world based on current parameters. 
        If start_state contains multiple sttates (list of lists), randomly select one.
        If start_state is a single state (list), use that directly.
        '''
        if isinstance(self.start_states[0], list):
            # multiple start states, randomly select one
            rand_index = np.random.randint(0, len(self.start_states))
            selected_start_state = self.start_states[rand_index]
        else: 
            # single start state
            selected_start_state = self.start_states    
            
        # add selected start state to DFA_alphabet_dict if applicable
        if self.fetch_return_bool and self.DFA_alphabet_dict is not None:
            self.DFA_alphabet_dict["object_returned"] = selected_start_state
        
        # initialize grid_world
        self.grid_world = grid_world(start_state=selected_start_state, 
                                     goal_state=self.goal_state, 
                                     grid_size_array=self.grid_size_array)
    
    def joint_angle_plan(self, joint_angles: List[float]) -> Tuple:
        '''
        Method to move the UR5e to the specified joint angles
        
        Args:
            joint_angles (list): list of joint angles to move to
        Returns:
            bool: True if the movement was successful, False otherwise
        '''
                
        # set target joint angles in move group
        self.UR5e_move_group.set_joint_value_target(joint_angles)
        
        # look for plan
        success_bool, trajectory, plan_time, error_code = self.UR5e_move_group.plan()
        return success_bool, trajectory, plan_time, error_code

    def joint_angle_move(self, joint_angles: List[float]) -> Tuple:
        '''
        Method to move the UR5e to the specified joint angles
        
        Args:
            joint_angles (list): list of joint angles to move to
        Returns:
            bool: True if the movement was successful, False otherwise
        '''
                
        # set target joint angles in move group
        self.UR5e_move_group.set_joint_value_target(joint_angles)
        
        # look for plan
        success_bool, trajectory, plan_time, error_code = self.UR5e_move_group.plan()

        # execute plan if found
        if success_bool and len(trajectory.joint_trajectory.points) > 0:
            # execute plan
            self.UR5e_move_group.execute(trajectory, wait=True)
            print("Moved to specified joint angles")
            self.update_obs()
        else:
            print("Failed to move to specified joint angles") 
                
        self.UR5e_move_group.clear_pose_targets()

        return success_bool, trajectory, plan_time, error_code

    def grid_state2obs(self, state):
        return np.array(state, dtype=np.float32)
    
    def update_grid_state(self):
        '''
        Query current end-effector pose and convert to grid state from move_group
        '''
        
        # End-effector link name (defaults to tool0 or whatever you set)
        eef_link = self.UR5e_move_group.get_end_effector_link()

        # Query the current pose
        pose_stamped = self.UR5e_move_group.get_current_pose(eef_link)
        current_pose = pose_stamped.pose   # geometry_msgs/Pose

        # convert to grid state
        current_grid_state = self.grid_world.rect_pose_r2grid_state(current_pose)
        self.current_grid_state = current_grid_state
        print(f"Current grid state updated to: {self.current_grid_state}")
        
        # update distance to goal
        self.distance2goal = self.distance(self.current_grid_state, self.grid_world.goal_state)
    
    def update_obs(self):
        '''
        Update observation based on current grid state, end-effector pose and joint angles
        '''
        
        # update grid state
        self.update_grid_state()
        
        # get end-effector pose
        eef_link = self.UR5e_move_group.get_end_effector_link()
        pose_stamped = self.UR5e_move_group.get_current_pose(eef_link)
        current_pose = pose_stamped.pose   # geometry_msgs/Pose
        
        # get joint angles
        joint_angles = self.UR5e_move_group.get_current_joint_values()

        # get DFA state
        dfa_state_array = self.DFA_monitor.state_label2array()

        # join grid state, eef pose, joint angles into single observation array
        obs_array_temp = np.array(self.current_grid_state + 
                             joint_angles, dtype=np.float32)
        
        # add DFA state to array
        obs_array = np.concatenate( (obs_array_temp, dfa_state_array), axis=0 )
        
        self.current_obs = obs_array
        return obs_array
    
    def move2start_state(self):
        '''
        Move UR5e to start state defined in grid world
        '''
        
        # initialize success flag
        self.reset_success_bool = False
        
        start_pose = self.grid_world.grid_state2rect_pose_r(self.grid_world.start_state)

        while not self.reset_success_bool:
            # repeat loop until we reach start state
            print("Attempting to move to start state...")
            self.grid_move(self.grid_world.start_state)
            
            if self.current_grid_state == self.grid_world.start_state:
                self.reset_success_bool = True
                print("Moved to start state successfully")
            else:
                print("Still not at start state, retrying...")  
                # tray again after moving to a random safe config if available
                if self.safe_configs is not None:
                    rand_index = np.random.randint(0, len(self.safe_configs))
                    safe_config = self.safe_configs[rand_index]
                    print(f"Moving to random safe config: {safe_config} before retrying...")
                    self.joint_angle_move(safe_config)
            
            
            
        # # set target pose in move group
        # self.UR5e_move_group.set_pose_target(start_pose, end_effector_link="tool0")
        
        # # look for plan
        # success_bool, trajectory, _, _ = self.UR5e_move_group.plan()
        
        # # execute plan if found
        # if success_bool and len(trajectory.joint_trajectory.points) > 0:
        #     # execute plan
        #     self.UR5e_move_group.execute(trajectory, wait=True)
            
        #     # update current grid state
        #     self.update_grid_state()
            
        #     # check if execution was success
        #     if self.current_grid_state == self.grid_world.start_state:
        #         self.reset_success_bool = True
        #         print("Moved to start state successfully using one-shot approach")
        #     else:
        #         print("Still not at start state, retrying...")
                
        # else:
            
        #     if self.safe_configs is None:
        #         print("Failed to move to start state using oneshot RRT, switching methods") 
        #         # keep trying to move to start state using step-by-step approach
        #         while not self.reset_success_bool:
        #             self.move2start_state_slow()
        #             if self.current_grid_state == self.grid_world.start_state:
        #                 self.reset_success_bool = True
        #                 print("Moved to start state successfully using step-by-step approach")
        #             else:
        #                 print("Still not at start state, retrying...")
        #     else:
        #         # 
        #         pass

        self.UR5e_move_group.clear_pose_targets()
        
    def move2start_state_slow(self):
        '''
        Method to move the UR5e to the start state defined in the grid world
        using a slow, step-by-step approach to ensure feasibility of each step
        '''
                
        # update grid state
        self.update_grid_state()
        
        # get start state from grid world
        start_state = self.grid_world.start_state
        
        # calculate difference between current and start state
        delta_state_array = np.array(start_state) - np.array(self.current_grid_state)
        
        # iterate through each dimension (x,y,z)
        dir_index_array = np.arange(3)
        
        # randomize order of directions to take steps in to avoid bias
        np.random.shuffle(dir_index_array) 
        
        for dir_index in dir_index_array: # iterate through x,y,z
            
            # clear prior targets
            self.UR5e_move_group.clear_pose_targets()
            
            # reset flag flips if a transition fails and forces us to restart motion planning
            reset_slow_move = False
            
            # get number of steps to take in this dimension
            num_steps = abs(delta_state_array[dir_index])
            
            # determine step direction (+1 or -1)
            step_direction = int(np.sign(delta_state_array[dir_index])) # +1 or -1
            
            for step in range(num_steps):
                # create action to take one step in this dimension
                action = [0, 0, 0]
                action[dir_index] = step_direction
                
                # get next grid state
                next_grid_state = self.grid_world.grid_iso_step(action=action, current_grid_state=self.current_grid_state)
                
                # get corresponding pose in ur5e frame
                target_pose = self.grid_world.grid_state2rect_pose_r(grid_state=next_grid_state)
                
                # plan to target pose
                self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
                
                # look for plan
                success_bool, trajectory, _, _ = self.UR5e_move_group.plan()

                # execute plan if found
                if success_bool and len(trajectory.joint_trajectory.points) > 0:
                    # execute plan
                    self.UR5e_move_group.execute(trajectory, wait=True)
                    
                    # update current grid state
                    self.update_grid_state()
                else:
                    # Retrying slow reset
                    print("Failed to transition")
                    reset_slow_move = True 
                    break
            
            if reset_slow_move:
                # break if we failed a transition and need to replan from current state
                break        

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.current_step = 0
        
        # generate grid world
        self.generate_grid_world()
        
        # generate action map if fetch and return task since 
        # action map depends on selected start state
        if self.fetch_return_bool:
            self.generate_action_map()
        
        # move to start state
        self.move2start_state()
        
        # update min distance to goal as initial distance since we just started
        self.min_distance = self.distance2goal.copy()
        
        # reset DFA monitor
        self.DFA_monitor.reset()
        
        # get initial observation
        self.update_obs()
        return self.current_obs, {}
    
    def distance(self, state1, state2)-> float:
        '''
        Compute taxi-cab distance between two grid states
        
        Args:
            state1 (list): first grid state
            state2 (list): second grid state    
            
        Returns:
            distance (float): taxi-cab distance between two grid states
        '''
        tc_distance_array = np.abs(np.array(state1) - np.array(state2))
        distance = np.sum(tc_distance_array)
        return distance

    def grid_state2label(self, grid_state: Optional[Sequence[int]] = None) -> str:
        '''
        Convert grid state to DFA label
        
        
        Args:
            state (list): grid state
        Returns:
            label_list (list): list of binary indicators for each coordinate
                                [indicator_x, indicator_y, indicator_z] where indicator_i = 1 
                                if at goal coordinate in dimension i, else 0
        '''
        label_list = []
        if grid_state is None:
            grid_state = self.current_grid_state

        for index in np.arange(3):
            coordinate = grid_state[index]
            find_goal_coord = self.grid_world.goal_state[index]
            if coordinate == find_goal_coord:
                # at goal coordinate
                label_list.append(1)
            else:
                # not at goal coordinate
                label_list.append(0)
                
        return label_list

    def label2auto_letter(self, prev_label_list: List[int], 
                                current_label_list: List[int]) -> Tuple[str, bool]:
        '''
        Convert label list to automaton letter
        
        Args:
            label_list (list): list of binary indicators for each coordinate
        Returns:
            letter (str): automaton letter corresponding to label list
            impossible_trans_bool (bool): flag for impossible transition indicates termination
        '''
        if len(prev_label_list) != 3 or len(current_label_list) != 3:
            raise ValueError("Label lists must be of length 3 for x,y,z indicators")
        
        diff_ind = [index for index in range(3) if prev_label_list[index] != current_label_list[index]]
        
        if len(diff_ind) > 1:
            # raise ValueError("Only one coordinate can change per step in grid world")
            print("Warning: More than one coordinate changed in label, terminating environment")
            letter = 'invalid'
            
        if len(diff_ind) == 0:
            # no change in label
            letter = "no_delta"
        else:
            # get corresponding letter from labeling dict for a change in one coordinate
            letter = self.labeling_dict[diff_ind[0]]
        return letter

    def get_auto_letter(self) -> str:
        '''
        Get automaton letter based on change in grid state
    
        Return:
            letter (str): automaton letter corresponding to change in grid state
        NOTE:
        Technically, the automaton alphabet is equal to the label of indicators, but 
        we use the current grid state label and prior grid state label to reduce 
        the automaton alphabet from 8 letters to 4 letters for easier implementation. 
        
        '''
        
        # intialize letter to None
        letter = None
        
        # check if we have a DFA alphabet dict
        if self.DFA_alphabet_dict:
            # if we have a DFA alphabet dict, we are dealing with specific configurations
            # so we can directly map the current grid state to the corresponding letter
            for letter_iter, grid_state in self.DFA_alphabet_dict.items():
                if grid_state == self.current_grid_state:
                    letter = letter_iter
                    break
        else:
            # if no DFA alphabet dict is present, fall back to label-based mapping
            # get previous and current labels
            prev_label_list = self.grid_state2label(self.prev_grid_state)
            current_label_list = self.grid_state2label(self.current_grid_state)
            
            # get automaton letter from labels
            letter = self.label2auto_letter(prev_label_list, current_label_list)

        return letter

    def UR5e_step(self, action) -> Tuple:
        '''
        Take a step in the UR5e grid world environment dictated by action
        
        Args:
            action (int): discrete action to take eg move down, mode to a goal config, 
                            or move to a safe config
        Returns:
            success (bool): True if the movement was successful, False otherwise
            plan (RobotTrajectory): planned trajectory
            plan_time (float): time taken to plan
            error_code (MoveItErrorCodes): error code from MoveIt planning
        '''
        
        # interpret if action is a movement direction or a specific configuration
        
        if self.safe_configs:
            # no frid world movement, only safe config or specific configurations
            target_state = self.action_map[action] # could be joint angles or grid state
            
            # determine if action is joint angles or grid state
            if len(target_state) == len(self.UR5e_move_group.get_active_joints()):
                # action is joint angles
                joint_angles = target_state
                
                # plan and execute joint angle move
                success, plan, plan_time, error_code = self.joint_angle_plan(joint_angles)
            else:
                # action is grid state
                grid_state = target_state
                
                success, plan, plan_time, error_code = self.grid_plan(grid_state)
        else:
            # grid world movement + specific configurations + safe config
            if self.action_map[action] == "safe_config":
                # move to safe configuration
                joint_angles = self.grid_world.safe_joint_angles
            
                # plan and execute joint angle move
                success, plan, plan_time, error_code = self.joint_angle_plan(joint_angles)
            else:
                # We are moving to a grid state either through cartesian movement or specific configuration
                if action in range(6):
                    # action is a movement direction in cartesian space 0<=action<=5 for cartesian movement
                    action_list = self.action_map[int(action)]

                    # compute target grid state
                    target_grid_state = self.grid_world.grid_iso_step(action_list, self.current_grid_state)
                
                else:
                    # action is a specific configuration to move to
                    target_grid_state = self.action_map[int(action)]
                

                success, plan, plan_time, error_code = self.grid_plan(target_grid_state)

        return success, plan, plan_time, error_code
    
    def step(self, action):
        # update previous distance to goal
        self.prev_grid_state = self.current_grid_state.copy()

        # update previous distance to goal
        self.prev_distance2goal = self.distance(self.current_grid_state, self.grid_world.goal_state)

        # increment step count
        self.current_step += 1

        # initialize reward with small penalty for each step to promote efficiency
        reward = -self.efficiency_penalty 
        
        # get plan based on action
        success, plan, plan_time, error_code = self.UR5e_step(action)

        # update info dictionary
        self.info_dict["plan_time"] = plan_time
        self.info_dict["error_code"] = error_code
        self.info_dict["success_plan_bool"] = success
        
        # initialize reward and termination flags
        terminated = False
        truncated = False

        # execute if plan is successful
        if success and len(plan.joint_trajectory.points) > 0:
            # execute plan
            success_exec_bool = self.UR5e_move_group.execute(plan, wait=True)
            
            # update information dictionary with progress
            self.info_dict["success_exec_bool"] = success_exec_bool
            
            # update current grid state and observation
            self.update_obs()
            print(f"current grid state after step is {self.current_grid_state}")
            
            # get label for DFA from current and previous states 
            auto_letter = self.get_auto_letter()
            print(f"Automaton letter for this step: {auto_letter}")
            
            # update DFA monitor state
            self.DFA_monitor.step(auto_letter)
            
            # analyze DFAnfinality to determine reward/penalty
            if self.DFA_monitor.current_state == 'sink':
                # check if we entered sink state or 
                # impossible transition
                print("Terminated because we Entered Sink State of DFA or made impossible transition")
                terminated = True
            else:
                # DID NOT ENTER SINK STATE
                # investigate if moved closer through automaton or completed goal
                if self.DFA_monitor.current_state in self.DFA_monitor.dfa.final_states:
                    # add completion reward
                    reward += self.completion_reward
                    terminated = True
                    print("Reached Goal DFA state")
            
            # add potential difference reward from DFA monitor multiplied by scaler
            # to reward forward progress and punish backward progress
            # if a letter is passed into DFA monitor
            # DFA monitor
            if auto_letter:
                reward += self.auto_reward_scaler * self.DFA_monitor.delta_potential
                
            # check if chosen action produced intended motion
            if not success_exec_bool:
                # failed during path execution and ended in unexpected state
                reward -= self.failed_trans_penalty
        else:
            # subtract penalty for failure to plan
            reward -= self.failed_trans_penalty
            
            # failure to plan implies failure to execute
            self.info_dict["success_exec_bool"] = False

        # check for truncation
        if self.current_step >= self.max_episode_steps:
            truncated = True
            print("Truncation - too many steps")            

        obs = self.current_obs
        info = self.info_dict
        return obs, reward, terminated, truncated, info

    def grid_plan(self, grid_state: List[int] ):
        '''
        Plan path to move to specified grid state directly
        
        Args:
            grid_state (list): target grid state to move to
        '''
        
        target_pose = self.grid_world.grid_state2rect_pose_r(grid_state)

        # set target pose in move group
        self.UR5e_move_group.set_start_state_to_current_state()
        self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
        
        # look for plan
        success_bool, trajectory, plan_time, error_code = self.UR5e_move_group.plan()
        
        return success_bool, trajectory, plan_time, error_code

    def grid_move(self, grid_state: List[int], plot_bool: Optional[bool]=False):
        '''
        Move UR5e to specified grid state directly
        
        Args:
            grid_state (list): target grid state to move to
            plot_bool (bool): boolean indicating whether or not to plot the trajectory
        '''
        
        target_pose = self.grid_world.grid_state2rect_pose_r(grid_state)

        # set target pose in move group
        self.UR5e_move_group.set_start_state_to_current_state()
        self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
        
        # look for plan
        success_bool, trajectory, plan_time, error_code = self.UR5e_move_group.plan()
        
        # execute plan if found
        if success_bool and len(trajectory.joint_trajectory.points) > 0:
            # execute plan
            self.UR5e_move_group.execute(trajectory, wait=True)
            
            # update current grid state
            self.update_grid_state()
            
            # check if execution was success
            if self.current_grid_state == grid_state:
                print(f"Moved to grid state {grid_state} successfully")
            else:
                print(f"Still not at grid state {grid_state}, current state is {self.current_grid_state}")
                print(f"Planned for {plan_time} seconds with error code {error_code}")
                
            # plot if desired
            if plot_bool:
                self.grid_world.plot_plan_trajectories(plan=trajectory)
        else:
            print(f"Failed to plan for grid state {grid_state} using RRT connect") 
            print(f"Planned for {plan_time} seconds with error code {error_code}")    
        self.UR5e_move_group.clear_pose_targets()

        return success_bool, trajectory, plan_time, error_code 

    def spawn_sphere(self, name, radius_m, x, y, z, roll=0, pitch=0, yaw=0,
                 static=True, collide=False, reference_frame="world", 
                 color_rgb=(1,0,0), emissive_scale=0.25):
        '''
        Spawn a sphere in the Gazebo simulation environment.
        Args:
            name (str): Name of the sphere model.
            radius_m (float): Radius of the sphere in meters.
            x (float): X position of the sphere center.
            y (float): Y position of the sphere center.
            z (float): Z position of the sphere center.
            roll (float): Roll orientation in radians.
            pitch (float): Pitch orientation in radians.
            yaw (float): Yaw orientation in radians.
            static (bool): Whether the sphere is static or dynamic.
            collide (bool): Whether the sphere has collision properties.
            reference_frame (str): Reference frame for the sphere's initial pose.
            color_rgb (tuple): RGB color values for the sphere.
            emissive_scale (float): Emissive scale for the sphere's material.
        Returns:
            SpawnModelResponse: Response from the spawn service.
        '''
        
        SDF_TPL = """<?xml version="1.0"?>
                        <sdf version="1.6">
                        <model name="{name}">
                            <static>{static}</static>
                            <link name="link">
                            {collision_block}
                            <visual name="vis">
                                <geometry><sphere><radius>{radius}</radius></sphere></geometry>
                                <material>
                                <ambient>1 0 0 1</ambient>
                                <diffuse>1 0 0 1</diffuse>
                                <emissive>0.25 0 0 1</emissive>
                                </material>
                            </visual>
                            </link>
                        </model>
                        </sdf>"""
                                
       # clamp helper function to ensure color values are in [0,1]
        def clamp01(v): return max(0.0, min(1.0, float(v)))
        r, g, b = (clamp01(c) for c in color_rgb)
        a = 1.0
        er, eg, eb, ea = (clamp01(emissive_scale * r),
                        clamp01(emissive_scale * g),
                        clamp01(emissive_scale * b), 1.0)

        # define material block with specified colors for sphere
        MATERIAL_BLOCK = f"""
        <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>{er} {eg} {eb} {ea}</emissive>
        </material>"""

        # define SDF template for the sphere with material block
        SDF_TPL = f"""<?xml version="1.0"?>
    <sdf version="1.6">
    <model name="{{name}}">
        <static>{{static}}</static>
        <link name="link">
        {{collision_block}}
        <visual name="vis">
            <geometry><sphere><radius>{{radius}}</radius></sphere></geometry>
            {MATERIAL_BLOCK}
        </visual>
        </link>
    </model>
    </sdf>"""

        # define collision block based on collide flag
        COLLISION_BLOCK = """<collision name="col">
    <geometry><sphere><radius>{radius}</radius></sphere></geometry>
    </collision>"""
        NO_COLLISION_BLOCK = ""

        # create service proxy for spawning model
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)

        # format collision block from input boolean
        collision_block = (COLLISION_BLOCK if collide else NO_COLLISION_BLOCK).format(radius=radius_m)
        
        # format SDF with input parameters
        sdf = SDF_TPL.format(name=name,
                            static=str(static).lower(),
                            collision_block=collision_block,
                            radius=radius_m)

        # define initial pose
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        qx, qy, qz, qw = T.quaternion_from_euler(roll, pitch, yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw

        return spawn(model_name=name, model_xml=sdf, robot_namespace="",
                    initial_pose=p, reference_frame=reference_frame)

    def delete_model(self, name):
        """Delete a model from the simulation.

        Args:
            name (str): Name of the model to delete.

        Returns:
            DeleteModelResponse: Response from the delete service.
        """
        
        # Define SDF template for the sphere
        rospy.wait_for_service("/gazebo/delete_model")
        
        # create service proxy
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        return delete(name)

    def initialize_waypoint_markers(self):
        '''
        Initialize waypoint markers from DFA in Gazebo simulation environment
        '''
        # get number of wapoints to determine color scaling
        num_waypoints = len(self.DFA_alphabet_dict)
        
        color_delta = 1.0 / max(1, num_waypoints - 1)
        
        # iterate through DFA alphabet dictionary to spawn markers
        for letter, grid_state in self.DFA_alphabet_dict.items():
            # get corresponding pose for grid state
            pose = self.grid_world.grid_state2rect_pose_r(grid_state)
            
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            
            # determine color based on letter index

            # get ordered list of letters (each letter is a "key" in the DFA alphabet dict)
            ordered_letter_list = list(self.DFA_alphabet_dict.keys())
            
            # scale color based on letter index, brightness increases linearly with index
            letter_index = ordered_letter_list.index(letter)
            color_value = letter_index * color_delta

            # create RGB color tuple
            color_rgb = (color_value, color_value, color_value)  # grayscale color

            self.waypoint_names.append(f"waypoint_{letter}")
            
            # spawn sphere at this location
            self.spawn_sphere(name=f"waypoint_{letter}", 
                              radius_m=0.03, 
                              x=x, y=y, z=z, 
                              static=True, 
                              collide=False, 
                              reference_frame="world", 
                              color_rgb=color_rgb)
           
            print(f"Spawned waypoint marker for letter {letter} at grid state {grid_state}")

    def delete_all_markers(self):
        '''
        Delete all waypoint markers from Gazebo simulation environment
        '''
        
        # iterate through DFA alphabet dictionary to delete markers
        for name in self.waypoint_names:
            self.delete_model(name)
            print(f"Deleted waypoint marker {name} from simulation")    
    
        self.waypoint_names.clear()
    
    def close(self):
        # shutdown moveit commander
        self.UR5e_move_group.clear_pose_targets()
        self.UR5e_move_group.clear_path_constraints()
        self.delete_all_markers()
        
        moveit_commander.roscpp_shutdown()        
        
class UR5e_TQ_agent:
    '''
    UR5e agent for tabular Q-learning. 
    
    Args:
        UR5e_env (gym.Env): UR5e gym grid world environment
        learning_rate (float): learning rate for Q-learning updates
        initial_epsilon (float): initial epsilon for epsilon-greedy exploration
        final_epsilon (float): final epsilon for epsilon-greedy exploration
        exploration_fraction (float): fraction of training steps over which to decrease epsilon
        discount_factor (float): discount factor for future rewards
    '''
    def __init__(self, 
                 UR5e_env: gym.Env, 
                 learning_rate: float = 1e-3, 
                 initial_epsilon: float = 1.0,
                 final_epsilon: float = 0.1,
                 exploration_fraction: float = 0.4,
                 discount_factor: float = 0.93, 
                 angle_size: float = 0.52,
                 max_steps: int = 100_000):
        
        # initialize UR5e environment
        self.env = UR5e_env

        # learning rate for Q-learning updates
        self.learning_rate = learning_rate

        # epsilon-greedy exploration parameters
        self.initial_epsilon = initial_epsilon
        self.final_epsilon = final_epsilon
        self.exploration_fraction = exploration_fraction

        # discount factor for future rewards
        self.discount_factor = discount_factor
        
        # initialize epsilon
        self.epsilon = initial_epsilon
        
        # joint angle discretization size
        self.angle_size = angle_size
        
        # maximum number of training steps
        self.max_steps = max_steps
        
        # initialize sampling counter
        self.step_counter = 0
        
        # initialize training error
        self.training_error = []
        
        # initialize Q-table
        self.init_q_table()
        
    def init_q_table(self):
        '''
        Initialize Q-table for tabular Q-learning
        
        Args:
            angle_size (int): thickness of angle discretizations per joint
        '''
        # used to determine observation space limits, 7 is overestimation of 2pi to account for joint angles
        max_angle_int = np.ceil(2*np.pi / self.angle_size)

        max_obs_index = max( (np.ceil( self.env.grid_world.arm_radius / self.env.grid_world.min_thickness ), max_angle_int ) )  
        
        # number of joint angles
        n_joints = len(self.env.UR5e_move_group.get_active_joints())

        # diemension of grid space
        grid_dim = 3
        
        # number of DFA states
        DFA_states = self.env.DFA_monitor.num_states
        
        # observation space dimension is sum of grid dim, eef dim, DFA state size and n_joints
        obs_space_dimension = grid_dim + DFA_states + n_joints
        
        # observation space dependent on grid world
        self.observation_space = spaces.Box(
            low=-max_obs_index, high=max_obs_index, shape=(obs_space_dimension,), dtype=np.int32
        )

        self.q_values = defaultdict(lambda: np.zeros(self.env.action_space.n))
    
    def discrete_obs(self, obs) -> np.array:
        '''
        Discretize continuous observation into discrete state
        
        Args:
            obs (np.array): observation from UR5e gym env that is mix of discrete and continuous values
        
        Returns:
            discrete_obs (tuple): discretized observation as tuple for Q-table indexing
        '''
        
        # split observation into grid state, joint angles, DFA state
        grid_state_array  = obs[0:3]
        joint_angle_array = obs[3:9]
        dfa_state_array   = obs[9:]
        
        # discretize joint angles
        joint_angle_disc_array = self.true_joint_angles2discrete_joint_angles(joint_angle_array)

        # concatenate all discrete components into single observation
        # add initial state to observation for fetch and return task
        if self.env.fetch_return_bool:
            start_grid_state_array = np.array(self.env.grid_world.start_state, dtype=np.int32)
            discrete_obs = np.concatenate(( grid_state_array, joint_angle_disc_array, dfa_state_array, start_grid_state_array))
        else:
            discrete_obs = np.concatenate((grid_state_array, joint_angle_disc_array, dfa_state_array))

        return discrete_obs

    def true_joint_angles2discrete_joint_angles(self, joint_angle_array) -> np.array:
        '''
        Convert true joint angles to discrete joint angles for Q-table indexing
        
        Args:
            joint_angle_array (np.array): array of true joint angles

        Returns:
            discrete_joint_angles (np.array): array of discrete joint angles
        '''
        # discretize joint angles
        # theta_discrete = ceil( (theta - angle_size/2) / angle_size )
        discrete_joint_angles = np.ceil((joint_angle_array - self.angle_size / 2) / self.angle_size).astype(np.int32)
        
        return discrete_joint_angles

    def get_action(self, obs) -> int:
        '''
        Get action using epsilon-greedy policy
        
        Args:
            obs (np.array): current observation from UR5e gym env
        
        Returns:
            action (int): action to take
        '''
        
        # map observations to discrete states
        obs_discrete = self.discrete_obs(obs)

        # map arrays to tuples for table indexing
        obs_key = tuple(np.asarray(obs_discrete, dtype=int))

        if np.random.rand() < self.epsilon:
            # explore
            action = self.env.action_space.sample()
        else:
            # exploit
            action = np.argmax(self.q_values[obs_key])
        
        return action
    
    def update_q_table(self, 
                       obs: np.array, 
                       action: int, 
                       reward: float, 
                       done: bool, 
                       next_obs: np.array):
        '''
        Update Q-table using with temporal difference learning
        Also updatestraining error and num steps
        Args:
            obs (np.array): current observation from UR5e gym env
            action (int): action taken
            reward (float): reward received
            done (bool): whether episode ended
            next_obs (np.array): next observation
        '''
        
        # map observations to discrete states
        obs_discrete      = self.discrete_obs(obs)
        next_obs_discrete = self.discrete_obs(next_obs)
        
        # map arrays to tuples for table indexing
        obs_key = tuple(np.asarray(obs_discrete, dtype=int))
        next_obs_key = tuple(np.asarray(next_obs_discrete, dtype=int))

        # get current Q-value
        current_q_value = self.q_values[obs_key][action]

        # get maximum Q-value for next observation as long as not terminated
        if done:
            # episode finished so no next Q-value
            max_next_q_value = 0  
        else:
            # get max Q-value for next observation
            max_next_q_value = np.max(self.q_values[next_obs_key])
        
        # compute target Q-value
        target_q_value = reward + self.discount_factor * max_next_q_value
        
        # compute temporal difference
        temporal_difference = target_q_value - current_q_value
        
        # update Q-value
        new_q_value = current_q_value + self.learning_rate * temporal_difference
        self.q_values[obs_key][action] = new_q_value

        # record training error
        self.training_error.append(temporal_difference)
        
        # update step counter
        self.step_counter += 1
        
        return temporal_difference

    def decay_epsilon(self):
        '''
        Decay the exploration rate (epsilon) over time.
        '''
        delta = np.amin( ( self.step_counter / (self.exploration_fraction * self.max_steps), 1 ) )
        delta_epsilon = delta * (self.initial_epsilon - self.final_epsilon)
        self.epsilon = self.initial_epsilon - delta_epsilon

    def save_model_data(self, path: str, 
                     time_elapsed: Optional[float], 
                     episode_returns: Optional[List[float]] = None, 
                     episode_lengths: Optional[List[int]] = None):
        '''
        Save the RL model data to a file with provided path.
        Args:
            path (str): file path to save the Q-table
            time_elapsed (float): total time elapsed during training
            episode_returns (list): list of episode returns during training
            episode_lengths (list): list of episode lengths during training
        '''
        
        # get learning agent pararmeters
        
        TQL_agent_params = {
            "learning_rate": self.learning_rate,
            "initial_epsilon": self.initial_epsilon,
            "final_epsilon": self.final_epsilon,
            "exploration_fraction": self.exploration_fraction,
            "discount_factor": self.discount_factor,
            "angle_size": self.angle_size,
            "max_steps": self.max_steps
        }
        
        # get environment info
        env_info = self.env.get_env_info()
        
        # assemble payload
        payload = {
            "n_actions": int(self.env.num_actions),
            "q": {k: v.astype(np.float32) for k, v in self.q_values.items()},
            "time_elapsed": time_elapsed,
            "episode_returns": episode_returns,
            'episode_lengths': episode_lengths,
            "env_info": env_info, 
            "TQL_agent_params": TQL_agent_params
        }
        with open(path, "wb") as f:
            pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)

def env_info2UR5e_grid_env(env_info: dict) -> UR5eGridEnvwDFA:
    """
    Create UR5eGridEnvwDFA from env_info dict by unpacking only valid ctor args.
    The env_info dict should include the required keys (e.g. 'start_states','goal_state',...).
    """
    # collect valid constructor parameter names (exclude 'self')
    sig = inspect.signature(UR5eGridEnvwDFA.__init__)
    valid_params = [p for p in sig.parameters if p != "self"]

    # keep only keys present in the constructor signature
    filtered = {k: v for k, v in env_info.items() if k in valid_params}

    # optional: provide defaults or raise if required params missing
    missing = [p for p in valid_params if p not in filtered and sig.parameters[p].default is inspect._empty]
    if missing:
        raise TypeError(f"Missing required env_info keys: {missing}")

    # construct environment using keyword expansion
    return UR5eGridEnvwDFA(**filtered)

def TQL_params2TQL_agent(env: UR5eGridEnvwDFA, TQL_agent_params: dict) -> UR5e_TQ_agent:
    """
    Create UR5e_TQ_agent from TQL_agent_params dict by unpacking only valid ctor args.
    The TQL_agent_params dict should include the required keys (e.g. 'learning_rate','initial_epsilon',...).
    """
    # collect valid constructor parameter names (exclude 'self' and 'UR5e_env')
    sig = inspect.signature(UR5e_TQ_agent.__init__)
    valid_params = [p for p in sig.parameters if p not in ("self", "UR5e_env")]

    # keep only keys present in the constructor signature
    filtered = {k: v for k, v in TQL_agent_params.items() if k in valid_params}

    # optional: provide defaults or raise if required params missing
    missing = [p for p in valid_params if p not in filtered and sig.parameters[p].default is inspect._empty]
    if missing:
        raise TypeError(f"Missing required TQL_agent_params keys: {missing}")

    # construct agent using keyword expansion
    return UR5e_TQ_agent(UR5e_env=env, **filtered)

if __name__ == "__main__":
    # simple test of environment
    env = UR5eGridEnv()
    obs, info = env.reset()
    print(f"Initial Observation: {obs}")
    
    done = False
    total_reward = 0.0
    
    while not done:
        action = env.action_space.sample()  # random action
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        done = terminated or truncated
        print(f"Step: {env.current_step}, Action: {action}, Observation: {obs}, Reward: {reward}")
    
    print(f"Episode finished. Total Reward: {total_reward}")
    env.close()
