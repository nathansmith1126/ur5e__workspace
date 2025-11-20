from cProfile import label
import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from src.Utils.AUTOMATA.auto_funcs import DFAMonitor
from typing import Optional, Sequence, List, Tuple, Dict, Union
from collections import defaultdict

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
                 plan_time: Optional[float] = 3.0,):
        super().__init__()
        
        # Initialize moveit planning node
        moveit_commander.roscpp_initialize([])
        
        # initialize this script as a ROS node
        rospy.init_node("ur5e_grid_env", anonymous=True)

        # Initialize custom grid world parameters
        self.start_states = start_states
        self.goal_state = goal_state
        self.grid_size_array = grid_size_array
        
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
        
        # add position tolerances for moveit
        self.UR5e_move_group.set_goal_position_tolerance(pos_tolerance) # 5mm
        
        # add time allowance for planning
        self.UR5e_move_group.set_planning_time(plan_time) # 5 seconds

        # initialize info dictionary that is returned at each bym env step
        # It will house planning time and error codes from MoveIt
        
        self.info_dict = {"plan_time": None, "error_code": None}
        
        # initialize DFA monitor for temporal rewards 
        self.DFA_monitor = DFA_monitor 
        
        # optional alphabet dictionary to map grid labels to DFA letters (IF NECESSARY)
        self.DFA_alphabet_dict = DFA_alphabet_dict
      
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

         # Generate action map
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
        
    def generate_action_map(self):
        '''
        Generate action map based on type of DFA provided
        '''    
        
        if self.DFA_alphabet_dict:
            # indicates we are dealing with a DFA that involves moving through a fixed set of configurations
            # Action space: 6 discrete moves + num_configs DFA letters + movement to "safe" configuration
            num_actions = len(self.DFA_alphabet_dict) + 6 + 1
            self.action_space = spaces.Discrete(num_actions)

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
            safe_traj_dict = {int(num_actions): "safe_config" }
            
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
            
            # update current observation
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
        
        # generate grid world
        self.generate_grid_world()
        
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
        
        if self.action_map[action] == "safe_config":
            # move to safe configuration
           joint_angles = self.grid_world.safe_joint_angles
           
            # plan and execute joint angle move
           success, plan, plan_time, error_code = self.joint_angle_plan(joint_angles)
        else:
            if isinstance(self.action_map[action], list):
                # action is a movement direction in cartesian space
                action_list = self.action_map[int(action)]

                # compute target grid state
                target_grid_state = self.grid_world.grid_iso_step(action_list, self.current_grid_state)
            
            else:
                # action is a specific configuration to move to
                config_name = self.action_map[int(action)]
                
                # pull from DFA alphabet dict to get current target grid state
                target_grid_state = self.DFA_alphabet_dict[config_name]

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
        
        # # map action to xyz translation
        # action_list = self.action_map[int(action)]

        # # compute target grid state
        # target_grid_state = self.grid_world.grid_iso_step(action_list, self.current_grid_state)
        # target_pose = self.grid_world.grid_state2rect_pose_r(target_grid_state)

        # # plan to target pose
        # self.UR5e_move_group.set_start_state_to_current_state()
        # self.UR5e_move_group.set_pose_target(target_pose, end_effector_link="tool0")
        # success, plan, plan_time, error_code = self.UR5e_move_group.plan()
        
        # get plan based on action
        success, plan, plan_time, error_code = self.UR5e_step(action)

        # update info dictionary
        self.info_dict["plan_time"] = plan_time
        self.info_dict["error_code"] = error_code
        
        # initialize reward and termination flags
        terminated = False
        truncated = False

        # execute if plan is successful
        if success and len(plan.joint_trajectory.points) > 0:
            # execute plan
            self.UR5e_move_group.execute(plan, wait=True)
            
            # update current grid state and observation
            self.update_obs()
            print(f"current grid state after step is {self.current_grid_state}")
            
            # get label for DFA from current and previous states 
            auto_letter = self.get_auto_letter()
            print(f"Automaton letter for this step: {auto_letter}")
            
            # update DFA monitor state
            self.DFA_monitor.step(auto_letter)
            
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
            reward += self.auto_reward_scaler * self.DFA_monitor.delta_potential
        else:
            # subtract penalty for failure to execute
            reward -= self.failed_trans_penalty

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

    def grid_move(self, grid_state: List[int] ):
        '''
        Move UR5e to specified grid state directly
        
        Args:
            grid_state (list): target grid state to move to
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
        else:
            print(f"Failed to plan for grid state {grid_state} using RRT connect") 
            print(f"Planned for {plan_time} seconds with error code {error_code}")    
        self.UR5e_move_group.clear_pose_targets()

        return success_bool, trajectory, plan_time, error_code 

    def close(self):
        # shutdown moveit commander
        self.UR5e_move_group.clear_pose_targets()
        self.UR5e_move_group.clear_path_constraints()
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
            obs (np.array): current observation
        
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
            obs (np.array): current observation
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

    def decay_epsilon(self):
        '''
        Decay the exploration rate (epsilon) over time.
        '''
        delta = self.step_counter / (self.exploration_fraction * self.max_steps)
        self.epsilon = self.initial_epsilon - delta * (self.initial_epsilon - self.final_epsilon)

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
