import gymnasium as gym 
from gymnasium import spaces
import numpy as np
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from src.Utils.grid_world2cart_space import grid_world
from src.Utils.misc import add_table2scene
from typing import Optional, Sequence   

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
      
        # used to determine observation space limits
        max_grid_index = np.ceil( self.grid_world.arm_radius / self.grid_world.min_thickness )
        
        # observation space dependent on grid world
        self.observation_space = spaces.Box(
            low=-max_grid_index, high=max_grid_index, shape=(3,), dtype=np.float32
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
