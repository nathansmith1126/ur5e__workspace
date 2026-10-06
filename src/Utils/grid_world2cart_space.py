#!/usr/bin/env python3

import sys
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetPositionFKRequest, GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose 
from typing import Optional, Union, Sequence 
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import RobotTrajectory
from geometry_msgs.msg import PoseStamped 
from typing import Optional
import matplotlib.pyplot as plt

class grid_world:
    '''
    Class to define a grid world environment based on a discretization of the cartesian space
    '''  
    def __init__(self, 
                 start_state: Optional[Sequence[int]] = [1, 1, 3],
                 goal_state: Optional[Sequence[int]] = [-1, -1, 1],
                 arm_radius: Optional[float] = 0.850, 
                 coordinate_system: Optional[str] = "UR5e_cent_rect", 
                 grid_size_array: Optional[Sequence[float]] = [0.25, 0.25, 0.25], 
                 safe_joint_angles: Optional[Sequence[float]] = None):
       # Ur5e radius at maximum extension
       self.arm_radius = arm_radius
       
       # safe joint angles for ur5e
       if safe_joint_angles is None:
           self.safe_joint_angles = [0.0, -np.pi/2, np.pi/2, 0.0, np.pi/2, 0.0]
       else:
           self.safe_joint_angles = safe_joint_angles

       # Choice of coordinate system
       self.coordinate_system = coordinate_system
       
       # initialize potential discretizations
       self.polar_slices = None 
       self.rectangular_slices = None 
       
       # rectangular grid world origin to ur5e origin in grid world frame
       self.OgrOr = None
       
       # set grid parameters
       self.grid_size_array = grid_size_array

       # define start state based on grid size array
       self.start_state = start_state # easy to get to
        
       # define goal state based on grid size array
       self.goal_state = goal_state # hard to get to 
        
       self.set_grid()
    
    def set_grid(self):
        if self.coordinate_system == 'polar':
            radius_slices = 9
            theta_slices  = 18
            z_slices      = 9
            self.polar_slices = (radius_slices, theta_slices, z_slices)
            
            # thickness of slice in each direction
            self.radial_slice_thickness = self.arm_radius/self.polar_slices[0]
            self.theta_slice_thickness  = 2*np.pi/self.polar_slices[1]
            self.z_slice_thickness      = self.arm_radius/self.polar_slices[2]
        elif self.coordinate_system == 'rectangular':
            # rectangular coordinate system established around workshop table
            
            # origin in table grid world is in southwest corner of table
            # gr - denotes grid world frame
            # r  - denotes ur5e robot frame
            self.OgrOr = np.array([-3.5, -30, 0])/39.37 # meters
            
            # define size of grid world
            x_num_slices = 4
            y_num_slices = 4
            z_num_slices = 4
            
            # define thickness of grid states
            x_slice_thickness = 2*self.arm_radius/x_num_slices
            y_slice_thickness = 2*self.arm_radius/y_num_slices
            z_slice_thickness = 2*self.arm_radius/z_num_slices

            # organize into class variable vectors
            self.rectangular_num_slices_list = [x_num_slices, y_num_slices, z_num_slices]
            self.rectangular_thickness_list = [x_slice_thickness, y_slice_thickness, z_slice_thickness]
            
        elif self.coordinate_system is "UR5e_cent_rect":
            # rectangular coordinate system established around ur5e base
            
            # origin in table grid world is in southwest corner of table
            # gr - denotes grid world frame
            # r  - denotes ur5e robot frame
            
            # grid world origin is at ur5e base/origin
            self.OgrOr = np.array([0, 0, 0])
            
            # define size of grid world
            # x_num_slices = 6
            # y_num_slices = 6
            # z_num_slices = 6  
            
            # define thickness of grid states based on number of slices
            # x_slice_thickness = 2*self.arm_radius/x_num_slices
            # y_slice_thickness = 2*self.arm_radius/y_num_slices
            # z_slice_thickness = 2*self.arm_radius/z_num_slices
            
            # define size of grid world based on provided grid size array
            x_slice_thickness = self.grid_size_array[0]
            y_slice_thickness = self.grid_size_array[1]
            z_slice_thickness = self.grid_size_array[2]
            
            # organize into list of thicknesses
            self.rectangular_thickness_list = [x_slice_thickness, y_slice_thickness, z_slice_thickness]
            self.min_thickness = np.min(self.rectangular_thickness_list)
            
        else:
            raise ValueError('coordinate system must be polar or rectangular')
        
    def polar_pose2grid_pose(self, polar_pose: Union[Sequence[float], np.ndarray] ):
        '''
        Method to map the gripper arm location in 3D space 
        to it's state in the discretized grid world
        
        Args:
        polar_pose: ndarray-like - [r, theta, z]
        
        Returns:
        grid_polar_pose: (3,) ndarray - [radius_index, theta_index, z_index]
        '''
        
        # index := floor( pose/slice_thickness )
        radial_index = np.floor(polar_pose[0]/self.radial_slice_thickness)
        theta_index  = np.floor(polar_pose[1]/self.theta_slice_thickness)
        z_index      = np.floor(polar_pose[2]/self.z_slice_thickness)  
        
        grid_polar_pose = np.asarray([radial_index, theta_index, z_index])
        return grid_polar_pose

    def rect_pose_r2rect_pose_g(self, pose_r: Union[Pose, np.ndarray])-> np.ndarray:
        '''
        Maps rectangular coordinates from ur5e robot frame to grid world with table
        
        Args:
        pose: Pose - vector in ur5e frame
        
        Returns:
        pose_g_array: np.darray - vector in grid world frame
        '''
        # extract position from Pose message or numpy array
        if isinstance(pose_r, Pose):
            # extract position from Pose message
            x_r = pose_r.position.x
            y_r = pose_r.position.y
            z_r = pose_r.position.z
        elif isinstance(pose_r, np.ndarray):
            # assume pose_r is already a numpy array
            x_r = pose_r[0]
            y_r = pose_r[1]
            z_r = pose_r[2]
        else:
            raise ValueError('pose_r must be of type geometry_msgs/Pose or numpy.ndarray')

        pose_r_array = np.array([x_r, y_r, z_r])
        
        # translate by origin of grid world
        # p_g = p_r + O_grO_r
        pose_g_array = pose_r_array + self.OgrOr
        
        return pose_g_array
    
    def rect_pose_g2rect_pose_r(self, pose_g: Union[Pose, np.ndarray])-> np.ndarray:
        '''
        Maps rectangular coordinates from grid world frame to ur5e robot frame
        
        Args:
        pose: Pose - vector in grid world frame
        
        Returns:
        pose_r_array: np.darray - vector in ur5e robot frame
        '''
        # extract position from Pose message or numpy array
        if isinstance(pose_g, Pose):
            # extract position from Pose message
            x_g = pose_g.position.x
            y_g = pose_g.position.y
            z_g = pose_g.position.z
        elif isinstance(pose_g, np.ndarray):
            # assume pose_r is already a numpy array
            x_g = pose_g[0]
            y_g = pose_g[1]
            z_g = pose_g[2]
        else:
            raise ValueError('pose_g must be of type geometry_msgs/Pose or numpy.ndarray')

        pose_g_array = np.array([x_g, y_g, z_g])
        
        # translate by origin of grid world 
        # p_g = p_r + O_grO_r  =>  p_r = p_g - O_grO_r
        pose_r_array = pose_g_array - self.OgrOr
        
        return pose_r_array

    def rect_pose_r2grid_state(self, pose_r: Union[Pose, np.ndarray]) -> list:
        '''
        Maps rectangular coordinates from ur5e robot frame to grid world state with table
        
        Args:
        pose: Pose - vector in ur5e frame
        
        Returns:
        grid_state: list - [x_index, y_index, z_index]
        '''
        # initialize grid state
        grid_state = []
        
        # map pose to array in grid frame
        pose_g_array = self.rect_pose_r2rect_pose_g(pose_r=pose_r)
        
        for index in np.arange(3):
            # get coordinate and thickness for each dimension
            coord = pose_g_array[index]
            thickness = self.rectangular_thickness_list[index]
            
            # calculate index
            if self.coordinate_system == "UR5e_cent_rect":
                # grid world centered at ur5e base
                coord_index = np.ceil( ( coord/thickness - 1/2 ) )
            else:
                # grid world centered at table corner
                coord_index = np.floor(coord/thickness)

            grid_state.append(int(coord_index))

        return grid_state

    def plan2tool_traj(self, trajectory: RobotTrajectory, 
                       fk_link: Optional[str]="tool0", 
                       robot: Optional[RobotCommander] = None)-> tuple:
        """
        Function to map a move_group plan to a trajectory in cartesian space as s 
        list of dicts: {'t': float_seconds, 'x': ..., 'y': ..., 'z': ..., 'r': ..., 'theta': ... ,} for each waypoint
        
        Args:
        
        trajectory: - instance of RobotTrajectory class from move_group 
        to analyze corresponding to waypoints for a path of interest
        
        Returns:
        
        trajectory_list: - list of dicts:  {'t': float_seconds, 'x': ..., 'y': ..., 'z': ..., 'r': ..., 'theta': ... ,} for each waypoint
        grid_trajectory_list: - list of grid states corresponding to each waypoint
        joint_trajectory_list: [{'t', 'q': [q1,...,q6]}, ...]  - joint angles in same order as 
                            joint_names
        joint_names: [str, ...] - names for q indices corresponding to the name of each joint angle
        """

        # Extract the JointTrajectory from the RobotTrajectory
        jt = trajectory.joint_trajectory

        # Cache joint name order used in each trajectory point
        joint_names = jt.joint_names
        
        # Connect to MoveIt to query model and frames
        if robot is None:
            # initialize robot object if one is not provided
            robot = RobotCommander()

        # Determine the reference frame for FK requests. Fallback to "world" if empty
        planning_frame = robot.get_planning_frame() or "world"

        # Default service name used by MoveIt for FK
        fk_service_name = '/compute_fk'

        # Block until the FK service becomes available or timeout
        try:
            rospy.wait_for_service(fk_service_name, timeout=5.0)
        except rospy.ROSException:
            # Fallback service name without a leading slash
            fk_service_name = 'compute_fk'
            # Wait again for the fallback name
            rospy.wait_for_service(fk_service_name, timeout=5.0)

        # Create a callable client for the FK service
        fk = rospy.ServiceProxy(fk_service_name, GetPositionFK)

        # Will hold {'t', 'x', 'y', 'z'} dicts for each waypoint
        trajectory_list = []
        
        # grid trajectory list
        grid_trajectory_list = []
        
        # will house trajectory of UR5e joint angles
        joint_trajectory_list = []
        
        # Iterate over each JointTrajectoryPoint in the plan
        for pt in jt.points:

            # Build a JointState message for this waypoint
            js = JointState()

            # Set joint name order for the FK solver
            js.name = list(joint_names)

            # Set joint positions at this waypoint
            js.position = list(pt.positions)

            # Create an FK request container
            req = GetPositionFKRequest()

            # Ask results to be expressed in the planning frame
            req.header.frame_id = planning_frame

            # Request FK for one link only, default "tool0"
            req.fk_link_names = [fk_link]

            # Provide the robot state for FK at this waypoint
            req.robot_state.joint_state = js

            # Call the FK service and get the response
            resp = fk(req)

            # If FK failed or returned no pose then skip this waypoint
            if resp.error_code.val <= 0 or not resp.pose_stamped:
                continue

            # Extract geometry_msgs/Point from the first PoseStamped
            p = resp.pose_stamped[0].pose.position

            # calculate polar coordinates
            radius = np.sqrt(p.x**2 + p.y**2)
            theta  = np.arctan2(p.y, p.x)
            
            # Append time-stamped XYZ and polar coordinatescoordinates
            trajectory_list.append({
                't': pt.time_from_start.to_sec(),
                'x': p.x, 'y': p.y, 'z': p.z, 'r': radius, 'theta': theta
            })
            
            # map to grid state
            if self.coordinate_system == 'rectangular' or self.coordinate_system == 'UR5e_cent_rect':
                # map pose to numpy array first and then to grid state
                pose_array_r = np.array([p.x, p.y, p.z])
                grid_state = self.rect_pose_r2grid_state(pose_array_r)
            elif self.coordinate_system == 'polar':
                # map pose to numpy array first and then to grid state
                polar_pose = np.array([radius, theta, p.z])
                grid_state = self.polar_pose2grid_pose(polar_pose).tolist()
            else:
                raise ValueError('coordinate system must be polar, rectangular or UR5e_cent_rect')
            
            grid_trajectory_list.append( grid_state )
            
            # save joint angles
            joint_trajectory_list.append({
                't': pt.time_from_start.to_sec(),
                'q': list(pt.positions)  # radians
            })
                
        # Return the time-stamped XYZ series
        return trajectory_list, grid_trajectory_list, joint_trajectory_list, joint_names

    def plot_plan_trajectories(self,
                            plan: RobotTrajectory,
                            fk_link: str = "tool0",
                            robot: Optional[RobotCommander] = None,
                            save_basename: Optional[str] = None) -> None:
        """
        Plot x/y/z pose vs time AND grid (gx/gy/gz) vs time for a MoveIt plan.

        Args:
            plan: RobotTrajectory returned by MoveIt.
            fk_link: end-effector link for FK sampling (default: "tool0").
            robot: optional RobotCommander to reuse (faster if provided).
            save_basename: if provided, saves two PNGs:
                f"{save_basename}_pose_xyz.png" and f"{save_basename}_grid_gxgygz.png"
                Otherwise, just shows the figures.
        """

        # ---- Extract trajectories using your helper ----
        traj_list, grid_list, joint_traj, joint_names = self.plan2tool_traj(plan, fk_link=fk_link, robot=robot)

        if len(traj_list) == 0 or len(grid_list) == 0:
            print("No waypoints to plot (FK failed or empty plan).")
            return

        # ---- Convert to arrays ----
        t = np.array([d["t"] for d in traj_list], dtype=float)
        x = np.array([d["x"] for d in traj_list], dtype=float)
        y = np.array([d["y"] for d in traj_list], dtype=float)
        z = np.array([d["z"] for d in traj_list], dtype=float)
        Q = np.array([d['q'] for d in joint_traj], float)   # shape [N, n_joints]
        
        # shape [N, 3] assumed (gx, gy, gz)
        grid_arr = np.asarray(grid_list, dtype=float)  
        if grid_arr.ndim != 2 or grid_arr.shape[1] != 3:
            raise ValueError(f"Expected grid states of shape [N,3], got {grid_arr.shape}")

        gx, gy, gz = grid_arr[:, 0], grid_arr[:, 1], grid_arr[:, 2]

        LINE_STYLES = [
            ("solid",   "o"),   # ───── with circles
            ("dashed",  "s"),   # ─ ─ ─ with squares
            ("dashdot", "^"),   # ─·─·─ with triangles
            ("dotted",  "x"),   # ····· with x markers
            ("solid",   "D"),   # solid with diamonds
            ("dashed",  "v"),   # dashed with down triangles
        ]

        def style(i):
            ls, mk = LINE_STYLES[i % len(LINE_STYLES)]
            return dict(linestyle=ls, marker=mk, linewidth=2.0, markersize=5)
        
        # ---- Figure 1: Pose XYZ vs time ----
        fig1 = plt.figure(figsize=(8, 5))
        ax1 = fig1.add_subplot(111)
        ax1.plot(t, x, linestyle =  'solid', label="x (m)")
        ax1.plot(t, y, linestyle = ':', label="y (m)")
        ax1.plot(t, z, linestyle = '--', label="z (m)")
        ax1.set_xlabel("time (s)")
        ax1.set_ylabel("position (m)")
        ax1.set_title("End-effector pose vs time")
        ax1.grid(True, linestyle="--", alpha=0.4)
        ax1.legend(loc="best")
        fig1.tight_layout()

        # ---- Figure 2: Grid indices vs time (step plots) ----
        fig2 = plt.figure(figsize=(8, 5))
        ax2 = fig2.add_subplot(111)
        ax2.step(t, gx, where="post", linestyle = 'solid', label="gx")
        ax2.step(t, gy, where="post", linestyle = 'dashed', label="gy")
        ax2.step(t, gz, where="post", linestyle = 'dashdot', label="gz")
        ax2.set_xlabel("time (s)")
        ax2.set_ylabel("grid index")
        ax2.set_title("Grid state vs time")
        ax2.grid(True, linestyle="--", alpha=0.4)
        ax2.legend(loc="best")
        fig2.tight_layout()

        # ---- Figure 3 Joint Angles vs time
        fig3 = plt.figure(figsize=(8,5))
        for j,name in enumerate(joint_names):
            plt.plot(t, Q[:, j], label=name)
        plt.xlabel("time (s)")
        plt.ylabel("joint angle (rad)")
        plt.title("Joint angles vs time")
        plt.grid(True, linestyle="--", alpha=0.4)
        plt.legend(loc="best")
        fig3.tight_layout()


        # ---- Save or show ----
        if save_basename:
            p1 = f"{save_basename}_pose_xyz.png"
            p2 = f"{save_basename}_grid_gxgygz.png"
            fig1.savefig(p1, dpi=150)
            fig2.savefig(p2, dpi=150)
            print(f"Saved: {p1}\nSaved: {p2}")
            plt.close(fig1)
            plt.close(fig2)
        else:
            plt.show()

    def grid_state2rect_pose_g(self, grid_state: Union[Sequence[int], np.ndarray]) -> Pose:
        '''
        Maps a grid state in the rectangular grid world to a pose in the grid world frame
        
        Args:
        grid_state: list or ndarray - [x_index, y_index, z_index]
        
        Returns:
        pose_g_array: ndarray - [x, y, z] in rectangular Ur5e frame
        '''
        if len(grid_state) != 3:
            raise ValueError('grid_state must be of length 3')
        
        # initialize pose and numpy array
        pose_g_array = np.zeros(3)
        pose_g = Pose()
        
        # iterate through each dimension (x,y,z)
        for dim_index in np.arange(3):
            
            # get index and thickness for each dimension
            state_index = grid_state[dim_index]
            thickness = self.rectangular_thickness_list[dim_index]

            if self.coordinate_system == "UR5e_cent_rect":
                # grid world centered at ur5e base
                # calculate coordinate defined as center of grid cube
                rect_coord = state_index*thickness 
            else:
                # grid world centered at table corner
                # coordinate defined at center of grid cube
                rect_coord = state_index*thickness + thickness/2
            
            # assign to pose array
            pose_g_array[dim_index] = rect_coord
            
        # convert to Pose message
        pose_g.position.x = pose_g_array[0]
        pose_g.position.y = pose_g_array[1]
        pose_g.position.z = pose_g_array[2]
        return pose_g

    def grid_state2rect_pose_r(self, grid_state: Union[Sequence[int], np.ndarray]) -> Pose:
        '''
        Maps a grid state in the rectangular grid world to a pose in the ur5e robot frame
        
        Args:
        grid_state: list or ndarray - [x_index, y_index, z_index]
        
        Returns:
        pose_r_array: ndarray - [x, y, z] in rectangular Ur5e frame
        '''
        if len(grid_state) != 3:
            raise ValueError('grid_state must be of length 3')
        
        # get pose in grid world frame first
        pose_g = self.grid_state2rect_pose_g(grid_state=grid_state)
        
        # map to ur5e robot frame
        pose_r_array = self.rect_pose_g2rect_pose_r(pose_g=pose_g)
        
        # convert to Pose message
        pose_r = Pose()
        pose_r.position.x = pose_r_array[0]
        pose_r.position.y = pose_r_array[1]
        pose_r.position.z = pose_r_array[2]
        
        return pose_r
    
    def grid_iso_step(self, action: Union[Sequence[int], np.ndarray], 
                      current_grid_state: Union[Sequence[int], np.ndarray]) -> list:
        '''
        Method to take an action in the grid world and return the next grid state.
        Note: ignores feasibility of movement dictated by URR5e move_group planner

        Args:
        action: list or ndarray - [dx, dy, dz] where each element is -1, 0, or 1
        current_grid_state: list or ndarray - [x_index, y_index, z_index]
        
        Returns:
        next_grid_state: list - [x_index, y_index, z_index]
        '''
        if len(action) != 3:
            raise ValueError('action must be of length 3')
        if len(current_grid_state) != 3:
            raise ValueError('current_grid_state must be of length 3')

        # convert actions and current state numpy arrays for transition calculations
        action_array = np.array(action)
        current_grid_state_array = np.array(current_grid_state)

        # calculate next grid state
        next_grid_state_array = current_grid_state_array + action_array

        # convert to list
        next_grid_state = next_grid_state_array.tolist()
        return next_grid_state
  
    