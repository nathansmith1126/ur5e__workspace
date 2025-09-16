#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
<<<<<<< HEAD
from geometry_msgs.msg import PoseStamped, Pose 
from typing import Optional, Union, Sequence 
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import RobotTrajectory
=======
from geometry_msgs.msg import PoseStamped 
from typing import Optional
>>>>>>> parent of 1b63faf... modifiedgrid world class

class grid_world:
    '''
    Class to define a grid world environment based on a discretization of the cartesian space
    '''  
<<<<<<< HEAD
    def __init__(self, arm_radius: Optional[float] = 0.850, 
                 coordinate_system: Optional[str] = "UR5e_cent_rect", 
                 grid_size_array: Optional[Sequence[int]] = [0.25, 0.25, 0.25]):
       # Ur5e radius at maximum extension
       self.arm_radius = arm_radius
       
       # Choice of coordinate system
       self.coordinate_system = coordinate_system
       
       # initialize potential discretizations
       self.polar_slices = None 
       self.rectangular_slices = None 
       
       # rectangular grid world origin to ur5e origin in grid world frame
       self.OgrOr = None
       
       # set grid parameters
       self.grid_size_array = grid_size_array

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
            
            # define start state based on grid size array
            self.start_state = [1,1,3] # easy to get to
            
            # define goal state based on grid size array
            self.goal_state = [-1,-1,1] # hard to get to
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
            
        # Return the time-stamped XYZ series
        return trajectory_list, grid_trajectory_list
    
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
=======
    def __init__(self, grid_size: Optional[tuple] = (4,4,4)):
        self.grid_size = grid_size
>>>>>>> parent of 1b63faf... modifiedgrid world class
