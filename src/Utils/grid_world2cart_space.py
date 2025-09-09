#!/usr/bin/env python3
import numpy as np
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 
from typing import Optional, Union, Sequence 
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import RobotTrajectory

class grid_world:
    '''
    Class to define a grid world environment based on a discretization of the cartesian space
    '''  
    def __init__(self, arm_radius: Optional[float] = 0.850, coordinate_system: Optional[str] = "polar"):
       # Ur5e radius at maximum extension
       self.arm_radius = arm_radius
       
       # Choice of coordinate system
       self.coordinate_system = coordinate_system
       
       # initialize potential discretizations
       self.polar_slices = None 
       self.rectangular_slices = None 
    
    def set_grid(self):
        if self.coordinate_system == 'polar' or self.coordinate_system == 'rectangular':
            if self.coordinate_system == 'polar':
                radius_slices = 9
                theta_slices  = 18
                z_slices      = 9
                self.polar_slices = (radius_slices, theta_slices, z_slices)
            else:
                # rectangular coordinate system
                x_slices = 8
                y_slices = 8
                z_slices = 8
                self.rectangular_slices = (x_slices, y_slices, z_slices)
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
        
        # thickness of slice in each direction
        radial_slice = self.arm_radius/self.polar_slices[0]
        theta_slice  = 2*np.pi/self.polar_slices[1]
        z_slice      = self.arm_radius/self.polar_slices[2]
        
        # index := floor( pose/slice )
        radial_index = np.floor(polar_pose[0]/radial_slice)
        theta_index  = np.floor(polar_pose[1]/theta_slice)
        z_index      = np.floor(polar_pose[2]/z_slice)  
        
        grid_polar_pose = np.asarray([radial_index, theta_index, z_index])
        return grid_polar_pose
    
    def plan2tool_traj(self, trajectory: RobotTrajectory, 
                       fk_link: Optional[str]="tool0", 
                       robot: Optional[RobotCommander] = None)->list:
        """
        Function to map a move_group plan to a trajectory in cartesian space as s 
        list of dicts: {'t': float_seconds, 'x': ..., 'y': ..., 'z': ..., 'r': ..., 'theta': ... ,} for each waypoint
        
        Args:
        
        trajectory: - instance of RobotTrajectory class from move_group 
        to analyze corresponding to waypoints for a path of interest
        
        Returns:
        
        trajectory_list: - list of dicts:  {'t': float_seconds, 'x': ..., 'y': ..., 'z': ..., 'r': ..., 'theta': ... ,} for each waypoint
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

        # Return the time-stamped XYZ series
        return trajectory_list
        