#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped 
from typing import Optional

class grid_world:
    '''
    Class to define a grid world environment based on a discretization of the cartesian space
    '''  
    def __init__(self, grid_size: Optional[tuple] = (4,4,4)):
        self.grid_size = grid_size