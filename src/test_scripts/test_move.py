#!/usr/bin/env python3
"""After sourcing setup_project.sh with open_venv and required ros/move_it nodes, run from ur5e_ws:

python3 -m src.test_scripts.test_move

Gazebo or the real robot driver and MoveIt must already be running.
Edit the target below before running.
"""

import numpy as np
import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from src.Utils.misc import add_table2scene, add_gripper2scene


# Choose "joints" or "pose".
TARGET_TYPE = "joints"
# TARGET_TYPE = "pose"

# Radians, in MoveIt's active-joint order (printed when running).
JOINT_ANGLES = [np.pi/2, -np.pi / 2, np.pi / 2, 0.0, np.pi / 2, 0.0]

# tool0 pose in the planning frame: 
# position in [x, y, z] meters.
TOOL_POSITION = [0.4, 0.1, 0.4]

# orientation quaternions [qx, qy, qz, qw].
TOOL_ORIENTATION = [0.0, 1.0, 0.0, 0.0]

def main():
    
    # initialize move_it node
    moveit_commander.roscpp_initialize([])
    rospy.init_node("test_move", anonymous=True)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    
    try:
        # define end_effector as toolpiece
        group.set_end_effector_link("tool0")
        
        # limit velocity and accelaration
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)
        
        # set planning time for sample based planner
        group.set_planning_time(10.0)

        # inform move_it planner of table
        add_table2scene(robot, scene)
        
        # inform move_it planner of gripper arm
        add_gripper2scene(scene, group)
        if "Table" not in scene.get_known_object_names() or not scene.get_attached_objects(["gripper_box"]):
            raise RuntimeError("Table or gripper was not added to the planning scene")

        group.set_start_state_to_current_state()
        
        # set desired coordinates to move group
        if TARGET_TYPE == "joints":
            # move to specifieid joint angles
            rospy.loginfo("Joint order: %s", group.get_active_joints())
            
            # add joint angles to group
            group.set_joint_value_target(JOINT_ANGLES)
        elif TARGET_TYPE == "pose":
            # move to specified to toolpiece pose
            # initialize empty pose object
            pose = Pose()
            
            # add position attributes
            pose.position.x, pose.position.y, pose.position.z = TOOL_POSITION
            
            # add orientations attributes as quaternions
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = TOOL_ORIENTATION
            
            # fix reference frame
            group.set_pose_reference_frame(group.get_planning_frame())
            
            # add desired pose to group
            group.set_pose_target(pose, "tool0")
        else:
            raise ValueError('TARGET_TYPE must be "joints" or "pose"')

        # plan to new pose or joint angles
        success, trajectory, _, _ = group.plan()
        
        # planning failed
        if not success or not trajectory.joint_trajectory.points:
            raise RuntimeError("MoveIt could not find a motion plan")
        
        # attempt to follow the trajectory
        if not group.execute(trajectory, wait=True):
            raise RuntimeError("Motion execution failed")
        rospy.loginfo("Motion completed")
    finally:
        # clean up
        group.stop()
        group.clear_pose_targets()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    main()
