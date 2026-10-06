#!/usr/bin/env python3
"""Run from ur5e_ws after sourcing setup_project.sh:

    python3 -m src.test_scripts.pick_place

Requires the real UR driver, MoveIt, and an activated gripper.
Edit both example tool0 poses for your workspace before running.
Paths are planned between poses; no approach/lift waypoints or carried-object
collision geometry are added by this minimal example.
"""
import rospy 
import argparse
import socket # communication channel to gripper
from typing import Tuple

import moveit_commander
import rospy
from geometry_msgs.msg import Pose
from object_localization.srv import Centroid
from src.Utils.misc import add_gripper2scene, add_table2scene
from src.Utils.misc import (
    GRIPPER_ID,
    ROBOT_IP,
    get_value,
    move_gripper,
    set_values,
)


def move_arm(
    group: moveit_commander.MoveGroupCommander,
    position: Tuple[float, float, float],
    orientation: Tuple[float, float, float, float],
) -> None:
    """Plan and execute a tool0 pose; raise if planning or execution fails."""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = position
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation

    # Plan from the current arm state to the requested tool pose.
    group.set_start_state_to_current_state()
    group.set_pose_target(pose, "tool0")
    try:
        success, trajectory, _, _ = group.plan()
        if not success or not trajectory.joint_trajectory.points:
            raise RuntimeError("Could not plan to the requested pose")
        if not group.execute(trajectory, wait=True):
            raise RuntimeError("Arm motion failed")
    finally:
        group.stop()
        group.clear_pose_targets()


def main(pos_from_camera_bool: bool = False) -> None:
    '''
    Args:
    pos_from_camera_bool - str to indicate if we are getting pickup pose from camera
    
    Returns:
    None
    '''
    # EASY POS FOR PICKUP BELOW
    # Position: (-0.6255004641758128, -0.15797115828552535, 0.0868659814542469)
    # Orientation: (0.5028374568493601, -0.48288946903178914, -0.49939166998252693, 0.5143736119199038)

    # PICK - object location
    # PREP - directly above pick location to avoid hitting object
    # MIDDLE - inbetweeen PICK and PLACE away from table
    # PLACE - object moving place
    
    rospy.init_node("pick_place")
    # pick up object with orientation below
    # horizontal
    PICK_ORIENTATION: Tuple[float, float, float, float] = (0.5028374568493601, -0.48288946903178914, -0.49939166998252693, 0.5143736119199038)
    if pos_from_camera_bool:
        # call service from camera node to get pos of object in Ur5e frame
        rospy.wait_for_service("/localize_object_server/get_location")

        get_location = rospy.ServiceProxy(
            "/localize_object_server/get_location", Centroid
        )

        response = get_location()
        centroid = response.centroid

        x = centroid.point.x
        y = centroid.point.y
        z = centroid.point.z
        
        PICK_POSITION: Tuple[float, float, float] = (x, y, z)

        print(f"Object in {centroid.header.frame_id}: ({x}, {y}, {z})")
    else:
        PICK_POSITION: Tuple[float, float, float] = (-0.6255004641758128, -0.15797115828552535, 0.0868659814542469)
    # Positions are tool0 coordinates in meters in MoveIt's planning frame.
    # Orientations are unit quaternions in (x, y, z, w) order.

    # directly above the object/pick location
    delta_z = 0.12 # 12cm
    PREP_POSITION: Tuple[float, float, float] = (PICK_POSITION[0], PICK_POSITION[1], PICK_POSITION[2] + delta_z)
    PREP_ORIENTATION: Tuple[float, float, float, float] = (0.5028374568493601, -0.48288946903178914, -0.49939166998252693, 0.514373611)

    # To be reached before final position
    MIDDLE_POSITION: Tuple[float, float, float] = (-0.44, -0.30, 0.20)
    MIDDLE_ORIENTATION: Tuple[float, float, float, float] = (0.5028374568493601, -0.48288946903178914, -0.49939166998252693, 0.5143736119199038)

    # drop object here
    PLACE_POSITION: Tuple[float, float, float] = (-0.40, -0.30, 0.09)
    PLACE_ORIENTATION: Tuple[float, float, float, float] = (0.5028374568493601, -0.48288946903178914, -0.49939166998252693, 0.5143736119199038)

    # Raw finger targets; speed and force come from test_gripper.py.
    OPEN_POSITION: int = 0
    CLOSE_POSITION: int = 255
    """Open, move to pick, grasp, move to place, and release in sequence."""
    moveit_commander.roscpp_initialize([])
    
    try:
        # initiate arm objects
        robot = moveit_commander.RobotCommander()
        group = moveit_commander.MoveGroupCommander("manipulator")
        scene = moveit_commander.PlanningSceneInterface(synchronous=True)

        # set toolpiece, dynamic limits and planning time limits
        group.set_end_effector_link("tool0")
        group.set_pose_reference_frame(group.get_planning_frame())
        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.1)
        group.set_planning_time(10.0)
        
        # add table and gripper to the scene
        add_table2scene(robot, scene)
        add_gripper2scene(scene, group)
        if "Table" not in scene.get_known_object_names() or not scene.get_attached_objects(["gripper_box"]):
            raise RuntimeError("Table or gripper was not added to the planning scene")

        # Select the physical gripper and check readiness before any movement.
        # sock is communication channel object
        with socket.create_connection((ROBOT_IP, 63352), timeout=3) as sock:
            sock.settimeout(3)
            
            # check for healthy gripper communications
            set_values(sock, f"SID {GRIPPER_ID}")
            if get_value(sock, "STA") != 3 or get_value(sock, "FLT") != 0:
                raise RuntimeError("Activate the gripper and clear its faults first")

            # Open before approaching the prep pose.
            move_gripper(sock, OPEN_POSITION)
            
            # check gripper reached open position
            if get_value(sock, "OBJ") != 3:
                raise RuntimeError("Gripper opening was obstructed")
            
            # move arm to prep pos and orientation
            move_arm(group=group, position=PREP_POSITION, orientation=PREP_ORIENTATION)
            
            # move arm to pick pos and orientation
            move_arm(group, PICK_POSITION, PICK_ORIENTATION)

            # Require closing contact before transporting the object.
            move_gripper(sock, CLOSE_POSITION)
            
            # check if gripper is contacted with object, eject if no contact
            if get_value(sock, "OBJ") != 2:
                raise RuntimeError("No closing contact detected; place motion cancelled")
            
            # move arm to middle location to avoid table
            move_arm(group, MIDDLE_POSITION, MIDDLE_ORIENTATION)
            
            # move arm to place location and orientation
            move_arm(group, PLACE_POSITION, PLACE_ORIENTATION)

            # Release only after the place motion succeeds.
            move_gripper(sock, OPEN_POSITION)
            
            # check if gripper is in open position
            if get_value(sock, "OBJ") != 3:
                raise RuntimeError("Gripper release was obstructed")
            rospy.loginfo("Pick-and-place sequence completed")
    finally:
        # Do not automatically release a held object if an earlier step fails.
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    
    # parse the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose-from-camera",
                        action="store_true", 
                        help="Use camera localization instead of specified pickup location if none is passed, defaults to false", 
                        )
    # remove unnecessary ros arguments
    args = parser.parse_args(rospy.myargv()[1:])
    
    # run pick and place 
    main(pos_from_camera_bool=args.pose_from_camera)
