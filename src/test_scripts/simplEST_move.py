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

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ur5e_minimal_move", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("manipulator")

    # Remove constraints and give more time
    group.clear_path_constraints()
    group.set_planning_time(10.0)
    group.allow_replanning(True)

    # Force MoveIt to sync with current joint states
    group.set_start_state_to_current_state()

    # Get current pose
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Current pose:\n{current_pose}")

    # Small move so it's reachable
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = current_pose.position.x + 0.10
    target_pose.position.y = current_pose.position.y
    target_pose.position.z = current_pose.position.z
    target_pose.orientation = current_pose.orientation

    # Create service proxy
    rospy.wait_for_service('/compute_ik')
    compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    # Build IK request
    ik_req = GetPositionIKRequest()
    ik_req.ik_request.group_name = "manipulator"   # your MoveGroup
    ik_req.ik_request.pose_stamped = PoseStamped()
    ik_req.ik_request.pose_stamped.header.frame_id = robot.get_planning_frame()
    ik_req.ik_request.pose_stamped.pose = target_pose  # the pose you want to check
    ik_req.ik_request.timeout = rospy.Duration(0.1)   # small timeout
    ik_req.ik_request.avoid_collisions = False        # ignore collisions if just testing reachability

    # Optionally provide current joint state
    joint_state = rospy.wait_for_message('/joint_states', JointState)
    ik_req.ik_request.robot_state.joint_state = joint_state

    # Call the service
    try:
        ik_res = compute_ik(ik_req)
        if ik_res.error_code.val == ik_res.error_code.SUCCESS:
            print("IK feasible! Joint solution:", ik_res.solution.joint_state.position)
        else:
            print("IK not feasible.")
    except rospy.ServiceException as e:
        print("IK service call failed:", e)


    group.set_pose_target(target_pose)

    # Unpack tuple from plan()
    success, plan, planning_time, error_code = group.plan()
    rospy.loginfo(f"Planning time: {planning_time}")

    if not success or len(plan.joint_trajectory.points) == 0:
        rospy.logwarn("No valid plan found.")
    else:
        rospy.loginfo("Executing plan...")
        group.execute(plan, wait=True)

    group.stop()
    group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
