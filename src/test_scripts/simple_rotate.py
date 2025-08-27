#!/usr/bin/env python3
import sys, math
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur5e_rotate_tool", anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator")

# get current pose
current = group.get_current_pose().pose

# build a quaternion for a 180 deg rotation about X axis
q_rot = tf.quaternion_from_euler( math.pi, 0 , 0)

# apply that rotation on top of the existing orientation
# multiply quaternions: q_new = q_rot * q_current
q_curr = [
    current.orientation.x,
    current.orientation.y,
    current.orientation.z,
    current.orientation.w
]
q_new = tf.quaternion_multiply(q_rot, q_curr)

# create a new target pose with same position but our new orientation
target = geometry_msgs.msg.Pose()
target.position = current.position
target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = q_new

group.set_pose_target(target)
plan = group.plan()[1]
if plan.joint_trajectory.points:
    group.execute(plan, wait=True)
moveit_commander.roscpp_shutdown()