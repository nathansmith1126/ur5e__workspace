#!/usr/bin/env python3
import sys, yaml, rospy
import moveit_commander
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("save_load_pose", anonymous=True)
group = moveit_commander.MoveGroupCommander("manipulator")

# 1) Get current pose
pose = group.get_current_pose().pose

# 2) Convert to dict
pose_dict = {
    'position': {
        'x': pose.position.x,
        'y': pose.position.y,
        'z': pose.position.z
    },
    'orientation': {
        'x': pose.orientation.x,
        'y': pose.orientation.y,
        'z': pose.orientation.z,
        'w': pose.orientation.w
    }
}

# 3) Save to file
with open('saved_pose.yaml', 'w') as f:
    yaml.dump(pose_dict, f)

moveit_commander.roscpp_shutdown()