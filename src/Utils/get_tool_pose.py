'''
Run this from ur5e_ws to get a print out of toolpiece pose 
(position and orientation)

python3 -m src.Utils.get_tool_pose
'''

import moveit_commander
import rospy

moveit_commander.roscpp_initialize([])
rospy.init_node("read_tool_pose", anonymous=True)

try:
    group = moveit_commander.MoveGroupCommander("manipulator")
    current = group.get_current_pose("tool0")

    p = current.pose.position
    q = current.pose.orientation

    print("Frame:", current.header.frame_id)
    print("Position (meters):", (p.x, p.y, p.z))
    print("Quaternion (x, y, z, w):", (q.x, q.y, q.z, q.w))
finally:
    moveit_commander.roscpp_shutdown()