#!/usr/bin/env python
import rospy
import csv
import os
from apriltag_ros.msg import AprilTagDetectionArray

# Output file
OUT_CSV = os.path.expanduser("~/tag_detections.csv")

def cb(msg):
    global frame_id
    with open(OUT_CSV, "a") as f:
        writer = csv.writer(f)
        for det in msg.detections:
            t = det.pose.pose.pose.position
            q = det.pose.pose.pose.orientation
            writer.writerow([
                frame_id,
                det.id[0],
                t.x, t.y, t.z,
                q.x, q.y, q.z, q.w
            ])
    frame_id += 1

if __name__ == "__main__":
    rospy.init_node("tag_dump_csv")
    frame_id = 0
    with open(OUT_CSV, "w") as f:
        writer = csv.writer(f)
        writer.writerow(["frame_id","tag_id","tx","ty","tz","qx","qy","qz","qw"])
    sub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, cb, queue_size=10)
    rospy.spin()
