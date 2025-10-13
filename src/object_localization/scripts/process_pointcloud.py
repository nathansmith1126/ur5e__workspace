#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script contains 2 functions for converting cloud format between Open3D and ROS:   
* convertCloudFromOpen3dToRos  
* convertCloudFromRosToOpen3d
where the ROS format refers to "sensor_msgs/PointCloud2.msg" type.

This script also contains a test case, which does such a thing:  
(1) Read a open3d_cloud from .pcd file by Open3D.
(2) Convert it to ros_cloud.
(3) Publish ros_cloud to topic.
(4) Subscribe the ros_cloud from the same topic.
(5) Convert ros_cloud back to open3d_cloud.
(6) Display it.  
You can test this script's function by rosrun this script.

'''

import open3d
import numpy as np
from ctypes import * # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # Set "header"
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    # Set "fields" and "cloud_data"
    points=np.asarray(open3d_cloud.points)
    if not open3d_cloud.colors: # XYZ only
        fields=FIELDS_XYZ
        cloud_data=points
    else: # XYZ + RGB
        fields=FIELDS_XYZRGB
        # -- Change rgb color from "three float" to "one 24-byte int"
        # 0x00FFFFFF is white, 0x00000000 is black.
        colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
        colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        cloud_data=np.c_[points, colors]
    
    # create ros_cloud
    return pc2.create_cloud(header, fields, cloud_data)


def convertCloudFromRosToOpen3d(ros_cloud):
    print("Converting ROS PointCloud2 to Open3D format...")
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud

def remove_far_points(pcd, threshold):
    print("Removing far points...")
    points = np.asarray(pcd.points)
    mask = np.linalg.norm(points, axis=1) < threshold
    pcd_filtered = pcd.select_by_index(np.where(mask)[0])
    return pcd_filtered


def RANSAC(pcd, max_plane_idx = 5):
    """
    Remove all planar regions in the point cloud using RANSAC.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.

    Returns:
        open3d.geometry.PointCloud: The point cloud with all planar regions removed.
    """

    print("Removing planes using RANSAC...")

    segment_models={}
    segments={}
    
    rest=pcd
    d_threshold=0.01
    
    for i in range(max_plane_idx):
        segment_models[i], inliers = rest.segment_plane(distance_threshold=d_threshold,ransac_n=3,num_iterations=1000)
        
        if len(inliers) < 100:  # If less than 100 inliers, stop the process
            break
        
        segments[i]=rest.select_by_index(inliers)
        
        rest = rest.select_by_index(inliers, invert=True)
        
        # labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*10, min_points=10))
        # candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
        # best_candidate=int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])

        # rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
        # segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
    
    return rest
        

import matplotlib.pyplot as plt

def DBSCAN(pcd):
    """
    Apply DBSCAN clustering to the point cloud and draw.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.

    """
    
    print("Segmenting objects using DBSCAN...")
    
    labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10))
    max_label = labels.max()
    if max_label == -1:
        return pcd.select_by_index([])  # No clusters found

    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = open3d.utility.Vector3dVector(colors[:, :3])
    open3d.visualization.draw_geometries([pcd])


# Server node implementation
import threading
from std_srvs.srv import Trigger, TriggerResponse

class PointCloudSaveServer:
    def __init__(self):
        rospy.init_node('localize_server')
        self.topic_name = rospy.get_param('~pointcloud_topic', 'camera/depth/points')
        self.output_filename = rospy.get_param('~output_filename', '/tmp/conversion_result.pcd')
        self.received_ros_cloud = None
        self.subscriber = None
        self.lock = threading.Lock()
        self.service = rospy.Service('~get_location', Trigger, self.handle_request)
        rospy.loginfo("LocalizeServer ready. Call service to get location.")

    def callback(self, ros_cloud):
        with self.lock:
            if self.received_ros_cloud is None:
                self.received_ros_cloud = ros_cloud
                rospy.loginfo("Received PointCloud2 message. Unsubscribing.")
                if self.subscriber:
                    self.subscriber.unregister()

    def handle_request(self, req):
        with self.lock:
            self.received_ros_cloud = None
        self.subscriber = rospy.Subscriber(self.topic_name, PointCloud2, self.callback)
        rospy.loginfo(f"Subscribed to {self.topic_name}, waiting for pointcloud...")
        timeout = rospy.get_param('~timeout', 5.0)
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if self.received_ros_cloud is not None:
                    break
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn("Timeout waiting for pointcloud message.")
                return TriggerResponse(success=False, message="Timeout waiting for pointcloud message.")
            rate.sleep()
        with self.lock:
            ros_cloud = self.received_ros_cloud
        try:
            o3d_cloud = convertCloudFromRosToOpen3d(ros_cloud)
            rospy.loginfo("Converted successfully.")
            # Remove far points
            processed_cloud = remove_far_points(o3d_cloud, threshold=1.0)
            rospy.loginfo("Removed far points.")
            if processed_cloud is None:
                processed_cloud = o3d_cloud
            # Remove planes using RANSAC
            processed_cloud = RANSAC(o3d_cloud)
            rospy.loginfo("Removed planes using RANSAC.")
            if processed_cloud is None:
                processed_cloud = o3d_cloud
            # Segment objects and plot using DBSCAN
            DBSCAN(processed_cloud)
            rospy.loginfo("Plotted segmented pointcloud.")
            return TriggerResponse(success=True, message="Plotted segmented pointcloud.")
        except Exception as e:
            rospy.logerr(f"Error processing pointcloud: {e}")
            return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    server = PointCloudSaveServer()
    rospy.spin()