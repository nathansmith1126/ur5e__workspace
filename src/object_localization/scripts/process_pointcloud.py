#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

def remove_points(pcd, threshold, axis=2, keep_below=True):
    """
    Filter a point cloud by a threshold along a specified axis.

    Args:
        pcd (open3d.geometry.PointCloud): input point cloud
        threshold (float): threshold value along the chosen axis
        axis (int): axis index (0=x, 1=y, 2=z). Default z-axis.
        keep_below (bool): if True, keep points with axis value <= threshold
                           (i.e. remove 'far' points with axis > threshold).
                           If False, keep points with axis value >= threshold.

    Returns:
        open3d.geometry.PointCloud: filtered point cloud (may be empty)
    """
    if pcd is None:
        return None

    points = np.asarray(pcd.points)

    if points.size == 0:
        return pcd  # nothing to filter

    if axis < 0 or axis > 2:
        raise ValueError("axis must be 0, 1 or 2")

    if keep_below:
        mask = points[:, axis] <= threshold
    else:
        mask = points[:, axis] >= threshold

    indices = np.where(mask)[0]
    print(f"Points before filtering: {len(points)}, after filtering: {len(indices)}")
    return pcd.select_by_index(indices)


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

def DBSCAN(pcd, eps=0.02, visualize=False):
    """
    Apply DBSCAN clustering to the point cloud and draw.

    Args:
        pcd (open3d.geometry.PointCloud): The input point cloud.

    """
    
    print("Segmenting objects using DBSCAN...")
    
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=50))
    max_label = labels.max()
    if max_label == -1:
        return pcd.select_by_index([])  # No clusters found
    
    # Count points in each cluster
    unique_labels, counts = np.unique(labels[labels != -1], return_counts=True)
    
    # Identify largest cluster
    if len(unique_labels) > 0:
        largest_cluster_label = unique_labels[np.argmax(counts)]
    else:
        print("No clusters found (only noise or empty point cloud).")
        largest_cluster_label = None
        
    # Extract largest cluster
    if largest_cluster_label is not None:
        largest_cluster_indices = np.where(labels == largest_cluster_label)[0]
        largest_cluster_pcd = pcd.select_by_index(largest_cluster_indices)
    else:
        largest_cluster_pcd = open3d.geometry.PointCloud() # Empty point cloud

    # Plot segmentations
    if visualize:
        print("Plotting segmented pointcloud...")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = open3d.utility.Vector3dVector(colors[:, :3])
        open3d.visualization.draw_geometries([pcd])
        
    return largest_cluster_pcd


# Server node implementation
import threading
from std_srvs.srv import Trigger, TriggerResponse
from object_localization.srv import Centroid, CentroidResponse
from geometry_msgs.msg import PointStamped
import tf

class GetCentroidServer:
    def __init__(self):
        rospy.init_node('localize_server')

        # subscriber topic name
        self.topic_name = rospy.get_param('~pointcloud_topic', 'camera/depth/points')
        self.received_ros_cloud = None
        
        # frame transformation
        self.frame_in = rospy.get_param('~pcl_frame', 'camera_color_optical_frame')
        self.frame_out = rospy.get_param('~robot_frame', 'base_link')
        self.tf_listener = tf.TransformListener()

        self.lock = threading.Lock()
        # Create a persistent subscriber so we continuously buffer the latest pointcloud
        self.subscriber = rospy.Subscriber(self.topic_name, PointCloud2, self.callback)
        self.service = rospy.Service('~get_location', Centroid, self.handle_request)
        rospy.loginfo("LocalizeServer ready. Subscribed to %s and service available at ~get_location.", self.topic_name)

    def callback(self, ros_cloud):
        # Always update the buffered latest pointcloud. Do not unregister the subscriber;
        # keep receiving messages so subsequent service calls can use the latest data.
        with self.lock:
            self.received_ros_cloud = ros_cloud
            # Optionally store a timestamp if needed in the future
            # self.last_cloud_time = rospy.Time.now()
        rospy.logdebug("Buffered latest PointCloud2 message.")

    def handle_request(self, req):
        # Clear any previous buffered cloud so we wait for a fresh message, but keep the
        # persistent subscriber active (it was created in __init__).
        with self.lock:
            self.received_ros_cloud = None
        rospy.loginfo("Waiting for next PointCloud2 message (subscriber is persistent)...")

        timeout = rospy.get_param('~timeout', 5.0)
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(10)

        # -- Server response
        point = PointStamped()
        point.header.frame_id = self.frame_out
        response = CentroidResponse()

        # Wait for a new buffered cloud
        while not rospy.is_shutdown():
            with self.lock:
                if self.received_ros_cloud is not None:
                    ros_cloud = self.received_ros_cloud
                    break
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn("Timeout waiting for pointcloud message.")
                point.point.x = point.point.y = point.point.z = 0.0
                point.header.stamp = rospy.Time(0)
                response.centroid = point
                return response

            rate.sleep()

        try:
            o3d_cloud = convertCloudFromRosToOpen3d(ros_cloud)
            rospy.loginfo("Converted successfully.")

            # Remove far points
            processed_cloud_far = remove_points(o3d_cloud, threshold=1.0)
            rospy.loginfo("Removed far points.")

            if processed_cloud_far is None:
                processed_cloud_far = o3d_cloud

            # Remove near points
            processed_cloud = remove_points(processed_cloud_far, threshold=0.2, keep_below=False)
            rospy.loginfo("Removed near points.")

            if processed_cloud is None:
                processed_cloud = processed_cloud_far

            # Remove planes using RANSAC
            processed_cloud_ransac = RANSAC(processed_cloud, 1)
            rospy.loginfo("Removed planes using RANSAC.")
            if processed_cloud_ransac is None:
                processed_cloud_ransac = processed_cloud

            # Segment objects and plot using DBSCAN
            object_pcd = DBSCAN(processed_cloud_ransac, 0.02)
            rospy.loginfo("Segmented object using DBSCAN.")

            # -- Return centroid to the client
            if object_pcd is not None and len(np.asarray(object_pcd.points)) > 0:
                # Get centroid
                centroid = object_pcd.get_center()

                # Centroid in camera frame
                point_camera_frame = PointStamped()
                point_camera_frame.header.frame_id = self.frame_in
                point_camera_frame.point.x = float(centroid[0])
                point_camera_frame.point.y = float(centroid[1])
                point_camera_frame.point.z = float(centroid[2])
                
                # Wait for the transform to become available
                # This is good practice to avoid exceptions if the transform isn't available immediately
                try:
                    self.tf_listener.waitForTransform(self.frame_out, self.frame_in, rospy.Time(0), rospy.Duration(4.0))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("Transform unavailable: %s", e)
                    return
                
                # Transform the point
                try:
                    point_robot_frame = self.tf_listener.transformPoint(self.frame_out, point_camera_frame)
                    point_robot_frame.header.stamp = rospy.Time(0)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("Transformation failed: %s", e)
                
                # return reponse
                response.centroid = point_robot_frame

                rospy.loginfo(f"Centroid returned (x: {point_robot_frame.point.x}, y: {point_robot_frame.point.y}, z: {point_robot_frame.point.z})")

                return response

            # If no object found, return zeros
            point.point.x = point.point.y = point.point.z = 0.0
            point.header.stamp = rospy.Time(0)
            response.centroid = point
            return response

        except Exception as e:
            rospy.logerr(f"Error processing pointcloud: {e}")
            point.point.x = point.point.y = point.point.z = 0.0
            point.header.stamp = rospy.Time(0)
            response.centroid = point
            return response

if __name__ == "__main__":
    server = GetCentroidServer()
    rospy.spin()