#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from std_msgs.msg import Header

def pointcloud2_to_array(cloud_msg):
    # Convert PointCloud2 to array
    points_list = []

    for data in pc2.read_points(cloud_msg, skip_nans=True):
        points_list.append([data[0], data[1], data[2]])

    return np.array(points_list, dtype=np.float32)

def array_to_pointcloud2(points, frame_id="base_link"):
    # Convert numpy array to PointCloud2
    header = Header(frame_id=frame_id)
    return pc2.create_cloud_xyz32(header, points)

def crop_pointcloud(pointcloud, distance_threshold):
    # Crop the point cloud based on distance threshold
    # print(type(pointcloud.points))
    # print((pointcloud.points))
    idx = np.where(np.linalg.norm(pointcloud.points,axis=1) > distance_threshold)[0]
    # print(len(idx[0]))
    cropped_cloud = pointcloud.select_by_index(  idx    
    )
    return cropped_cloud

def pointcloud_callback(cloud_msg):
    # Callback for PointCloud2 messages
    pointcloud_np = pointcloud2_to_array(cloud_msg)
    pointcloud_o3d = o3d.geometry.PointCloud()
    pointcloud_o3d.points = o3d.utility.Vector3dVector(pointcloud_np)

    # Crop the point cloud
    cropped_cloud = crop_pointcloud(pointcloud_o3d, distance_threshold=1) # Example threshold

    # Convert cropped cloud to PointCloud2 and publish
    cropped_cloud_np = np.asarray(cropped_cloud.points)
    cropped_cloud_msg = array_to_pointcloud2(cropped_cloud_np, frame_id=cloud_msg.header.frame_id)
    cropped_cloud_msg.header.frame_id ="os_sensor"
    pub.publish(cropped_cloud_msg)

if __name__ == '__main__':
    rospy.init_node('pointcloud_processor', anonymous=True)
    print("Start PCL Filter")
    # Subscriber
    rospy.Subscriber("/ouster/points", PointCloud2, pointcloud_callback)
    
    # Publisher
    pub = rospy.Publisher("/ouster/points_filtered", PointCloud2, queue_size=10)

    rospy.spin()
