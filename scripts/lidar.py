#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math

def spherical_to_cartesian(range, elevation, azimuth, extrinsic_params):
    """
    Converts a spherical point (range, elevation, azimuth) to a Cartesian point (X, Y, Z).

    Args:
        range: Distance to the point from the sensor origin (meters).
        elevation: Angle above the sensor's horizontal plane (radians).
        azimuth: Angle from the sensor's positive X-axis (radians).
        extrinsic_params: A 4x4 homogeneous transformation matrix representing the sensor's pose.

    Returns:
        A tuple containing the X, Y, and Z coordinates of the point in the reference frame.
    """

    x = range * math.cos(elevation) * math.cos(azimuth)
    y = range * math.cos(elevation) * math.sin(azimuth)
    z = range * math.sin(elevation)

    # Apply extrinsic transformation (homogeneous transformation multiplication)
    transformed_point = np.matmul(extrinsic_params, np.array([x, y, z, 1.0]))

    return transformed_point[:3]  # Return only the first three elements (X, Y, Z)

def lidar_cb(cloud_msg):
    # Extract necessary information from PointCloud2 message
    points = pc2.read_points(cloud_msg, skip_nans=True)
    extrinsic_params = np.eye(4)  # Example: identity matrix (no transformation)

    # Process each point
    for point in points:
        range = point[0]  # Distance to the point from the sensor origin
        elevation = point[1]  # Angle above the sensor's horizontal plane
        azimuth = point[2]  # Angle from the sensor's positive X-axis

        # Convert spherical coordinates to Cartesian coordinates
        x, y, z = spherical_to_cartesian(range, elevation, azimuth, extrinsic_params)

        # Process the Cartesian point (e.g., store it, perform further calculations)
        print("X:", x, "Y:", y, "Z:", z)

if __name__ == '__main__':
    node_name = "lidar_data_process"
    rospy.init_node(node_name)
    sub_topic_name = "/velodyne_points"
    rospy.Subscriber(sub_topic_name, PointCloud2, lidar_cb)
    rospy.spin()










# import sensor_msgs.msg as sensor_msgs
# from sensor_msgs.msg import PointField  # Import PointField for clarity
# from sensor_msgs.msg import PointCloud2  # Explicit import for readability
# import pcl
# import rospy
# import velodyne_pcl

# def cloud_callback(cloud_msg):
#     """
#     Converts a sensor_msgs/PointCloud2 message to a pcl.PointCloud object.

#     Args:
#         cloud_msg (sensor_msgs.msg.PointCloud2): The PointCloud2 message to convert.

#     Returns:
#         pcl.PointCloud: The converted PCL PointCloud object.
#     """

#     # Check if the message format matches the expected Velodyne data
#     if cloud_msg.fields[0].name != 'x' or \
#        cloud_msg.fields[1].name != 'y' or \
#        cloud_msg.fields[2].name != 'z' or \
#        cloud_msg.fields[3].name != 'intensity' or \
#        cloud_msg.fields[4].name != 'ring' or \
#        cloud_msg.fields[5].name != 'time':
#         raise ValueError("Unexpected field names in PointCloud2 message. "
#                          "Ensure it matches Velodyne PointXYZIRT format.")

#     # Convert the PointCloud2 message to a PCL PointCloud2 object
#     pcl_pc2 = pcl.PointCloud2()
#     pcl.from_sensor_msgs(cloud_msg, pcl_pc2)

#     # Create a PCL PointCloud object with the appropriate point type
#     pcl_cloud = pcl.PointCloud(velodyne_pcl.PointXYZIRT)

#     # Convert the PCL PointCloud2 object to a PCL PointCloud object
#     pcl.from_PCLPointCloud2(pcl_pc2, pcl_cloud)

#     return pcl_cloud

# # Example usage (assuming you have a ROS subscriber to the PointCloud2 topic)
# def main():
#     # Replace with your actual subscriber setup
#     # ...
#     subscriber = rospy.Subscriber("/velodyne_points", PointCloud2, cloud_callback)
#     rospy.spin()

# if __name__ == "__main__":
#     main()







