#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters

class CombinedDataProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        
        # Subscribers
        self.lidar_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
        self.camera_sub = message_filters.Subscriber('/depth_camera/depth/image_raw/compressed', CompressedImage)
        
        # Synchronizer
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub], 10, 0.1)
        self.ts.registerCallback(self.data_callback)
        
        # Publishers
        self.min_distance_pub = rospy.Publisher('/min_distance', Float32, queue_size=10)
        self.obstacle_warning_pub = rospy.Publisher('/obstacle_warning', String, queue_size=10)

        self.obstacle_threshold = 1.0  # Threshold distance in meters to detect an obstacle

    def data_callback(self, lidar_data, camera_data):
        try:
            # Process LiDAR data
            points = list(pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True))
            lidar_distances = [np.linalg.norm([p[0], p[1], p[2]]) for p in points]
            lidar_min_distance = min(lidar_distances) if lidar_distances else float('inf')

            # Process compressed camera data
            depth_image = self.bridge.compressed_imgmsg_to_cv2(camera_data, desired_encoding="passthrough")
            # Handle NaN values in depth_image
            depth_image = np.where(np.isnan(depth_image), np.inf, depth_image)
            depth_image[depth_image == 0] = np.inf
            camera_min_distance = np.min(depth_image)

            # Combine data: Calculate the fused distance (for demonstration purposes, using the average)
            combined_min_distance = (lidar_min_distance + camera_min_distance) / 2

            rospy.loginfo(f"LIDAR minimum distance: {lidar_min_distance}")
            rospy.loginfo(f"Camera minimum distance: {camera_min_distance}")
            rospy.loginfo(f"Combined minimum distance: {combined_min_distance}")

            # Publish the minimum distance
            self.min_distance_pub.publish(Float32(combined_min_distance))

            # Obstacle detection
            if combined_min_distance < self.obstacle_threshold:
                warning_message = f"Obstacle detected within {combined_min_distance} meters!"
                rospy.logwarn(warning_message)
                self.obstacle_warning_pub.publish(warning_message)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

if __name__ == '__main__':
    rospy.init_node('combined_data_processor', anonymous=True)
    rospy.set_param('/use_sim_time', True)  # Ensure that simulated time is being used
    dp = CombinedDataProcessor()
    rospy.spin()
