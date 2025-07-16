#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Float32
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
        self.lidar_min_distance_pub = rospy.Publisher('/lidar_min_distance', Float32, queue_size=10)
        self.camera_min_distance_pub = rospy.Publisher('/camera_min_distance', Float32, queue_size=10)

        self.min_distance = float('inf')
        self.lidar_min_distance = float('inf')
        self.camera_min_distance = float('inf')

    def data_callback(self, lidar_data, camera_data):
        try:
            # Process Lidar data
            points = list(pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True))
            self.lidar_min_distance = min(np.linalg.norm([p[0], p[1], p[2]]) for p in points)

            # Process compressed camera data
            np_arr = np.frombuffer(camera_data.data, np.uint8)
            depth_image = self.bridge.compressed_imgmsg_to_cv2(camera_data, desired_encoding="passthrough")

            # Handle NaN values in depth_image
            depth_image = np.where(np.isnan(depth_image), np.inf, depth_image)
            depth_image[depth_image == 0] = np.inf  # Replace 0 with inf to avoid using them in min calculation
            self.camera_min_distance = np.min(depth_image)  # Use min to get the smallest distance

            # Calculate the minimum distance
            self.min_distance = max(self.lidar_min_distance, self.camera_min_distance)
            rospy.loginfo(f"LIDAR minimum distance: {self.lidar_min_distance}")
            rospy.loginfo(f"Camera minimum distance: {self.camera_min_distance}")
            rospy.loginfo(f"Combined minimum distance: {self.min_distance}")

            # Publish the minimum distances
            self.lidar_min_distance_pub.publish(Float32(self.lidar_min_distance))
            self.camera_min_distance_pub.publish(Float32(self.camera_min_distance))
            self.min_distance_pub.publish(Float32(self.min_distance))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

    def get_min_distance(self):
        return self.min_distance

if __name__ == '__main__':
    rospy.init_node('combined_data_processor', anonymous=True)
    rospy.set_param('/use_sim_time', True)  # Ensure that simulated time is being used
    dp = CombinedDataProcessor()
    rospy.spin()

