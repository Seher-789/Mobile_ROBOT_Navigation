#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import message_filters
import csv
import os

class CombinedDataProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.lidar_sub = message_filters.Subscriber('/velodyne_points', PointCloud2)
        self.camera_sub = message_filters.Subscriber('/depth_camera/depth/image_raw/compressed', CompressedImage)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.camera_sub], 10, 0.1)
        self.ts.registerCallback(self.data_callback)

        self.min_distance_pub = rospy.Publisher('/min_distance', Float32, queue_size=10)
        self.lidar_min_distance_pub = rospy.Publisher('/lidar_min_distance', Float32, queue_size=10)
        self.camera_min_distance_pub = rospy.Publisher('/camera_min_distance', Float32, queue_size=10)
        self.obstacle_warning_pub = rospy.Publisher('/obstacle_warning', String, queue_size=10)

        self.obstacle_threshold = 1.0  # Threshold distance in meters to detect an obstacle

        log_dir = '/home/seher/Project/src/analysis_for_control'  # Change this to your desired log directory
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.lidar_log_file_path = os.path.join(log_dir, 'lidar_log.csv')
        self.combined_log_file_path = os.path.join(log_dir, 'combined_log.csv')
        self.lidar_log_file = open(self.lidar_log_file_path, 'w', newline='')
        self.combined_log_file = open(self.combined_log_file_path, 'w', newline='')
        self.lidar_csv_writer = csv.writer(self.lidar_log_file)
        self.combined_csv_writer = csv.writer(self.combined_log_file)
        self.lidar_csv_writer.writerow(['timestamp', 'min_distance'])
        self.combined_csv_writer.writerow(['timestamp', 'min_distance'])

    def data_callback(self, lidar_data, camera_data):
        try:
            points = list(pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True))
            lidar_min_distance = min(np.linalg.norm([p[0], p[1], p[2]]) for p in points)

            np_arr = np.frombuffer(camera_data.data, np.uint8)
            depth_image = self.bridge.compressed_imgmsg_to_cv2(camera_data, desired_encoding="passthrough")
            depth_image = np.where(np.isnan(depth_image), np.inf, depth_image)
            depth_image[depth_image == 0] = np.inf
            camera_min_distance = np.min(depth_image)

            min_distance = min(lidar_min_distance, camera_min_distance)

            rospy.loginfo(f"LIDAR minimum distance: {lidar_min_distance}")
            rospy.loginfo(f"Camera minimum distance: {camera_min_distance}")
            rospy.loginfo(f"Combined minimum distance: {min_distance}")

            self.lidar_min_distance_pub.publish(Float32(lidar_min_distance))
            self.camera_min_distance_pub.publish(Float32(camera_min_distance))
            self.min_distance_pub.publish(Float32(min_distance))

            self.lidar_csv_writer.writerow([rospy.Time.now().to_sec(), lidar_min_distance])
            self.combined_csv_writer.writerow([rospy.Time.now().to_sec(), min_distance])

            if min_distance < self.obstacle_threshold:
                warning_message = f"Obstacle detected within {min_distance} meters!"
                rospy.logwarn(warning_message)
                self.obstacle_warning_pub.publish(warning_message)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing data: {e}")

    def __del__(self):
        self.lidar_log_file.close()
        self.combined_log_file.close()

if __name__ == '__main__':
    rospy.init_node('combined_data_processor', anonymous=True)
    dp = CombinedDataProcessor()
    rospy.spin()
