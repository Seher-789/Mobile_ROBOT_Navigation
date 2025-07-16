#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import csv
import os

class LidarProcessor:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        self.min_distance_pub = rospy.Publisher('/lidar_min_distance', Float32, queue_size=10)
        self.obstacle_warning_pub = rospy.Publisher('/obstacle_warning', String, queue_size=10)
        self.obstacle_threshold = 1.0  # Threshold distance in meters to detect an obstacle

        log_dir = '/path/to/your/logs'  # Change this to your desired log directory
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        self.log_file_path = os.path.join(log_dir, 'lidar_log.csv')
        self.log_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['timestamp', 'min_distance'])

    def lidar_callback(self, data):
        try:
            points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
            distances = [np.linalg.norm([p[0], p[1], p[2]]) for p in points]
            min_distance = min(distances) if distances else float('inf')

            rospy.loginfo(f"LIDAR minimum distance: {min_distance}")
            self.min_distance_pub.publish(Float32(min_distance))
            self.csv_writer.writerow([rospy.Time.now().to_sec(), min_distance])

            if min_distance < self.obstacle_threshold:
                warning_message = f"Obstacle detected within {min_distance} meters!"
                rospy.logwarn(warning_message)
                self.obstacle_warning_pub.publish(warning_message)
        except Exception as e:
            rospy.logerr(f"Error processing LiDAR data: {e}")

    def __del__(self):
        self.log_file.close()

if __name__ == '__main__':
    rospy.init_node('lidar_processor', anonymous=True)
    lp = LidarProcessor()
    rospy.spin()
