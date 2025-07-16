#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class LidarProcessor:
    def __init__(self):
        # Subscriber
        self.lidar_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        
        # Publishers
        self.min_distance_pub = rospy.Publisher('/lidar_min_distance', Float32, queue_size=10)
        self.obstacle_warning_pub = rospy.Publisher('/obstacle_warning', String, queue_size=10)
        
        self.obstacle_threshold = 1.0  # Threshold distance in meters to detect an obstacle

    def lidar_callback(self, data):
        try:
            # Process LiDAR data
            points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
            distances = [np.linalg.norm([p[0], p[1], p[2]]) for p in points]
            min_distance = min(distances) if distances else float('inf')
            
            rospy.loginfo(f"LIDAR minimum distance: {min_distance}")
            
            # Publish the minimum distance
            self.min_distance_pub.publish(Float32(min_distance))
            
            # Obstacle detection
            if min_distance < self.obstacle_threshold:
                warning_message = f"Obstacle detected within {min_distance} meters!"
                rospy.logwarn(warning_message)
                self.obstacle_warning_pub.publish(warning_message)
        except Exception as e:
            rospy.logerr(f"Error processing LiDAR data: {e}")

if __name__ == '__main__':
    rospy.init_node('lidar_processor', anonymous=True)
    lp = LidarProcessor()
    rospy.spin()
