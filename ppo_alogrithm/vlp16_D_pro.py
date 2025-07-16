#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class VelodyneProcessingNode:
    def __init__(self):
        self.pub = rospy.Publisher('/processed_velodyne_data', Float32MultiArray, queue_size=10)
        rospy.Subscriber("/velodyne_points", PointCloud2, self.velodyne_callback)
    
    def velodyne_callback(self, data):
        point_list = []
        for point in pc2.read_points(data, skip_nans=True):
            point_list.append([point[0], point[1], point[2]])
        
        point_array = np.array(point_list)
        self.process_point_cloud(point_array)
    
    def process_point_cloud(self, points):
        if points.size > 0:
            centroid = np.mean(points, axis=0)
            rospy.loginfo(f"Centroid of point cloud: {centroid}")
            
            msg = Float32MultiArray()
            msg.data = centroid
            self.pub.publish(msg)
        else:
            rospy.logwarn("No points received in the point cloud")

def main():
    rospy.init_node('velodyne_processing_node', anonymous=True)
    node = VelodyneProcessingNode()
    rospy.spin()

if __name__ == '__main__':
    main()
