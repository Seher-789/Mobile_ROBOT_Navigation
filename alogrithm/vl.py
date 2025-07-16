#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

class DepthCameraProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/depth_camera/depth/image_raw/compressed", CompressedImage, self.callback)
        self.min_dist_pub = rospy.Publisher("/Depth_camera_min_obstacle_distance", Float32, queue_size=10)

    def callback(self, data):
        # Convert the compressed image to an OpenCV image
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        
        # Process the depth image to find the minimum distance
        min_distance = self.process_depth_image(cv_image)
        
        # Publish the minimum distance
        self.min_dist_pub.publish(min_distance)
    
    def process_depth_image(self, cv_image):
        # Assuming the depth image is a single-channel (grayscale) image with depth values in meters
        min_distance = np.min(cv_image)
        return Float32(min_distance)

if __name__ == "__main__":
    rospy.init_node('depth_camera_processor', anonymous=True)
    dcp = DepthCameraProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")