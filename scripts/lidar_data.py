#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
class lidar_check:
   def __init__(self): 
    sub_topic_name="/velodyne_points"
    self.lidar_subscriber=rospy.Subscriber(sub_topic_name,PointCloud2, self.lidar_cb)
   def lidar_cb(self,cloud_msg):
        container_type = cloud_msg.fields[0].datatype
        field_names = [field.name for field in cloud_msg.fields]

        # Check if required fields (x, y, z) are present
        assert 'x' in field_names and 'y' in field_names and 'z' in field_names, "Required fields (x, y, z) not found in PointCloud2 message"

        # Get indices of x, y, and z fields
        x_idx = field_names.index('x')
        y_idx = field_names.index('y')
        z_idx = field_names.index('z')

        # Read points and extract data
        points = list(pc2.read_points(cloud_msg, skip_nans=True))  # Skip NaN values (optional)
        x = [point[x_idx] for point in points]
        y = [point[y_idx] for point in points]
        z = [point[z_idx] for point in points]
        a=np.array(x)
        b=np.array(y)
        c=np.array(z)
        print("x:", a.shape, x[:5])  # Print first 5 elements
        print("y:", b.shape, y[:5])
        print("z:", c.shape, z[:5])
if __name__=='__main__':
   node_name="lidar_data_process"
   rospy.init_node(node_name)
   lidar_check()
   rospy.spin()
            
     
