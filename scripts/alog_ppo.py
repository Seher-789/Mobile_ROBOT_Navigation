#!/usr/bin/env python3
import gym
from gym import spaces
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class LidarEnv(gym.Env):
    def __init__(self):
        super(LidarEnv, self).__init__()
        
        # Initialize ROS node
        rospy.init_node('lidar_env', anonymous=True)
        
        # ROS subscriber
        self.scan_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.scan_callback)
        
        # ROS publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Observation space: 360 degrees lidar range values
        self.observation_space = spaces.Box(low=0, high=100, shape=(360,), dtype=np.float32)
        
        # Action space: 4 discrete actions (e.g., move forward, backward, turn left, turn right)
        self.action_space = spaces.Discrete(4)
        
        # Internal state
        self.scan_data = np.zeros(360)
        self.action_to_twist = [
            Twist(linear=Vector3(x=0.5, y=0, z=0), angular=Vector3(x=0, y=0, z=0)),  # Move forward
            Twist(linear=Vector3(x=-0.5, y=0, z=0), angular=Vector3(x=0, y=0, z=0)), # Move backward
            Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0.5)),  # Turn left
            Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=-0.5))  # Turn right
        ]

    def scan_callback(self, data):
        points = list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True))
        distances = [np.linalg.norm([p[0], p[1], p[2]]) for p in points]
        self.scan_data = np.array(distances)

    def step(self, action):
        # Execute the action by publishing the corresponding Twist message
        twist_msg = self.action_to_twist[action]
        self.cmd_vel_pub.publish(twist_msg)

        # Simulate for a fixed amount of time (e.g., 0.1 seconds)
        rospy.sleep(0.1)
        
        # Assume some reward calculation based on the action taken
        reward = 0.0
        done = False
        
        # You can implement more complex logic here
        # For example, reward for moving forward, penalties for collisions, etc.

        return self.scan_data, reward, done, {"linear_vel": twist_msg.linear, "angular_vel": twist_msg.angular}

    def reset(self):
        # Reset the environment to an initial state
        self.scan_data = np.zeros(360)
        
        # Stop any robot motion
        self.cmd_vel_pub.publish(Twist())
        
        return self.scan_data

    def render(self, mode='human'):
        # Render the environment if needed
        pass

if __name__ == '__main__':
    # Test the environment
    env = LidarEnv()
    obs = env.reset()
    for _ in range(10):
        action = env.action_space.sample()  # Take random action
        obs, reward, done, info = env.step(action)
        print(f"Observation: {obs}, Reward: {reward}, Done: {done}, Info: {info}")
