#!/usr/bin/env python3

import rospy
import gym
from stable_baselines3 import PPO
from sb3_contrib.ppo_mask import MaskablePPO
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class YourLidarEnv(gym.Env):
    def __init__(self):
        super(YourLidarEnv, self).__init__()
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        
        rospy.init_node('lidar_env', anonymous=True)
        rospy.Subscriber("/processed_velodyne_data", Float32MultiArray, self.velodyne_callback)
        
        self.processed_data = np.zeros(3)

    def velodyne_callback(self, data):
        self.processed_data = np.array(data.data)
    
    def step(self, action):
        obs = self._get_obs()
        reward = self._compute_reward(obs)
        done = self._is_done(obs)
        info = {}
        return obs, reward, done, info

    def reset(self):
        return self._get_obs()

    def _get_obs(self):
        return self.processed_data

    def _compute_reward(self, obs):
        return 0.0  # Replace with actual reward calculation

    def _is_done(self, obs):
        return False  # Replace with actual termination condition

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.env = YourLidarEnv()
        self.model = MaskablePPO.load("models/ppo_lidar")

    def run(self):
        obs = self.env.reset()
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            action, _states = self.model.predict(obs, deterministic=True)
            self.send_action(action)
            obs, rewards, dones, info = self.env.step(action)
            rate.sleep()

    def send_action(self, action):
        twist = Twist()
        twist.linear.x = action[0]
        twist.angular.z = action[1]
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    nav_node = NavigationNode()
    nav_node.run()
