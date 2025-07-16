#!/usr/bin/env python3

import gym
from gym import spaces
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
from stable_baselines3 import PPO
from sb3_contrib.ppo_mask import MaskablePPO  # Use MaskablePPO

class YourLidarEnv(gym.Env):
    def __init__(self):
        super(YourLidarEnv, self).__init__()
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32)
        
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

def main():
    env = YourLidarEnv()
    model = MaskablePPO('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=100000)
    model.save("models/ppo_lidar")

if __name__ == '__main__':
    main()
