# File path: ~/Project/src/robot_second/ppo_algorithm/scripts/train_ppo.py
#!/usr/bin/env python3
import sys
import os

# Add the directory containing ppo_algorithm to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import gym
import rospy
import stable_baselines3 as sb3
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from ppo_algorithm.envs.two_wheel_robot_env import TwoWheelRobotEnv

def main():
    rospy.init_node('ppo_training_node')

    # Initialize the custom Gym environment
    map_file = '~/Project/src/robot_second/ppo_algorithm/maps/dlo_map.pcd'
    env = TwoWheelRobotEnv(map_file)

    # Check if the environment follows the Gym API
    check_env(env)

    # PPO model setup
    model = PPO('MlpPolicy', env, verbose=1)

    # Train the model
    model.learn(total_timesteps=100000)

    # Save the model
    model.save("ppo_two_wheel_robot_with_map")

    rospy.loginfo("Training completed and model saved.")

if __name__ == '__main__':
    main()
