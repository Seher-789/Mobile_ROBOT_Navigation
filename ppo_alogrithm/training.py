#!/usr/bin/env python3
#!/usr/bin/env python3
import gymnasium as gym
import rospy
import stable_baselines3 as sb3
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from PPO import TwoWheelRobotEnv

def main():
    # Initialize the ROS node
    rospy.init_node('ppo_training_node', anonymous=True)

    # Initialize the custom Gym environment without map_yaml_path
    env = TwoWheelRobotEnv()  # No map_yaml_path argument

    # Ensure the environment follows the Gym API
    check_env(env)

    # Setup PPO model
    model = PPO('MlpPolicy', env, verbose=1)

    # Train the model
    model.learn(total_timesteps=100000)

    # Save the trained model
    model.save("ppo_two_wheel_robot_with_map")

    rospy.loginfo("Training completed and model saved.")

if __name__ == '__main__':
    main()

