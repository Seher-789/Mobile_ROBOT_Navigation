# File path: ~/Project/src/robot_second/ppo_algorithm/envs/two_wheel_robot_env.py

import gym
import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Twist
from gym import spaces
import open3d as o3d
import random
import matplotlib.pyplot as plt

class TwoWheelRobotEnv(gym.Env):
    def __init__(self, map_file):
        super(TwoWheelRobotEnv, self).__init__()

        # ROS Setup
        rospy.init_node('two_wheel_robot_env', anonymous=True)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Load the map
        self.map = self.load_map(map_file)
        self.map_resolution = 0.05  # Set the resolution of your map (meters per pixel)
        self.map_origin = np.array([-10.0, -10.0])  # Example origin, adjust as necessary

        # Define observation and action spaces
        laser_low = 0.0
        laser_high = 10.0
        self.observation_space = spaces.Box(low=laser_low, high=laser_high, shape=(360,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        # Robot State
        self.pose = Pose()
        self.scan_data = np.zeros(360)
        self.goal_position = self.set_goal()

    def load_map(self, map_file):
        pcd = o3d.io.read_point_cloud(map_file)
        points = np.asarray(pcd.points)
        max_points = np.max(points, axis=0)
        min_points = np.min(points, axis=0)
        
        # Discretize the map into a grid (Occupancy Grid)
        grid_size = 512  # Define the grid size (number of pixels)
        map_grid = np.zeros((grid_size, grid_size))
        
        for point in points:
            x_idx = int((point[0] - min_points[0]) / (max_points[0] - min_points[0]) * grid_size)
            y_idx = int((point[1] - min_points[1]) / (max_points[1] - min_points[1]) * grid_size)
            map_grid[y_idx, x_idx] = 1
        
        return map_grid

    def set_goal(self):
        free_space = np.argwhere(self.map == 0)
        goal_idx = random.choice(free_space)
        goal_position = self.grid_to_world(goal_idx)
        return goal_position

    def grid_to_world(self, grid_position):
        x = grid_position[1] * self.map_resolution + self.map_origin[0]
        y = grid_position[0] * self.map_resolution + self.map_origin[1]
        return np.array([x, y])

    def world_to_grid(self, world_position):
        x_idx = int((world_position[0] - self.map_origin[0]) / self.map_resolution)
        y_idx = int((world_position[1] - self.map_origin[1]) / self.map_resolution)
        return np.array([y_idx, x_idx])

    def odom_callback(self, data):
        self.pose = data.pose.pose

    def scan_callback(self, data):
        self.scan_data = np.array(data.ranges)

    def step(self, action):
        # Apply action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)

        # Wait for the action to be applied
        rospy.sleep(0.1)

        # Get observations
        observation = self.scan_data

        # Compute reward
        reward = self.compute_reward()

        # Check if episode is done
        done = self.is_done()

        return observation, reward, done, {}

    def reset(self):
        # Reset robot position and goal
        self.pose = Pose()  # Reset to starting position or random valid position
        self.goal_position = self.set_goal()  # Set a new goal
        observation = self.scan_data
        return observation

    def compute_reward(self):
        # Distance to goal
        current_position = np.array([self.pose.position.x, self.pose.position.y])
        distance_to_goal = np.linalg.norm(current_position - self.goal_position)
        
        # Reward based on proximity to goal
        reward = -distance_to_goal
        
        # Penalize for collision (e.g., using laser scan data)
        if np.min(self.scan_data) < 0.2:  # Threshold for collision
            reward -= 100

        return reward

    def is_done(self):
        # Terminate the episode if the robot reaches the goal or collides
        current_position = np.array([self.pose.position.x, self.pose.position.y])
        distance_to_goal = np.linalg.norm(current_position - self.goal_position)

        if distance_to_goal < 0.5:  # Threshold for reaching the goal
            return True
        if np.min(self.scan_data) < 0.2:  # Collision detected
            return True

        return False

    def render(self, mode='human'):
        # Visualize the map, robot position, and goal
        plt.imshow(self.map, cmap='gray', origin='lower')
        current_position = self.world_to_grid([self.pose.position.x, self.pose.position.y])
        goal_position = self.world_to_grid(self.goal_position)
        plt.plot(current_position[1], current_position[0], 'bo')  # Robot position in blue
        plt.plot(goal_position[1], goal_position[0], 'ro')  # Goal position in red
        plt.show()

    def close(self):
        rospy.signal_shutdown('Closing environment')

# Registering the environment with Gym
from gym.envs.registration import register
register(id='TwoWheelRobot-v0', entry_point='ppo_algorithm.envs:TwoWheelRobotEnv')
