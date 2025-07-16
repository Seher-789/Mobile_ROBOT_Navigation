#!/usr/bin/env python3
import math
import gymnasium as gym
import numpy as np
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, Twist
from gymnasium import spaces
import random
import matplotlib.pyplot as plt


class TwoWheelRobotEnv(gym.Env):
    def __init__(self, seed_value=42):
        super(TwoWheelRobotEnv, self).__init__()

        # ROS Publishers and Subscribers
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/velodyne_points', PointCloud2, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # Set the seed for reproducibility
        self.seed_value = seed_value
        self.seed(seed_value)

        # Robot state
        self.pose = Pose()
        self.map = None
        self.map_resolution = 0.05  # Default resolution; will be set from the map
        self.map_origin = np.array([0.0, 0.0])  # Default origin; will be set from the map
        self.goal_position = None  # Will be set after receiving map data

        # Define gaps and environment dimensions
        self.gaps = [(-math.pi, -math.pi/2), (-math.pi/2, 0), (0, math.pi/2), (math.pi/2, math.pi)]
        self.environment_dim = len(self.gaps)

        # Define observation and action spaces
        laser_low = 0.0
        laser_high = 10.0
        self.observation_space = spaces.Box(low=laser_low, high=laser_high, shape=(self.environment_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)

        # Initialize Velodyne data (observation space dimensions)
        self.velodyne_data = np.ones(self.environment_dim) * 10  # Initialize with max range (e.g., 10 meters)

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        np.random.seed(seed)
        random.seed(seed)
        return [seed]

    def map_callback(self, data):
        # Process the map received from the /map topic
        self.map_resolution = data.info.resolution
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])
        width = data.info.width
        height = data.info.height
        map_data = np.array(data.data).reshape((height, width))

        # Convert -1 values (unknown) to 0 (free space)
        map_data[map_data == -1] = 0
        self.map = map_data

        # Set goal after receiving the map
        self.goal_position = self.set_goal()

        rospy.loginfo("Map received and processed.")

    def set_goal(self):
        if self.map is None:
            rospy.logwarn("Map has not been received yet!")
            return None

        # Find free space in the map
        free_space = np.argwhere(self.map == 0)  # Free space is marked by 0
        if len(free_space) == 0:
            rospy.logwarn("No free space found in the map!")
            return None

        # Set a fixed goal or randomly choose from free space
        goal_idx = random.choice(free_space)
        goal_position = self.grid_to_world(goal_idx)
        rospy.loginfo(f"Goal set at {goal_position}")
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
        points = list(pc2.read_points(data, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10  # Reset to max range for each scan

        for point in points:
            if point[2] > -0.2:  # Filter points below a certain Z threshold (e.g., ground filtering)
                # Calculate the angle (beta) and distance
                dist = math.sqrt(point[0] ** 2 + point[1] ** 2 + point[2] ** 2)
                angle = math.atan2(point[1], point[0])

                for i in range(self.environment_dim):
                    if self.gaps[i][0] <= angle < self.gaps[i][1]:
                        self.velodyne_data[i] = min(self.velodyne_data[i], dist)
                        break

    def step(self, action):
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)

        rospy.sleep(0.1)

        observation = self.velodyne_data.astype(np.float32)  # Ensure observation is float32
        reward = self.calculate_reward()
        
        # Determine if the episode has terminated
        terminated = self.is_done()
        
        # For simplicity, we'll assume episodes are never truncated in this implementation
        truncated = False

        return observation, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        # Handle the seed for reproducibility
        super().reset(seed=seed)  # Ensures Gym's seeding mechanism is handled correctly
        if seed is not None:
            np.random.seed(seed)
            random.seed(seed)

        # Wait until the map is received
        while self.map is None:
            rospy.logwarn("Waiting for map...")
            rospy.sleep(1.0)

        # Reset robot state and set the goal
        self.pose = Pose()
        self.goal_position = self.set_goal()

        # Reset Velodyne data for observation
        self.velodyne_data = np.ones(self.environment_dim, dtype=np.float32) * 10  # Reset observation with correct dtype

        return self.velodyne_data, {} 

    def calculate_reward(self):
        if self.goal_position is None:
            return -1000

        current_position = np.array([self.pose.position.x, self.pose.position.y])
        distance_to_goal = np.linalg.norm(current_position - self.goal_position)

        reward = -distance_to_goal

        # Penalize proximity to obstacles
        if np.min(self.velodyne_data) < 0.2:
            reward -= 100

        return reward

    def is_done(self):
        if self.goal_position is None:
            return True

        current_position = np.array([self.pose.position.x, self.pose.position.y])
        distance_to_goal = np.linalg.norm(current_position - self.goal_position)

        # Check if goal is reached or if the robot is too close to an obstacle
        if distance_to_goal < 0.5 or np.min(self.velodyne_data) < 0.2:
            return True

        return False

    def render(self, mode='human'):
        if self.map is None:
            rospy.logwarn("Map has not been received yet!")
            return

        plt.imshow(self.map, cmap='gray', origin='lower')
        current_position = self.world_to_grid([self.pose.position.x, self.pose.position.y])
        goal_position = self.world_to_grid(self.goal_position)
        plt.plot(current_position[1], current_position[0], 'bo')  # Robot's position in blue
        plt.plot(goal_position[1], goal_position[0], 'ro')  # Goal position in red
        plt.show()

    def close(self):
        rospy.signal_shutdown('Closing environment')
