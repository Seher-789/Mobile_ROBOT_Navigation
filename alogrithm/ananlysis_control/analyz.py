import pandas as pd
import matplotlib.pyplot as plt

# Paths to the log files
lidar_log_path = '/home/seher/Project/src/analysis_for_control/lidar_log.csv'  # Change this to the correct path
combined_log_path = '/home/seher/Project/src/analysis_for_control/combined_log.csv'  # Change this to the correct path

# Load the logged data
lidar_data = pd.read_csv(lidar_log_path)
combined_data = pd.read_csv(combined_log_path)

# Ensure the timestamps are in the correct format
lidar_data['timestamp'] = pd.to_datetime(lidar_data['timestamp'], unit='s')
combined_data['timestamp'] = pd.to_datetime(combined_data['timestamp'], unit='s')

# Plot the minimum distances
plt.figure(figsize=(12, 6))
plt.plot(lidar_data['timestamp'], lidar_data['min_distance'], label='LiDAR Min Distance', color='blue')
plt.plot(combined_data['timestamp'], combined_data['min_distance'], label='Combined Min Distance', color='red')
plt.xlabel('Time')
plt.ylabel('Distance (m)')
plt.title('Minimum Distance from Obstacle Over Time')
plt.legend()
plt.grid()
plt.show()

# Analyze the consistency of obstacle detection
obstacle_threshold = 1.0  # Threshold in meters to consider an obstacle

lidar_obstacle_detections = lidar_data[lidar_data['min_distance'] < obstacle_threshold]
combined_obstacle_detections = combined_data[combined_data['min_distance'] < obstacle_threshold]

print(f"Number of LiDAR obstacle detections: {len(lidar_obstacle_detections)}")
print(lidar_obstacle_detections)

print(f"Number of Combined obstacle detections: {len(combined_obstacle_detections)}")
print(combined_obstacle_detections)

# Save the analysis results to CSV files
lidar_obstacle_detections.to_csv('/home/seher/Project/src/analysis_for_control/lidar_obstacle_detections.csv', index=False)  # Change this to the correct path
combined_obstacle_detections.to_csv('/home/seher/Project/src/analysis_for_control/combined_obstacle_detections.csv', index=False)  # Change this to the correct path

# Summary statistics
print("\nSummary Statistics for LiDAR Min Distances:")
print(lidar_data['min_distance'].describe())

print("\nSummary Statistics for Combined Min Distances:")
print(combined_data['min_distance'].describe())
