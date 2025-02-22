# Trajectory Visualization for Anscer Robot

This package provides a ROS-based solution for collecting, visualizing, and saving the trajectory of an Anscer robot as it navigates in a Gazebo simulation. The trajectory data is visualized in RViz, and you can save portions of the path using a custom ROS service.

## Overview

- **Trajectory Collection & Visualization:** The robot's trajectory is continuously recorded and published as a `MarkerArray` to RViz for visualization
- **Trajectory Saving:** A custom ROS service (`/save_trajectory`) allows you to save the most recent trajectory data (for a specified duration) to a CSV file. The saved trajectory is then displayed in green in RViz
- **Integrated Simulation & Teleop:** The package integrates the Anscer robot simulation in Gazebo, teleoperation via keyboard, and the visualization nodes

## Dependencies

- ROS (Noetic recommended)
- Gazebo (for simulation)
- ROS Packages:
  - `roscpp`
  - `std_msgs`
  - `nav_msgs`
  - `visualization_msgs`
  - `tf2`
  - `tf2_ros`
  - `message_generation`
  - `message_runtime`

## Installation

### 1. Create a Catkin Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

### 2. Install the AR100 Package
```bash
git clone https://github.com/anscer/AR100.git
```

### 3. Install the Trajectory Visualization Package
```bash
git clone https://github.com/raj-tagore/trajectory_visualization.git
```

### 4. Build the Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

The system requires two terminals for operation:

### Terminal 1 – Launch Simulation & Visualization
Launch everything with a single command:
```bash
roslaunch trajectory_visualization trajectory_visualizer.launch
```

This will:
- Start the Anscer robot simulation in Gazebo
- Launch the teleop node for keyboard control
- Start custom nodes (`trajectory_publisher_saver_node` and `trajectory_reader_publisher`)
- Open RViz with preconfigured settings

### Terminal 2 – Save Trajectory Data
After moving the robot, save the trajectory:
```bash
rosservice call /save_trajectory "{file_name: 'trajectory.csv', duration: 10.0}"
```

This will:
- Save the last 10 seconds of trajectory to `trajectory.csv`
- Display the saved path in green in RViz
- Can be called multiple times to update saved trajectory

## Package Contents

### Nodes
- `trajectory_publisher_saver_node`: Collects and publishes robot trajectory as MarkerArray
  ```
  while running:
      listen to /odom topic
      store points in memory buffer
      publish blue markers to /robot_current_path
      
      on save_trajectory service call:
          filter last N seconds of trajectory
          save filtered points to CSV file
  ```
- `trajectory_reader_publisher`: Reads and republishes saved trajectory data
  ```
  while running:
      read trajectory.csv
      convert points to green markers
      clear previous markers
      publish new markers to /robot_saved_path
      sleep for 1 second
  ```

### Service
- `SaveTrajectory.srv`: Custom service for specifying filename and duration for trajectory saving

### Configuration Files
- `trajectory_visualizer.launch`: Master launch file for all components
- `rviz/trajectory-2.rviz`: Pre-configured RViz settings

## Troubleshooting

### If the rviz window seems bugged out or shows errors:

- This might be due to rviz launching before the robot is spawned with the gazebo simulation
- try to modify the launch file to increase the time delay between robot spawning and rviz launching