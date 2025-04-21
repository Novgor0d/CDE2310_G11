# **Thermal Detection and Autonomous Exploration System**
## **Overview**
This repository contains the software framework for an autonomous robotic system capable of detecting thermal sources while performing autonomous exploration of an unkown environment and firing a flare to signal the detection. The system is built on ROS2 and meant for deployement on Raspberry Pi. 

## **Key Features**
- **Thermal Source Detection:** Detects heat sources based on certain temperature thresholds.
- **Autonmous Navigation:** Uses a frontier exploration algorithm to autnomously navigate towards unexplored regions.
- **Action Execution:** Fires a flare to indicate successful detection of a heat source, using a flywheel and servo-based system.
- **Data Logging**: Provides detailed logging of system status, sensor data, and actions for debugging and performance monitoring.

## **Prerequisites**
- ROS2 Humble (or later)
- Python 3.8 or higher
- Required ROS2 Packages:
  - rclpy
  - std_msgs
  - nav_msgs
  - geometry_msgs
  - action_msgs
  - RPi.GPIO
  - time
  - math

## **Installation and Setup**
1. **Clone the repository:**
```bash
cd ~/ros2_ws/
git clone https://github.com/Novgor0d/CDE2310_G11.git
```
2. **Build the ROS2 workspace:**
```bash
cd ~/ros2_ws
colcon build
```
3. **Source the workspace:**
```bash
source install/setup.bash
```
## **Running the System**
### ***On remote system:***
Run the following commands in separate terminal windows:
1. **Start RViz2 for visualization:**
```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```
2. **Launch SLAM:**
```bash
ros2 launch slam_toolbox online_async_launch.py
```
3. **Start the robot navigation system:**
```bash
ros2 launch robot_config nav2_launch.py
```
4. **Run the custom exploration node:**
```bash
ros2 run custom_explorer explorer
```




