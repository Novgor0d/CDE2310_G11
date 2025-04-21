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
1. **Create the ROS2 workspace:**
```bash
mkdir -p ~/ros2_ws/src
```
2. **Clone the repository:**
(On Laptop)
```bash
cd ~/ros2_ws/src
git clone https://github.com/Novgor0d/CDE2310_G11.git
cd CDE2310_G11/software/ros2_ws/src
cp -r * ~/ros2_ws/src/
rm -rf ~/ros2_ws/src/CDE2310_G11
rm -rf ~/ros2_ws/src/thermal_explorer
rm -rf ~/ros2_ws/src/fire_flare
```
(On RPi)
```bash
cd ~/ros2_ws/src
git clone https://github.com/Novgor0d/CDE2310_G11.git
cd CDE2310_G11/software/ros2_ws/src
cp -r * ~/ros2_ws/src/
rm -rf ~/ros2_ws/src/CDE2310_G11
rm -rf ~/ros2_ws/src/custom_explorer
rm -rf ~/ros2_ws/src/robot_config
```
3. **Install dependencies and build the workspace:**
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
4. **Source the workspace:**
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
### **On Raspberry Pi:**
Run the following commands in separate terminal windows:
1. **Launch TurtleBot3 bringup:**
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
2. **Start the thermal detection node:**
```bash
ros2 run thermal_explorer thermal
```
3. **Run the flare firing mechanism:**
```bash
ros2 run fire_flare motor
```
## **Simulating in Gazebo:**
To simulate the navigation in a virutal environment, run the following commands in separate terminals:
1. **Launch the TurtleBot3 in a Gazebo simulation world:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
2. **Launch SLAM with simulation time enabled:**
```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
3. **Launch Nav2 stack with simulation time enabled:**
```bash
ros2 launch robot_config nav2_launch.py use_sim_time:=True
```
4. **Run the custom exploration node in the simulation:**
```bash
ros2 run custom_explorer explorer
```


 




