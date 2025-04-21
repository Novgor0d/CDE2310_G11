# **Thermal Detection and Autonomous Exploration System**
## **Overview**
This repository contains the software framework for a robotic system capable of detecting thermal sources while performing autonomous exploration of an unkown environment. The system is built on ROS2 and meant for deployement on Raspberry Pi. It maps the surroundings using SLAM, detects heat signatures, and deploys a flare mechanism.

## **Key Features**
- **Autonmous frontier-based exploration:** Uses a frontier exploration algorithm to autnomously navigate towards unexplored regions.
- **Real-time SLAM integration**
- **Thermal Source Detection:** Detects heat sources based on certain temperature thresholds.
- **Motorized flare deployment:** Fires a flare to indicate successful detection of a heat source, using a flywheel and servo-based system.
- **Data Logging**: Provides detailed logging of system status, sensor data, and actions for debugging and performance monitoring.
- **Full ROS2 Humble compatibility**:

## **System Architecture**
The software stack is divided across two platforms:
  - **Remote PC (Main Control Node):** Runs SLAM, Nav2 Stack, RViz visualisation, and the exploration code.
  - **Raspberry Pi (Onboard Robot):** Handles robot bring-up, thermal sensing, and flare deployment mechanisms.
  
## **Prerequisites**
- ROS2 Humble
- Python 3
- TurtleBot3-compatible hardware (or Gazebo simulation)

## **Installation and Setup**
1. **Create the ROS2 workspace:**
```bash
mkdir -p ~/ros2_ws/src
```
2. **Clone the repository:**

  (On remote pc)
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
3. **Use Custom Navigatoin Parameters:**
This system uses a customized nav2_params.yaml file located at: robot_config/config/nav2_params.yaml
```bash
ros2 launch robot_config nav2_launch.py
```
This step ensures that all Nav2 nodes (e.g., planner, controller, recovery behaviours) run with the tuned parameters specific to this project. These parameters can be adjusted as per requirement.
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

##**System Functionality**
###**Thermal Detection and Flare Firing**
Upon initialization, the system continuously scans the environment using the thermal sensor. When a heat source (e.g., fire) is detected, the robot navigates towards the source and fires a flare to indicate the detection. The detection and flare firing are performed based on a predefined temperature threshold, which can be configured to suit various use cases.

###**Autonomous Exploration**
The system employs a frontier exploration algorithm that identifies unexplored areas (frontiers) in the environment. The robot autonomously navigates towards these regions to gather more data, expanding its knowledge of the environment. The system dynamically switches between frontier-based exploration and random exploration after specified time intervals.

###**ROS2 Communication**
The system utilizes ROS2 for communication between the various components:
**Published Topics:**
  - /fire_flare ([Bool]): Indicates when the robot detects a thermal source and fires a flare.
  - /cmd_vel ([geometry_msgs/msg/Twist]): Command velocity for controlling robot movement.
  - /goal_pose ([geometry_msgs/msg/PoseStamped]): The target position for the robot (e.g., towards a detected heat source or frontier).

**Subscribed Topics:**
   - /odom ([nav_msgs/msg/Odometry]): Odometry data used for robot localization.
   - /map ([nav_msgs/msg/OccupancyGrid]): The map of the environment for exploration and frontier detection.

##**Node Architecture:**
1. **ThermalDetectorNode:**
  - Purpose: Detects thermal sources and triggers corresponding actions.
  - Operation: Scans the environment at regular intervals and activates the flare system when a source exceeding the temperature threshold is detected.
2. **FlareExplorerNode**
  - Purpose: Controls the flare mechanism (flywheel and servo) to fire flares.
  - Operation: Spins the flywheel and activates the servo motor to fire the flare.
3. **ExplorerNode**
  - Purpose: Responsible for autonomous exploration using a frontier detection algorithm.
  - Operation: Detects unexplored frontiers and navigates the robot towards them to enhance map coverage.

##Testing and Validation
The system has been tested both in simulated environments (Gazebo) and with real-world harware. The modular nature of the sytem ensures that individual components can be tested in isolation for reliability.



 




