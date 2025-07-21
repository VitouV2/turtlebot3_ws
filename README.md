# TurtleBot3 ROS 2 Workspace with Bug2 Navigation

This workspace contains a ROS 2 setup for simulating TurtleBot3 in Gazebo and running a custom **Bug2 navigation algorithm** for autonomous exploration or goal navigation.

Includes:
- TurtleBot3 simulation in Gazebo
- Custom `bug2_nav` package implementing the Bug2 wall-following/goal-seeking algorithm
- Sample world: `attr_field.world`

---

## Prerequisites

Make sure you have installed:
- [ROS 2 Humble](https://docs.ros.org/en/humble/ ) (or compatible version like Foxy/Iron)
- `turtlebot3` and `turtlebot3_simulations`
- Gazebo (Gazebo Classic, included with ROS 2 desktop-full)

Install dependencies:

```bash
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo
```

## Set environment variables:
```bash
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
```
## How to Build 

Clone this workspace into your development environment: 
```bash
cd ~/turtlebot3_ws/src
# (Your repo should be here)
```
Build the workspace: 

```bash
 cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```
## Launch TurtleBot3 in Gazebo World 

Launch the custom Gazebo world (attr_field.world) where Bug2 will operate: 

```bash
export TURTLEBOT3_MODEL= waffle
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py
```
 
⚠️ If you have a custom world file in ~/turtlebot3_ws/src/bug2_nav/bug2_nav/worlds/attr_field.world, make sure it's referenced correctly. You may need to set: 
    
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/v2/turtlebot3_ws/src/bug2_nav/bug2_nav/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/v2/turtlebot3_ws/src/bug2_nav/bug2_nav/worlds
```

🧠 Run the Bug2 Navigation Algorithm 

In a new terminal, source the workspace and run the Bug2 node: 

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run bug2_nav attr_field.py
 ```
This script assumes: 

- The robot starts at an initial pose
- Goal coordinates are defined inside attr_field.py
- Uses /scan (LIDAR) and /odom for localization
- Publishes velocity commands to /cmd_vel
         
To modify the goal, edit the file: 

```bash
nano /home/v2/turtlebot3_ws/src/bug2_nav/bug2_nav/attr_field.py
```
Look for lines like: 
python

self.goal_x = 2.0
self.goal_y = 2.0
 
Change them and re-run. 
