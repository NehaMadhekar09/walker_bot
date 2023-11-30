# walker_bot

## Overview
This repository contains a simple walker algorithm much like a Roomba robot vacuum cleaner. The turtlebot moves forward until it reaches an obstacle, then rotate in place until the way ahead is clear, then move forward again and repeat.This is demonstrated in gazebo world. 

## Dependencies
* ROS 2 Humble
* Ubuntu 22.04
* Gazebo
* Turtlebot3

## Build Instructions
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/NehaMadhekar09/walker_bot.git

cd ..

rosdep install -i --from-path src --rosdistro humble -y

colcon build 

. install/setup.bash

```
## Run instructions
### 1. Go to directory
```
cd ros2_ws
```
### 2. Source the workspace
```
. install/setup.bash
```
### 3. Run walker node with launch file
```
ros2 launch walker_bot launch.py
```

### 4. Run walker node with launch file and start ros bag recording
To run launch file with rosbag recording,
```
ros2 launch walker_bot launch.py record_rosbag:=True
```
To play ros bag
```
cd bag_list
```
```
ros2 bag play bag_list_0.db3
```

For viewing bag file information
```
ros2 bag info bag_list_0.db3
```

## Recorded Rosbag

Recorded rosbag is [here](results/bag_list/)
