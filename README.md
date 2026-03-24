# Lidar Car — Autonomous Obstacle Avoidance
ROS2 Humble + Gazebo Classic simulation of a differential drive
robot using 2D LiDAR and Vector Field Histogram (VFH) navigation.
https://youtu.be/ngs3ylgHqjU

## Features
- 360° LiDAR obstacle detection
- VFH-based autonomous navigation
- Stuck detection + collision recovery
- Custom Gazebo world generator (Python)
- RViz2 auto-configuration with path tracing

## Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Classic (gazebo-ros-pkgs)

## Install Dependencies
```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2
```

## Build
```bash
cd ~/lidar_car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
# Default maze world
ros2 launch lidar_car_controller simulation.launch.py

# Custom world
ros2 launch lidar_car_controller simulation.launch.py \
  world:=my_custom_world.world

# Generate your own world
cd src/lidar_car_controller/scripts
python3 generate_world.py
```

## Project Structure
```
lidar_car_ws/
├── src/
│   ├── lidar_car_description/     # URDF robot model + RViz config
│   │   ├── urdf/lidar_car.urdf.xacro
│   │   ├── rviz/lidar_car.rviz
│   │   └── launch/
│   └── lidar_car_controller/      # Navigation nodes
│       ├── lidar_car_controller/
│       │   ├── vfh_controller.py  # VFH + recovery
│       │   ├── path_tracer.py     # RViz path display
│       │   └── obstacle_spawner.py
│       ├── worlds/                # Gazebo world files
│       ├── scripts/
│       │   └── generate_world.py  # World generator
│       └── launch/
```

## Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | LaserScan | LiDAR data |
| `/cmd_vel` | Twist | Robot velocity |
| `/odom` | Odometry | Robot position |
| `/vfh_debug` | String | VFH state info |
| `/odom_path` | Path | Travelled route |

## Author
Arun — Robotics & Embedded Systems Engineer
EOF
```
https://github.com/ARUN-KUMAR-86/LARA-LIDAR-Autonomous-Robot-Avoidance.git
---
