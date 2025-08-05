# Wormhole Using ROS2

## Overview

This project demonstrates how to navigate between two maps using a custom action server that manages wormholes, which are defined as overlapping areas between two maps. The robot can autonomously switch between maps and navigate to specified poses within those maps.

### Mechanism:
Instead of passing the navigation goal directly to the Nav2 Stack (go_to_pose action), it is bypassed through a custom action-server. This action-server, takes the goal with the map name (extra parameter). Based on this, the node will query the DB and get the appropriate wormhole which connects two maps (present map to goal map) and manages the series goal assigning and map switching.

### Tools Used:

- **ROS 2 Humble**: Robot Operating System for building robot applications.
- **TurtleBot3**: A low-cost, open-source robot platform.
- **SLAM Toolbox**: For mapping.
- **PostgreSQL**: A relational database for storing wormhole coordinates.
- **Nav2 Stack**: Navigation stack for autonomous navigation.
- **Custom Action-Server**: A custom C++ action-server, to handle inter-map navigation goals and map switching.
- **Gazebo Classic**: A simulation environment for testing the robot in custom worlds.
- **RViz2**: A 3D visualization tool for ROS applications.

---


<details>
  <summary><strong><h1>Installation Instructions</h1></strong></summary>

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble installed


## Installation Steps

Install the dependencies required for this project initially, including postgresql requirement.
```bash
sudo apt update
sudo apt install -y \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-turtlebot3* \
ros-humble-rclcpp \
ros-humble-rclcpp-action \
ros-humble-geometry-msgs \
ros-humble-nav-msgs \
ros-humble-nav2-msgs \
ros-humble-std-msgs \
ros-humble-action-msgs \
ros-humble-rosidl-default-generators \
ros-humble-rosidl-default-runtime \
libpq-dev \
postgresql postgresql-contrib
```

Next, clone the repository and build the workspace.
```bash
mkdir -p wormhole_nav/src
cd wormhole_nav/src
git clone https://github.com/0RBalaji/wormhole_ros2.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-select wormhole_action
source install/setup.bash
colcon build --symlink-install --packages-select assesment
```
After building the workspace, you need to set up the PostgreSQL database. You can do this by running the following commands:

```bash
sudo -i -u postgres
psql -c "CREATE USER headuser WITH PASSWORD 'sqlpass123';"
psql -c "CREATE DATABASE wormhole_database OWNER headuser;"
psql -c "GRANT ALL PRIVILEGES ON DATABASE wormhole_database TO headuser;"
psql -h localhost -U headuser -d wormhole_database -h 127.0.0.1 -W

INSERT INTO wormholes (map1_name, map2_name, map1_position_x, map1_position_y, map2_position_x, map2_position_y)
VALUES ('map_1', 'map_2', 4.2, -4.1, 4.1, -4.4);
```

To exit the postgres command panel:
```bash
\q
```

The Setup is completed and now, the system is ready to run.

Make sure, the turtlebot3 files are installed and are run it before, to make sure the .STL files are cached locally.

### Launch methods
```bash
echo "source ~/wormhole_nav/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

#### Terminal tab 1(Main launch)

```bash
ros2 launch assesment multiMapNav.launch.py
```
This will launch the turtlebot3 rsp, the gazebo with the world file, spawn the robot, launch the localization and navigation. It later launches the custom multi_map action server

#### Terminal tab 2(multi_map_action_server Action Client)

```bash
ros2 action send_goal /multi_map_action_server wormhole_action/action/NavigateMultiMap "{
  target_map_name: 'map_2',
  target_pose: {
    header: {
      stamp: {sec: 0, nanosec: 0},
      frame_id: 'map'
    },
    pose: {
      position: {x: 0.0, y: -0.6, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: -1.57, w: 1.0}
    }
  }
}"
```
This will send a goal to the action server to navigate to the `map_2` with the specified pose in the map_2.
---
</details>

## Features

- Independent mapping of rooms using SLAM Toolbox (`async_slam_toolbox_node`)
- Wormhole-based transitions between maps through overlapping doorways
- Autonomous navigation using Nav2 and AMCL
- SQL for managing wormhole co-ordinates between maps
- Custom C++ Action Server for inter-map navigation
- Visualization of maps and robot state in RViz2

## Package Structure

```bash
├── assesment
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── database_setup.sql
│   │   ├── default_view.rviz
│   ├── include
│   │   └── assesment
│   │       ├── database_manager.hpp
│   │       └── multi_map_action_server.hpp
│   ├── launch
│   │   ├── bringup.launch.py
│   │   └── multiMapNav.launch.py
│   ├── maps
│   │   ├── map_1.yaml
│   │   ├── map_2.yaml
│   ├── package.xml
│   ├── src
│   │   ├── database_manager.cpp
│   │   └── multi_map_action_server.cpp
│   └── worlds
│       ├── model.world
│       └── multihouse.world
├── wormhole_action
    ├── action
    │   └── NavigateMultiMap.action
    ├── CMakeLists.txt
    ├── include
    │   └── wormhole_action
    ├── LICENSE
    ├── package.xml
    └── src
├── README.md
└── LICENSE

```

---
## Working Demo

![Wormhole Navigation Demo](demo.mp4)

---

## Future Improvements

- Multiple maps (more than 2) to improve the demonstration.

- Logic for intermediate wormhole travel, if no direct wormhole exists.

---

## Acknowledgements
- [TurtleBot3](https://www.turtlebot.com/) for the robot platform.
- [SLAM Toolbox]() for the async_slam_toolbox plugin.
- [Nav2 Stack](https://navigation.ros.org/) for the navigation framework.
- [PostgreSQL](https://www.postgresql.org/) for the database management.
- [Gazebo Classic](http://gazebosim.org/) for the simulation environment.
- [RViz2](https://www.ros.org/reps/rep-0105.html) for visualization.
---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details