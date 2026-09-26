# LiDAR Cluster Tracker

A ROS 2 package for clustering LiDAR scan points into obstacle groups and publishing the results as visualization markers. The project combines a small pure-C clustering engine with a C++ ROS 2 node that subscribes to `sensor_msgs/msg/LaserScan` and publishes `visualization_msgs/msg/MarkerArray` on `/detected_obstacles`.

## Overview

This repository implements a lightweight obstacle tracker that:

- reads range values from a laser scan,
- filters invalid points,
- converts polar measurements into Cartesian coordinates,
- groups nearby points into clusters using an Euclidean distance threshold,
- computes obstacle centroid and size,
- publishes each detected cluster as a ROS marker.

The implementation is intentionally split between:

- a C cluster engine for point processing and obstacle grouping,
- a ROS 2 node for message transport and visualization output.

## Project structure

```text
lidar_cluster_tracker/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/
│   └── lidar_cluster_tracker/
│       └── cluster_engine.h
├── src/
│   ├── cluster_engine.c
│   └── tracker_node.cpp
└── .vscode/
```

## Current implementation

The current codebase contains a working prototype rather than just a scaffold.

### Cluster engine

The C engine in `src/cluster_engine.c` exposes a lightweight data model for obstacle tracking:

- `lidar_map` stores the linked list of detected obstacles and the next obstacle ID.
- `obstacle` stores the cluster centroid, size, point count, and linked point list.
- `point_node` stores x/y coordinates for each cluster member.

Key functions:

```c
lidar_map* int_lidar_search(double tolerance);
int lidar_process_steps(lidar_map* brain, const int* range, size_t number_range,
                       const float* angle_min, const float* angle_increment,
                       const float* range_min, const float* range_max);
int free_lidar_map(lidar_map* brain);
```

The clustering step:

- ignores NaN/infinite values and points outside `[range_min, range_max]`,
- converts each valid range to Cartesian coordinates,
- uses a queue-based connected-component search,
- groups points whose Euclidean distance stays within a configured threshold.

### ROS 2 node

`src/tracker_node.cpp` creates a node that:

1. initializes ROS 2 support and a node,
2. initializes a subscriber for `/scan`,
3. initializes a publisher for `/detected_obstacles`,
4. allocates marker storage,
5. processes incoming `LaserScan` messages,
6. converts obstacles into `MarkerArray` messages.

## Dependencies

This package depends on the standard ROS 2 build and message packages:

- `ament_cmake`
- `rcl`
- `rclc`
- `sensor_msgs`
- `visualization_msgs`

Install them on a sourced ROS 2 environment:

```bash
sudo apt-get update
sudo apt-get install \
  ros-$ROS_DISTRO-rcl \
  ros-$ROS_DISTRO-rclc \
  ros-$ROS_DISTRO-sensor-msgs \
  ros-$ROS_DISTRO-visualization-msgs
```

## Build

From a ROS 2 workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_cluster_tracker
source install/setup.bash
```

The package is configured with CMake in `CMakeLists.txt` and declares its dependencies in `package.xml`.

## Run

Start the node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_cluster_tracker tracker_node
```

Publish a `LaserScan` stream from a simulator, sensor driver, or bag file, for example:

```bash
ros2 topic pub /scan sensor_msgs/msg/LaserScan "{...}"
```

Or replay a bag file:

```bash
ros2 bag play <bag_name>
```

Inspect output markers in RViz:

```bash
ros2 run rviz2 rviz2
```

Then add the `/scan` topic and `/detected_obstacles` marker topic to visualize the clustered obstacles.

## Algorithm notes

For each valid scan point:

```text
x = range * cos(angle)
y = range * sin(angle)
```

The cluster engine keeps points in the same obstacle while their pairwise Euclidean distance remains below the configured tolerance:

```text
d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
```

Each cluster is summarized by:

- centroid position,
- approximate size,
- number of contributing points.

## Notes

- The package currently builds as a C/C++ ROS 2 executable using `ament_cmake`.
- The C engine is responsible for clustering logic; the C++ node is responsible for ROS integration.
- The package is suitable as a starting point for further development such as obstacle tracking over time, more robust lifecycle handling, or message type refinement.

## License

This project is distributed under the Apache 2.0 license, as declared in `package.xml`.
