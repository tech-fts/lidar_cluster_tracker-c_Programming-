# Lidar Cluster Tracker

A pure-C ROS 2 package for extracting and tracking obstacles from a `sensor_msgs/msg/LaserScan` stream. The planned implementation uses `rclc` and the C message APIs, without `rclcpp`, C++ source files, or C++ headers.

> **Project status:** This repository is currently a scaffold. The source files, headers, `CMakeLists.txt`, and `package.xml` are empty. The sections below describe the implementation that belongs in those files; the build and run commands will become usable after the implementation is added.

## Architecture

The package has two deliberately separate layers:

- **Clustering engine**: standard C data structures and math. It receives Cartesian points, groups nearby points into obstacles, and updates tracked obstacle state.
- **ROS 2 node**: `rclc` initialization, the executor, the `/scan` subscription, and the marker publisher. The callback owns no hidden C++ object state; it receives application state through a context pointer.

Expected layout:

```text
lidar_cluster_tracker/
├── CMakeLists.txt
├── package.xml
├── include/lidar_cluster_tracker/
│   ├── cluster_engine.h
│   └── tracker_node.h
└── src/
		├── cluster_engine.c
		└── tracker_node.c
```

## Data model

The engine should expose plain C types similar to these:

```c
typedef struct {
	float x;
	float y;
} lidar_point_t;

typedef struct {
	uint32_t id;
	lidar_point_t centroid;
	float width;
	float length;
	uint32_t point_count;
	uint32_t missed_frames;
} obstacle_t;
```

The tracker state should own its point buffer, obstacle list, capacity limits, and configuration such as the point-to-point distance threshold. Ownership and cleanup must remain explicit so that callbacks do not leak memory or invalidate message buffers.

## Clustering algorithm

For each valid scan range, convert polar coordinates to Cartesian points:

```text
x = range * cos(angle)
y = range * sin(angle)
```

The first valid point starts a cluster. Each following point remains in the current cluster when its Euclidean distance from the previous point is below the configured threshold:

```text
d = sqrt((x2 - x1)^2 + (y2 - y1)^2)
```

When `d` exceeds the threshold, close the current cluster, calculate its centroid and bounds, and start a new one. Invalid ranges (`NaN`, infinity, or values outside the configured scan limits) are ignored. A production implementation should also reject clusters below a minimum point count to reduce noise.

The tracking stage can associate a new centroid with an existing obstacle when the centroid distance is below a separate association threshold. Unmatched tracks increment `missed_frames` and should be removed after a configurable timeout.

## ROS 2 communication

The node is intended to:

1. Initialize an `rcl_context_t`, `rclc_support_t`, and `rcl_node_t`.
2. Initialize an `rcl_subscription_t` for `/scan` using `sensor_msgs__msg__LaserScan`.
3. Initialize an `rcl_publisher_t` for `/detected_obstacles` using a visualization message type such as `visualization_msgs__msg__MarkerArray`.
4. Initialize an `rclc_executor_t` with the exact number of handles.
5. Add the subscription with the preallocated `LaserScan` message and application context.
6. Spin the executor and publish markers from the callback after clustering and tracking.
7. Finalize the publisher, subscription, node, support object, executor, and allocated buffers on shutdown.

For deterministic behavior, allocate the scan ranges, cluster storage, obstacle storage, and marker storage before spinning. The exact initialization of variable-length ROS messages must follow the message API provided by the selected ROS 2 distribution; array capacities and string storage must be set before the executor receives data.

## Dependencies

The target package depends on:

- `ament_cmake`
- `rclc`
- `sensor_msgs`
- `visualization_msgs`

On a sourced ROS 2 installation, install the C client library and package dependencies using the distribution name in `$ROS_DISTRO`:

```bash
sudo apt-get update
sudo apt-get install \
	ros-$ROS_DISTRO-rclc \
	ros-$ROS_DISTRO-sensor-msgs \
	ros-$ROS_DISTRO-visualization-msgs
```

`rclc_lifecycle` is not required for the basic node described here. Add it only if lifecycle-node behavior is implemented.

## Build

After the C sources and package metadata have been implemented:

```bash
cd ~/ros2_ws
colcon build --packages-select lidar_cluster_tracker
source install/setup.bash
```

The package must be built with a C compiler and must link the generated C type-support targets for the message packages. Exact target names can vary by ROS 2 distribution, so the final `CMakeLists.txt` should use the conventions supported by the installed `rosidl` version rather than assuming a C++ target.

## Run

Source the workspace and start the node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lidar_cluster_tracker tracker_node
```

Provide a `/scan` publisher from a simulator, a real sensor, or a bag file in another terminal:

```bash
ros2 bag play sample_lidar_data/
```

To inspect the output, run RViz 2 and add the `/scan` topic plus `/detected_obstacles`:

```bash
ros2 run rviz2 rviz2
```

## Implementation checklist

- Define bounded C structs and explicit initialization/finalization functions.
- Allocate all message and tracker buffers before executor spin.
- Keep clustering and tracking independent of ROS headers where practical.
- Validate ranges and capacities before indexing arrays.
- Publish stable obstacle IDs and delete stale marker IDs when tracks expire.
- Add tests for empty scans, invalid ranges, threshold boundaries, separated clusters, and track expiration.