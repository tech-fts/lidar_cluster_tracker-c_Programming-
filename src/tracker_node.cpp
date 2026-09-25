#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <visualization_msgs/msg/marker_array.h>
#include <stdio.h>
#include <stdlib.h>

#include "lidar_cluster_tracker/cluster_engine.h"

// Globals or context bindings for handling data layers inside pure C callbacks
rcl_publisher_t marker_pub;
lidar_map* engine;
visualization_msgs__msg__MarkerArray output_markers;

// Callback triggered whenever a raw LIDAR point array lands on the ROS network
void lidar_scan_callback(const void* msgin) {
    const sensor_msgs__msg__LaserScan* scan_msg = (const sensor_msgs__msg__LaserScan*)msgin;
    
    if (scan_msg->ranges.size == 0) return;

    // Run pure C linked list grouping engine
    process_lidar_data(
        engine,
        scan_msg->ranges.data,
        scan_msg->ranges.size,
        scan_msg->angle_min,
        scan_msg->angle_increment,
        scan_msg->range_min,
        scan_msg->range_max
    );

    // Clean out previous marker memory fields to prepare output
    for (size_t i = 0; i < output_markers.markers.capacity; i++) {
        output_markers.markers.data[i].action = visualization_msgs__msg__Marker__DELETE;
    }

    // Convert obstacle nodes into ROS 2 Marker shapes
    size_t idx = 0;
    Obstacle* curr = engine->obstacles_head;
    
    while (curr && idx < output_markers.markers.capacity) {
        visualization_msgs__msg__Marker* m = &output_markers.markers.data[idx];
        
        m->header.frame_id.data = "laser_frame";
        m->header.frame_id.size = strlen(m->header.frame_id.data);
        m->ns.data = "obstacles";
        m->ns.size = strlen(m->ns.data);
        m->id = curr->id;
        m->type = visualization_msgs__msg__Marker__SPHERE;
        m->action = visualization_msgs__msg__Marker__ADD;
        
        // Target spatial tracking frames computed by the C engine
        m->pose.position.x = curr->center_x;
        m->pose.position.y = curr->center_y;
        m->pose.position.z = 0.0;
        m->pose.orientation.w = 1.0;
        
        // Scale sphere bounding profile based on actual point configurations
        m->scale.x = (curr->size < 0.2) ? 0.2 : curr->size;
        m->scale.y = (curr->size < 0.2) ? 0.2 : curr->size;
        m->scale.z = 0.2;
        
        // Vibrant tracking color layout (Green markers)
        m->color.r = 0.0f;
        m->color.g = 1.0f;
        m->color.b = 0.0f;
        m->color.a = 0.8f;
        
        idx++;
        curr = curr->next;
    }
    
    output_markers.markers.size = idx;

    // Direct publish command over network middleware
    rcl_publish(&marker_pub, &output_markers, NULL);
}

int main(int argc, const char* const* argv) {
    // 1. Core Allocation & Subsystem Spin up
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_ret_t rc = rclc_support_init(&support, argc, argv, &allocator);
    
    // Create Node configuration
    rcl_node_t node;
    rc = rclc_node_init_default(&node, "lidar_tracker_node", "", &support);

    // Initialize the computational layout engine (0.25 meters Euclidean limit)
    engine = init_cluster_engine(0.25);

    // 2. Setup Communication Ports (Subscriber & Publisher)
    rc = rclc_publisher_init_default(
        &marker_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, MarkerArray),
        "detected_obstacles"
    );

    rcl_subscription_t lidar_sub;
    sensor_msgs__msg__LaserScan input_scan_msg;
    sensor_msgs__msg__LaserScan__init(&input_scan_msg);
    
    rc = rclc_subscription_init_default(
        &lidar_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan"
    );

    // 3. Pre-allocate Output Arrays to prevent middleware frame tearing
    visualization_msgs__msg__MarkerArray__init(&output_markers);
    output_markers.markers.data = (visualization_msgs__msg__Marker*)malloc(sizeof(visualization_msgs__msg__Marker) * 50);
    output_markers.markers.capacity = 50;
    output_markers.markers.size = 0;
    
    for (size_t i = 0; i < 50; i++) {
        visualization_msgs__msg__Marker__init(&output_markers.markers.data[i]);
    }

    // 4. Connect Executor Runtime Scheduler Loop
    rclc_executor_t executor;
    rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
    rc = rclc_executor_add_subscription(&executor, &lidar_sub, &input_scan_msg, &lidar_scan_callback, ON_NEW_DATA);

    printf("Pure C Obstacle Clustering Engine Active. Listening on topic: /scan ...\n");
    
    // Non-stop thread spin handling incoming communications
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }

    // Clean up paths before shutdown
    free_cluster_engine(engine);
    rc = rcl_subscription_fini(&lidar_sub, &node);
    rc = rcl_publisher_fini(&marker_pub, &node);
    rc = rcl_node_fini(&node);
    rc = rclc_support_clean(&support);
    
    return 0;
}
