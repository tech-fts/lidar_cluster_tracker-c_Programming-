#ifndef CLUSTER_ENGINE_H
#define CLUSTER_ENGINE_H

#include <stddef.h>

typedef struct{
    double x;
    double y;
    struct point_node* next;
}point_node;

typedef struct{
    int id;
    double center_x; //use center 
    double center_y;
    double size;
    int point_count;
    struct point_node* point_head;
    struct obstacle* next;
}obstacle;

typedef struct{
    obstacle* obstacle_head;
    int next_obstacle_id;
    double obstacle_tolurance; //need to change
}lidar_map;

lidar_map* int_lidar_search(double tolerance);
int lidar_process_steps(lidar_map* brain, const int* range, size_t number_range, const float* angle_min, const float* angle_increment, const float* range_min, const float* range_max);
int free_lidar_map(lidar_map* brain);

#endif