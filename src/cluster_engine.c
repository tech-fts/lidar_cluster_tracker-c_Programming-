#include "include/lidar_cluster_tracker/cluster_engine.h"
#include <stdlib.h>


lidar_map* int_lidar_search(double tolerance){
    lidar_map* engine = (lidar_map*)malloc(sizeof(lidar_map)); // use lidar_map
    if(!engine) return NULL;
    engine->obstacle_head = NULL;
    engine->next_obstacle_id = 1;
    engine->obstacle_tolurance = tolerance;
}

void free_point_list(point_node* head){
    while(head){
        point_node* temp = head;
        head = head->next;
        free(temp);
    }
}

void free_obstacle_list(obstacle* head){
    while(head){
        obstacle* temp = head;
        head = head->next;
        free_point_list(temp->point_head);
        free(temp);
    }
}

int lidar_process_steps(lidar_map* brain, const int* range, size_t number_range, const float* angle_min, const float* angle_increment, const float* range_min, const float* range_max){
   free_obstacle_list(brain->obstacle_head);
}