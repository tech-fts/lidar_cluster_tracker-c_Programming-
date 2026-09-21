#include "include/lidar_cluster_tracker/cluster_engine.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>


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
   brain->obstacle_head = NULL;

   double* valid_x = (double*)malloc(sizeof(double) * number_range);
   double* valid_y = (double*)malloc(sizeof(double) * number_range);
   size_t valid_count = 0;

   for(int i=0; i < number_range; i++){
        int r = range[i];
        if(r >= *range_min && r <= *range_max && !isnan((double)r) && !isinf((double)r)){
            float angle = *angle_min + (i * *angle_increment);
            valid_x[valid_count] = r * cos(angle);
            valid_y[valid_count] = r * sin(angle);
            valid_count++;
        }
   }

   if(!valid_count){
        free(valid_x);
        free(valid_y);
        return 0;
   }

   int* visited = (int*)calloc(valid_count, sizeof(int));

   for(int i=0; i < valid_count; i++){
        if(visited[i]) continue;

        obstacle* newobs = (obstacle*)malloc(sizeof(obstacle));
        if(!newobs) continue;

        newobs->id = brain->next_obstacle_id++;
        newobs->point_head = NULL;
        newobs->point_count = 0;
        newobs->next = NULL;
   }

   return 0;
}