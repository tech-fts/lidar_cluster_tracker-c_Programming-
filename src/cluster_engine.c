#include "lidar_cluster_tracker/cluster_engine.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>


lidar_map* int_lidar_search(double tolerance){
    lidar_map* engine = (lidar_map*)malloc(sizeof(lidar_map)); // use lidar_map
    if(!engine) return NULL;
    engine->obstacle_head = NULL;
    engine->next_obstacle_id = 1;
    engine->obstacle_tolurance = tolerance;
    return engine;
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

int free_lidar_map(lidar_map* brain){
    if (!brain) return 0;
    free_obstacle_list(brain->obstacle_head);
    free(brain);
    return 0;
}

int lidar_process_steps(lidar_map* brain, const int* range, size_t number_range, const float* angle_min, const float* angle_increment, const float* range_min, const float* range_max){
   free_obstacle_list(brain->obstacle_head);
   brain->obstacle_head = NULL;

   double* valid_x = (double*)malloc(sizeof(double) * number_range);
   double* valid_y = (double*)malloc(sizeof(double) * number_range);
   size_t valid_count = 0;

   for(size_t i=0; i < number_range; ++i){
        float r = range[i];
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

   for(size_t i=0; i < valid_count; ++i){
        if(visited[i]) continue;

        obstacle* newobs = (obstacle*)malloc(sizeof(obstacle));
        if(!newobs) continue;

        newobs->id = brain->next_obstacle_id++;
        newobs->point_head = NULL;
        newobs->point_count = 0;
        newobs->next = NULL;

        size_t* waiting_list = (size_t*)malloc(sizeof(size_t) * valid_count);
        size_t first = 0, last = 0;

        waiting_list[last++] = i;
        visited[i] = 1;

        double sum_x = 0;
        double sum_y = 0;

        while(first < last){
            size_t cur_Idx = waiting_list[first++];
            double cx = valid_x[cur_Idx];
            double cy = valid_y[cur_Idx];

            sum_x += cx;
            sum_y += cy;

            //tempory point create
            point_node* p_node = (point_node*)malloc(sizeof(point_node));
            if(p_node){
                p_node->x = cx;
                p_node->y = cy;
                p_node->next = newobs->point_head;
                newobs->point_head = p_node;
                newobs->point_count++;
            }

            //search unvisited point
            for(size_t j = 0; j < valid_count; ++j){
                if(visited[j]) continue;

                double dx = valid_x[j] - cx;
                double dy = valid_y[j] - cy;
                double dist = sqrt(dx*dx + dy* dy);

                if(dist <= brain->obstacle_tolurance){
                    waiting_list[last++] = j;
                    visited[j] = 1;
                }
            }

            newobs->center_x = sum_x / newobs->point_count;
            newobs->center_y = sum_y / newobs->point_count;

            double max_dist = 0;
            for(point_node* curr_pt = newobs->point_head; curr_pt != NULL; curr_pt = curr_pt->next){
                double dx = curr_pt->x - newobs->center_x;
                double dy = curr_pt->y - newobs->center_y;
                double d = sqrt(dx*dx+ dy*dy);
                if(d > max_dist) max_dist = d;
            }

            newobs->size = max_dist * 2.0;

        }

        newobs->next = brain->obstacle_head;
        brain->obstacle_head = newobs;
        free(waiting_list);

   }

   free(visited);
   free(valid_x);
   free(valid_y);
    return 0;
}