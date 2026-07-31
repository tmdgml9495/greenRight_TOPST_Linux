#ifndef TEMP_TRAFFIC_LIGHT_MANAGER_H
#define TEMP_TRAFFIC_LIGHT_MANAGER_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "types.h"

typedef struct {
    pthread_mutex_t lock;
    TrafficLight table[MAX_TRAFFIC_LIGHTS];
    bool valid[MAX_TRAFFIC_LIGHTS];
    TrafficLight candidate;
    uint8_t candidate_id;
    bool candidate_valid;
} TrafficLightManager;

bool traffic_light_manager_init(TrafficLightManager* manager);
void traffic_light_manager_destroy(TrafficLightManager* manager);
void traffic_light_manager_update(TrafficLightManager* manager, uint8_t tl_id, const TrafficLight* tl);
bool traffic_light_manager_get(TrafficLightManager* manager, uint8_t tl_id, TrafficLight* out);
void traffic_light_manager_select_candidate(TrafficLightManager* manager, uint8_t tl_id);
bool traffic_light_manager_get_candidate(TrafficLightManager* manager, uint8_t* tl_id_out, TrafficLight* out);

#endif
