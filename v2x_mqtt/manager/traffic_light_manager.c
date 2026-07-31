#include "traffic_light_manager.h"

#include <string.h>

bool traffic_light_manager_init(TrafficLightManager* manager)
{
    if (!manager) return false;
    memset(manager, 0, sizeof(*manager));
    pthread_mutex_init(&manager->lock, NULL);
    manager->candidate_id = 0xFF;
    return true;
}

void traffic_light_manager_destroy(TrafficLightManager* manager)
{
    if (!manager) return;
    pthread_mutex_destroy(&manager->lock);
}

void traffic_light_manager_update(TrafficLightManager* manager, uint8_t tl_id, const TrafficLight* tl)
{
    if (!manager || !tl || tl_id >= MAX_TRAFFIC_LIGHTS) return;
    pthread_mutex_lock(&manager->lock);
    manager->table[tl_id] = *tl;
    manager->valid[tl_id] = true;
    pthread_mutex_unlock(&manager->lock);
}

bool traffic_light_manager_get(TrafficLightManager* manager, uint8_t tl_id, TrafficLight* out)
{
    if (!manager || !out || tl_id >= MAX_TRAFFIC_LIGHTS) return false;
    pthread_mutex_lock(&manager->lock);
    bool valid = manager->valid[tl_id];
    if (valid) *out = manager->table[tl_id];
    pthread_mutex_unlock(&manager->lock);
    return valid;
}

void traffic_light_manager_select_candidate(TrafficLightManager* manager, uint8_t tl_id)
{
    if (!manager) return;
    pthread_mutex_lock(&manager->lock);
    if (tl_id < MAX_TRAFFIC_LIGHTS && manager->valid[tl_id]) {
        manager->candidate = manager->table[tl_id];
        manager->candidate_id = tl_id;
        manager->candidate_valid = true;
    } else {
        memset(&manager->candidate, 0, sizeof(manager->candidate));
        manager->candidate_id = 0xFF;
        manager->candidate_valid = false;
    }
    pthread_mutex_unlock(&manager->lock);
}

bool traffic_light_manager_get_candidate(TrafficLightManager* manager, uint8_t* tl_id_out, TrafficLight* out)
{
    if (!manager || !out) return false;
    pthread_mutex_lock(&manager->lock);
    bool valid = manager->candidate_valid;
    if (valid) {
        *out = manager->candidate;
        if (tl_id_out) *tl_id_out = manager->candidate_id;
    }
    pthread_mutex_unlock(&manager->lock);
    return valid;
}
