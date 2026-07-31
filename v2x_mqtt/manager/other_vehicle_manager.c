#include "other_vehicle_manager.h"

#include <string.h>
#include <time.h>

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

bool other_vehicle_manager_init(OtherVehicleManager* manager)
{
    if (!manager) return false;
    memset(manager, 0, sizeof(*manager));
    pthread_mutex_init(&manager->lock, NULL);
    return true;
}

void other_vehicle_manager_destroy(OtherVehicleManager* manager)
{
    if (!manager) return;
    pthread_mutex_destroy(&manager->lock);
}

void other_vehicle_manager_update(OtherVehicleManager* manager, const VehicleInfo* vehicle)
{
    if (!manager || !vehicle || vehicle->vehicle_id >= MAX_OTHER_VEHICLES) return;
    pthread_mutex_lock(&manager->lock);
    manager->table[vehicle->vehicle_id] = *vehicle;
    manager->valid[vehicle->vehicle_id] = true;
    manager->last_updated_ms[vehicle->vehicle_id] = monotonic_ms();
    pthread_mutex_unlock(&manager->lock);
}

bool other_vehicle_manager_get(OtherVehicleManager* manager, uint8_t vehicle_id, VehicleInfo* out)
{
    if (!manager || !out || vehicle_id >= MAX_OTHER_VEHICLES) return false;
    pthread_mutex_lock(&manager->lock);
    bool valid = manager->valid[vehicle_id];
    if (valid) *out = manager->table[vehicle_id];
    pthread_mutex_unlock(&manager->lock);
    return valid;
}

int other_vehicle_manager_count(OtherVehicleManager* manager)
{
    if (!manager) return 0;
    int count = 0;
    pthread_mutex_lock(&manager->lock);
    for (int i = 0; i < MAX_OTHER_VEHICLES; i++) {
        if (manager->valid[i]) count++;
    }
    pthread_mutex_unlock(&manager->lock);
    return count;
}

int other_vehicle_manager_copy_valid(OtherVehicleManager* manager, VehicleInfo* out, int max_count)
{
    if (!manager || !out || max_count <= 0) return 0;

    int count = 0;
    pthread_mutex_lock(&manager->lock);
    for (int i = 0; i < MAX_OTHER_VEHICLES && count < max_count; i++) {
        if (!manager->valid[i]) continue;
        out[count++] = manager->table[i];
    }
    pthread_mutex_unlock(&manager->lock);
    return count;
}

void other_vehicle_manager_cleanup_stale(OtherVehicleManager* manager, uint64_t timeout_ms)
{
    if (!manager) return;
    uint64_t now = monotonic_ms();
    pthread_mutex_lock(&manager->lock);
    for (int i = 0; i < MAX_OTHER_VEHICLES; i++) {
        if (!manager->valid[i]) continue;
        if (now - manager->last_updated_ms[i] > timeout_ms) {
            manager->valid[i] = false;
            if (manager->candidate_valid && manager->candidate.vehicle_id == i) {
                memset(&manager->candidate, 0, sizeof(manager->candidate));
                manager->candidate_valid = false;
            }
        }
    }
    pthread_mutex_unlock(&manager->lock);
}

void other_vehicle_manager_set_candidate(OtherVehicleManager* manager, const VehicleInfo* candidate)
{
    if (!manager) return;
    pthread_mutex_lock(&manager->lock);
    if (candidate) {
        manager->candidate = *candidate;
        manager->candidate_valid = true;
    } else {
        memset(&manager->candidate, 0, sizeof(manager->candidate));
        manager->candidate_valid = false;
    }
    pthread_mutex_unlock(&manager->lock);
}

bool other_vehicle_manager_get_candidate(OtherVehicleManager* manager, VehicleInfo* out)
{
    if (!manager || !out) return false;
    pthread_mutex_lock(&manager->lock);
    bool valid = manager->candidate_valid;
    if (valid) *out = manager->candidate;
    pthread_mutex_unlock(&manager->lock);
    return valid;
}
