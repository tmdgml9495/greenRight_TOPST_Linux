#ifndef TEMP_OTHER_VEHICLE_MANAGER_H
#define TEMP_OTHER_VEHICLE_MANAGER_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "types.h"

typedef struct {
    pthread_mutex_t lock;
    VehicleInfo table[MAX_OTHER_VEHICLES];
    bool valid[MAX_OTHER_VEHICLES];
    uint64_t last_updated_ms[MAX_OTHER_VEHICLES];
    VehicleInfo candidate;
    bool candidate_valid;
} OtherVehicleManager;

bool other_vehicle_manager_init(OtherVehicleManager* manager);
void other_vehicle_manager_destroy(OtherVehicleManager* manager);
void other_vehicle_manager_update(OtherVehicleManager* manager, const VehicleInfo* vehicle);
bool other_vehicle_manager_get(OtherVehicleManager* manager, uint8_t vehicle_id, VehicleInfo* out);
int other_vehicle_manager_count(OtherVehicleManager* manager);
int other_vehicle_manager_copy_valid(OtherVehicleManager* manager, VehicleInfo* out, int max_count);
void other_vehicle_manager_cleanup_stale(OtherVehicleManager* manager, uint64_t timeout_ms);
void other_vehicle_manager_set_candidate(OtherVehicleManager* manager, const VehicleInfo* candidate);
bool other_vehicle_manager_get_candidate(OtherVehicleManager* manager, VehicleInfo* out);

#endif
