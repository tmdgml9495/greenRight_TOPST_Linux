#ifndef TEMP_SELF_VEHICLE_MANAGER_H
#define TEMP_SELF_VEHICLE_MANAGER_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include "types.h"
#include "map_service.h"

typedef struct {
    pthread_mutex_t lock;
    EgoVehicle ego;
    VehicleInfo info;
    TurnState turn_state;
    uint8_t vehicle_id;
    bool valid;
} SelfVehicleManager;

bool self_vehicle_manager_init(SelfVehicleManager* manager, uint8_t vehicle_id);
void self_vehicle_manager_destroy(SelfVehicleManager* manager);
void self_vehicle_manager_update_from_can(
    SelfVehicleManager* manager,
    const MapService* map_service,
    const EgoVehicle* ego
);
bool self_vehicle_manager_get_info(const SelfVehicleManager* manager, VehicleInfo* out);
bool self_vehicle_manager_get_ego(const SelfVehicleManager* manager, EgoVehicle* out);
TurnState self_vehicle_manager_get_turn_state(const SelfVehicleManager* manager);
void self_vehicle_manager_set_turn_state(SelfVehicleManager* manager, TurnState state);

#endif
