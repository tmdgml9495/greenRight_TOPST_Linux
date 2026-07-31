#ifndef TEMP_CAN_HANDLER_H
#define TEMP_CAN_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

typedef void (*CanEgoCallback)(const EgoVehicle* ego, void* user_data);

typedef struct {
    CanEgoCallback on_ego;
    void* user_data;
} CanHandlerCallbacks;

typedef struct {
    const char* ifname;
    int fd;
    bool mock_mode;
    uint16_t mock_tick;
    EgoVehicle last_ego;
    bool has_last_ego;
    CanHandlerCallbacks callbacks;
    bool initialized;
} CanHandler;

bool can_handler_init(
    CanHandler* handler,
    const char* ifname,
    bool mock_mode,
    const CanHandlerCallbacks* callbacks
);
void can_handler_cleanup(CanHandler* handler);
bool can_handler_poll(CanHandler* handler, int timeout_ms);
bool can_handler_send_candidate_vehicle_intro(CanHandler* handler, uint8_t type_mask, uint16_t cz_x, uint16_t cz_y);
bool can_handler_send_candidate_vehicle_status(CanHandler* handler, uint8_t type_mask, const VehicleInfo* candidate);
bool can_handler_send_no_candidate_vehicle(CanHandler* handler);
bool can_handler_send_candidate_vehicle_unavailable(CanHandler* handler);
bool can_handler_send_traffic_light(
    CanHandler* handler,
    uint8_t tl_type_mask,
    const TrafficLight* traffic_light,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t maneuver
);
bool can_handler_send_no_traffic_light(CanHandler* handler, uint16_t cz_x, uint16_t cz_y, uint8_t maneuver);
bool can_handler_send_traffic_light_unavailable(CanHandler* handler, uint8_t maneuver);

#endif
