#ifndef TEMP_APP_CONTEXT_H
#define TEMP_APP_CONTEXT_H

#include <stdatomic.h>
#include <stdint.h>
#include <stdbool.h>
#include "can_handler.h"
#include "mqtt_handler.h"
#include "map_service.h"
#include "self_vehicle_manager.h"
#include "other_vehicle_manager.h"
#include "traffic_light_manager.h"

typedef struct {
    atomic_bool running;
    atomic_bool candidate_vehicle_tx_enabled;
    char mqtt_host[128];
    int mqtt_port;
    uint8_t vehicle_id;
    const char* can_ifname;
    bool can_mock;
    bool can_tx_real;
    CanHandler can;
    MqttHandler mqtt;
    MapService map;
    SelfVehicleManager self;
    OtherVehicleManager others;
    TrafficLightManager traffic_lights;
} AppContext;

#endif
