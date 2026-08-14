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
#include "vehicle_publish_queue.h"

#define NTP_SYNC_NORMAL_PERIOD_MS (100U)
#define NTP_SYNC_TEST_PERIOD_MS   (1000U)

typedef struct {
    atomic_bool running;
    atomic_bool candidate_vehicle_tx_enabled;
    atomic_uint ntp_sync_tx_period_ms;
    char mqtt_host[128];
    int mqtt_port;
    uint8_t vehicle_id;
    const char* can_dev_path;
    bool can_rx_mock;
    bool can_tx_mock;
    CanHandler can;
    MqttHandler mqtt;
    MapService map;
    SelfVehicleManager self;
    VehiclePublishQueue self_publish_queue;
    OtherVehicleManager others;
    TrafficLightManager traffic_lights;
} AppContext;

#endif
