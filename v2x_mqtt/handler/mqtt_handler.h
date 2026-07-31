#ifndef TEMP_MQTT_HANDLER_H
#define TEMP_MQTT_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>
#include <mosquitto.h>
#include "types.h"

typedef void (*MqttVehicleCallback)(const VehicleInfo* vehicle, void* user_data);
typedef void (*MqttTrafficLightCallback)(uint8_t tl_id, const TrafficLight* traffic_light, void* user_data);

typedef struct {
    MqttVehicleCallback on_vehicle;
    MqttTrafficLightCallback on_traffic_light;
    void* user_data;
} MqttHandlerCallbacks;

typedef struct {
    struct mosquitto* mosq;
    uint8_t vehicle_id;
    char status_topic[64];
    MqttHandlerCallbacks callbacks;
    atomic_bool connected;
    bool initialized;
} MqttHandler;

bool mqtt_handler_init(
    MqttHandler* handler,
    const char* host,
    int port,
    uint8_t vehicle_id,
    const MqttHandlerCallbacks* callbacks
);
void mqtt_handler_cleanup(MqttHandler* handler);
bool mqtt_handler_publish_vehicle_info(MqttHandler* handler, const VehicleInfo* vehicle);
bool mqtt_handler_is_connected(const MqttHandler* handler);

#endif
