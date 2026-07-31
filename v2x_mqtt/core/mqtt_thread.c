#include "mqtt_thread.h"

#include <stdatomic.h>
#include <stdio.h>
#include <time.h>

#define SELF_PUBLISH_PERIOD_MS 50
#define MQTT_THREAD_SLEEP_MS 10
#define OTHER_VEHICLE_TIMEOUT_MS 500
#define MQTT_RECONNECT_DELAY_MS 1000

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static void on_vehicle_message(const VehicleInfo* vehicle, void* user_data)
{
    AppContext* context = (AppContext*)user_data;
    if (!context || !vehicle) return;
    other_vehicle_manager_update(&context->others, vehicle);
}

static void on_traffic_light_message(uint8_t tl_id, const TrafficLight* traffic_light, void* user_data)
{
    AppContext* context = (AppContext*)user_data;
    if (!context || !traffic_light) return;

    traffic_light_manager_update(&context->traffic_lights, tl_id, traffic_light);
}

static void* mqtt_thread_main(void* arg)
{
    AppContext* context = (AppContext*)arg;

    MqttHandlerCallbacks callbacks = {
        .on_vehicle = on_vehicle_message,
        .on_traffic_light = on_traffic_light_message,
        .user_data = context,
    };

    while (atomic_load(&context->running)) {
        long publish_elapsed = 0;
        long cleanup_elapsed = 0;

        if (!mqtt_handler_init(&context->mqtt, context->mqtt_host, context->mqtt_port, context->vehicle_id, &callbacks)) {
            fprintf(stderr, "[mqtt_thread] mqtt handler init failed, retrying in %d ms\n", MQTT_RECONNECT_DELAY_MS);
            sleep_ms(MQTT_RECONNECT_DELAY_MS);
            continue;
        }

        while (atomic_load(&context->running) && context->mqtt.initialized) {
            publish_elapsed += MQTT_THREAD_SLEEP_MS;
            cleanup_elapsed += MQTT_THREAD_SLEEP_MS;

            if (publish_elapsed >= SELF_PUBLISH_PERIOD_MS) {
                VehicleInfo self;
                if (self_vehicle_manager_get_info(&context->self, &self)) {
                    mqtt_handler_publish_vehicle_info(&context->mqtt, &self);
                }
                publish_elapsed = 0;
            }

            if (cleanup_elapsed >= OTHER_VEHICLE_TIMEOUT_MS) {
                other_vehicle_manager_cleanup_stale(&context->others, OTHER_VEHICLE_TIMEOUT_MS);
                cleanup_elapsed = 0;
            }

            sleep_ms(MQTT_THREAD_SLEEP_MS);
        }

        mqtt_handler_cleanup(&context->mqtt);
    }

    return NULL;
}

int mqtt_thread_start(pthread_t* thread, AppContext* context)
{
    if (!thread || !context) return -1;
    return pthread_create(thread, NULL, mqtt_thread_main, context);
}
