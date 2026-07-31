#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <mosquitto.h>

#include "map_service.h"
#include "mqtt_topics.h"
#include "self_vehicle_manager.h"
#include "types.h"
#include "vehicle_codec.h"

#define DEFAULT_VEHICLE_ID 2
#define DEFAULT_SPEED 30
#define DEFAULT_HEADING 90
#define DEFAULT_MAP_PATH "map/intersection_lanelet_v1.xml"
#define PUBLISH_PERIOD_MS 50
#define ROUTE_START_X 30
#define ROUTE_END_X 270
#define ROUTE_STEP_X 4
#define ROUTE_Y 177
#define RESET_PAUSE_MS 500

static volatile sig_atomic_t g_running = 1;

static void handle_signal(int signo)
{
    (void)signo;
    g_running = 0;
}

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static void fill_ego(EgoVehicle* ego, uint16_t x)
{
    memset(ego, 0, sizeof(*ego));
    ego->x = x;
    ego->y = ROUTE_Y;
    ego->speed = DEFAULT_SPEED;
    ego->heading = DEFAULT_HEADING;
    ego->turn_signal = TURN_SIGNAL_NONE;
}

static bool publish_vehicle(struct mosquitto* mqtt, const char* topic, const VehicleInfo* vehicle)
{
    char* payload = vehicle_info_to_json_string(vehicle);
    if (!payload) return false;

    int rc = mosquitto_publish(mqtt, NULL, topic, (int)strlen(payload), payload, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] publish failed: %s\n", mosquitto_strerror(rc));
        free(payload);
        return false;
    }

    printf(
        "[fake_vehicle] pub id=%u x=%u y=%u lane=%s cz_count=%u cz0=%s tl=%s\n",
        vehicle->vehicle_id,
        vehicle->x,
        vehicle->y,
        vehicle->lanelet_id,
        vehicle->conflict_zone_count,
        vehicle->conflict_zone_count > 0 ? vehicle->conflict_zone_ids[0] : "",
        vehicle->linked_tl_id
    );
    free(payload);
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <mqtt_host> <mqtt_port> [vehicle_id] [map_path]\n", argv[0]);
        return 1;
    }

    const char* host = argv[1];
    int port = atoi(argv[2]);
    uint8_t vehicle_id = argc > 3 ? (uint8_t)atoi(argv[3]) : DEFAULT_VEHICLE_ID;
    const char* map_path = argc > 4 ? argv[4] : DEFAULT_MAP_PATH;

    MapService map_service;
    SelfVehicleManager self_manager;
    if (!map_service_init(&map_service, map_path)) {
        fprintf(stderr, "[fake_vehicle] map load failed: %s\n", map_path);
        return 1;
    }
    if (!self_vehicle_manager_init(&self_manager, vehicle_id)) {
        fprintf(stderr, "[fake_vehicle] self vehicle manager init failed\n");
        return 1;
    }
    self_vehicle_manager_set_turn_state(&self_manager, TURN_STATE_STRAIGHT);

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    mosquitto_lib_init();
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "greenright_fake_vehicle_%u", vehicle_id);

    struct mosquitto* mqtt = mosquitto_new(client_id, true, NULL);
    if (!mqtt) {
        fprintf(stderr, "[fake_vehicle] mosquitto_new failed\n");
        mosquitto_lib_cleanup();
        self_vehicle_manager_destroy(&self_manager);
        return 1;
    }

    int rc = mosquitto_connect(mqtt, host, port, 30);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] connect failed: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mqtt);
        mosquitto_lib_cleanup();
        self_vehicle_manager_destroy(&self_manager);
        return 1;
    }

    rc = mosquitto_loop_start(mqtt);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "[fake_vehicle] loop start failed: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mqtt);
        mosquitto_lib_cleanup();
        self_vehicle_manager_destroy(&self_manager);
        return 1;
    }

    char topic[128];
    snprintf(topic, sizeof(topic), TOPIC_VEHICLE_STATUS_FMT, vehicle_id);
    printf("[fake_vehicle] publishing to %s every %d ms\n", topic, PUBLISH_PERIOD_MS);

    uint16_t x = ROUTE_START_X;
    while (g_running) {
        EgoVehicle ego;
        VehicleInfo vehicle;
        fill_ego(&ego, x);
        self_vehicle_manager_update_from_can(&self_manager, &map_service, &ego);
        if (self_vehicle_manager_get_info(&self_manager, &vehicle)) {
            publish_vehicle(mqtt, topic, &vehicle);
        }

        sleep_ms(PUBLISH_PERIOD_MS);

        if (x >= ROUTE_END_X) {
            sleep_ms(RESET_PAUSE_MS);
            x = ROUTE_START_X;
        } else {
            x = (uint16_t)(x + ROUTE_STEP_X);
        }
    }

    mosquitto_loop_stop(mqtt, true);
    mosquitto_disconnect(mqtt);
    mosquitto_destroy(mqtt);
    mosquitto_lib_cleanup();
    self_vehicle_manager_destroy(&self_manager);
    return 0;
}
