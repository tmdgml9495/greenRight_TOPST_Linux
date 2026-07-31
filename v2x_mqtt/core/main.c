#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

#include "app_context.h"
#include "can_rx_thread.h"
#include "can_tx_thread.h"
#include "mqtt_thread.h"

#define DEFAULT_MAP_PATH "map/intersection_lanelet_v1.xml"
#define DEFAULT_MQTT_HOST "127.0.0.1"
#define DEFAULT_MQTT_PORT 1883
#define DEFAULT_VEHICLE_ID 1

static AppContext g_app;

static void handle_signal(int signo)
{
    (void)signo;
    atomic_store(&g_app.running, false);
}

static bool app_init(AppContext* app, int argc, char** argv)
{
    const char* mqtt_host = argc > 1 ? argv[1] : DEFAULT_MQTT_HOST;
    int mqtt_port = argc > 2 ? atoi(argv[2]) : DEFAULT_MQTT_PORT;
    const char* map_path = argc > 3 ? argv[3] : DEFAULT_MAP_PATH;
    uint8_t vehicle_id = argc > 4 ? (uint8_t)atoi(argv[4]) : DEFAULT_VEHICLE_ID;
    const char* can_mock_env = getenv("CAN_MOCK");
    const char* can_tx_real_env = getenv("CAN_TX_REAL");
    bool can_mock = can_mock_env && strcmp(can_mock_env, "1") == 0;
    bool can_tx_real = can_tx_real_env && strcmp(can_tx_real_env, "1") == 0;

    memset(app, 0, sizeof(*app));
    atomic_init(&app->running, true);
    atomic_init(&app->candidate_vehicle_tx_enabled, false);

    if (!map_service_init(&app->map, map_path)) return false;
    if (!self_vehicle_manager_init(&app->self, vehicle_id)) return false;
    if (!other_vehicle_manager_init(&app->others)) return false;
    if (!traffic_light_manager_init(&app->traffic_lights)) return false;
    snprintf(app->mqtt_host, sizeof(app->mqtt_host), "%s", mqtt_host);
    app->mqtt_port = mqtt_port;
    app->vehicle_id = vehicle_id;
    app->can_ifname = "can0";
    app->can_mock = can_mock;
    app->can_tx_real = can_tx_real;

    return true;
}

static void app_cleanup(AppContext* app)
{
    traffic_light_manager_destroy(&app->traffic_lights);
    other_vehicle_manager_destroy(&app->others);
    self_vehicle_manager_destroy(&app->self);
}

int main(int argc, char** argv)
{
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    if (!app_init(&g_app, argc, argv)) {
        fprintf(stderr, "[main] app init failed\n");
        app_cleanup(&g_app);
        return 1;
    }

    pthread_t can_rx_thread;
    pthread_t can_tx_thread;
    pthread_t mqtt_thread;

    if (can_rx_thread_start(&can_rx_thread, &g_app) != 0) {
        fprintf(stderr, "[main] can rx thread start failed\n");
        atomic_store(&g_app.running, false);
    }

    if (mqtt_thread_start(&mqtt_thread, &g_app) != 0) {
        fprintf(stderr, "[main] mqtt thread start failed\n");
        atomic_store(&g_app.running, false);
    }

    if (can_tx_thread_start(&can_tx_thread, &g_app) != 0) {
        fprintf(stderr, "[main] can tx thread start failed\n");
        atomic_store(&g_app.running, false);
    }

    pthread_join(can_rx_thread, NULL);
    pthread_join(can_tx_thread, NULL);
    pthread_join(mqtt_thread, NULL);

    app_cleanup(&g_app);
    return 0;
}
