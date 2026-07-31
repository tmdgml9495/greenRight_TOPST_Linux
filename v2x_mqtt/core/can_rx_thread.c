#include "can_rx_thread.h"

#include <stdatomic.h>
#include <stdio.h>
#include <time.h>

#define CAN_POLL_TIMEOUT_MS 100
#define CAN_RECONNECT_DELAY_MS 1000

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static TurnState decide_turn_state(
    TurnState current,
    const EgoVehicle* ego,
    const MapContext* map_context
)
{
    if (!ego || !map_context) return current;

    bool signal_off = (ego->turn_signal == TURN_SIGNAL_NONE);
    bool right_signal = (ego->turn_signal == TURN_SIGNAL_RIGHT);
    bool left_signal = (ego->turn_signal == TURN_SIGNAL_LEFT);

    if (current != TURN_STATE_STRAIGHT) {
        return signal_off ? TURN_STATE_STRAIGHT : current;
    }

    if (!map_context->found) {
        return TURN_STATE_STRAIGHT;
    }

    if (right_signal && map_context->direction == DIRECTION_RIGHT) {
        return TURN_STATE_RIGHT_TURN;
    }
    if (left_signal && map_context->direction == DIRECTION_LEFT) {
        return TURN_STATE_LEFT_TURN;
    }
    if (left_signal && map_context->direction == DIRECTION_UNPROTECTED_LEFT) {
        return TURN_STATE_UNPROTECTED_LEFT;
    }

    return TURN_STATE_STRAIGHT;
}

static void on_ego_frame(const EgoVehicle* ego, void* user_data)
{
    AppContext* context = (AppContext*)user_data;
    if (!context || !ego) return;

    MapContext map_context;
    map_service_query_vehicle_context(&context->map, ego->x, ego->y, &map_context);

    TurnState current = self_vehicle_manager_get_turn_state(&context->self);
    TurnState next = decide_turn_state(current, ego, &map_context);
    self_vehicle_manager_set_turn_state(&context->self, next);

    self_vehicle_manager_update_from_can(&context->self, &context->map, ego);

    bool was_candidate_mode =
        current == TURN_STATE_RIGHT_TURN ||
        current == TURN_STATE_UNPROTECTED_LEFT;
    bool is_candidate_mode =
        next == TURN_STATE_RIGHT_TURN ||
        next == TURN_STATE_UNPROTECTED_LEFT;

    if (was_candidate_mode != is_candidate_mode) {
        atomic_store(&context->candidate_vehicle_tx_enabled, is_candidate_mode);
        printf("[can_rx_thread] candidate vehicle tx %s\n", is_candidate_mode ? "enabled" : "disabled");
    }
}

static void* can_rx_thread_main(void* arg)
{
    AppContext* context = (AppContext*)arg;

    CanHandlerCallbacks callbacks = {
        .on_ego = on_ego_frame,
        .user_data = context,
    };

    while (atomic_load(&context->running)) {
        if (!can_handler_init(&context->can, context->can_ifname, context->can_mock, &callbacks)) {
            fprintf(stderr, "[can_rx_thread] can handler init failed, retrying in %d ms\n", CAN_RECONNECT_DELAY_MS);
            sleep_ms(CAN_RECONNECT_DELAY_MS);
            continue;
        }

        while (atomic_load(&context->running) && context->can.initialized) {
            can_handler_poll(&context->can, CAN_POLL_TIMEOUT_MS);
        }

        can_handler_cleanup(&context->can);
    }

    return NULL;
}

int can_rx_thread_start(pthread_t* thread, AppContext* context)
{
    if (!thread || !context) return -1;
    return pthread_create(thread, NULL, can_rx_thread_main, context);
}
