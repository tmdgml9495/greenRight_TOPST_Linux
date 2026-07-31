#include "can_tx_thread.h"

#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define CANDIDATE_TX_PERIOD_MS 50
#define CANDIDATE_STATUS_TX_REAL_PERIOD_MS 1000
#define TRAFFIC_LIGHT_TX_REAL_PERIOD_MS 300
#define CANDIDATE_ID_NONE VEHICLE_ID_NONE
#define TYPE_MASK_NONE 0
#define TYPE_MASK_RIGHT_VS_STRAIGHT 1
#define TYPE_MASK_RIGHT_VS_LEFT 2
#define TYPE_MASK_LEFT_VS_STRAIGHT 4
#define TYPE_MASK_LEFT_VS_RIGHT 8
#define MANEUVER_STRAIGHT 0
#define MANEUVER_RIGHT_TURN 1
#define MANEUVER_UNPROTECTED_LEFT 2
#define MANEUVER_LEFT_TURN 3

typedef struct {
    VehicleInfo vehicle;
    uint8_t type_mask;
    char conflict_zone_id[VEHICLE_INFO_ID_LEN];
    uint16_t conflict_zone_center_x;
    uint16_t conflict_zone_center_y;
} CandidateSelection;

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static bool vehicle_has_conflict_zone(const VehicleInfo* vehicle, const char* conflict_zone_id)
{
    if (!vehicle || !conflict_zone_id || conflict_zone_id[0] == '\0') return false;

    for (uint8_t i = 0; i < vehicle->conflict_zone_count && i < VEHICLE_INFO_MAX_CONFLICT_ZONES; i++) {
        if (strcmp(vehicle->conflict_zone_ids[i], conflict_zone_id) == 0) {
            return true;
        }
    }
    return false;
}

static bool find_shared_conflict_zone(const VehicleInfo* self, const VehicleInfo* other, char* out_id, size_t out_size)
{
    if (!self || !other) return false;

    for (uint8_t i = 0; i < self->conflict_zone_count && i < VEHICLE_INFO_MAX_CONFLICT_ZONES; i++) {
        if (vehicle_has_conflict_zone(other, self->conflict_zone_ids[i])) {
            if (out_id && out_size > 0) {
                snprintf(out_id, out_size, "%s", self->conflict_zone_ids[i]);
            }
            return true;
        }
    }
    return false;
}

static uint32_t distance_squared(const VehicleInfo* a, const VehicleInfo* b)
{
    int32_t dx = (int32_t)a->x - (int32_t)b->x;
    int32_t dy = (int32_t)a->y - (int32_t)b->y;
    return (uint32_t)(dx * dx + dy * dy);
}

static bool is_turn_state(const VehicleInfo* vehicle, const char* state)
{
    return vehicle && state && strcmp(vehicle->turn_state, state) == 0;
}

static bool is_protected_left_turn_state(const VehicleInfo* vehicle)
{
    return is_turn_state(vehicle, "left_turn");
}

static uint8_t maneuver_code_from_self(const VehicleInfo* self)
{
    if (!self) return MANEUVER_STRAIGHT;
    if (is_turn_state(self, "right_turn")) return MANEUVER_RIGHT_TURN;
    if (is_turn_state(self, "unprotected_left")) return MANEUVER_UNPROTECTED_LEFT;
    if (is_turn_state(self, "left_turn")) return MANEUVER_LEFT_TURN;
    return MANEUVER_STRAIGHT;
}

static bool compute_type_mask(const VehicleInfo* self, const VehicleInfo* other, uint8_t* out_type_mask)
{
    if (!self || !other || !out_type_mask) return false;

    if (is_turn_state(self, "right_turn")) {
        if (is_turn_state(other, "straight")) {
            *out_type_mask = TYPE_MASK_RIGHT_VS_STRAIGHT;
            return true;
        }
        if (is_protected_left_turn_state(other)) {
            *out_type_mask = TYPE_MASK_RIGHT_VS_LEFT;
            return true;
        }
        return false;
    }

    if (is_turn_state(self, "unprotected_left")) {
        if (is_turn_state(other, "straight")) {
            *out_type_mask = TYPE_MASK_LEFT_VS_STRAIGHT;
            return true;
        }
        if (is_turn_state(other, "right_turn")) {
            *out_type_mask = TYPE_MASK_LEFT_VS_RIGHT;
            return true;
        }
        return false;
    }

    return false;
}

static bool select_candidate_vehicle(AppContext* context, const VehicleInfo* self, CandidateSelection* out)
{
    VehicleInfo others[MAX_OTHER_VEHICLES];
    int count = other_vehicle_manager_copy_valid(&context->others, others, MAX_OTHER_VEHICLES);

    bool found = false;
    uint32_t best_distance = UINT32_MAX;
    CandidateSelection best;
    memset(&best, 0, sizeof(best));

    for (int i = 0; i < count; i++) {
        if (others[i].vehicle_id == self->vehicle_id) {
            continue;
        }

        char shared_conflict_zone_id[VEHICLE_INFO_ID_LEN] = {0};
        if (!find_shared_conflict_zone(self, &others[i], shared_conflict_zone_id, sizeof(shared_conflict_zone_id))) {
            continue;
        }

        uint8_t type_mask = TYPE_MASK_NONE;
        if (!compute_type_mask(self, &others[i], &type_mask)) {
            continue;
        }

        double center_x = 0.0;
        double center_y = 0.0;
        if (!map_service_get_conflict_zone_center(&context->map, shared_conflict_zone_id, &center_x, &center_y)) {
            continue;
        }

        uint32_t dist = distance_squared(self, &others[i]);
        if (!found || dist < best_distance) {
            best.vehicle = others[i];
            best.type_mask = type_mask;
            snprintf(best.conflict_zone_id, sizeof(best.conflict_zone_id), "%s", shared_conflict_zone_id);
            best.conflict_zone_center_x = (uint16_t)center_x;
            best.conflict_zone_center_y = (uint16_t)center_y;
            best_distance = dist;
            found = true;
        }
    }

    if (found && out) {
        *out = best;
    }
    return found;
}

static bool select_farthest_self_conflict_zone_center(
    AppContext* context,
    const VehicleInfo* self,
    uint16_t* out_x,
    uint16_t* out_y
)
{
    if (!context || !self || !out_x || !out_y) return false;

    bool found = false;
    uint32_t best_distance = 0;
    uint16_t best_x = 0;
    uint16_t best_y = 0;

    for (uint8_t i = 0; i < self->conflict_zone_count && i < VEHICLE_INFO_MAX_CONFLICT_ZONES; i++) {
        double center_x = 0.0;
        double center_y = 0.0;
        if (!map_service_get_conflict_zone_center(&context->map, self->conflict_zone_ids[i], &center_x, &center_y)) {
            continue;
        }

        int32_t dx = (int32_t)((uint16_t)center_x) - (int32_t)self->x;
        int32_t dy = (int32_t)((uint16_t)center_y) - (int32_t)self->y;
        uint32_t dist = (uint32_t)(dx * dx + dy * dy);
        if (!found || dist > best_distance) {
            best_distance = dist;
            best_x = (uint16_t)center_x;
            best_y = (uint16_t)center_y;
            found = true;
        }
    }

    if (!found) return false;
    *out_x = best_x;
    *out_y = best_y;
    return true;
}

static uint8_t select_candidate_traffic_light_id(const VehicleInfo* self)
{
    if (!self) return INVALID_ID;
    return map_service_parse_traffic_light_num(self->linked_tl_id);
}

static void send_candidate_traffic_light(AppContext* context, const VehicleInfo* self, const CandidateSelection* vehicle_selection)
{
    uint8_t maneuver = maneuver_code_from_self(self);

    if (!context) {
        return;
    }

    if (!mqtt_handler_is_connected(&context->mqtt)) {
        can_handler_send_traffic_light_unavailable(&context->can, maneuver);
        return;
    }

    if (!self) {
        can_handler_send_no_traffic_light(&context->can, 0, 0, maneuver);
        return;
    }

    uint16_t cz_x = 0;
    uint16_t cz_y = 0;
    select_farthest_self_conflict_zone_center(context, self, &cz_x, &cz_y);

    (void)vehicle_selection;
    uint8_t tl_id = select_candidate_traffic_light_id(self);
    traffic_light_manager_select_candidate(&context->traffic_lights, tl_id);

    TrafficLight traffic_light;
    uint8_t candidate_tl_id = INVALID_ID;
    if (!traffic_light_manager_get_candidate(&context->traffic_lights, &candidate_tl_id, &traffic_light)) {
        can_handler_send_no_traffic_light(&context->can, cz_x, cz_y, maneuver);
        return;
    }

    can_handler_send_traffic_light(&context->can, candidate_tl_id, &traffic_light, cz_x, cz_y, maneuver);
}

static void* can_tx_thread_main(void* arg)
{
    AppContext* context = (AppContext*)arg;
    bool last_active = false;
    bool last_had_candidate = false;
    uint8_t last_intro_vehicle_id = CANDIDATE_ID_NONE;
    uint32_t candidate_status_elapsed_ms = CANDIDATE_STATUS_TX_REAL_PERIOD_MS;
    uint32_t traffic_light_elapsed_ms = TRAFFIC_LIGHT_TX_REAL_PERIOD_MS;

    while (atomic_load(&context->running)) {
        bool active = atomic_load(&context->candidate_vehicle_tx_enabled);
        bool status_due = !context->can_tx_real ||
            candidate_status_elapsed_ms >= CANDIDATE_STATUS_TX_REAL_PERIOD_MS;
        bool traffic_light_due = !context->can_tx_real ||
            traffic_light_elapsed_ms >= TRAFFIC_LIGHT_TX_REAL_PERIOD_MS;
        VehicleInfo self;
        CandidateSelection selection;
        memset(&self, 0, sizeof(self));
        memset(&selection, 0, sizeof(selection));
        bool has_self = self_vehicle_manager_get_info(&context->self, &self);
        bool has_candidate = false;

        if (!active) {
            if (last_active) {
                can_handler_send_no_candidate_vehicle(&context->can);
                other_vehicle_manager_set_candidate(&context->others, NULL);
                printf("[can_tx_thread] candidate vehicle tx stopped\n");
                last_active = false;
                last_had_candidate = false;
                last_intro_vehicle_id = CANDIDATE_ID_NONE;
                candidate_status_elapsed_ms = CANDIDATE_STATUS_TX_REAL_PERIOD_MS;
            }

            if (traffic_light_due) {
                send_candidate_traffic_light(context, has_self ? &self : NULL, NULL);
                traffic_light_elapsed_ms = 0;
            }
            sleep_ms(CANDIDATE_TX_PERIOD_MS);
            candidate_status_elapsed_ms += CANDIDATE_TX_PERIOD_MS;
            traffic_light_elapsed_ms += CANDIDATE_TX_PERIOD_MS;
            continue;
        }

        if (!last_active) {
            printf("[can_tx_thread] candidate vehicle tx started\n");
            last_active = true;
            last_had_candidate = false;
            last_intro_vehicle_id = CANDIDATE_ID_NONE;
            candidate_status_elapsed_ms = CANDIDATE_STATUS_TX_REAL_PERIOD_MS;
            traffic_light_elapsed_ms = TRAFFIC_LIGHT_TX_REAL_PERIOD_MS;
            status_due = true;
            traffic_light_due = true;
        }

        bool mqtt_connected = mqtt_handler_is_connected(&context->mqtt);

        if (!mqtt_connected) {
            other_vehicle_manager_set_candidate(&context->others, NULL);
            if (status_due || last_had_candidate) {
                can_handler_send_candidate_vehicle_unavailable(&context->can);
                candidate_status_elapsed_ms = 0;
            }
            last_intro_vehicle_id = CANDIDATE_ID_NONE;
            last_had_candidate = false;
        } else if (has_self) {
            has_candidate = select_candidate_vehicle(context, &self, &selection);

            if (has_candidate) {
                other_vehicle_manager_set_candidate(&context->others, &selection.vehicle);
                if (selection.vehicle.vehicle_id != last_intro_vehicle_id) {
                    can_handler_send_candidate_vehicle_intro(
                        &context->can,
                        selection.type_mask,
                        selection.conflict_zone_center_x,
                        selection.conflict_zone_center_y
                    );
                    last_intro_vehicle_id = selection.vehicle.vehicle_id;
                    status_due = true;
                }
                if (status_due) {
                    can_handler_send_candidate_vehicle_status(&context->can, selection.type_mask, &selection.vehicle);
                    candidate_status_elapsed_ms = 0;
                }
                last_had_candidate = true;
            } else {
                other_vehicle_manager_set_candidate(&context->others, NULL);
                if (status_due || last_had_candidate) {
                    can_handler_send_no_candidate_vehicle(&context->can);
                    candidate_status_elapsed_ms = 0;
                }
                last_intro_vehicle_id = CANDIDATE_ID_NONE;
                last_had_candidate = false;
            }
        } else {
            other_vehicle_manager_set_candidate(&context->others, NULL);
            if (status_due || last_had_candidate) {
                can_handler_send_no_candidate_vehicle(&context->can);
                candidate_status_elapsed_ms = 0;
            }
            last_intro_vehicle_id = CANDIDATE_ID_NONE;
            last_had_candidate = false;
        }

        if (traffic_light_due) {
            send_candidate_traffic_light(context, has_self ? &self : NULL, has_candidate ? &selection : NULL);
            traffic_light_elapsed_ms = 0;
        }
        sleep_ms(CANDIDATE_TX_PERIOD_MS);
        candidate_status_elapsed_ms += CANDIDATE_TX_PERIOD_MS;
        traffic_light_elapsed_ms += CANDIDATE_TX_PERIOD_MS;
    }
    return NULL;
}

int can_tx_thread_start(pthread_t* thread, AppContext* context)
{
    if (!thread || !context) return -1;
    return pthread_create(thread, NULL, can_tx_thread_main, context);
}
