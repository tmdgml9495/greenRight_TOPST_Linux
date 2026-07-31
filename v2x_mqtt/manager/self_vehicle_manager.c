#include "self_vehicle_manager.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, dst_size, "%s", src);
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static const char* turn_state_to_string(TurnState state)
{
    switch (state) {
        case TURN_STATE_RIGHT_TURN:
            return "right_turn";
        case TURN_STATE_LEFT_TURN:
            return "left_turn";
        case TURN_STATE_UNPROTECTED_LEFT:
            return "unprotected_left";
        case TURN_STATE_STRAIGHT:
        default:
            return "straight";
    }
}

static const char* movement_from_turn_state(TurnState state)
{
    switch (state) {
        case TURN_STATE_RIGHT_TURN:
            return "right_turn";
        case TURN_STATE_LEFT_TURN:
        case TURN_STATE_UNPROTECTED_LEFT:
            return "left_turn";
        case TURN_STATE_STRAIGHT:
        default:
            return "straight";
    }
}

static void clear_conflict_zones(VehicleInfo* info)
{
    if (!info) return;
    info->conflict_zone_count = 0;
    for (uint8_t i = 0; i < VEHICLE_INFO_MAX_CONFLICT_ZONES; i++) {
        info->conflict_zone_ids[i][0] = '\0';
    }
}

static void fill_map_fields(VehicleInfo* info, const MapContext* context)
{
    safe_copy(info->lanelet_id, sizeof(info->lanelet_id), context->found ? context->lanelet_id : "");
    safe_copy(info->linked_tl_id, sizeof(info->linked_tl_id), context->found ? context->traffic_light_id : "");
}

static void fill_conflict_zones_from_lane(
    VehicleInfo* info,
    const MapService* map_service,
    const char* lanelet_id,
    TurnState turn_state
)
{
    clear_conflict_zones(info);

    if (!info || !map_service || !lanelet_id || lanelet_id[0] == '\0') return;

    const char* cz_ids[VEHICLE_INFO_MAX_CONFLICT_ZONES];
    size_t cz_count = map_service_get_conflict_zone_ids_for_movement(
        map_service,
        lanelet_id,
        movement_from_turn_state(turn_state),
        cz_ids,
        VEHICLE_INFO_MAX_CONFLICT_ZONES
    );

    info->conflict_zone_count = (uint8_t)cz_count;
    for (size_t i = 0; i < cz_count; i++) {
        safe_copy(info->conflict_zone_ids[i], sizeof(info->conflict_zone_ids[i]), cz_ids[i]);
    }
}

static bool conflict_zone_is_behind_vehicle(
    const MapService* map_service,
    const char* conflict_zone_id,
    uint16_t vehicle_x,
    uint16_t vehicle_y,
    uint16_t heading_deg
)
{
    const double passed_threshold_cm = 10.0;
    const double pi = 3.14159265358979323846;

    double center_x = 0.0;
    double center_y = 0.0;
    if (!map_service_get_conflict_zone_center(map_service, conflict_zone_id, &center_x, &center_y)) {
        return false;
    }

    double heading_rad = ((double)heading_deg) * pi / 180.0;
    double dir_x = sin(heading_rad);
    double dir_y = cos(heading_rad);
    double to_cz_x = center_x - (double)vehicle_x;
    double to_cz_y = center_y - (double)vehicle_y;
    double projection = to_cz_x * dir_x + to_cz_y * dir_y;

    return projection < -passed_threshold_cm;
}

static void filter_existing_conflict_zones_by_projection(
    VehicleInfo* info,
    const MapService* map_service
)
{
    if (!info || !map_service) return;

    char kept_ids[VEHICLE_INFO_MAX_CONFLICT_ZONES][VEHICLE_INFO_ID_LEN];
    uint8_t kept_count = 0;
    memset(kept_ids, 0, sizeof(kept_ids));

    for (uint8_t i = 0; i < info->conflict_zone_count && i < VEHICLE_INFO_MAX_CONFLICT_ZONES; i++) {
        const char* cz_id = info->conflict_zone_ids[i];
        if (cz_id[0] == '\0') continue;

        if (!conflict_zone_is_behind_vehicle(map_service, cz_id, info->x, info->y, info->heading)) {
            safe_copy(kept_ids[kept_count], sizeof(kept_ids[kept_count]), cz_id);
            kept_count++;
        }
    }

    clear_conflict_zones(info);
    info->conflict_zone_count = kept_count;
    for (uint8_t i = 0; i < kept_count; i++) {
        safe_copy(info->conflict_zone_ids[i], sizeof(info->conflict_zone_ids[i]), kept_ids[i]);
    }
}

static void update_conflict_zones(
    VehicleInfo* info,
    const MapService* map_service,
    const MapContext* context,
    TurnState turn_state
)
{
    if (!info || !map_service || !context) return;

    if (context->found && context->lanelet_id[0] != '\0') {
        fill_conflict_zones_from_lane(info, map_service, context->lanelet_id, turn_state);
        return;
    }

    filter_existing_conflict_zones_by_projection(info, map_service);
}

bool self_vehicle_manager_init(SelfVehicleManager* manager, uint8_t vehicle_id)
{
    if (!manager) return false;
    memset(manager, 0, sizeof(*manager));
    pthread_mutex_init(&manager->lock, NULL);
    manager->vehicle_id = vehicle_id;
    manager->info.vehicle_id = vehicle_id;
    safe_copy(manager->info.turn_state, sizeof(manager->info.turn_state), "straight");
    manager->turn_state = TURN_STATE_STRAIGHT;
    return true;
}

void self_vehicle_manager_destroy(SelfVehicleManager* manager)
{
    if (!manager) return;
    pthread_mutex_destroy(&manager->lock);
}

void self_vehicle_manager_update_from_can(
    SelfVehicleManager* manager,
    const MapService* map_service,
    const EgoVehicle* ego
)
{
    if (!manager || !map_service || !ego) return;

    MapContext context;
    map_service_query_vehicle_context(map_service, ego->x, ego->y, &context);

    pthread_mutex_lock(&manager->lock);
    manager->ego = *ego;
    manager->info.vehicle_id = manager->vehicle_id;
    manager->info.x = ego->x;
    manager->info.y = ego->y;
    manager->info.speed = ego->speed;
    manager->info.heading = ego->heading;
    manager->info.timestamp_ms = monotonic_ms();
    fill_map_fields(&manager->info, &context);
    safe_copy(manager->info.turn_state, sizeof(manager->info.turn_state), turn_state_to_string(manager->turn_state));
    update_conflict_zones(&manager->info, map_service, &context, manager->turn_state);
    manager->valid = true;
    pthread_mutex_unlock(&manager->lock);
}

bool self_vehicle_manager_get_info(const SelfVehicleManager* manager, VehicleInfo* out)
{
    if (!manager || !out) return false;
    pthread_mutex_lock((pthread_mutex_t*)&manager->lock);
    bool valid = manager->valid;
    if (valid) *out = manager->info;
    pthread_mutex_unlock((pthread_mutex_t*)&manager->lock);
    return valid;
}

bool self_vehicle_manager_get_ego(const SelfVehicleManager* manager, EgoVehicle* out)
{
    if (!manager || !out) return false;
    pthread_mutex_lock((pthread_mutex_t*)&manager->lock);
    bool valid = manager->valid;
    if (valid) *out = manager->ego;
    pthread_mutex_unlock((pthread_mutex_t*)&manager->lock);
    return valid;
}

TurnState self_vehicle_manager_get_turn_state(const SelfVehicleManager* manager)
{
    if (!manager) return TURN_STATE_STRAIGHT;
    pthread_mutex_lock((pthread_mutex_t*)&manager->lock);
    TurnState state = manager->turn_state;
    pthread_mutex_unlock((pthread_mutex_t*)&manager->lock);
    return state;
}

void self_vehicle_manager_set_turn_state(SelfVehicleManager* manager, TurnState state)
{
    if (!manager) return;
    pthread_mutex_lock(&manager->lock);
    manager->turn_state = state;
    pthread_mutex_unlock(&manager->lock);
}
