#include "map_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, dst_size, "%s", src);
}

static const MapLanelet* find_lanelet(const MapService* service, const char* lanelet_id)
{
    if (!service || !service->loaded || !lanelet_id) return NULL;
    for (int i = 0; i < service->map.lanelet_count; i++) {
        if (strcmp(service->map.lanelets[i].id, lanelet_id) == 0) {
            return &service->map.lanelets[i];
        }
    }
    return NULL;
}

static const MapArea* find_conflict_zone(const MapService* service, const char* conflict_zone_id)
{
    if (!service || !service->loaded || !conflict_zone_id) return NULL;
    for (int i = 0; i < service->map.conflict_zone_count; i++) {
        if (strcmp(service->map.conflict_zones[i].id, conflict_zone_id) == 0) {
            return &service->map.conflict_zones[i];
        }
    }
    return NULL;
}

static Direction parse_direction(const char* maneuver, bool unprotected_left)
{
    if (!maneuver) return DIRECTION_UNKNOWN;
    if (strcmp(maneuver, "straight_right") == 0) return DIRECTION_RIGHT;
    if (strcmp(maneuver, "straight_left") == 0) {
        return unprotected_left ? DIRECTION_UNPROTECTED_LEFT : DIRECTION_LEFT;
    }
    return DIRECTION_STRAIGHT;
}

bool map_service_init(MapService* service, const char* xml_path)
{
    if (!service || !xml_path) return false;
    memset(service, 0, sizeof(*service));

    service->loaded = intersection_map_load_xml(&service->map, xml_path);
    if (!service->loaded) {
        fprintf(stderr, "[MapService] map load failed: %s\n", xml_path);
        return false;
    }

    printf("[MapService] loaded: lanelets=%d areas=%d tls=%d cz=%d\n",
           service->map.lanelet_count,
           service->map.area_count,
           service->map.traffic_light_count,
           service->map.conflict_zone_count);
    return true;
}

bool map_service_query_vehicle_context(
    const MapService* service,
    uint16_t x,
    uint16_t y,
    MapContext* out
)
{
    if (!service || !service->loaded || !out) return false;
    memset(out, 0, sizeof(*out));

    const char* lane_id = NULL;
    MapQueryResult result = intersection_map_get_lane_id(&service->map, x, y, &lane_id);
    out->in_intersection_center = (result == MAP_QUERY_INTERSECTION_CENTER);

    if (result != MAP_QUERY_LANE || !lane_id) {
        return out->in_intersection_center;
    }

    const MapLanelet* lanelet = find_lanelet(service, lane_id);
    if (!lanelet) return false;

    out->found = true;
    safe_copy(out->lanelet_id, sizeof(out->lanelet_id), lanelet->id);
    out->unprotected_left = lanelet->unprotected_left;
    out->direction = parse_direction(lanelet->maneuver, out->unprotected_left);
    safe_copy(out->traffic_light_id, sizeof(out->traffic_light_id), lanelet->traffic_light_id);

    return true;
}

bool map_service_lane_allows_right(const MapService* service, const char* lanelet_id)
{
    const MapLanelet* lanelet = find_lanelet(service, lanelet_id);
    return lanelet && strcmp(lanelet->maneuver, "straight_right") == 0;
}

bool map_service_lane_allows_unprotected_left(const MapService* service, const char* lanelet_id)
{
    const MapLanelet* lanelet = find_lanelet(service, lanelet_id);
    return lanelet && strcmp(lanelet->maneuver, "straight_left") == 0 && lanelet->unprotected_left;
}

size_t map_service_get_conflict_zone_ids_for_movement(
    const MapService* service,
    const char* lanelet_id,
    const char* movement,
    const char* out_ids[],
    size_t max_out
)
{
    if (!service || !service->loaded) return 0;

    return intersection_map_get_conflict_zone_ids_for_movement(
        &service->map,
        lanelet_id,
        movement,
        out_ids,
        max_out
    );
}

bool map_service_get_conflict_zone_center(
    const MapService* service,
    const char* conflict_zone_id,
    double* center_x,
    double* center_y
)
{
    if (!center_x || !center_y) return false;
    *center_x = 0.0;
    *center_y = 0.0;

    const MapArea* zone = find_conflict_zone(service, conflict_zone_id);
    if (!zone || zone->point_count <= 0) return false;

    double sum_x = 0.0;
    double sum_y = 0.0;
    for (int i = 0; i < zone->point_count; i++) {
        sum_x += zone->area_points[i].x;
        sum_y += zone->area_points[i].y;
    }

    *center_x = sum_x / (double)zone->point_count;
    *center_y = sum_y / (double)zone->point_count;
    return true;
}

uint8_t map_service_parse_lane_num(const char* lanelet_id)
{
    if (!lanelet_id || lanelet_id[0] != 'L') return INVALID_ID;
    long value = strtol(lanelet_id + 1, NULL, 10);
    if (value < 0 || value > 255) return INVALID_ID;
    return (uint8_t)value;
}

uint8_t map_service_parse_traffic_light_num(const char* tl_id)
{
    if (!tl_id || tl_id[0] == '\0') return INVALID_ID;
    if (tl_id[0] == 'T' && tl_id[1] == 'L') {
        long value = strtol(tl_id + 2, NULL, 10);
        if (value >= 0 && value <= 255) return (uint8_t)value;
    }
    return INVALID_ID;
}
