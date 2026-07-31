#ifndef TEMP_MAP_SERVICE_H
#define TEMP_MAP_SERVICE_H

#include <stdbool.h>
#include "IntersectionMap.h"
#include "types.h"

typedef struct {
    IntersectionMap map;
    bool loaded;
} MapService;

bool map_service_init(MapService* service, const char* xml_path);
bool map_service_query_vehicle_context(
    const MapService* service,
    uint16_t x,
    uint16_t y,
    MapContext* out
);
bool map_service_lane_allows_right(const MapService* service, const char* lanelet_id);
bool map_service_lane_allows_unprotected_left(const MapService* service, const char* lanelet_id);
size_t map_service_get_conflict_zone_ids_for_movement(
    const MapService* service,
    const char* lanelet_id,
    const char* movement,
    const char* out_ids[],
    size_t max_out
);
bool map_service_get_conflict_zone_center(
    const MapService* service,
    const char* conflict_zone_id,
    double* center_x,
    double* center_y
);
uint8_t map_service_parse_lane_num(const char* lanelet_id);
uint8_t map_service_parse_traffic_light_num(const char* tl_id);

#endif
