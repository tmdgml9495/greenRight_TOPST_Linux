#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "vehicle_codec.h"

/* ============================ VehicleInfo ============================ */

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    snprintf(dst, dst_size, "%s", src);
}

static bool get_uint_field(const cJSON *root, const char *key, unsigned long long *out)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsNumber(item)) return false;
    *out = (unsigned long long)item->valuedouble;
    return true;
}

static bool get_string_field(const cJSON *root, const char *key, char *out, size_t out_size)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (!cJSON_IsString(item) || !item->valuestring) return false;
    safe_copy(out, out_size, item->valuestring);
    return true;
}

cJSON *vehicle_info_to_json(const VehicleInfo *v)
{
    if (!v) return NULL;

    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddNumberToObject(root, "vehicle_id", v->vehicle_id);
    cJSON_AddNumberToObject(root, "x", v->x);
    cJSON_AddNumberToObject(root, "y", v->y);
    cJSON_AddNumberToObject(root, "speed", v->speed);
    cJSON_AddNumberToObject(root, "heading", v->heading);
    cJSON_AddStringToObject(root, "lanelet_id", v->lanelet_id);
    cJSON_AddStringToObject(root, "turn_state", v->turn_state);

    cJSON *cz_array = cJSON_CreateArray();
    if (!cz_array) {
        cJSON_Delete(root);
        return NULL;
    }

    uint8_t cz_count = v->conflict_zone_count;
    if (cz_count > VEHICLE_INFO_MAX_CONFLICT_ZONES) {
        cz_count = VEHICLE_INFO_MAX_CONFLICT_ZONES;
    }
    for (uint8_t i = 0; i < cz_count; i++) {
        cJSON_AddItemToArray(cz_array, cJSON_CreateString(v->conflict_zone_ids[i]));
    }
    cJSON_AddItemToObject(root, "conflict_zone_ids", cz_array);
    cJSON_AddNumberToObject(root, "conflict_zone_count", cz_count);
    cJSON_AddStringToObject(root, "linked_tl_id", v->linked_tl_id);
    cJSON_AddNumberToObject(root, "timestamp_ms", (double)v->timestamp_ms);

    return root;
}

char *vehicle_info_to_json_string(const VehicleInfo *v)
{
    cJSON *root = vehicle_info_to_json(v);
    if (!root) return NULL;
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}

bool vehicle_info_from_json(const cJSON *root, VehicleInfo *out)
{
    if (!root || !out) return false;
    unsigned long long val;
    memset(out, 0, sizeof(*out));

    if (!get_uint_field(root, "vehicle_id", &val)) return false;
    out->vehicle_id = (uint8_t)val;

    if (!get_uint_field(root, "x", &val)) return false;
    out->x = (uint16_t)val;

    if (!get_uint_field(root, "y", &val)) return false;
    out->y = (uint16_t)val;

    if (!get_uint_field(root, "speed", &val)) return false;
    out->speed = (uint8_t)val;

    if (!get_uint_field(root, "heading", &val)) return false;
    out->heading = (uint16_t)val;

    if (!get_string_field(root, "lanelet_id", out->lanelet_id, sizeof(out->lanelet_id))) return false;
    if (!get_string_field(root, "turn_state", out->turn_state, sizeof(out->turn_state))) return false;

    const cJSON *cz_array = cJSON_GetObjectItemCaseSensitive(root, "conflict_zone_ids");
    if (!cJSON_IsArray(cz_array)) return false;

    uint8_t count = 0;
    const cJSON *item = NULL;
    cJSON_ArrayForEach(item, cz_array) {
        if (count >= VEHICLE_INFO_MAX_CONFLICT_ZONES) break;
        if (!cJSON_IsString(item) || !item->valuestring) return false;
        safe_copy(out->conflict_zone_ids[count], sizeof(out->conflict_zone_ids[count]), item->valuestring);
        count++;
    }
    out->conflict_zone_count = count;

    if (!get_string_field(root, "linked_tl_id", out->linked_tl_id, sizeof(out->linked_tl_id))) return false;

    if (!get_uint_field(root, "timestamp_ms", &val)) return false;
    out->timestamp_ms = (uint64_t)val;

    return true;
}

bool vehicle_info_from_json_string(const char *json_str, VehicleInfo *out)
{
    if (!json_str || !out) return false;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;
    bool ok = vehicle_info_from_json(root, out);
    cJSON_Delete(root);
    return ok;
}

char *vehicle_info_array_to_json_string(const VehicleInfo *arr, int count)
{
    if (!arr && count > 0) return NULL;

    cJSON *jarr = cJSON_CreateArray();
    if (!jarr) return NULL;

    for (int i = 0; i < count; i++) {
        cJSON *item = vehicle_info_to_json(&arr[i]);
        if (!item) {
            cJSON_Delete(jarr);
            return NULL;
        }
        cJSON_AddItemToArray(jarr, item);
    }

    char *str = cJSON_PrintUnformatted(jarr);
    cJSON_Delete(jarr);
    return str;
}

int vehicle_info_array_from_json_string(const char *json_str, VehicleInfo *out, int max_count)
{
    if (!json_str || !out || max_count <= 0) return -1;

    cJSON *jarr = cJSON_Parse(json_str);
    if (!jarr || !cJSON_IsArray(jarr)) {
        if (jarr) cJSON_Delete(jarr);
        return -1;
    }

    int n = 0;
    cJSON *item;
    cJSON_ArrayForEach(item, jarr) {
        if (n >= max_count) break;
        if (vehicle_info_from_json(item, &out[n])) {
            n++;
        }
    }

    cJSON_Delete(jarr);
    return n;
}

/* ============================ TrafficLight ============================ */

static const char* traffic_light_color_to_string(uint8_t color)
{
    switch (color) {
        case TRAFFIC_LIGHT_COLOR_RED:
            return "red";
        case TRAFFIC_LIGHT_COLOR_YELLOW:
            return "yellow";
        case TRAFFIC_LIGHT_COLOR_GREEN:
            return "green";
        default:
            return "unknown";
    }
}

static bool traffic_light_color_from_string(const char* color, uint8_t* out)
{
    if (!color || !out) return false;
    if (strcmp(color, "red") == 0) {
        *out = TRAFFIC_LIGHT_COLOR_RED;
        return true;
    }
    if (strcmp(color, "yellow") == 0) {
        *out = TRAFFIC_LIGHT_COLOR_YELLOW;
        return true;
    }
    if (strcmp(color, "green") == 0) {
        *out = TRAFFIC_LIGHT_COLOR_GREEN;
        return true;
    }
    return false;
}

cJSON *traffic_light_to_json(const TrafficLight *tl)
{
    if (!tl) return NULL;
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;

    cJSON_AddStringToObject(root, "color", traffic_light_color_to_string(tl->color));
    cJSON_AddNumberToObject(root, "time_left", tl->time_left);

    return root;
}

char *traffic_light_to_json_string(const TrafficLight *tl)
{
    cJSON *root = traffic_light_to_json(tl);
    if (!root) return NULL;
    char *str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return str;
}

bool traffic_light_from_json(const cJSON *root, TrafficLight *out)
{
    if (!root || !out) return false;
    unsigned long long val;
    char color[16] = {0};

    if (!get_string_field(root, "color", color, sizeof(color))) return false;
    if (!traffic_light_color_from_string(color, &out->color)) return false;

    if (!get_uint_field(root, "time_left", &val)) return false;
    out->time_left = (uint8_t)val;

    return true;
}

bool traffic_light_from_json_string(const char *json_str, TrafficLight *out)
{
    if (!json_str || !out) return false;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) return false;
    bool ok = traffic_light_from_json(root, out);
    cJSON_Delete(root);
    return ok;
}
