#ifndef _VEHICLE_CODEC_H_
#define _VEHICLE_CODEC_H_

#include <cjson/cJSON.h>
#include "types.h"

/* ---- VehicleInfo ---- */

/* VehicleInfo -> 새로 할당된 cJSON 객체 (사용 후 cJSON_Delete 필요) */
cJSON *vehicle_info_to_json(const VehicleInfo *v);

/* VehicleInfo -> malloc 된 JSON 문자열 (사용 후 free 필요), NULL 이면 실패 */
char *vehicle_info_to_json_string(const VehicleInfo *v);

/* cJSON 객체 -> VehicleInfo. 성공 시 true */
bool vehicle_info_from_json(const cJSON *root, VehicleInfo *out);

/* JSON 문자열 -> VehicleInfo. 성공 시 true */
bool vehicle_info_from_json_string(const char *json_str, VehicleInfo *out);

/* VehicleInfo 배열 -> JSON 배열 문자열 (v2x/vehicle/all 브로드캐스트용) */
char *vehicle_info_array_to_json_string(const VehicleInfo *arr, int count);

/* JSON 배열 문자열 -> VehicleInfo 배열. out 은 max_count 크기로 미리 할당되어 있어야 함.
 * 반환값: 실제로 채운 개수 (실패 시 -1) */
int vehicle_info_array_from_json_string(const char *json_str, VehicleInfo *out, int max_count);

/* ---- TrafficLight ---- */

cJSON *traffic_light_to_json(const TrafficLight *tl);
char *traffic_light_to_json_string(const TrafficLight *tl);
bool traffic_light_from_json(const cJSON *root, TrafficLight *out);
bool traffic_light_from_json_string(const char *json_str, TrafficLight *out);

#endif /* _VEHICLE_CODEC_H_ */