#ifndef INTERSECTION_MAP_H
#define INTERSECTION_MAP_H

#include <stdbool.h>
#include <stddef.h>

#define MAP_MAX_ID_LEN            32
#define MAP_MAX_POINTS            256
#define MAP_MAX_LANELETS          32
#define MAP_MAX_AREAS             32
#define MAP_MAX_TRAFFIC_LIGHTS    16
#define MAP_MAX_CONFLICT_ZONES    16
#define MAP_MAX_POLYGON_POINTS    8
#define MAP_MAX_CZ_PER_LANE       8
#define MAP_MAX_CONFLICT_RELATIONS 32
#define MAP_MAX_CZ_PARTICIPANTS    4

typedef enum {
    MAP_QUERY_NOT_FOUND = 0,
    MAP_QUERY_LANE,
    MAP_QUERY_INTERSECTION_CENTER
} MapQueryResult;

typedef struct {
    char id[MAP_MAX_ID_LEN];
    double x;
    double y;
} MapPoint;

typedef struct {
    char id[MAP_MAX_ID_LEN];

    int point_count;
    MapPoint points[MAP_MAX_POLYGON_POINTS];
} MapPolygon;

typedef struct {
    char id[MAP_MAX_ID_LEN];

    int point_count;
    MapPoint area_points[MAP_MAX_POLYGON_POINTS];

    int travel_heading_deg;
    char maneuver[MAP_MAX_ID_LEN];
    bool unprotected_left;

    char traffic_light_id[MAP_MAX_ID_LEN];

    int conflict_zone_count;
    char conflict_zone_ids[MAP_MAX_CZ_PER_LANE][MAP_MAX_ID_LEN];
} MapLanelet;

typedef struct {
    char id[MAP_MAX_ID_LEN];

    int point_count;
    MapPoint area_points[MAP_MAX_POLYGON_POINTS];
} MapArea;

typedef struct {
    char lanelet_id[MAP_MAX_ID_LEN];
    char movement[MAP_MAX_ID_LEN];
} MapConflictParticipant;

typedef struct {
    char id[MAP_MAX_ID_LEN];
    char conflict_zone_id[MAP_MAX_ID_LEN];

    int participant_count;
    MapConflictParticipant participants[MAP_MAX_CZ_PARTICIPANTS];
} MapConflictRelation;

typedef struct {
    char id[MAP_MAX_ID_LEN];

    double width;
    double height;

    int lanelet_count;
    MapLanelet lanelets[MAP_MAX_LANELETS];

    int area_count;
    MapArea areas[MAP_MAX_AREAS];

    int traffic_light_count;
    MapPolygon traffic_lights[MAP_MAX_TRAFFIC_LIGHTS];

    int conflict_zone_count;
    MapArea conflict_zones[MAP_MAX_CONFLICT_ZONES];

    int conflict_relation_count;
    MapConflictRelation conflict_relations[MAP_MAX_CONFLICT_RELATIONS];

    char intersection_center_id[MAP_MAX_ID_LEN];
} IntersectionMap;

/*
 * XML 파일을 파싱해서 IntersectionMap 구조체에 저장한다.
 * 성공 true, 실패 false.
 */
bool intersection_map_load_xml(IntersectionMap* map, const char* xml_path);

/*
 * x, y 좌표가 어느 차선에 속하는지 조회한다.
 *
 * 반환:
 * - MAP_QUERY_LANE                : lane_id_out에 차선 ID 저장
 * - MAP_QUERY_INTERSECTION_CENTER : 교차로 중심부
 * - MAP_QUERY_NOT_FOUND           : 맵 밖 또는 해당 차선 없음
 */
MapQueryResult intersection_map_get_lane_id(
    const IntersectionMap* map,
    double x,
    double y,
    const char** lane_id_out
);

/*
 * lane_id에 연결된 conflict zone ID 목록을 조회한다.
 *
 * 반환값:
 * - conflict zone 개수
 * - 없으면 0
 */
size_t intersection_map_get_conflict_zone_ids(
    const IntersectionMap* map,
    const char* lane_id,
    const char* out_ids[],
    size_t max_out
);

/*
 * lane_id와 movement 조합에 연결된 conflict zone ID 목록을 조회한다.
 * movement 예: "straight", "right_turn", "left_turn"
 */
size_t intersection_map_get_conflict_zone_ids_for_movement(
    const IntersectionMap* map,
    const char* lane_id,
    const char* movement,
    const char* out_ids[],
    size_t max_out
);

/*
 * lane_id에 연결된 traffic light ID를 조회한다.
 *
 * 반환:
 * - 있으면 "TL1" 같은 문자열
 * - 없으면 NULL
 */
const char* intersection_map_get_traffic_light_id(
    const IntersectionMap* map,
    const char* lane_id
);

/*
 * 특정 좌표가 교차로 중심부인지 확인한다.
 */
bool intersection_map_is_in_intersection_center(
    const IntersectionMap* map,
    double x,
    double y
);

#endif