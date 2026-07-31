#include "IntersectionMap.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                           */

static void safe_copy(char* dst, size_t dst_size, const char* src)
{
    if (!dst || dst_size == 0) return;

    if (!src) {
        dst[0] = '\0';
        return;
    }

    snprintf(dst, dst_size, "%s", src);
}

static double dabs_local(double v)
{
    return (v < 0.0) ? -v : v;
}

/*
 * tag_start가 가리키는 XML tag 내부에서 attr_name="..." 값을 읽는다.
 * 예: <lanelet id="L4" maneuver="straight_right">
 */
static bool get_attr_string(
    const char* tag_start,
    const char* attr_name,
    char* out,
    size_t out_size
)
{
    if (!tag_start || !attr_name || !out || out_size == 0) {
        return false;
    }

    const char* tag_end = strchr(tag_start, '>');
    if (!tag_end) return false;

    char pattern[64];
    snprintf(pattern, sizeof(pattern), "%s=\"", attr_name);

    const char* p = tag_start;

    while ((p = strstr(p, pattern)) != NULL) {
        if (p >= tag_end) return false;

        const char* value_start = p + strlen(pattern);
        if (value_start >= tag_end) return false;

        const char* value_end = strchr(value_start, '"');
        if (!value_end || value_end > tag_end) return false;

        size_t len = (size_t)(value_end - value_start);
        if (len >= out_size) len = out_size - 1;

        memcpy(out, value_start, len);
        out[len] = '\0';

        return true;
    }

    return false;
}

static bool get_attr_double(const char* tag_start, const char* attr_name, double* out)
{
    char buf[64];

    if (!out) return false;

    if (!get_attr_string(tag_start, attr_name, buf, sizeof(buf))) {
        return false;
    }

    *out = atof(buf);
    return true;
}

static bool get_attr_int(const char* tag_start, const char* attr_name, int* out)
{
    char buf[64];

    if (!out) return false;

    if (!get_attr_string(tag_start, attr_name, buf, sizeof(buf))) {
        return false;
    }

    *out = atoi(buf);
    return true;
}

static char* read_file_to_buffer(const char* path)
{
    FILE* fp = fopen(path, "rb");
    if (!fp) {
        perror("[IntersectionMap] fopen failed");
        return NULL;
    }

    if (fseek(fp, 0, SEEK_END) != 0) {
        fclose(fp);
        return NULL;
    }

    long size = ftell(fp);
    if (size < 0) {
        fclose(fp);
        return NULL;
    }

    rewind(fp);

    char* buffer = (char*)malloc((size_t)size + 1);
    if (!buffer) {
        fclose(fp);
        return NULL;
    }

    size_t read_size = fread(buffer, 1, (size_t)size, fp);
    buffer[read_size] = '\0';

    fclose(fp);
    return buffer;
}

/* -------------------------------------------------------------------------- */
/* Temporary point table                                                      */

typedef struct {
    char id[MAP_MAX_ID_LEN];
    double x;
    double y;
} PointEntry;

static bool find_point(
    const PointEntry* points,
    int point_count,
    const char* id,
    MapPoint* out
)
{
    if (!points || !id || !out) return false;

    for (int i = 0; i < point_count; i++) {
        if (strcmp(points[i].id, id) == 0) {
            safe_copy(out->id, sizeof(out->id), points[i].id);
            out->x = points[i].x;
            out->y = points[i].y;
            return true;
        }
    }

    return false;
}

/* -------------------------------------------------------------------------- */
/* Geometry                                                                   */

static bool point_on_segment(
    double px,
    double py,
    const MapPoint* a,
    const MapPoint* b
)
{
    const double eps = 1e-9;

    double cross = (px - a->x) * (b->y - a->y) - (py - a->y) * (b->x - a->x);
    if (dabs_local(cross) > eps) {
        return false;
    }

    double dot = (px - a->x) * (b->x - a->x) + (py - a->y) * (b->y - a->y);
    if (dot < -eps) {
        return false;
    }

    double len_sq = (b->x - a->x) * (b->x - a->x) +
                    (b->y - a->y) * (b->y - a->y);

    if (dot - len_sq > eps) {
        return false;
    }

    return true;
}

/*
 * Ray casting 기반 point-in-polygon.
 * 경계선 위에 있는 점도 inside로 처리한다.
 */
static bool point_in_polygon(
    double x,
    double y,
    const MapPoint* polygon,
    int point_count
)
{
    if (!polygon || point_count < 3) {
        return false;
    }

    for (int i = 0; i < point_count; i++) {
        int j = (i + 1) % point_count;

        if (point_on_segment(x, y, &polygon[i], &polygon[j])) {
            return true;
        }
    }

    bool inside = false;

    for (int i = 0, j = point_count - 1; i < point_count; j = i++) {
        double xi = polygon[i].x;
        double yi = polygon[i].y;
        double xj = polygon[j].x;
        double yj = polygon[j].y;

        bool intersect = ((yi > y) != (yj > y)) &&
                         (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi);

        if (intersect) {
            inside = !inside;
        }
    }

    return inside;
}

/* -------------------------------------------------------------------------- */
/* Map search/update helpers                                                  */

static MapLanelet* find_lanelet_mut(IntersectionMap* map, const char* lane_id)
{
    if (!map || !lane_id) return NULL;

    for (int i = 0; i < map->lanelet_count; i++) {
        if (strcmp(map->lanelets[i].id, lane_id) == 0) {
            return &map->lanelets[i];
        }
    }

    return NULL;
}

static const MapLanelet* find_lanelet_const(const IntersectionMap* map, const char* lane_id)
{
    if (!map || !lane_id) return NULL;

    for (int i = 0; i < map->lanelet_count; i++) {
        if (strcmp(map->lanelets[i].id, lane_id) == 0) {
            return &map->lanelets[i];
        }
    }

    return NULL;
}

static void lanelet_add_conflict_zone(MapLanelet* lane, const char* conflict_zone_id)
{
    if (!lane || !conflict_zone_id || conflict_zone_id[0] == '\0') {
        return;
    }

    for (int i = 0; i < lane->conflict_zone_count; i++) {
        if (strcmp(lane->conflict_zone_ids[i], conflict_zone_id) == 0) {
            return;
        }
    }

    if (lane->conflict_zone_count >= MAP_MAX_CZ_PER_LANE) {
        fprintf(stderr,
                "[IntersectionMap] lane %s conflict zone capacity exceeded\n",
                lane->id);
        return;
    }

    safe_copy(
        lane->conflict_zone_ids[lane->conflict_zone_count],
        MAP_MAX_ID_LEN,
        conflict_zone_id
    );

    lane->conflict_zone_count++;
}

static void lanelet_set_traffic_light(MapLanelet* lane, const char* traffic_light_id)
{
    if (!lane || !traffic_light_id || traffic_light_id[0] == '\0') {
        return;
    }

    safe_copy(lane->traffic_light_id, sizeof(lane->traffic_light_id), traffic_light_id);
}

static MapConflictRelation* map_add_conflict_relation(
    IntersectionMap* map,
    const char* relation_id,
    const char* conflict_zone_id
)
{
    if (!map || !relation_id || !conflict_zone_id || conflict_zone_id[0] == '\0') {
        return NULL;
    }

    if (map->conflict_relation_count >= MAP_MAX_CONFLICT_RELATIONS) {
        fprintf(stderr,
                "[IntersectionMap] conflict relation capacity exceeded for %s\n",
                relation_id);
        return NULL;
    }

    MapConflictRelation* relation = &map->conflict_relations[map->conflict_relation_count++];
    memset(relation, 0, sizeof(*relation));
    safe_copy(relation->id, sizeof(relation->id), relation_id);
    safe_copy(relation->conflict_zone_id, sizeof(relation->conflict_zone_id), conflict_zone_id);
    return relation;
}

static void conflict_relation_add_participant(
    MapConflictRelation* relation,
    const char* lanelet_id,
    const char* movement
)
{
    if (!relation || !lanelet_id || lanelet_id[0] == '\0') {
        return;
    }

    if (relation->participant_count >= MAP_MAX_CZ_PARTICIPANTS) {
        fprintf(stderr,
                "[IntersectionMap] conflict relation %s participant capacity exceeded\n",
                relation->id);
        return;
    }

    MapConflictParticipant* participant = &relation->participants[relation->participant_count++];
    safe_copy(participant->lanelet_id, sizeof(participant->lanelet_id), lanelet_id);
    safe_copy(participant->movement, sizeof(participant->movement), movement ? movement : "");
}

/* -------------------------------------------------------------------------- */
/* XML parsing pieces                                                         */

static int parse_points(const char* xml, PointEntry* points, int max_points)
{
    int count = 0;
    const char* p = xml;

    while ((p = strstr(p, "<point ")) != NULL) {
        if (count >= max_points) {
            fprintf(stderr, "[IntersectionMap] point capacity exceeded\n");
            break;
        }

        char id[MAP_MAX_ID_LEN];
        double x = 0.0;
        double y = 0.0;

        if (get_attr_string(p, "id", id, sizeof(id)) &&
            get_attr_double(p, "x", &x) &&
            get_attr_double(p, "y", &y)) {

            safe_copy(points[count].id, sizeof(points[count].id), id);
            points[count].x = x;
            points[count].y = y;
            count++;
        }

        const char* next = strchr(p, '>');
        if (!next) break;
        p = next + 1;
    }

    return count;
}

static void parse_area_point_refs(
    const char* block_start,
    const char* block_end,
    const PointEntry* point_table,
    int point_count,
    MapPoint* out_points,
    int* out_count,
    int max_out
)
{
    const char* p = block_start;
    *out_count = 0;

    while ((p = strstr(p, "<area_point_ref ")) != NULL && p < block_end) {
        if (*out_count >= max_out) {
            fprintf(stderr, "[IntersectionMap] area_point_ref capacity exceeded\n");
            return;
        }

        char point_id[MAP_MAX_ID_LEN];

        if (get_attr_string(p, "id", point_id, sizeof(point_id))) {
            MapPoint resolved;

            if (find_point(point_table, point_count, point_id, &resolved)) {
                out_points[*out_count] = resolved;
                (*out_count)++;
            } else {
                fprintf(stderr,
                        "[IntersectionMap] point not found: %s\n",
                        point_id);
            }
        }

        const char* next = strchr(p, '>');
        if (!next || next >= block_end) break;
        p = next + 1;
    }
}

static void parse_point_refs(
    const char* block_start,
    const char* block_end,
    const PointEntry* point_table,
    int point_count,
    MapPoint* out_points,
    int* out_count,
    int max_out
)
{
    const char* p = block_start;
    *out_count = 0;

    while ((p = strstr(p, "<point_ref ")) != NULL && p < block_end) {
        if (*out_count >= max_out) {
            fprintf(stderr, "[IntersectionMap] point_ref capacity exceeded\n");
            return;
        }

        char point_id[MAP_MAX_ID_LEN];

        if (get_attr_string(p, "id", point_id, sizeof(point_id))) {
            MapPoint resolved;

            if (find_point(point_table, point_count, point_id, &resolved)) {
                out_points[*out_count] = resolved;
                (*out_count)++;
            } else {
                fprintf(stderr,
                        "[IntersectionMap] point not found: %s\n",
                        point_id);
            }
        }

        const char* next = strchr(p, '>');
        if (!next || next >= block_end) break;
        p = next + 1;
    }
}

static void parse_areas(
    IntersectionMap* map,
    const char* xml,
    const PointEntry* point_table,
    int point_count
)
{
    const char* p = xml;

    while ((p = strstr(p, "<area ")) != NULL) {
        const char* end = strstr(p, "</area>");
        if (!end) break;

        char id[MAP_MAX_ID_LEN] = {0};
        char type[MAP_MAX_ID_LEN] = {0};

        get_attr_string(p, "id", id, sizeof(id));
        get_attr_string(p, "type", type, sizeof(type));

        MapArea area;
        memset(&area, 0, sizeof(area));
        safe_copy(area.id, sizeof(area.id), id);

        parse_point_refs(
            p,
            end,
            point_table,
            point_count,
            area.area_points,
            &area.point_count,
            MAP_MAX_POLYGON_POINTS
        );

        if (map->area_count < MAP_MAX_AREAS) {
            map->areas[map->area_count++] = area;
        } else {
            fprintf(stderr, "[IntersectionMap] area capacity exceeded\n");
        }

        if (strcmp(type, "intersection_center") == 0) {
            safe_copy(map->intersection_center_id,
                      sizeof(map->intersection_center_id),
                      id);
        }

        if (strcmp(type, "conflict_zone") == 0) {
            if (map->conflict_zone_count < MAP_MAX_CONFLICT_ZONES) {
                map->conflict_zones[map->conflict_zone_count++] = area;
            } else {
                fprintf(stderr, "[IntersectionMap] conflict zone capacity exceeded\n");
            }
        }

        p = end + strlen("</area>");
    }
}

static void parse_polygons(
    IntersectionMap* map,
    const char* xml,
    const PointEntry* point_table,
    int point_count
)
{
    const char* p = xml;

    while ((p = strstr(p, "<polygon ")) != NULL) {
        const char* end = strstr(p, "</polygon>");
        if (!end) break;

        char id[MAP_MAX_ID_LEN] = {0};
        char type[MAP_MAX_ID_LEN] = {0};

        get_attr_string(p, "id", id, sizeof(id));
        get_attr_string(p, "type", type, sizeof(type));

        if (strcmp(type, "traffic_light") == 0) {
            if (map->traffic_light_count >= MAP_MAX_TRAFFIC_LIGHTS) {
                fprintf(stderr, "[IntersectionMap] traffic light capacity exceeded\n");
                p = end + strlen("</polygon>");
                continue;
            }

            MapPolygon polygon;
            memset(&polygon, 0, sizeof(polygon));
            safe_copy(polygon.id, sizeof(polygon.id), id);

            parse_point_refs(
                p,
                end,
                point_table,
                point_count,
                polygon.points,
                &polygon.point_count,
                MAP_MAX_POLYGON_POINTS
            );

            map->traffic_lights[map->traffic_light_count++] = polygon;
        }

        p = end + strlen("</polygon>");
    }
}

static void parse_lanelets(
    IntersectionMap* map,
    const char* xml,
    const PointEntry* point_table,
    int point_count
)
{
    const char* p = xml;

    while ((p = strstr(p, "<lanelet ")) != NULL) {
        const char* end = strstr(p, "</lanelet>");
        if (!end) break;

        if (map->lanelet_count >= MAP_MAX_LANELETS) {
            fprintf(stderr, "[IntersectionMap] lanelet capacity exceeded\n");
            break;
        }

        MapLanelet lane;
        memset(&lane, 0, sizeof(lane));

        get_attr_string(p, "id", lane.id, sizeof(lane.id));
        get_attr_string(p, "maneuver", lane.maneuver, sizeof(lane.maneuver));
        get_attr_int(p, "travel_heading_deg", &lane.travel_heading_deg);

        char unprotected_left[16] = {0};
        if (get_attr_string(p, "unprotected_left", unprotected_left, sizeof(unprotected_left))) {
            lane.unprotected_left = strcmp(unprotected_left, "true") == 0 ||
                                    strcmp(unprotected_left, "yes") == 0 ||
                                    strcmp(unprotected_left, "1") == 0;
        }

        /*
         * lanelet tag 자체에 traffic_light_ref가 있는 경우 먼저 저장.
         * regulatory_element에서도 한 번 더 보강한다.
         */
        get_attr_string(
            p,
            "traffic_light_ref",
            lane.traffic_light_id,
            sizeof(lane.traffic_light_id)
        );

        parse_area_point_refs(
            p,
            end,
            point_table,
            point_count,
            lane.area_points,
            &lane.point_count,
            MAP_MAX_POLYGON_POINTS
        );

        map->lanelets[map->lanelet_count++] = lane;

        p = end + strlen("</lanelet>");
    }
}

static void parse_regulatory_elements(IntersectionMap* map, const char* xml)
{
    const char* p = xml;

    while ((p = strstr(p, "<regulatory_element ")) != NULL) {
        const char* end = strstr(p, "</regulatory_element>");
        if (!end) break;

        char type[MAP_MAX_ID_LEN] = {0};
        get_attr_string(p, "type", type, sizeof(type));

        if (strcmp(type, "traffic_light") == 0) {
            char tl_id[MAP_MAX_ID_LEN] = {0};

            const char* tl_ref = strstr(p, "<traffic_light_ref ");
            if (tl_ref && tl_ref < end) {
                get_attr_string(tl_ref, "id", tl_id, sizeof(tl_id));
            }

            const char* scan = p;
            while ((scan = strstr(scan, "<applies_to ")) != NULL && scan < end) {
                char lane_id[MAP_MAX_ID_LEN] = {0};

                if (get_attr_string(scan, "lanelet_id", lane_id, sizeof(lane_id))) {
                    MapLanelet* lane = find_lanelet_mut(map, lane_id);
                    if (lane) {
                        lanelet_set_traffic_light(lane, tl_id);
                    }
                }

                const char* next = strchr(scan, '>');
                if (!next || next >= end) break;
                scan = next + 1;
            }
        }
        else if (strcmp(type, "conflict_zone") == 0) {
            char relation_id[MAP_MAX_ID_LEN] = {0};
            char cz_id[MAP_MAX_ID_LEN] = {0};

            get_attr_string(p, "id", relation_id, sizeof(relation_id));

            const char* cz_ref = strstr(p, "<conflict_zone_ref ");
            if (cz_ref && cz_ref < end) {
                get_attr_string(cz_ref, "id", cz_id, sizeof(cz_id));
            }

            MapConflictRelation* relation = NULL;
            const char* scan = p;
            bool has_participant = false;

            while ((scan = strstr(scan, "<participant ")) != NULL && scan < end) {
                char lane_id[MAP_MAX_ID_LEN] = {0};
                char movement[MAP_MAX_ID_LEN] = {0};

                if (get_attr_string(scan, "lanelet_id", lane_id, sizeof(lane_id))) {
                    get_attr_string(scan, "movement", movement, sizeof(movement));

                    if (!relation) {
                        relation = map_add_conflict_relation(map, relation_id, cz_id);
                    }

                    conflict_relation_add_participant(relation, lane_id, movement);

                    MapLanelet* lane = find_lanelet_mut(map, lane_id);
                    if (lane) {
                        lanelet_add_conflict_zone(lane, cz_id);
                    }
                    has_participant = true;
                }

                const char* next = strchr(scan, '>');
                if (!next || next >= end) break;
                scan = next + 1;
            }

            if (!has_participant) {
                scan = p;
                while ((scan = strstr(scan, "<applies_to ")) != NULL && scan < end) {
                    char lane_id[MAP_MAX_ID_LEN] = {0};

                    if (get_attr_string(scan, "lanelet_id", lane_id, sizeof(lane_id))) {
                        MapLanelet* lane = find_lanelet_mut(map, lane_id);
                        if (lane) {
                            lanelet_add_conflict_zone(lane, cz_id);
                        }
                    }

                    const char* next = strchr(scan, '>');
                    if (!next || next >= end) break;
                    scan = next + 1;
                }
            }
        }

        p = end + strlen("</regulatory_element>");
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */

bool intersection_map_load_xml(IntersectionMap* map, const char* xml_path)
{
    if (!map || !xml_path) {
        return false;
    }

    memset(map, 0, sizeof(*map));

    char* xml = read_file_to_buffer(xml_path);
    if (!xml) {
        fprintf(stderr, "[IntersectionMap] failed to read XML: %s\n", xml_path);
        return false;
    }

    /*
     * map tag의 width/height 읽기.
     * 없으면 0으로 남는다.
     */
    const char* map_tag = strstr(xml, "<map ");
    if (map_tag) {
        get_attr_double(map_tag, "width", &map->width);
        get_attr_double(map_tag, "height", &map->height);
    }

    PointEntry point_table[MAP_MAX_POINTS];
    memset(point_table, 0, sizeof(point_table));

    int point_count = parse_points(xml, point_table, MAP_MAX_POINTS);

    parse_areas(map, xml, point_table, point_count);
    parse_polygons(map, xml, point_table, point_count);
    parse_lanelets(map, xml, point_table, point_count);
    parse_regulatory_elements(map, xml);

    free(xml);

    printf("[IntersectionMap] loaded\n");
    printf("  width=%.1f, height=%.1f\n", map->width, map->height);
    printf("  lanelets=%d\n", map->lanelet_count);
    printf("  areas=%d\n", map->area_count);
    printf("  conflict_zones=%d\n", map->conflict_zone_count);
    printf("  conflict_relations=%d\n", map->conflict_relation_count);
    printf("  traffic_lights=%d\n", map->traffic_light_count);
    printf("  intersection_center=%s\n",
           map->intersection_center_id[0] ? map->intersection_center_id : "NONE");

    return true;
}

bool intersection_map_is_in_intersection_center(
    const IntersectionMap* map,
    double x,
    double y
)
{
    if (!map || map->intersection_center_id[0] == '\0') {
        return false;
    }

    for (int i = 0; i < map->area_count; i++) {
        const MapArea* area = &map->areas[i];

        if (strcmp(area->id, map->intersection_center_id) == 0) {
            return point_in_polygon(x, y, area->area_points, area->point_count);
        }
    }

    return false;
}

MapQueryResult intersection_map_get_lane_id(
    const IntersectionMap* map,
    double x,
    double y,
    const char** lane_id_out
)
{
    if (!map || !lane_id_out) {
        return MAP_QUERY_NOT_FOUND;
    }

    *lane_id_out = NULL;

    /*
     * 요구사항:
     * 교차로 중심부일 경우 lane id를 반환하지 않고,
     * MAP_QUERY_INTERSECTION_CENTER를 반환한다.
     */
    if (intersection_map_is_in_intersection_center(map, x, y)) {
        return MAP_QUERY_INTERSECTION_CENTER;
    }

    for (int i = 0; i < map->lanelet_count; i++) {
        const MapLanelet* lane = &map->lanelets[i];

        if (point_in_polygon(x, y, lane->area_points, lane->point_count)) {
            *lane_id_out = lane->id;
            return MAP_QUERY_LANE;
        }
    }

    return MAP_QUERY_NOT_FOUND;
}

size_t intersection_map_get_conflict_zone_ids(
    const IntersectionMap* map,
    const char* lane_id,
    const char* out_ids[],
    size_t max_out
)
{
    if (!map || !lane_id || !out_ids || max_out == 0) {
        return 0;
    }

    const MapLanelet* lane = find_lanelet_const(map, lane_id);
    if (!lane) {
        return 0;
    }

    size_t count = 0;

    for (int i = 0; i < lane->conflict_zone_count && count < max_out; i++) {
        out_ids[count++] = lane->conflict_zone_ids[i];
    }

    return count;
}

size_t intersection_map_get_conflict_zone_ids_for_movement(
    const IntersectionMap* map,
    const char* lane_id,
    const char* movement,
    const char* out_ids[],
    size_t max_out
)
{
    if (!map || !lane_id || !movement || !out_ids || max_out == 0) {
        return 0;
    }

    size_t count = 0;

    for (int i = 0; i < map->conflict_relation_count && count < max_out; i++) {
        const MapConflictRelation* relation = &map->conflict_relations[i];

        for (int j = 0; j < relation->participant_count; j++) {
            const MapConflictParticipant* participant = &relation->participants[j];

            if (strcmp(participant->lanelet_id, lane_id) == 0 &&
                strcmp(participant->movement, movement) == 0) {
                bool exists = false;
                for (size_t k = 0; k < count; k++) {
                    if (strcmp(out_ids[k], relation->conflict_zone_id) == 0) {
                        exists = true;
                        break;
                    }
                }

                if (!exists) {
                    out_ids[count++] = relation->conflict_zone_id;
                }
                break;
            }
        }
    }

    return count;
}

const char* intersection_map_get_traffic_light_id(
    const IntersectionMap* map,
    const char* lane_id
)
{
    if (!map || !lane_id) {
        return NULL;
    }

    const MapLanelet* lane = find_lanelet_const(map, lane_id);
    if (!lane) {
        return NULL;
    }

    if (lane->traffic_light_id[0] == '\0') {
        return NULL;
    }

    return lane->traffic_light_id;
}