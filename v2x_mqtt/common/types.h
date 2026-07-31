#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#define VEHICLE_ID_MIN  1
#define VEHICLE_ID_MAX  63
#define VEHICLE_ID_NONE 0

#define VEHICLE_INFO_ID_LEN 16
#define VEHICLE_INFO_TURN_STATE_LEN 24
#define VEHICLE_INFO_MAX_CONFLICT_ZONES 4
#define MAX_CONFLICT_ZONES VEHICLE_INFO_MAX_CONFLICT_ZONES
#define MAX_OTHER_VEHICLES 64
#define MAX_TRAFFIC_LIGHTS 8
#define INVALID_ID 0xFF

// MQTT / Linux 내부 처리용 차량 정보. Map 객체 ID는 XML의 문자열 ID를 그대로 유지한다.
typedef struct {
    uint8_t vehicle_id;

    uint16_t x;
    uint16_t y;
    uint8_t speed;
    uint16_t heading;

    char lanelet_id[VEHICLE_INFO_ID_LEN];
    char turn_state[VEHICLE_INFO_TURN_STATE_LEN];

    char conflict_zone_ids[VEHICLE_INFO_MAX_CONFLICT_ZONES][VEHICLE_INFO_ID_LEN];
    uint8_t conflict_zone_count;

    char linked_tl_id[VEHICLE_INFO_ID_LEN];

    uint64_t timestamp_ms;
} VehicleInfo;

// CAN message ID 0000 자차 정보 구조체 (RTOS로부터 수신)
typedef struct {
    uint16_t x;              // 10 bit (0~1023)
    uint16_t y;              // 11 bit (0~2047)
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit (0~511)
    uint8_t turn_signal;     // 2bit
    uint16_t timestamp;      // 12bit
} EgoVehicle;

typedef enum {
    TRAFFIC_LIGHT_COLOR_RED = 0,
    TRAFFIC_LIGHT_COLOR_YELLOW = 1,
    TRAFFIC_LIGHT_COLOR_GREEN = 2,
    TRAFFIC_LIGHT_COLOR_UNKNOWN = 255
} TrafficLightColor;

typedef struct {
    uint8_t color;           // TrafficLightColor
    uint8_t time_left;       // 남은 시간
} TrafficLight;

typedef enum {
    DIRECTION_UNKNOWN = 0,
    DIRECTION_STRAIGHT = 1,
    DIRECTION_RIGHT = 2,
    DIRECTION_LEFT = 3,
    DIRECTION_UNPROTECTED_LEFT = 4
} Direction;

typedef enum {
    TURN_SIGNAL_NONE = 0,
    TURN_SIGNAL_RIGHT = 1,
    TURN_SIGNAL_LEFT = 2
} TurnSignal;

typedef enum {
    TURN_STATE_STRAIGHT = 0,
    TURN_STATE_RIGHT_TURN = 1,
    TURN_STATE_LEFT_TURN = 2,
    TURN_STATE_UNPROTECTED_LEFT = 3
} TurnState;

typedef struct {
    bool found;
    bool in_intersection_center;
    char lanelet_id[VEHICLE_INFO_ID_LEN];
    Direction direction;
    bool unprotected_left;
    char traffic_light_id[VEHICLE_INFO_ID_LEN];
    uint8_t conflict_zone_count;
    char conflict_zone_ids[MAX_CONFLICT_ZONES][VEHICLE_INFO_ID_LEN];
} MapContext;

static inline bool is_right_signal(uint8_t turn_signal)
{
    return (turn_signal & TURN_SIGNAL_RIGHT) != 0;
}

static inline bool is_left_signal(uint8_t turn_signal)
{
    return (turn_signal & TURN_SIGNAL_LEFT) != 0;
}

// MSB 프로토콜 변환용 64비트 프레임 (가독성을 위한 공용체/구조체 조합)
typedef union {
    uint64_t raw_data;
    uint8_t bytes[8];
} MSB_Frame;

#endif // _TYPES_H_
