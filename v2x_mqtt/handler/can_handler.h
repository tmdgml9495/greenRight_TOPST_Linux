#ifndef TEMP_CAN_HANDLER_H
#define TEMP_CAN_HANDLER_H

#include <stdbool.h>
#include <stdint.h>
#include "types.h"

#define CAN_HANDLER_DEFAULT_DEV_PATH "/dev/tcc_ipc_micom"

typedef void (*CanEgoCallback)(const EgoVehicle* ego, void* user_data);

typedef struct {
    CanEgoCallback on_ego;
    void* user_data;
} CanHandlerCallbacks;

typedef struct {
    const char* dev_path;   /* 예: "/dev/tcc_ipc_micom" (SocketCAN ifname 아님) */
    int fd;
    bool rx_mock_mode;
    bool tx_mock_mode;
    uint16_t mock_tick;
    uint16_t tx_tick;       /* TX 프레임 timestamp(12bit)용 free-running 카운터 */
    CanHandlerCallbacks callbacks;
    bool initialized;
} CanHandler;

bool can_handler_init(
    CanHandler* handler,
    const char* dev_path,
    bool rx_mock_mode,
    bool tx_mock_mode,
    const CanHandlerCallbacks* callbacks
);
void can_handler_cleanup(CanHandler* handler);
bool can_handler_poll(CanHandler* handler, int timeout_ms);

/* ===================== TX (CA72 -> MICOM, via IPC device) =====================
 * VehicleInfo / TrafficLight 는 types.h(app_context.h 경유)에 정의되어 있다.
 * (VehicleInfo: speed/x/y/heading, TrafficLight: color/time_left 필드명 확인 완료) */

/* 0100(binary) - Candidate Vehicle Intro */
void can_handler_send_candidate_vehicle_intro(
    CanHandler* handler,
    uint8_t type_mask,
    uint16_t cz_x,
    uint16_t cz_y
);

/* 0101(binary) - Candidate Vehicle Status */
void can_handler_send_candidate_vehicle_status(
    CanHandler* handler,
    uint8_t type_mask,
    const VehicleInfo* vehicle
);
void can_handler_send_no_candidate_vehicle(CanHandler* handler);
void can_handler_send_candidate_vehicle_unavailable(CanHandler* handler);

/* 0110(binary) - Traffic Light / Maneuver Status */
void can_handler_send_traffic_light(
    CanHandler* handler,
    uint8_t tl_id,
    const TrafficLight* traffic_light,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t maneuver
);
void can_handler_send_no_traffic_light(CanHandler* handler, uint16_t cz_x, uint16_t cz_y, uint8_t maneuver);
void can_handler_send_traffic_light_unavailable(CanHandler* handler, uint8_t maneuver);

#endif
