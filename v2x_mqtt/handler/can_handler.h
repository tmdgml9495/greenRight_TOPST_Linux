#ifndef TEMP_CAN_HANDLER_H
#define TEMP_CAN_HANDLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "types.h"

#define CAN_DEMO_FRAME_LOG_ENABLE (0U)

#define CAN_HANDLER_DEFAULT_DEV_PATH "/dev/tcc_ipc_micom"
#define CAN_HANDLER_RX_BUFFER_SIZE 4096u

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
    uint8_t rx_buffer[CAN_HANDLER_RX_BUFFER_SIZE];
    size_t rx_buffer_len;
    uint64_t ego_rx_count;
    uint64_t ego_rx_last_ms;
    uint64_t ego_rx_last_gap_ms;
    uint64_t ego_rx_max_gap_ms;
    uint64_t ego_rx_last_log_ms;
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

/* 0010(binary): 40-bit epoch milliseconds and NTP synchronization state. */
void can_handler_send_ntp_sync(CanHandler* handler);

/* ===================== TX (CA72 -> MICOM, via IPC device) =====================
 * VehicleInfo / TrafficLight 는 types.h(app_context.h 경유)에 정의되어 있다.
 * (VehicleInfo: speed/x/y/heading, TrafficLight: color/time_left 필드명 확인 완료) */

/* 0100(binary) - Candidate Vehicle Intro */
void can_handler_send_candidate_vehicle_intro(
    CanHandler* handler,
    uint8_t type_mask,
    uint16_t cz_x,
    uint16_t cz_y,
    uint64_t timestamp_epoch_ms
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
