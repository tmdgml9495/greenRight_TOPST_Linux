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
    bool mock_mode;
    uint16_t mock_tick;
    CanHandlerCallbacks callbacks;
    bool initialized;
} CanHandler;

bool can_handler_init(
    CanHandler* handler,
    const char* dev_path,
    bool mock_mode,
    const CanHandlerCallbacks* callbacks
);
void can_handler_cleanup(CanHandler* handler);
bool can_handler_poll(CanHandler* handler, int timeout_ms);

/* TX 함수들(candidate vehicle / traffic light)은 IPC 경유로 별도 구현 예정 */

#endif
