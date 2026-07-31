#include "can_handler.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#endif

#define CAN_MSG_ID_EGO_STATUS 0x0
#define CAN_MSG_ID_FILTERED_INTRO   0x4
#define CAN_MSG_ID_FILTERED_STATUS  0x5
#define CAN_MSG_ID_TRAFFIC_LIGHT    0x6

#define TYPE_MASK_V2X_UNAVAILABLE 0x80
#define TL_TYPE_MASK_MQTT_UNAVAILABLE 0x80

#define UPD_BIT_SPEED       (1u << 0)
#define UPD_BIT_X           (1u << 1)
#define UPD_BIT_Y           (1u << 2)
#define UPD_BIT_HEADING     (1u << 3)
#define UPD_BIT_TURN_SIGNAL (1u << 4)

#define CAN_TX_ARBITRATION_ID 0x321

#ifdef __linux__
static void emit_ego(CanHandler* handler, const EgoVehicle* ego)
{
    if (handler && ego && handler->callbacks.on_ego) {
        handler->callbacks.on_ego(ego, handler->callbacks.user_data);
    }
}

static bool decode_ego_status(uint16_t timestamp, uint8_t update_mask, uint64_t payload, const EgoVehicle* previous, bool has_previous, EgoVehicle* ego)
{
    if (!ego) return false;

    if (has_previous && previous) {
        *ego = *previous;
    } else {
        memset(ego, 0, sizeof(*ego));
    }

    if (update_mask & UPD_BIT_SPEED) {
        ego->speed = (uint8_t)((payload >> 32) & 0xFF);
    }
    if (update_mask & UPD_BIT_X) {
        ego->x = (uint16_t)((payload >> 22) & 0x3FF);
    }
    if (update_mask & UPD_BIT_Y) {
        ego->y = (uint16_t)((payload >> 11) & 0x7FF);
    }
    if (update_mask & UPD_BIT_HEADING) {
        ego->heading = (uint16_t)((payload >> 2) & 0x1FF);
    }
    if (update_mask & UPD_BIT_TURN_SIGNAL) {
        ego->turn_signal = (uint8_t)(payload & 0x3);
    }

    ego->timestamp = timestamp;
    return true;
}

static bool decode_can_payload(CanHandler* handler, const uint8_t* data, uint8_t dlc, EgoVehicle* ego)
{
    if (!handler || !data || !ego || dlc < 8) return false;

    uint64_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw = (raw << 8) | data[i];
    }

    uint8_t message_id = (uint8_t)((raw >> 60) & 0xF);
    if (message_id != CAN_MSG_ID_EGO_STATUS) {
        return false;
    }

    uint16_t timestamp = (uint16_t)((raw >> 48) & 0xFFF);
    uint8_t update_mask = (uint8_t)((raw >> 40) & 0xFF);
    uint64_t payload = raw & 0xFFFFFFFFFFULL;
    return decode_ego_status(timestamp, update_mask, payload, &handler->last_ego, handler->has_last_ego, ego);
}

#endif

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000 * 1000;
    nanosleep(&ts, NULL);
}

static bool poll_mock(CanHandler* handler)
{
    sleep_ms(20);

    EgoVehicle ego;
    memset(&ego, 0, sizeof(ego));

    /*
     * CAN_MOCK=1 is used to exercise the Linux decision pipeline without
     * real sensor/CAN input. Keep the mock ego vehicle inside lanelet L4
     * (straight_right, northbound) and hold the right turn signal on so the
     * normal CanRxTask logic enters right_turn mode.
     */
    ego.x = (uint16_t)(205 + (handler->mock_tick % 20));
    ego.y = (uint16_t)(70 + (handler->mock_tick % 35));
    ego.speed = 30;
    ego.heading = 0;
    ego.turn_signal = TURN_SIGNAL_RIGHT;
    ego.timestamp = handler->mock_tick++;

    handler->last_ego = ego;
    handler->has_last_ego = true;

    if (handler->callbacks.on_ego) {
        handler->callbacks.on_ego(&ego, handler->callbacks.user_data);
    }
    return true;
}

#ifdef __linux__
static bool open_socketcan(CanHandler* handler, const char* ifname)
{
    struct sockaddr_can addr;
    struct ifreq ifr;

    handler->fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (handler->fd < 0) {
        perror("[CanHandler] socket failed");
        return false;
    }

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname ? ifname : "can0", IFNAMSIZ - 1);

    if (ioctl(handler->fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("[CanHandler] ioctl(SIOCGIFINDEX) failed");
        close(handler->fd);
        handler->fd = -1;
        return false;
    }

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(handler->fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("[CanHandler] bind failed");
        close(handler->fd);
        handler->fd = -1;
        return false;
    }

    printf("[CanHandler] tx/rx socket opened if=%s\n", ifr.ifr_name);
    return true;
}
#endif

bool can_handler_init(
    CanHandler* handler,
    const char* ifname,
    bool mock_mode,
    const CanHandlerCallbacks* callbacks
)
{
    if (!handler) return false;
    memset(handler, 0, sizeof(*handler));
    handler->ifname = ifname;
    handler->fd = -1;
    handler->mock_mode = mock_mode;
    if (callbacks) {
        handler->callbacks = *callbacks;
    }

    if (mock_mode) {
#ifdef __linux__
        const char* tx_real_env = getenv("CAN_TX_REAL");
        bool tx_real = tx_real_env && strcmp(tx_real_env, "1") == 0;
        if (tx_real && !open_socketcan(handler, ifname)) {
            return false;
        }
#endif
        handler->initialized = true;
        printf(
            "[CanHandler] init if=%s rx_mock=1 tx_real=%s\n",
            ifname ? ifname : "none",
            handler->fd >= 0 ? "1" : "0"
        );
        return true;
    }

#ifndef __linux__
    fprintf(stderr, "[CanHandler] real SocketCAN is only available on Linux.\n");
    return false;
#else
    if (!open_socketcan(handler, ifname)) return false;

    handler->initialized = true;
    printf("[CanHandler] init if=%s rx_real=1 tx_real=1\n", ifname ? ifname : "can0");
    return true;
#endif
}

void can_handler_cleanup(CanHandler* handler)
{
    if (!handler || !handler->initialized) return;
#ifdef __linux__
    if (handler->fd >= 0) {
        close(handler->fd);
        handler->fd = -1;
    }
#endif
    handler->initialized = false;
}

bool can_handler_poll(CanHandler* handler, int timeout_ms)
{
    if (!handler || !handler->initialized) return false;
    if (handler->mock_mode) return poll_mock(handler);

#ifndef __linux__
    (void)timeout_ms;
    return false;
#else
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(handler->fd, &readfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ready = select(handler->fd + 1, &readfds, NULL, NULL, &tv);
    if (ready < 0) {
        if (errno == EINTR) return false;
        perror("[CanHandler] select failed");
        return false;
    }
    if (ready == 0) return false;

    struct can_frame frame;
    ssize_t nbytes = read(handler->fd, &frame, sizeof(frame));
    if (nbytes < 0) {
        if (errno != EINTR) perror("[CanHandler] read failed");
        return false;
    }
    if (nbytes < (ssize_t)sizeof(struct can_frame)) return false;

    EgoVehicle ego;
    if (!decode_can_payload(handler, frame.data, frame.can_dlc, &ego)) return false;
    handler->last_ego = ego;
    handler->has_last_ego = true;
    emit_ego(handler, &ego);
    return true;
#endif
}

static bool can_handler_send_raw_frame(CanHandler* handler, uint8_t message_id, uint8_t update_mask, uint64_t payload40)
{
    if (!handler || !handler->initialized) return false;

    static uint16_t tx_timestamp = 0;
    tx_timestamp = (uint16_t)((tx_timestamp + 1u) & 0x0FFFu);
    uint64_t raw = 0;
    raw |= ((uint64_t)message_id & 0x0Fu) << 60;
    raw |= ((uint64_t)tx_timestamp & 0x0FFFu) << 48;
    raw |= ((uint64_t)update_mask & 0xFFu) << 40;
    raw |= payload40 & 0xFFFFFFFFFFULL;

    if (handler->mock_mode && handler->fd < 0) {
        return true;
    }

#ifndef __linux__
    (void)raw;
    return false;
#else
    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_TX_ARBITRATION_ID;
    frame.can_dlc = 8;
    for (int i = 7; i >= 0; i--) {
        frame.data[i] = (uint8_t)(raw & 0xFFu);
        raw >>= 8;
    }

    ssize_t nbytes = write(handler->fd, &frame, sizeof(frame));
    if (nbytes != (ssize_t)sizeof(frame)) {
        if (nbytes < 0) perror("[CanHandler] tx write failed");
        return false;
    }
    return true;
#endif
}

bool can_handler_send_candidate_vehicle_intro(CanHandler* handler, uint8_t type_mask, uint16_t cz_x, uint16_t cz_y)
{
    uint64_t payload = 0;
    payload |= ((uint64_t)(type_mask & 0xFFu)) << 32;
    payload |= ((uint64_t)(cz_x & 0x03FFu)) << 22;
    payload |= ((uint64_t)(cz_y & 0x07FFu)) << 11;

    uint8_t update_mask = 0x07;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_FILTERED_INTRO, update_mask, payload);
}

bool can_handler_send_candidate_vehicle_status(CanHandler* handler, uint8_t type_mask, const VehicleInfo* candidate)
{
    if (!candidate) {
        return can_handler_send_no_candidate_vehicle(handler);
    }

    uint64_t payload = 0;
    payload |= ((uint64_t)(type_mask & 0xFFu)) << 32;
    payload |= ((uint64_t)(candidate->speed & 0xFFu)) << 24;
    payload |= ((uint64_t)(candidate->x & 0x03FFu)) << 14;
    payload |= ((uint64_t)(candidate->y & 0x07FFu)) << 3;

    uint8_t update_mask = 0x0F;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_FILTERED_STATUS, update_mask, payload);
}

bool can_handler_send_no_candidate_vehicle(CanHandler* handler)
{
    uint8_t update_mask = 0x01;
    uint64_t payload = 0;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_FILTERED_STATUS, update_mask, payload);
}

bool can_handler_send_candidate_vehicle_unavailable(CanHandler* handler)
{
    uint8_t update_mask = 0x01;
    uint64_t payload = 0;
    payload |= ((uint64_t)TYPE_MASK_V2X_UNAVAILABLE) << 32;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_FILTERED_STATUS, update_mask, payload);
}

bool can_handler_send_traffic_light(
    CanHandler* handler,
    uint8_t tl_type_mask,
    const TrafficLight* traffic_light,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t maneuver
)
{
    if (!traffic_light || tl_type_mask == 0) {
        return can_handler_send_no_traffic_light(handler, cz_x, cz_y, maneuver);
    }

    uint64_t payload = 0;
    payload |= ((uint64_t)(tl_type_mask & 0xFFu)) << 32;
    payload |= ((uint64_t)(traffic_light->color & 0x03u)) << 30;
    payload |= ((uint64_t)(traffic_light->time_left & 0x0Fu)) << 26;
    payload |= ((uint64_t)(cz_x & 0x03FFu)) << 16;
    payload |= ((uint64_t)(cz_y & 0x07FFu)) << 5;
    payload |= ((uint64_t)(maneuver & 0x03u)) << 3;

    uint8_t update_mask = 0x3F;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_TRAFFIC_LIGHT, update_mask, payload);
}

bool can_handler_send_no_traffic_light(CanHandler* handler, uint16_t cz_x, uint16_t cz_y, uint8_t maneuver)
{
    uint64_t payload = 0;
    payload |= ((uint64_t)(cz_x & 0x03FFu)) << 16;
    payload |= ((uint64_t)(cz_y & 0x07FFu)) << 5;
    payload |= ((uint64_t)(maneuver & 0x03u)) << 3;

    uint8_t update_mask = 0x21;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_TRAFFIC_LIGHT, update_mask, payload);
}

bool can_handler_send_traffic_light_unavailable(CanHandler* handler, uint8_t maneuver)
{
    uint64_t payload = 0;
    payload |= ((uint64_t)TL_TYPE_MASK_MQTT_UNAVAILABLE) << 32;
    payload |= ((uint64_t)(maneuver & 0x03u)) << 3;

    uint8_t update_mask = 0x21;
    return can_handler_send_raw_frame(handler, CAN_MSG_ID_TRAFFIC_LIGHT, update_mask, payload);
}
