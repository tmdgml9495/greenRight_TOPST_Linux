#include "can_handler.h"
#include "ipc_frame.h"
#include "ntp_time.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#ifdef __linux__
#include <sys/select.h>
#endif

/* ===================== CAN Frame : Ego Vehicle Status (message_id = 0x0) =====================
 * 64bit frame layout (MSB-first):
 *   [63:60] message_id  (4bit)  = 0x0
 *   [59:48] timestamp   (12bit) - sensing tick, NOT absolute time (freshness/staleness check only)
 *   [47:40] unused      (8bit)
 *   [39:32] speed       (8bit)
 *   [31:22] x           (10bit)
 *   [21:11] y           (11bit)
 *   [10:2]  heading     (9bit)
 *   [1:0]   turn_signal (2bit) : 00=OFF, 01=RIGHT, 10=LEFT, 11=Reserved
 *
 * 매 프레임 전체 상태를 담아 보내므로 update_mask 방식은 사용하지 않는다.
 * turn_signal == Reserved(0x3)는 여기서 걸러내지 않고 그대로 저장한다;
 * 상위 로직(decide_turn_state)이 LEFT/RIGHT만 명시적으로 처리하므로
 * Reserved 값은 자연히 무시된다. */

/* RX 프레임에는 canID 헤더가 없어 더 이상 필터링에 쓰이지 않는다.
 * TX 방향(channel/tx_only/canID 헤더 존재) 구현 시 참고용으로만 보존. */
#define CAN_ID_EGO_STATUS   0x0100u

#define EGO_FRAME_MSG_ID          0x0u
#define EGO_RX_LOG_PERIOD_MS      1000ULL
#define EGO_RX_GAP_WARN_MS        300ULL

#define EGO_SHIFT_TIMESTAMP       48
#define EGO_SHIFT_SPEED           32
#define EGO_SHIFT_X               22
#define EGO_SHIFT_Y               11
#define EGO_SHIFT_HEADING         2
#define EGO_SHIFT_TURN_SIGNAL     0

#define EGO_MASK_TIMESTAMP        0x0FFFu   /* 12bit */
#define EGO_MASK_SPEED            0xFFu     /* 8bit  */
#define EGO_MASK_X                0x3FFu    /* 10bit */
#define EGO_MASK_Y                0x7FFu    /* 11bit */
#define EGO_MASK_HEADING          0x1FFu    /* 9bit  */
#define EGO_MASK_TURN_SIGNAL      0x3u      /* 2bit  */

/* ===================== CAN TX (CA72 -> MICOM, via IPC device) =====================
 * RX와 동일하게 message_id(4bit) + timestamp(12bit) + payload(48bit) = 64bit 구조를 쓴다.
 * message_id 값은 문서상 "0100", "0101", "0110"으로 표기되어 있는데, 이는 4bit 필드의
 * 2진수 표기이므로 각각 0x4, 0x5, 0x6 이다 (RX의 ego status가 0x0인 것과 같은 체계). */

/* TODO(확인 필요): 아래 IPC 목적지 파라미터는 vendor IPC ICD 문서 기준으로 재확인해야 한다.
 *   - channel_bitmask / tx_only_channel_bitmask : ipc_example.c 기본값(channel=1 -> bit0)을
 *     잠정 채택했을 뿐, 실제 프로젝트에서 어떤 채널을 쓰는지 확인 필요.
 *   - canID : IPC 헤더에 들어가는 값으로, RX에는 없던 필드라 실측 기준이 없다.
 *     기존 SocketCAN 시절 CAN_ID_EGO_STATUS(0x0100)를 참고용으로만 남겨뒀던 것과 같은 맥락. */
#define CAN_TX_CHANNEL_BITMASK      0x01u   /* TODO(확인 필요) */
#define CAN_TX_ONLY_CHANNEL_BITMASK 0x00u   /* TODO(확인 필요) */
#define CAN_TX_ID                   0x0101u /* Linux -> VCP; keep VCP EGO status at 0x100. */

#define NTP_SYNC_MSG_ID                 0x2u
#define NTP_SYNC_SHIFT_EPOCH_MS         8
#define NTP_SYNC_MASK_EPOCH_MS          0xFFFFFFFFFFULL

#define CANDIDATE_INTRO_MSG_ID          0x4u
#define CANDIDATE_INTRO_SHIFT_TYPE_MASK 32
#define CANDIDATE_INTRO_SHIFT_CZ_X      22
#define CANDIDATE_INTRO_SHIFT_CZ_Y      11
#define CANDIDATE_INTRO_MASK_TYPE_MASK  0xFFu
#define CANDIDATE_INTRO_MASK_CZ_X       0x3FFu  /* 10bit */
#define CANDIDATE_INTRO_MASK_CZ_Y       0x7FFu   /* 11bit */

#define CANDIDATE_STATUS_MSG_ID          0x5u
#define CANDIDATE_STATUS_SHIFT_TYPE_MASK 40
#define CANDIDATE_STATUS_MASK_TYPE_MASK  0xFFu
#define CANDIDATE_STATUS_TYPE_NONE       0x00u  /* 후보 차량 없음 */
#define CANDIDATE_STATUS_TYPE_COMM_ERROR 0x80u  /* 통신 오류 */
/* speed/x/y/heading 비트 위치는 EGO 프레임과 동일한 레이아웃을 재사용한다
 * (EGO_SHIFT_SPEED/EGO_SHIFT_X/EGO_SHIFT_Y/EGO_SHIFT_HEADING, 아래에 이미 정의됨) */

#define TL_STATUS_MSG_ID           0x6u
#define TL_STATUS_ID_NONE          0x00u  /* traffic light 없음 */
#define TL_STATUS_ID_COMM_ERROR    0x80u  /* 통신 오류 */
#define TL_SHIFT_TL_TYPE_MASK      32
#define TL_SHIFT_COLOR             30
#define TL_SHIFT_TIME_LEFT         26
#define TL_SHIFT_CZ_X              16
#define TL_SHIFT_CZ_Y              5
#define TL_SHIFT_MANEUVER          3
#define TL_MASK_TL_TYPE_MASK       0xFFu
#define TL_MASK_COLOR              0x3u   /* 2bit */
#define TL_MASK_TIME_LEFT          0xFu   /* 4bit */
#define TL_MASK_CZ_X                0x3FFu /* 10bit */
#define TL_MASK_CZ_Y                0x7FFu /* 11bit */
#define TL_MASK_MANEUVER            0x3u   /* 2bit */

static void emit_ego(CanHandler* handler, const EgoVehicle* ego)
{
    if (handler && ego && handler->callbacks.on_ego) {
        handler->callbacks.on_ego(ego, handler->callbacks.user_data);
    }
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

static void emit_ego_with_diagnostics(CanHandler* handler, const EgoVehicle* ego)
{
    uint64_t now_ms = monotonic_ms();
    uint64_t gap_ms = 0;

    if (handler->ego_rx_last_ms != 0) {
        gap_ms = now_ms - handler->ego_rx_last_ms;
        if (gap_ms > handler->ego_rx_max_gap_ms) {
            handler->ego_rx_max_gap_ms = gap_ms;
        }
    }

    handler->ego_rx_count++;
    handler->ego_rx_last_ms = now_ms;
    handler->ego_rx_last_gap_ms = gap_ms;

    /* Update freshness and enqueue before any terminal output can block. */
    emit_ego(handler, ego);

    if (gap_ms > EGO_RX_GAP_WARN_MS) {
        fprintf(stderr, "[CanHandler][RX-EGO GAP] gap=%llums maxGap=%llums count=%llu\n",
                (unsigned long long)gap_ms,
                (unsigned long long)handler->ego_rx_max_gap_ms,
                (unsigned long long)handler->ego_rx_count);
    }

    if (handler->ego_rx_last_log_ms == 0 ||
        now_ms - handler->ego_rx_last_log_ms >= EGO_RX_LOG_PERIOD_MS) {
        printf("[CanHandler][RX-EGO] count=%llu gap=%llums maxGap=%llums "
               "x=%u y=%u speed=%u heading=%u turn_signal=%u timestamp=%u\n",
               (unsigned long long)handler->ego_rx_count,
               (unsigned long long)gap_ms,
               (unsigned long long)handler->ego_rx_max_gap_ms,
               ego->x, ego->y, ego->speed, ego->heading,
               ego->turn_signal, ego->timestamp);
        handler->ego_rx_last_log_ms = now_ms;
    }
}

static bool decode_ego_status(uint16_t timestamp, uint64_t payload48, EgoVehicle* ego)
{
    if (!ego) return false;

    ego->speed       = (uint8_t) ((payload48 >> EGO_SHIFT_SPEED)       & EGO_MASK_SPEED);
    ego->x           = (uint16_t)((payload48 >> EGO_SHIFT_X)           & EGO_MASK_X);
    ego->y           = (uint16_t)((payload48 >> EGO_SHIFT_Y)           & EGO_MASK_Y);
    ego->heading     = (uint16_t)((payload48 >> EGO_SHIFT_HEADING)     & EGO_MASK_HEADING);
    ego->turn_signal = (uint8_t) ((payload48 >> EGO_SHIFT_TURN_SIGNAL) & EGO_MASK_TURN_SIGNAL);
    ego->timestamp   = timestamp;

    return true;
}

/* CAN 8byte payload -> EgoVehicle. message_id nibble이 0x0이 아니면 이 프레임이
 * ego status가 아니라는 뜻이므로 false 반환. */
static bool decode_can_payload(const uint8_t* data, size_t dlc, EgoVehicle* ego)
{
    if (!data || !ego || dlc < 8) return false;

    uint64_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw = (raw << 8) | data[i];
    }

    uint8_t message_id = (uint8_t)((raw >> 60) & 0xFu);
    if (message_id != EGO_FRAME_MSG_ID) {
        return false;
    }

    uint16_t timestamp = (uint16_t)((raw >> EGO_SHIFT_TIMESTAMP) & EGO_MASK_TIMESTAMP);
    uint64_t payload48 = raw & 0xFFFFFFFFFFFFULL;

    return decode_ego_status(timestamp, payload48, ego);
}

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
     * CAN_RX_MOCK=1은 실제 IPC RX 없이 Linux 쪽 판단 로직만 검증하기 위한 모드.
     * 항상 우회전 시그널을 켠 채로 lanelet 안쪽 좌표를 흘려보낸다.
     */
    ego.x = (uint16_t)(255 + (handler->mock_tick % 20));
    ego.y = (uint16_t)(70 + (handler->mock_tick % 35));
    ego.speed = 30;
    ego.heading = 0;
    ego.turn_signal = TURN_SIGNAL_RIGHT;
    ego.timestamp = handler->mock_tick++;

    if (handler->callbacks.on_ego) {
        handler->callbacks.on_ego(&ego, handler->callbacks.user_data);
    }
    return true;
}

static bool open_ipc_device(CanHandler* handler, const char* dev_path)
{
    const char* path = dev_path ? dev_path : "/dev/tcc_ipc_micom";
    handler->fd = open(path, O_RDWR | O_NONBLOCK);
    if (handler->fd < 0) {
        perror("[CanHandler] IPC device open failed");
        return false;
    }
    printf("[CanHandler] IPC device opened: %s (fd=%d)\n", path, handler->fd);
    return true;
}

static void consume_rx_bytes(CanHandler* handler, size_t count)
{
    if (count >= handler->rx_buffer_len) {
        handler->rx_buffer_len = 0;
        return;
    }

    memmove(handler->rx_buffer, handler->rx_buffer + count, handler->rx_buffer_len - count);
    handler->rx_buffer_len -= count;
}

static size_t find_ipc_header(const uint8_t* data, size_t length)
{
    for (size_t i = 0; i + 2 < length; i++) {
        if (data[i] == IPC_SYNC && data[i + 1] == IPC_START1 && data[i + 2] == IPC_START2) {
            return i;
        }
    }
    return length;
}

static bool process_ipc_rx_buffer(CanHandler* handler)
{
    bool processed = false;

    while (handler->rx_buffer_len > 0) {
        size_t header_offset = find_ipc_header(handler->rx_buffer, handler->rx_buffer_len);
        if (header_offset == handler->rx_buffer_len) {
            /* Retain a possible partial FF 55 header for the next read. */
            if (handler->rx_buffer_len > 2) consume_rx_bytes(handler, handler->rx_buffer_len - 2);
            break;
        }
        if (header_offset > 0) consume_rx_bytes(handler, header_offset);
        if (handler->rx_buffer_len < IPC_PACKET_PREPARE_SIZE) break;

        uint16_t payload_length = (uint16_t)((handler->rx_buffer[7] << 8) |
                                             handler->rx_buffer[8]);
        size_t frame_length = IPC_PACKET_PREPARE_SIZE + (size_t)payload_length + IPC_PACKET_CRC_SIZE;
        if (payload_length < 8 || frame_length > IPC_MAX_PACKET_SIZE) {
            fprintf(stderr, "[CanHandler] IPC RX invalid length=%u; resynchronizing\n", payload_length);
            consume_rx_bytes(handler, 1);
            continue;
        }
        if (handler->rx_buffer_len < frame_length) break;

        IpcFrame frame;
        IpcParseResult result = ipc_frame_parse(handler->rx_buffer, (ssize_t)frame_length, &frame);
        if (result != IPC_PARSE_OK) {
            fprintf(stderr, "[CanHandler] IPC frame drop: %s; resynchronizing\n",
                    ipc_frame_parse_result_str(result));
            consume_rx_bytes(handler, 1);
            continue;
        }

        EgoVehicle ego;
        if (!decode_can_payload(frame.data, frame.data_len, &ego)) {
            fprintf(stderr, "[CanHandler] message_id nibble mismatch or dlc<8 "
                            "(data_len=%zu, first_byte=0x%02X)\n",
                    frame.data_len, frame.data_len > 0 ? frame.data[0] : 0);
        } else {
            emit_ego_with_diagnostics(handler, &ego);
            processed = true;
        }
        consume_rx_bytes(handler, frame_length);
    }

    return processed;
}

bool can_handler_init(
    CanHandler* handler,
    const char* dev_path,
    bool rx_mock_mode,
    bool tx_mock_mode,
    const CanHandlerCallbacks* callbacks
)
{
    if (!handler) return false;
    memset(handler, 0, sizeof(*handler));
    handler->dev_path = dev_path;
    handler->fd = -1;
    handler->rx_mock_mode = rx_mock_mode;
    handler->tx_mock_mode = tx_mock_mode;
    if (callbacks) {
        handler->callbacks = *callbacks;
    }

    if (rx_mock_mode && tx_mock_mode) {
        handler->initialized = true;
        printf("[CanHandler] init dev_path=%s rx_mock=1 tx_mock=1\n",
               dev_path ? dev_path : "none");
        return true;
    }

    if (!open_ipc_device(handler, dev_path)) return false;

    handler->initialized = true;
    printf("[CanHandler] init dev_path=%s rx_mock=%d tx_mock=%d\n",
           dev_path ? dev_path : CAN_HANDLER_DEFAULT_DEV_PATH,
           rx_mock_mode ? 1 : 0,
           tx_mock_mode ? 1 : 0);
    return true;
}

void can_handler_cleanup(CanHandler* handler)
{
    if (!handler || !handler->initialized) return;
    if (handler->fd >= 0) {
        close(handler->fd);
        handler->fd = -1;
    }
    handler->initialized = false;
}

bool can_handler_poll(CanHandler* handler, int timeout_ms)
{
    if (!handler || !handler->initialized) return false;
    if (handler->rx_mock_mode) return poll_mock(handler);

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
    if (ready == 0) return false; /* 이번 timeout 동안 들어온 데이터 없음 */

    uint8_t recv_buf[IPC_MAX_PACKET_SIZE];
    ssize_t n = read(handler->fd, recv_buf, sizeof(recv_buf));
    if (n <= 0) {
        if (n < 0 && errno != EAGAIN) perror("[CanHandler] IPC read failed");
        return false;
    }

    if ((size_t)n > sizeof(handler->rx_buffer) - handler->rx_buffer_len) {
        fprintf(stderr, "[CanHandler] IPC RX buffer overflow; discarding buffered data\n");
        handler->rx_buffer_len = 0;
    }
    if ((size_t)n > sizeof(handler->rx_buffer)) return false;

    memcpy(handler->rx_buffer + handler->rx_buffer_len, recv_buf, (size_t)n);
    handler->rx_buffer_len += (size_t)n;
    return process_ipc_rx_buffer(handler);
}

/* ===================== TX helpers ===================== */

/* message_id(4bit) + timestamp(12bit) + payload48 을 64bit big-endian CAN data(8byte)로 packing */
static void pack_can_data(uint8_t message_id, uint16_t timestamp, uint64_t payload48, uint8_t out8[8])
{
    uint64_t raw = ((uint64_t)(message_id & 0xFu) << 60) |
                   ((uint64_t)(timestamp & EGO_MASK_TIMESTAMP) << EGO_SHIFT_TIMESTAMP) |
                   (payload48 & 0xFFFFFFFFFFFFULL);

    for (int i = 7; i >= 0; i--) {
        out8[i] = (uint8_t)(raw & 0xFFu);
        raw >>= 8;
    }
}

/* CAN data(8byte)를 IPC 프레임으로 조립해 device fd로 전송한다.
 * mock 모드에서는 실제 write() 없이 로그만 남긴다. */
static void can_handler_send_frame(
    CanHandler* handler,
    uint8_t message_id,
    uint64_t payload48,
    uint64_t timestamp_epoch_ms
)
{
    if (!handler || !handler->initialized) return;

    uint16_t timestamp = (uint16_t)(timestamp_epoch_ms & EGO_MASK_TIMESTAMP);
    uint8_t can_data[8];
    pack_can_data(message_id, timestamp, payload48, can_data);

    if (handler->tx_mock_mode) {
        printf("[CanHandler][TX-MOCK] msg_id=0x%X data=%02X%02X%02X%02X%02X%02X%02X%02X\n",
               message_id,
               can_data[0], can_data[1], can_data[2], can_data[3],
               can_data[4], can_data[5], can_data[6], can_data[7]);
        return;
    }

    if (handler->fd < 0) {
        fprintf(stderr, "[CanHandler] TX skipped: device not open (msg_id=0x%X)\n", message_id);
        return;
    }

    if (ipc_frame_send(handler->fd, CAN_TX_CHANNEL_BITMASK, CAN_TX_ONLY_CHANNEL_BITMASK, CAN_TX_ID,
                        can_data, sizeof(can_data)) != 0) {
        fprintf(stderr, "[CanHandler] IPC TX send failed (msg_id=0x%X): %s\n", message_id, strerror(errno));
    } else if (message_id == CANDIDATE_INTRO_MSG_ID) {
        printf("[CanHandler][TX] candidate intro msg_id=0x4 data=%02X%02X%02X%02X%02X%02X%02X%02X\n",
               can_data[0], can_data[1], can_data[2], can_data[3],
               can_data[4], can_data[5], can_data[6], can_data[7]);
    }
}

/* ===================== 0010(binary) - NTP Time Synchronization ===================== */
void can_handler_send_ntp_sync(CanHandler* handler)
{
    if (!handler || !handler->initialized) return;

    uint64_t sync_epoch_ms = ntp_time_sync_epoch_ms();
    NtpSyncStatus sync_status = ntp_time_get_sync_status();
    uint16_t timestamp = (uint16_t)(sync_epoch_ms & EGO_MASK_TIMESTAMP);
    uint64_t payload =
        ((sync_epoch_ms & NTP_SYNC_MASK_EPOCH_MS) << NTP_SYNC_SHIFT_EPOCH_MS) |
        (uint64_t)sync_status;
    uint8_t can_data[8];

    /* The timestamp is the lower 12 bits of the same epoch carried in the
     * payload. RTOS uses it for modulo-4096 ms timestamp reconstruction. */
    pack_can_data(NTP_SYNC_MSG_ID, timestamp, payload, can_data);

    if (handler->tx_mock_mode) {
        printf("[CanHandler][TX-MOCK][NTP] epoch_ms=%llu status=%u data=%02X%02X%02X%02X%02X%02X%02X%02X\n",
               (unsigned long long)sync_epoch_ms, (unsigned int)sync_status,
               can_data[0], can_data[1], can_data[2], can_data[3],
               can_data[4], can_data[5], can_data[6], can_data[7]);
        return;
    }

    if (handler->fd < 0) {
        fprintf(stderr, "[CanHandler] NTP sync TX skipped: device not open\n");
        return;
    }

    if (ipc_frame_send(handler->fd, CAN_TX_CHANNEL_BITMASK, CAN_TX_ONLY_CHANNEL_BITMASK, CAN_TX_ID,
                       can_data, sizeof(can_data)) != 0) {
        fprintf(stderr, "[CanHandler] NTP sync IPC TX failed: %s\n", strerror(errno));
    }
}

/* ===================== 0100(binary) - Candidate Vehicle Intro ===================== */
void can_handler_send_candidate_vehicle_intro(
    CanHandler* handler,
    uint8_t type_mask,
    uint16_t cz_x,
    uint16_t cz_y,
    uint64_t timestamp_epoch_ms
)
{
    uint64_t payload =
        ((uint64_t)(type_mask & CANDIDATE_INTRO_MASK_TYPE_MASK) << CANDIDATE_INTRO_SHIFT_TYPE_MASK) |
        ((uint64_t)(cz_x & CANDIDATE_INTRO_MASK_CZ_X)           << CANDIDATE_INTRO_SHIFT_CZ_X) |
        ((uint64_t)(cz_y & CANDIDATE_INTRO_MASK_CZ_Y)           << CANDIDATE_INTRO_SHIFT_CZ_Y);

    can_handler_send_frame(handler, CANDIDATE_INTRO_MSG_ID, payload, timestamp_epoch_ms);
}

/* ===================== 0101(binary) - Candidate Vehicle Status ===================== */
void can_handler_send_candidate_vehicle_status(
    CanHandler* handler,
    uint8_t type_mask,
    const VehicleInfo* vehicle
)
{
    uint8_t speed = 0;
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t heading = 0;

    if (vehicle) {
        speed   = vehicle->speed;
        x       = vehicle->x;
        y       = vehicle->y;
        heading = vehicle->heading;
    }

    uint64_t payload =
        ((uint64_t)(type_mask & CANDIDATE_STATUS_MASK_TYPE_MASK) << CANDIDATE_STATUS_SHIFT_TYPE_MASK) |
        ((uint64_t)(speed & EGO_MASK_SPEED)                      << EGO_SHIFT_SPEED) |
        ((uint64_t)(x & EGO_MASK_X)                              << EGO_SHIFT_X) |
        ((uint64_t)(y & EGO_MASK_Y)                              << EGO_SHIFT_Y) |
        ((uint64_t)(heading & EGO_MASK_HEADING)                  << EGO_SHIFT_HEADING);

    uint64_t timestamp_epoch_ms = vehicle ? vehicle->timestamp_ms : ntp_time_sync_epoch_ms();
    can_handler_send_frame(handler, CANDIDATE_STATUS_MSG_ID, payload, timestamp_epoch_ms);
}

void can_handler_send_no_candidate_vehicle(CanHandler* handler)
{
    can_handler_send_candidate_vehicle_status(handler, CANDIDATE_STATUS_TYPE_NONE, NULL);
}

void can_handler_send_candidate_vehicle_unavailable(CanHandler* handler)
{
    can_handler_send_candidate_vehicle_status(handler, CANDIDATE_STATUS_TYPE_COMM_ERROR, NULL);
}

/* ===================== 0110(binary) - Traffic Light / Maneuver Status ===================== */
void can_handler_send_traffic_light(
    CanHandler* handler,
    uint8_t tl_id,
    const TrafficLight* traffic_light,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t maneuver
)
{
    uint8_t color = 0;
    uint8_t time_left = 0;

    if (traffic_light) {
        color     = traffic_light->color;
        time_left = traffic_light->time_left;
    }

    uint64_t payload =
        ((uint64_t)(tl_id & TL_MASK_TL_TYPE_MASK)   << TL_SHIFT_TL_TYPE_MASK) |
        ((uint64_t)(color & TL_MASK_COLOR)          << TL_SHIFT_COLOR) |
        ((uint64_t)(time_left & TL_MASK_TIME_LEFT)  << TL_SHIFT_TIME_LEFT) |
        ((uint64_t)(cz_x & TL_MASK_CZ_X)            << TL_SHIFT_CZ_X) |
        ((uint64_t)(cz_y & TL_MASK_CZ_Y)            << TL_SHIFT_CZ_Y) |
        ((uint64_t)(maneuver & TL_MASK_MANEUVER)    << TL_SHIFT_MANEUVER);

    uint64_t timestamp_epoch_ms = traffic_light ? traffic_light->timestamp_ms : ntp_time_sync_epoch_ms();
    can_handler_send_frame(handler, TL_STATUS_MSG_ID, payload, timestamp_epoch_ms);
}

void can_handler_send_no_traffic_light(CanHandler* handler, uint16_t cz_x, uint16_t cz_y, uint8_t maneuver)
{
    uint64_t payload =
        ((uint64_t)(TL_STATUS_ID_NONE)           << TL_SHIFT_TL_TYPE_MASK) |
        ((uint64_t)(cz_x & TL_MASK_CZ_X)         << TL_SHIFT_CZ_X) |
        ((uint64_t)(cz_y & TL_MASK_CZ_Y)         << TL_SHIFT_CZ_Y) |
        ((uint64_t)(maneuver & TL_MASK_MANEUVER) << TL_SHIFT_MANEUVER);

    can_handler_send_frame(handler, TL_STATUS_MSG_ID, payload, ntp_time_sync_epoch_ms());
}

void can_handler_send_traffic_light_unavailable(CanHandler* handler, uint8_t maneuver)
{
    uint64_t payload =
        ((uint64_t)(TL_STATUS_ID_COMM_ERROR)     << TL_SHIFT_TL_TYPE_MASK) |
        ((uint64_t)(maneuver & TL_MASK_MANEUVER) << TL_SHIFT_MANEUVER);

    can_handler_send_frame(handler, TL_STATUS_MSG_ID, payload, ntp_time_sync_epoch_ms());
}
