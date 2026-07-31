#include "can_handler.h"
#include "ipc_frame.h"

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

static void emit_ego(CanHandler* handler, const EgoVehicle* ego)
{
    if (handler && ego && handler->callbacks.on_ego) {
        handler->callbacks.on_ego(ego, handler->callbacks.user_data);
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
     * CAN_MOCK=1은 실제 IPC/RTOS 없이 Linux 쪽 판단 로직만 검증하기 위한 모드.
     * 항상 우회전 시그널을 켠 채로 lanelet 안쪽 좌표를 흘려보낸다.
     */
    ego.x = (uint16_t)(205 + (handler->mock_tick % 20));
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
    const char* path = dev_path ? dev_path : CAN_HANDLER_DEFAULT_DEV_PATH;
    handler->fd = open(path, O_RDWR | O_NONBLOCK);
    if (handler->fd < 0) {
        perror("[CanHandler] IPC device open failed");
        return false;
    }
    printf("[CanHandler] IPC device opened: %s (fd=%d)\n", path, handler->fd);
    return true;
}

bool can_handler_init(
    CanHandler* handler,
    const char* dev_path,
    bool mock_mode,
    const CanHandlerCallbacks* callbacks
)
{
    if (!handler) return false;
    memset(handler, 0, sizeof(*handler));
    handler->dev_path = dev_path;
    handler->fd = -1;
    handler->mock_mode = mock_mode;
    if (callbacks) {
        handler->callbacks = *callbacks;
    }

    if (mock_mode) {
        handler->initialized = true;
        printf("[CanHandler] init dev_path=%s rx_mock=1\n", dev_path ? dev_path : "none");
        return true;
    }

    if (!open_ipc_device(handler, dev_path)) return false;

    handler->initialized = true;
    printf("[CanHandler] init dev_path=%s rx_real=1\n", dev_path ? dev_path : CAN_HANDLER_DEFAULT_DEV_PATH);
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
    if (handler->mock_mode) return poll_mock(handler);

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

    IpcFrame frame;
    IpcParseResult result = ipc_frame_parse(recv_buf, n, &frame);
    if (result != IPC_PARSE_OK) {
        fprintf(stderr, "[CanHandler] IPC frame drop: %s\n", ipc_frame_parse_result_str(result));
        return false;
    }

    /* RX 프레임에는 canID 헤더가 없다(TX와 달리 channel/tx_only/canID 4byte가
     * 존재하지 않음). frame.canID는 항상 0이므로 여기서 필터링하면 안 되고,
     * CAN payload 안의 message_id(상위 4bit)만으로 ego status 프레임인지 판별한다. */
    EgoVehicle ego;
    if (!decode_can_payload(frame.data, frame.data_len, &ego)) {
        fprintf(stderr, "[CanHandler] message_id nibble mismatch or dlc<8 (data_len=%zu, first_byte=0x%02X)\n",
                frame.data_len, frame.data_len > 0 ? frame.data[0] : 0);
        return false; /* message_id nibble이 EGO_FRAME_MSG_ID(0x0)가 아닌 경우 */
    }

    emit_ego(handler, &ego);
    return true;
}
