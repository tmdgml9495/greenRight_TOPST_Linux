#ifndef IPC_FRAME_H
#define IPC_FRAME_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <sys/types.h>  /* ssize_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== IPC Frame (vendor char device: /dev/tcc_ipc_micom) =====================
 * ipc_example.c / ipc_library.c(테스트 코드)와 별개로, CAN RX 전용으로
 * 필요한 최소 파싱 로직만 자체 구현한다.
 *
 * 실측 결과, TX(CA72 -> MICOM)와 RX(MICOM -> CA72)의 프레임 구조가 다르다:
 *   - TX 프레임에는 channel_bitmask/tx_only_channel_bitmask/canID(4byte) 헤더가 존재한다.
 *   - RX 프레임에는 그 헤더가 없다. LENGTH 뒤에 CAN data(8byte)가 바로 이어진다.
 * ipc_frame_parse()는 RX 전용이므로 아래 레이아웃을 따른다.
 *
 * RX Frame layout:
 *   [0]     SYNC    = 0xFF
 *   [1]     START1  = 0x55
 *   [2]     START2  = 0xAA
 *   [3:4]   CMD1    (big-endian, 고정값)
 *   [5:6]   CMD2    (big-endian, 방향에 따라 고정값)
 *   [7:8]   LENGTH  (big-endian) = CAN data length (우리 프로토콜에서는 항상 8)
 *   [9:]    CAN data (dlc bytes, 우리 프로토콜에서는 항상 8byte)
 *   [-2:-1] CRC16 (big-endian, SYNC부터 CRC 앞까지 전체에 대한 CRC)
 *
 * canID는 이 헤더에 존재하지 않으므로 IpcFrame.canID로 필터링하면 안 되며,
 * 프레임 판별은 CAN payload 안의 message_id(상위 4bit)로 해야 한다.
 *
 * CMD1/CMD2 값 자체는 검증하지 않는다 - 송신측(RTOS)은 message_id만
 * 신경써서 보내고 CMD 값을 우리 쪽 가정과 맞출 필요가 없으므로, 여기서는
 * 프레임 내 위치를 건너뛰는 용도로만 다루고 값 비교는 하지 않는다. */

#define IPC_SYNC    0xFFu
#define IPC_START1  0x55u
#define IPC_START2  0xAAu

/* SYNC + START1 + START2 + CMD1(2) + CMD2(2) + LENGTH(2) */
#define IPC_PACKET_PREPARE_SIZE  9u
#define IPC_PACKET_CRC_SIZE      2u
#define IPC_MAX_PACKET_SIZE      0x400u

typedef enum {
    IPC_PARSE_OK = 0,
    IPC_PARSE_ERR_TOO_SHORT,
    IPC_PARSE_ERR_HEADER_MISMATCH,   /* SYNC/START1/START2 */
    IPC_PARSE_ERR_LENGTH_INVALID,    /* canID 헤더(4byte)도 안 들어가는 LENGTH */
    IPC_PARSE_ERR_CRC_MISMATCH,
} IpcParseResult;

typedef struct {
    uint16_t canID;           /* RX 프레임에는 canID 헤더가 없음 - 항상 0, 필터링에 사용 금지 */
    const uint8_t* data;      /* buf 내부를 가리키는 포인터 (복사 아님), RX에서는 CAN data(8byte) 시작점 */
    size_t   data_len;
} IpcFrame;

/* buf 안의 IPC 프레임 하나를 파싱한다. CMD1/CMD2 값은 검증하지 않는다.
 * - CRC가 안 맞으면 IPC_PARSE_ERR_CRC_MISMATCH (이 경우도 out_frame은 채워짐,
 *   호출측에서 로그만 남기고 버릴지 결정 가능)
 * 반환값이 IPC_PARSE_OK가 아니면 out_frame->data는 유효하지 않을 수 있다. */
IpcParseResult ipc_frame_parse(const uint8_t* buf, ssize_t len, IpcFrame* out_frame);

/* 사람이 읽기 좋은 에러 문자열 (로그용) */
const char* ipc_frame_parse_result_str(IpcParseResult result);

#ifdef __cplusplus
}
#endif

#endif /* IPC_FRAME_H */
