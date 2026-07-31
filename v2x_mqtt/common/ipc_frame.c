#include "ipc_frame.h"

#include <errno.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static uint16_t ipc_calc_crc16(const uint8_t* data, size_t size)
{
    uint16_t crc = 0;
    for (size_t i = 0; i < size; i++) {
        uint8_t temp8 = (uint8_t)((crc >> 8) & 0xFFu);
        temp8 ^= data[i];
        crc = (uint16_t)(crc16_table[temp8] ^ (uint16_t)(crc << 8));
    }
    return crc;
}

IpcParseResult ipc_frame_parse(const uint8_t* buf, ssize_t len, IpcFrame* out_frame)
{
    if (!buf || !out_frame) return IPC_PARSE_ERR_TOO_SHORT;

    memset(out_frame, 0, sizeof(*out_frame));

    if (len < (ssize_t)(IPC_PACKET_PREPARE_SIZE + IPC_PACKET_CRC_SIZE)) {
        return IPC_PARSE_ERR_TOO_SHORT;
    }

    if (buf[0] != IPC_SYNC || buf[1] != IPC_START1 || buf[2] != IPC_START2) {
        return IPC_PARSE_ERR_HEADER_MISMATCH;
    }

    /* buf[3:4]=CMD1, buf[5:6]=CMD2 - 값은 검증하지 않고 위치만 건너뛴다. */

    uint16_t length = (uint16_t)((buf[7] << 8) | buf[8]);
    size_t avail = (size_t)len - IPC_PACKET_PREPARE_SIZE;
    size_t take = length <= avail ? length : avail;

    /* RX(MICOM -> CA72) 프레임에는 channel_bitmask/tx_only/canID 헤더가 없다.
     * LENGTH 뒤에 CAN data(8byte)가 바로 이어지므로 최소 길이도 8이어야 한다. */
    if (take < 8) {
        return IPC_PARSE_ERR_LENGTH_INVALID;
    }

    out_frame->canID    = 0; /* RX 프레임에는 canID 헤더가 없음 - 항상 0, 사용하지 말 것 */
    out_frame->data     = &buf[IPC_PACKET_PREPARE_SIZE];
    out_frame->data_len = take;

    uint16_t crc_received   = (uint16_t)((buf[len - 2] << 8) | buf[len - 1]);
    uint16_t crc_calculated = ipc_calc_crc16(buf, (size_t)len - 2);
    if (crc_received != crc_calculated) {
        return IPC_PARSE_ERR_CRC_MISMATCH;
    }

    return IPC_PARSE_OK;
}

int ipc_frame_build(
    uint8_t channel_bitmask,
    uint8_t tx_only_channel_bitmask,
    uint16_t canID,
    const uint8_t* data,
    size_t data_len,
    uint8_t* out_buf,
    size_t out_cap
)
{
    if (!data || !out_buf || data_len == 0) {
        return -1;
    }

    size_t payload_size = (size_t)IPC_FRAME_TX_HEADER_SIZE + data_len;
    size_t packet_size  = IPC_PACKET_PREPARE_SIZE + payload_size;
    size_t total_size   = packet_size + IPC_PACKET_CRC_SIZE;

    if (payload_size > 0xFFFFu || total_size > out_cap) {
        return -1;
    }

    /* SYNC / START1 / START2 */
    out_buf[0] = IPC_SYNC;
    out_buf[1] = IPC_START1;
    out_buf[2] = IPC_START2;

    /* CMD1 / CMD2 (big-endian) */
    out_buf[3] = (uint8_t)((IPC_FRAME_TX_CMD1 >> 8) & 0xFFu);
    out_buf[4] = (uint8_t)(IPC_FRAME_TX_CMD1 & 0xFFu);
    out_buf[5] = (uint8_t)((IPC_FRAME_TX_CMD2 >> 8) & 0xFFu);
    out_buf[6] = (uint8_t)(IPC_FRAME_TX_CMD2 & 0xFFu);

    /* LENGTH = header(4) + CAN data(8) = 12 (RX와 달리 canID 헤더 포함) */
    uint16_t length = (uint16_t)payload_size;
    out_buf[7] = (uint8_t)((length >> 8) & 0xFFu);
    out_buf[8] = (uint8_t)(length & 0xFFu);

    /* channel_bitmask / tx_only_channel_bitmask / canID (big-endian) */
    out_buf[IPC_PACKET_PREPARE_SIZE + 0] = channel_bitmask;
    out_buf[IPC_PACKET_PREPARE_SIZE + 1] = tx_only_channel_bitmask;
    out_buf[IPC_PACKET_PREPARE_SIZE + 2] = (uint8_t)((canID >> 8) & 0xFFu);
    out_buf[IPC_PACKET_PREPARE_SIZE + 3] = (uint8_t)(canID & 0xFFu);

    /* CAN data */
    memcpy(&out_buf[IPC_PACKET_PREPARE_SIZE + IPC_FRAME_TX_HEADER_SIZE], data, data_len);

    /* CRC16: SYNC부터 CRC 필드 직전까지 전체에 대해 계산 (RX 파싱과 동일 알고리즘) */
    uint16_t crc = ipc_calc_crc16(out_buf, packet_size);
    out_buf[packet_size]     = (uint8_t)((crc >> 8) & 0xFFu);
    out_buf[packet_size + 1] = (uint8_t)(crc & 0xFFu);

    return (int)total_size;
}

int ipc_frame_send(
    int fd,
    uint8_t channel_bitmask,
    uint8_t tx_only_channel_bitmask,
    uint16_t canID,
    const uint8_t* data,
    size_t data_len
)
{
    uint8_t frame_buf[IPC_MAX_PACKET_SIZE];
    int frame_len = ipc_frame_build(
        channel_bitmask, tx_only_channel_bitmask, canID,
        data, data_len,
        frame_buf, sizeof(frame_buf)
    );
    if (frame_len < 0) {
        return -1;
    }

    size_t total_size = (size_t)frame_len;
    while (1) {
        ssize_t n = write(fd, frame_buf, total_size);
        if (n == (ssize_t)total_size) {
            return 0;
        }
        if (n < 0 && (errno == 62 /* ETIME on some platforms */ || errno == EAGAIN)) {
            struct timespec ts = {0, 100 * 1000 * 1000}; /* 100ms */
            nanosleep(&ts, NULL);
            continue;
        }
        /* 그 외 에러(짧은 write 포함)는 이 단순 char device 프로토콜에서는
         * 재시도 대상이 아니므로 실패로 처리한다. */
        return -1;
    }
}

const char* ipc_frame_parse_result_str(IpcParseResult result)
{
    switch (result) {
        case IPC_PARSE_OK:                    return "OK";
        case IPC_PARSE_ERR_TOO_SHORT:          return "frame too short";
        case IPC_PARSE_ERR_HEADER_MISMATCH:    return "SYNC/START header mismatch";
        case IPC_PARSE_ERR_LENGTH_INVALID:     return "LENGTH field invalid (too short for 8-byte CAN data)";
        case IPC_PARSE_ERR_CRC_MISMATCH:       return "CRC mismatch";
        default:                               return "unknown error";
    }
}
