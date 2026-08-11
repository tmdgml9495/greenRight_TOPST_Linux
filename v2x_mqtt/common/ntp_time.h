#ifndef NTP_TIME_H
#define NTP_TIME_H

#include <stdbool.h>
#include <stdint.h>

/* 2026-01-01 00:00:00 UTC. */
#define NTP_SYNC_EPOCH_UNIX_MS 1767225600000ULL
#define NTP_TIME_ISO8601_UTC_STRLEN 24U /* YYYY-MM-DDTHH:MM:SS.mmmZ */

typedef enum {
    NTP_SYNC_OK = 0x00,
    NTP_SYNC_RTC_ONLY = 0x01,
    NTP_SYNC_NONE = 0x02
} NtpSyncStatus;

uint64_t ntp_time_now_ms(void);
uint64_t ntp_time_sync_epoch_ms(void);
NtpSyncStatus ntp_time_get_sync_status(void);
/* linux_now_epoch_ms must use the same 2026-01-01 reference epoch. */
uint64_t ntp_time_reconstruct(uint16_t rtos_ts_12bit, uint64_t linux_now_epoch_ms);

bool ntp_time_format_iso8601_utc(
    uint64_t timestamp_epoch_ms,
    char out[NTP_TIME_ISO8601_UTC_STRLEN + 1]
);
bool ntp_time_parse_iso8601_utc(const char* iso8601_utc, uint64_t* out_timestamp_epoch_ms);

#endif
