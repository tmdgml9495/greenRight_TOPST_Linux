#define _GNU_SOURCE

#include "ntp_time.h"

#include <stdio.h>
#include <string.h>
#include <sys/timex.h>
#include <time.h>

#define NTP_TS_MODULO_MS 4096ULL
#define NTP_TS_MASK      0x0FFFU

uint64_t ntp_time_now_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return 0;
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000ULL);
}

uint64_t ntp_time_sync_epoch_ms(void)
{
    uint64_t now_ms = ntp_time_now_ms();
    return now_ms >= NTP_SYNC_EPOCH_UNIX_MS ? now_ms - NTP_SYNC_EPOCH_UNIX_MS : 0;
}

NtpSyncStatus ntp_time_get_sync_status(void)
{
    struct timex tx = {0};
    int ret = adjtimex(&tx);

    if (ret < 0) return NTP_SYNC_NONE;
    if ((tx.status & STA_UNSYNC) != 0 || ret == TIME_ERROR) return NTP_SYNC_RTC_ONLY;
    return NTP_SYNC_OK;
}

uint64_t ntp_time_reconstruct(uint16_t rtos_ts_12bit, uint64_t linux_now_epoch_ms)
{
    uint64_t candidate = (linux_now_epoch_ms & ~(NTP_TS_MODULO_MS - 1ULL)) |
                         (uint64_t)(rtos_ts_12bit & NTP_TS_MASK);
    if (candidate > linux_now_epoch_ms) candidate -= NTP_TS_MODULO_MS;
    return candidate;
}

bool ntp_time_format_iso8601_utc(
    uint64_t timestamp_epoch_ms,
    char out[NTP_TIME_ISO8601_UTC_STRLEN + 1]
)
{
    if (!out) return false;

    uint64_t unix_ms = NTP_SYNC_EPOCH_UNIX_MS + timestamp_epoch_ms;
    time_t seconds = (time_t)(unix_ms / 1000ULL);
    struct tm utc;
    if (!gmtime_r(&seconds, &utc)) return false;

    int length = snprintf(
        out,
        NTP_TIME_ISO8601_UTC_STRLEN + 1,
        "%04d-%02d-%02dT%02d:%02d:%02d.%03lluZ",
        utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday,
        utc.tm_hour, utc.tm_min, utc.tm_sec,
        (unsigned long long)(unix_ms % 1000ULL)
    );
    return length == (int)NTP_TIME_ISO8601_UTC_STRLEN;
}

bool ntp_time_parse_iso8601_utc(const char* iso8601_utc, uint64_t* out_timestamp_epoch_ms)
{
    if (!iso8601_utc || !out_timestamp_epoch_ms ||
        strlen(iso8601_utc) != NTP_TIME_ISO8601_UTC_STRLEN) return false;

    int year, month, day, hour, minute, second, millisecond;
    if (sscanf(iso8601_utc, "%4d-%2d-%2dT%2d:%2d:%2d.%3dZ",
               &year, &month, &day, &hour, &minute, &second, &millisecond) != 7 ||
        month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 ||
        second < 0 || second > 59 || millisecond < 0 || millisecond > 999) return false;

    struct tm parsed = {0};
    parsed.tm_year = year - 1900;
    parsed.tm_mon = month - 1;
    parsed.tm_mday = day;
    parsed.tm_hour = hour;
    parsed.tm_min = minute;
    parsed.tm_sec = second;

    time_t seconds = timegm(&parsed);
    struct tm verified;
    if (seconds < 0 || !gmtime_r(&seconds, &verified) ||
        verified.tm_year != parsed.tm_year || verified.tm_mon != parsed.tm_mon ||
        verified.tm_mday != parsed.tm_mday || verified.tm_hour != parsed.tm_hour ||
        verified.tm_min != parsed.tm_min || verified.tm_sec != parsed.tm_sec) return false;

    uint64_t unix_ms = (uint64_t)seconds * 1000ULL + (uint64_t)millisecond;
    if (unix_ms < NTP_SYNC_EPOCH_UNIX_MS) return false;
    *out_timestamp_epoch_ms = unix_ms - NTP_SYNC_EPOCH_UNIX_MS;
    return true;
}
