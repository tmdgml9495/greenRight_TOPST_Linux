#include "ntp_time.h"

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
