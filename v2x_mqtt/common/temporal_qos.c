#include "temporal_qos.h"

#include <stdio.h>

#include "ntp_time.h"

void TemporalQos_TraceStage(
    uint16_t log_id,
    uint16_t src_timestamp12
)
{
    char current_time[NTP_TIME_ISO8601_UTC_STRLEN + 1];
    uint64_t current_epoch_ms = ntp_time_sync_epoch_ms();

    if (!ntp_time_format_iso8601_utc(current_epoch_ms, current_time)) {
        printf(
            "[T%u] src_ts12=%u current=INVALID\n",
            (unsigned int)log_id,
            (unsigned int)(src_timestamp12 & TEMPORAL_QOS_TIMESTAMP_MASK)
        );
        return;
    }

    printf(
        "[T%u] src_ts12=%u [TIME SYNC] %.12s UTC\n",
        (unsigned int)log_id,
        (unsigned int)(src_timestamp12 & TEMPORAL_QOS_TIMESTAMP_MASK),
        &current_time[11]
    );
}
