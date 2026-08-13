#ifndef TEMPORAL_QOS_H
#define TEMPORAL_QOS_H

#include <stdint.h>

#define TEMPORAL_QOS_TRACE_STAGE_ENABLE  (1U)
#define TEMPORAL_QOS_TIMESTAMP_MASK      (0x0FFFU)

void TemporalQos_TraceStage(
    uint16_t log_id,
    uint16_t src_timestamp12
);

#endif /* TEMPORAL_QOS_H */
