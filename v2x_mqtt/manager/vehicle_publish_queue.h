#ifndef VEHICLE_PUBLISH_QUEUE_H
#define VEHICLE_PUBLISH_QUEUE_H

#include <pthread.h>
#include <stdbool.h>
#include "types.h"

typedef struct {
    pthread_mutex_t lock;
    VehicleInfo latest;
    bool pending;
} VehiclePublishQueue;

bool vehicle_publish_queue_init(
    VehiclePublishQueue* queue
);

void vehicle_publish_queue_destroy(
    VehiclePublishQueue* queue
);

void vehicle_publish_queue_push(
    VehiclePublishQueue* queue,
    const VehicleInfo* vehicle
);

bool vehicle_publish_queue_try_pop(
    VehiclePublishQueue* queue,
    VehicleInfo* out
);

#endif
