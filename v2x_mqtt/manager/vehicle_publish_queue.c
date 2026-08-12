#include "vehicle_publish_queue.h"

#include <string.h>

bool vehicle_publish_queue_init(VehiclePublishQueue* queue)
{
    if (!queue) return false;
    memset(queue, 0, sizeof(*queue));
    return pthread_mutex_init(&queue->lock, NULL) == 0;
}

void vehicle_publish_queue_destroy(VehiclePublishQueue* queue)
{
    if (!queue) return;
    pthread_mutex_destroy(&queue->lock);
}

void vehicle_publish_queue_push(
    VehiclePublishQueue* queue,
    const VehicleInfo* vehicle
)
{
    if (!queue || !vehicle) return;

    pthread_mutex_lock(&queue->lock);
    queue->latest = *vehicle;
    queue->pending = true;
    pthread_mutex_unlock(&queue->lock);
}

bool vehicle_publish_queue_try_pop(
    VehiclePublishQueue* queue,
    VehicleInfo* out
)
{
    if (!queue || !out) return false;

    pthread_mutex_lock(&queue->lock);
    bool pending = queue->pending;
    if (pending) {
        *out = queue->latest;
        queue->pending = false;
    }
    pthread_mutex_unlock(&queue->lock);
    return pending;
}
