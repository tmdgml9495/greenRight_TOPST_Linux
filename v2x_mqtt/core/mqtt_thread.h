#ifndef TEMP_MQTT_THREAD_H
#define TEMP_MQTT_THREAD_H

#include <pthread.h>
#include "app_context.h"

int mqtt_thread_start(pthread_t* thread, AppContext* context);

#endif
