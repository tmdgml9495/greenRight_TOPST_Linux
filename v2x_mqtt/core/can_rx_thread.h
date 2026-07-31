#ifndef TEMP_CAN_RX_THREAD_H
#define TEMP_CAN_RX_THREAD_H

#include <pthread.h>
#include "app_context.h"

int can_rx_thread_start(pthread_t* thread, AppContext* context);

#endif
