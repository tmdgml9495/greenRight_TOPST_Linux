#ifndef TEMP_CAN_TX_THREAD_H
#define TEMP_CAN_TX_THREAD_H

#include <pthread.h>
#include "app_context.h"

int can_tx_thread_start(pthread_t* thread, AppContext* context);

#endif
