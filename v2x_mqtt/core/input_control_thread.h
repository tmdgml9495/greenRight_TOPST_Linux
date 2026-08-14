#ifndef INPUT_CONTROL_THREAD_H
#define INPUT_CONTROL_THREAD_H

#include <pthread.h>

#include "app_context.h"

int input_control_thread_start(pthread_t* thread, AppContext* context);

#endif
