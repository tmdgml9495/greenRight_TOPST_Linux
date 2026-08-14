#include "input_control_thread.h"

#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#define KEY_INPUT_POLL_TIMEOUT_US (100000)

static void set_ntp_sync_period(
    AppContext* context,
    uint32_t period_ms,
    const char* mode_name
)
{
    uint32_t previous_period_ms = atomic_exchange(
        &context->ntp_sync_tx_period_ms,
        period_ms
    );

    if (previous_period_ms != period_ms) {
        printf(
            "[TimeSync Control] %s mode: msgId 0x2 period=%u ms\n",
            mode_name,
            period_ms
        );
        fflush(stdout);
    }
}

static void* input_control_thread_main(void* arg)
{
    AppContext* context = (AppContext*)arg;
    struct termios original_termios;
    struct termios input_termios;

    if (!isatty(STDIN_FILENO)) {
        printf("[TimeSync Control] keyboard disabled: stdin is not a terminal\n");
        return NULL;
    }

    if (tcgetattr(STDIN_FILENO, &original_termios) != 0) {
        perror("[TimeSync Control] tcgetattr");
        return NULL;
    }

    input_termios = original_termios;
    input_termios.c_lflag &= (tcflag_t)~(ICANON | ECHO);
    input_termios.c_cc[VMIN] = 0;
    input_termios.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &input_termios) != 0) {
        perror("[TimeSync Control] tcsetattr");
        return NULL;
    }

    printf(
        "[TimeSync Control] keys: t=test(1000 ms), n=normal(100 ms)\n"
    );
    fflush(stdout);

    while (atomic_load(&context->running)) {
        fd_set read_fds;
        struct timeval timeout;
        int select_result;

        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        timeout.tv_sec = 0;
        timeout.tv_usec = KEY_INPUT_POLL_TIMEOUT_US;

        select_result = select(
            STDIN_FILENO + 1,
            &read_fds,
            NULL,
            NULL,
            &timeout
        );

        if (select_result > 0 && FD_ISSET(STDIN_FILENO, &read_fds)) {
            char key;
            ssize_t read_result = read(STDIN_FILENO, &key, sizeof(key));

            if (read_result == (ssize_t)sizeof(key)) {
                if (key == 't' || key == 'T') {
                    set_ntp_sync_period(
                        context,
                        NTP_SYNC_TEST_PERIOD_MS,
                        "TEST"
                    );
                } else if (key == 'n' || key == 'N') {
                    set_ntp_sync_period(
                        context,
                        NTP_SYNC_NORMAL_PERIOD_MS,
                        "NORMAL"
                    );
                }
            } else if (read_result < 0 && errno != EINTR) {
                perror("[TimeSync Control] read");
                break;
            }
        } else if (select_result < 0 && errno != EINTR) {
            perror("[TimeSync Control] select");
            break;
        }
    }

    if (tcsetattr(STDIN_FILENO, TCSANOW, &original_termios) != 0) {
        perror("[TimeSync Control] restore tcsetattr");
    }

    return NULL;
}

int input_control_thread_start(pthread_t* thread, AppContext* context)
{
    if (!thread || !context) return -1;
    return pthread_create(thread, NULL, input_control_thread_main, context);
}
