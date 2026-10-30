/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Test that Ctrl-C stops birria_udp while a read runs without delivering any
 * sample, and that Ctrl-C during setup closes the device without a read. The
 * example is included directly, with its main renamed, and a stub library
 * replaces libbirriasdr: its read never calls back and ends only when
 * cancelled, or after a five-second limit that marks the test as failed.
 */
#define main birria_udp_program_main
#include "../examples/birria_udp.c"
#undef main

enum { STUB_IDLE, STUB_RUNNING, STUB_CANCELLING };
static atomic_uint stub_state = STUB_IDLE;
static atomic_int stub_reads;
static atomic_int stub_closes;

/* Library stubs: everything succeeds except the read, described above. */
const char *birriasdr_strerror(int error) { return error ? "error" : "success"; }
int birriasdr_open(birriasdr_dev_t **dev, uint32_t index)
{
    (void)index;
    *dev = (birriasdr_dev_t *)&stub_state;
    return 0;
}
int birriasdr_close(birriasdr_dev_t *dev)
{
    (void)dev;
    atomic_fetch_add(&stub_closes, 1);
    return 0;
}
int birriasdr_set_center_freq(birriasdr_dev_t *dev, uint64_t frequency)
{
    (void)dev;
    (void)frequency;
    return 0;
}
int birriasdr_set_tuner_gain_mode(birriasdr_dev_t *dev, int manual)
{
    (void)dev;
    (void)manual;
    return 0;
}
int birriasdr_set_gain_codes(birriasdr_dev_t *dev, const struct birriasdr_gain *gain)
{
    (void)dev;
    (void)gain;
    return 0;
}
int birriasdr_set_recovery(birriasdr_dev_t *dev, uint32_t limit, birriasdr_recovery_cb_t callback,
                           void *context)
{
    (void)dev;
    (void)limit;
    (void)callback;
    (void)context;
    return 0;
}
int birriasdr_get_stream_stats(birriasdr_dev_t *dev, struct birriasdr_stream_stats *stats)
{
    (void)dev;
    memset(stats, 0, sizeof(*stats));
    return 0;
}
int birriasdr_read_async(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback, void *context,
                         uint32_t transfer_count, uint32_t buffer_samples)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)dev;
    (void)callback;
    (void)context;
    (void)transfer_count;
    (void)buffer_samples;
    atomic_fetch_add(&stub_reads, 1);
    atomic_store(&stub_state, STUB_RUNNING);
    for (int i = 0; i < 500 && atomic_load(&stub_state) == STUB_RUNNING; ++i)
        nanosleep(&tick, NULL);
    return atomic_exchange(&stub_state, STUB_IDLE) == STUB_CANCELLING ? 0
                                                                       : BIRRIASDR_ERROR_TIMEOUT;
}
int birriasdr_cancel_async(birriasdr_dev_t *dev)
{
    unsigned expected = STUB_RUNNING;
    (void)dev;
    if (atomic_compare_exchange_strong(&stub_state, &expected, STUB_CANCELLING) ||
        expected == STUB_CANCELLING)
        return 0;
    return BIRRIASDR_ERROR_STATE;
}

/* Send SIGINT once the stub read is running. Give up after five seconds, so a
 * setup failure ends the test promptly instead of at the CTest timeout.
 */
static void *interrupt_later(void *unused)
{
    const struct timespec tick = {.tv_nsec = 10000000};
    (void)unused;
    for (int i = 0; i < 500 && atomic_load(&stub_state) != STUB_RUNNING; ++i)
        nanosleep(&tick, NULL);
    if (atomic_load(&stub_state) != STUB_RUNNING)
        return NULL;
    nanosleep(&(struct timespec){.tv_nsec = 200000000}, NULL);
    kill(getpid(), SIGINT);
    return NULL;
}

int main(void)
{
    char *argv[] = {(char *)"birria_udp", (char *)"--host", (char *)"127.0.0.1",
                    (char *)"--port", (char *)"9", NULL};
    pthread_t interrupter;
    int result;

    if (pthread_create(&interrupter, NULL, interrupt_later, NULL))
        return 1;
    result = birria_udp_program_main(5, argv);
    pthread_join(interrupter, NULL);
    if (result != 0 || atomic_load(&stub_reads) != 1) {
        fprintf(stderr, "Ctrl-C did not stop a read without callbacks (exit %d)\n", result);
        return 1;
    }

    /* The flag is still set, as after a Ctrl-C during setup: the program must
     * close the device without starting a read.
     */
    result = birria_udp_program_main(5, argv);
    if (result != 0 || atomic_load(&stub_reads) != 1 || atomic_load(&stub_closes) != 2) {
        fprintf(stderr, "Ctrl-C during setup still started a read (exit %d)\n", result);
        return 1;
    }
    puts("UDP interrupt test passed");
    return 0;
}
