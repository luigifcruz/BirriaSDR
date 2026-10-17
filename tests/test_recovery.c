/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * Physical overrun and reopen regression, deliberately excluded from CTest.
 *
 * A slow application callback fills the bounded host queue. Separately, a parent
 * process pauses its receiver child for 250 ms: outstanding USB requests drain,
 * the device cannot deliver more samples, and its circular SRAM must overrun.
 * The parent never opens libusb; it always resumes its child before accepting
 * another command. Keeping the launcher running also prevents sudo or a shell
 * from stopping its own job when it observes a stopped direct child.
 *
 * These two failures have different origins. Both must be reported as overrun,
 * release USB ownership, and permit another finite acquisition. Check recovery
 * at both tuning bounds and the channel-6 anchor, with and without reopening.
 * A successful Linux run does not establish macOS endpoint recovery behavior.
 */
#include "birriasdr.h"
#include <errno.h>
#include <inttypes.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

struct capture {
    birriasdr_dev_t *dev;
    int trigger_fd;
    bool pause_process, triggered;
    int error;
    uint64_t samples;
    bool automatic;
    unsigned gaps;
    uint64_t resume_until;
};

/* Complete the requested delay even if interrupted by a signal. */
static void pause_host(void)
{
    struct timespec delay = {0, 250000000};
    while (nanosleep(&delay, &delay) && errno == EINTR) {}
}

/* Suspend the receiver only after a callback proves acquisition has started. */
static int supervise(int fd, pid_t receiver)
{
    char command;
    ssize_t count;
    while ((count = read(fd, &command, 1)) != 0) {
        if (count < 0) {
            if (errno == EINTR) continue;
            return 1;
        }
        if (kill(receiver, SIGSTOP)) return 1;
        pause_host();
        if (kill(receiver, SIGCONT)) return 1;
    }
    return 0;
}

/* Inject one scheduling stall; ordinary callbacks otherwise only count data. */
static void receive(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct capture *run = context;
    if (!iq || !count || first != run->samples) run->error = BIRRIASDR_ERROR_PROTOCOL;
    run->samples += count;
    if (!run->triggered && run->samples >= 1048576) {
        run->triggered = true;
        if (run->pause_process) {
            if (write(run->trigger_fd, "p", 1) != 1) run->error = BIRRIASDR_ERROR_IO;
        } else {
            pause_host();
        }
    }
    if (run->error || run->samples > UINT64_C(40000000) * 5)
        birriasdr_cancel_async(run->dev);
    if (run->gaps && run->samples >= run->resume_until)
        birriasdr_cancel_async(run->dev);
}

/* Require an explicit gap before resumed samples, with the original fault's
 * diagnostics intact. A cumulative host offset is not elapsed RF time.
 */
static void gap(const struct birriasdr_recovery_event *event, void *context)
{
    struct capture *run = context;
    ++run->gaps;
    if (event->error != BIRRIASDR_ERROR_OVERRUN || event->next_sample != run->samples ||
        event->attempt != 1 || (run->pause_process && event->previous.firmware_state != 9))
        run->error = BIRRIASDR_ERROR_PROTOCOL;
    run->resume_until = event->next_sample + 65536;
    printf("{\"event\":\"gap\",\"error\":%d,\"state\":%u,\"next_sample\":%" PRIu64 "}\n",
           event->error, event->previous.firmware_state, event->next_sample);
    if (run->error) birriasdr_cancel_async(run->dev);
}

/* Preserve error origin and reset status before the next capture clears them. */
static int force_overrun(struct capture *run, bool source)
{
    struct birriasdr_stream_stats stats = {0};
    run->pause_process = source;
    run->triggered = false;
    run->samples = 0;
    run->error = 0;
    run->gaps = 0;
    int configured = birriasdr_set_recovery(run->dev, run->automatic ? 2 : 0, gap, run);
    if (configured) return configured;
    int result = birriasdr_read_async(run->dev, receive, run, 0, 0);
    int status = birriasdr_get_stream_stats(run->dev, &stats);
    printf("{\"event\":\"overrun\",\"source\":%s,\"result\":%d,\"state\":%u,"
           "\"stage\":%u,\"samples\":%" PRIu64 ",\"dma_errors\":%u,\"reset\":%d}\n",
           source ? "true" : "false", result, stats.firmware_state,
           (unsigned)stats.stage, stats.samples, stats.dma_errors, stats.reset_error);
    if (status) return status;
    if (run->error) return run->error;
    if (!run->triggered || stats.reset_error || stats.dma_errors)
        return BIRRIASDR_ERROR_STATE;
    if (run->automatic)
        return result || run->gaps != 1 || stats.firmware_state != 3 ||
               run->samples < run->resume_until ? BIRRIASDR_ERROR_STATE : 0;
    if (result != BIRRIASDR_ERROR_OVERRUN || (source && stats.firmware_state != 9))
        return BIRRIASDR_ERROR_STATE;
    return 0;
}

/* Read fresh packets at a selected center and require normal finite completion. */
static int recover(struct capture *run, uint64_t frequency, bool reopen)
{
    int16_t iq[65536 * 2];
    struct birriasdr_stream_stats stats = {0};
    uint32_t received = 0;
    int result = 0;
    if (reopen) {
        result = birriasdr_close(run->dev);
        run->dev = NULL;
        if (!result) result = birriasdr_open(&run->dev, 0);
    }
    if (!result) result = birriasdr_set_center_freq(run->dev, frequency);
    if (!result) result = birriasdr_read_sync(run->dev, iq, 65536, &received);
    int status = birriasdr_get_stream_stats(run->dev, &stats);
    if (!result) result = status;
    if (!result && (received != 65536 || stats.samples != 65536 ||
        stats.firmware_state != 2 || stats.reset_error || stats.dma_errors))
        result = BIRRIASDR_ERROR_PROTOCOL;
    printf("{\"event\":\"recovery\",\"reopen\":%s,\"frequency_hz\":%" PRIu64
           ",\"result\":%d,\"state\":%u,\"stage\":%u,\"reset\":%d}\n",
           reopen ? "true" : "false", frequency, result, stats.firmware_state,
           (unsigned)stats.stage, stats.reset_error);
    return result;
}

/* Own libusb only in the receiver child; the supervisor has no device handle. */
static int run_tests(int trigger_fd, bool automatic)
{
    uint64_t frequencies[3] = {0, UINT64_C(2437000000), 0};
    int result, close_result;
    struct capture run = {0};
    run.trigger_fd = trigger_fd;
    run.automatic = automatic;
    result = birriasdr_open(&run.dev, 0);
    if (!result) result = birriasdr_get_tuning_range(run.dev, &frequencies[0], &frequencies[2]);
    for (unsigned trial = 0; trial < 12 && !result; ++trial) {
        result = birriasdr_set_center_freq(run.dev, UINT64_C(2437000000));
        if (!result) result = force_overrun(&run, trial % 2 == 0);
        if (!result) result = recover(&run, frequencies[(trial / 2) % 3], trial >= 6);
    }
    close_result = run.dev ? birriasdr_close(run.dev) : 0;
    close(trigger_fd);
    printf("{\"event\":\"end\",\"result\":%d,\"close\":%d}\n", result, close_result);
    return result || close_result ? 1 : 0;
}

int main(int argc, char **argv)
{
    int channel[2], child_status = 0;
    if (argc < 2 || argc > 3 || strcmp(argv[1], "--run") ||
        (argc == 3 && strcmp(argv[2], "auto"))) {
        fprintf(stderr, "Usage: %s --run [auto] (forces overruns on a physical dongle)\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    if (pipe(channel)) return 1;
    pid_t child = fork();
    if (child < 0) { close(channel[0]); close(channel[1]); return 1; }
    if (!child) { close(channel[0]); return run_tests(channel[1], argc == 3); }
    close(channel[1]);
    int result = supervise(channel[0], child);
    close(channel[0]);
    if (result) { kill(child, SIGCONT); kill(child, SIGTERM); }
    pid_t waited;
    do { waited = waitpid(child, &child_status, 0); } while (waited < 0 && errno == EINTR);
    return result || waited < 0 || !WIFEXITED(child_status) || WEXITSTATUS(child_status) ? 1 : 0;
}
