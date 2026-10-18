/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/*
 * Explicit public-API hardware stress test for the supported MT7961 board.
 *
 * A successful run establishes transport/control behavior on the attached
 * board, not sensitivity, oscillator accuracy or performance on another host.
 * Sweep every 1 MHz in both directions, including both command endpoints.
 * The restart phase uses finite reads upward and cancelled streams downward.
 * Sweeps cover each tuning band separately; BIRRIA_STABILITY_STEP_HZ sets the step.
 * The live phase keeps one acquisition running across both sweeps, fine-step
 * requests, all 468 raw gain combinations, and release/manual transitions.
 * The soak phase holds one stream open for five minutes.
 *
 * Only public operations are used. The library validates every packet and raw
 * sample; this callback also checks delivered offsets. A separate application
 * thread performs live controls, leaving sample callbacks free to drain USB.
 * Reports are JSON lines and captures are discarded, avoiding filesystem load
 * at the sample rate. Every error stops the phase and attempts a recovery read.
 * No driver detach, power cycle, vendor download or transmit mode is used.
 */
#include "birriasdr.h"
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FINITE_SAMPLES 65536u
#define STEP_HZ UINT64_C(1000000)  /* Default; BIRRIA_STABILITY_STEP_HZ overrides. */
#define SOAK_SECONDS 300u

struct run {
    birriasdr_dev_t *dev;
    uint64_t minimum, maximum, step;
    atomic_uint_fast64_t samples;
    atomic_int callback_error;
    atomic_bool done;
    uint64_t cancel_after;
    bool cancel_sent;
    bool soak;
    int control_error;
    unsigned changes;
};

/* Use elapsed monotonic time, independent of wall-clock synchronization. */
static uint64_t now_ns(void)
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (uint64_t)time.tv_sec * UINT64_C(1000000000) + (uint64_t)time.tv_nsec;
}

/* Sleep only in the control thread; the callback never waits or prints. */
static void pause_ms(unsigned ms)
{
    struct timespec delay = {ms / 1000, (long)(ms % 1000) * 1000000};
    nanosleep(&delay, NULL);
}

/* Verify callback continuity independently of the device packet counters. */
static void receive(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct run *run = context;
    if (!iq || !count || first != atomic_load(&run->samples))
        atomic_store(&run->callback_error, BIRRIASDR_ERROR_PROTOCOL);
    atomic_fetch_add(&run->samples, count);
    if (!run->cancel_sent && (atomic_load(&run->callback_error) ||
        (run->cancel_after && first + count >= run->cancel_after))) {
        run->cancel_sent = true;
        birriasdr_cancel_async(run->dev);
    }
}

/* Inspect final state/counts and cleanup after every individual acquisition. */
static int report_capture(struct run *run, const char *phase, int result,
                          uint64_t expected, uint32_t terminal)
{
    struct birriasdr_stream_stats stats = {0};
    int status = birriasdr_get_stream_stats(run->dev, &stats);
    if (!result) result = status;
    if (!result) result = atomic_load(&run->callback_error);
    if (!result && (stats.firmware_state != terminal || stats.dma_errors ||
        stats.reset_error || stats.samples != expected || stats.samples != stats.packets * 2048))
        result = BIRRIASDR_ERROR_PROTOCOL;
    printf("{\"event\":\"capture\",\"phase\":\"%s\",\"frequency_hz\":%" PRIu64
           ",\"result\":%d,\"samples\":%" PRIu64 ",\"packets\":%" PRIu64
           ",\"state\":%u,\"stage\":%u,\"dma_errors\":%u,\"reset\":%d}\n",
           phase, birriasdr_get_center_freq(run->dev), result, stats.samples,
           stats.packets, stats.firmware_state, (unsigned)stats.stage, stats.dma_errors,
           stats.reset_error);
    return result;
}

/* Alternate released/manual starts and cover gain indices without assuming dB. */
static int stage_settings(struct run *run, uint64_t frequency, unsigned trial)
{
    struct birriasdr_gain gain = {trial % 6, (trial / 6) % 9, (trial / 54) % 13};
    struct birriasdr_gain actual;
    int mode, manual = trial % 3 != 0;
    int result = birriasdr_set_center_freq(run->dev, frequency);
    if (!result) result = birriasdr_set_tuner_gain_mode(run->dev, 1);
    if (!result) result = birriasdr_set_gain_codes(run->dev, &gain);
    if (!result) result = birriasdr_set_tuner_gain_mode(run->dev, manual);
    if (!result) result = birriasdr_get_gain_codes(run->dev, &actual, &mode);
    if (!result && (mode != manual || actual.lna != gain.lna || actual.lpf != gain.lpf ||
        actual.digital != gain.digital || birriasdr_get_center_freq(run->dev) != frequency))
        result = BIRRIASDR_ERROR_STATE;
    return result;
}

/* Repeated finite and cancelled reads reuse one handle; periodically reopen it
 * to also exercise descriptor discovery, interface release and lock ownership.
 */
/* Load band `index` into run->minimum and maximum. Returns 1 past the last band,
 * -1 for a band this board cannot reach and 0 when the band should be swept.
 */
static int load_band(struct run *run, uint32_t index)
{
    int result = birriasdr_get_tuning_band(run->dev, index, &run->minimum, &run->maximum);
    if (result == BIRRIASDR_ERROR_INVALID) return 1;
    printf("{\"event\":\"band\",\"index\":%u,\"minimum_hz\":%" PRIu64 ",\"maximum_hz\":%" PRIu64
           ",\"result\":%d}\n", index, run->minimum, run->maximum, result);
    return result ? -1 : 0;
}

static int restart_band(struct run *run, int16_t *iq, unsigned *trial_count)
{
    unsigned trial = *trial_count;
    uint64_t intervals = (run->maximum - run->minimum + run->step - 1) / run->step;
    for (unsigned direction = 0; direction < 2; ++direction) {
        for (uint64_t i = 0; i <= intervals; ++i, ++trial, *trial_count = trial) {
            uint64_t offset = i * run->step;
            uint32_t received = 0;
            int result;
            if (offset > run->maximum - run->minimum) offset = run->maximum - run->minimum;
            result = stage_settings(run, direction ? run->maximum - offset : run->minimum + offset, trial);
            if (result) return result;
            atomic_store(&run->samples, 0);
            run->cancel_sent = false;
            run->cancel_after = FINITE_SAMPLES;
            if (!direction) {
                result = birriasdr_read_sync(run->dev, iq, FINITE_SAMPLES, &received);
                result = report_capture(run, "finite_up", result, FINITE_SAMPLES, 2);
                if (!result && received != FINITE_SAMPLES) result = BIRRIASDR_ERROR_PROTOCOL;
            } else {
                result = birriasdr_read_async(run->dev, receive, run, 0, 0);
                result = report_capture(run, "cancel_down", result, atomic_load(&run->samples), 3);
            }
            if (result) return result;
            if (trial % 31 == 30) {
                result = birriasdr_close(run->dev);
                run->dev = NULL;
                if (!result) result = birriasdr_open(&run->dev, 0);
                printf("{\"event\":\"reopen\",\"trial\":%u,\"result\":%d}\n", trial, result);
                if (result) return result;
            }
        }
    }
    return 0;
}

/* Sweep every reachable band; the gaps between bands are never requested. */
static int restart_sweep(struct run *run, int16_t *iq)
{
    unsigned trial = 0;
    for (uint32_t band = 0;; ++band) {
        int state = load_band(run, band), result;
        if (state > 0) return 0;
        if (state < 0) continue;
        result = restart_band(run, iq, &trial);
        if (result) return result;
    }
}

/* Invalid live requests must preserve the committed center and running stream. */
static int invalid_controls(struct run *run)
{
    const struct birriasdr_gain invalid = {6, 0, 0};
    uint64_t before = birriasdr_get_center_freq(run->dev);
    if (birriasdr_set_center_freq(run->dev, run->minimum - 1) != BIRRIASDR_ERROR_UNSUPPORTED ||
        birriasdr_set_center_freq(run->dev, run->maximum + 1) != BIRRIASDR_ERROR_UNSUPPORTED ||
        birriasdr_set_center_freq(run->dev, UINT64_MAX) != BIRRIASDR_ERROR_UNSUPPORTED ||
        birriasdr_set_gain_codes(run->dev, &invalid) != BIRRIASDR_ERROR_INVALID ||
        birriasdr_get_center_freq(run->dev) != before)
        return BIRRIASDR_ERROR_STATE;
    return 0;
}

/* Log the command acknowledgment, never claim it identifies an RF sample boundary. */
static int live_frequency(struct run *run, uint64_t frequency)
{
    uint64_t start = now_ns(), before = atomic_load(&run->samples);
    int result;
    if (atomic_load(&run->done)) return BIRRIASDR_ERROR_STATE;
    result = birriasdr_set_center_freq(run->dev, frequency);
    if (!result && birriasdr_get_center_freq(run->dev) != frequency)
        result = BIRRIASDR_ERROR_STATE;
    ++run->changes;
    printf("{\"event\":\"tune\",\"frequency_hz\":%" PRIu64 ",\"result\":%d,"
           "\"duration_ns\":%" PRIu64 ",\"host_before\":%" PRIu64 ",\"host_after\":%" PRIu64 "}\n",
           frequency, result, now_ns() - start, before, atomic_load(&run->samples));
    return result;
}

/* Sweep all raw gain triples, including released/manual round trips. A successful
 * register transaction and getter do not establish calibrated gain or complete AGC.
 */
static int live_gains(struct run *run)
{
    for (unsigned i = 0; i < 6 * 9 * 13; ++i) {
        struct birriasdr_gain gain = {i % 6, (i / 6) % 9, i / 54}, actual;
        int manual, result;
        if (atomic_load(&run->done)) return BIRRIASDR_ERROR_STATE;
        result = birriasdr_set_gain_codes(run->dev, &gain);
        if (!result) result = birriasdr_get_gain_codes(run->dev, &actual, &manual);
        if (!result && (!manual || actual.lna != gain.lna || actual.lpf != gain.lpf ||
            actual.digital != gain.digital)) result = BIRRIASDR_ERROR_STATE;
        if (!result && i % 13 == 0) {
            result = birriasdr_set_tuner_gain_mode(run->dev, 0);
            if (!result) result = birriasdr_get_gain_codes(run->dev, &actual, &manual);
            if (!result && manual) result = BIRRIASDR_ERROR_STATE;
            if (!result) result = birriasdr_set_tuner_gain_mode(run->dev, 1);
        }
        printf("{\"event\":\"gain\",\"lna\":%u,\"lpf\":%u,\"digital\":%u,\"result\":%d}\n",
               gain.lna, gain.lpf, gain.digital, result);
        if (result) return result;
    }
    return 0;
}

struct gain_job { struct run *run; int result; };

/* Give gain changes their own application thread while another thread tunes.
 * The library must serialize complete transactions without blocking callbacks.
 */
static void *gain_thread(void *context)
{
    struct gain_job *job = context;
    job->result = live_gains(job->run);
    return NULL;
}

/* Visit both bounds in both directions, then exercise individual low fractional
 * bits and non-channel requests. Leave one complete gain sweep at channel 6.
 */
static int live_controls(struct run *run)
{
    static const uint64_t fine[] = {2400000000, 2400000029, 2400000057, 2400000086,
                                    2437123456, 2437000000};
    int result = 0;
    for (uint32_t band = 0;; ++band) {
        int state = load_band(run, band);
        uint64_t intervals;
        if (state > 0) break;
        if (state < 0) continue;
        intervals = (run->maximum - run->minimum + run->step - 1) / run->step;
        result = invalid_controls(run);
        if (result) return result;
        for (unsigned direction = 0; direction < 2; ++direction) {
            for (uint64_t i = 0; i <= intervals; ++i) {
                uint64_t offset = i * run->step;
                if (offset > run->maximum - run->minimum) offset = run->maximum - run->minimum;
                result = live_frequency(run, direction ? run->maximum - offset : run->minimum + offset);
                if (result) return result;
            }
        }
    }
    for (size_t i = 0; i < sizeof(fine) / sizeof(fine[0]); ++i) {
        result = live_frequency(run, fine[i]);
        if (result) return result;
    }
    struct gain_job job = {.run = run};
    pthread_t gain_worker;
    if (pthread_create(&gain_worker, NULL, gain_thread, &job)) return BIRRIASDR_ERROR_IO;
    for (unsigned i = 0; i < 16 && !result; ++i)
        result = live_frequency(run, UINT64_C(2437000000) + (i % 2 ? 29 : 57));
    pthread_join(gain_worker, NULL);
    if (!result) result = job.result;
    return result ? result : live_frequency(run, UINT64_C(2437000000));
}

/* Wait for real samples before issuing controls; stop if startup/read fails.
 * A soak checks progress once a second and does not modify register state.
 */
static void *control_thread(void *context)
{
    struct run *run = context;
    uint64_t deadline = now_ns() + UINT64_C(10000000000);
    while (!atomic_load(&run->samples) && !atomic_load(&run->done) && now_ns() < deadline)
        pause_ms(1);
    if (!atomic_load(&run->samples) || atomic_load(&run->done))
        run->control_error = BIRRIASDR_ERROR_TIMEOUT;
    else if (!run->soak)
        run->control_error = live_controls(run);
    else {
        uint64_t previous = atomic_load(&run->samples);
        for (unsigned second = 0; second < SOAK_SECONDS; ++second) {
            uint64_t current;
            pause_ms(1000);
            current = atomic_load(&run->samples);
            if (atomic_load(&run->done) || current == previous) {
                run->control_error = BIRRIASDR_ERROR_TIMEOUT;
                break;
            }
            if (second % 10 == 9)
                printf("{\"event\":\"soak\",\"seconds\":%u,\"samples\":%" PRIu64 "}\n", second + 1, current);
            previous = current;
        }
    }
    birriasdr_cancel_async(run->dev);
    return NULL;
}

/* Join the control thread before statistics/close and verify a clean terminal.
 * The worker records failures separately so transport errors are not concealed.
 */
static int live_run(struct run *run, bool soak)
{
    pthread_t control;
    int result = stage_settings(run, UINT64_C(2437000000), 1);
    if (result) return result;
    atomic_store(&run->samples, 0);
    atomic_store(&run->done, false);
    run->cancel_after = 0;
    run->cancel_sent = false;
    run->control_error = 0;
    run->soak = soak;
    if (pthread_create(&control, NULL, control_thread, run)) return BIRRIASDR_ERROR_IO;
    result = birriasdr_read_async(run->dev, receive, run, 0, 0);
    atomic_store(&run->done, true);
    pthread_join(control, NULL);
    if (!result) result = run->control_error;
    return report_capture(run, soak ? "soak" : "live", result, atomic_load(&run->samples), 3);
}

/* Require explicit hardware authorization via --run and keep earlier failures
 * visible even when a subsequent recovery read and device close succeed.
 */
int main(int argc, char **argv)
{
    struct run run = {0};
    const char *phase = argc == 3 ? argv[2] : "all";
    int result, closed;
    int16_t *iq;
    if (argc < 2 || argc > 3 || strcmp(argv[1], "--run") ||
        (strcmp(phase, "all") && strcmp(phase, "restart") && strcmp(phase, "live") && strcmp(phase, "soak"))) {
        fprintf(stderr, "Usage: %s --run [all|restart|live|soak]\n", argv[0]);
        return 2;
    }
    setvbuf(stdout, NULL, _IOLBF, 0);
    iq = malloc(FINITE_SAMPLES * 4);
    if (!iq) return 1;
    result = birriasdr_open(&run.dev, 0);
    run.step = getenv("BIRRIA_STABILITY_STEP_HZ") ? strtoull(getenv("BIRRIA_STABILITY_STEP_HZ"), NULL, 10) : STEP_HZ;
    if (!run.step) run.step = STEP_HZ;
    if (!result) result = birriasdr_get_tuning_range(run.dev, &run.minimum, &run.maximum);
    printf("{\"event\":\"begin\",\"phase\":\"%s\",\"minimum_hz\":%" PRIu64
           ",\"maximum_hz\":%" PRIu64 ",\"step_hz\":%" PRIu64 ",\"result\":%d}\n",
           phase, run.minimum, run.maximum, run.step, result);
    if (!result && (!strcmp(phase, "all") || !strcmp(phase, "restart"))) result = restart_sweep(&run, iq);
    if (!result && (!strcmp(phase, "all") || !strcmp(phase, "live"))) result = live_run(&run, false);
    if (!result && (!strcmp(phase, "all") || !strcmp(phase, "soak"))) result = live_run(&run, true);
    if (run.dev) {
        uint32_t received = 0;
        int recovery = stage_settings(&run, UINT64_C(2437000000), 1);
        if (!recovery) recovery = birriasdr_read_sync(run.dev, iq, FINITE_SAMPLES, &received);
        atomic_store(&run.callback_error, 0);
        recovery = report_capture(&run, "recovery", recovery, FINITE_SAMPLES, 2);
        if (!result) result = recovery;
    }
    closed = run.dev ? birriasdr_close(run.dev) : 0;
    free(iq);
    printf("{\"event\":\"end\",\"result\":%d,\"close\":%d,\"live_tunes\":%u}\n", result, closed, run.changes);
    return result || closed ? 1 : 0;
}
