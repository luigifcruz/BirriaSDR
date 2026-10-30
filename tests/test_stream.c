/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/* Exercise the real public read engine and packet decoder with a simulated
 * transport/backend. Faults deliberately cross lifecycle stages, including a
 * terminal firmware fault whose submitted packets can never all reach USB.
 * This tests ownership/error reporting, not analog or physical USB behavior.
 */
#include "usb_stream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(t) do { if (!(t)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #t); exit(1); } } while (0)

enum fault { NONE, CREATE, PREPARE, SUBMIT, START, STOP, STATUS, RESET,
             BAD_FRAME, SOURCE_OVERRUN, DMA_ERROR, GAIN_ERROR, TUNE_ERROR, CANCEL_PREPARE,
             TUNE_STOP_ERROR, REMOVED, RETAINED_PATCH, RF_STATE, HOST_OVERRUN,
             OVERRUN_RESET_ERROR, OVERRUN_DMA_ERROR, TIMING_OVERRUN };
struct model {
    enum fault fault;
    unsigned resets, starts, stops, created, destroyed;
    uint32_t limit, generated, terminal;
    bool started, stopped, tune_called;
    unsigned preparations;
    bool recovery_test, persistent;
    bool local_recovery;
    unsigned local_gaps;
    const enum fault *sequence;
    unsigned sequence_length;
};
struct bs_reader { birriasdr_dev_t *dev; };
struct observation { birriasdr_dev_t *dev; uint64_t samples; unsigned callbacks; };

/* The test handle is already open; reaching discovery would be a test bug. */
int bs_usb_discover(uint32_t index, struct birriasdr_device_info *info, birriasdr_dev_t *dev)
{
    (void)index; (void)info; (void)dev;
    abort();
}
void bs_usb_close(birriasdr_dev_t *dev) { (void)dev; abort(); }

/* Retain a borrowed test device until destruction, including failed startup. */
int bs_reader_create(birriasdr_dev_t *dev, unsigned depth, struct bs_reader **out)
{
    struct model *model = dev->io.context;
    CHECK(depth >= 4 && depth <= 256);
    *out = NULL;
    if (model->fault == CREATE) return BIRRIASDR_ERROR_NO_MEMORY;
    *out = calloc(1, sizeof(**out));
    CHECK(*out);
    (*out)->dev = dev;
    ++model->created;
    return 0;
}
int bs_reader_start(struct bs_reader *reader)
{
    struct model *model = reader->dev->io.context;
    return model->fault == SUBMIT ? BIRRIASDR_ERROR_IO : 0;
}
void bs_reader_destroy(struct bs_reader *reader)
{
    if (reader) {
        struct model *model = reader->dev->io.context;
        ++model->destroyed;
        free(reader);
    }
}

/* Generate independently specified wire events. Faulted firmware can advertise
 * more submitted descriptors than it delivered; the host must not await them
 * indefinitely or report a generic timeout instead of the known source fault.
 */
int bs_reader_take(struct bs_reader *reader, uint8_t *bytes, size_t *length, unsigned timeout)
{
    birriasdr_dev_t *dev = reader->dev;
    struct model *model = dev->io.context;
    *length = 0;
    if (model->local_recovery && model->generated &&
        model->generated % 3 == 0 && model->local_gaps < model->generated / 3) {
        memset(bytes, 0, 48);
        bs_put_le32(bytes, 0x3800002c);
        bs_put_le32(bytes + 8, model->local_gaps % 2 ? 8 : 9);
        bs_put_le32(bytes + 12, ++model->local_gaps);
        bs_put_le32(bytes + 24, 0x38000014);
        bytes[28] = 0xed;
        bytes[32] = 0xf6;
        bs_put_le32(bytes + 36, model->generated);
        bs_put_le32(bytes + 40, dev->nonce);
        *length = 48;
        return 0;
    }
    if (model->generated == model->limit || model->stopped) {
        if (model->fault == HOST_OVERRUN) return BIRRIASDR_ERROR_OVERRUN;
        bs_sleep_us(timeout * 1000);
        return BIRRIASDR_ERROR_TIMEOUT;
    }
    CHECK(model->started);
    memset(bytes, 0, BS_FRAME_BYTES);
    bs_put_le32(bytes, 0x38000000u | (BS_FRAME_BYTES - 4));
    bs_put_le32(bytes + 24, 0x38000014u + BS_PAYLOAD_BYTES);
    bytes[28] = 0xed;
    bytes[32] = 0xf5;
    bs_put_le32(bytes + 36, model->generated++);
    bs_put_le32(bytes + 40, dev->nonce);
    if (model->local_recovery) {
        bool packed = bs_packet_samples(dev) < BS_PACKET_SAMPLES;
        for (unsigned offset = 0; offset < BS_PAYLOAD_BYTES; offset += 4) {
            /* The last packet is deliberately invalid: a gap must discard it
             * before the decoder or any application callback can see it. At
             * 10 and 20 MSps the last two words of each 16-byte record are
             * the packer's zero padding. */
            if (packed && offset % 16 >= 8)
                continue;
            bs_put_le32(bytes + 44 + offset, model->generated % 3 ?
                (model->generated << 16) | 7 : UINT32_MAX);
        }
    }
    if (model->fault == BAD_FRAME) bytes[40] ^= 1;
    *length = BS_FRAME_BYTES;
    return 0;
}

/* Backend setup records that reset is needed even if preparation fails. */
static int prepare(birriasdr_dev_t *dev, uint32_t packets)
{
    struct model *model = dev->io.context;
    ++model->preparations;
    /* Model a fresh preparation consuming any pending USB-recovery request. */
    dev->needs_usb_reset = false;
    if (model->sequence)
        model->fault = model->preparations <= model->sequence_length ?
            model->sequence[model->preparations - 1] : NONE;
    if (model->recovery_test) {
        if (!model->persistent && model->preparations > 1) model->fault = NONE;
        model->generated = 0;
        model->started = model->stopped = false;
    }
    dev->touched = true;
    dev->nonce = 0x12345678;
    dev->stats.stage = BIRRIASDR_STAGE_RF;
    model->limit = packets ? packets : model->recovery_test ? 3 : 100;
    model->terminal = packets ? 2 : 3;
    if (model->fault == CANCEL_PREPARE) CHECK(birriasdr_cancel_async(dev) == 0);
    if (model->fault == REMOVED) return BIRRIASDR_ERROR_NO_DEVICE;
    if (model->fault == RETAINED_PATCH) return BIRRIASDR_ERROR_COLD_BOOT;
    if (model->fault == RF_STATE) return BIRRIASDR_ERROR_STATE;
    return model->fault == PREPARE ? BIRRIASDR_ERROR_IO : 0;
}
static int start(birriasdr_dev_t *dev)
{
    struct model *model = dev->io.context;
    ++model->starts;
    model->started = true;
    return model->fault == START ? BIRRIASDR_ERROR_IO : 0;
}
static int stop(birriasdr_dev_t *dev)
{
    struct model *model = dev->io.context;
    ++model->stops;
    model->stopped = true;
    return model->fault == STOP || model->fault == TUNE_STOP_ERROR ? BIRRIASDR_ERROR_IO : 0;
}
static int status(birriasdr_dev_t *dev, uint32_t words[16])
{
    struct model *model = dev->io.context;
    memset(words, 0, 64);
    words[7] = model->stopped ? 3 : model->terminal;
    words[10] = words[11] = model->generated;
    if (model->fault == SOURCE_OVERRUN || model->fault == OVERRUN_RESET_ERROR ||
        model->fault == OVERRUN_DMA_ERROR || model->fault == TIMING_OVERRUN) {
        words[7] = model->fault == TIMING_OVERRUN ? 8 : 9;
        words[10] = words[11] = model->generated + 1;
    }
    if (model->fault == OVERRUN_DMA_ERROR) words[13] = 1;
    if (model->fault == DMA_ERROR) {
        words[13] = 1;
        ++words[10];
    }
    return model->fault == STATUS ? BIRRIASDR_ERROR_IO : 0;
}
static int reset(birriasdr_dev_t *dev)
{
    struct model *model = dev->io.context;
    ++model->resets;
    CHECK(model->created == model->destroyed);
    return model->fault == RESET || model->fault == OVERRUN_RESET_ERROR ?
        BIRRIASDR_ERROR_IO : 0;
}
static int validate(uint64_t frequency, uint32_t rate, const struct birriasdr_gain *gain)
{
    (void)gain;
    return frequency && rate ? 0 : BIRRIASDR_ERROR_INVALID;
}
static int gain(birriasdr_dev_t *dev, bool manual, const struct birriasdr_gain *codes)
{
    struct model *model = dev->io.context;
    (void)manual; (void)codes;
    return model->fault == GAIN_ERROR ? BIRRIASDR_ERROR_IO : 0;
}
static int tune(birriasdr_dev_t *dev, uint64_t frequency)
{
    struct model *model = dev->io.context;
    (void)frequency;
    model->tune_called = true;
    return model->fault == TUNE_STOP_ERROR ? BIRRIASDR_ERROR_STATE :
           model->fault == TUNE_ERROR ? BIRRIASDR_ERROR_IO : 0;
}

static const struct bs_backend backend = {
    .capabilities = BIRRIASDR_CAP_RAW_GAIN | BIRRIASDR_CAP_LIVE_GAIN |
                    BIRRIASDR_CAP_LIVE_TUNING,
    .validate = validate, .prepare = prepare, .start = start, .stop = stop,
    .status = status, .reset = reset, .change_gain = gain, .change_frequency = tune
};

/* Public controls are legal inside callbacks; stop after a small validated
 * prefix. Closing a live handle must return BUSY without freeing anything.
 */
static void observe(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct observation *observation = context;
    birriasdr_dev_t *dev = observation->dev;
    struct model *model = dev->io.context;
    CHECK(iq && count && first == observation->samples);
    observation->samples += count;
    if (observation->callbacks++ == 0) {
        CHECK(birriasdr_close(dev) == BIRRIASDR_ERROR_BUSY);
        if (model->fault == GAIN_ERROR)
            CHECK(birriasdr_set_tuner_gain_mode(dev, 1) == BIRRIASDR_ERROR_IO);
        else if (model->fault == TUNE_ERROR) {
            uint64_t old = birriasdr_get_center_freq(dev);
            CHECK(birriasdr_set_center_freq(dev, old + 29) == BIRRIASDR_ERROR_IO);
            CHECK(birriasdr_get_center_freq(dev) == old);
        }
    }
    if (observation->samples >= 6144) {
        CHECK(birriasdr_cancel_async(dev) == 0);
        CHECK(birriasdr_cancel_async(dev) == 0);
    }
}

/* Check every failure returns ownership to idle, destroys the reader, resets
 * touched hardware and permits a new finite read on that same device handle.
 */
static void test_case(enum fault fault, int expected, uint32_t rate)
{
    struct model model = {.fault = fault};
    birriasdr_dev_t dev = {.backend = &backend, .frequency = 2437000000, .rate = rate,
                          .control_mutex = PTHREAD_MUTEX_INITIALIZER};
    struct observation observation = {.dev = &dev};
    int16_t samples[4096];
    uint32_t received;
    int result;
    atomic_init(&dev.run_state, BS_IDLE);
    dev.io.context = &model;
    result = birriasdr_read_async(&dev, observe, &observation, 4, 2048);
    if (result != expected) fprintf(stderr, "fault=%d expected=%d actual=%d\n", fault, expected, result);
    CHECK(result == expected);
    CHECK(atomic_load(&dev.run_state) == BS_IDLE && !dev.control_ready);
    CHECK(model.created == model.destroyed);
    CHECK(model.resets == (fault == CREATE ? 0u : 1u));
    CHECK(dev.touched == (fault == RESET));
    if (fault == GAIN_ERROR) CHECK(dev.stats.stage == BIRRIASDR_STAGE_GAIN);
    if (fault == TUNE_ERROR) CHECK(dev.stats.stage == BIRRIASDR_STAGE_RF && model.tune_called);
    if (fault == SOURCE_OVERRUN) CHECK(dev.stats.firmware_state == 9);
    if (fault == DMA_ERROR) CHECK(dev.stats.dma_errors == 1);
    memset(&model, 0, sizeof(model));
    CHECK(birriasdr_read_sync(&dev, samples, 2048, &received) == 0);
    CHECK(received == 2048 && dev.stats.samples == 2048 &&
          dev.stats.packets == 2048 / bs_packet_samples(&dev));
    CHECK(!dev.control_error && !dev.touched && model.resets == 1);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

struct recovery_observation {
    birriasdr_dev_t *dev;
    uint64_t samples;
    unsigned gaps;
    unsigned cancel_at_gap;
    bool cancel_in_gap, cancel_after_control;
};

/* A gap arrives after all reader storage has been destroyed and before any
 * samples of the next acquisition. Reentrant close/read/setters remain BUSY.
 */
static void gap(const struct birriasdr_recovery_event *event, void *context)
{
    struct recovery_observation *observation = context;
    birriasdr_dev_t *dev = observation->dev;
    struct model *model = dev->io.context;
    CHECK(event->attempt == ++observation->gaps && event->next_sample == observation->samples);
    CHECK(event->error < 0 && model->created == model->destroyed);
    CHECK(!dev->control_ready && atomic_load(&dev->run_state) == BS_RUNNING);
    CHECK(birriasdr_close(dev) == BIRRIASDR_ERROR_BUSY);
    CHECK(birriasdr_set_center_freq(dev, 2442000000) == BIRRIASDR_ERROR_BUSY);
    CHECK(birriasdr_set_recovery(dev, 0, NULL, NULL) == BIRRIASDR_ERROR_BUSY);
    if (model->fault == TUNE_STOP_ERROR) {
        CHECK(event->error == BIRRIASDR_ERROR_STATE);
        CHECK(event->previous.stage == BIRRIASDR_STAGE_RF);
    }
    if (model->fault == SOURCE_OVERRUN || model->fault == TIMING_OVERRUN)
        CHECK(!dev->needs_usb_reset);
    if (model->fault == HOST_OVERRUN || model->fault == OVERRUN_RESET_ERROR ||
        model->fault == OVERRUN_DMA_ERROR)
        CHECK(dev->needs_usb_reset);
    if (observation->cancel_in_gap || observation->gaps == observation->cancel_at_gap)
        CHECK(birriasdr_cancel_async(dev) == 0);
}

/* Offsets continue to count delivered samples across explicit discontinuities;
 * a failed control must retain the preceding committed center or gain mode.
 */
static void observe_recovery(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct recovery_observation *observation = context;
    birriasdr_dev_t *dev = observation->dev;
    struct model *model = dev->io.context;
    CHECK(iq && count && first == observation->samples);
    observation->samples += count;
    if (!dev->control_error && (model->fault == TUNE_ERROR || model->fault == TUNE_STOP_ERROR)) {
        uint64_t frequency = birriasdr_get_center_freq(dev);
        CHECK(birriasdr_set_center_freq(dev, frequency + 29) ==
              (model->fault == TUNE_ERROR ? BIRRIASDR_ERROR_IO : BIRRIASDR_ERROR_STATE));
        CHECK(birriasdr_get_center_freq(dev) == frequency);
    } else if (!dev->control_error && model->fault == GAIN_ERROR) {
        CHECK(birriasdr_set_tuner_gain_mode(dev, 1) == BIRRIASDR_ERROR_IO);
        CHECK(!dev->manual);
    }
    if (model->fault == NONE || observation->cancel_after_control)
        CHECK(birriasdr_cancel_async(dev) == 0);
}

/* Recover one-shot failures, bound persistent device faults and keep restarting
 * clean overflows until explicitly cancelled, beyond the configured budget.
 */
static void test_recovery(enum fault fault, bool persistent, bool cancel_gap,
                          bool cancel_control, bool finite, int expected, unsigned gaps, uint32_t rate)
{
    struct model model = {.fault = fault, .recovery_test = true, .persistent = persistent};
    birriasdr_dev_t dev = {.backend = &backend, .frequency = 2437000000, .rate = rate,
                          .control_mutex = PTHREAD_MUTEX_INITIALIZER};
    struct recovery_observation observation = {.dev = &dev, .cancel_in_gap = cancel_gap,
                                              .cancel_after_control = cancel_control};
    if (persistent && (fault == SOURCE_OVERRUN || fault == HOST_OVERRUN ||
                       fault == TIMING_OVERRUN))
        observation.cancel_at_gap = gaps;
    atomic_init(&dev.run_state, BS_IDLE);
    dev.io.context = &model;
    CHECK(birriasdr_set_recovery(&dev, 2, NULL, NULL) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_set_recovery(&dev, 2, gap, &observation) == 0);
    int result;
    if (finite) {
        int16_t iq[4096];
        uint32_t received;
        result = birriasdr_read_sync(&dev, iq, 2048, &received);
    } else {
        result = birriasdr_read_async(&dev, observe_recovery, &observation, 4, 2048);
    }
    if (result != expected) fprintf(stderr, "recovery fault=%d result=%d expected=%d\n", fault, result, expected);
    CHECK(result == expected && observation.gaps == gaps);
    CHECK(model.preparations == 1 + gaps - ((cancel_gap || observation.cancel_at_gap) ? 1u : 0u));
    CHECK(model.created == model.destroyed);
    CHECK(model.resets == (fault == REMOVED ? 0u : model.preparations));
    CHECK(atomic_load(&dev.run_state) == BS_IDLE && !dev.control_ready);
    if (!finite) CHECK(dev.stats.samples == observation.samples);
    if (fault == REMOVED) {
        unsigned preparations = model.preparations, created = model.created;
        CHECK(dev.usb_lost && !dev.touched);
        CHECK(birriasdr_read_async(&dev, observe_recovery, &observation, 4, 2048) ==
              BIRRIASDR_ERROR_NO_DEVICE);
        CHECK(model.preparations == preparations && model.created == created && !model.resets);
    }
    CHECK(birriasdr_set_recovery(&dev, 0, NULL, NULL) == 0);
    CHECK(!dev.recovery_callback && !dev.recovery_limit);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Overflows neither spend nor replenish the separate transport-fault budget.
 * A zero budget disables recovery and remains strict.
 */
static void test_mixed_overflows(uint32_t budget, uint32_t rate)
{
    const enum fault sequence[] = {SOURCE_OVERRUN, PREPARE, SOURCE_OVERRUN,
                                  HOST_OVERRUN, PREPARE, TIMING_OVERRUN, PREPARE};
    struct model model = {.recovery_test = true, .persistent = true,
                         .sequence = sequence, .sequence_length = 7};
    birriasdr_dev_t dev = {.backend = &backend, .frequency = 2437000000, .rate = rate,
                          .control_mutex = PTHREAD_MUTEX_INITIALIZER};
    struct recovery_observation observation = {.dev = &dev};
    atomic_init(&dev.run_state, BS_IDLE);
    dev.io.context = &model;
    CHECK(birriasdr_set_recovery(&dev, budget, gap, &observation) == 0);
    int result = birriasdr_read_async(&dev, observe_recovery, &observation, 4, 2048);
    CHECK(result == (budget ? BIRRIASDR_ERROR_IO : BIRRIASDR_ERROR_OVERRUN));
    CHECK(observation.gaps == (budget ? 6u : 0u));
    CHECK(model.preparations == (budget ? 7u : 1u));
    CHECK(model.created == model.destroyed && model.resets == model.preparations);
    CHECK(dev.stats.samples == observation.samples);
    CHECK(atomic_load(&dev.run_state) == BS_IDLE);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

struct local_observation {
    birriasdr_dev_t *dev;
    uint64_t samples;
    unsigned gaps;
    bool cancel_gap;
};

/* A local gap keeps the transport and live controls available. Buffered old
 * samples are flushed before this event, and no uncertain packet is exposed.
 */
static void local_gap(const struct birriasdr_recovery_event *event, void *context)
{
    struct local_observation *o = context;
    struct model *m = o->dev->io.context;
    uint32_t packet_samples = bs_packet_samples(o->dev);
    CHECK(event->attempt == ++o->gaps && event->next_sample == o->samples);
    CHECK(event->error == BIRRIASDR_ERROR_OVERRUN);
    CHECK(event->previous.firmware_state == (o->gaps % 2 ? 9u : 8u));
    CHECK(event->previous.packets == o->gaps * 3);
    CHECK(o->samples == (uint64_t)o->gaps * 2 * packet_samples);
    CHECK(m->preparations == 1 && m->created == 1 && !m->destroyed && !m->resets);
    CHECK(o->dev->control_ready && !o->dev->needs_usb_reset);
    if (atomic_load(&o->dev->run_state) == BS_RUNNING) {
        CHECK(birriasdr_set_center_freq(o->dev, 2437000000 + o->gaps * 1000000) == 0);
        CHECK(birriasdr_set_tuner_gain_mode(o->dev, 1) == 0);
    } else {
        CHECK(atomic_load(&o->dev->run_state) == BS_CANCELLING);
        CHECK(birriasdr_set_center_freq(o->dev, 2437000000) == BIRRIASDR_ERROR_BUSY);
    }
    if (o->cancel_gap && o->gaps == 6)
        CHECK(birriasdr_cancel_async(o->dev) == 0);
}

/* Pattern values identify every delivered packet. Each third source packet is
 * invalid and must be absent, with exactly one gap before the next good pair.
 */
static void local_samples(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct local_observation *o = context;
    uint32_t packet_samples = bs_packet_samples(o->dev);
    CHECK(first == o->samples && count % packet_samples == 0);
    for (uint32_t n = 0; n < count; ++n) {
        uint64_t packet = (first + n) / packet_samples;
        CHECK(iq[n * 2] == (int)(packet + packet / 2 + 1));
        CHECK(iq[n * 2 + 1] == 7);
        CHECK(o->gaps == packet / 2);
    }
    o->samples += count;
    if (!o->cancel_gap && o->gaps >= 6)
        CHECK(birriasdr_cancel_async(o->dev) == 0);
}

/* Local recovery must bypass the host restart budget and preserve one reader,
 * one boot and one nonce. Cover partial callback buffers and cancellation both
 * at a gap and after fresh data. Finite and strict paths are tested separately.
 */
static void test_local_recovery(uint32_t rate, bool cancel_gap)
{
    struct model m = {.local_recovery = true};
    birriasdr_dev_t dev = {.backend = &backend, .frequency = 2437000000, .rate = rate,
                          .control_mutex = PTHREAD_MUTEX_INITIALIZER};
    struct local_observation o = {.dev = &dev, .cancel_gap = cancel_gap};
    atomic_init(&dev.run_state, BS_IDLE);
    dev.io.context = &m;
    CHECK(birriasdr_set_recovery(&dev, 1, local_gap, &o) == 0);
    CHECK(birriasdr_read_async(&dev, local_samples, &o, 4, 8192) == 0);
    CHECK(dev.stats.samples == o.samples);
    CHECK(o.gaps >= 6);
    CHECK(m.preparations == 1 && m.created == 1 && m.destroyed == 1 && m.resets == 1);
    CHECK(atomic_load(&dev.run_state) == BS_IDLE && !dev.control_ready);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Run with no USB access. Each scenario drives the same implementation as users. */
static void test_rate(uint32_t rate)
{
    static const int expected[] = {0, -7, -5, -5, -5, -5, -5, -5, -10, -11, -10, -5, -5, 0};
    for (unsigned fault = 0; fault < sizeof(expected) / sizeof(expected[0]); ++fault)
        test_case((enum fault)fault, expected[fault], rate);
    const enum fault temporary[] = {PREPARE, SUBMIT, START, SOURCE_OVERRUN, HOST_OVERRUN,
                                    TIMING_OVERRUN,
                                    GAIN_ERROR, TUNE_ERROR, TUNE_STOP_ERROR};
    for (size_t i = 0; i < sizeof(temporary) / sizeof(temporary[0]); ++i)
        test_recovery(temporary[i], false, false, false, false, 0, 1, rate);
    test_recovery(PREPARE, true, false, false, false, BIRRIASDR_ERROR_IO, 2, rate);
    test_recovery(SOURCE_OVERRUN, true, false, false, false, 0, 6, rate);
    test_recovery(HOST_OVERRUN, true, false, false, false, 0, 6, rate);
    test_recovery(TIMING_OVERRUN, true, false, false, false, 0, 6, rate);
    test_recovery(OVERRUN_RESET_ERROR, true, false, false, false, BIRRIASDR_ERROR_OVERRUN, 2, rate);
    test_recovery(OVERRUN_DMA_ERROR, true, false, false, false, BIRRIASDR_ERROR_OVERRUN, 2, rate);
    test_recovery(SOURCE_OVERRUN, false, true, false, false, 0, 1, rate);
    test_recovery(SOURCE_OVERRUN, false, false, false, true, BIRRIASDR_ERROR_OVERRUN, 0, rate);
    test_mixed_overflows(0, rate);
    test_mixed_overflows(2, rate);
    test_recovery(TUNE_ERROR, false, false, true, false, BIRRIASDR_ERROR_IO, 0, rate);
    test_recovery(PREPARE, false, false, false, true, BIRRIASDR_ERROR_IO, 0, rate);
    test_recovery(REMOVED, false, false, false, false, BIRRIASDR_ERROR_NO_DEVICE, 0, rate);
    test_recovery(RETAINED_PATCH, false, false, false, false, BIRRIASDR_ERROR_COLD_BOOT, 0, rate);
    test_recovery(RF_STATE, false, false, false, false, BIRRIASDR_ERROR_STATE, 0, rate);
    test_recovery(BAD_FRAME, false, false, false, false, BIRRIASDR_ERROR_PROTOCOL, 0, rate);
}

/* Every rate runs the whole suite. 10 and 20 MSps deliver packed payloads,
 * with half as many samples per packet.
 */
int main(void)
{
    static const uint32_t rates[] = {10000000, 20000000, 40000000, 80000000};
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); ++i) {
        test_local_recovery(rates[i], false);
        test_local_recovery(rates[i], true);
        test_rate(rates[i]);
    }
    puts("stream lifecycle and recovery tests passed at 10, 20, 40 and 80 MSps");
    return 0;
}
