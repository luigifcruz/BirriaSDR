/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "usb_stream.h"
#include <stdlib.h>
#include <string.h>

/* macOS can delay USB completions long enough to drain 64 queued transfers. */
#ifdef __APPLE__
#define BS_DEFAULT_DEPTH 256u
#else
#define BS_DEFAULT_DEPTH 64u
#endif

struct delivery {
    birriasdr_dev_t *dev;
    birriasdr_read_async_cb_t callback;
    void *context;
    int16_t *iq;
    uint32_t capacity, used;
    uint32_t restarts;
    uint64_t delivered, segment_start;
    uint8_t held[BS_PAYLOAD_BYTES];
    bool holdback, held_valid;
};

/* Deliver only complete validated samples. A short final buffer is allowed;
 * sequence offsets are complex-sample counts, independent of callback sizing.
 */
static void flush_samples(struct delivery *delivery)
{
    if (!delivery->used)
        return;
    delivery->callback(delivery->iq, delivery->used, delivery->delivered, delivery->context);
    delivery->delivered += delivery->used;
    delivery->dev->stats.samples = delivery->delivered - delivery->segment_start;
    delivery->used = 0;
}

/* Reorder one complete raw Q,I payload into public I,Q pairs. The fixed
 * format supplies 2048 samples (1024 at 20 MSps); callback capacity is a whole
 * number of full-rate packets, which also divides evenly at 20 MSps.
 */
static int decode_payload(struct delivery *delivery, const uint8_t *raw)
{
    if (delivery->dev->rate == 20000000)
        BS_TRY(bs_decode_iq_halved(raw, delivery->iq + 2u * delivery->used, BS_PACKET_SAMPLES));
    else
        BS_TRY(bs_decode_iq(raw, delivery->iq + 2u * delivery->used, BS_PACKET_SAMPLES));
    delivery->used += bs_packet_samples(delivery->dev);
    if (delivery->used == delivery->capacity)
        flush_samples(delivery);
    return 0;
}

/* With local recovery enabled, the next data header confirms that the prior
 * DMA packet survived the source checks. A gap marker instead invalidates it.
 * Keep its raw bytes until then, so even overwritten sample bits cannot leak
 * into callbacks or cause a spurious decoder failure before the marker arrives.
 */
static int receive_payload(void *context, const uint8_t *raw)
{
    struct delivery *delivery = context;
    if (!delivery->holdback)
        return decode_payload(delivery, raw);
    if (delivery->held_valid)
        BS_TRY(decode_payload(delivery, delivery->held));
    memcpy(delivery->held, raw, sizeof(delivery->held));
    delivery->held_valid = true;
    return 0;
}

/* One notification counter spans local gaps and full acquisition restarts.
 * Saturate at the largest public counter value rather than wrapping to zero.
 */
static uint32_t next_attempt(struct delivery *delivery)
{
    if (delivery->restarts != UINT32_MAX)
        ++delivery->restarts;
    return delivery->restarts;
}

/* The ordered device marker discards the single possibly overwritten packet.
 * Deliver the preceding validated prefix before announcing the gap. Receiver
 * settings, the USB reader and the boot nonce remain live, so no host reset or
 * artificial backoff is needed. Missing sample counts are still unknown.
 */
static int receive_gap(void *context, uint32_t state, uint64_t packets)
{
    struct delivery *delivery = context;
    birriasdr_dev_t *dev = delivery->dev;
    delivery->held_valid = false;
    flush_samples(delivery);
    struct birriasdr_recovery_event event = {
        .next_sample = delivery->delivered, .attempt = next_attempt(delivery),
        .error = BIRRIASDR_ERROR_OVERRUN, .previous = dev->stats
    };
    event.previous.firmware_state = state;
    event.previous.packets = packets;
    dev->recovery_callback(&event, dev->recovery_context);
    return 0;
}

/* Validate terminal firmware state before waiting for its advertised packets.
 * A fault can strand submitted descriptors that will never arrive over USB.
 * Preserve that known fault instead of waiting for them and reporting timeout.
 */
static int validate_terminal_status(birriasdr_dev_t *dev, const uint32_t words[16], uint32_t limit)
{
    dev->stats.firmware_state = words[7];
    dev->stats.stage = BIRRIASDR_STAGE_VERIFY;
    dev->stats.dma_errors = words[13];
    if (words[7] == 8 || words[7] == 9)
        return BIRRIASDR_ERROR_OVERRUN;
    if (words[7] == 4 || words[7] == 5 || words[7] == 10)
        return BIRRIASDR_ERROR_TIMEOUT;
    if (words[7] != 2 && words[7] != 3)
        return BIRRIASDR_ERROR_STATE;
    if (words[13] || words[10] != words[11])
        return BIRRIASDR_ERROR_PROTOCOL;
    if (words[7] == 2 && (!limit || words[10] != limit))
        return BIRRIASDR_ERROR_PROTOCOL;
    unsigned state = atomic_load(&dev->run_state);
    if (words[7] == 3 && state != BS_CANCELLING && state != BS_CONTROL_FAILED)
        return BIRRIASDR_ERROR_STATE;
    return 0;
}

/* Serialize complete control procedures with live setters. The USB event pump
 * does not acquire this lock, so sample transfers continue during RF updates.
 * Closing the live gate before STOP prevents a late update on a parked MCU.
 */
static int stop_capture(birriasdr_dev_t *dev)
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    result = dev->backend->stop(dev);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* A terminal status also closes the live-control gate. A finite capture can have
 * already ended while buffered samples are still being delivered to the host.
 */
static int capture_status(birriasdr_dev_t *dev, uint32_t words[16])
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    result = dev->backend->status(dev, words);
    if (result < 0 || words[7] != 1)
        dev->control_ready = false;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Publish live readiness only after initialization, queued reads and GO. An
 * application control thread cannot interleave RF operations with startup.
 */
static int prepare_capture(birriasdr_dev_t *dev, struct bs_reader *reader, uint32_t packets)
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    result = dev->backend->prepare(dev, packets);
    if (!result)
        result = bs_reader_start(reader);
    if (!result)
        result = dev->backend->start(dev);
    dev->control_ready = result == 0 && atomic_load(&dev->run_state) == BS_RUNNING;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Consume independently of the USB event pump. EP0 status reads are deferred
 * until a finite run finishes, cancellation is requested, or progress pauses;
 * hammering EP0 during capture would itself compete with the data endpoint.
 */
static int receive_loop(birriasdr_dev_t *dev, struct bs_reader *reader,
                        struct bs_framer *framer, uint32_t limit, uint8_t *bytes)
{
    uint64_t last_progress = bs_now_ns(), next_poll = 0, expected_packets = 0;
    uint32_t words[16] = {0};
    bool stop_sent = false, terminal = false;
    for (;;) {
        size_t length;
        uint64_t now;
        int result;
        unsigned state = atomic_load(&dev->run_state);
        if ((state == BS_CANCELLING || state == BS_CONTROL_FAILED) && !stop_sent) {
            BS_TRY(stop_capture(dev));
            stop_sent = true;
        }
        result = bs_reader_take(reader, bytes, &length, 20);
        if (result != BIRRIASDR_ERROR_TIMEOUT) {
            BS_TRY(result);
            dev->stats.usb_bytes += length;
            BS_TRY(bs_framer_feed(framer, bytes, length));
            dev->stats.packets = framer->packets;
            if (length)
                last_progress = bs_now_ns();
        }
        now = bs_now_ns();
        if (!terminal && now >= next_poll && (stop_sent ||
            (limit && framer->packets >= limit) || now - last_progress > 250000000)) {
            BS_TRY(capture_status(dev, words));
            terminal = words[7] != 1;
            if (terminal) {
                uint32_t remaining;
                BS_TRY(validate_terminal_status(dev, words, limit));
                /* Wire counters wrap at 32 bits. The bounded transport cannot
                 * retain half that many packets; a backwards count is invalid. */
                remaining = words[10] - (uint32_t)framer->packets;
                if (remaining > UINT32_MAX / 2)
                    return BIRRIASDR_ERROR_PROTOCOL;
                expected_packets = framer->packets + remaining;
            }
            next_poll = now + 50000000;
        }
        if (terminal && framer->packets >= expected_packets) {
            /* Drain already queued/trailing events: duplicates must fail too. */
            do {
                result = bs_reader_take(reader, bytes, &length, 20);
                if (result == BIRRIASDR_ERROR_TIMEOUT)
                    break;
                BS_TRY(result);
                dev->stats.usb_bytes += length;
                BS_TRY(bs_framer_feed(framer, bytes, length));
            } while (length);
            dev->stats.packets = framer->packets;
            return bs_framer_finish(framer, expected_packets);
        }
        if (now - last_progress > UINT64_C(3000000000))
            return BIRRIASDR_ERROR_TIMEOUT;
    }
}

/* Decide whether another complete acquisition can plausibly recover. A failed
 * control transaction may return STATE after rollback, while corrupted packets,
 * missing devices and retained vendor patches require application intervention.
 */
static bool recoverable(int error, bool control_error)
{
    return error == BIRRIASDR_ERROR_OVERRUN || error == BIRRIASDR_ERROR_IO ||
           error == BIRRIASDR_ERROR_TIMEOUT ||
           (control_error && error == BIRRIASDR_ERROR_STATE);
}

/* An overflow loses sample continuity, not necessarily the device connection.
 * Only exempt it from the fault budget after successful teardown with no DMA
 * or live-control failure. A failed reset must still spend a bounded retry.
 */
static bool clean_overrun(const birriasdr_dev_t *dev, int error)
{
    return error == BIRRIASDR_ERROR_OVERRUN && !dev->stats.reset_error &&
           !dev->stats.dma_errors && !dev->control_error && !dev->usb_lost;
}

/* Execute one acquisition without releasing ownership of the public handle.
 * Destroy the reader (including cancellation callbacks) BEFORE reset. Keep the
 * first live-control fault as the primary error if STOP subsequently fails, but
 * never mask device removal or framing corruption. The outer loop decides
 * whether to report a discontinuity and try the committed settings again.
 */
static int capture_once(struct delivery *delivery, uint8_t *bytes, uint32_t depth,
                        uint32_t packets, bool *can_retry)
{
    birriasdr_dev_t *dev = delivery->dev;
    struct bs_reader *reader = NULL;
    struct bs_framer framer = {0};
    int result = 0;
    *can_retry = false;
    memset(&dev->stats, 0, sizeof(dev->stats));
    delivery->segment_start = delivery->delivered;
    delivery->holdback = !packets && dev->recovery_limit && dev->recovery_callback;
    delivery->held_valid = false;
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    dev->control_error = 0;
    dev->control_error_stage = BIRRIASDR_STAGE_IDLE;
    pthread_mutex_unlock(&dev->control_mutex);
    if (dev->usb_lost) {
        result = BIRRIASDR_ERROR_NO_DEVICE;
        goto cleanup;
    }
    if (atomic_load(&dev->run_state) == BS_CANCELLING)
        goto cleanup;
    result = bs_reader_create(dev, depth, &reader);
    if (result < 0)
        goto cleanup;
    result = prepare_capture(dev, reader, packets);
    if (result < 0)
        goto cleanup;
    framer.nonce = dev->nonce;
    framer.payload = receive_payload;
    framer.gap = delivery->holdback ? receive_gap : NULL;
    framer.context = delivery;
    dev->stats.stage = BIRRIASDR_STAGE_STREAM;
    result = receive_loop(dev, reader, &framer, packets, bytes);
    dev->stats.packets = framer.packets;
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    pthread_mutex_unlock(&dev->control_mutex);
    /* Only a verified clean terminal state releases the final held packet. */
    if (!result && delivery->held_valid)
        result = decode_payload(delivery, delivery->held);
    delivery->held_valid = false;
    flush_samples(delivery);
cleanup:
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    if (dev->control_error && (!result || result == BIRRIASDR_ERROR_IO ||
        result == BIRRIASDR_ERROR_TIMEOUT || result == BIRRIASDR_ERROR_STATE)) {
        result = dev->control_error;
        dev->stats.stage = dev->control_error_stage;
    }
    pthread_mutex_unlock(&dev->control_mutex);
    bs_reader_destroy(reader);
    pthread_mutex_lock(&dev->control_mutex);
    if (result == BIRRIASDR_ERROR_NO_DEVICE)
        dev->usb_lost = true;
    if (dev->usb_lost)
        dev->touched = false;
    if (dev->touched) {
        dev->stats.reset_error = dev->backend->reset(dev);
        dev->touched = dev->stats.reset_error != 0;
        if (dev->stats.reset_error == BIRRIASDR_ERROR_NO_DEVICE) {
            dev->usb_lost = true;
            dev->touched = false;
        }
        if (!result)
            result = dev->stats.reset_error;
    }
    *can_retry = recoverable(result, dev->control_error != 0) &&
        (!dev->stats.reset_error || recoverable(dev->stats.reset_error, true));
    /* A drained source overflow needs a new acquisition, but a successful
     * subsystem reset need not reset USB enumeration as well. Startup still
     * escalates to USB recovery if the ordinary prepare fails. Host queue
     * overflows retain transport recovery because queued transfers were lost.
     */
    bool drained_overrun = clean_overrun(dev, result) &&
        (dev->stats.firmware_state == 8 || dev->stats.firmware_state == 9);
    if ((*can_retry && !drained_overrun) || result == BIRRIASDR_ERROR_PROTOCOL)
        dev->needs_usb_reset = true;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Publish a gap after teardown and before any new sample callback. Compare/exchange
 * changes only our internal control-failure STOP back to RUNNING; an application
 * cancellation is sticky across every recovery phase and cannot be cleared here.
 */
static bool announce_restart(struct delivery *delivery, uint32_t attempt, int error)
{
    birriasdr_dev_t *dev = delivery->dev;
    struct birriasdr_recovery_event event = {
        .next_sample = delivery->delivered, .attempt = attempt,
        .error = error, .previous = dev->stats
    };
    unsigned state = BS_CONTROL_FAILED;
    atomic_compare_exchange_strong(&dev->run_state, &state, BS_RUNNING);
    if (atomic_load(&dev->run_state) == BS_CANCELLING)
        return false;
    dev->recovery_callback(&event, dev->recovery_context);
    /* Avoid spinning on a persistent transport failure; cancellation can still
     * prevent startup immediately after this short, bounded delay. */
    if (atomic_load(&dev->run_state) != BS_CANCELLING)
        bs_sleep_us(50000);
    return atomic_load(&dev->run_state) != BS_CANCELLING;
}

/* One owner spans local source gaps and full acquisition restarts. Legacy
 * callers remain strict, and finite reads never join discontinuous segments.
 * Device markers preserve firmware, nonce and packet counters. Faults that
 * require full recovery instead boot with a fresh nonce and committed settings.
 * Delivered offsets remain cumulative across both kinds of discontinuity.
 */
int bs_stream(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
               void *context, uint32_t depth, uint32_t buffer_samples, uint32_t packets)
{
    struct delivery delivery = {0};
    uint64_t total_packets = 0, total_bytes = 0;
    uint8_t *bytes;
    uint32_t faults = 0;
    unsigned idle = BS_IDLE;
    int result = BIRRIASDR_ERROR_NO_MEMORY;
    if (!dev || !callback)
        return BIRRIASDR_ERROR_INVALID;
    if (!depth) depth = BS_DEFAULT_DEPTH;
    if (!buffer_samples) buffer_samples = 16384;
    if (depth < 4 || depth > 256 || buffer_samples > 262144 || buffer_samples % BS_PACKET_SAMPLES)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if (!atomic_compare_exchange_strong(&dev->run_state, &idle, BS_RUNNING)) {
        pthread_mutex_unlock(&dev->control_mutex);
        return BIRRIASDR_ERROR_BUSY;
    }
    dev->control_ready = false;
    dev->stream_continuous = packets == 0;
    pthread_mutex_unlock(&dev->control_mutex);
    memset(&dev->stats, 0, sizeof(dev->stats));
    delivery = (struct delivery){.dev = dev, .callback = callback, .context = context,
                                .capacity = buffer_samples};
    delivery.iq = calloc((size_t)buffer_samples * 2, sizeof(*delivery.iq));
    bytes = malloc(BS_USB_BYTES);
    if (delivery.iq && bytes) {
        for (;;) {
            bool can_retry;
            result = capture_once(&delivery, bytes, depth, packets, &can_retry);
            total_packets += dev->stats.packets;
            total_bytes += dev->stats.usb_bytes;
            bool overflow = clean_overrun(dev, result);
            if (!can_retry || packets || !dev->recovery_callback ||
                !dev->recovery_limit || (!overflow && faults == dev->recovery_limit))
                break;
            if (!overflow)
                ++faults;
            if (!announce_restart(&delivery, next_attempt(&delivery), result))
                break;
        }
    }
    /* In recovering mode an application may stop at an overflow boundary.
     * No further samples are joined, and the retained statistics still show
     * that last gap. Preserve real transport/control failures and strict reads.
     */
    if (!packets && dev->recovery_limit && dev->recovery_callback &&
        atomic_load(&dev->run_state) == BS_CANCELLING && clean_overrun(dev, result))
        result = 0;
    dev->stats.samples = delivery.delivered;
    dev->stats.packets = total_packets;
    dev->stats.usb_bytes = total_bytes;
    free(bytes);
    free(delivery.iq);
    pthread_mutex_lock(&dev->control_mutex);
    dev->stream_continuous = false;
    atomic_store(&dev->run_state, BS_IDLE);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}
