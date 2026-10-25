/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include <stdlib.h>
#include <string.h>

/* Controls without a live implementation remain idle-only. Gain and tuning
 * have serialized paths; cancellation uses the atomic acquisition state.
 */
static int require_idle(birriasdr_dev_t *dev)
{
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    return atomic_load(&dev->run_state) != BS_IDLE ? BIRRIASDR_ERROR_BUSY : 0;
}

/* Keep the ABI version independent of individual firmware image revisions. */
const char *birriasdr_get_version(void)
{
    return "0.1.0";
}

/* Errors are library-owned values, not ambiguous errno/libusb mixtures. */
const char *birriasdr_strerror(int error)
{
    switch (error) {
    case 0: return "success";
    case BIRRIASDR_ERROR_INVALID: return "invalid argument";
    case BIRRIASDR_ERROR_NO_DEVICE: return "device absent or disconnected";
    case BIRRIASDR_ERROR_ACCESS: return "USB or device-lock permission denied";
    case BIRRIASDR_ERROR_BUSY: return "device, driver, or stream is busy";
    case BIRRIASDR_ERROR_IO: return "USB/register I/O failed";
    case BIRRIASDR_ERROR_TIMEOUT: return "hardware handshake timed out";
    case BIRRIASDR_ERROR_NO_MEMORY: return "allocation failed";
    case BIRRIASDR_ERROR_UNSUPPORTED: return "chipset, board, or control not implemented";
    case BIRRIASDR_ERROR_COLD_BOOT: return "retained vendor patch: cold power-on required";
    case BIRRIASDR_ERROR_PROTOCOL: return "invalid, missing, or stale stream data";
    case BIRRIASDR_ERROR_OVERRUN: return "capture continuity lost to device or host queue overrun";
    case BIRRIASDR_ERROR_STATE: return "unexpected hardware or receiver state";
    default: return "unknown library error";
    }
}

/* Discovery uses descriptors only, so unsupported backends remain visible. */
int birriasdr_get_device_count(void)
{
    return bs_usb_discover(0, NULL, NULL);
}

/* Copy descriptive information; do not hold libusb descriptor pointers. */
int birriasdr_get_device_info(uint32_t index, struct birriasdr_device_info *info)
{
    if (!info)
        return BIRRIASDR_ERROR_INVALID;
    memset(info, 0, sizeof(*info));
    return bs_usb_discover(index, info, NULL);
}

/* Allocate one independent context, then claim a qualified interface. Booting
 * is delayed until read so setters configure the next acquisition atomically.
 */
int birriasdr_open(birriasdr_dev_t **out, uint32_t index)
{
    birriasdr_dev_t *dev;
    int result;
    if (!out)
        return BIRRIASDR_ERROR_INVALID;
    *out = NULL;
    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return BIRRIASDR_ERROR_NO_MEMORY;
    dev->lock_fd = -1;
    dev->interface_number = -1;
    atomic_init(&dev->run_state, BS_IDLE);
    atomic_init(&dev->frequency, 0);
    if (pthread_mutex_init(&dev->control_mutex, NULL)) {
        free(dev);
        return BIRRIASDR_ERROR_NO_MEMORY;
    }
    dev->gain = (struct birriasdr_gain){1, 4, 0};
    result = bs_usb_discover(index, NULL, dev);
    if (result < 0) {
        bs_usb_close(dev);
        pthread_mutex_destroy(&dev->control_mutex);
        free(dev);
        return result;
    }
    *out = dev;
    return 0;
}

/* A read owns every outstanding transfer. Requiring it to return before close
 * prevents transfer callbacks from dereferencing a freed device/context.
 */
int birriasdr_close(birriasdr_dev_t *dev)
{
    int result = 0;
    BS_TRY(require_idle(dev));
    if (dev->touched && !dev->usb_lost)
        result = dev->backend->reset(dev);
    bs_usb_close(dev);
    pthread_mutex_destroy(&dev->control_mutex);
    free(dev);
    return result;
}

/* Capabilities describe implemented operations, never silicon marketing limits. */
uint32_t birriasdr_get_capabilities(const birriasdr_dev_t *dev)
{
    return dev ? dev->backend->capabilities : 0;
}

/* With the control mutex held, turn a failed hardware update into an orderly
 * stop. Preserve its stage independently of later status/reset procedures.
 */
static void control_failed(birriasdr_dev_t *dev, int error, enum birriasdr_stage stage)
{
    unsigned expected = BS_RUNNING;
    dev->control_ready = false;
    dev->control_error = error;
    dev->control_error_stage = stage;
    /* Keep an application cancellation distinct from our internal STOP. */
    atomic_compare_exchange_strong(&dev->run_state, &expected, BS_CONTROL_FAILED);
}

/* Report backend command limits rather than inferring them from a Wi-Fi channel
 * table, narrowed by any measured board bounds. The bounds describe programming
 * support, not measured sensitivity.
 */
int birriasdr_get_tuning_range(const birriasdr_dev_t *dev, uint64_t *minimum, uint64_t *maximum)
{
    if (!dev || !minimum || !maximum)
        return BIRRIASDR_ERROR_INVALID;
    *minimum = dev->min_frequency ? dev->min_frequency : dev->backend->min_frequency;
    *maximum = dev->max_frequency ? dev->max_frequency : dev->backend->max_frequency;
    return *minimum && *maximum >= *minimum ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* Stage a center while idle, or synchronously retune under the same mutex as
 * gain/start/stop/reset. The backend calibrates its synthesizer; it never rounds
 * to a Wi-Fi channel or silently translates this into host-side mixing.
 */
int birriasdr_set_center_freq(birriasdr_dev_t *dev, uint64_t frequency)
{
    unsigned state;
    int result = 0;
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready)) {
        result = BIRRIASDR_ERROR_BUSY;
        goto done;
    }
    result = dev->backend->validate(frequency, dev->rate, &dev->gain);
    if (result < 0)
        goto done;
    if ((dev->min_frequency && frequency < dev->min_frequency) ||
        (dev->max_frequency && frequency > dev->max_frequency)) {
        result = BIRRIASDR_ERROR_UNSUPPORTED;
        goto done;
    }
    if (state == BS_RUNNING) {
        if (!(dev->backend->capabilities & BIRRIASDR_CAP_LIVE_TUNING) ||
            !dev->backend->change_frequency) {
            result = BIRRIASDR_ERROR_UNSUPPORTED;
            goto done;
        }
        result = dev->backend->change_frequency(dev, frequency);
        if (result < 0) {
            control_failed(dev, result, BIRRIASDR_STAGE_RF);
            goto done;
        }
    }
    atomic_store(&dev->frequency, frequency);
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read committed metadata atomically, including from sample callbacks while
 * another thread retunes. It does not label the frequency of queued samples.
 */
uint64_t birriasdr_get_center_freq(const birriasdr_dev_t *dev)
{
    return dev ? atomic_load(&dev->frequency) : 0;
}

/* The backend validates the rate and programs the matching capture tap and
 * filter profile at preparation. No host decimator is hidden here.
 */
int birriasdr_set_sample_rate(birriasdr_dev_t *dev, uint32_t rate)
{
    BS_TRY(require_idle(dev));
    BS_TRY(dev->backend->validate(dev->frequency, rate, &dev->gain));
    dev->rate = rate;
    return 0;
}

/* The nominal complex rate is configuration metadata, not measured timing. */
uint32_t birriasdr_get_sample_rate(const birriasdr_dev_t *dev)
{
    return dev ? dev->rate : 0;
}

/* The caller holds control_mutex across validation, the complete hardware
 * transaction and the settings commit. Startup/stop/status/reset use the same
 * mutex, so even an RF-SPI read/modify/write cannot straddle a device reset.
 * A failed live update stops this acquisition. Its error reaches either the
 * recovery callback or, if no restart succeeds, the read caller.
 */
static int set_gain_locked(birriasdr_dev_t *dev, bool manual,
                           const struct birriasdr_gain *gain)
{
    unsigned state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready))
        return BIRRIASDR_ERROR_BUSY;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_RAW_GAIN))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(dev->backend->validate(dev->frequency, dev->rate, gain));
    if (state == BS_RUNNING) {
        int result;
        if (!(dev->backend->capabilities & BIRRIASDR_CAP_LIVE_GAIN) ||
            !dev->backend->change_gain)
            return BIRRIASDR_ERROR_UNSUPPORTED;
        result = dev->backend->change_gain(dev, manual, gain);
        if (result < 0) {
            control_failed(dev, result, BIRRIASDR_STAGE_GAIN);
            return result;
        }
    }
    dev->manual = manual;
    dev->gain = *gain;
    return 0;
}

/* Stage mode while idle; during capture, synchronously apply the known force
 * fields. Released mode is not a claim about all internal AGC loops. A gain
 * acknowledgment means readback passed, not that every queued sample is newer.
 */
int birriasdr_set_tuner_gain_mode(birriasdr_dev_t *dev, int manual)
{
    int result;
    if (!dev || (manual != 0 && manual != 1))
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    result = set_gain_locked(dev, manual != 0, &dev->gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Reject invalid indices before hardware writes. A live request shares the
 * capture's control lock and retains the original boot baseline for cleanup.
 */
int birriasdr_set_gain_codes(birriasdr_dev_t *dev, const struct birriasdr_gain *gain)
{
    int result;
    if (!dev || !gain)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if (atomic_load(&dev->run_state) != BS_IDLE &&
        (atomic_load(&dev->run_state) != BS_RUNNING || !dev->control_ready))
        result = BIRRIASDR_ERROR_BUSY;
    else if (!dev->manual)
        result = BIRRIASDR_ERROR_STATE;
    else
        result = set_gain_locked(dev, true, gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read settings under the same lock used for live commits. This is not an
 * instantaneous AGC measurement, and performs no RF-SPI transaction.
 */
int birriasdr_get_gain_codes(birriasdr_dev_t *dev, struct birriasdr_gain *gain, int *manual)
{
    if (!dev || !gain || !manual)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    *gain = dev->gain;
    *manual = dev->manual;
    pthread_mutex_unlock(&dev->control_mutex);
    return 0;
}

/* Missing research must be observable to callers, not a successful no-op. */
static int unsupported_control(birriasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    return BIRRIASDR_ERROR_UNSUPPORTED;
}

/* TODO: measure and implement raw-code to calibrated RF dB conversion. */
int birriasdr_set_tuner_gain(birriasdr_dev_t *dev, int tenths_db)
{
    (void)tenths_db;
    return unsupported_control(dev);
}

/* TODO: publish measured stage combinations rather than inventing gain values. */
int birriasdr_get_tuner_gains(birriasdr_dev_t *dev, int *gains, size_t capacity)
{
    (void)gains;
    (void)capacity;
    return unsupported_control(dev);
}

/* TODO: identify every relevant AGC loop and validate a complete freeze/release. */
int birriasdr_set_agc_mode(birriasdr_dev_t *dev, int enabled)
{
    (void)enabled;
    return unsupported_control(dev);
}

/* TODO: independently selectable filter widths and calibrated RF response.
 * The current backend applies its fixed matched profile during preparation.
 */
int birriasdr_set_tuner_bandwidth(birriasdr_dev_t *dev, uint32_t bandwidth)
{
    (void)bandwidth;
    return unsupported_control(dev);
}

/* TODO: establish crystal/PLL correction without changing sample interpretation. */
int birriasdr_set_freq_correction(birriasdr_dev_t *dev, int ppm)
{
    (void)ppm;
    return unsupported_control(dev);
}

/* TODO: decode board routing; the ICAP chain selector is a different control. */
int birriasdr_set_antenna(birriasdr_dev_t *dev, unsigned antenna)
{
    (void)antenna;
    return unsupported_control(dev);
}

/* TODO: identify writable coefficient fields and their ordering/units. This API
 * reserves volatile application only; it must never become an eFuse writer.
 */
int birriasdr_set_calibration(birriasdr_dev_t *dev, const void *data, size_t length)
{
    (void)data;
    (void)length;
    return unsupported_control(dev);
}

/* There is no retained host queue between reads. Clear diagnostics only. */
int birriasdr_reset_buffer(birriasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    memset(&dev->stats, 0, sizeof(dev->stats));
    return 0;
}

/* Recovery requires a gap consumer and a finite per-read budget. Configuration
 * is idle-only; a read owns its callback/context for its entire lifetime.
 */
int birriasdr_set_recovery(birriasdr_dev_t *dev, uint32_t limit,
                          birriasdr_recovery_cb_t callback, void *context)
{
    BS_TRY(require_idle(dev));
    if (limit && !callback)
        return BIRRIASDR_ERROR_INVALID;
    dev->recovery_limit = limit;
    dev->recovery_callback = limit ? callback : NULL;
    dev->recovery_context = limit ? context : NULL;
    return 0;
}

/* Cancellation and running/idle share one atomic state, so startup cannot erase
 * a concurrent cancellation and idle cancellation cannot poison the next read.
 * The read thread owns STOP/drain; other live controls share its register lock.
 */
int birriasdr_cancel_async(birriasdr_dev_t *dev)
{
    unsigned expected;
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    expected = atomic_load(&dev->run_state);
    while (expected == BS_RUNNING || expected == BS_CONTROL_FAILED)
        if (atomic_compare_exchange_weak(&dev->run_state, &expected, BS_CANCELLING))
            return 0;
    return expected == BS_CANCELLING ? 0 : BIRRIASDR_ERROR_STATE;
}

/* Statistics are stable once the read thread has returned. */
int birriasdr_get_stream_stats(birriasdr_dev_t *dev, struct birriasdr_stream_stats *stats)
{
    BS_TRY(require_idle(dev));
    if (!stats)
        return BIRRIASDR_ERROR_INVALID;
    *stats = dev->stats;
    return 0;
}

/* The shared read engine owns startup, asynchronous transfers and cleanup. */
int birriasdr_read_async(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
                        void *context, uint32_t depth, uint32_t buffer_samples)
{
    return bs_stream(dev, callback, context, depth, buffer_samples, 0);
}

struct sync_buffer { int16_t *iq; uint32_t count; uint32_t received; };

/* The finite firmware packet count bounds this copy; keep a defensive bound
 * so malformed backend output cannot overflow the caller's allocation.
 */
static void copy_sync(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct sync_buffer *buffer = context;
    (void)first;
    if (count > buffer->count - buffer->received)
        count = buffer->count - buffer->received;
    memcpy(buffer->iq + 2u * (size_t)buffer->received, iq, 4u * (size_t)count);
    buffer->received += count;
}

/* Finite reads use the same queued USB engine and packet validation as async.
 * Requiring whole payloads avoids discarding unseen samples at the last packet.
 */
int birriasdr_read_sync(birriasdr_dev_t *dev, int16_t *iq, uint32_t count, uint32_t *n_read)
{
    struct sync_buffer buffer = {iq, count, 0};
    int result;
    if (n_read)
        *n_read = 0;
    if (!dev || !iq || !n_read || !count || count % BS_PACKET_SAMPLES)
        return BIRRIASDR_ERROR_INVALID;
    result = bs_stream(dev, copy_sync, &buffer, 64, 16384, count / BS_PACKET_SAMPLES);
    *n_read = buffer.received;
    if (!result && buffer.received != count)
        return BIRRIASDR_ERROR_PROTOCOL;
    return result;
}
