/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#ifndef BIRRIASDR_H
#define BIRRIASDR_H

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#pragma GCC visibility push(default)
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Opaque receiver. Each handle owns its USB context and one Wi-Fi interface. */
typedef struct birriasdr_dev birriasdr_dev_t;

enum birriasdr_error {
    BIRRIASDR_OK = 0,
    BIRRIASDR_ERROR_INVALID = -1,
    BIRRIASDR_ERROR_NO_DEVICE = -2,
    BIRRIASDR_ERROR_ACCESS = -3,
    BIRRIASDR_ERROR_BUSY = -4,
    BIRRIASDR_ERROR_IO = -5,
    BIRRIASDR_ERROR_TIMEOUT = -6,
    BIRRIASDR_ERROR_NO_MEMORY = -7,
    BIRRIASDR_ERROR_UNSUPPORTED = -8,
    BIRRIASDR_ERROR_COLD_BOOT = -9,
    BIRRIASDR_ERROR_PROTOCOL = -10,
    BIRRIASDR_ERROR_OVERRUN = -11,
    BIRRIASDR_ERROR_STATE = -12
};

enum birriasdr_chipset {
    BIRRIASDR_CHIP_UNKNOWN,
    BIRRIASDR_CHIP_MT7921,
    BIRRIASDR_CHIP_MT7925
};

enum birriasdr_capability {
    BIRRIASDR_CAP_RX = 1u << 0,
    BIRRIASDR_CAP_RAW_GAIN = 1u << 1,
    BIRRIASDR_CAP_GAIN_DB = 1u << 2,
    BIRRIASDR_CAP_AGC = 1u << 3,
    BIRRIASDR_CAP_FILTER = 1u << 4,
    BIRRIASDR_CAP_ANTENNA = 1u << 5,
    BIRRIASDR_CAP_CALIBRATION = 1u << 6,
    BIRRIASDR_CAP_LIVE_GAIN = 1u << 7,
    BIRRIASDR_CAP_TUNING = 1u << 8,
    BIRRIASDR_CAP_LIVE_TUNING = 1u << 9
};

/** Discovery includes recognized but unimplemented devices (capabilities=0).
 * Indices are enumeration snapshots: unplugging a device can change them.
 * No firmware is loaded and no interface is claimed during enumeration.
 */
struct birriasdr_device_info {
    uint16_t vid, pid;
    uint8_t bus, address;
    enum birriasdr_chipset chipset;
    uint32_t capabilities;
    char name[96];
};

/** Raw hardware indices, deliberately distinct from calibrated gain in dB. */
struct birriasdr_gain {
    uint8_t lna;       /**< Tested MT7972 range 0..3. */
    uint8_t lpf;       /**< Filter-stage gain 0..8; does not select bandwidth. */
    uint8_t digital;   /**< Digital gain code 0..12. */
};

/** Last hardware operation reached; useful when a read fails before samples arrive. */
enum birriasdr_stage {
    BIRRIASDR_STAGE_IDLE,
    BIRRIASDR_STAGE_RESET,
    BIRRIASDR_STAGE_ROM,
    BIRRIASDR_STAGE_DOWNLOAD,
    BIRRIASDR_STAGE_CAPTURE,
    BIRRIASDR_STAGE_RF,
    BIRRIASDR_STAGE_GAIN,
    BIRRIASDR_STAGE_STREAM,
    BIRRIASDR_STAGE_VERIFY
};

/** Summary of the most recent read, available after read_async/read_sync returns.
 * A nonzero read result invalidates continuity of that capture, even if some
 * samples were already delivered. firmware_state 9 means the ICAP ring overran.
 * An automatically recovered read can return zero despite notified gaps.
 * Samples/packets/bytes accumulate across attempts; the remaining fields describe
 * the final attempt. Each recovery event preserves the preceding attempt's stats.
 */
struct birriasdr_stream_stats {
    uint64_t samples;
    uint64_t packets;
    uint64_t usb_bytes;
    uint32_t firmware_state;
    uint32_t dma_errors;
    enum birriasdr_stage stage;
    int reset_error;
};

/** Samples are interleaved native-endian int16_t I,Q in [-8192,8191].
 * count and first_sample count COMPLEX samples, not scalar components/bytes.
 * The buffer is borrowed until callback return; copy it to retain it. There
 * is no averaging, decimation, left shifting, or floating-point conversion.
 * Run callbacks promptly: a stalled consumer can overrun the device's SRAM.
 * The center-frequency setter/getter, tuning-range getter, raw-gain mode/code
 * setters, gain-code getter and cancel_async may be called from the callback;
 * other operations must wait until the read returns.
 * Prefer a control thread for updates so callbacks keep draining data.
 */
typedef void (*birriasdr_read_async_cb_t)(const int16_t *iq, uint32_t count,
                                        uint64_t first_sample, void *context);

/** Notification before an automatic restart. The failed attempt has ended and
 * its transfers have been drained. New samples, if startup succeeds, follow
 * this event and are NOT continuous with the preceding samples. The number of
 * missing samples is unknown; no zeros or invented timestamps are inserted.
 */
struct birriasdr_recovery_event {
    uint64_t next_sample; /**< Total samples delivered in this read; not elapsed RF time. */
    uint32_t attempt;     /**< Restart number, starting at one for each read_async call. */
    int error;           /**< Error that caused this restart. */
    struct birriasdr_stream_stats previous; /**< Statistics of the failed attempt only. */
};

/** Called on the read thread, with no library lock held and no samples in flight
 * to application callbacks. May call cancel_async or getters allowed during a
 * read. Setters return BUSY until the next acquisition is ready. A notification
 * announces an attempt, not successful recovery; read_async reports final failure.
 */
typedef void (*birriasdr_recovery_cb_t)(const struct birriasdr_recovery_event *event,
                                      void *context);

/** Return a static library version string; never NULL. */
const char *birriasdr_get_version(void);

/** Return a static explanation for a library error; never NULL. */
const char *birriasdr_strerror(int error);

/** Return the number of recognized USB devices, or a negative error. */
int birriasdr_get_device_count(void);

/** Describe a recognized device without claiming it. Returns 0 or an error. */
int birriasdr_get_device_info(uint32_t index, struct birriasdr_device_info *info);

/** Claim a supported Wi-Fi USB interface without loading firmware yet.
 * The interface must be free of a kernel Wi-Fi driver. The library never
 * unloads drivers, power-cycles a hub, downloads vendor firmware, or writes
 * nonvolatile memory. Startup may reset USB to recover a previous failure;
 * a reset that invalidates the handle returns NO_DEVICE. A retained vendor
 * patch is rejected when reading.
 * On failure *dev is NULL. Currently only COMFAST 3574:6211 is qualified.
 */
int birriasdr_open(birriasdr_dev_t **dev, uint32_t index);

/** Release the handle. Returns BUSY during a read; cancel and join it first.
 * The Wi-Fi kernel driver is not rebound (that could load vendor firmware).
 */
int birriasdr_close(birriasdr_dev_t *dev);

/** Return this backend's implemented capability bits, or 0 for NULL. */
uint32_t birriasdr_get_capabilities(const birriasdr_dev_t *dev);

/** Return inclusive command limits in Hz. Returns UNSUPPORTED if unknown.
 * These are programming limits, not guaranteed RF sensitivity or passband
 * coverage. Safe during a read.
 */
int birriasdr_get_tuning_range(const birriasdr_dev_t *dev, uint64_t *minimum_hz,
                              uint64_t *maximum_hz);

/** Set a requested center in Hz, without channel-number rounding or host mixing.
 * The MT7921 range is 2156000000..2610000000 Hz, inclusive, with nominal
 * 28.6102294921875 Hz hardware steps; frequency accuracy is uncalibrated.
 * While idle, stage the center for the next read. With CAP_LIVE_TUNING, apply it
 * during capture without restarting ICAP/DMA or downloading firmware. Calls
 * serialize with gain and lifecycle controls; startup/shutdown may return BUSY.
 * During capture, success means calibration completed, known diagnostic rails
 * were absent and frequency readback passed. It does not prove PLL lock or settling.
 * Samples during a retune are disturbed; phase is not continuous. Queued data
 * can still belong to the old center after return. No exact sample boundary is
 * reported. A failed hardware retune attempts rollback and stops the acquisition;
 * enabled recovery can then restart with the preceding committed frequency.
 * An out-of-range request returns UNSUPPORTED without disrupting a live stream.
 */
int birriasdr_set_center_freq(birriasdr_dev_t *dev, uint64_t frequency_hz);

/** Return the last committed requested center in Hz, including during capture;
 * not a calibrated RF measurement or the center of buffered samples. Zero if NULL.
 */
uint64_t birriasdr_get_center_freq(const birriasdr_dev_t *dev);

/** Stage complex samples/second. MT7921 accepts 40000000 only at present.
 * The sampled span is 40 MHz, using matched nominal 40 MHz analog/digital
 * settings. This is not a calibrated passband or an alias-rejection guarantee.
 */
int birriasdr_set_sample_rate(birriasdr_dev_t *dev, uint32_t samples_per_second);

/** Return the nominal configured complex sample rate, or 0 for NULL. */
uint32_t birriasdr_get_sample_rate(const birriasdr_dev_t *dev);

/** Set gain mode: 1 forces the documented gain indices, 0 releases overrides.
 * Releasing overrides is not a claim of complete closed-loop AGC control.
 * While idle, settings apply at the next read. With CAP_LIVE_GAIN, a running
 * capture changes registers synchronously without rebooting or stopping ICAP.
 * Calls serialize with startup/cleanup; BUSY means no live control window is
 * available. Backends without live support return UNSUPPORTED during capture.
 * The update can take milliseconds and spans multiple register writes. Success
 * confirms readback, not an exact sample boundary: buffered samples may precede
 * the change and samples during the transition are not gain-calibrated.
 * A hardware update failure stops that acquisition and leaves cleanup to the
 * reader, which may automatically restart if set_recovery was enabled.
 */
int birriasdr_set_tuner_gain_mode(birriasdr_dev_t *dev, int manual);

/** Set manual LNA/LPF/digital indices. Requires manual mode. The same idle/live
 * semantics as set_tuner_gain_mode apply. The backend preserves unrelated fields,
 * verifies readbacks and restores the preceding settings if an update fails.
 */
int birriasdr_set_gain_codes(birriasdr_dev_t *dev, const struct birriasdr_gain *gain);

/** Read committed gain codes and mode, including during capture. These are
 * requested values while idle, not an instantaneous AGC or RF gain readback.
 */
int birriasdr_get_gain_codes(birriasdr_dev_t *dev, struct birriasdr_gain *gain,
                            int *manual);

/** Reserved: calibrated tuner gain in tenths of a dB. UNSUPPORTED until the
 * measured stage-gain mapping is known; raw indices must not be labeled dB.
 */
int birriasdr_set_tuner_gain(birriasdr_dev_t *dev, int tenths_db);

/** Reserved: enumerate calibrated gains. UNSUPPORTED; does not fill gains. */
int birriasdr_get_tuner_gains(birriasdr_dev_t *dev, int *gains, size_t capacity);

/** Reserved: global AGC enable/freeze. UNSUPPORTED because only individual
 * force fields are established; use tuner_gain_mode for those overrides.
 */
int birriasdr_set_agc_mode(birriasdr_dev_t *dev, int enabled);

/** Reserved: independently selectable RF filter bandwidth in Hz. The backend
 * applies its fixed nominal 40 MHz profile automatically. This setter remains
 * UNSUPPORTED until additional widths and their RF response are qualified.
 */
int birriasdr_set_tuner_bandwidth(birriasdr_dev_t *dev, uint32_t bandwidth_hz);

/** Reserved: oscillator correction in ppm. UNSUPPORTED; no software DDC added. */
int birriasdr_set_freq_correction(birriasdr_dev_t *dev, int ppm);

/** Reserved: physical antenna routing. UNSUPPORTED; chain != antenna switch. */
int birriasdr_set_antenna(birriasdr_dev_t *dev, unsigned antenna);

/** Reserved: apply volatile calibration coefficients. UNSUPPORTED until their
 * format, routing and calibration procedure are decoded. Never programs eFuse.
 */
int birriasdr_set_calibration(birriasdr_dev_t *dev, const void *data, size_t length);

/** Clear host statistics while idle. Each read starts a fresh hardware capture;
 * this operation does not pretend to flush an active ICAP stream.
 */
int birriasdr_reset_buffer(birriasdr_dev_t *dev);

/** Enable automatic recovery for read_async; configure only while idle.
 * max_restarts is the total restart budget per read call; nonzero requires a
 * callback so discontinuities cannot be hidden. Zero disables recovery (default).
 * Overruns, transient transport errors and failed live configuration updates
 * restart with the last committed settings. Explicit cancellation always wins.
 * Device removal, permissions, retained vendor patches and invalid/unsupported
 * requests are not retried. Exhaustion returns the last capture error.
 * Finite read_sync calls remain strict and never splice separate acquisitions.
 */
int birriasdr_set_recovery(birriasdr_dev_t *dev, uint32_t max_restarts,
                          birriasdr_recovery_cb_t callback, void *context);

/** Capture an exact finite number of complex samples into I,Q int16_t pairs.
 * count must be a positive multiple of 2048, with room for 2*count int16_t's.
 * *n_read is initialized to zero and reports any prefix even on error. Samples
 * are continuous only within one successful call; consecutive calls restart
 * the acquisition program and must not be concatenated as a continuous stream.
 */
int birriasdr_read_sync(birriasdr_dev_t *dev, int16_t *iq, uint32_t count,
                       uint32_t *n_read);

/** Stream until cancel_async, device loss, or an unrecovered hardware/protocol error.
 * Like librtlsdr, this blocks the calling thread while asynchronous USB reads
 * run. transfer_count=0 selects 64 queued transfers, or 256 on macOS.
 * Allowed values are 4..256.
 * buffer_samples=0 selects 16384; otherwise use a multiple of 2048, <=262144.
 * A final callback may be shorter. Cancellation drains submitted device
 * packets before returning; callbacks can still arrive after cancellation.
 * Returns 0 for a clean cancel, otherwise a negative error. Recovery is opt-in
 * through set_recovery and always reports a gap before another acquisition.
 * first_sample remains a cumulative delivered count across those gaps, not an
 * RF timestamp. Final statistics accumulate samples/packets/bytes over attempts;
 * firmware state, stage, DMA and reset errors describe the final attempt.
 * Without recovery enabled, no samples are silently skipped; check the return
 * value before treating a capture as valid.
 */
int birriasdr_read_async(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
                        void *context, uint32_t transfer_count,
                        uint32_t buffer_samples);

/** Request cancellation concurrently with read_async/read_sync or its callback.
 * Live gain/tuning operations are also allowed as documented above. Not async-signal-safe:
 * signal handlers should set their own flag, then cancel from a normal thread.
 * Repeated cancellation is harmless; returns STATE if no read is active.
 */
int birriasdr_cancel_async(birriasdr_dev_t *dev);

/** Copy final statistics while idle; returns BUSY until the read has returned. */
int birriasdr_get_stream_stats(birriasdr_dev_t *dev,
                             struct birriasdr_stream_stats *stats);

/* All functions return 0 on success unless their comments say otherwise.
 * The center-frequency setter/getter, tuning-range getter, raw-gain mode/code
 * setters, gain-code getter and cancel_async may run concurrently with a read
 * and with each other. Callers must serialize all other operations on a handle,
 * including close after joining BOTH the reader and any control threads.
 * Different handles may be used independently from different threads.
 */
#ifdef __cplusplus
}
#endif
#if defined(__GNUC__)
#pragma GCC visibility pop
#endif
#endif
