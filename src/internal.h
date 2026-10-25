/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#ifndef BIRRIASDR_INTERNAL_H
#define BIRRIASDR_INTERNAL_H

#include "birriasdr.h"
#include <libusb.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#define BS_TRY(expr) do { int bs_result_ = (expr); if (bs_result_ < 0) return bs_result_; } while (0)
#define BS_PAYLOAD_BYTES 8192u
#define BS_PACKET_SAMPLES 2048u
#define BS_FRAME_BYTES (44u + BS_PAYLOAD_BYTES + 4u)

enum bs_space { BS_MMIO, BS_UHW, BS_BRIDGE };
enum bs_run_state { BS_IDLE, BS_RUNNING, BS_CANCELLING, BS_CONTROL_FAILED };

/* Register I/O is injectable for ordered hardware-sequence and failure tests.
 * It keeps RF code independent of libusb packet details without hiding errors.
 */
struct bs_io {
    void *context;
    int (*read32)(void *, enum bs_space, uint32_t, uint32_t *);
    int (*write32)(void *, enum bs_space, uint32_t, uint32_t);
};

struct bs_backend {
    enum birriasdr_chipset chipset;
    const char *name;
    uint32_t capabilities;
    uint64_t default_frequency;
    uint64_t min_frequency, max_frequency;
    uint32_t default_rate;
    int (*validate)(uint64_t frequency, uint32_t rate, const struct birriasdr_gain *gain);
    int (*identify)(birriasdr_dev_t *);
    int (*prepare)(birriasdr_dev_t *, uint32_t packets);
    int (*start)(birriasdr_dev_t *);
    int (*stop)(birriasdr_dev_t *);
    int (*status)(birriasdr_dev_t *, uint32_t words[16]);
    int (*reset)(birriasdr_dev_t *);
    int (*change_gain)(birriasdr_dev_t *, bool manual, const struct birriasdr_gain *);
    int (*change_frequency)(birriasdr_dev_t *, uint64_t frequency);
};

struct bs_usb_id {
    uint16_t vid, pid;
    const char *name;
    const struct bs_backend *backend;
    bool qualified;
    /* Board tuning bounds; zero keeps the backend's. */
    uint64_t min_frequency, max_frequency;
};

struct birriasdr_dev {
    libusb_context *usb;
    libusb_device_handle *handle;
    int interface_number;
    int lock_fd;
    uint8_t ep_in, ep_command, ep_download;
    const struct bs_backend *backend;
    uint64_t min_frequency, max_frequency;
    struct bs_io io;
    atomic_uint run_state;
    /* Serializes whole register procedures, not individual USB transfers.
     * Application callbacks and the USB event pump never hold this mutex.
     */
    pthread_mutex_t control_mutex;
    bool control_ready;
    /* Immutable while a reader owns the handle; enables lossless USB backpressure. */
    bool stream_continuous;
    int control_error;
    enum birriasdr_stage control_error_stage;
    birriasdr_recovery_cb_t recovery_callback;
    void *recovery_context;
    uint32_t recovery_limit;
    atomic_uint_fast64_t frequency;
    uint32_t rate;
    bool manual;
    struct birriasdr_gain gain;
    uint32_t nonce;
    uint8_t command_sequence;
    bool touched;
    bool needs_usb_reset;
    bool usb_lost;
    struct {
        uint32_t gain_baseline[4];
        bool gain_saved;
        uint32_t filter_baseline[12];
        uint32_t filter_fields;
        bool filter_saved;
    } mt7921;
    struct birriasdr_stream_stats stats;
};

typedef int (*bs_payload_fn)(void *context, const uint8_t *payload);
typedef int (*bs_gap_fn)(void *context, uint32_t state, uint64_t packets);
struct bs_framer {
    uint8_t pending[BS_FRAME_BYTES];
    size_t used;
    uint32_t nonce;
    uint32_t gaps;
    uint64_t packets;
    unsigned acknowledgements;
    bs_payload_fn payload;
    bs_gap_fn gap;
    void *context;
};

extern const struct bs_backend bs_mt7921_backend;
extern const struct bs_backend bs_mt7925_backend;
const struct bs_usb_id *bs_lookup_id(uint16_t vid, uint16_t pid);
int bs_usb_discover(uint32_t index, struct birriasdr_device_info *info,
                    birriasdr_dev_t *open_device);
void bs_usb_close(birriasdr_dev_t *dev);
int bs_usb_error(int error);
int bs_control(birriasdr_dev_t *dev, uint8_t type, uint8_t request,
               uint16_t value, uint16_t index, uint8_t *data, uint16_t length);
int bs_bulk_out(birriasdr_dev_t *dev, uint8_t endpoint, const uint8_t *data, int length);
int bs_usb_prepare_endpoints(birriasdr_dev_t *dev);
int bs_usb_recover(birriasdr_dev_t *dev);
int bs_read32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t *value);
int bs_write32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t value);
int bs_update32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
                uint32_t clear, uint32_t set);
int bs_poll32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
              uint32_t mask, uint32_t expected, unsigned timeout_ms);
uint64_t bs_now_ns(void);
void bs_sleep_us(unsigned microseconds);
uint32_t bs_get_le32(const uint8_t *bytes);
void bs_put_le32(uint8_t *bytes, uint32_t value);
void bs_put_le16(uint8_t *bytes, uint16_t value);
int bs_framer_feed(struct bs_framer *framer, const uint8_t *data, size_t length);
int bs_framer_finish(const struct bs_framer *framer, uint64_t expected);
int bs_decode_iq(const uint8_t *raw, int16_t *iq, size_t samples);
int bs_decode_iq_halved(const uint8_t *raw, int16_t *iq, size_t words);

/* At 10 and 20 MSps the capture packer pads each 16-byte record with two zero
 * words, so a packet carries half as many samples.
 */
static inline uint32_t bs_packet_samples(const birriasdr_dev_t *dev)
{
    return dev->rate <= 20000000 ? BS_PACKET_SAMPLES / 2 : BS_PACKET_SAMPLES;
}
int bs_stream(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
               void *context, uint32_t depth, uint32_t buffer_samples, uint32_t packets);

#endif
