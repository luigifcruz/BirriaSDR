/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "usb_stream.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct transfer_slot {
    struct bs_reader *reader;
    struct libusb_transfer *transfer;
    bool pending;
};

struct bs_reader {
    birriasdr_dev_t *dev;
    pthread_mutex_t mutex;
    pthread_cond_t ready;
    pthread_t thread;
    bool started, stopping;
    int error;
    unsigned depth, inflight, capacity, head, tail, count;
    struct transfer_slot *slots;
    uint8_t *queue;
    unsigned *lengths;
};

/* Cancel once with the queue lock held. Transfer memory remains alive until
 * cancellation callbacks have retired every submitted request.
 */
static void cancel_transfers(struct bs_reader *reader)
{
    if (reader->stopping)
        return;
    reader->stopping = true;
    for (unsigned i = 0; i < reader->depth; ++i)
        if (reader->slots[i].pending)
            libusb_cancel_transfer(reader->slots[i].transfer);
}

/* Completion callbacks only copy USB bytes into a bounded queue and re-submit.
 * RF control and application callbacks run outside libusb callbacks instead,
 * avoiding libusb event-lock recursion and preventing Python-style scheduling
 * gaps between individual USB reads. A full queue is an explicit error.
 */
static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
    struct transfer_slot *slot = transfer->user_data;
    struct bs_reader *reader = slot->reader;
    pthread_mutex_lock(&reader->mutex);
    slot->pending = false;
    --reader->inflight;
    if (!reader->stopping && !reader->error) {
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            reader->error = transfer->status == LIBUSB_TRANSFER_NO_DEVICE ?
                BIRRIASDR_ERROR_NO_DEVICE : BIRRIASDR_ERROR_IO;
        } else if (transfer->actual_length > 0) {
            if (reader->count == reader->capacity) {
                reader->error = BIRRIASDR_ERROR_OVERRUN;
            } else {
                size_t length = (size_t)transfer->actual_length;
                memcpy(reader->queue + (size_t)reader->tail * BS_USB_BYTES, transfer->buffer, length);
                reader->lengths[reader->tail] = (unsigned)length;
                reader->tail = (reader->tail + 1) % reader->capacity;
                ++reader->count;
            }
        }
        if (!reader->error) {
            int result = libusb_submit_transfer(transfer);
            if (result < 0)
                reader->error = bs_usb_error(result);
            else {
                slot->pending = true;
                ++reader->inflight;
            }
        }
    }
    pthread_cond_broadcast(&reader->ready);
    pthread_mutex_unlock(&reader->mutex);
}

/* One event thread belongs to one handle/context. It keeps processing events
 * during shutdown until cancellation callbacks finish; freeing early is unsafe.
 */
static void *pump_events(void *context)
{
    struct bs_reader *reader = context;
    for (;;) {
        bool done;
        int result;
        struct timeval timeout = {0, 10000};
        pthread_mutex_lock(&reader->mutex);
        if (reader->error)
            cancel_transfers(reader);
        done = reader->stopping && reader->inflight == 0;
        pthread_mutex_unlock(&reader->mutex);
        if (done)
            return NULL;
        result = libusb_handle_events_timeout_completed(reader->dev->usb, &timeout, NULL);
        if (result < 0 && result != LIBUSB_ERROR_INTERRUPTED) {
            pthread_mutex_lock(&reader->mutex);
            if (!reader->error)
                reader->error = bs_usb_error(result);
            pthread_cond_broadcast(&reader->ready);
            pthread_mutex_unlock(&reader->mutex);
        }
    }
}

/* Allocate and touch every queue page before starting RF. A 512-slot queue is
 * 32 MiB; its bound prevents unbounded memory growth behind a slow callback.
 */
int bs_reader_create(birriasdr_dev_t *dev, unsigned depth, struct bs_reader **out)
{
    struct bs_reader *reader = calloc(1, sizeof(*reader));
    *out = NULL;
    if (!reader)
        return BIRRIASDR_ERROR_NO_MEMORY;
    reader->dev = dev;
    reader->depth = depth;
    reader->capacity = depth < 64 ? depth * 8 : 512;
    if (pthread_mutex_init(&reader->mutex, NULL)) {
        free(reader);
        return BIRRIASDR_ERROR_IO;
    }
    if (pthread_cond_init(&reader->ready, NULL)) {
        pthread_mutex_destroy(&reader->mutex);
        free(reader);
        return BIRRIASDR_ERROR_IO;
    }
    reader->slots = calloc(depth, sizeof(*reader->slots));
    reader->queue = malloc((size_t)reader->capacity * BS_USB_BYTES);
    reader->lengths = calloc(reader->capacity, sizeof(*reader->lengths));
    if (!reader->slots || !reader->queue || !reader->lengths)
        goto allocation_failed;
    memset(reader->queue, 0, (size_t)reader->capacity * BS_USB_BYTES);
    for (unsigned i = 0; i < depth; ++i) {
        struct transfer_slot *slot = &reader->slots[i];
        uint8_t *buffer;
        slot->reader = reader;
        slot->transfer = libusb_alloc_transfer(0);
        if (!slot->transfer)
            goto allocation_failed;
        buffer = malloc(BS_USB_BYTES);
        if (!buffer)
            goto allocation_failed;
        memset(buffer, 0, BS_USB_BYTES);
        libusb_fill_bulk_transfer(slot->transfer, dev->handle, dev->ep_in, buffer,
                                  BS_USB_BYTES, receive_transfer, slot, 0);
    }
    *out = reader;
    return 0;
allocation_failed:
    bs_reader_destroy(reader);
    return BIRRIASDR_ERROR_NO_MEMORY;
}

/* Queue all transfers before the MCU GO write. On partial submission failure,
 * still run/drain the event pump so libusb retains no pointers into freed memory.
 */
int bs_reader_start(struct bs_reader *reader)
{
    int result = 0;
    for (unsigned i = 0; i < reader->depth; ++i) {
        result = libusb_submit_transfer(reader->slots[i].transfer);
        if (result < 0)
            break;
        reader->slots[i].pending = true;
        ++reader->inflight;
    }
    if (result < 0)
        reader->error = bs_usb_error(result);
    if (pthread_create(&reader->thread, NULL, pump_events, reader)) {
        reader->error = BIRRIASDR_ERROR_IO;
        cancel_transfers(reader);
        pump_events(reader);
        return reader->error;
    }
    reader->started = true;
    return result < 0 ? bs_usb_error(result) : 0;
}

/* Copy one completed transfer while holding the queue lock. The consumer gets
 * independent storage so USB callbacks can immediately reuse their queue slots.
 * CLOCK_REALTIME matches POSIX's default condition clock, including on macOS.
 */
int bs_reader_take(struct bs_reader *reader, uint8_t *data, size_t *length, unsigned timeout_ms)
{
    struct timespec deadline;
    int result = 0;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += (long)timeout_ms * 1000000;
    deadline.tv_sec += deadline.tv_nsec / 1000000000;
    deadline.tv_nsec %= 1000000000;
    *length = 0;
    pthread_mutex_lock(&reader->mutex);
    while (!reader->count && !reader->error) {
        int wait = pthread_cond_timedwait(&reader->ready, &reader->mutex, &deadline);
        if (wait == ETIMEDOUT) {
            result = BIRRIASDR_ERROR_TIMEOUT;
            break;
        }
        if (wait) {
            result = BIRRIASDR_ERROR_IO;
            break;
        }
    }
    if (reader->error)
        result = reader->error;
    if (!result && reader->count) {
        *length = reader->lengths[reader->head];
        memcpy(data, reader->queue + (size_t)reader->head * BS_USB_BYTES, *length);
        reader->head = (reader->head + 1) % reader->capacity;
        --reader->count;
    }
    pthread_mutex_unlock(&reader->mutex);
    return result;
}

/* Drain cancellations, join, then free. Also handles pre-start allocation errors. */
void bs_reader_destroy(struct bs_reader *reader)
{
    if (!reader)
        return;
    if (reader->started) {
        pthread_mutex_lock(&reader->mutex);
        cancel_transfers(reader);
        pthread_mutex_unlock(&reader->mutex);
        pthread_join(reader->thread, NULL);
    }
    if (reader->slots) {
        for (unsigned i = 0; i < reader->depth; ++i) {
            struct libusb_transfer *transfer = reader->slots[i].transfer;
            if (transfer) {
                free(transfer->buffer);
                libusb_free_transfer(transfer);
            }
        }
    }
    free(reader->slots);
    free(reader->queue);
    free(reader->lengths);
    pthread_cond_destroy(&reader->ready);
    pthread_mutex_destroy(&reader->mutex);
    free(reader);
}
