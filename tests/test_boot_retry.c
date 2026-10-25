/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/* Exercise the actual boot retry policy with modeled register failures. Every
 * transfer or reset is mocked or aborts; no discovery or hardware I/O occurs.
 * Rejecting the first reset register separates whole-attempt retries from a
 * replay of the failed access. Fatal errors and explicit cancellation must not
 * trigger another attempt, even if a previous acquisition requested repair.
 */
#include "chips/mt7921.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(t) do { if (!(t)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #t); exit(1); } } while (0)
static struct {
    unsigned reads, resets;
    int first_error, later_error, reset_error;
    bool cancel;
} model;
const struct bs_backend bs_mt7921_backend = {0}, bs_mt7925_backend = {0};

/* No baseline gain exists in this model; later RF initialization is unreachable. */
int mt_restore_gain(birriasdr_dev_t *dev) { (void)dev; return 0; }
int mt_restore_filter(birriasdr_dev_t *dev) { (void)dev; return 0; }
int mt_restore_band(birriasdr_dev_t *dev) { (void)dev; return 0; }
int mt_configure_filter(birriasdr_dev_t *dev) { (void)dev; abort(); }
int mt_configure_capture(birriasdr_dev_t *dev) { (void)dev; abort(); }
int mt_configure_rf(birriasdr_dev_t *dev) { (void)dev; abort(); }
int mt_apply_gain(birriasdr_dev_t *dev) { (void)dev; abort(); }

/* Count software-reset requests while supplying the selected platform outcome. */
int LIBUSB_CALL libusb_reset_device(libusb_device_handle *handle)
{
    (void)handle;
    ++model.resets;
    return model.reset_error;
}

/* Register access uses the injected transport below. Reaching real USB is a bug. */
int LIBUSB_CALL libusb_control_transfer(libusb_device_handle *handle, uint8_t type,
    uint8_t request, uint16_t value, uint16_t index, unsigned char *data,
    uint16_t length, unsigned timeout)
{
    (void)handle; (void)type; (void)request; (void)value; (void)index;
    (void)data; (void)length; (void)timeout;
    abort();
}

int LIBUSB_CALL libusb_bulk_transfer(libusb_device_handle *handle, unsigned char endpoint,
    unsigned char *data, int size, int *actual, unsigned timeout)
{
    (void)handle; (void)endpoint; (void)data; (void)size; (void)actual; (void)timeout;
    abort();
}

/* Fail the first reset register of every preparation, before a write is possible. */
static int reject_read(void *context, enum bs_space space, uint32_t address, uint32_t *value)
{
    birriasdr_dev_t *dev = context;
    (void)value;
    CHECK(space == BS_MMIO && address == 0x74000018);
    ++model.reads;
    if (model.cancel) atomic_store(&dev->run_state, BS_CANCELLING);
    return model.reads == 1 ? model.first_error : model.later_error;
}

static int reject_write(void *context, enum bs_space space, uint32_t address, uint32_t value)
{
    (void)context; (void)space; (void)address; (void)value;
    abort();
}

/* Check observable retry counts, including zero register accesses on handle loss. */
static void trial(int first, int later, bool cancel, bool prior_reset, int reset_error,
                  int expected, unsigned reads, unsigned resets)
{
    birriasdr_dev_t dev = {0};
    memset(&model, 0, sizeof(model));
    model.first_error = first;
    model.later_error = later;
    model.cancel = cancel;
    model.reset_error = reset_error;
    dev.io = (struct bs_io){&dev, reject_read, reject_write};
    dev.needs_usb_reset = prior_reset;
    atomic_init(&dev.run_state, BS_RUNNING);
    CHECK(mt_prepare(&dev, 0) == expected);
    CHECK(model.reads == reads && model.resets == resets);
    if (reset_error == LIBUSB_ERROR_NOT_FOUND) CHECK(dev.usb_lost && !dev.touched);
}

int main(void)
{
    trial(BIRRIASDR_ERROR_IO, BIRRIASDR_ERROR_IO, false, false, 0, BIRRIASDR_ERROR_IO, 2, 1);
    trial(BIRRIASDR_ERROR_TIMEOUT, BIRRIASDR_ERROR_STATE, false, false, 0, BIRRIASDR_ERROR_STATE, 2, 1);
    trial(BIRRIASDR_ERROR_STATE, 0, false, false, 0, BIRRIASDR_ERROR_STATE, 1, 0);
    trial(BIRRIASDR_ERROR_ACCESS, 0, false, false, 0, BIRRIASDR_ERROR_ACCESS, 1, 0);
    trial(BIRRIASDR_ERROR_NO_DEVICE, 0, false, false, 0, BIRRIASDR_ERROR_NO_DEVICE, 1, 0);
    trial(BIRRIASDR_ERROR_IO, 0, true, false, 0, BIRRIASDR_ERROR_IO, 1, 0);
    trial(0, 0, false, true, LIBUSB_ERROR_NOT_FOUND, BIRRIASDR_ERROR_NO_DEVICE, 0, 1);
    trial(BIRRIASDR_ERROR_STATE, 0, false, true, 0, BIRRIASDR_ERROR_STATE, 1, 1);
    puts("Startup retry, cancellation and fatal-error tests passed");
    return 0;
}
