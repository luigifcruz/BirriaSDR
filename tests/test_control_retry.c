/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Some USB paths drop a fraction of a percent of control transfers with
 * "device not responding" (LIBUSB_ERROR_IO) while the next transfer succeeds.
 * A capture setup issues thousands of transfers, so bs_control resends that
 * one error a few times. Timeouts, removal and short transfers stay fatal on
 * the first failure. Every transfer here is mocked; no hardware is touched.
 */
#include "internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(t) do { if (!(t)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #t); exit(1); } } while (0)

const struct bs_backend bs_mt7921_backend = {0}, bs_mt7925_backend = {0};

static struct {
    unsigned calls;
    unsigned failures;  /* how many leading calls fail */
    int error;          /* what they fail with */
    int short_by;       /* successful calls return this many bytes too few */
} model;

int LIBUSB_CALL libusb_control_transfer(libusb_device_handle *handle, uint8_t type,
                                        uint8_t request, uint16_t value, uint16_t index,
                                        unsigned char *data, uint16_t length, unsigned timeout)
{
    (void)handle; (void)type; (void)request; (void)value; (void)index; (void)timeout;
    if (model.calls++ < model.failures)
        return model.error;
    if (data)
        memset(data, 0x5a, length);
    return length - model.short_by;
}

static int run(unsigned failures, int error, int short_by)
{
    birriasdr_dev_t dev;
    uint8_t bytes[4];
    memset(&dev, 0, sizeof(dev));
    memset(&model, 0, sizeof(model));
    model.failures = failures;
    model.error = error;
    model.short_by = short_by;
    return bs_control(&dev, 0xdf, 0x63, 0x7001, 0x0200, bytes, sizeof(bytes));
}

int main(void)
{
    /* transient "not responding" is retried until it succeeds */
    CHECK(run(2, LIBUSB_ERROR_IO, 0) == 0);
    CHECK(model.calls == 3);

    /* a persistent one gives up after BS_CONTROL_ATTEMPTS */
    CHECK(run(100, LIBUSB_ERROR_IO, 0) == BIRRIASDR_ERROR_IO);
    CHECK(model.calls == BS_CONTROL_ATTEMPTS);

    /* other errors are reported on the first failure */
    CHECK(run(1, LIBUSB_ERROR_TIMEOUT, 0) == BIRRIASDR_ERROR_TIMEOUT);
    CHECK(model.calls == 1);
    CHECK(run(1, LIBUSB_ERROR_NO_DEVICE, 0) == BIRRIASDR_ERROR_NO_DEVICE);
    CHECK(model.calls == 1);

    /* a short transfer is not a transport fault and is not replayed */
    CHECK(run(0, 0, 1) == BIRRIASDR_ERROR_IO);
    CHECK(model.calls == 1);

    puts("control retry: ok");
    return 0;
}
