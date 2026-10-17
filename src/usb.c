/* SPDX-License-Identifier: BSD-3-Clause-Clear
 * Copyright (c) 2026 Luigi C.
 * Register transport/endpoint conventions adapted from the mt76 userland driver.
 * See LICENSES/BSD-3-Clause-Clear.txt for retained upstream notices.
 */
#include "internal.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

/* Enumerate known identities without assuming identical RF wiring across boards. */
static const struct bs_usb_id devices[] = {
    {0x3574, 0x6211, "COMFAST MT7961 / MT7972", &bs_mt7921_backend, true, 0, 0},
    /* The tested EDUP AX3000 cannot calibrate below 2179 MHz. */
    {0x0e8d, 0x7961, "MediaTek MT7961 / MT7972, generic ID", &bs_mt7921_backend, true,
     UINT64_C(2190000000), 0},
    {0x0e8d, 0x7925, "MT7925 USB (not implemented)", &bs_mt7925_backend, false, 0, 0},
    {0x0846, 0x9050, "Netgear A8500 MT7925 (not implemented)", &bs_mt7925_backend, false, 0, 0},
    {0x0846, 0x9072, "Netgear A9000 MT7925 (not implemented)", &bs_mt7925_backend, false, 0, 0}
};

/* A NULL match excludes arbitrary Wi-Fi devices from probing or register writes. */
const struct bs_usb_id *bs_lookup_id(uint16_t vid, uint16_t pid)
{
    for (size_t i = 0; i < sizeof(devices) / sizeof(devices[0]); ++i)
        if (devices[i].vid == vid && devices[i].pid == pid)
            return &devices[i];
    return NULL;
}

/* Normalize transport errors so applications need not include libusb headers. */
int bs_usb_error(int error)
{
    switch (error) {
    case LIBUSB_SUCCESS: return 0;
    case LIBUSB_ERROR_ACCESS: return BIRRIASDR_ERROR_ACCESS;
    case LIBUSB_ERROR_NO_DEVICE: return BIRRIASDR_ERROR_NO_DEVICE;
    case LIBUSB_ERROR_BUSY: return BIRRIASDR_ERROR_BUSY;
    case LIBUSB_ERROR_TIMEOUT: return BIRRIASDR_ERROR_TIMEOUT;
    case LIBUSB_ERROR_NO_MEM: return BIRRIASDR_ERROR_NO_MEMORY;
    default: return BIRRIASDR_ERROR_IO;
    }
}

/* Short control transfers are errors: accepting partial register reads would
 * turn uninitialized bytes into hardware addresses, flags or gain settings.
 */
int bs_control(birriasdr_dev_t *dev, uint8_t type, uint8_t request,
               uint16_t value, uint16_t index, uint8_t *data, uint16_t length)
{
    int result = libusb_control_transfer(dev->handle, type, request, value, index,
                                         data, length, 1000);
    if (result < 0)
        return bs_usb_error(result);
    return result == length ? 0 : BIRRIASDR_ERROR_IO;
}

/* MMIO, host-wrapper reset, and direct RF bridge use distinct USB request types.
 * Their numerical address spaces overlap; selecting the wrong type is unsafe.
 */
static int usb_read32(void *context, enum bs_space space, uint32_t address, uint32_t *value)
{
    birriasdr_dev_t *dev = context;
    uint8_t bytes[4];
    uint8_t type = space == BS_UHW ? 0xde : space == BS_BRIDGE ? 0xc0 : 0xdf;
    uint8_t request = space == BS_UHW ? 0x01 : 0x63;
    BS_TRY(bs_control(dev, type, request, (uint16_t)(address >> 16),
                       (uint16_t)address, bytes, sizeof(bytes)));
    *value = bs_get_le32(bytes);
    return 0;
}

/* Use the corresponding write request, without exposing arbitrary register I/O
 * in the public API. Chipset backends are responsible for allowed sequences.
 */
static int usb_write32(void *context, enum bs_space space, uint32_t address, uint32_t value)
{
    birriasdr_dev_t *dev = context;
    uint8_t bytes[4];
    uint8_t type = space == BS_UHW ? 0x5e : space == BS_BRIDGE ? 0x40 : 0x5f;
    uint8_t request = space == BS_UHW ? 0x02 : 0x66;
    bs_put_le32(bytes, value);
    return bs_control(dev, type, request, (uint16_t)(address >> 16),
                       (uint16_t)address, bytes, sizeof(bytes));
}

/* ROM command/download packets must be transmitted in full. Never replay a
 * short packet: the ROM may already have consumed part of the command.
 */
int bs_bulk_out(birriasdr_dev_t *dev, uint8_t ep, const uint8_t *data, int length)
{
    int actual = 0;
    int result = libusb_bulk_transfer(dev->handle, ep, (uint8_t *)data, length, &actual, 3000);
    if (result < 0)
        return bs_usb_error(result);
    return actual == length ? 0 : BIRRIASDR_ERROR_IO;
}

/* Old aggregated I/Q can be larger than a ROM reply buffer. Drain with full
 * 64 KiB reads before sending any new command, so a stale event cannot overflow
 * the first semaphore reply or collide with a reused command sequence number.
 * A timeout carrying partial data is NOT an empty pipe. Bound both elapsed
 * time and transfer count, and never discard data once the new capture begins.
 * The caller has reaped asynchronous transfers and initialized ROM USB/DMA.
 */
int bs_usb_prepare_endpoints(birriasdr_dev_t *dev)
{
    uint8_t bytes[65536];
    uint64_t deadline;
    deadline = bs_now_ns() + UINT64_C(250000000);
    for (unsigned i = 0; i < 64 && bs_now_ns() < deadline; ++i) {
        int actual = 0;
        int result = libusb_bulk_transfer(dev->handle, dev->ep_in, bytes,
                                          sizeof(bytes), &actual, 10);
        if (result == LIBUSB_ERROR_TIMEOUT && actual == 0)
            return 0;
        if (result != LIBUSB_ERROR_TIMEOUT && result != LIBUSB_ERROR_OVERFLOW)
            BS_TRY(bs_usb_error(result));
    }
    return BIRRIASDR_ERROR_TIMEOUT;
}

/* Repair transport after failed acquisition or boot. A WFSYS register reset
 * does not necessarily repair the host/device USB protocol state. In testing,
 * a persistent bulk EPROTO survived WFSYS resets and close/reopen; this USB reset
 * restored it. Do not proactively clear halts on healthy pipes: doing so caused
 * that failure on the tested MT7961. No transfer may remain submitted here.
 *
 * Keep the same claimed handle and advisory lock. libusb restores configuration
 * on success. NOT_FOUND means re-enumeration invalidated the handle: stop and
 * report device loss, rather than guess an enumeration index or touch another
 * dongle. This resets USB, not hub power, and does not download a vendor image.
 */
int bs_usb_recover(birriasdr_dev_t *dev)
{
    if (dev->usb_lost)
        return BIRRIASDR_ERROR_NO_DEVICE;
    int result = libusb_reset_device(dev->handle);
    if (result == LIBUSB_ERROR_NOT_FOUND || result == LIBUSB_ERROR_NO_DEVICE) {
        dev->usb_lost = true;
        dev->touched = false;
        return BIRRIASDR_ERROR_NO_DEVICE;
    }
    return bs_usb_error(result);
}

/* Share the existing prototype's lock for the tested dongle. Additional devices
 * use bus/address locks; all locks are advisory alongside libusb's interface claim.
 * Refuse symlinks/nonregular files in the shared temporary directory.
 */
static int lock_device(birriasdr_dev_t *dev, const struct bs_usb_id *id, libusb_device *usb)
{
    char path[128];
    struct stat st;
    if (id->vid == 0x3574 && id->pid == 0x6211)
        snprintf(path, sizeof(path), "/tmp/sdr-wifi-3574-6211.lock");
    else
        snprintf(path, sizeof(path), "/tmp/birriasdr-%u-%u.lock",
                 libusb_get_bus_number(usb), libusb_get_device_address(usb));
    dev->lock_fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (dev->lock_fd < 0)
        return BIRRIASDR_ERROR_ACCESS;
    if (fstat(dev->lock_fd, &st) || !S_ISREG(st.st_mode))
        return BIRRIASDR_ERROR_ACCESS;
    if (flock(dev->lock_fd, LOCK_EX | LOCK_NB))
        return errno == EWOULDBLOCK ? BIRRIASDR_ERROR_BUSY : BIRRIASDR_ERROR_ACCESS;
    return 0;
}

/* mt76 assigns endpoint roles by descriptor order within the ff/ff/ff interface:
 * first IN = packet/event data, first OUT = commands, second OUT = downloads.
 * Do not hard-code the interface number: Bluetooth combination devices differ.
 */
static int find_interface(birriasdr_dev_t *dev, libusb_device *usb)
{
    struct libusb_config_descriptor *config = NULL;
    int matches = 0;
    int result = libusb_get_active_config_descriptor(usb, &config);
    if (result < 0)
        return bs_usb_error(result);
    for (unsigned i = 0; i < config->bNumInterfaces; ++i) {
        const struct libusb_interface *interface = &config->interface[i];
        for (int a = 0; a < interface->num_altsetting; ++a) {
            const struct libusb_interface_descriptor *alt = &interface->altsetting[a];
            uint8_t in[2] = {0}, out[6] = {0};
            unsigned ni = 0, no = 0;
            if (alt->bAlternateSetting || alt->bInterfaceClass != 0xff ||
                alt->bInterfaceSubClass != 0xff || alt->bInterfaceProtocol != 0xff)
                continue;
            for (unsigned e = 0; e < alt->bNumEndpoints; ++e) {
                const struct libusb_endpoint_descriptor *ep = &alt->endpoint[e];
                if ((ep->bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK)
                    continue;
                if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                    if (ni < 2) in[ni] = ep->bEndpointAddress;
                    ++ni;
                } else {
                    if (no < 6) out[no] = ep->bEndpointAddress;
                    ++no;
                }
            }
            if (ni != 2 || no != 6)
                continue;
            ++matches;
            dev->interface_number = alt->bInterfaceNumber;
            dev->ep_in = in[0];
            dev->ep_command = out[0];
            dev->ep_download = out[1];
        }
    }
    libusb_free_config_descriptor(config);
    return matches == 1 ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* Reject unqualified boards before even opening them. Kernel ownership is also
 * rejected instead of silently disconnecting a network or retaining a ROM patch.
 */
static int claim_device(birriasdr_dev_t *dev, libusb_device *usb, const struct bs_usb_id *id)
{
    int result;
    if (!id->qualified || !(id->backend->capabilities & BIRRIASDR_CAP_RX))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    if (libusb_get_device_speed(usb) < LIBUSB_SPEED_SUPER)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    dev->backend = id->backend;
    dev->min_frequency = id->min_frequency;
    dev->max_frequency = id->max_frequency;
    dev->frequency = dev->backend->default_frequency;
    dev->rate = dev->backend->default_rate;
    BS_TRY(lock_device(dev, id, usb));
    BS_TRY(bs_usb_error(libusb_open(usb, &dev->handle)));
    BS_TRY(find_interface(dev, usb));
    result = libusb_kernel_driver_active(dev->handle, dev->interface_number);
    if (result == 1)
        return BIRRIASDR_ERROR_BUSY;
    if (result < 0 && result != LIBUSB_ERROR_NOT_SUPPORTED)
        return bs_usb_error(result);
    result = libusb_claim_interface(dev->handle, dev->interface_number);
    if (result < 0) {
        dev->interface_number = -1;
        return bs_usb_error(result);
    }
    dev->io = (struct bs_io){dev, usb_read32, usb_write32};
    return dev->backend->identify(dev);
}

/* A single descriptor walk implements both counting and indexed opening. The
 * opened handle retains its context; read-only enumeration destroys its context.
 */
int bs_usb_discover(uint32_t index, struct birriasdr_device_info *info, birriasdr_dev_t *dev)
{
    libusb_context *context = NULL;
    libusb_device **list = NULL;
    ssize_t count;
    uint32_t found = 0;
    int result = libusb_init(&context);
    if (result < 0)
        return bs_usb_error(result);
    if (dev)
        dev->usb = context;
    count = libusb_get_device_list(context, &list);
    result = count < 0 ? bs_usb_error((int)count) : BIRRIASDR_ERROR_NO_DEVICE;
    for (ssize_t i = 0; i < count; ++i) {
        struct libusb_device_descriptor descriptor;
        const struct bs_usb_id *id;
        if (libusb_get_device_descriptor(list[i], &descriptor) < 0)
            continue;
        id = bs_lookup_id(descriptor.idVendor, descriptor.idProduct);
        if (!id)
            continue;
        if (found++ != index || (!info && !dev))
            continue;
        if (info) {
            info->vid = id->vid;
            info->pid = id->pid;
            info->bus = libusb_get_bus_number(list[i]);
            info->address = libusb_get_device_address(list[i]);
            info->chipset = id->backend->chipset;
            info->capabilities = id->qualified ? id->backend->capabilities : 0;
            snprintf(info->name, sizeof(info->name), "%s", id->name);
        }
        result = dev ? claim_device(dev, list[i], id) : 0;
        break;
    }
    if (!info && !dev && count >= 0)
        result = (int)found;
    if (list)
        libusb_free_device_list(list, 1);
    if (!dev)
        libusb_exit(context);
    return result;
}

/* Cleanup is valid after any partial open. Releasing a libusb interface does
 * not request automatic kernel reattachment, so it cannot load stock firmware.
 */
void bs_usb_close(birriasdr_dev_t *dev)
{
    if (dev->handle) {
        if (dev->interface_number >= 0 && !dev->usb_lost)
            libusb_release_interface(dev->handle, dev->interface_number);
        libusb_close(dev->handle);
    }
    if (dev->usb)
        libusb_exit(dev->usb);
    if (dev->lock_fd >= 0)
        close(dev->lock_fd);
}
