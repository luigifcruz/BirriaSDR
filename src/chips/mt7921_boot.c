/* SPDX-License-Identifier: BSD-3-Clause-Clear
 * Copyright (c) 2026 Luigi C.
 * USB/ROM/WFDMA sequencing adapted from the project's mt76 userland driver.
 * See LICENSES/BSD-3-Clause-Clear.txt for retained upstream notices.
 */

/*
 * MT7961 boot and transport handoff
 *
 * This file takes the device from its immutable boot ROM to our RAM-resident
 * I/Q producer. The caller already owns the USB interface and device lock.
 * mt_prepare() performs the following sequence, with ICAP stopped throughout:
 *
 *   restore gain -> WFSYS reset -> ROM USB/DMA -> stale input drain -> patch check
 *   -> original program download -> ICAP setup -> RF setup -> gain -> matched filter
 *
 * Reset is not equivalent to removing power. Analog gain overrides can survive
 * WFSYS reset, so mt_reset() restores our saved fields before resetting. A
 * vendor ROM patch can also survive; the ROM semaphore must report unpatched
 * state before we load code. A rejected patch state requires a cold boot, not
 * a fallback to a downloaded vendor image.
 *
 * There are two DMA domains to keep separate. Registers at 0x7c024xxx and
 * 0x7c026xxx configure the host-interface WFDMA rings and scheduler. The MCU
 * program owns the separate 0x5400xxxx device-to-host descriptor engine after
 * handoff. "TX" in the host-interface setup means USB host-to-device traffic;
 * it does not enable RF transmission. USB host-wrapper reset registers also
 * require BS_UHW accesses rather than ordinary BS_MMIO requests.
 *
 * ROM commands use the Connac2 envelope; scatter data bypasses its MCU header.
 * The image goes to instruction RAM at MT_ENTRY (0x00915000). PDA completion
 * and checksum checks establish delivery; the mailbox magic and fresh nonce
 * response establish execution. FW_START does not synchronously consume its
 * ACK here: the streaming framer accepts that remaining ROM event later.
 *
 * firmware/iq_stream.S is the source of the embedded image. It calls the
 * immutable ROM PHY power-on helper at 0x837142, initializes its DMA ring and
 * waits at the GO gate. Only after RF/gain setup and queued host USB reads may
 * mt_start() release GO. USB aggregation combines events, not I/Q samples.
 * These operations write volatile state, not flash or eFuse.
 *
 * Porting this path requires validating the ROM entry points, RAM layout,
 * request types and DMA ownership on the new die. A matching USB product ID
 * alone is insufficient. See docs/mt7921.md and mt7921_capture.c for the
 * handoff contract and mt7921_frame.c for the resulting event format.
 */
#include "mt7921.h"
#include "mt7921_image.h"
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

/* Disable special endpoint-reset preservation so WFSYS reset restores the ROM
 * transport consistently. This register uses the USB host-wrapper request pair.
 */
static int endpoint_reset_options(birriasdr_dev_t *dev)
{
    return bs_update32(dev, BS_UHW, 0x74011890, 0x007003f0, 0);
}

/* Flush UDMA and pulse WFSYS reset, leaving immutable boot ROM in control.
 * This does not clear a retained ROM patch; prepare explicitly checks for one.
 * Attempt reset even when the pre-reset flush times out, as the Python path did.
 */
static int reset_hardware(birriasdr_dev_t *dev)
{
    int idle;
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 0, 1u << 19));
    idle = bs_poll32(dev, BS_MMIO, 0x74000018, 0xc0000000, 0, 1000);
    if (idle < 0 && idle != BIRRIASDR_ERROR_TIMEOUT)
        return idle;
    BS_TRY(endpoint_reset_options(dev));
    BS_TRY(bs_update32(dev, BS_UHW, 0x70002600, 0, 1));
    bs_sleep_us(1000);
    BS_TRY(bs_update32(dev, BS_UHW, 0x70002600, 1, 0));
    BS_TRY(bs_write32(dev, BS_UHW, 0x74000a24, 0));
    return bs_poll32(dev, BS_UHW, 0x74000a20, 1u << 22, 1u << 22, 200);
}

/* Analog gain, bandwidth and band fields can survive WFSYS reset. Restore every
 * saved baseline first, but attempt every cleanup even when an earlier one fails.
 */
int mt_reset(birriasdr_dev_t *dev)
{
    int restore = mt_restore_gain(dev);
    int filter = mt_restore_filter(dev);
    int band = mt_restore_band(dev);
    int reset = reset_hardware(dev);
    if (restore < 0) return restore;
    if (filter < 0) return filter;
    return band < 0 ? band : reset;
}

/* Prepare host-side WFDMA descriptor prefetch and scheduler quotas. "TX" here
 * means host-to-device USB transport for ROM commands, not transmission on air.
 * Firmware owns the separate 0x5400xxxx device-to-host DMA ring after handoff.
 */
static int configure_dma(birriasdr_dev_t *dev)
{
    static const unsigned rings[] = {0, 1, 2, 3, 4, 16, 17};
    static const unsigned bases[] = {0x80, 0xc0, 0x100, 0x140, 0x180, 0x280, 0x2c0};
    for (size_t i = 0; i < 7; ++i)
        BS_TRY(bs_update32(dev, BS_MMIO, 0x7c024600 + 4 * rings[i],
                            0xffff00ff, bases[i] << 16 | 4));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c024208, 1u << 27, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c024208, 0, 0x10200205));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c026010, 0xffff0000, 0xffe00000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c02600c, 1u << 16, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c02601c, 0x0fff0fff, 1));
    for (unsigned i = 0; i < 16; ++i)
        BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026020 + i * 4, i < 5 ? 0x0fff0003 : 0));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026060, 0x32013201));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026064, 0x32013201));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026068, 0x55555444));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c02606c, 0x55555444));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026070, 0x76540132));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x7c026074, 0xfedcba98));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x54000120, 0, 2));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 1u << 19, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 0, 0x00d40000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7400000c, 0x0fffff00, 50000u << 8));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 0, 1u << 16));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 0xffff, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7400000c, 0xff, 0));
    BS_TRY(bs_poll32(dev, BS_MMIO, 0x7c024208, 8, 0, 1000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c024208, 4, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c027030, 0, 0x40));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7c024208, 0, 4));
    return endpoint_reset_options(dev);
}

/* Build the Connac2 ROM command envelope explicitly, avoiding packed C structs
 * and host-endian assumptions. Only boot, semaphore, scatter and start commands
 * exist here; there is no vendor RAM/patch loader or transmit command path.
 */
static int rom_command(birriasdr_dev_t *dev, uint8_t command,
                        const uint8_t *payload, size_t size, int *status)
{
    uint8_t frame[sizeof(mt7921_image) + 80] = {0}, reply[4096];
    bool scatter = command == 0xee;
    size_t body = size + (scatter ? 0 : 64);
    size_t total = ((body + 4 + 3) & ~(size_t)3) + 4;
    uint64_t deadline;
    if (total > sizeof(frame))
        return BIRRIASDR_ERROR_INVALID;
    dev->command_sequence = (uint8_t)(dev->command_sequence % 15 + 1);
    bs_put_le32(frame, (uint32_t)body);
    if (!scatter) {
        uint8_t *header = frame + 4;
        bs_put_le32(header, (uint32_t)body | 0x41000000);
        bs_put_le32(header + 4, 0x80010000);
        bs_put_le16(header + 32, (uint16_t)(body - 32));
        bs_put_le16(header + 34, 0x8000);
        header[36] = command;
        header[37] = 0xa0;
        header[38] = 3;
        header[39] = dev->command_sequence;
    }
    memcpy(frame + 4 + (scatter ? 0 : 64), payload, size);
    BS_TRY(bs_bulk_out(dev, scatter ? dev->ep_download : dev->ep_command, frame, (int)total));
    if (!status)
        return 0;
    deadline = bs_now_ns() + UINT64_C(3000000000);
    do {
        int actual = 0;
        int result = libusb_bulk_transfer(dev->handle, dev->ep_in, reply, sizeof(reply), &actual, 500);
        if (result == LIBUSB_ERROR_TIMEOUT)
            continue;
        if (result < 0)
            return bs_usb_error(result);
        if (actual >= 36 && bs_get_le32(reply) >> 27 == 7 &&
            reply[29] == dev->command_sequence) {
            *status = reply[32];
            return 0;
        }
    } while (bs_now_ns() < deadline);
    return BIRRIASDR_ERROR_TIMEOUT;
}

/* Refuse retained patches before downloading any code. A WFSYS reset is not
 * proof of a cold boot: ROM's semaphore distinguishes the unpatched state.
 */
static int require_unpatched_rom(birriasdr_dev_t *dev)
{
    uint8_t payload[4] = {1, 0, 0, 0};
    int status;
    BS_TRY(rom_command(dev, 0x10, payload, sizeof(payload), &status));
    if (status != 2)
        return BIRRIASDR_ERROR_COLD_BOOT;
    payload[0] = 0;
    BS_TRY(rom_command(dev, 0x10, payload, sizeof(payload), &status));
    return status == 3 ? 0 : BIRRIASDR_ERROR_STATE;
}

/* Seed a fresh challenge so old mailbox contents and stale USB frames cannot
 * masquerade as execution of this run's original firmware.
 */
static int initialize_mailbox(birriasdr_dev_t *dev, uint32_t packets)
{
    uint8_t nonce[4];
    int fd = open("/dev/urandom", O_RDONLY | O_CLOEXEC);
    ssize_t count;
    if (fd < 0)
        return BIRRIASDR_ERROR_IO;
    count = read(fd, nonce, sizeof(nonce));
    close(fd);
    if (count != sizeof(nonce))
        return BIRRIASDR_ERROR_IO;
    dev->nonce = bs_get_le32(nonce);
    for (unsigned i = 0; i < 16; ++i) {
        uint32_t expected = i == 1 ? dev->nonce : i == 8 ? packets : 0;
        uint32_t actual;
        BS_TRY(bs_write32(dev, BS_MMIO, MT_MAILBOX + i * 4, expected));
        BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + i * 4, &actual));
        if (actual != expected)
            return BIRRIASDR_ERROR_STATE;
    }
    /* Source recovery is opt-in and never joins gaps in a finite capture. */
    uint32_t recover = !packets && dev->recovery_limit && dev->recovery_callback;
    uint32_t actual;
    BS_TRY(bs_write32(dev, BS_MMIO, MT_MAILBOX + 96, recover));
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 96, &actual));
    if (actual != recover)
        return BIRRIASDR_ERROR_STATE;
    return 0;
}

/* Transfer our original image through ROM's PDA engine, including its generated
 * alignment/checksum trailer. Execution is proven by the nonce response, not
 * merely by accepting a start command. The device calls immutable ROM PHY power-on.
 */
static int download_original_program(birriasdr_dev_t *dev)
{
    uint8_t payload[12];
    uint32_t value;
    int status;
    BS_TRY(bs_write32(dev, BS_MMIO, 0x54000808, 0x80000000));
    bs_put_le32(payload, MT_ENTRY);
    bs_put_le32(payload + 4, sizeof(mt7921_image));
    bs_put_le32(payload + 8, 0x80000000);
    BS_TRY(rom_command(dev, 1, payload, sizeof(payload), &status));
    if (status)
        return BIRRIASDR_ERROR_PROTOCOL;
    BS_TRY(rom_command(dev, 0xee, mt7921_image, sizeof(mt7921_image), NULL));
    bs_sleep_us(10000);
    BS_TRY(bs_read32(dev, BS_MMIO, 0x54000808, &value));
    if (value != 0x25)
        return BIRRIASDR_ERROR_STATE;
    BS_TRY(bs_read32(dev, BS_MMIO, 0x54000814, &value));
    if (value)
        return BIRRIASDR_ERROR_PROTOCOL;
    bs_put_le32(payload, 1);
    bs_put_le32(payload + 4, MT_ENTRY);
    BS_TRY(rom_command(dev, 2, payload, 8, NULL));
    BS_TRY(bs_poll32(dev, BS_MMIO, MT_MAILBOX, UINT32_MAX, MT_BOOT_MAGIC, 3000));
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 4, &value));
    if (value != dev->nonce)
        return BIRRIASDR_ERROR_PROTOCOL;
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 8, &value));
    if (value != (dev->nonce ^ MT_RESPONSE_XOR))
        return BIRRIASDR_ERROR_PROTOCOL;
    BS_TRY(bs_read32(dev, BS_MMIO, MT_MAILBOX + 28, &value));
    return value == 1 ? 0 : BIRRIASDR_ERROR_STATE;
}

/* Reset -> ROM transport -> original program -> ICAP/RF/gain. The program waits
 * at its GO gate throughout host configuration. USB aggregation combines several
 * framed DMA packets per transfer; it does not change sample packing or rate.
 */
static int prepare_once(birriasdr_dev_t *dev, uint32_t packets)
{
    uint8_t power[4] = {1, 0, 0, 0};
    dev->touched = true;
    dev->stats.stage = BIRRIASDR_STAGE_RESET;
    BS_TRY(mt_reset(dev));
    dev->stats.stage = BIRRIASDR_STAGE_ROM;
    BS_TRY(bs_control(dev, 0x5f, 4, 0, 1, NULL, 0));
    BS_TRY(bs_poll32(dev, BS_MMIO, 0x7c0600f0, 1, 1, 500));
    BS_TRY(configure_dma(dev));
    BS_TRY(bs_write32(dev, BS_MMIO, 0x0041f23c, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000008, 0, 8));
    BS_TRY(bs_usb_prepare_endpoints(dev));
    BS_TRY(rom_command(dev, 4, power, sizeof(power), NULL));
    BS_TRY(bs_poll32(dev, BS_MMIO, 0x7c0600f0, 7, 1, 1000));
    BS_TRY(require_unpatched_rom(dev));
    dev->stats.stage = BIRRIASDR_STAGE_DOWNLOAD;
    BS_TRY(initialize_mailbox(dev, packets));
    BS_TRY(download_original_program(dev));
    dev->stats.stage = BIRRIASDR_STAGE_CAPTURE;
    BS_TRY(mt_configure_capture(dev));
    dev->stats.stage = BIRRIASDR_STAGE_RF;
    BS_TRY(mt_configure_rf(dev));
    dev->stats.stage = BIRRIASDR_STAGE_GAIN;
    BS_TRY(mt_apply_gain(dev));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x74000018, 0xffff, (1u << 21) | (32u << 8) | 100));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7400000c, 0xff, 30));
    dev->stats.stage = BIRRIASDR_STAGE_RF;
    return mt_configure_filter(dev);
}

/* Retry a transient startup failure once, from a complete USB/WFSYS reset.
 * No asynchronous reads have been submitted and GO has not been released yet,
 * so restarting this entire volatile sequence cannot splice sample timelines.
 * Never retry an individual ROM command: a failed transfer may have partially
 * executed. A new attempt checks patch state and boots with a fresh nonce.
 *
 * Failed streams may request the stronger transport reset on the next prepare.
 * A drained source overflow with successful cleanup first tries ordinary boot.
 * A lost/changed USB handle, retained vendor patch, calibration rejection or
 * unsupported hardware is reported directly. This does not replace cold power
 * removal when a stock ROM patch was loaded before our library acquired USB.
 */
int mt_prepare(birriasdr_dev_t *dev, uint32_t packets)
{
    int result = 0;
    if (dev->usb_lost)
        return BIRRIASDR_ERROR_NO_DEVICE;
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (dev->needs_usb_reset) {
            dev->stats.stage = BIRRIASDR_STAGE_RESET;
            BS_TRY(bs_usb_recover(dev));
            dev->needs_usb_reset = false;
        }
        result = prepare_once(dev, packets);
        if (!result || (result != BIRRIASDR_ERROR_IO && result != BIRRIASDR_ERROR_TIMEOUT))
            return result;
        dev->needs_usb_reset = true;
        if (atomic_load(&dev->run_state) == BS_CANCELLING)
            break;
    }
    return result;
}
