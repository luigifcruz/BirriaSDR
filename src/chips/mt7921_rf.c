/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 * Original RF implementation, ported from baremetal/rf_{spi,init,tune}.py and
 * agc_probe.py. See docs/mt7921.md for setup and unresolved field meanings.
 */

/*
 * MT7972 analog receiver initialization for the MT7961 capture path
 *
 * Successful USB DMA does not establish RF reception. This file supplies the
 * reference clocks, bias, synthesizer calibration and receive routing that
 * ICAP needs before its samples contain an external signal. The qualified
 * profile is the COMFAST board with analog ID 0x79728a10, a 40 MHz crystal and
 * channel 6 (2437 MHz). The synthesizer reference is 60 MHz; it is distinct
 * from both the crystal frequency and the 40 MSps complex capture rate.
 * Requested centers first establish that anchor, then retune only the
 * selected secondary oscillator through mt7921_tune.c with 20 fractional bits.
 *
 * Analog registers sit behind the RF SPI bridge at 0x7c004000, accessed using
 * BS_BRIDGE USB requests. Wi-Fi ports share command/data/result offsets
 * +0x10/+0x14/+0x18 and busy bit 1; TOP uses +0x50/+0x54/+0x58 and busy bit 5.
 * Writing DATA launches a transaction. Poll idle before and after each one.
 * The host must own this bridge: MCU RF work must be finished/parked, rather
 * than racing these read-modify-write sequences with another RF controller.
 *
 * Physical SPI port numbers are not logical receive-chain numbers. SPI 1 is
 * receive chain 0; SPI 0 supplies the selected secondary synthesizer despite
 * its receiver being idle; SPI 5 is TOP. TOP 0xb00 and chain 0xb00 are distinct
 * registers. Always retain the port when documenting or probing an address.
 *
 * Initialization order is part of the hardware protocol:
 *   - Set up TOP references, hand over the digital clock mux, then finish wake.
 *     When strap 0x70011094[2] selects it, TOP 0xb00[16] is temporarily set.
 *     Clear it, wait and check readback before tuning. Leaving it set was the
 *     cause of missing channel-6 signals despite a working sample transport.
 *   - Bias and prepare both synthesizer banks. The 2.4 GHz LO uses a VCO at
 *     twice the receive frequency. Bootstrap tuning retains its 18-bit fraction
 *     shifted left by two, with a self-clearing LOAD bit in 0xc68 or 0xca8.
 *     The final selected oscillator uses the full 20-bit fraction; the channel-6
 *     word is identical at either precision, preserving the bootstrap trace.
 *   - Run VCO calibration, poll 0xd7c[21], and latch 0xd78[2:0] into
 *     0xc0c[24:22]. DONE alone proves neither PLL lock nor tuning accuracy.
 *     Stop the calibration machinery on both success and failure.
 *   - Select the final LO and receive path. RF 0x1e4 differs between active
 *     SPI 1 (0x3fb0b) and idle SPI 0 (0x23c1). Clearing the idle bank's bits
 *     7:6 by copying the active value to both reproduced weak reception.
 *
 * The ordered tables contain decoded setup constants, not per-unit calibration
 * dumps. Several bias fields remain unidentified. Do not sort or deduplicate
 * writes: intermediate settings, action strobes and settling waits matter.
 * RF 0x160[31:30]=0 retains the established bootstrap profile during bring-up.
 * mt7921_filter.c then applies the matched analog/digital 40 MHz profile before
 * capture. Keep bootstrap ordering separate from the final bandwidth setting;
 * calibrated passband, other bands and full calibration remain unqualified.
 * See docs/mt7921.md for an overview and tests/python-channel6.trace for writes.
 */
#include "mt7921.h"

struct rf_word { uint32_t address, value; };

/* The RF SPI bridge is a command/data/result triplet. Writing DATA launches
 * the transfer; a busy bit protects both preceding and following accesses.
 * Wi-Fi ports share triplet +0x10/busy bit 1; TOP uses +0x50/busy bit 5.
 * The MCU must not use this bridge concurrently. Our streaming program leaves
 * it alone after GO; the control mutex serializes all host register procedures.
 */
static int rf_transfer(birriasdr_dev_t *dev, unsigned port, uint32_t address,
                       uint32_t *value, bool read)
{
    const uint32_t base = 0x7c004000;
    uint32_t command, control, busy;
    if ((port != 0 && port != 1 && port != 5) || address > 0xfff || address % 4)
        return BIRRIASDR_ERROR_INVALID;
    command = base + (port == MT_TOP_PORT ? 0x50 : 0x10);
    busy = 1u << (port == MT_TOP_PORT ? 5 : 1);
    BS_TRY(bs_poll32(dev, BS_BRIDGE, base, busy, 0, 100));
    BS_TRY(bs_read32(dev, BS_BRIDGE, command, &control));
    control = (control & 0xffff0000u) | port << 13 | address | (read ? 0x1000 : 0);
    BS_TRY(bs_write32(dev, BS_BRIDGE, command, control));
    BS_TRY(bs_write32(dev, BS_BRIDGE, command + 4, read ? 0 : *value));
    BS_TRY(bs_poll32(dev, BS_BRIDGE, base, busy, 0, 100));
    return read ? bs_read32(dev, BS_BRIDGE, command + 8, value) : 0;
}

/* Read one analog register; TOP and Wi-Fi banks are explicitly distinguished. */
int mt_rf_read(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t *value)
{
    return rf_transfer(dev, port, address, value, true);
}

/* Write one analog register through the bounded SPI handshake. */
int mt_rf_write(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t value)
{
    return rf_transfer(dev, port, address, &value, false);
}

/* Change only decoded fields, preserving calibration and unrelated state. */
int mt_rf_update(birriasdr_dev_t *dev, unsigned port, uint32_t address,
                 uint32_t clear, uint32_t set)
{
    uint32_t value;
    BS_TRY(mt_rf_read(dev, port, address, &value));
    return mt_rf_write(dev, port, address, (value & ~clear) | set);
}

/* Apply an ordered group of decoded constants. Tables hold only unconditional
 * writes; timing, handshakes and conditional decisions remain explicit code.
 */
static int rf_words(birriasdr_dev_t *dev, unsigned port,
                    const struct rf_word *words, size_t count)
{
    for (size_t i = 0; i < count; ++i)
        BS_TRY(mt_rf_write(dev, port, words[i].address, words[i].value));
    return 0;
}

/* Install the tested 60 MHz synthesizer-reference profile and common 2.4 GHz
 * bias fields on one bank. These constants were decoded from this die's setup;
 * individual resistor/bias meanings are not fully known. They are not a dump of
 * unit-specific calibration coefficients and must not be reused on another die.
 */
static int configure_bias(birriasdr_dev_t *dev, unsigned port)
{
    static const struct rf_word synthesizer[] = {
        {0xc08, 0x4001e000}, {0xc10, 0x00011f40}, {0xc1c, 0xd0402908},
        {0xc20, 0x06080208}, {0xcd0, 0x00010016}, {0xce4, 0x0b9c8765},
        {0xce8, 0x0000000b}, {0xcec, 0x000a09c6}, {0xc00, 0x00250028},
        {0xc54, 0x00081814}, {0xc60, 0x00082408}, {0xc64, 0x60000000},
        {0xc70, 0x03463011}, {0xcc8, 0x784a812b}, {0xd00, 0x00100000},
        {0xd04, 0x33444555}, {0xd08, 0x56664455}, {0xd0c, 0x66778899},
        {0xd10, 0x00003a27}, {0xc78, 0x0003737e}, {0xc94, 0x00081814},
        {0xca0, 0x00082408}, {0xca4, 0x60000000}, {0xcb0, 0x03463011},
        {0xcb8, 0x0003737e}
    };
    static const struct rf_word receiver[] = {
        {0xc78, 0x0003737e}, {0x000, 0x63602350}, {0x018, 0x00200000},
        {0x008, 0x03500008}, {0x124, 0x14ac4f40}, {0x120, 0x03d18ff2},
        {0x0b8, 0x734fc0fc}, {0x0b4, 0x36364ff0}, {0x0bc, 0x28dc303f},
        {0x0e4, 0x00517bfe}, {0x070, 0xe4800000}, {0x078, 0x1fbf4490},
        {0x0b0, 0xffa88f00}, {0x0f4, 0x414f0005}, {0x0c8, 0x00000c80},
        {0x060, 0x0047da7c}, {0x098, 0xf110cc00}, {0x040, 0x03509f88},
        {0x048, 0x0004c8a0}, {0x058, 0x00600359}, {0x170, 0x00000001},
        {0x038, 0x40000db0}, {0x084, 0x8f703000}, {0x0c0, 0xb5af2000},
        {0x03c, 0x1007a255}
    };
    BS_TRY(rf_words(dev, port, synthesizer, sizeof(synthesizer) / sizeof(synthesizer[0])));
    return rf_words(dev, port, receiver, sizeof(receiver) / sizeof(receiver[0]));
}

/* Program TOP's analog reference, hand the digital clock mux to it, then finish
 * the wake sequence. The original bug left TOP B00[16] set after clock setup:
 * that standby state suppressed channel-6 signals despite successful USB DMA.
 * It MUST be cleared before RF tuning when strap 70011094[2] selects this path.
 */
static int configure_clocks(birriasdr_dev_t *dev)
{
    static const struct rf_word reference[] = {
        {0x044, 0x462e0154}, {0x048, 0x062e0154}, {0x344, 0x009fe088},
        {0x374, 0x009fe088}, {0x034, 0xc0088de0}, {0xbb0, 0x43271421}
    };
    static const struct rf_word dividers[] = {
        {0x33c, 0x61}, {0x320, 0x62}, {0x36c, 0x61}, {0x350, 0x62}
    };
    uint32_t strap, switched;
    BS_TRY(rf_words(dev, MT_TOP_PORT, reference, sizeof(reference) / sizeof(reference[0])));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb04, 0x100, 0));
    bs_sleep_us(20);
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb04, 0, 0x100));
    bs_sleep_us(20);
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb04, 0x100, 0));
    BS_TRY(mt_rf_write(dev, MT_TOP_PORT, 0xbb8, 0xcacccccc));
    BS_TRY(mt_rf_write(dev, MT_TOP_PORT, 0xb08, 0x800));
    for (size_t i = 0; i < sizeof(dividers) / sizeof(dividers[0]); ++i)
        BS_TRY(mt_rf_update(dev, MT_TOP_PORT, dividers[i].address, 0xfff, dividers[i].value));

    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000148, 0, 0x10));
    BS_TRY(bs_poll32(dev, BS_MMIO, 0x70000148, 0x80, 0x80, 100));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000100, 0x3f0, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000108, 0, 0x20));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000108, 0x03000000, 0x01000000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000148, 0, 1));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70003000, 0, 8));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70000148, 0x10, 0));
    BS_TRY(bs_read32(dev, BS_MMIO, 0x70011094, &switched));
    if (switched & 4) {
        BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb00, 0, 0x10000));
        bs_sleep_us(20);
    }
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70003014, 0, 0x8000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70005000, 0, 0x8000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70005000, 0, 0x800));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70005000, 0, 8));
    BS_TRY(bs_read32(dev, BS_MMIO, 0x7001000c, &strap));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7000500c, 0xe000, strap & (1u << 13) ? 0x6000 : 0x4000));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x7000500c, 0xe, strap & (1u << 14) ? 6 : 4));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70005008, 0xe00, strap & (1u << 10) ? 0x600 : 0x400));

    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb04, 0xf000, 0x7000));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xaa0, 0x30000000, 0x20000000));
    bs_sleep_us(20);
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb04, 0xf000, 0));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xaa0, 0x80000000, 0));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x308, 0x3ff, 9));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x42c, 0x3ff, 9));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x300, 0x03ff0000, 0x00080000));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x424, 0x03ff0000, 0x00080000));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x00c, 0x30000000, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70004100, 8, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x70004110, 8, 0));
    BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0x00c, 0xc0000000, 0));
    if (switched & 4) {
        BS_TRY(mt_rf_update(dev, MT_TOP_PORT, 0xb00, 0x10000, 0));
        bs_sleep_us(20);
        BS_TRY(mt_rf_read(dev, MT_TOP_PORT, 0xb00, &strap));
        if (strap & 0x10000)
            return BIRRIASDR_ERROR_STATE;
    }
    return 0;
}

/* Bring up the active chain before preparing the companion synthesizer bank.
 * These stages deliberately retain the validated Python ordering; several fields
 * are action strobes and a sorted or deduplicated register table is not equivalent.
 */
static int initialize_chains(birriasdr_dev_t *dev)
{
    BS_TRY(configure_bias(dev, MT_RX_PORT));
    BS_TRY(mt_rf_update(dev, 1, 0xc38, 0, 3));
    BS_TRY(mt_rf_update(dev, 1, 0x51c, 0xc0000, 0x40000));
    for (int p = 1; p >= 0; --p) {
        BS_TRY(mt_rf_write(dev, (unsigned)p, 0x1e4, 0x23c1));
        BS_TRY(mt_rf_update(dev, (unsigned)p, 0x1e8, 0, 0x80000000));
        BS_TRY(mt_rf_update(dev, (unsigned)p, 0x204, 0xff000000, 0x82000000));
    }
    BS_TRY(mt_rf_write(dev, 1, 0x1e4, 0x3fbcb));
    BS_TRY(mt_tune_2g(dev, 1, false, MT_RF_ANCHOR_HZ, 18));
    BS_TRY(configure_bias(dev, 0));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x83090004, 0, 0x3840));
    BS_TRY(bs_update32(dev, BS_MMIO, 0x83090008, 0, 0x3840));
    for (unsigned p = 0; p < 2; ++p) {
        BS_TRY(mt_rf_write(dev, p, 0x1e4, 0x3fbcb));
        BS_TRY(mt_rf_update(dev, p, 0x204, 0xff000000, 0x600));
        BS_TRY(mt_rf_update(dev, p, 0xc38, 0, 3));
        BS_TRY(mt_rf_update(dev, p, 0x0ec, 0, 0x10000000));
        BS_TRY(mt_rf_write(dev, p, 0x638, 0x20410108));
        BS_TRY(mt_rf_write(dev, p, 0x63c, 0x08102081));
    }
    BS_TRY(mt_tune_2g(dev, 0, false, MT_RF_ANCHOR_HZ, 18));
    return bs_update32(dev, BS_MMIO, 0x830ad448, 0xf00, 0x400);
}

/* Final receive routing uses chain 0 (physical SPI 1), but its LO comes from
 * physical SPI 0's secondary synthesizer. Keep that idle bank's 1E4[7:6] set:
 * applying the active bank's value to both reproduced the weak-input fault.
 * RF 160[31:30]=0 preserves the bootstrap; mt_configure_filter() widens it later.
 */
static int select_receive_path(birriasdr_dev_t *dev)
{
    static const struct rf_word companion[] = {
        {0xc54, 0x81818}, {0xc60, 0x82608}, {0xc68, 0x86f00000},
        {0xc6c, 0x6811}, {0xc70, 0x014d3027}, {0xc78, 0x37370}
    };
    static const struct rf_word receive_pairs[] = {
        {0x73c, 0x00aac028}, {0x740, 0x02558000},
        {0x798, 0x00aac028}, {0x79c, 0x02558000}
    };
    BS_TRY(mt_tune_2g(dev, 0, true, MT_RF_ANCHOR_HZ, 18));
    for (unsigned p = 0; p < 2; ++p) {
        BS_TRY(mt_rf_update(dev, p, 0x204, 0x00c00000, 0x00c00000));
        BS_TRY(mt_rf_update(dev, p, 0x21c, 0x00060000, 0x00060000));
        BS_TRY(mt_rf_update(dev, p, 0xc38, 3, 0));
        BS_TRY(mt_rf_update(dev, p, 0x240, 0xf0000000, p == 1 ? 0xa0000000 : 0));
        BS_TRY(mt_rf_update(dev, p, 0x950, 0x60000000, p == 1 ? 0x20000000 : 0x60000000));
        BS_TRY(mt_rf_update(dev, p, 0x51c, 0xc0000, 0));
        BS_TRY(mt_rf_write(dev, p, 0x1e4, p == 1 ? 0x3fb0b : 0x23c1));
    }
    BS_TRY(rf_words(dev, 0, companion, sizeof(companion) / sizeof(companion[0])));
    BS_TRY(mt_tune_2g(dev, 0, true, MT_RF_ANCHOR_HZ, MT_RF_FRACTION_BITS));
    BS_TRY(mt_rf_update(dev, 0, 0x240, 0xf0000000, 0));
    for (unsigned p = 0; p < 2; ++p) {
        BS_TRY(mt_rf_update(dev, p, 0x204, 0x600, 0));
        BS_TRY(rf_words(dev, p, receive_pairs, sizeof(receive_pairs) / sizeof(receive_pairs[0])));
        BS_TRY(mt_rf_update(dev, p, 0x160, 0xc0000000, 0));
    }
    return bs_update32(dev, BS_MMIO, 0x830ad448, 0xf00, 0x400);
}

/* Validate the die/crystal, initialize the established channel-6 clock/bias
 * profile, then retune only the selected secondary oscillator when requested.
 * Keeping the anchor sequence intact avoids guessing new companion-oscillator
 * or filter settings. Normal tuning uses all 20 fractional bits.
 */
int mt_configure_rf(birriasdr_dev_t *dev)
{
    uint32_t analog, crystal;
    if (!mt_frequency_supported(dev->frequency) || dev->rate != 40000000)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(mt_rf_read(dev, MT_TOP_PORT, 0x2c, &analog));
    BS_TRY(bs_read32(dev, BS_MMIO, 0x02004600, &crystal));
    if (analog != 0x79728a10 || crystal != 40000000)
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(configure_clocks(dev));
    BS_TRY(initialize_chains(dev));
    BS_TRY(select_receive_path(dev));
    /* Bias tables and the anchor tune have put every band field back to 2.4 GHz. */
    dev->mt7921.high_band = false;
    return dev->frequency == MT_RF_ANCHOR_HZ ? 0 : mt_set_frequency(dev, dev->frequency);
}
