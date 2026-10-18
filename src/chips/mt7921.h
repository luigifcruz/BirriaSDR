/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 * Private MT7961/MT7972 interface. Do not share these registers with MT7925.
 */

/*
 * Internal contract between the MT7961 boot, RF, gain and capture modules.
 *
 * MT_ENTRY, MT_MAILBOX and the challenge constants must agree with the original
 * program in firmware/iq_stream.S. They describe RAM and a boot handshake,
 * not persistent device configuration. Physical RF SPI ports are named here to
 * avoid confusing receive chain 0 (port 1) with its companion synth (port 0).
 *
 * The caller owns the USB interface. Prepare configures hardware while the MCU
 * waits for GO; start follows queued USB reads; stop requests DMA drain. Reset
 * restores saved gain and filter fields before resetting WFSYS. RF access must not
 * race an active RF controller. These helpers are private so the public API
 * can enforce that lifecycle rather than expose uncoordinated register writes.
 * File-level comments in each implementation explain the hardware mechanisms
 * and the limits of the measurements supporting them.
 */
#ifndef BIRRIASDR_MT7921_H
#define BIRRIASDR_MT7921_H
#include "internal.h"

#define MT_MAILBOX 0x02038360u
#define MT_BOOT_MAGIC 0x41495242u
#define MT_RESPONSE_XOR 0xb1771a5du
#define MT_ENTRY 0x00915000u
#define MT_TOP_PORT 5u
#define MT_RX_PORT 1u
#define MT_SYNTH_PORT 0u
#define MT_RF_ANCHOR_HZ UINT64_C(2437000000)
/* Command limits inside the observed calibration transitions, not a
 * calibrated passband or a guarantee of sensitivity at every frequency.
 */
#define MT_RF_MIN_HZ UINT64_C(2156000000)
#define MT_RF_MAX_HZ UINT64_C(2610000000)
/* 5 GHz command limits inside the observed calibration rails near 4.3 and
 * 6.31 GHz. Reception has been verified only from 5250 to 5815 MHz.
 */
#define MT_RF_5G_MIN_HZ UINT64_C(4500000000)
#define MT_RF_5G_MAX_HZ UINT64_C(6200000000)
#define MT_RF_FRACTION_BITS 20u

/* A mathematical frequency plan, not a claim of analog tuning coverage.
 * nominal_hz_num / nominal_hz_den is exact for the assumed 60 MHz reference.
 * Both bands share this layout; only the VCO multiplier differs.
 */
struct mt_2g_tuning {
    uint32_t word, calibration_cycles;
    uint64_t nominal_hz_num;
    uint32_t nominal_hz_den;
};

int mt_plan_2g(uint64_t frequency_hz, unsigned fraction_bits, struct mt_2g_tuning *plan);
int mt_plan_5g(uint64_t frequency_hz, unsigned fraction_bits, struct mt_2g_tuning *plan);
bool mt_frequency_supported(uint64_t frequency_hz);
int mt_tune_2g(birriasdr_dev_t *dev, unsigned port, bool secondary,
               uint64_t frequency_hz, unsigned fraction_bits);
int mt_retune_2g_probe(birriasdr_dev_t *dev, uint64_t frequency_hz, unsigned fraction_bits);
int mt_set_frequency(birriasdr_dev_t *dev, uint64_t frequency_hz);
int mt_change_frequency(birriasdr_dev_t *dev, uint64_t frequency_hz);

int mt_rf_read(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t *value);
int mt_rf_write(birriasdr_dev_t *dev, unsigned port, uint32_t address, uint32_t value);
int mt_rf_update(birriasdr_dev_t *dev, unsigned port, uint32_t address,
                 uint32_t clear, uint32_t set);
int mt_configure_rf(birriasdr_dev_t *dev);
int mt_configure_filter(birriasdr_dev_t *dev);
int mt_restore_filter(birriasdr_dev_t *dev);
int mt_apply_gain(birriasdr_dev_t *dev);
int mt_change_gain(birriasdr_dev_t *dev, bool manual, const struct birriasdr_gain *gain);
int mt_restore_gain(birriasdr_dev_t *dev);
int mt_configure_capture(birriasdr_dev_t *dev);
int mt_reset(birriasdr_dev_t *dev);
int mt_prepare(birriasdr_dev_t *dev, uint32_t packets);
int mt_start(birriasdr_dev_t *dev);
int mt_stop(birriasdr_dev_t *dev);
int mt_status(birriasdr_dev_t *dev, uint32_t words[16]);
#endif
