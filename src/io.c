/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include <errno.h>
#include <time.h>

/* Explicit byte assembly keeps protocol access safe on unaligned/big-endian hosts. */
uint32_t bs_get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* USB register and mailbox words are always little-endian. */
void bs_put_le32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
        p[i] = (uint8_t)(value >> (i * 8));
}

/* MCU command lengths use 16-bit little-endian fields. */
void bs_put_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

/* Hardware deadlines use a monotonic clock so wall-clock adjustments are harmless. */
uint64_t bs_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}

/* Preserve settling delays when a process receives a signal. */
void bs_sleep_us(unsigned us)
{
    struct timespec t = {(time_t)(us / 1000000u), (long)(us % 1000000u) * 1000};
    while (nanosleep(&t, &t) < 0 && errno == EINTR) {}
}

/* Dispatch one register read through the selected transport/test harness. */
int bs_read32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t *value)
{
    return dev->io.read32(dev->io.context, space, address, value);
}

/* Never retry a failed write here: some registers trigger actions. The USB
 * transport resends only a control transfer the device did not answer.
 */
int bs_write32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t value)
{
    return dev->io.write32(dev->io.context, space, address, value);
}

/* Preserve unrelated register bits. set may include bits outside clear, matching
 * the decoded Python driver semantics; this is also used for pure set operations.
 */
int bs_update32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
                uint32_t clear, uint32_t set)
{
    uint32_t value;
    BS_TRY(bs_read32(dev, space, address, &value));
    return bs_write32(dev, space, address, (value & ~clear) | set);
}

/* Every busy/done handshake is bounded. Returning a timeout aborts the outer
 * setup and leads to reset instead of continuing with a half-configured radio.
 */
int bs_poll32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
              uint32_t mask, uint32_t expected, unsigned timeout_ms)
{
    uint64_t deadline = bs_now_ns() + (uint64_t)timeout_ms * 1000000u;
    do {
        uint32_t value;
        BS_TRY(bs_read32(dev, space, address, &value));
        if ((value & mask) == expected)
            return 0;
        bs_sleep_us(100);
    } while (bs_now_ns() < deadline);
    return BIRRIASDR_ERROR_TIMEOUT;
}
