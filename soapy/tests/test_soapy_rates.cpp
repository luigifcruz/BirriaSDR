/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Test the module's sample rates without hardware. Device.cpp is built
 * against stub library functions that record the rate the module requests.
 * The module must list exactly the native rates, pass each one through
 * unchanged and refuse any other value before it reaches the library.
 */
#include "../SoapyBirria.hpp"
#include <SoapySDR/Logger.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define CHECK(test)                                                                      \
    do {                                                                                 \
        if (!(test)) {                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test);                   \
            exit(1);                                                                     \
        }                                                                                \
    } while (0)

static uint32_t stub_rate = 40000000;
static unsigned stub_rate_calls;

/* Library stubs: only the rate setter and getter keep state. */
extern "C" {
const char *birriasdr_strerror(int error) { return error ? "error" : "success"; }
int birriasdr_open(birriasdr_dev_t **dev, uint32_t)
{
    *dev = reinterpret_cast<birriasdr_dev_t *>(&stub_rate);
    return 0;
}
int birriasdr_close(birriasdr_dev_t *) { return 0; }
int birriasdr_set_recovery(birriasdr_dev_t *, uint32_t, birriasdr_recovery_cb_t, void *)
{
    return 0;
}
int birriasdr_set_sample_rate(birriasdr_dev_t *, uint32_t rate)
{
    ++stub_rate_calls;
    stub_rate = rate;
    return 0;
}
uint32_t birriasdr_get_sample_rate(const birriasdr_dev_t *) { return stub_rate; }
int birriasdr_set_tuner_gain_mode(birriasdr_dev_t *, int) { return 0; }
int birriasdr_set_gain_codes(birriasdr_dev_t *, const struct birriasdr_gain *) { return 0; }
int birriasdr_get_gain_codes(birriasdr_dev_t *, struct birriasdr_gain *gain, int *manual)
{
    *gain = birriasdr_gain{};
    *manual = 0;
    return 0;
}
int birriasdr_set_center_freq(birriasdr_dev_t *, uint64_t) { return 0; }
uint64_t birriasdr_get_center_freq(const birriasdr_dev_t *) { return 2437000000u; }
int birriasdr_get_tuning_band(const birriasdr_dev_t *, uint32_t, uint64_t *, uint64_t *)
{
    return BIRRIASDR_ERROR_INVALID;
}
int birriasdr_read_async(birriasdr_dev_t *, birriasdr_read_async_cb_t, void *, uint32_t, uint32_t)
{
    return 0;
}
int birriasdr_cancel_async(birriasdr_dev_t *) { return BIRRIASDR_ERROR_STATE; }
}

int main()
{
    static const double native[] = {10e6, 20e6, 40e6, 80e6};
    SoapySDR_setLogLevel(SOAPY_SDR_CRITICAL);
    SoapyBirria dev(0, {});

    const auto listed = dev.listSampleRates(SOAPY_SDR_RX, 0);
    CHECK(listed.size() == 4);
    const auto ranges = dev.getSampleRateRange(SOAPY_SDR_RX, 0);
    CHECK(ranges.size() == 4);
    for (unsigned i = 0; i < 4; ++i) {
        CHECK(listed[i] == native[i]);
        CHECK(ranges[i].minimum() == native[i] && ranges[i].maximum() == native[i]);
    }

    for (double rate : native) {
        dev.setSampleRate(SOAPY_SDR_RX, 0, rate);
        CHECK(stub_rate == static_cast<uint32_t>(rate));
        CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == rate);
    }

    /* Refused values never reach the library, and the rate stays at 80 MSps. */
    const unsigned calls = stub_rate_calls;
    for (double rate : {5e6, 60e6, 160e6, 4334967296.0, -40e6, std::nan("")})
        dev.setSampleRate(SOAPY_SDR_RX, 0, rate);
    CHECK(stub_rate_calls == calls);
    CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == 80e6);

    puts("SoapySDR native sample rate tests passed");
    return 0;
}
