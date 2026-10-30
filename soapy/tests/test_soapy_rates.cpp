/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/* Test the module's sample rates without hardware. Device.cpp is built
 * against stub library functions that record the rate the module requests.
 * The module must list exactly the native rates, pass each one through
 * unchanged, forward a request within 1 Hz as the exact native rate and refuse
 * any other value before it reaches the library. A rate change while
 * streaming must restart the read with the library idle, report one overflow
 * at the change and never deliver old-rate samples after it.
 */
#include "../SoapyBirria.hpp"
#include <SoapySDR/Formats.h>
#include <SoapySDR/Logger.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#define CHECK(test)                                                                      \
    do {                                                                                 \
        if (!(test)) {                                                                   \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test);                   \
            exit(1);                                                                     \
        }                                                                                \
    } while (0)

static std::atomic<uint32_t> stub_rate{40000000};
static unsigned stub_rate_calls;
static std::atomic<bool> stub_reading{false}, stub_cancel{false};
static std::atomic<unsigned> stub_reads{0}, stub_busy_rate_calls{0};

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
    if (stub_reading) {
        ++stub_busy_rate_calls; /* the real library refuses this */
        return BIRRIASDR_ERROR_BUSY;
    }
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
/* Each sample's I component is the rate in MSps, so a reader can tell which
 * rate produced it.
 */
int birriasdr_read_async(birriasdr_dev_t *, birriasdr_read_async_cb_t callback, void *context,
                         uint32_t, uint32_t)
{
    static int16_t iq[2 * 1024];
    ++stub_reads;
    stub_cancel = false;
    stub_reading = true;
    const int16_t tag = static_cast<int16_t>(stub_rate / 1000000);
    for (unsigned i = 0; i < 1024; ++i) {
        iq[2 * i] = tag;
        iq[2 * i + 1] = 0;
    }
    while (!stub_cancel) {
        callback(iq, 1024, 0, context);
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    stub_reading = false;
    return 0;
}
int birriasdr_cancel_async(birriasdr_dev_t *)
{
    if (!stub_reading)
        return BIRRIASDR_ERROR_STATE;
    stub_cancel = true;
    return 0;
}
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

    /* A request within 1 Hz of a native rate reaches the library as that exact
     * rate, never rounded to a neighboring value.
     */
    dev.setSampleRate(SOAPY_SDR_RX, 0, 10000000.75);
    CHECK(stub_rate == 10000000);
    dev.setSampleRate(SOAPY_SDR_RX, 0, 79999999.5);
    CHECK(stub_rate == 80000000);

    /* Refused values never reach the library, and the rate stays at 80 MSps. */
    const unsigned calls = stub_rate_calls;
    for (double rate : {5e6, 60e6, 160e6, 4334967296.0, -40e6, 10000001.0, std::nan("")})
        dev.setSampleRate(SOAPY_SDR_RX, 0, rate);
    CHECK(stub_rate_calls == calls);
    CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == 80e6);

    /* Change the rate while a reader streams on another thread. */
    dev.setSampleRate(SOAPY_SDR_RX, 0, 40e6);
    auto *stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16);
    CHECK(dev.activateStream(stream) == 0);
    std::atomic<bool> stop{false};
    std::string events;
    std::thread reader([&] {
        static int16_t buf[2 * 4096];
        void *buffs[] = {buf};
        while (!stop) {
            int flags = 0;
            long long time = 0;
            const int n = dev.readStream(stream, buffs, 4096, flags, time, 100000);
            if (n == SOAPY_SDR_OVERFLOW)
                events += 'O';
            else if (n == SOAPY_SDR_STREAM_ERROR)
                events += 'E';
            else if (n > 0)
                for (int i = 0; i < n; ++i)
                    if (events.empty() || events.back() != (buf[2 * i] == 40 ? 'a' : 'b'))
                        events += buf[2 * i] == 40 ? 'a' : 'b';
        }
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dev.setSampleRate(SOAPY_SDR_RX, 0, 20e6);
    CHECK(dev.getSampleRate(SOAPY_SDR_RX, 0) == 20e6);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const unsigned reads = stub_reads;
    dev.setSampleRate(SOAPY_SDR_RX, 0, 20e6); /* same rate: no restart */
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    stop = true;
    reader.join();
    CHECK(dev.deactivateStream(stream) == 0);
    dev.closeStream(stream);
    /* 40 MSps samples, one overflow at the change, then only 20 MSps samples. */
    if (events != "aOb")
        fprintf(stderr, "events: %s\n", events.c_str());
    CHECK(events == "aOb");
    CHECK(stub_rate == 20000000 && stub_reads == 2 && reads == 2);
    CHECK(stub_busy_rate_calls == 0);

    puts("SoapySDR native sample rate tests passed");
    return 0;
}
