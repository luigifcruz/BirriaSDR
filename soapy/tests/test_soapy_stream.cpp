/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 * Explicit hardware test of the SoapySDR module, never run automatically by CTest.
 *
 * test_soapy_stream <freq_hz> <rate> <seconds> [out.cf32]: capture through the
 * module and report the averaged spectrum peak, to check the frequency axis
 * and the sample format.
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <utility>
#include <vector>

static constexpr size_t kFrame = 1024;

/* In-place radix-2 FFT of one frame. */
static void fft(std::vector<std::complex<double>> &x)
{
    const size_t n = x.size();
    for (size_t i = 1, j = 0; i < n; i++) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(x[i], x[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1) {
        const double a = -2 * M_PI / len;
        const std::complex<double> wl(cos(a), sin(a));
        for (size_t i = 0; i < n; i += len) {
            std::complex<double> w(1);
            for (size_t j = 0; j < len / 2; j++) {
                const auto u = x[i + j], v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wl;
            }
        }
    }
}

/* Add the power spectrum of one Hann-windowed frame, with DC moved to the center. */
static void accumulate(const std::complex<float> *frame, std::vector<double> &psd)
{
    std::vector<std::complex<double>> x(kFrame);
    for (size_t k = 0; k < kFrame; k++)
        x[k] = std::complex<double>(frame[k]) * (0.5 - 0.5 * cos(2 * M_PI * k / (kFrame - 1)));
    fft(x);
    for (size_t k = 0; k < kFrame; k++)
        psd[(k + kFrame / 2) % kFrame] += std::norm(x[k]);
}

/* Print why the capture failed, release the device and return failure. */
static int fail(SoapySDR::Device *dev, FILE *out, const char *why)
{
    fprintf(stderr, "test_soapy_stream: %s\n", why);
    if (out)
        fclose(out);
    SoapySDR::Device::unmake(dev);
    return 1;
}

/* Capture, check that the module applied the request and report the averaged
 * spectrum. Any capture or file error returns failure.
 */
int main(int argc, char **argv)
{
    if (argc < 4)
        return 2;
    const double freq = atof(argv[1]), requested = atof(argv[2]), secs = atof(argv[3]);
    auto *dev = SoapySDR::Device::make("driver=birria");
    dev->setSampleRate(SOAPY_SDR_RX, 0, requested);
    dev->setFrequency(SOAPY_SDR_RX, 0, freq);
    const double rate = dev->getSampleRate(SOAPY_SDR_RX, 0);
    if (std::fabs(rate - requested) > 1.0)
        return fail(dev, nullptr, "the module refused the sample rate");
    if (std::fabs(dev->getFrequency(SOAPY_SDR_RX, 0) - freq) > 1.0)
        return fail(dev, nullptr, "the module refused the frequency");
    FILE *out = nullptr;
    if (argc > 4 && !(out = fopen(argv[4], "wb")))
        return fail(dev, nullptr, "cannot open the output file");
    auto *st = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    if (dev->activateStream(st) != 0)
        return fail(dev, out, "cannot activate the stream");

    std::vector<std::complex<float>> buf(65536), pend;
    std::vector<double> psd(kFrame, 0.0);
    size_t total = 0, frames = 0, overflows = 0, errs = 0;
    bool written = true, stalled = false;
    /* Gaps alone do not count as errors, so also stop when no samples arrive. */
    auto progress = std::chrono::steady_clock::now();
    while (total < rate * secs && errs <= 20 && written) {
        if (std::chrono::steady_clock::now() - progress > std::chrono::seconds(5)) {
            stalled = true;
            break;
        }
        void *b[] = {buf.data()};
        int flags = 0;
        long long t = 0;
        const int n = dev->readStream(st, b, buf.size(), flags, t, 200000);
        if (n == SOAPY_SDR_OVERFLOW) {
            overflows++;
            continue;
        }
        if (n < 0) {
            errs++;
            continue;
        }
        total += n;
        if (n > 0)
            progress = std::chrono::steady_clock::now();
        if (out)
            written = fwrite(buf.data(), 8, n, out) == static_cast<size_t>(n);
        pend.insert(pend.end(), buf.begin(), buf.begin() + n);
        while (pend.size() >= kFrame) {
            accumulate(pend.data(), psd);
            frames++;
            pend.erase(pend.begin(), pend.begin() + kFrame);
        }
    }
    dev->deactivateStream(st);
    dev->closeStream(st);
    if (errs > 20)
        return fail(dev, out, "too many read errors");
    if (stalled)
        return fail(dev, out, "no samples arrived for five seconds");
    if (!written)
        return fail(dev, out, "cannot write the output file");
    if (out && fclose(out) != 0)
        return fail(dev, nullptr, "cannot close the output file");
    out = nullptr;

    size_t best = 0;
    double floor = 0;
    for (size_t k = 0; k < kFrame; k++) {
        if (psd[k] > psd[best])
            best = k;
        floor += psd[k];
    }
    floor /= kFrame;
    printf("samples %zu overflows %zu frames %zu\n", total, overflows, frames);
    if (frames == 0 || floor <= 0)
        return fail(dev, nullptr, "no signal captured");
    printf("peak at %+.2f MHz, %.1f dB over mean; edge bins %.1f / %.1f dB vs mean\n",
           (static_cast<double>(best) - kFrame / 2.0) * rate / kFrame / 1e6,
           10 * log10(psd[best] / floor), 10 * log10(psd[8] / floor),
           10 * log10(psd[kFrame - 8] / floor));
    SoapySDR::Device::unmake(dev);
    return 0;
}
