/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 *
 * test_stream <freq_hz> <rate> <seconds> [out.cf32]: capture through the Soapy
 * module and report the averaged spectrum peak, to check decimation and the
 * frequency axis. Not built by CMake; see docs/soapy.md for the build command.
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <vector>
int main(int argc, char **argv)
{
    if (argc < 4) return 2;
    const double freq = atof(argv[1]), rate = atof(argv[2]), secs = atof(argv[3]);
    auto *dev = SoapySDR::Device::make("driver=birria");
    dev->setSampleRate(SOAPY_SDR_RX, 0, rate);
    dev->setFrequency(SOAPY_SDR_RX, 0, freq);
    auto *st = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32);
    dev->activateStream(st);
    const int N = 1024;
    std::vector<std::complex<float>> buf(65536), acc;
    std::vector<double> psd(N, 0.0);
    size_t total = 0, frames = 0, overflows = 0, errs = 0;
    std::vector<std::complex<float>> pend;
    FILE *out = argc > 4 ? fopen(argv[4], "wb") : nullptr;
    while (total < rate * secs) {
        void *b[] = {buf.data()};
        int flags = 0; long long t = 0;
        int n = dev->readStream(st, b, buf.size(), flags, t, 200000);
        if (n == SOAPY_SDR_OVERFLOW) { overflows++; continue; }
        if (n < 0) { if (++errs > 20) break; continue; }
        total += n;
        if (out) fwrite(buf.data(), 8, n, out);
        pend.insert(pend.end(), buf.begin(), buf.begin() + n);
        while (pend.size() >= N) {
            // radix-2 FFT of one Hann-windowed frame
            std::vector<std::complex<double>> x(N);
            for (int k = 0; k < N; k++) x[k] = std::complex<double>(pend[k]) * (0.5 - 0.5 * cos(2 * M_PI * k / (N - 1)));
            for (int i = 1, j = 0; i < N; i++) { int bit = N >> 1; for (; j & bit; bit >>= 1) j ^= bit; j ^= bit; if (i < j) std::swap(x[i], x[j]); }
            for (int len = 2; len <= N; len <<= 1) { double a = -2 * M_PI / len; std::complex<double> wl(cos(a), sin(a));
                for (int i = 0; i < N; i += len) { std::complex<double> w(1); for (int j = 0; j < len / 2; j++) { auto u = x[i + j], v = x[i + j + len / 2] * w; x[i + j] = u + v; x[i + j + len / 2] = u - v; w *= wl; } } }
            for (int k = 0; k < N; k++) psd[(k + N / 2) % N] += std::norm(x[k]);
            frames++;
            pend.erase(pend.begin(), pend.begin() + N);
        }
    }
    dev->deactivateStream(st);
    dev->closeStream(st);
    int best = 0; double floor = 0;
    for (int k = 0; k < N; k++) { if (psd[k] > psd[best]) best = k; floor += psd[k]; }
    floor /= N;
    printf("samples %zu overflows %zu frames %zu\n", total, overflows, frames);
    printf("peak at %+.2f MHz, %.1f dB over mean; edge bins %.1f / %.1f dB vs mean\n",
           (best - N / 2) * rate / N / 1e6, 10 * log10(psd[best] / floor),
           10 * log10(psd[8] / floor), 10 * log10(psd[N - 8] / floor));
    SoapySDR::Device::unmake(dev);
}
