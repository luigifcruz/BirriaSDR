/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#pragma once
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.h>
#include <birriasdr.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

/* Skeleton SoapySDR driver for libbirriasdr. Settings that the library supports
 * are wired up; RX streams run birriasdr_read_async on a worker thread into a ring and
 * decimate by an integer factor on the reader side (the hardware rate is fixed at 40 MSps).
 */
class SoapyBirria : public SoapySDR::Device
{
public:
    SoapyBirria(uint32_t index, const SoapySDR::Kwargs &args);
    ~SoapyBirria() override;

    std::string getDriverKey() const override { return "birria"; }
    std::string getHardwareKey() const override { return "MT7921"; }
    SoapySDR::Kwargs getHardwareInfo() const override;

    size_t getNumChannels(const int direction) const override;

    std::vector<std::string> listGains(const int direction, const size_t channel) const override;
    SoapySDR::Range getGainRange(const int direction, const size_t channel,
                                 const std::string &name) const override;
    bool hasGainMode(const int direction, const size_t channel) const override;
    void setGainMode(const int direction, const size_t channel, const bool automatic) override;
    bool getGainMode(const int direction, const size_t channel) const override;
    void setGain(const int direction, const size_t channel, const std::string &name,
                 const double value) override;
    double getGain(const int direction, const size_t channel, const std::string &name) const override;

    void setFrequency(const int direction, const size_t channel, const std::string &name,
                      const double frequency, const SoapySDR::Kwargs &args) override;
    double getFrequency(const int direction, const size_t channel, const std::string &name) const override;
    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const override;
    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel,
                                          const std::string &name) const override;

    void setSampleRate(const int direction, const size_t channel, const double rate) override;
    double getSampleRate(const int direction, const size_t channel) const override;
    std::vector<double> listSampleRates(const int direction, const size_t channel) const override;

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const override;
    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const override;

    SoapySDR::Stream *setupStream(const int direction, const std::string &format,
                                  const std::vector<size_t> &channels = std::vector<size_t>(),
                                  const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) override;
    void closeStream(SoapySDR::Stream *stream) override;
    int activateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0,
                       const size_t numElems = 0) override;
    int deactivateStream(SoapySDR::Stream *stream, const int flags = 0, const long long timeNs = 0) override;
    int readStream(SoapySDR::Stream *stream, void *const *buffs, const size_t numElems, int &flags,
                   long long &timeNs, const long timeoutUs = 100000) override;

private:
    static void asyncCallback(const int16_t *iq, uint32_t count, uint64_t first, void *ctx);
    void onSamples(const int16_t *iq, uint32_t count);
    void buildFilter();

    static constexpr double kBaseRate = 40e6;
    static constexpr size_t kMaxQueuedSamples = 16u << 20; /* complex samples, ~64 MiB */

    birriasdr_dev_t *dev_ = nullptr;
    birriasdr_gain gain_{2, 8, 0};
    bool streamOpen_ = false;
    bool cs16_ = false;
    unsigned decim_ = 8; /* default 5 MSps */

    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<int> workerResult_{0};
    std::mutex mutex_;
    std::condition_variable cond_;
    std::deque<std::vector<int16_t>> queue_;
    size_t queued_ = 0;
    bool overflow_ = false;

    std::vector<float> taps_;
    std::vector<float> in_; /* history followed by new samples, interleaved I,Q floats */
};
