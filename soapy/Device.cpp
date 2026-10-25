/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#include "SoapyBirria.hpp"
#include <SoapySDR/Logger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <stdexcept>

static void check(int result, const char *what)
{
    if (result != 0)
        throw std::runtime_error(std::string("birriasdr: ") + what + " failed (" + std::to_string(result) + ")");
}

/* Setters called while an application runs log a refusal instead of throwing:
 * some front ends (SDR++) do not catch exceptions from these calls and abort.
 */
static bool report(int result, const char *what)
{
    if (result != 0)
        SoapySDR::logf(SOAPY_SDR_ERROR, "birria: %s failed: %s (%d)", what, birriasdr_strerror(result), result);
    return result == 0;
}

/* bandN_min / bandN_max (Hz) replace one band's bounds, N indexing the library's
 * band list. Use them for a board measured to tune beyond its table entry.
 */
SoapyBirria::SoapyBirria(uint32_t index, const SoapySDR::Kwargs &args)
{
    check(birriasdr_open(&dev_, index), "open");
    const int bands = birriasdr_get_tuning_ranges(dev_, nullptr, 0);
    for (int band = 0; band < bands; ++band) {
        const std::string prefix = "band" + std::to_string(band);
        const auto lo = args.find(prefix + "_min"), hi = args.find(prefix + "_max");
        if (lo == args.end() && hi == args.end())
            continue;
        const uint64_t minimum = lo == args.end() ? 0 : static_cast<uint64_t>(std::stod(lo->second));
        const uint64_t maximum = hi == args.end() ? 0 : static_cast<uint64_t>(std::stod(hi->second));
        const int result = birriasdr_set_tuning_range(dev_, band, minimum, maximum);
        if (result != 0) {
            birriasdr_close(dev_);
            check(result, "set_tuning_range");
        }
    }
    SoapySDR::logf(SOAPY_SDR_INFO, "birria: opened device %u", index);
}

SoapyBirria::~SoapyBirria()
{
    if (dev_)
        birriasdr_close(dev_);
}

SoapySDR::Kwargs SoapyBirria::getHardwareInfo() const
{
    SoapySDR::Kwargs info;
    info["note"] = "experimental; raw gain codes, not dB; live retuning stays within one band";
    return info;
}

size_t SoapyBirria::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? 1 : 0;
}

/* Raw hardware codes (the library has no calibrated dB gain yet). */
std::vector<std::string> SoapyBirria::listGains(const int, const size_t) const
{
    return {"LNA", "LPF", "DIG"};
}

SoapySDR::Range SoapyBirria::getGainRange(const int, const size_t, const std::string &name) const
{
    if (name == "LNA") return SoapySDR::Range(0, 3, 1);
    if (name == "LPF") return SoapySDR::Range(0, 8, 1);
    if (name == "DIG") return SoapySDR::Range(0, 12, 1);
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* The library starts in automatic mode and accepts codes only in manual mode,
 * so codes set during AGC are kept and applied when AGC is turned off.
 */
bool SoapyBirria::hasGainMode(const int, const size_t) const
{
    return true;
}

void SoapyBirria::setGainMode(const int, const size_t, const bool automatic)
{
    if (report(birriasdr_set_tuner_gain_mode(dev_, automatic ? 0 : 1), "set_tuner_gain_mode") && !automatic)
        report(birriasdr_set_gain_codes(dev_, &gain_), "set_gain_codes");
}

bool SoapyBirria::getGainMode(const int, const size_t) const
{
    birriasdr_gain gain;
    int manual = 0;
    return birriasdr_get_gain_codes(dev_, &gain, &manual) != 0 || !manual;
}

void SoapyBirria::setGain(const int, const size_t, const std::string &name, const double value)
{
    const auto range = getGainRange(SOAPY_SDR_RX, 0, name);
    const uint8_t code = static_cast<uint8_t>(std::lround(std::clamp(value, range.minimum(), range.maximum())));
    if (name == "LNA") gain_.lna = code;
    else if (name == "LPF") gain_.lpf = code;
    else gain_.digital = code;
    if (!getGainMode(SOAPY_SDR_RX, 0))
        report(birriasdr_set_gain_codes(dev_, &gain_), "set_gain_codes");
}

double SoapyBirria::getGain(const int, const size_t, const std::string &name) const
{
    if (name == "LNA") return gain_.lna;
    if (name == "LPF") return gain_.lpf;
    if (name == "DIG") return gain_.digital;
    throw std::runtime_error("birria: unknown gain element " + name);
}

void SoapyBirria::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &)
{
    report(birriasdr_set_center_freq(dev_, static_cast<uint64_t>(std::llround(frequency))), "set_center_freq");
}

double SoapyBirria::getFrequency(const int, const size_t, const std::string &) const
{
    return static_cast<double>(birriasdr_get_center_freq(dev_));
}

std::vector<std::string> SoapyBirria::listFrequencies(const int, const size_t) const
{
    return {"RF"};
}

SoapySDR::RangeList SoapyBirria::getFrequencyRange(const int, const size_t, const std::string &) const
{
    std::vector<birriasdr_tuning_range> bands(4);
    int count = birriasdr_get_tuning_ranges(dev_, bands.data(), bands.size());
    if (count > static_cast<int>(bands.size())) {
        bands.resize(count);
        count = birriasdr_get_tuning_ranges(dev_, bands.data(), bands.size());
    }
    if (count < 0)
        check(count, "get_tuning_ranges");
    SoapySDR::RangeList ranges;
    for (int i = 0; i < count; ++i)
        ranges.emplace_back(static_cast<double>(bands[i].minimum_hz), static_cast<double>(bands[i].maximum_hz));
    return ranges;
}

/* The hardware rate is fixed at 40 MSps. Lower rates are 40 MSps / N, decimated here. */
static const unsigned kFactors[] = {1, 2, 4, 5, 8, 10, 16, 20, 40};

void SoapyBirria::setSampleRate(const int, const size_t, const double rate)
{
    if (running_) {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: cannot change the sample rate while streaming");
        return;
    }
    for (unsigned f : kFactors) {
        if (std::fabs(rate - kBaseRate / f) < 1.0) {
            decim_ = f;
            return;
        }
    }
    SoapySDR::log(SOAPY_SDR_ERROR, "birria: sample rate must be 40 MSps divided by 1, 2, 4, 5, 8, 10, 16, 20 or 40");
}

double SoapyBirria::getSampleRate(const int, const size_t) const
{
    return kBaseRate / decim_;
}

std::vector<double> SoapyBirria::listSampleRates(const int, const size_t) const
{
    std::vector<double> rates;
    for (auto it = std::rbegin(kFactors); it != std::rend(kFactors); ++it)
        rates.push_back(kBaseRate / *it);
    return rates;
}

std::vector<std::string> SoapyBirria::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16};
}

std::string SoapyBirria::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    fullScale = 8192.0; /* signed 14-bit samples carried in int16 */
    return SOAPY_SDR_CS16;
}

/* Windowed-sinc low-pass, Blackman, cutoff at 0.45 of the output rate. */
void SoapyBirria::buildFilter()
{
    taps_.clear();
    if (decim_ == 1)
        return;
    const size_t n = 16 * decim_ + 1;
    const double fc = 0.45 / decim_; /* cycles per input sample */
    const double mid = (n - 1) / 2.0;
    double sum = 0;
    for (size_t k = 0; k < n; ++k) {
        const double x = k - mid;
        const double s = x == 0 ? 2 * fc : std::sin(2 * M_PI * fc * x) / (M_PI * x);
        const double w = 0.42 - 0.5 * std::cos(2 * M_PI * k / (n - 1)) + 0.08 * std::cos(4 * M_PI * k / (n - 1));
        taps_.push_back(static_cast<float>(s * w));
        sum += s * w;
    }
    for (auto &t : taps_)
        t = static_cast<float>(t / sum);
}

SoapySDR::Stream *SoapyBirria::setupStream(const int direction, const std::string &format,
                                           const std::vector<size_t> &, const SoapySDR::Kwargs &)
{
    if (direction != SOAPY_SDR_RX || (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32))
        throw std::runtime_error("birria: only RX CF32 and CS16 streams are supported");
    cs16_ = format == SOAPY_SDR_CS16;
    streamOpen_ = true;
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

void SoapyBirria::closeStream(SoapySDR::Stream *)
{
    deactivateStream(nullptr);
    streamOpen_ = false;
}

void SoapyBirria::asyncCallback(const int16_t *iq, uint32_t count, uint64_t, void *ctx)
{
    static_cast<SoapyBirria *>(ctx)->onSamples(iq, count);
}

/* Runs on the USB thread: copy and return. Never block here or the device SRAM overruns. */
void SoapyBirria::onSamples(const int16_t *iq, uint32_t count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_ + count > kMaxQueuedSamples) {
        overflow_ = true;
        return;
    }
    queue_.emplace_back(iq, iq + 2 * static_cast<size_t>(count));
    queued_ += count;
    cond_.notify_one();
}

int SoapyBirria::activateStream(SoapySDR::Stream *, const int, const long long, const size_t)
{
    if (running_)
        return 0;
    buildFilter();
    in_.assign(taps_.empty() ? 0 : 2 * (taps_.size() - 1), 0.0f);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
        overflow_ = false;
    }
    workerResult_ = 0;
    running_ = true;
    worker_ = std::thread([this] {
        const int rc = birriasdr_read_async(dev_, &SoapyBirria::asyncCallback, this, 0, 0);
        workerResult_ = rc;
        running_ = false;
        cond_.notify_all();
    });
    return 0;
}

int SoapyBirria::deactivateStream(SoapySDR::Stream *, const int, const long long)
{
    if (worker_.joinable()) {
        birriasdr_cancel_async(dev_);
        worker_.join();
    }
    running_ = false;
    return 0;
}

int SoapyBirria::readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems, int &flags,
                            long long &timeNs, const long timeoutUs)
{
    flags = 0;
    timeNs = 0;
    const size_t history = taps_.empty() ? 0 : taps_.size() - 1;
    const size_t need = history + numElems * decim_; /* complex samples */
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);

    while (in_.size() / 2 < need) {
        std::vector<int16_t> chunk;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty() && !overflow_) {
                if (!running_)
                    return workerResult_ < 0 ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_NOT_SUPPORTED;
                if (!cond_.wait_until(lock, deadline, [this] { return !queue_.empty() || overflow_ || !running_; }))
                    return SOAPY_SDR_TIMEOUT;
                if (queue_.empty() && !overflow_)
                    continue;
            }
            if (overflow_) {
                overflow_ = false;
                queue_.clear();
                queued_ = 0;
                in_.assign(2 * history, 0.0f); /* restart the filter after a gap */
                return SOAPY_SDR_OVERFLOW;
            }
            chunk = std::move(queue_.front());
            queue_.pop_front();
            queued_ -= chunk.size() / 2;
        }
        const size_t old = in_.size();
        in_.resize(old + chunk.size());
        for (size_t i = 0; i < chunk.size(); ++i)
            in_[old + i] = chunk[i];
    }

    const size_t outCount = std::min(numElems, (in_.size() / 2 - history) / decim_);
    constexpr float kScale = 1.0f / 8192.0f;
    float *outF = static_cast<float *>(buffs[0]);
    int16_t *outS = static_cast<int16_t *>(buffs[0]);
    for (size_t o = 0; o < outCount; ++o) {
        float ar, ai;
        if (taps_.empty()) {
            ar = in_[2 * o];
            ai = in_[2 * o + 1];
        } else {
            const float *p = &in_[2 * o * decim_];
            ar = ai = 0;
            for (size_t k = 0; k < taps_.size(); ++k) {
                ar += taps_[k] * p[2 * k];
                ai += taps_[k] * p[2 * k + 1];
            }
        }
        if (cs16_) {
            outS[2 * o] = static_cast<int16_t>(std::lrintf(ar));
            outS[2 * o + 1] = static_cast<int16_t>(std::lrintf(ai));
        } else {
            outF[2 * o] = ar * kScale;
            outF[2 * o + 1] = ai * kScale;
        }
    }
    in_.erase(in_.begin(), in_.begin() + 2 * outCount * decim_);
    return static_cast<int>(outCount);
}
