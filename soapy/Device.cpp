/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#include "SoapyBirria.hpp"
#include <SoapySDR/Logger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <shared_mutex>
#include <stdexcept>

/* Construction has no front end to report to, so a failed open throws. */
static void check(int result, const char *what)
{
    if (result != 0)
        throw std::runtime_error(std::string("birriasdr: ") + what + " failed (" +
                                 std::to_string(result) + ")");
}

/* Log an error as best effort. Logging can allocate, and it runs on failure
 * paths where an exception would terminate the process, so it never throws.
 */
static void logError(const char *message) noexcept
{
    try {
        SoapySDR::log(SOAPY_SDR_ERROR, message);
    } catch (...) {
    }
}

/* Setters called while an application runs log a refusal instead of throwing:
 * some front ends do not catch exceptions from these calls and abort.
 */
static bool report(int result, const char *what) noexcept
{
    if (result != 0) {
        try {
            SoapySDR::logf(SOAPY_SDR_ERROR, "birria: %s failed: %s (%d)", what,
                           birriasdr_strerror(result), result);
        } catch (...) {
        }
    }
    return result == 0;
}

/* Open the dongle at its enumeration index. Device arguments are not used.
 * Recovery is configured here because the library accepts it only while idle.
 * It keeps a stream running across overruns and reports each gap.
 */
SoapyBirria::SoapyBirria(uint32_t index, const SoapySDR::Kwargs &)
{
    check(birriasdr_open(&dev_, index), "open");
    const int rc = birriasdr_set_recovery(dev_, kMaxRestarts, &SoapyBirria::recoveryCallback, this);
    if (rc != 0) {
        birriasdr_close(dev_);
        check(rc, "set_recovery");
    }
    rate_ = birriasdr_get_sample_rate(dev_);
    SoapySDR::logf(SOAPY_SDR_INFO, "birria: opened device %u", index);
}

/* Stop any stream first: the library requires the reader to be joined before
 * close, and destroying a joinable thread terminates the process.
 */
SoapyBirria::~SoapyBirria()
{
    deactivateStream(nullptr);
    if (dev_)
        birriasdr_close(dev_);
}

/* Warn front ends that the gains are raw codes. */
SoapySDR::Kwargs SoapyBirria::getHardwareInfo() const
{
    SoapySDR::Kwargs info;
    info["note"] = "experimental; raw gain codes, not dB";
    return info;
}

/* The receiver has one RX channel and no TX path. */
size_t SoapyBirria::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? 1 : 0;
}

/* Raw hardware codes (the library has no calibrated dB gain yet). */
std::vector<std::string> SoapyBirria::listGains(const int, const size_t) const
{
    return {"LNA", "LPF", "DIG"};
}

/* Code limits for each gain element, in steps of one. The library accepts LNA
 * 0 to 5, and 5 GHz needs 4 or 5 for full sensitivity.
 */
SoapySDR::Range SoapyBirria::getGainRange(const int, const size_t, const std::string &name) const
{
    if (name == "LNA")
        return SoapySDR::Range(0, 5, 1);
    if (name == "LPF")
        return SoapySDR::Range(0, 8, 1);
    if (name == "DIG")
        return SoapySDR::Range(0, 12, 1);
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* The library starts in automatic mode and accepts codes only in manual mode,
 * so codes set during AGC are kept and applied when AGC is turned off.
 */
bool SoapyBirria::hasGainMode(const int, const size_t) const
{
    return true;
}

/* Switch modes, then send the stored codes when entering manual mode. The
 * gain lock keeps the codes consistent with concurrent gain calls.
 */
void SoapyBirria::setGainMode(const int, const size_t, const bool automatic)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    if (report(birriasdr_set_tuner_gain_mode(dev_, automatic ? 0 : 1), "set_tuner_gain_mode") &&
        !automatic)
        report(birriasdr_set_gain_codes(dev_, &gain_), "set_gain_codes");
}

/* Report automatic mode unless the library confirms manual mode. */
bool SoapyBirria::getGainMode(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    birriasdr_gain gain;
    int manual = 0;
    return birriasdr_get_gain_codes(dev_, &gain, &manual) != 0 || !manual;
}

/* Round and clamp the code. In automatic mode it is stored for later. In manual
 * mode it replaces only its own element of the library's committed codes, so a
 * stale stored code for another element is never sent. It is stored only if
 * the library accepts it, so a refused change does not look applied.
 */
void SoapyBirria::setGain(const int, const size_t, const std::string &name, const double value)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    const auto range = getGainRange(SOAPY_SDR_RX, 0, name);
    const uint8_t code = static_cast<uint8_t>(
        std::lround(std::clamp(value, range.minimum(), range.maximum())));
    std::lock_guard<std::mutex> lock(gainMutex_);
    birriasdr_gain committed;
    int manual = 0;
    const bool known = birriasdr_get_gain_codes(dev_, &committed, &manual) == 0;
    birriasdr_gain next = known && manual ? committed : gain_;
    if (name == "LNA")
        next.lna = code;
    else if (name == "LPF")
        next.lpf = code;
    else
        next.digital = code;
    if (!(known && manual) || report(birriasdr_set_gain_codes(dev_, &next), "set_gain_codes"))
        gain_ = next;
}

/* Return the code the library committed in manual mode, or the stored code
 * that manual mode will apply while automatic mode is on.
 */
double SoapyBirria::getGain(const int, const size_t, const std::string &name) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    birriasdr_gain gain = gain_;
    birriasdr_gain committed;
    int manual = 0;
    if (birriasdr_get_gain_codes(dev_, &committed, &manual) == 0 && manual)
        gain = committed;
    if (name == "LNA")
        return gain.lna;
    if (name == "LPF")
        return gain.lpf;
    if (name == "DIG")
        return gain.digital;
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* Request a center in Hz. The library rejects it without retuning if invalid. */
void SoapyBirria::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    report(birriasdr_set_center_freq(dev_, static_cast<uint64_t>(std::llround(frequency))),
           "set_center_freq");
}

/* Return the committed center, not the frequency of samples already queued. */
double SoapyBirria::getFrequency(const int, const size_t, const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    return static_cast<double>(birriasdr_get_center_freq(dev_));
}

/* The only tunable element is the RF center. */
std::vector<std::string> SoapyBirria::listFrequencies(const int, const size_t) const
{
    return {"RF"};
}

/* Report one range per contiguous band, so front ends cannot offer the gaps
 * between them. A band this board cannot reach returns UNSUPPORTED and is
 * skipped. INVALID marks the end of the list.
 */
SoapySDR::RangeList SoapyBirria::getFrequencyRange(const int, const size_t,
                                                   const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    SoapySDR::RangeList ranges;
    for (uint32_t index = 0;; ++index) {
        uint64_t minimum = 0, maximum = 0;
        const int rc = birriasdr_get_tuning_band(dev_, index, &minimum, &maximum);
        if (rc == BIRRIASDR_ERROR_INVALID)
            break;
        if (rc == BIRRIASDR_ERROR_UNSUPPORTED)
            continue;
        if (rc != 0) {
            report(rc, "get_tuning_band");
            break;
        }
        ranges.push_back(
            SoapySDR::Range(static_cast<double>(minimum), static_cast<double>(maximum)));
    }
    return ranges;
}

/* Rates the hardware produces without resampling. The library cannot list its
 * rates, so add one here when the backend gains it.
 */
static const double kNativeRates[] = {40e6};

/* Accept only a native rate and pass it to the library. The library requires
 * this setter to run alone on the handle, so it holds the control lock
 * exclusively, which also keeps a stream from starting meanwhile.
 */
void SoapyBirria::setSampleRate(const int, const size_t, const double rate)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (running_) {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: cannot change the sample rate while streaming");
        return;
    }
    /* Check the list before narrowing, so a huge value cannot wrap to a valid rate. */
    if (std::none_of(std::begin(kNativeRates), std::end(kNativeRates),
                     [rate](double native) { return std::fabs(rate - native) < 1.0; })) {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: the sample rate must be a native rate");
        return;
    }
    if (report(birriasdr_set_sample_rate(dev_, static_cast<uint32_t>(std::llround(rate))),
               "set_sample_rate"))
        rate_ = birriasdr_get_sample_rate(dev_);
}

/* Return the rate last committed to the library, cached atomically so this
 * needs no library call.
 */
double SoapyBirria::getSampleRate(const int, const size_t) const
{
    return rate_;
}

/* List the native rates from lowest to highest. */
std::vector<double> SoapyBirria::listSampleRates(const int, const size_t) const
{
    return std::vector<double>(std::begin(kNativeRates), std::end(kNativeRates));
}

/* The library delivers CS16. CF32 is a scaled copy. */
std::vector<std::string> SoapyBirria::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16};
}

/* Report the native format and its full-scale value. */
std::string SoapyBirria::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    fullScale = 8192.0; /* signed 14-bit samples carried in int16 */
    return SOAPY_SDR_CS16;
}

/* Accept one RX stream on channel 0 in CS16 or CF32. The device is the stream
 * handle, so a second stream would change the format under the first.
 */
SoapySDR::Stream *SoapyBirria::setupStream(const int direction, const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &)
{
    if (direction != SOAPY_SDR_RX || (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32))
        throw std::runtime_error("birria: only RX CF32 and CS16 streams are supported");
    if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0))
        throw std::runtime_error("birria: only channel 0 is supported");
    if (streamOpen_)
        throw std::runtime_error("birria: close the open stream before setting up another");
    cs16_ = format == SOAPY_SDR_CS16;
    streamOpen_ = true;
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

/* Stop the stream if it is running, then allow a new one. */
void SoapyBirria::closeStream(SoapySDR::Stream *)
{
    deactivateStream(nullptr);
    streamOpen_ = false;
}

/* Forward the library callback to the device instance. No exception may cross
 * into the C library, so a failure here, such as an allocation, stops the read.
 */
void SoapyBirria::asyncCallback(const int16_t *iq, uint32_t count, uint64_t, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onSamples(iq, count);
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Forward the library's gap notification to the device instance, with the same
 * failure handling as the sample callback.
 */
void SoapyBirria::recoveryCallback(const birriasdr_recovery_event *, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onGap();
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Record the failure and cancel the read, which callbacks may do. The read then
 * returns normally, so the library still runs its cleanup. Callbacks that
 * arrive while it drains are dropped, so no samples follow the lost ones
 * without a gap.
 */
void SoapyBirria::onCallbackFailure()
{
    callbackFailed_ = true;
    birriasdr_cancel_async(dev_);
}

/* Runs on the read thread: copy and return. Never block here or the device SRAM overruns. */
void SoapyBirria::onSamples(const int16_t *iq, uint32_t count)
{
    if (count == 0 || callbackFailed_)
        return; /* an empty chunk would read as a gap, and a failure ends the stream */
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_ + count > kMaxQueuedSamples) {
        markGap(); /* the application is not keeping up, so drop these samples */
        return;
    }
    queue_.emplace_back(iq, iq + 2 * static_cast<size_t>(count));
    queued_ += count;
    cond_.notify_one();
}

/* Runs on the read thread after the library has delivered every sample from
 * before the gap, so a marker queued here sits at the gap's position.
 */
void SoapyBirria::onGap()
{
    if (callbackFailed_)
        return;
    std::lock_guard<std::mutex> lock(mutex_);
    markGap();
}

/* Queue a gap marker unless one is already last, so markers stay bounded by
 * the data chunks between them. The caller holds mutex_.
 */
void SoapyBirria::markGap()
{
    if (queue_.empty() || !queue_.back().empty())
        queue_.emplace_back();
    cond_.notify_one();
}

/* Reset the queue, then run the blocking library read on a worker. A worker
 * whose read already ended on its own is joined before replacing it. Timed
 * starts, triggers and finite bursts are not supported, so any flag is refused.
 */
int SoapyBirria::activateStream(SoapySDR::Stream *, const int flags, const long long,
                                const size_t numElems)
{
    if (flags != 0 || numElems != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    /* Exclusive, so an idle-only setter cannot overlap the start of the read. */
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (running_)
        return 0;
    if (worker_.joinable())
        worker_.join();
    current_.clear();
    offset_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
    }
    callbackFailed_ = false;
    active_ = true;
    running_ = true;
    try {
        worker_ = std::thread([this] {
            report(birriasdr_read_async(dev_, &SoapyBirria::asyncCallback, this, 0, 0),
                   "read_async");
            if (callbackFailed_)
                logError("birria: stopped the stream after a callback failure");
            {
                /* Publish under the lock so a reader cannot miss the wakeup. */
                std::lock_guard<std::mutex> lock(mutex_);
                running_ = false;
            }
            cond_.notify_all();
        });
    } catch (const std::exception &) {
        active_ = false;
        running_ = false;
        logError("birria: cannot start the stream thread");
        return SOAPY_SDR_STREAM_ERROR;
    }
    return 0;
}

/* Cancel the library read and join its worker. Right after activation the
 * worker may not have started the read yet, and cancel returns STATE, so keep
 * retrying until the read accepts it or the worker has finished. Samples still
 * queued are discarded. Timed and triggered stops are not supported.
 */
int SoapyBirria::deactivateStream(SoapySDR::Stream *, const int flags, const long long)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    active_ = false;
    if (worker_.joinable()) {
        while (birriasdr_cancel_async(dev_) == BIRRIASDR_ERROR_STATE && running_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        worker_.join();
    }
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    queued_ = 0;
    return 0;
}

/* Copy what is available, up to numElems, converting to the stream format.
 * Wait only when nothing is ready. A gap ends the read early and returns
 * OVERFLOW on the next call, so no buffer spans it. An inactive stream waits
 * for the timeout and returns TIMEOUT, as SoapySDR requires. An active stream
 * whose read has ended returns STREAM_ERROR once its samples are drained.
 */
int SoapyBirria::readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems,
                            int &flags, long long &timeNs, const long timeoutUs)
{
    flags = 0;
    timeNs = 0;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
    if (!active_) {
        std::this_thread::sleep_until(deadline);
        return SOAPY_SDR_TIMEOUT;
    }
    size_t count = 0;
    while (count < numElems) {
        if (offset_ == current_.size() / 2) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                if (count > 0)
                    break;
                if (running_ && !cond_.wait_until(lock, deadline,
                                                  [this] { return !queue_.empty() || !running_; }))
                    return SOAPY_SDR_TIMEOUT;
                if (queue_.empty())
                    return active_ ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_TIMEOUT;
            }
            if (queue_.front().empty()) {
                if (count > 0)
                    break;
                queue_.pop_front();
                return SOAPY_SDR_OVERFLOW;
            }
            current_ = std::move(queue_.front());
            queue_.pop_front();
            queued_ -= current_.size() / 2;
            offset_ = 0;
        }
        const size_t n = std::min(current_.size() / 2 - offset_, numElems - count);
        const int16_t *in = &current_[2 * offset_];
        if (cs16_) {
            std::memcpy(static_cast<int16_t *>(buffs[0]) + 2 * count, in, 4 * n);
        } else {
            constexpr float kScale = 1.0f / 8192.0f;
            float *out = static_cast<float *>(buffs[0]) + 2 * count;
            for (size_t k = 0; k < 2 * n; ++k)
                out[k] = in[k] * kScale;
        }
        count += n;
        offset_ += n;
    }
    return static_cast<int>(count);
}
