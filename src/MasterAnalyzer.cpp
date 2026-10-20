// SPDX-License-Identifier: MIT
#include <crucible/MasterAnalyzer.h>

#include <crucible/Fft.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

float db(float amplitude)
{
    return amplitude > 1.0e-6f ? 20.0f * std::log10(amplitude) : -120.0f;
}
} // namespace

void MasterAnalyzer::prepare(double sampleRate)
{
    sr = sampleRate > 0.0 ? sampleRate : 48000.0;
    meter.prepare(sr, 2, true);
    blockLength = std::max(1, static_cast<int>(sr * 0.1));
    blockFill = 0;
    sumLR = sumLL = sumRR = 0.0;
    resetRequested.store(false, std::memory_order_relaxed);
    scopeL.fill(0.0f);
    scopeR.fill(0.0f);
    scopeWrite.store(0);
    momentary.store(-120.0f);
    shortTerm.store(-120.0f);
    truePeak.store(0.0f);
    maxTruePeak.store(0.0f);
    correlationValue.store(0.0f);
}

void MasterAnalyzer::process(const float* left, const float* right, int n)
{
    if (!enabled.load(std::memory_order_relaxed))
        return;
    if (resetRequested.exchange(false, std::memory_order_relaxed))
        meter.resetStatistics();
    meter.process(left, right, n);
    int w = scopeWrite.load(std::memory_order_relaxed);
    for (int i = 0; i < n; ++i)
    {
        const double l = left[i], r = right[i];
        sumLR += l * r;
        sumLL += l * l;
        sumRR += r * r;
        scopeL[static_cast<size_t>(w)] = left[i];
        scopeR[static_cast<size_t>(w)] = right[i];
        w = (w + 1) % scopeSize;
        if (++blockFill >= blockLength)
        {
            const double denom = std::sqrt(sumLL * sumRR);
            correlationValue.store(denom > 1.0e-12 ? static_cast<float>(sumLR / denom) : 0.0f, std::memory_order_relaxed);
            sumLR = sumLL = sumRR = 0.0;
            blockFill = 0;
        }
    }
    scopeWrite.store(w, std::memory_order_release);
    momentary.store(meter.momentaryLufs(), std::memory_order_relaxed);
    shortTerm.store(meter.shortTermLufs(), std::memory_order_relaxed);
    truePeak.store(meter.decayingTruePeak(), std::memory_order_relaxed);
    maxTruePeak.store(std::max(meter.truePeak(0), meter.truePeak(1)), std::memory_order_relaxed);
}

LoudnessSnapshot MasterAnalyzer::loudness() const
{
    LoudnessSnapshot s;
    s.momentaryLufs = momentary.load(std::memory_order_relaxed);
    s.shortTermLufs = shortTerm.load(std::memory_order_relaxed);
    s.truePeakDb = db(truePeak.load(std::memory_order_relaxed));
    s.maxTruePeakDb = db(maxTruePeak.load(std::memory_order_relaxed));
    s.correlation = correlationValue.load(std::memory_order_relaxed);
    return s;
}

void MasterAnalyzer::copyScope(float* left, float* right, int count) const
{
    count = std::clamp(count, 0, scopeSize);
    const int end = scopeWrite.load(std::memory_order_acquire);
    for (int i = 0; i < count; ++i)
    {
        const int idx = ((end - count + i) % scopeSize + scopeSize) % scopeSize;
        left[i] = scopeL[static_cast<size_t>(idx)];
        right[i] = scopeR[static_cast<size_t>(idx)];
    }
}

void MasterAnalyzer::scope(std::vector<float>& left, std::vector<float>& right, int count) const
{
    count = std::clamp(count, 0, scopeSize);
    left.resize(static_cast<size_t>(count));
    right.resize(static_cast<size_t>(count));
    copyScope(left.data(), right.data(), count);
}

int MasterAnalyzer::fftLength(int fftSize)
{
    int order = 8;
    while ((1 << order) < fftSize && order < 12)
        ++order;
    return 1 << order;
}

std::vector<float> MasterAnalyzer::spectrum(int fftSize) const
{
    const int n = fftLength(fftSize);
    std::vector<float> l, r;
    scope(l, r, n);
    return spectrumOf(l.data(), r.data(), n);
}

std::vector<float> MasterAnalyzer::spectrumOf(const float* l, const float* r, int fftSize)
{
    int order = 8;
    while ((1 << order) < fftSize && order < 12)
        ++order;
    const int n = 1 << order;
    std::vector<float> buf(static_cast<size_t>(2 * n), 0.0f);
    double windowSum = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const double w = 0.5 - 0.5 * std::cos(2.0 * kPi * i / (n - 1));
        windowSum += w;
        buf[static_cast<size_t>(i)] = static_cast<float>(0.5 * (l[i] + r[i]) * w);
    }
    const Fft fft(order);
    fft.forward(buf.data());
    std::vector<float> out(static_cast<size_t>(n / 2 + 1));
    for (int k = 0; k <= n / 2; ++k)
    {
        const double re = buf[static_cast<size_t>(2 * k)], im = buf[static_cast<size_t>(2 * k + 1)];
        const double amp = 2.0 * std::sqrt(re * re + im * im) / windowSum;
        out[static_cast<size_t>(k)] = amp > 1.0e-7 ? static_cast<float>(20.0 * std::log10(amp)) : -140.0f;
    }
    return out;
}

} // namespace crucible
