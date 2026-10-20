// SPDX-License-Identifier: MIT
#pragma once

// Level detectors for dynamics, limiters and meters (docs/plans/ENGINE-FX-PLAN.md, section 3.1):
//
//   samplePeak        largest |x| of a block
//   TruePeakDetector  inter-sample peak by 4x or 8x polyphase interpolation (the method of ITU-R BS.1770 Annex 2,
//                     with a Kaiser-windowed sinc designed here) plus optional parabolic refinement
//   RmsDetector       exponential true RMS, time constant in seconds (63 % of a power step)
//   SlidingRms        true RMS over a rectangular window of N samples
//   RunningMax/Min    sliding-window extremum in O(1) per sample (monotonic deque), the core of lookahead limiters
//   BoxSum            running sum / average over N samples (box filter, double precision, no drift)
//
// prepare()/configure() calls allocate or compute tables and belong to the control thread; everything else is
// allocation free and audio-thread safe. Out-of-line members live in src/fx/FxCore.cpp.

#include <crucible/Simd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace crucible::fx
{

inline float samplePeak(const float* x, int n)
{
    float peak = 0.0f;
    for (int i = 0; i < n; ++i)
        peak = std::max(peak, std::abs(x[i]));
    return peak;
}

// ---------------------------------------------------------------------------------------------------------------
// True peak. Every input sample produces `factor` interpolated points; process() returns the largest magnitude among
// them. The interpolation is delayed by latencySamples() (half the filter length), so a reading belongs to the input
// that many samples ago. With refinement on, the three oversampled points around each local maximum are replaced by
// the vertex of the parabola through them, which brings the 4x reading of an fs/4 sine within 0.05 dB at every phase
// (without it the 4x grid can miss the crest by 0.17 dB; 8x by 0.04 dB).
class TruePeakDetector
{
public:
    static constexpr int kMaxTaps = 32;
    static constexpr int kMaxFactor = 8;

    // factor 4 or 8; tapsPerPhase 0 picks the default (12 at 4x, 16 at 8x, the 48-tap filter of BS.1770 Annex 2 at 4x), an
    // even number up to kMaxTaps gives a longer filter: more taps follow the tails of the sinc that hard-clipped, wide-band
    // material needs (see the limiter, which uses 32). Control thread.
    void configure(int factor = 4, bool parabolicRefinement = true, int tapsPerPhase = 0);
    void reset();

    int factor() const { return upFactor; }
    int latencySamples() const { return taps / 2; }
    // Largest factor by which an interpolated point can exceed the largest of the `taps` newest samples (the sum of the
    // magnitudes of the weights of the worst phase, with a margin for the parabolic refinement). A caller that knows the
    // recent samples cannot reach a level it already has to compare against can skip process() with it.
    float gainBound() const { return bound; }
    // Takes a sample into the history without evaluating the interpolated points; the refinement starts afresh.
    void skip(float x)
    {
        pos = pos == 0 ? taps - 1 : pos - 1;
        history[static_cast<size_t>(pos)] = history[static_cast<size_t>(pos + taps)] = x;
        m1 = m2 = 0.0f;
        previous = simd::Float4::splat(0.0f);
    }

    float process(float x)
    {
        pos = pos == 0 ? taps - 1 : pos - 1;
        history[static_cast<size_t>(pos)] = history[static_cast<size_t>(pos + taps)] = x;
        // history[pos .. pos + taps) runs newest to oldest. All phases accumulate together, each in the order of k, so the
        // four (or eight) independent sums overlap instead of waiting on one another.
        const float* h = history.data() + pos;
        alignas(16) std::array<float, kMaxFactor> acc;
        simd::Float4 lo = simd::Float4::splat(0.0f), hi = lo;
        if (upFactor == 4)
        {
            if (taps == 12)
            {
                // The default length, unrolled: the loads and broadcasts do not wait on the loop counter.
                const float* t = table[0].data();
                lo = simd::Float4::load(t) * simd::Float4::splat(h[0]);
                lo = lo + simd::Float4::load(t + 1 * kMaxFactor) * simd::Float4::splat(h[1]);
                lo = lo + simd::Float4::load(t + 2 * kMaxFactor) * simd::Float4::splat(h[2]);
                lo = lo + simd::Float4::load(t + 3 * kMaxFactor) * simd::Float4::splat(h[3]);
                lo = lo + simd::Float4::load(t + 4 * kMaxFactor) * simd::Float4::splat(h[4]);
                lo = lo + simd::Float4::load(t + 5 * kMaxFactor) * simd::Float4::splat(h[5]);
                lo = lo + simd::Float4::load(t + 6 * kMaxFactor) * simd::Float4::splat(h[6]);
                lo = lo + simd::Float4::load(t + 7 * kMaxFactor) * simd::Float4::splat(h[7]);
                lo = lo + simd::Float4::load(t + 8 * kMaxFactor) * simd::Float4::splat(h[8]);
                lo = lo + simd::Float4::load(t + 9 * kMaxFactor) * simd::Float4::splat(h[9]);
                lo = lo + simd::Float4::load(t + 10 * kMaxFactor) * simd::Float4::splat(h[10]);
                lo = lo + simd::Float4::load(t + 11 * kMaxFactor) * simd::Float4::splat(h[11]);
            }
            else
                for (int k = 0; k < taps; ++k)
                    lo = lo + simd::Float4::load(table[static_cast<size_t>(k)].data()) * simd::Float4::splat(h[k]);
        }
        else
        {
            for (int k = 0; k < taps; ++k)
            {
                const simd::Float4 hk = simd::Float4::splat(h[k]);
                lo = lo + simd::Float4::load(table[static_cast<size_t>(k)].data()) * hk;
                hi = hi + simd::Float4::load(table[static_cast<size_t>(k)].data() + 4) * hk;
            }
        }
        float best = 0.0f;
        if (upFactor == 4)
        {
            // The four points of this sample against the last two of the previous one, all at once: lane j of (a, b, c) is
            // the triple (m2, m1, m) of point j.
            const simd::Float4 c = simd::Float4::abs(lo);
            alignas(16) std::array<float, 8> mag;
            c.store(mag.data());
            if (refine)
            {
                const simd::Float4 a = simd::Float4::ext<2>(previous, c), b = simd::Float4::ext<3>(previous, c);
                const simd::Float4 zero = simd::Float4::splat(0.0f);
                const simd::Float4 den =
                    simd::Float4::min(a - simd::Float4::splat(2.0f) * b + c, simd::Float4::splat(-1.0e-30f));
                const simd::Float4 diff = a - c;
                const simd::Float4 vertex = b - simd::Float4::splat(0.125f) * diff * diff / den;
                // The local-maximum test is a lane mask, so the noise-like sign of it costs no mispredicted branch.
                simd::Float4 peakHere = simd::Float4::select(simd::Float4::geq(b, a), simd::Float4::gt(b, c), zero);
                peakHere = simd::Float4::select(peakHere, simd::Float4::gt(b, zero), zero);
                simd::Float4::select(peakHere, vertex, zero).store(mag.data() + 4);
                previous = c;
            }
            else
                simd::Float4::splat(0.0f).store(mag.data() + 4);
            for (const float m : mag)
                best = std::max(best, m);
            return best;
        }
        lo.store(acc.data());
        hi.store(acc.data() + 4);
        for (int p = 0; p < upFactor; ++p)
        {
            const float m = std::abs(acc[static_cast<size_t>(p)]);
            best = std::max(best, m);
            if (refine)
            {
                // Vertex of the parabola through (m2, m1, m): evaluated for every point and selected by the local-maximum
                // test, so the noise-like sign of the test costs no mispredicted branch.
                const float den = std::min(m2 - 2.0f * m1 + m, -1.0e-30f);
                const float diff = m2 - m;
                const float vertex = m1 - 0.125f * diff * diff / den;
                const bool peakHere = m1 >= m2 && m1 > m && m1 > 0.0f;
                best = std::max(best, peakHere ? vertex : 0.0f);
                m2 = m1;
                m1 = m;
            }
        }
        return best;
    }
    float process(const float* x, int n)
    {
        float best = 0.0f;
        for (int i = 0; i < n; ++i)
            best = std::max(best, process(x[i]));
        return best;
    }

private:
    // table[k][p] weights the k-th newest sample (k = 0 is the newest) for the interpolated point p / factor
    // samples after the sample that is taps / 2 samples old.
    std::array<std::array<float, kMaxFactor>, kMaxTaps> table{};
    std::array<float, 2 * kMaxTaps> history{};
    int upFactor = 4, taps = 12, pos = 0;
    float bound = 1.5f;
    bool refine = true;
    float m1 = 0.0f, m2 = 0.0f;                        // eight-fold: the last two magnitudes
    simd::Float4 previous = simd::Float4::splat(0.0f); // four-fold: the four magnitudes of the previous sample
};

// ---------------------------------------------------------------------------------------------------------------
// True RMS, exponential: power follows the input's power with the given time constant; the reading is its square
// root, so a sine of amplitude A settles at A / sqrt(2).
class RmsDetector
{
public:
    void setTimeConstant(float seconds, double sampleRate)
    {
        coefficient =
            seconds <= 0.0f ? 1.0f : 1.0f - static_cast<float>(std::exp(-1.0 / (static_cast<double>(seconds) * sampleRate)));
    }
    void reset() { power = 0.0; }
    float process(float x)
    {
        power += static_cast<double>(coefficient) * (static_cast<double>(x) * x - power);
        if (power < 1.0e-30)
            power = 0.0;
        return static_cast<float>(std::sqrt(power));
    }
    float meanSquare() const { return static_cast<float>(power); }

private:
    float coefficient = 1.0f;
    double power = 0.0;
};

// ---------------------------------------------------------------------------------------------------------------
// Sliding-window extremum, O(1) amortised per sample. The window is the last `window` samples including the newest.
// Shrinking the window takes effect at once. Growing it can only reveal samples the deque still holds, so the result
// is exact again once `window` samples have passed.
template <bool IsMax> class RunningExtreme
{
public:
    // Control thread: capacity for windows up to maxWindow.
    void prepare(int maxWindow)
    {
        capacityWindow = std::max(1, maxWindow);
        size_t cap = 4;
        while (cap < static_cast<size_t>(capacityWindow) + 2)
            cap <<= 1;
        mask = cap - 1;
        index.assign(cap, 0);
        value.assign(cap, 0.0f);
        windowSize = std::min(windowSize, capacityWindow);
        reset();
    }
    void reset()
    {
        head = tail = 0;
        time = 0;
    }
    void setWindow(int w) { windowSize = std::clamp(w, 1, capacityWindow); }
    int window() const { return windowSize; }

    float push(float x)
    {
        while (tail != head && !keeps(value[(tail - 1) & mask], x))
            --tail;
        index[tail & mask] = time;
        value[tail & mask] = x;
        ++tail;
        while (index[head & mask] + static_cast<std::uint64_t>(windowSize) <= time)
            ++head;
        ++time;
        return value[head & mask];
    }
    float current() const { return tail == head ? 0.0f : value[head & mask]; }

private:
    static bool keeps(float older, float newer) { return IsMax ? older > newer : older < newer; }

    std::vector<std::uint64_t> index;
    std::vector<float> value;
    std::uint64_t head = 0, tail = 0, time = 0, mask = 3;
    int windowSize = 1, capacityWindow = 1;
};
using RunningMax = RunningExtreme<true>;
using RunningMin = RunningExtreme<false>;

// ---------------------------------------------------------------------------------------------------------------
// Running sum over the last `window` values. Double precision; the sum is rebuilt from the stored values once per
// window, so rounding never accumulates and silence after a loud passage reads exactly zero.
class BoxSum
{
public:
    void prepare(int maxWindow);
    void reset();
    void setWindow(int w);
    int window() const { return windowSize; }
    double push(double x)
    {
        const size_t out = (pos - static_cast<size_t>(windowSize)) & mask;
        total += x - ring[out];
        ring[pos & mask] = x;
        ++pos;
        if (++sinceRebuild >= windowSize)
            rebuild();
        return total;
    }
    double sum() const { return total; }
    double average() const { return total / windowSize; }

private:
    void rebuild();
    std::vector<double> ring;
    size_t pos = 0, mask = 0;
    double total = 0.0;
    int windowSize = 1, sinceRebuild = 0, capacityWindow = 1;
};

// True RMS over a rectangular window.
class SlidingRms
{
public:
    void prepare(int maxWindow) { box.prepare(maxWindow); }
    void reset() { box.reset(); }
    void setWindow(int w) { box.setWindow(w); }
    float process(float x)
    {
        const double ms = box.push(static_cast<double>(x) * x) / box.window();
        return static_cast<float>(std::sqrt(std::max(ms, 0.0)));
    }

private:
    BoxSum box;
};

} // namespace crucible::fx
