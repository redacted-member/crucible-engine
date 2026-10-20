// SPDX-License-Identifier: MIT
#pragma once

// Fractional delay line with Kaiser-windowed sinc reads (docs/plans/ENGINE-FX-PLAN.md, section 3.5, package FX10).
//
// The read at a delay d = i + f is the band-limited interpolation of the stored samples at that position: a polyphase
// table of Kaiser-windowed sinc kernels (the window is centred on the true position, so every phase is symmetric about
// the delay it represents, and each phase is normalised to unity gain at DC). The table has kPhases + 1 phases and the
// read interpolates linearly between two neighbouring phases, so the delay is continuous and can change every sample
// (modulation) without the stepping a nearest-phase read would add. Laakso, Valimaki, Karjalainen and Laine, "Splitting
// the unit delay", IEEE Signal Processing Magazine 1996, for the background; Kaiser and Schafer, "On the use of the
// I0-sinh window for spectrum analysis", IEEE Trans. ASSP 1980, for the window.
//
// Position convention: write() stores the newest sample, then read(d) returns the signal d samples ago (read(0) is the
// sample just written). A kernel of `taps` points needs samples up to taps / 2 after the position, so the smallest
// delay a read can take is minDelay() = taps / 2 - 1; shorter requests are clamped to it. The largest is maxDelay().
// Quality: 16 taps is flat to about 0.35 of the sample rate, 32 taps (the default) to 0.41, 64 taps to 0.45.
//
// prepare() allocates; everything else is real-time safe.

#include <crucible/Simd.h>

#include <cstddef>
#include <vector>

namespace crucible::fx
{

class FractionalDelay
{
public:
    static constexpr int kPhases = 512;

    // Allocates for delays up to maxDelaySamples. taps is even, 8 to 64.
    void prepare(double maxDelaySamples, int taps = 32);
    void reset();

    int taps() const { return numTaps; }
    double minDelay() const { return static_cast<double>(numTaps / 2 - 1); }
    double maxDelay() const { return maxDelaySamples; }

    void write(float x)
    {
        const std::size_t i = writePos;
        buffer[i] = x;
        buffer[i + size] = x;
        writePos = (i + 1) & mask;
    }

    // The signal `delay` samples ago (see the position convention above).
    float read(double delay) const
    {
        if (delay < minD)
            delay = minD;
        else if (delay > maxD)
            delay = maxD;
        const long whole = static_cast<long>(delay);
        const double phasePos = (delay - static_cast<double>(whole)) * kPhases;
        const int p = static_cast<int>(phasePos);
        const float w = static_cast<float>(phasePos - p);
        // Newest sample is at writePos - 1. The kernel covers lags whole - (taps/2 - 1) .. whole + taps/2, oldest first.
        const std::size_t newest = writePos + size - 1;
        const std::size_t start = newest - static_cast<std::size_t>(whole + numTaps / 2);
        const float* x = buffer.data() + start;
        const float* a = table.data() + static_cast<std::size_t>(p) * static_cast<std::size_t>(numTaps);
        const float* b = a + numTaps;
        // Four partial sums (a serial sum of 16 to 64 products is a chain of dependent additions) and a scalar tail.
        float acc[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        int k = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
        // The four partial sums are the four lanes of one vector: the same operations in the same order per lane (MSVC
        // left the loop scalar).
        {
            const __m128 wv = _mm_set1_ps(w);
            __m128 accV = _mm_setzero_ps();
            for (; k + 4 <= numTaps; k += 4)
            {
                const __m128 av = _mm_loadu_ps(a + k);
                accV = _mm_add_ps(accV, _mm_mul_ps(_mm_add_ps(av, _mm_mul_ps(wv, _mm_sub_ps(_mm_loadu_ps(b + k), av))),
                                                   _mm_loadu_ps(x + k)));
            }
            _mm_storeu_ps(acc, accV);
        }
#else
        for (; k + 4 <= numTaps; k += 4)
            for (int q = 0; q < 4; ++q)
                acc[q] += (a[k + q] + w * (b[k + q] - a[k + q])) * x[k + q];
#endif
        float tail = 0.0f;
        for (; k < numTaps; ++k)
            tail += (a[k] + w * (b[k] - a[k])) * x[k];
        return ((acc[0] + acc[1]) + (acc[2] + acc[3])) + tail;
    }

    // True when read(delay) after `pending` more write() calls can already be computed now with readEarly(): no clamping,
    // and none of the kernel's samples is among the pending writes or overwritten by them.
    bool canReadEarly(double delay, int pending) const
    {
        return delay >= minD && delay <= maxD && delay - static_cast<double>(pending) >= static_cast<double>(numTaps) &&
               delay + static_cast<double>(pending + numTaps + 2) < static_cast<double>(size);
    }
    // The value read(delay) returns after `pending` more write() calls (when canReadEarly(delay, pending)): the same
    // samples, kernel phase and arithmetic. delay - pending is exact (pending is an integer, the delay below 2^52), so
    // its whole part is the delay's minus pending and its fraction the same.
    float readEarly(double delay, int pending) const { return read(delay - static_cast<double>(pending)); }

    float process(float x, double delay)
    {
        write(x);
        return read(delay);
    }

private:
    std::vector<float> buffer; // twice `size`: every sample is stored at i and i + size so a window is contiguous
    std::vector<float> table;  // (kPhases + 1) phases of numTaps, oldest sample first
    std::size_t size = 0;
    std::size_t mask = 0;
    std::size_t writePos = 0;
    int numTaps = 32;
    double minD = 15.0;
    double maxD = 0.0;
    double maxDelaySamples = 0.0;
};

} // namespace crucible::fx
