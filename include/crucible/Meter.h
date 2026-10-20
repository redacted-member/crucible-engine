// SPDX-License-Identifier: MIT
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace crucible
{

// Output level meter and clip detector. process() runs on the audio thread and only touches atomics;
// snapshot() and resetClip() can be called from any thread.
class OutputMeter
{
public:
    struct Snapshot
    {
        float peakLeftDb = -120.0f;
        float peakRightDb = -120.0f;
        bool clip = false;
    };

    static constexpr float floorDb = -120.0f;
    static constexpr float ceilingDb = 12.0f;

    void prepare(double rate)
    {
        sampleRate = rate;
        peakL.store(0.0f);
        peakR.store(0.0f);
        clipRemaining.store(0);
        latched.store(false);
    }

    // holdSeconds <= 0 latches the clip flag until resetClip().
    void setHoldSeconds(float seconds) { hold.store(seconds); }

    void process(const float* left, const float* right, int n)
    {
        float bl = 0.0f, br = 0.0f;
        bool over = false;
        for (int i = 0; i < n; ++i)
        {
            const float a = std::abs(left[i]);
            const float b = std::abs(right[i]);
            // NaN compares false everywhere, so it never registers; infinities count as over.
            if (a > bl)
                bl = a;
            if (b > br)
                br = b;
            if (a >= 1.0f || b >= 1.0f)
                over = true;
        }
        const float decay = std::exp(-static_cast<float>(n) / (releaseSeconds * static_cast<float>(sampleRate)));
        peakL.store(std::max(bl, peakL.load(std::memory_order_relaxed) * decay), std::memory_order_relaxed);
        peakR.store(std::max(br, peakR.load(std::memory_order_relaxed) * decay), std::memory_order_relaxed);

        const float h = hold.load(std::memory_order_relaxed);
        if (over)
        {
            if (h <= 0.0f)
                latched.store(true, std::memory_order_relaxed);
            clipRemaining.store(static_cast<std::int64_t>(std::max(h, 0.0f) * static_cast<float>(sampleRate)),
                                std::memory_order_relaxed);
        }
        else
        {
            const auto r = clipRemaining.load(std::memory_order_relaxed);
            if (r > 0)
                clipRemaining.store(std::max<std::int64_t>(0, r - n), std::memory_order_relaxed);
        }
    }

    Snapshot snapshot() const
    {
        const auto toDb = [](float v)
        {
            const float db = v > 1.0e-6f ? 20.0f * std::log10(v) : floorDb;
            return std::clamp(std::isfinite(db) ? db : ceilingDb, floorDb, ceilingDb);
        };
        Snapshot s;
        s.peakLeftDb = toDb(peakL.load(std::memory_order_relaxed));
        s.peakRightDb = toDb(peakR.load(std::memory_order_relaxed));
        s.clip = latched.load(std::memory_order_relaxed) || clipRemaining.load(std::memory_order_relaxed) > 0;
        return s;
    }

    void resetClip()
    {
        latched.store(false);
        clipRemaining.store(0);
    }

private:
    static constexpr float releaseSeconds = 0.3f;
    double sampleRate = 48000.0;
    std::atomic<float> peakL{0.0f};
    std::atomic<float> peakR{0.0f};
    std::atomic<std::int64_t> clipRemaining{0};
    std::atomic<bool> latched{false};
    std::atomic<float> hold{2.0f};
};

} // namespace crucible
