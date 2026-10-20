// SPDX-License-Identifier: MIT
#pragma once

// The one way a module oversamples (docs/plans/ENGINE-FX-PLAN.md, sections 3.1, 3.9 and 4.4). It wraps one
// Oversampler per channel: up, a per-block kernel at the high rate, down. It reports the latency, keeps the dry path
// aligned for wet / dry mixing, and allocates in prepare() only.
//
// Dry path (DryPath):
//   Delay    the input delayed by latencySamples() whole samples. Right for the linear-phase FIR family, where the
//            wet path is a pure delay plus pass-band ripple; at mix 0 the output is the delayed input exactly
//            (a null at float precision, about -150 dB), and it costs one ring buffer.
//   Matched  the input sent through a second up / down pair with the same filters and no kernel. Its phase is the
//            wet path's phase, so mixing a linear kernel never combs, which matters for the minimum-latency IIR
//            family (not linear phase). It costs a second oversampler per channel.
// With an identity kernel Matched gives wet == dry, so every mix setting returns the same signal.
//
// stages == 0 means no oversampling: the kernel runs at the base rate, the latency is 0 and both dry paths are the
// input itself.

#include <crucible/Oversampler.h>

#include <array>
#include <vector>

namespace crucible::fx
{

enum class DryPath
{
    Delay,
    Matched
};

class OversampledBlock
{
public:
    static constexpr int kMaxChannels = 2;

    // Control thread. stages: 0 = 1x, 1 = 2x, 2 = 4x, 3 = 8x.
    void prepare(int numChannels, int stages, OversamplingFilter filter, Quality quality, int maxBlock,
                 DryPath dry = DryPath::Delay);
    void reset();

    int factor() const { return upFactor; }
    // Measured latency of up + down in base-rate samples (fractional for the IIR family), and the whole-sample value
    // that modules report and that the Delay dry path uses.
    float latency() const { return measuredLatency; }
    int latencySamples() const { return reported; }
    DryPath dryPath() const { return dryMode; }
    int channelCount() const { return channels; }

    // In place. kernel(float* const* x, int channels, int count) receives the oversampled blocks (count =
    // n * factor() samples per channel) and processes them in place. mix 0 returns the dry path, 1 the wet path; the
    // mix moves linearly from the previous call's value across the block.
    template <class Kernel> void process(float* const* io, int n, float mix, Kernel&& kernel)
    {
        int done = 0;
        while (done < n)
        {
            const int len = n - done < blockLimit ? n - done : blockLimit;
            float* chans[kMaxChannels];
            for (int c = 0; c < channels; ++c)
                chans[c] = io[c] + done;
            processChunk(chans, len, mix, kernel);
            done += len;
        }
    }

private:
    template <class Kernel> void processChunk(float* const* io, int n, float mix, Kernel& kernel)
    {
        float* wetPtr[kMaxChannels];
        for (int c = 0; c < channels; ++c)
        {
            auto cs = static_cast<size_t>(c);
            // Dry path first: io is overwritten below.
            if (dryMode == DryPath::Delay || upFactor == 1)
            {
                auto& ring = dryRing[cs];
                size_t p = dryPos;
                for (int i = 0; i < n; ++i, ++p)
                {
                    ring[p & dryMask] = io[c][i];
                    dryBlock[cs][static_cast<size_t>(i)] = ring[(p - static_cast<size_t>(reported)) & dryMask];
                }
            }
            else
            {
                dryOs[cs].up(io[c], n, wet[cs].data());
                dryOs[cs].down(wet[cs].data(), n, dryBlock[cs].data());
            }
            if (upFactor == 1)
                std::copy(io[c], io[c] + n, wet[cs].begin());
            else
                os[cs].up(io[c], n, wet[cs].data());
            wetPtr[c] = wet[cs].data();
        }
        dryPos += static_cast<size_t>(n);
        kernel(wetPtr, channels, n * upFactor);
        const float m0 = mixPrimed ? lastMix : mix;
        mixPrimed = true;
        for (int c = 0; c < channels; ++c)
        {
            auto cs = static_cast<size_t>(c);
            if (upFactor == 1)
                std::copy(wet[cs].begin(), wet[cs].begin() + n, io[c]);
            else
                os[cs].down(wet[cs].data(), n, io[c]);
            const float* d = dryBlock[cs].data();
            for (int i = 0; i < n; ++i)
            {
                const float m = m0 + (mix - m0) * static_cast<float>(i + 1) / static_cast<float>(n);
                io[c][i] = d[i] + (io[c][i] - d[i]) * m;
            }
        }
        lastMix = mix;
    }

    std::array<Oversampler, kMaxChannels> os, dryOs;
    std::array<std::vector<float>, kMaxChannels> wet, dryBlock, dryRing;
    size_t dryMask = 0, dryPos = 0;
    int channels = 2, upFactor = 1, reported = 0, blockLimit = 512;
    float measuredLatency = 0.0f, lastMix = 1.0f;
    bool mixPrimed = false;
    DryPath dryMode = DryPath::Delay;
};

} // namespace crucible::fx
