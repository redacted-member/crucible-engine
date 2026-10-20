// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <vector>

namespace crucible
{

// Partition plan of a NonUniformConvolver.
//
// The kernel is split in time: the first `head` taps run in direct form (zero latency), then stages of FFT partitions
// with block sizes `blocks[0] < blocks[1] < ...` (powers of two, at least 16). A stage with block size B must start at
// least 2 * B samples after the input (latency + head >= 2 * blocks[0]); the following stages start as early as that
// rule allows, so the first stage covers a short stretch with small partitions and the last one the bulk of the kernel
// with the largest partitions. Every stage's work for one block is spread evenly over the following block period.
// `latency` delays the whole output by that many samples (0 for the zero-latency mode).
struct ConvolverPlan
{
    int latency = 0;
    int head = 64;
    std::vector<int> blocks{32, 128, 1024, 8192};

    // Latency of the two presets, as constants: reading them never builds a plan (a plan owns a vector, so building one
    // allocates and must stay off the audio thread).
    static constexpr int kZeroLatencySamples = 0;
    static constexpr int kLowCpuLatencySamples = 128;

    // Zero latency: 64 direct taps, partitions 32 / 128 / 1024 / 8192.
    static ConvolverPlan zeroLatency() { return ConvolverPlan{kZeroLatencySamples, 64, {32, 128, 1024, 8192}}; }
    // Low CPU: output delayed by 128 samples, no direct taps, partitions 64 / 512 / 4096.
    static ConvolverPlan lowCpu() { return ConvolverPlan{kLowCpuLatencySamples, 0, {64, 512, 4096}}; }

    // True when the plan can be run (see above).
    bool valid() const noexcept;
};

// Mono non-uniform partitioned convolution (Gardner 1995, Wefers 2015) on the FFT of crucible/Fft.h and the split-complex
// helpers of the vendored FFTConvolver sources (private to the library). process() allocates nothing and takes no locks.
// The output equals the double-precision direct convolution of the input with the kernel, delayed by plan().latency,
// to the accuracy of single-precision FFTs.
class NonUniformConvolver
{
public:
    NonUniformConvolver();
    ~NonUniformConvolver();
    NonUniformConvolver(const NonUniformConvolver&) = delete;
    NonUniformConvolver& operator=(const NonUniformConvolver&) = delete;

    // Control thread (allocates). Installs the kernel and clears all state. Returns false (and leaves the convolver
    // silent) when the plan is invalid or the kernel is empty.
    bool prepare(const ConvolverPlan& plan, const float* kernel, int length);
    // Clears the signal history and the tails; keeps the kernel.
    void reset() noexcept;

    // Any n; `out` may alias `in`. Silence until prepare() has succeeded.
    void process(const float* in, float* out, int n) noexcept;

    int latency() const noexcept;
    int kernelLength() const noexcept;
    bool ready() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

} // namespace crucible
