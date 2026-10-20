// SPDX-License-Identifier: MIT
#include <crucible/fx/PartitionedConvolver.h>
#include "AllocCounter.h"
#include "CpuTiming.h"
#include "TestSupport.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG) ||                                    \
    (defined(__x86_64__) && defined(__APPLE__))
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#include <crucible/Simd.h>
#if !CRUCIBLE_SIMD_NATIVE
#define CRUCIBLE_TEST_UNTIMED 1
#endif

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Lcg
{
    std::uint64_t s;
    double next() // uniform in [-1, 1)
    {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<double>(static_cast<std::int64_t>(s >> 11)) / 4503599627370496.0 - 1.0;
    }
};

std::vector<float> makeKernel(int length, std::uint64_t seed)
{
    Lcg r{seed};
    std::vector<float> h(static_cast<size_t>(length));
    for (int i = 0; i < length; ++i)
        h[static_cast<size_t>(i)] = static_cast<float>(r.next() * std::exp(-3.0 * i / std::max(1, length)));
    return h;
}

// Input: noise for the first part, then silence (so that the tails are checked too).
std::vector<float> makeInput(int total, int active, std::uint64_t seed)
{
    Lcg r{seed};
    std::vector<float> x(static_cast<size_t>(total), 0.0f);
    for (int i = 0; i < active; ++i)
        x[static_cast<size_t>(i)] = static_cast<float>(r.next());
    return x;
}

// Double-precision direct convolution, delayed by `latency`.
std::vector<double> directReference(const std::vector<float>& x, const std::vector<float>& h, int latency)
{
    std::vector<double> y(x.size(), 0.0);
    for (size_t i = 0; i < x.size(); ++i)
    {
        if (x[i] == 0.0f)
            continue;
        const double xi = x[i];
        for (size_t j = 0; j < h.size(); ++j)
        {
            const size_t t = i + j + static_cast<size_t>(latency);
            if (t >= y.size())
                break;
            y[t] += xi * h[j];
        }
    }
    return y;
}

double relativeErrorDb(const std::vector<float>& got, const std::vector<double>& want)
{
    double e = 0.0, s = 0.0;
    for (size_t i = 0; i < want.size(); ++i)
    {
        const double d = static_cast<double>(got[i]) - want[i];
        e += d * d;
        s += want[i] * want[i];
    }
    return 10.0 * std::log10((e + 1.0e-300) / std::max(s, 1.0e-300));
}

std::vector<float> run(const ConvolverPlan& plan, const std::vector<float>& h, const std::vector<float>& x, int block,
                       bool* ok = nullptr)
{
    NonUniformConvolver c;
    const bool prepared = c.prepare(plan, h.data(), static_cast<int>(h.size()));
    if (ok != nullptr)
        *ok = prepared;
    std::vector<float> y(x.size());
    for (size_t at = 0; at < x.size(); at += static_cast<size_t>(block))
    {
        const int n = static_cast<int>(std::min(static_cast<size_t>(block), x.size() - at));
        c.process(x.data() + at, y.data() + at, n);
    }
    return y;
}
} // namespace

class PartitionedConvolverTests : public UnitTest
{
public:
    PartitionedConvolverTests() : UnitTest("Non-uniform partitioned convolver", "Fx") {}

    void runTest() override
    {
        struct NamedPlan
        {
            const char* name;
            ConvolverPlan plan;
        };
        const std::vector<NamedPlan> plans = {
            {"zero latency", ConvolverPlan::zeroLatency()},
            {"low cpu", ConvolverPlan::lowCpu()},
            {"small 16/64/256", ConvolverPlan{0, 32, {16, 64, 256}}},
            {"latency 64, 32/256/2048", ConvolverPlan{64, 0, {32, 256, 2048}}},
            {"one big jump 64/4096", ConvolverPlan{0, 128, {64, 4096}}},
            {"single stage 32", ConvolverPlan{0, 64, {32}}},
        };
        const std::vector<int> lengths = {1, 31, 64, 65, 300, 2500, 20000};
        const std::vector<int> blocks = {1, 7, 64, 127, 480, 1024, 5000};
        constexpr int total = 26000;
        const std::vector<float> x = makeInput(total, 13000, 7);

        beginTest("plans are validated");
        {
            expect(ConvolverPlan::zeroLatency().valid() && ConvolverPlan::lowCpu().valid());
            expect(!(ConvolverPlan{0, 32, {32}}).valid(), "first stage must start at 2 blocks");
            expect(!(ConvolverPlan{0, 64, {128, 64}}).valid(), "ascending");
            expect(!(ConvolverPlan{0, 64, {24}}).valid(), "power of two");
            expect(!(ConvolverPlan{0, 64, {}}).valid(), "not empty");
            NonUniformConvolver c;
            const std::vector<float> h = {1.0f};
            expect(!c.prepare(ConvolverPlan{0, 8, {32}}, h.data(), 1) && !c.ready());
            float in[4] = {1, 2, 3, 4}, out[4] = {9, 9, 9, 9};
            c.process(in, out, 4);
            expect(out[0] == 0.0f && out[3] == 0.0f, "silent when not prepared");
        }

        beginTest("equals the double-precision direct convolution to -120 dB");
        double worst = -300.0;
        for (const int length : lengths)
        {
            const std::vector<float> h = makeKernel(length, 100 + static_cast<std::uint64_t>(length));
            for (const auto& np : plans)
            {
                const auto want = directReference(x, h, np.plan.latency);
                for (const int block : blocks)
                {
                    bool ok = false;
                    const auto got = run(np.plan, h, x, block, &ok);
                    const double db = relativeErrorDb(got, want);
                    worst = std::max(worst, db);
                    expect(ok && db < -120.0, String(np.name) + " ir " + String(length) + " block " + String(block) + ": " +
                                                  String(db) + " dB");
                }
            }
        }
        std::cout << "  worst relative error " << worst << " dB\n";

        beginTest("block size does not change the output; reset restarts it");
        {
            const std::vector<float> h = makeKernel(6000, 5);
            const auto a = run(ConvolverPlan::zeroLatency(), h, x, 64);
            const auto b = run(ConvolverPlan::zeroLatency(), h, x, 1000);
            double maxDiff = 0.0;
            for (size_t i = 0; i < a.size(); ++i)
                maxDiff = std::max(maxDiff, static_cast<double>(std::abs(a[i] - b[i])));
            expect(maxDiff < 1.0e-5, "block sizes agree: " + String(maxDiff));

            NonUniformConvolver c;
            c.prepare(ConvolverPlan::zeroLatency(), h.data(), static_cast<int>(h.size()));
            std::vector<float> y1(x.size()), y2(x.size());
            c.process(x.data(), y1.data(), static_cast<int>(x.size()));
            c.reset();
            c.process(x.data(), y2.data(), static_cast<int>(x.size()));
            expect(y1 == y2, "reset reproduces the output exactly");
        }

        beginTest("unit impulse kernel and delayed impulse");
        {
            std::vector<float> h(3000, 0.0f);
            h[0] = 1.0f;
            h[2000] = 0.5f;
            const auto want = directReference(x, h, 0);
            const auto got = run(ConvolverPlan::zeroLatency(), h, x, 100);
            expect(relativeErrorDb(got, want) < -120.0);
            expect(std::abs(got[5] - x[5]) < 1.0e-6f, "zero latency");
        }

#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU and per-block spread: 4 s stereo IR at 48 kHz, 128-sample blocks");
        {
            constexpr double rate = 48000.0;
            constexpr int ir = static_cast<int>(4.0 * rate), block = 128, passes = 5;
            const int blocksTotal = static_cast<int>(6.0 * rate) / block;
            const std::vector<float> hl = makeKernel(ir, 21), hr = makeKernel(ir, 22);
            std::vector<float> in(static_cast<size_t>(blocksTotal) * block), out(static_cast<size_t>(block));
            Lcg r{3};
            for (auto& v : in)
                v = static_cast<float>(0.3 * r.next());
            struct Mode
            {
                const char* name;
                ConvolverPlan plan;
                double limit;
            };
            for (const Mode& mode :
                 {Mode{"low-CPU", ConvolverPlan::lowCpu(), 0.6}, Mode{"zero-latency", ConvolverPlan::zeroLatency(), 1.2}})
            {
                NonUniformConvolver l, rr;
                l.prepare(mode.plan, hl.data(), ir);
                rr.prepare(mode.plan, hr.data(), ir);
                std::vector<double> best(static_cast<size_t>(blocksTotal), 1.0e9);
                double bestTotal = 1.0e9;
                const crucible::testing::ScopedUserInteractiveQos qos;
                for (int pass = 0; pass < passes; ++pass)
                {
                    l.reset();
                    rr.reset();
                    double total = 0.0;
                    for (int b = 0; b < blocksTotal; ++b)
                    {
                        const float* x = in.data() + static_cast<size_t>(b) * block;
                        const auto t0 = std::chrono::steady_clock::now();
                        const double c0 = crucible::testing::threadCpuSeconds();
                        l.process(x, out.data(), block);
                        rr.process(x, out.data(), block);
                        total += crucible::testing::threadCpuSeconds() - c0;
                        // Wall time per block is for the spread check only: the best pass of each block.
                        const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                        best[static_cast<size_t>(b)] = std::min(best[static_cast<size_t>(b)], dt);
                    }
                    bestTotal = std::min(bestTotal, total);
                }
                double mean = 0.0, peak = 0.0;
                for (const double v : best)
                {
                    mean += v;
                    peak = std::max(peak, v);
                }
                mean /= static_cast<double>(best.size());
                const double percent = 100.0 * bestTotal / (static_cast<double>(blocksTotal) * block / rate);
                const double spread = peak / mean;
                std::cout << "  " << mode.name << ": " << percent << " % of one core, per-block max/mean " << spread << "\n";
                expect(percent <= mode.limit,
                       String(mode.name) + " CPU " + String(percent, 3) + " % over " + String(mode.limit));
                expect(spread <= 2.0, String(mode.name) + " per-block spread " + String(spread, 2));
            }
        }
#endif

        beginTest("process does not allocate");
        {
            const std::vector<float> h = makeKernel(20000, 11);
            NonUniformConvolver c;
            c.prepare(ConvolverPlan::zeroLatency(), h.data(), static_cast<int>(h.size()));
            std::vector<float> y(x.size());
            gAllocations = 0;
            gTrackAllocations = true;
            for (size_t at = 0; at + 333 <= x.size(); at += 333)
                c.process(x.data() + at, y.data() + at, 333);
            gTrackAllocations = false;
            expectEquals(gAllocations, 0L, "allocations in process");
        }
    }
};

static PartitionedConvolverTests partitionedConvolverTests;
