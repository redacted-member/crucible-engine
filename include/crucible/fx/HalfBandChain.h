// SPDX-License-Identifier: MIT
#pragma once

// A cascade of linear-phase half-band stages for one channel, 2x to 16x, built for modules whose nonlinearity needs a
// deep stop band and a flat pass band to 20 kHz (docs/plans/ENGINE-FX-PLAN.md, sections 3.9 and 4.4). It is the
// polyphase form of the filters in crucible/Oversampler.h with two differences: every stage has a design of its own
// (the stage at the base rate carries the steep transition, the stages above it work at higher rates and are short),
// and the taps of a stage are one contiguous dot product over a linear history, which the compiler vectorises.
//
// Stage s takes the signal from base * 2^s to base * 2^(s + 1). A stage with 4m + 3 taps has its centre tap at the
// odd index 2m + 1, so only the odd-index taps are non-zero (the half-band property) and:
//   up    even output = 2 * dot(w, x[n - d - P .. n - d + P - 1]), odd output = x[n - d], with d = m, P = m + 1
//   down  y[i] = 0.5 * e[i - m] + dot(w, o[i - m - P .. i - m + P - 1]), e and o the even and odd samples of the input
// where w holds the odd-index taps mirrored around the centre. Latency per stage is the centre delay; latency() is
// measured from an impulse at prepare time, like Oversampler.
//
// prepare() allocates; up(), down() and reset() never do.

#include <crucible/Simd.h>
#include <crucible/fx/HalfBandAvxKernel.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

// MSVC x64: the dot products also have an eight-lane AVX form (src/fx/HalfBandAvx.cpp, the only file of it built for AVX),
// used on CPUs with AVX; the same operations per output, so the same bits.
#if CRUCIBLE_HALFBAND_AVX_PLATFORM && CRUCIBLE_SIMD_SSE_EXACT
#define CRUCIBLE_HALFBAND_AVX 1
#else
#define CRUCIBLE_HALFBAND_AVX 0
#endif

namespace crucible::fx
{

#if CRUCIBLE_HALFBAND_AVX
namespace detail
{
// True when the CPU and the OS support AVX (checked once, src/fx/FxCore.cpp). The AVX kernel itself is declared in
// crucible/fx/HalfBandAvxKernel.h, which the AVX unit includes alone.
bool halfBandAvx();
} // namespace detail
#endif

class HalfBandChain
{
public:
    static constexpr int kMaxStages = 4;
    static constexpr int kMaxTaps = 127;

    struct Design
    {
        int taps = 47;     // 4m + 3
        double beta = 7.0; // Kaiser window shape
    };

    // Design of stage `stage` (0 = at the base rate) for a stop band of `attenuationDb`: pass band to min(20 kHz,
    // 0.45 * rate), stop band from half the stage's output rate minus that edge, Kaiser window (Kaiser's estimates).
    static Design designFor(double baseRate, int stage, double attenuationDb)
    {
        const double pass = std::min(20000.0, 0.45 * baseRate);
        const double out = baseRate * static_cast<double>(2 << stage);
        const double width = std::max(1.0e-3, (0.5 * out - pass - pass) / out);
        double taps = (attenuationDb - 7.95) / (14.36 * width) + 1.0;
        int m = static_cast<int>(std::ceil((taps - 3.0) / 4.0));
        m = std::clamp(m, 1, (kMaxTaps - 3) / 4);
        return {4 * m + 3, 0.1102 * (attenuationDb - 8.7)};
    }

    // Control thread. designs: one per stage, index 0 at the base rate.
    void prepare(int numStages, const Design* designs, int maxBlock)
    {
        stages = std::clamp(numStages, 1, kMaxStages);
        block = std::max(maxBlock, 32);
        for (int s = 0; s < stages; ++s)
            design(stage[static_cast<size_t>(s)], designs[s]);
        size_t most = static_cast<size_t>(block) << stages;
        scratchA.assign(most + 8, 0.0f);
        scratchB.assign(most + 8, 0.0f);
        even.assign(most + 8, 0.0f);
        for (int s = 0; s < stages; ++s)
        {
            auto& st = stage[static_cast<size_t>(s)];
            const size_t inLen = static_cast<size_t>(block) << s; // stage input length when going up
            const size_t outLen = inLen * 2;
            st.upWork.assign(static_cast<size_t>(st.history) + inLen + 8, 0.0f);
            st.oddWork.assign(static_cast<size_t>(st.history) + outLen / 2 + 8, 0.0f);
            st.evenWork.assign(static_cast<size_t>(st.m) + outLen / 2 + 8, 0.0f);
        }
        fineLatency = 0;
        for (int s = 0; s < stages; ++s)
            fineLatency += (4 * stage[static_cast<size_t>(s)].m + 1) << (stages - s - 1);
        reset();
    }

    void reset()
    {
        for (auto& s : stage)
        {
            std::fill(s.upWork.begin(), s.upWork.end(), 0.0f);
            std::fill(s.oddWork.begin(), s.oddWork.end(), 0.0f);
            std::fill(s.evenWork.begin(), s.evenWork.end(), 0.0f);
        }
    }

    int factor() const { return 1 << stages; }
    // Latency of up() followed by down(): exact, in samples at the finest rate (a stage with 4m + 3 taps delays 4m + 1
    // samples at its output rate), and in base-rate samples.
    int latencyFine() const { return fineLatency; }
    float latency() const { return static_cast<float>(fineLatency) / static_cast<float>(factor()); }

    // in: n samples, out: n * factor() samples; n <= the maxBlock of prepare().
    void up(const float* in, int n, float* out)
    {
        const float* src = in;
        size_t len = static_cast<size_t>(n);
        for (int s = 0; s < stages; ++s)
        {
            float* dst = (s == stages - 1) ? out : (s % 2 == 0 ? scratchA.data() : scratchB.data());
            upStage(stage[static_cast<size_t>(s)], src, len, dst);
            src = dst;
            len *= 2;
        }
    }

    // in: n * factor() samples, out: n samples.
    void down(const float* in, int n, float* out)
    {
        const float* src = in;
        size_t len = static_cast<size_t>(n) << stages;
        for (int s = stages - 1; s >= 0; --s)
        {
            len /= 2;
            float* dst = (s == 0) ? out : (s % 2 == 0 ? scratchA.data() : scratchB.data());
            downStage(stage[static_cast<size_t>(s)], src, len, dst);
            src = dst;
        }
    }

private:
    struct Stage
    {
        int m = 0;       // taps = 4m + 3
        int pairs = 0;   // P = m + 1
        int window = 0;  // dot-product length: 2P padded with zeros (old end) to a multiple of 8
        int history = 0; // samples kept ahead of a block in the up / odd work buffers
        std::array<float, 72> w{};
        std::vector<float> upWork, oddWork, evenWork;
    };

    static double besselI0(double x)
    {
        double sum = 1.0, term = 1.0;
        for (int k = 1; k < 60; ++k)
        {
            term *= (x / (2.0 * k)) * (x / (2.0 * k));
            sum += term;
        }
        return sum;
    }

    static void design(Stage& s, const Design& d)
    {
        constexpr double kPi = 3.14159265358979323846;
        const int taps = std::clamp(d.taps, 7, kMaxTaps) / 4 * 4 + 3;
        const int centre = (taps - 1) / 2;
        s.m = (centre - 1) / 2;
        s.pairs = s.m + 1;
        const int pad = (8 - (2 * s.pairs) % 8) % 8;
        s.window = 2 * s.pairs + pad;
        s.history = s.m + s.pairs + pad;
        s.w.fill(0.0f);
        std::vector<double> h(static_cast<size_t>(taps));
        double total = 0.0;
        for (int k = 0; k < taps; ++k)
        {
            const int n = k - centre;
            double v = 0.0;
            if (n == 0)
                v = 0.5;
            else if (n % 2 != 0)
                v = std::sin(kPi * n / 2.0) / (kPi * n);
            const double r = static_cast<double>(n) / centre;
            const double wnd = besselI0(d.beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(d.beta);
            h[static_cast<size_t>(k)] = v * wnd;
            total += v * wnd;
        }
        for (int j = 0; j < s.pairs; ++j)
        {
            const auto odd = static_cast<float>(h[static_cast<size_t>(centre + 2 * j + 1)] / total);
            s.w[static_cast<size_t>(pad + s.pairs + j)] = odd;
            s.w[static_cast<size_t>(pad + s.pairs - 1 - j)] = odd;
        }
    }

    // y[i] = sum over k < taps of w[k] * x[i + k], for i < n. The outputs are vectorised (sixteen at a time, four
    // independent accumulators), so there is no horizontal sum. Every output is the same sequence of adds whatever
    // the grouping: an accumulator from 0, then w[k] * x[i + k] for k = 0, 1, ...
    static void convolve(const float* w, int taps, const float* x, size_t n, float* y)
    {
        size_t i = 0;
#if CRUCIBLE_HALFBAND_AVX
        if (detail::halfBandAvx())
            i = detail::halfBandConvolveAvx(w, taps, x, n, y);
#endif
#if CRUCIBLE_SIMD_SSE_EXACT
        // MSVC left the loop below scalar. Here each lane is one output with the scalar loop's multiply and add per
        // tap (no FMA on this target): 32 outputs per pass (eight accumulators hide the add latency), then 4.
        for (; i + 32 <= n; i += 32)
        {
            const float* xi = x + i;
            __m128 a0 = _mm_setzero_ps(), a1 = a0, a2 = a0, a3 = a0, a4 = a0, a5 = a0, a6 = a0, a7 = a0;
            for (int k = 0; k < taps; ++k)
            {
                const __m128 wk = _mm_set1_ps(w[k]);
                const float* p = xi + k;
                a0 = _mm_add_ps(a0, _mm_mul_ps(wk, _mm_loadu_ps(p)));
                a1 = _mm_add_ps(a1, _mm_mul_ps(wk, _mm_loadu_ps(p + 4)));
                a2 = _mm_add_ps(a2, _mm_mul_ps(wk, _mm_loadu_ps(p + 8)));
                a3 = _mm_add_ps(a3, _mm_mul_ps(wk, _mm_loadu_ps(p + 12)));
                a4 = _mm_add_ps(a4, _mm_mul_ps(wk, _mm_loadu_ps(p + 16)));
                a5 = _mm_add_ps(a5, _mm_mul_ps(wk, _mm_loadu_ps(p + 20)));
                a6 = _mm_add_ps(a6, _mm_mul_ps(wk, _mm_loadu_ps(p + 24)));
                a7 = _mm_add_ps(a7, _mm_mul_ps(wk, _mm_loadu_ps(p + 28)));
            }
            _mm_storeu_ps(y + i, a0);
            _mm_storeu_ps(y + i + 4, a1);
            _mm_storeu_ps(y + i + 8, a2);
            _mm_storeu_ps(y + i + 12, a3);
            _mm_storeu_ps(y + i + 16, a4);
            _mm_storeu_ps(y + i + 20, a5);
            _mm_storeu_ps(y + i + 24, a6);
            _mm_storeu_ps(y + i + 28, a7);
        }
        for (; i + 4 <= n; i += 4)
        {
            const float* xi = x + i;
            __m128 a0 = _mm_setzero_ps();
            for (int k = 0; k < taps; ++k)
                a0 = _mm_add_ps(a0, _mm_mul_ps(_mm_set1_ps(w[k]), _mm_loadu_ps(xi + k)));
            _mm_storeu_ps(y + i, a0);
        }
#else
        // 32 outputs per pass: eight vector accumulators keep the multiply-add latency hidden, and each tap's weight is
        // broadcast once for all of them.
        for (; i + 32 <= n; i += 32)
        {
            float acc[32] = {};
            for (int k = 0; k < taps; ++k)
            {
                const float wk = w[k];
                for (int l = 0; l < 32; ++l)
                    acc[l] += wk * x[i + static_cast<size_t>(k) + static_cast<size_t>(l)];
            }
            for (int l = 0; l < 32; ++l)
                y[i + static_cast<size_t>(l)] = acc[l];
        }
        for (; i + 16 <= n; i += 16)
        {
            float acc[16] = {};
            for (int k = 0; k < taps; ++k)
            {
                const float wk = w[k];
                for (int l = 0; l < 16; ++l)
                    acc[l] += wk * x[i + static_cast<size_t>(k) + static_cast<size_t>(l)];
            }
            for (int l = 0; l < 16; ++l)
                y[i + static_cast<size_t>(l)] = acc[l];
        }
#endif
        for (; i < n; ++i)
        {
            float acc = 0.0f;
            for (int k = 0; k < taps; ++k)
                acc += w[k] * x[i + static_cast<size_t>(k)];
            y[i] = acc;
        }
    }

    void upStage(Stage& s, const float* in, size_t n, float* out)
    {
        float* work = s.upWork.data();
        const size_t hist = static_cast<size_t>(s.history);
        std::copy(in, in + n, work + hist);
        convolve(s.w.data(), s.window, work, n, even.data());
        // x[n - d] sits d = m behind the newest sample
        const size_t dly = static_cast<size_t>(s.m);
        size_t i = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
        const __m128 two = _mm_set1_ps(2.0f);
        for (; i + 4 <= n; i += 4)
        {
            const __m128 e = _mm_mul_ps(two, _mm_loadu_ps(even.data() + i));
            const __m128 o = _mm_loadu_ps(work + hist + i - dly);
            _mm_storeu_ps(out + 2 * i, _mm_unpacklo_ps(e, o));
            _mm_storeu_ps(out + 2 * i + 4, _mm_unpackhi_ps(e, o));
        }
#endif
        for (; i < n; ++i)
        {
            out[2 * i] = 2.0f * even[i];
            out[2 * i + 1] = work[hist + i - dly];
        }
        std::copy(work + n, work + n + hist, work);
    }

    void downStage(Stage& s, const float* in, size_t n, float* out)
    {
        float* odd = s.oddWork.data();
        float* evn = s.evenWork.data();
        const size_t hist = static_cast<size_t>(s.history), eh = static_cast<size_t>(s.m);
        size_t i = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
        for (; i + 4 <= n; i += 4)
        {
            const __m128 a = _mm_loadu_ps(in + 2 * i), b = _mm_loadu_ps(in + 2 * i + 4);
            _mm_storeu_ps(evn + eh + i, _mm_shuffle_ps(a, b, _MM_SHUFFLE(2, 0, 2, 0)));
            _mm_storeu_ps(odd + hist + i, _mm_shuffle_ps(a, b, _MM_SHUFFLE(3, 1, 3, 1)));
        }
#endif
        for (; i < n; ++i)
        {
            evn[eh + i] = in[2 * i];
            odd[hist + i] = in[2 * i + 1];
        }
        convolve(s.w.data(), s.window, odd, n, even.data());
        i = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
        const __m128 half = _mm_set1_ps(0.5f);
        for (; i + 4 <= n; i += 4)
            _mm_storeu_ps(out + i, _mm_add_ps(_mm_mul_ps(half, _mm_loadu_ps(evn + i)), _mm_loadu_ps(even.data() + i)));
#endif
        for (; i < n; ++i)
            out[i] = 0.5f * evn[i] + even[i];
        std::copy(odd + n, odd + n + hist, odd);
        std::copy(evn + n, evn + n + eh, evn);
    }

    int stages = 1, block = 512;
    int fineLatency = 0;
    std::array<Stage, kMaxStages> stage{};
    std::vector<float> scratchA, scratchB, even;
};

} // namespace crucible::fx
