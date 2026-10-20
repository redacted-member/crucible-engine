// SPDX-License-Identifier: MIT
#include <crucible/Oversampler.h>
#include <crucible/Simd.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double besselI0(double x)
{
    double sum = 1.0;
    double term = 1.0;
    for (int k = 1; k < 60; ++k)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
    }
    return sum;
}

// Coefficients of the polyphase all-pass half-band filters (even entries run in one branch, odd in the other),
// designed for a 20 kHz pass band and a 28 kHz stop band at a 96 kHz stage rate by minimising the stop-band peak:
// about 65 dB (4), 96 dB (6) and 122 dB (8) of rejection.
constexpr std::array<double, 4> kIir4 = {0.08965279936788881, 0.3107971898806194, 0.5772094133980521, 0.850600009394436};
constexpr std::array<double, 6> kIir6 = {0.04413809392052576, 0.16397135710502875, 0.3302904125194603,
                                         0.5145431356697716,  0.7016255182888512,  0.8947170769567319};
constexpr std::array<double, 8> kIir8 = {0.02267514594189869, 0.08896622732883862, 0.1924908356302284, 0.32192921314336626,
                                         0.4646410633359093,  0.6115595439031657,  0.7601938839594213, 0.9157169952072117};
} // namespace

void Oversampler::prepare(int numStages, OversamplingFilter filter, Quality quality, int maxBlock)
{
    stages = std::clamp(numStages, 1, kMaxStages);
    kind = filter;
    const int q = quality == Quality::Draft ? 0 : (quality == Quality::Standard ? 1 : 2);
    for (auto& s : stage)
        s = Stage{};
    for (int i = 0; i < stages; ++i)
    {
        auto& s = stage[static_cast<size_t>(i)];
        if (kind == OversamplingFilter::HalfBand)
        {
            const int taps = q == 0 ? 23 : (q == 1 ? 47 : 95);
            const double beta = q == 0 ? 5.0 : (q == 1 ? 7.0 : 9.5);
            s.centre = (taps - 1) / 2;
            s.pairs = (s.centre + 1) / 2;
            std::vector<double> h(static_cast<size_t>(taps));
            double total = 0.0;
            for (int k = 0; k < taps; ++k)
            {
                const int n = k - s.centre;
                double v = 0.0;
                if (n == 0)
                    v = 0.5;
                else if (n % 2 != 0)
                    v = std::sin(kPi * n / 2.0) / (kPi * n);
                const double r = static_cast<double>(n) / s.centre;
                const double w = besselI0(beta * std::sqrt(std::max(0.0, 1.0 - r * r))) / besselI0(beta);
                h[static_cast<size_t>(k)] = v * w;
                total += v * w;
            }
            for (int j = 0; j < s.pairs; ++j)
                s.odd[static_cast<size_t>(j)] = static_cast<float>(h[static_cast<size_t>(s.centre + 2 * j + 1)] / total);
        }
        else
        {
            const double* c = q == 0 ? kIir4.data() : (q == 1 ? kIir6.data() : kIir8.data());
            s.coefficients = q == 0 ? 4 : (q == 1 ? 6 : 8);
            for (int k = 0; k < s.coefficients; ++k)
                s.a[static_cast<size_t>(k)] = static_cast<float>(c[k]);
        }
    }
    bufA.assign(static_cast<size_t>(std::max(maxBlock, 32) * factor() + 8), 0.0f);
    bufB.assign(bufA.size(), 0.0f);
    linA.assign(bufA.size() + kRing, 0.0f);
    linB.assign(bufA.size() + kRing, 0.0f);

    // Latency: run an impulse through up() and down() and find the peak of the response.
    std::vector<float> in(256, 0.0f), out(256, 0.0f), tmp(static_cast<size_t>(256 * factor()));
    in[0] = 1.0f;
    reset();
    for (int pos = 0; pos < 256; pos += 32)
    {
        up(in.data() + pos, 32, tmp.data());
        down(tmp.data(), 32, out.data() + pos);
    }
    size_t peak = 0;
    for (size_t i = 1; i < out.size(); ++i)
        if (std::abs(out[i]) > std::abs(out[peak]))
            peak = i;
    float frac = 0.0f;
    if (peak > 0 && peak + 1 < out.size())
    {
        const float a = out[peak - 1], b = out[peak], c = out[peak + 1];
        const float denom = a - 2.0f * b + c;
        frac = std::abs(denom) > 1.0e-9f ? 0.5f * (a - c) / denom : 0.0f;
    }
    measuredLatency = static_cast<float>(peak) + frac;
    reset();
}

void Oversampler::reset()
{
    for (auto& s : stage)
    {
        s.upBuffer.fill(0.0f);
        s.downBuffer.fill(0.0f);
        s.upPos = s.downPos = 0;
        s.upX.fill(0.0f);
        s.upY.fill(0.0f);
        s.downX.fill(0.0f);
        s.downY.fill(0.0f);
        s.downPrev = 0.0f;
    }
}

float Oversampler::allpass(const Stage& s, std::array<float, kMaxCoefficients>& x, std::array<float, kMaxCoefficients>& y,
                           int first, float v) const
{
    for (int k = first; k < s.coefficients; k += 2)
    {
        const auto i = static_cast<size_t>(k);
        const float out = s.a[i] * (v - y[i]) + x[i];
        x[i] = v;
        y[i] = out;
        v = out;
    }
    return v;
}

void Oversampler::upStage(Stage& s, const float* in, int n, float* out)
{
    if (kind == OversamplingFilter::Polyphase)
    {
        for (int i = 0; i < n; ++i)
        {
            // Branch states are kept apart by index parity: branch 0 uses even coefficients, branch 1 odd.
            out[2 * i] = allpass(s, s.upX, s.upY, 0, in[i]);
            out[2 * i + 1] = allpass(s, s.upX, s.upY, 1, in[i]);
        }
        return;
    }
    const int d = s.centre / 2;
#if CRUCIBLE_SIMD_SSE_EXACT
    // Four input samples at a time from a linear copy of the history and the input: lin[k] is the sample at index
    // upPos - hist + k. Each lane adds in the scalar loop's order (bit-identical); the ring is written as before.
    const int hist = d + s.pairs;
    if (n >= 4 && static_cast<size_t>(hist + n) <= linA.size())
    {
        using simd::Float4;
        float* lin = linA.data();
        for (int k = 0; k < hist; ++k)
            lin[k] = firAt(s.upBuffer, s.upPos - hist + k);
        std::copy(in, in + n, lin + hist);
        int i = 0;
        for (; i + 4 <= n; i += 4)
        {
            const float* at = lin + i + s.pairs; // lane l: the sample newest - d of input i + l
            Float4 acc = Float4::splat(0.0f);
            for (int j = 0; j < s.pairs; ++j)
                acc = Float4::mulAdd(acc, Float4::splat(s.odd[static_cast<size_t>(j)]),
                                     Float4::load(at + j) + Float4::load(at - 1 - j));
            const Float4 even = acc * Float4::splat(2.0f), odd = Float4::load(at);
            _mm_storeu_ps(out + 2 * i, _mm_unpacklo_ps(even.v, odd.v));
            _mm_storeu_ps(out + 2 * i + 4, _mm_unpackhi_ps(even.v, odd.v));
        }
        for (int k = 0; k < i; ++k)
        {
            s.upBuffer[static_cast<size_t>(s.upPos & (kRing - 1))] = in[k];
            ++s.upPos;
        }
        in += i;
        out += 2 * i;
        n -= i;
    }
#endif
    for (int i = 0; i < n; ++i)
    {
        s.upBuffer[static_cast<size_t>(s.upPos & (kRing - 1))] = in[i];
        ++s.upPos;
        const int newest = s.upPos - 1;
        out[2 * i + 1] = firAt(s.upBuffer, newest - d);
        float acc = 0.0f;
        for (int j = 0; j < s.pairs; ++j)
            acc +=
                s.odd[static_cast<size_t>(j)] * (firAt(s.upBuffer, newest - d + j) + firAt(s.upBuffer, newest - d - 1 - j));
        out[2 * i] = acc * 2.0f;
    }
}

void Oversampler::downStage(Stage& s, const float* in, int n, float* out)
{
    if (kind == OversamplingFilter::Polyphase)
    {
        for (int i = 0; i < n; ++i)
        {
            const float even = allpass(s, s.downX, s.downY, 0, in[2 * i]);
            const float odd = allpass(s, s.downX, s.downY, 1, s.downPrev);
            s.downPrev = in[2 * i + 1];
            out[i] = 0.5f * (even + odd);
        }
        return;
    }
#if CRUCIBLE_SIMD_SSE_EXACT
    // Four outputs at a time. The linear sequence (history, then the input) starts at index downPos - hist and is split
    // into its even and odd samples; hist = centre + 2 pairs - 2 puts every output's centre tap on an odd position, so
    // lane l of output i reads ev / od at i + l + offset. Each lane adds in the scalar loop's order (bit-identical).
    const int hist = s.centre + 2 * s.pairs - 2;
    if (n >= 4 && static_cast<size_t>((hist + 2 * n) / 2 + 1) <= linA.size())
    {
        using simd::Float4;
        float* ev = linA.data();
        float* od = linB.data();
        for (int k = 0; k < hist; ++k)
        {
            const float v = firAt(s.downBuffer, s.downPos - hist + k);
            if ((k & 1) == 0)
                ev[k >> 1] = v;
            else
                od[k >> 1] = v;
        }
        for (int k = 0; k < 2 * n; ++k)
        {
            const int p = hist + k;
            if ((p & 1) == 0)
                ev[p >> 1] = in[k];
            else
                od[p >> 1] = in[k];
        }
        const int c0 = s.pairs - 1; // od[i + c0] is the centre tap of output i
        int i = 0;
        for (; i + 4 <= n; i += 4)
        {
            Float4 acc = Float4::splat(0.5f) * Float4::load(od + i + c0);
            for (int j = 0; j < s.pairs; ++j)
                acc = Float4::mulAdd(acc, Float4::splat(s.odd[static_cast<size_t>(j)]),
                                     Float4::load(ev + i + c0 - j) + Float4::load(ev + i + c0 + 1 + j));
            acc.store(out + i);
        }
        for (int k = 0; k < 2 * i; ++k)
        {
            s.downBuffer[static_cast<size_t>(s.downPos & (kRing - 1))] = in[k];
            ++s.downPos;
        }
        in += 2 * i;
        out += i;
        n -= i;
    }
#endif
    for (int i = 0; i < n; ++i)
    {
        s.downBuffer[static_cast<size_t>(s.downPos & (kRing - 1))] = in[2 * i];
        ++s.downPos;
        s.downBuffer[static_cast<size_t>(s.downPos & (kRing - 1))] = in[2 * i + 1];
        ++s.downPos;
        const int c = s.downPos - 1 - s.centre;
        float acc = 0.5f * firAt(s.downBuffer, c);
        for (int j = 0; j < s.pairs; ++j)
            acc += s.odd[static_cast<size_t>(j)] *
                   (firAt(s.downBuffer, c - (2 * j + 1)) + firAt(s.downBuffer, c + (2 * j + 1)));
        out[i] = acc;
    }
}

void Oversampler::up(const float* in, int n, float* out)
{
    const float* src = in;
    int len = n;
    for (int i = 0; i < stages; ++i)
    {
        float* dst = (i == stages - 1) ? out : (i % 2 == 0 ? bufA.data() : bufB.data());
        upStage(stage[static_cast<size_t>(i)], src, len, dst);
        src = dst;
        len *= 2;
    }
}

void Oversampler::down(const float* in, int n, float* out)
{
    const float* src = in;
    int len = n << stages;
    for (int i = stages - 1; i >= 0; --i)
    {
        len /= 2;
        float* dst = (i == 0) ? out : (i % 2 == 0 ? bufA.data() : bufB.data());
        downStage(stage[static_cast<size_t>(i)], src, len, dst);
        src = dst;
    }
}

} // namespace crucible
