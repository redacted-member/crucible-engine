// SPDX-License-Identifier: MIT
#include <crucible/fx/Loudness.h>
#include <crucible/Simd.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace crucible::fx
{
namespace
{
inline double finiteSample(float x)
{
    const double v = x;
    return (v == v && std::abs(v) < 1.0e18) ? v : 0.0;
}
} // namespace

void designKWeighting(double sampleRate, KWeightingCoefficients& shelf, KWeightingCoefficients& highpass)
{
    constexpr double pi = std::numbers::pi;
    {
        // Analog high shelf: 3.9998 dB at 1681.97 Hz, Q 0.7072 (the recommendation's stage 1).
        const double gain = 3.999843853973347, q = 0.7071752369554196, fc = 1681.974450955533;
        const double k = std::tan(pi * fc / sampleRate);
        const double vh = std::pow(10.0, gain / 20.0), vb = std::pow(vh, 0.4996667741545416);
        const double a0 = 1.0 + k / q + k * k;
        shelf.b0 = (vh + vb * k / q + k * k) / a0;
        shelf.b1 = 2.0 * (k * k - vh) / a0;
        shelf.b2 = (vh - vb * k / q + k * k) / a0;
        shelf.a1 = 2.0 * (k * k - 1.0) / a0;
        shelf.a2 = (1.0 - k / q + k * k) / a0;
    }
    {
        // Stage 2, the RLB weighting: a second-order high-pass at 38.1355 Hz, Q 0.5003.
        const double q = 0.5003270373238773, fc = 38.13547087602444;
        const double k = std::tan(pi * fc / sampleRate);
        const double a0 = 1.0 + k / q + k * k;
        highpass.b0 = 1.0;
        highpass.b1 = -2.0;
        highpass.b2 = 1.0;
        highpass.a1 = 2.0 * (k * k - 1.0) / a0;
        highpass.a2 = (1.0 - k / q + k * k) / a0;
    }
}

namespace
{
// A transposed direct form II biquad rewritten so that the output is not on the recursion: with y = b0 x + s1,
//   s1' = (b1 - a1 b0) x - a1 s1 + s2,  s2' = (b2 - a2 b0) x - a2 s1,
// which is the same filter, but the state recursion is one multiply-add deep instead of three.
struct FastBiquad
{
    double b0, k1, k2, a1, a2;
    explicit FastBiquad(const KWeightingCoefficients& c)
        : b0(c.b0), k1(c.b1 - c.a1 * c.b0), k2(c.b2 - c.a2 * c.b0), a1(c.a1), a2(c.a2)
    {
    }
};
} // namespace

double KWeighting::energy(const float* x, int n, double sum)
{
    const FastBiquad sh(shelf), hi(hp);
    double a1 = s1, a2 = s2, c1 = h1, c2 = h2;
    for (int i = 0; i < n; ++i)
    {
        const double v = finiteSample(x[i]);
        const double y = sh.b0 * v + a1;
        const double p = sh.k1 * v + a2;
        a2 = sh.k2 * v - sh.a2 * a1;
        a1 = p - sh.a1 * a1;
        const double z = y + c1;
        const double q = hi.k1 * y + c2;
        c2 = hi.k2 * y - hi.a2 * c1;
        c1 = q - hi.a1 * c1;
        sum += z * z;
    }
    s1 = a1;
    s2 = a2;
    h1 = c1;
    h2 = c2;
    return sum;
}

void KWeighting::energy(KWeighting& a, const float* xa, KWeighting& b, const float* xb, int n, double& energyA,
                        double& energyB)
{
    const FastBiquad sh(a.shelf), hi(a.hp); // both channels share the design
    double a1 = a.s1, a2 = a.s2, c1 = a.h1, c2 = a.h2, d1 = b.s1, d2 = b.s2, e1 = b.h1, e2 = b.h2;
    double sumA = energyA, sumB = energyB;
#if CRUCIBLE_SIMD_SSE_EXACT
    // x86 without FMA: the two channels in the lanes of an __m128d, the same multiplies, adds and subtractions per lane
    // (finiteSample as a mask: a NaN, an infinity or a value beyond 1e18 becomes +0).
    {
        const __m128d b0 = _mm_set1_pd(sh.b0), k1 = _mm_set1_pd(sh.k1), k2 = _mm_set1_pd(sh.k2), sa1 = _mm_set1_pd(sh.a1),
                      sa2 = _mm_set1_pd(sh.a2), hk1 = _mm_set1_pd(hi.k1), hk2 = _mm_set1_pd(hi.k2), ha1 = _mm_set1_pd(hi.a1),
                      ha2 = _mm_set1_pd(hi.a2), limit = _mm_set1_pd(1.0e18), sign = _mm_set1_pd(-0.0);
        __m128d s1 = _mm_set_pd(d1, a1), s2 = _mm_set_pd(d2, a2), h1 = _mm_set_pd(e1, c1), h2 = _mm_set_pd(e2, c2),
                sum = _mm_set_pd(sumB, sumA);
        for (int i = 0; i < n; ++i)
        {
            const __m128d x = _mm_cvtps_pd(_mm_unpacklo_ps(_mm_load_ss(xa + i), _mm_load_ss(xb + i)));
            const __m128d ok = _mm_and_pd(_mm_cmpord_pd(x, x), _mm_cmplt_pd(_mm_andnot_pd(sign, x), limit));
            const __m128d v = _mm_and_pd(ok, x);
            const __m128d y = _mm_add_pd(_mm_mul_pd(b0, v), s1);
            const __m128d p = _mm_add_pd(_mm_mul_pd(k1, v), s2);
            s2 = _mm_sub_pd(_mm_mul_pd(k2, v), _mm_mul_pd(sa2, s1));
            s1 = _mm_sub_pd(p, _mm_mul_pd(sa1, s1));
            const __m128d z = _mm_add_pd(y, h1);
            const __m128d q = _mm_add_pd(_mm_mul_pd(hk1, y), h2);
            h2 = _mm_sub_pd(_mm_mul_pd(hk2, y), _mm_mul_pd(ha2, h1));
            h1 = _mm_sub_pd(q, _mm_mul_pd(ha1, h1));
            sum = _mm_add_pd(sum, _mm_mul_pd(z, z));
        }
        _mm_storel_pd(&a1, s1);
        _mm_storeh_pd(&d1, s1);
        _mm_storel_pd(&a2, s2);
        _mm_storeh_pd(&d2, s2);
        _mm_storel_pd(&c1, h1);
        _mm_storeh_pd(&e1, h1);
        _mm_storel_pd(&c2, h2);
        _mm_storeh_pd(&e2, h2);
        _mm_storel_pd(&sumA, sum);
        _mm_storeh_pd(&sumB, sum);
        n = 0;
    }
#endif
    for (int i = 0; i < n; ++i)
    {
        const double va = finiteSample(xa[i]), vb = finiteSample(xb[i]);
        const double ya = sh.b0 * va + a1, yb = sh.b0 * vb + d1;
        const double pa = sh.k1 * va + a2, pb = sh.k1 * vb + d2;
        a2 = sh.k2 * va - sh.a2 * a1;
        d2 = sh.k2 * vb - sh.a2 * d1;
        a1 = pa - sh.a1 * a1;
        d1 = pb - sh.a1 * d1;
        const double za = ya + c1, zb = yb + e1;
        const double qa = hi.k1 * ya + c2, qb = hi.k1 * yb + e2;
        c2 = hi.k2 * ya - hi.a2 * c1;
        e2 = hi.k2 * yb - hi.a2 * e1;
        c1 = qa - hi.a1 * c1;
        e1 = qb - hi.a1 * e1;
        sumA += za * za;
        sumB += zb * zb;
    }
    a.s1 = a1;
    a.s2 = a2;
    a.h1 = c1;
    a.h2 = c2;
    b.s1 = d1;
    b.s2 = d2;
    b.h1 = e1;
    b.h2 = e2;
    energyA = sumA;
    energyB = sumB;
}

void KWeighting::prepare(double sampleRate)
{
    designKWeighting(sampleRate, shelf, hp);
    reset();
}

// ---------------------------------------------------------------------------------------------------------------
void LoudnessMeter::Histogram::clear()
{
    std::fill(count.begin(), count.end(), 0u);
    std::fill(energy.begin(), energy.end(), 0.0);
    total = 0;
    energySum = 0.0;
}

int LoudnessMeter::binOf(double lufs)
{
    const int b = static_cast<int>(std::floor((lufs - kBinLow) / kBinWidth));
    return std::clamp(b, 0, kBins - 1);
}

void LoudnessMeter::Histogram::add(double meanSquare)
{
    if (!(meanSquare > 0.0))
        return;
    const double l = -0.691 + 10.0 * std::log10(meanSquare);
    if (l <= kBinLow)
        return; // absolute gate
    const int b = binOf(l);
    ++count[static_cast<size_t>(b)];
    energy[static_cast<size_t>(b)] += meanSquare;
    ++total;
    energySum += meanSquare;
}

void LoudnessMeter::prepare(double sampleRate, int channels, bool truePeak)
{
    sr = sampleRate > 0.0 ? sampleRate : 48000.0;
    numChannels = std::clamp(channels, 1, kMaxChannels);
    wantTruePeak = truePeak;
    for (int c = 0; c < kMaxChannels; ++c)
    {
        kfilter[static_cast<size_t>(c)].prepare(sr);
        weight[static_cast<size_t>(c)] = 1.0;
        tp[static_cast<size_t>(c)].configure(4, true);
    }
    bound = tp[0].gainBound();
    holdDecay = static_cast<float>(std::exp(-1.0 / (1.5 * sr)));
    subLength = sr / 100.0;
    gated.prepare();
    ranged.prepare();
    reset();
}

void LoudnessMeter::reset()
{
    for (int c = 0; c < kMaxChannels; ++c)
    {
        kfilter[static_cast<size_t>(c)].reset();
        tp[static_cast<size_t>(c)].reset();
    }
    position = 0.0;
    nextEdge = subLength;
    subSum = 0.0;
    channelSum.fill(0.0);
    subCount = 0;
    ringSum.fill(0.0);
    ringCount.fill(0);
    ringHead = 0;
    subBlockCount = 0;
    momentary = shortTerm = kLoudnessFloorLufs;
    resetStatistics();
}

void LoudnessMeter::resetStatistics()
{
    gated.clear();
    ranged.clear();
    integrated = kLoudnessFloorLufs;
    momentaryMax = shortTermMax = kLoudnessFloorLufs;
    rangeLu = 0.0f;
    rangeLow = rangeHigh = kLoudnessFloorLufs;
    peaks.fill(0.0f);
    tpMax.fill(0.0f);
    hold.fill(0.0f);
    for (auto& g : groupMax)
        g.fill(0.0f);
    groupCount = 0;
    blockTp = blockPeak = 0.0f;
}

void LoudnessMeter::setChannelWeight(int channel, double w)
{
    if (channel >= 0 && channel < kMaxChannels)
        weight[static_cast<size_t>(channel)] = std::max(0.0, w);
}

void LoudnessMeter::process(const float* const* in, int n)
{
    float bp = 0.0f, bt = 0.0f;
    int done = 0;
    while (done < n)
    {
        // The samples up to the next 10 ms edge (at least one, as the edge test of a per-sample loop would close).
        const int toEdge = static_cast<int>(std::ceil(nextEdge - position));
        const int len = std::min(n - done, std::max(1, toEdge));

        // The energy of each channel is summed sample by sample across calls, so the result does not depend on how the
        // input is cut into blocks; the weights apply when the sub-block closes.
        int channel = 0;
        if (numChannels >= 2)
        {
            KWeighting::energy(kfilter[0], in[0] + done, kfilter[1], in[1] + done, len, channelSum[0], channelSum[1]);
            channel = 2;
        }
        for (; channel < numChannels; ++channel)
        {
            const auto cs = static_cast<size_t>(channel);
            channelSum[cs] = kfilter[cs].energy(in[channel] + done, len, channelSum[cs]);
        }
        subCount += len;

        // Peaks. The channels advance together (their interpolators are independent, so the processor overlaps them).
        struct Lane
        {
            float hold, top, g0, g1, g2, peak, blockTp;
        };
        std::array<Lane, kMaxChannels> lane;
        for (int c = 0; c < numChannels; ++c)
        {
            const auto cs = static_cast<size_t>(c);
            lane[cs] = {hold[cs], tpMax[cs], groupMax[cs][0], groupMax[cs][1], groupMax[cs][2], 0.0f, 0.0f};
        }
        int gc = groupCount;
        if (!wantTruePeak)
        {
            for (int c = 0; c < numChannels; ++c)
            {
                float peak = 0.0f;
                const float* x = in[c] + done;
                for (int i = 0; i < len; ++i)
                    peak = std::max(peak, static_cast<float>(std::abs(finiteSample(x[i]))));
                lane[static_cast<size_t>(c)].peak = peak;
            }
            gc = (gc + len) % 8;
        }
        else
        {
            for (int i = 0; i < len; ++i)
            {
                for (int c = 0; c < numChannels; ++c)
                {
                    const auto cs = static_cast<size_t>(c);
                    auto& l = lane[cs];
                    const double v = finiteSample(in[c][done + i]);
                    const float a = static_cast<float>(std::abs(v));
                    l.peak = std::max(l.peak, a);
                    l.g0 = std::max(l.g0, a);
                    float t = a;
                    // Interpolate only when the newest samples could reach the decaying peak (their largest magnitude
                    // times the interpolator's gain bound); anything lower cannot raise the maximum or the decaying value.
                    if (std::max(l.g0, std::max(l.g1, l.g2)) * bound > l.hold)
                        t = std::max(a, tp[cs].process(static_cast<float>(v)));
                    else
                        tp[cs].skip(static_cast<float>(v));
                    l.hold = std::max(t, l.hold * holdDecay);
                    l.hold = l.hold < 1.0e-12f ? 0.0f : l.hold;
                    l.top = std::max(l.top, t);
                    l.blockTp = std::max(l.blockTp, t);
                }
                if (++gc == 8)
                {
                    gc = 0;
                    for (int c = 0; c < numChannels; ++c)
                    {
                        auto& l = lane[static_cast<size_t>(c)];
                        l.g2 = l.g1;
                        l.g1 = l.g0;
                        l.g0 = 0.0f;
                    }
                }
            }
        }
        for (int c = 0; c < numChannels; ++c)
        {
            const auto cs = static_cast<size_t>(c);
            const auto& l = lane[cs];
            hold[cs] = l.hold;
            tpMax[cs] = l.top;
            groupMax[cs] = {l.g0, l.g1, l.g2};
            peaks[cs] = std::max(peaks[cs], l.peak);
            bp = std::max(bp, l.peak);
            bt = std::max(bt, l.blockTp);
        }
        groupCount = gc;

        done += len;
        position += len;
        if (position >= nextEdge)
        {
            subSum = 0.0;
            for (int c = 0; c < numChannels; ++c)
                subSum += weight[static_cast<size_t>(c)] * channelSum[static_cast<size_t>(c)];
            channelSum.fill(0.0);
            closeSubBlock();
            nextEdge += subLength;
        }
    }
    blockPeak = bp;
    blockTp = std::max(bt, bp);
}

double LoudnessMeter::windowMeanSquare(int blocks) const
{
    double sum = 0.0;
    long long count = 0;
    for (int k = 0; k < blocks; ++k)
    {
        const int idx = (ringHead - 1 - k + 2 * kShortTermBlocks) % kShortTermBlocks;
        sum += ringSum[static_cast<size_t>(idx)];
        count += ringCount[static_cast<size_t>(idx)];
    }
    return count > 0 ? sum / static_cast<double>(count) : 0.0;
}

void LoudnessMeter::closeSubBlock()
{
    ringSum[static_cast<size_t>(ringHead)] = subSum;
    ringCount[static_cast<size_t>(ringHead)] = subCount;
    ringHead = (ringHead + 1) % kShortTermBlocks;
    ++subBlockCount;
    subSum = 0.0;
    subCount = 0;

    const auto available = static_cast<int>(std::min<std::uint64_t>(subBlockCount, kShortTermBlocks));
    momentary = static_cast<float>(lufsOfMeanSquare(windowMeanSquare(std::min(available, kMomentaryBlocks))));
    shortTerm = static_cast<float>(lufsOfMeanSquare(windowMeanSquare(available)));
    if (subBlockCount >= kMomentaryBlocks)
        momentaryMax = std::max(momentaryMax, momentary);
    if (subBlockCount >= kShortTermBlocks)
        shortTermMax = std::max(shortTermMax, shortTerm);
    if (subBlockCount >= kMomentaryBlocks && (subBlockCount - kMomentaryBlocks) % 10 == 0)
    {
        gated.add(windowMeanSquare(kMomentaryBlocks));
        updateIntegrated();
    }
    if (subBlockCount >= kShortTermBlocks && (subBlockCount - kShortTermBlocks) % 10 == 0)
    {
        ranged.add(windowMeanSquare(kShortTermBlocks));
        updateRange();
    }
}

void LoudnessMeter::updateIntegrated()
{
    if (gated.total == 0)
    {
        integrated = kLoudnessFloorLufs;
        return;
    }
    const double threshold = -0.691 + 10.0 * std::log10(gated.energySum / static_cast<double>(gated.total)) - 10.0;
    const int first = std::clamp(static_cast<int>(std::ceil((threshold - kBinLow) / kBinWidth)), 0, kBins);
    double energy = 0.0;
    std::uint64_t count = 0;
    for (int b = first; b < kBins; ++b)
    {
        energy += gated.energy[static_cast<size_t>(b)];
        count += gated.count[static_cast<size_t>(b)];
    }
    integrated = count > 0 ? static_cast<float>(lufsOfMeanSquare(energy / static_cast<double>(count))) : kLoudnessFloorLufs;
}

void LoudnessMeter::updateRange()
{
    if (ranged.total == 0)
        return;
    const double threshold = -0.691 + 10.0 * std::log10(ranged.energySum / static_cast<double>(ranged.total)) - 20.0;
    const int first = std::clamp(static_cast<int>(std::ceil((threshold - kBinLow) / kBinWidth)), 0, kBins);
    std::uint64_t n = 0;
    for (int b = first; b < kBins; ++b)
        n += ranged.count[static_cast<size_t>(b)];
    if (n < 2)
    {
        rangeLu = 0.0f;
        return;
    }
    const auto percentile = [&](double p)
    {
        const auto rank = static_cast<std::uint64_t>(std::floor(p * static_cast<double>(n - 1) + 0.5));
        std::uint64_t seen = 0;
        for (int b = first; b < kBins; ++b)
        {
            seen += ranged.count[static_cast<size_t>(b)];
            if (seen > rank)
                return kBinLow + (static_cast<double>(b) + 0.5) * kBinWidth;
        }
        return kBinLow + (kBins - 0.5) * kBinWidth;
    };
    const double low = percentile(0.10), high = percentile(0.95);
    rangeLow = static_cast<float>(low);
    rangeHigh = static_cast<float>(high);
    rangeLu = static_cast<float>(std::max(0.0, high - low));
}

float LoudnessMeter::truePeakDb() const
{
    float m = 0.0f;
    for (int c = 0; c < numChannels; ++c)
        m = std::max(m, wantTruePeak ? std::max(tpMax[static_cast<size_t>(c)], peaks[static_cast<size_t>(c)])
                                     : peaks[static_cast<size_t>(c)]);
    return m > 1.0e-6f ? 20.0f * std::log10(m) : -120.0f;
}

float LoudnessMeter::samplePeakDb() const
{
    float m = 0.0f;
    for (int c = 0; c < numChannels; ++c)
        m = std::max(m, peaks[static_cast<size_t>(c)]);
    return m > 1.0e-6f ? 20.0f * std::log10(m) : -120.0f;
}

// ---------------------------------------------------------------------------------------------------------------
void PpmMeter::prepare(double sampleRate, PpmType type)
{
    groupLength = std::max(1, static_cast<int>(std::lround(0.0005 * sampleRate)));
    // About 10.75 ms of burst plus hold charge the stage to 89 % (-1 dB): 1 - exp(-10.75 ms / tau) = 0.891.
    const double tau = 0.01075 / std::log(1.0 / (1.0 - std::pow(10.0, -1.0 / 20.0)));
    rise = 1.0 - std::exp(-1.0 / (tau * sampleRate));
    const double dbPerSecond = type == PpmType::TypeI ? 20.0 / 1.5 : 24.0 / 2.8;
    fall = std::pow(10.0, -dbPerSecond / (20.0 * sampleRate));
    reset();
}

double VuMeter::naturalFrequency(double riseSeconds, double damping)
{
    // Step response of w0^2 / (s^2 + 2 zeta w0 s + w0^2) in the normalised time x = w0 t: the first x with y = 0.99.
    const double root = std::sqrt(1.0 - damping * damping), phase = std::acos(damping);
    double x = 0.0;
    for (; x < 40.0; x += 1.0e-3)
    {
        const double y = 1.0 - std::exp(-damping * x) / root * std::sin(root * x + phase);
        if (y >= 0.99)
            break;
    }
    return x / riseSeconds;
}

void VuMeter::prepare(double sampleRate)
{
    constexpr double zeta = 0.8; // step overshoot exp(-pi zeta / sqrt(1 - zeta^2)) = 1.5 %
    const double w0 = naturalFrequency(0.300, zeta);
    const double k = std::tan(w0 / (2.0 * sampleRate));
    const double norm = 1.0 / (1.0 + 2.0 * zeta * k + k * k);
    b0 = k * k * norm;
    b1 = 2.0 * b0;
    b2 = b0;
    a1 = 2.0 * (k * k - 1.0) * norm;
    a2 = (1.0 - 2.0 * zeta * k + k * k) * norm;
    reset();
}

} // namespace crucible::fx
