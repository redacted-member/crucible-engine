// SPDX-License-Identifier: MIT
#include <crucible/fx/Clipper.h>
#include <crucible/fx/ClipperEffect.h>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace crucible::fx
{

const LogCoshTable& LogCoshTable::instance()
{
    static const LogCoshTable table;
    return table;
}

LogCoshTable::LogCoshTable()
{
    const size_t n = static_cast<size_t>(kRange * kPerUnit) + 2;
    tanhValue.resize(n);
    tanhSlope.resize(n);
    lnValue.resize(n);
    lnSlope.resize(n);
    intValue.resize(n);
    for (size_t j = 0; j < n; ++j)
    {
        const double u = static_cast<double>(j) / kPerUnit;
        const double t = std::tanh(u);
        tanhValue[j] = t;
        tanhSlope[j] = 1.0 - t * t;
        lnValue[j] = u + std::log1p(std::exp(-2.0 * u)) - 0.69314718055994531;
        lnSlope[j] = t;
    }
    // G(u): composite Simpson over each 1/32 step with 16 sub-intervals.
    intValue[0] = 0.0;
    const auto lnCosh = [](double u) { return u + std::log1p(std::exp(-2.0 * u)) - 0.69314718055994531; };
    for (size_t j = 1; j < n; ++j)
    {
        const double a = static_cast<double>(j - 1) / kPerUnit, b = static_cast<double>(j) / kPerUnit;
        constexpr int sub = 16;
        const double h = (b - a) / sub;
        double acc = lnCosh(a) + lnCosh(b);
        for (int k = 1; k < sub; ++k)
            acc += lnCosh(a + k * h) * (k % 2 == 1 ? 4.0 : 2.0);
        intValue[j] = intValue[j - 1] + acc * h / 3.0;
    }
}

// ---------------------------------------------------------------------------------------------------------------
void ClipperEffect::prepare(double sampleRate, int maxBlock, Quality quality, OversamplingFilter filter)
{
    sr = sampleRate > 0.0 ? sampleRate : 48000.0;
    // Below 48 kHz the first half-band stage's transition band reaches the 5th harmonic of a 5 kHz tone (25 kHz folds to
    // 19 kHz at 44.1 kHz): the steeper filter one quality step up is used there for Standard.
    const Quality osQuality = (sr < 47999.0 && quality == Quality::Standard) ? Quality::High : quality;
    for (int i = 0; i < 3; ++i)
        blocks[static_cast<size_t>(i)].prepare(2, i + 1, filter, osQuality, std::max(1, maxBlock));
    guardLimiter.prepare(sr, TruePeakLimiter::Detect::TruePeak, 1);
    eqScratch.assign((static_cast<size_t>(std::max(512, maxBlock)) * 8) + 64, 0.0f);
    for (auto* v : {&ctlDrive, &ctlInv, &ctlCeil, &ctlSoft})
        v->assign(eqScratch.size(), 0.0f);
    for (int order = 1; order <= 2; ++order)
        for (int stages = 0; stages < 4; ++stages)
            equalisers[static_cast<size_t>(order)][static_cast<size_t>(stages)] = designEqualiser(order, stages, sr);
    int worst = guardLimiter.latencyFor(LimiterSettings{.lookaheadMs = kGuardLookaheadMs});
    for (const auto& b : blocks)
        worst =
            std::max(worst, b.latencySamples() + guardLimiter.latencyFor(LimiterSettings{.lookaheadMs = kGuardLookaheadMs}));
    dry.prepare(2, worst + 16, sr);
    started = false;
    active = -1;
    latency.store(0, std::memory_order_relaxed);
    reset();
}

void ClipperEffect::reset()
{
    for (auto& b : blocks)
        b.reset();
    for (auto& c : clippers)
        c.reset();
    for (auto& h : eqHistory)
        h.fill(0.0f);
    guardLimiter.reset();
    dry.reset();
    smoothInit = false;
}

// Least-squares FIR for the inverse of the ADAA kernel. Order 2: K(w) = (1 + 2 cos w) / 3, an odd-length symmetric FIR of 9
// taps (basis cos(k w), delay 4). Order 1: K(w) = cos(w / 2) with the half-sample delay of the two-point mean, an
// even-length FIR of 10 taps (basis cos((k + 1/2) w), delay 4.5 - the kernel's half sample makes the sum an integer).
ClipperEffect::Equaliser ClipperEffect::designEqualiser(int order, int stages, double sampleRate)
{
    Equaliser out;
    if (order <= 0)
        return out;
    const double factor = static_cast<double>(1 << stages);
    const auto kernel = [&](double w) { return order >= 2 ? (1.0 + 2.0 * std::cos(w)) / 3.0 : std::cos(0.5 * w); };
    // Fit up to 21.5 kHz (45 % of the rate when that is lower) at the base rate, but not past the point where the kernel has
    // lost 6 dB.
    double wMax = 2.0 * std::numbers::pi * std::min(21500.0, 0.45 * sampleRate) / (sampleRate * factor);
    while (wMax > 0.01 && kernel(wMax) < 0.5)
        wMax *= 0.98;
    const int unknowns = 5; // cos(k w), k = 0..4 (order 2) / cos((k + 1/2) w), k = 0..4 (order 1)
    const double offset = order >= 2 ? 0.0 : 0.5;
    // Past the fitted band the target is held at its last value with a small weight, so that the filter does not ring wildly
    // where the kernel has a zero.
    constexpr int grid = 512;
    double a[5][6] = {};
    const double edge = 1.0 / kernel(wMax);
    for (int g = 0; g < grid; ++g)
    {
        const double w = std::numbers::pi * (static_cast<double>(g) + 0.5) / grid;
        const bool inBand = w <= wMax;
        const double target = inBand ? 1.0 / kernel(w) : edge;
        const double weight = inBand ? 1.0 : 1.0e-4;
        double basis[5];
        for (int k = 0; k < unknowns; ++k)
            basis[k] = std::cos((static_cast<double>(k) + offset) * w);
        for (int r = 0; r < unknowns; ++r)
        {
            for (int c = 0; c < unknowns; ++c)
                a[r][c] += weight * basis[r] * basis[c];
            a[r][5] += weight * basis[r] * target;
        }
    }
    for (int r = 0; r < unknowns; ++r)
        a[r][r] += 1.0e-9;
    // Gaussian elimination.
    for (int col = 0; col < unknowns; ++col)
    {
        int pivot = col;
        for (int r = col + 1; r < unknowns; ++r)
            if (std::abs(a[r][col]) > std::abs(a[pivot][col]))
                pivot = r;
        for (int c = 0; c < 6; ++c)
            std::swap(a[col][c], a[pivot][c]);
        for (int r = 0; r < unknowns; ++r)
        {
            if (r == col)
                continue;
            const double f = a[r][col] / a[col][col];
            for (int c = col; c < 6; ++c)
                a[r][c] -= f * a[col][c];
        }
    }
    double h[5];
    for (int k = 0; k < unknowns; ++k)
        h[k] = a[k][5] / a[k][k];
    // Response sum_k c_k cos((k + offset) w) with c_0 = h_0 for order 2 (centre tap) and every other coefficient doubled by
    // the symmetric pair; order 1: each term is a symmetric pair.
    if (order >= 2)
    {
        out.length = 9;
        out.taps[4] = static_cast<float>(h[0]);
        for (int k = 1; k < 5; ++k)
            out.taps[static_cast<size_t>(4 - k)] = out.taps[static_cast<size_t>(4 + k)] = static_cast<float>(0.5 * h[k]);
    }
    else
    {
        out.length = 10;
        for (int k = 0; k < 5; ++k)
            out.taps[static_cast<size_t>(4 - k)] = out.taps[static_cast<size_t>(5 + k)] = static_cast<float>(0.5 * h[k]);
    }
    return out;
}

// Below 48 kHz the Standard 2x setting runs 4x with the High filter (see prepare()).
int ClipperEffect::stagesFor(const ClipperSettings& s) const
{
    const int stages = std::clamp(s.oversampleStages, 0, 3);
    return (sr < 47999.0 && stages == 1) ? 2 : stages;
}

int ClipperEffect::latencyFor(const ClipperSettings& s) const
{
    const int stages = stagesFor(s);
    // ADAA sits half (first order) or one (second order) sample of the rate it runs at behind its input; the equaliser adds
    // 4 or 4.5 samples, so both orders total 5 (whole).
    const double adaa = s.adaaOrder >= 1 ? 5.0 : 0.0;
    const double os = stages == 0 ? 0.0 : static_cast<double>(blocks[static_cast<size_t>(stages - 1)].latency());
    LimiterSettings guard;
    guard.lookaheadMs = kGuardLookaheadMs;
    return static_cast<int>(std::lround(os + adaa / static_cast<double>(1 << stages))) +
           (s.guard ? guardLimiter.latencyFor(guard) : 0);
}

void ClipperEffect::process(float* left, float* right, int n, const ClipperSettings& settings)
{
    // At 1x the second-order kernel has a zero at 16 kHz that no equaliser can undo: it runs as first order there.
    ClipperSettings s = settings;
    s.adaaOrder = std::clamp(s.adaaOrder, 0, s.oversampleStages <= 0 ? 1 : 2);
    const int stages = stagesFor(s);
    if (!started || stages != active || s.guard != guardWasOn)
    {
        if (started)
        {
            // A new signal path starts from silence; the dry line keeps playing.
            for (auto& c : clippers)
                c.reset();
            for (auto& h : eqHistory)
                h.fill(0.0f);
            if (stages != active && stages > 0)
                blocks[static_cast<size_t>(stages - 1)].reset();
            if (s.guard && !guardWasOn)
                guardLimiter.reset();
        }
        active = stages;
        guardWasOn = s.guard;
        started = true;
    }
    const int total = latencyFor(s);
    dry.setDelay(total);
    if (latency.exchange(total, std::memory_order_relaxed) != total)
        latencyChanged.store(true, std::memory_order_relaxed);

    const float mix = std::clamp(s.mix, 0.0f, 1.0f);
    const double driveTarget = std::clamp(s.driveDb, 0.0f, 36.0f);
    const double ceilTarget = std::clamp(s.ceilingDb, -60.0f, 0.0f);
    const double softTarget = std::clamp(s.softness, 0.0f, 1.0f);
    if (!smoothInit)
    {
        driveSm.set(driveTarget);
        ceilSm.set(ceilTarget);
        softSm.set(softTarget);
        mixSm.set(mix);
        smoothInit = true;
    }
    // About 4 ms per pole at the rate each control runs at.
    constexpr double kSmoothSeconds = 0.004;
    const double baseCoef = 1.0 - std::exp(-1.0 / (kSmoothSeconds * sr));
    const double osCoef = 1.0 - std::exp(-1.0 / (kSmoothSeconds * sr * static_cast<double>(1 << stages)));
    constexpr double kDbToLin = 0.11512925464970229;
    float dryL[512], dryR[512];
    for (int base = 0; base < n; base += 512)
    {
        const int m = std::min(512, n - base);
        float* io[2] = {left + base, right + base};
        std::copy(io[0], io[0] + m, dryL);
        std::copy(io[1], io[1] + m, dryR);
        float* dryChans[2] = {dryL, dryR};
        dry.process(dryChans, m);

        const float ceil = std::pow(10.0f, std::clamp(s.ceilingDb, -60.0f, 0.0f) / 20.0f);
        const float drive = std::pow(10.0f, std::clamp(s.driveDb, 0.0f, 36.0f) / 20.0f);
        const float inv = 1.0f / ceil;
        // Settled controls take the block-constant path below (bit-identical to a clipper without smoothing).
        const double ceilBefore = ceilSm.s3;
        const bool moving = !(driveSm.at(driveTarget) && ceilSm.at(ceilTarget) && softSm.at(softTarget));
        if (!moving)
            for (auto& c : clippers)
                c.setSoftness(s.softness);
        const Equaliser& eq = equalisers[static_cast<size_t>(std::clamp(s.adaaOrder, 0, 2))][static_cast<size_t>(stages)];
        const auto kernel = [&](float* const* x, int channels, int count)
        {
            if (moving)
                for (int i = 0; i < count; ++i)
                {
                    const size_t k = static_cast<size_t>(i);
                    const double d = driveSm.step(driveTarget, osCoef);
                    const double cl = ceilSm.step(ceilTarget, osCoef);
                    ctlSoft[k] = static_cast<float>(softSm.step(softTarget, osCoef));
                    ctlDrive[k] = static_cast<float>(std::exp(d * kDbToLin));
                    ctlCeil[k] = static_cast<float>(std::exp(cl * kDbToLin));
                    ctlInv[k] = 1.0f / ctlCeil[k];
                }
            for (int c = 0; c < channels; ++c)
            {
                float* data = x[c];
                if (moving)
                    for (int i = 0; i < count; ++i)
                    {
                        const size_t k = static_cast<size_t>(i);
                        clippers[c].setSoftness(ctlSoft[k]);
                        data[i] = clippers[c].process(data[i] * ctlDrive[k] * ctlInv[k], s.adaaOrder) * ctlCeil[k];
                    }
                else
                    for (int i = 0; i < count; ++i)
                        data[i] = clippers[c].process(data[i] * drive * inv, s.adaaOrder) * ceil;
                if (eq.length == 0)
                    continue;
                // Symmetric FIR over the block with the previous inputs in front.
                auto& hist = eqHistory[static_cast<size_t>(c)];
                const int past = eq.length - 1;
                float* buf = eqScratch.data();
                for (int k = 0; k < past; ++k)
                    buf[k] = hist[static_cast<size_t>(k)];
                std::copy(data, data + count, buf + past);
                for (int k = 0; k < past; ++k)
                    hist[static_cast<size_t>(k)] = buf[count + k];
                const float* tap = eq.taps.data();
                int start = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
                // Four outputs at a time, each with the scalar loop's order of operations (bit-identical; MSVC does not
                // vectorise the loops below).
                {
                    using simd::Float4;
                    for (; start + 4 <= count; start += 4)
                    {
                        const float* src = buf + past + start;
                        Float4 acc = Float4::splat(tap[0]) * Float4::load(src);
                        for (int k = 1; k < eq.length; ++k)
                            acc = Float4::mulAdd(acc, Float4::splat(tap[k]), Float4::load(src - k));
                        acc.store(data + start);
                    }
                }
#endif
                for (int i = start; i < count; ++i)
                    data[i] = tap[0] * buf[past + i];
                for (int k = 1; k < eq.length; ++k)
                    for (int i = start; i < count; ++i)
                        data[i] += tap[k] * buf[past + i - k];
            }
        };
        if (stages == 0)
            kernel(io, 2, m);
        else
            blocks[static_cast<size_t>(stages - 1)].process(io, m, 1.0f, kernel);

        if (s.guard)
        {
            LimiterSettings guard;
            // While the ceiling moves the guard sits 3 dB above the highest level the clipper used in the block: the clipper already
            // holds the moving level per sample, and a guard ceiling that follows it in block steps is a zipper.
            guard.ceilingDb = ceilSm.at(ceilTarget) && ceilBefore == ceilTarget
                                  ? s.ceilingDb
                                  : static_cast<float>(std::min(0.0, std::max(ceilBefore, ceilSm.s3) + 3.0));
            guard.lookaheadMs = kGuardLookaheadMs;
            guard.releaseMs = 50.0f;
            guard.releaseMode = ReleaseMode::Fixed;
            guard.holdMs = 0.0f;
            guard.autoHold = false;
            guardLimiter.process(io[0], io[1], m, guard);
        }
        if (!mixSm.at(mix))
            for (int i = 0; i < m; ++i)
            {
                const float k = static_cast<float>(mixSm.step(mix, baseCoef));
                io[0][i] = dryL[i] + (io[0][i] - dryL[i]) * k;
                io[1][i] = dryR[i] + (io[1][i] - dryR[i]) * k;
            }
        else if (mix < 0.9999f)
            for (int i = 0; i < m; ++i)
            {
                io[0][i] = dryL[i] + (io[0][i] - dryL[i]) * mix;
                io[1][i] = dryR[i] + (io[1][i] - dryR[i]) * mix;
            }
    }
}

} // namespace crucible::fx
