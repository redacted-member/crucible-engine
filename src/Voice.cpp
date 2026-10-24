// SPDX-License-Identifier: MIT
#include <crucible/Voice.h>

#include <crucible/Modulation.h>
#include <crucible/Simd.h>
#include <crucible/Spectral.h>

#include <cstring>

namespace crucible
{
namespace
{
constexpr float kSqrt2 = 1.41421356f;
constexpr float kPi = 3.14159265f;

float noteToHz(float note)
{
    return 440.0f * std::exp2((note - 69.0f) / 12.0f);
}

// One wavetable position: two neighbouring frames of one mip level.
struct Wt
{
    const float* a = nullptr;
    const float* b = nullptr;
    float size = 2048.0f;
    int isize = 2048;
    int mip = 0;
    float mix = 0.0f;
    float mixStep = 0.0f; // per sample: the frame position moves smoothly instead of in steps per chunk
    bool blend = false;   // the second frame takes part (mix or mixStep not zero)
    void advance() { mix += mixStep; }

    // 6-point, 5th-order Lagrange interpolation of both frames with shared weights. The tables carry enough samples
    // per cycle (see WavetableBank) that the interpolation error stays below -120 dB.
    CRUCIBLE_ALWAYS_INLINE float read(float ph) const
    {
        ph -= std::floor(ph);
        const float x = ph * size;
        const int n = std::min(static_cast<int>(x), isize - 1);
        const float f = x - static_cast<float>(n);
        const float d0 = f + 2.0f, d1 = f + 1.0f, d2 = f, d3 = f - 1.0f, d4 = f - 2.0f, d5 = f - 3.0f;
        const float s5 = d5, s4 = d4 * d5, s3 = d3 * s4, s2 = d2 * s3, s1 = d1 * s2;
        const float p2 = d0 * d1, p3 = p2 * d2, p4 = p3 * d3, p5 = p4 * d4;
        const float w0 = s1 * (-1.0f / 120.0f);
        const float w1 = (d0 * s2) * (1.0f / 24.0f);
        const float w2 = (p2 * s3) * (-1.0f / 12.0f);
        const float w3 = (p3 * s4) * (1.0f / 12.0f);
        const float w4 = (p4 * s5) * (-1.0f / 24.0f);
        const float w5 = p5 * (1.0f / 120.0f);
        const float* pa = a + n;
        const float sa = w0 * pa[-2] + w1 * pa[-1] + w2 * pa[0] + w3 * pa[1] + w4 * pa[2] + w5 * pa[3];
        if (!blend)
            return sa;
        const float* pb = b + n;
        const float sb = w0 * pb[-2] + w1 * pb[-1] + w2 * pb[0] + w3 * pb[1] + w4 * pb[2] + w5 * pb[3];
        return sa + (sb - sa) * mix;
    }

    // Derivative with respect to the phase (cycles), by a central difference of one table sample.
    float slope(float ph) const
    {
        const float e = 1.0f / size;
        return (read(ph + e) - read(ph - e)) * (0.5f * size);
    }
};

// Silent frame with guard samples, for user slots that hold no table.
constexpr std::array<float, WavetableBank::maxMipSize + WavetableBank::guardBefore + WavetableBank::guardAfter> kSilent{};

const float* silentFrame()
{
    return kSilent.data() + WavetableBank::guardBefore;
}

// One cache entry of the spectral path holds a cycle of the largest mip with its guard samples.
constexpr int kSpecStride = spectral::kMaxSynthSize + 8;

// plainIncrement (cycles per output sample, 0 = not given) is the increment of a plain, non-oversampled read of the same
// pitch. A zoned user table assigns its zones per mip assuming that read, so an oversampled one (which asks for more
// partials) never reads a finer mip than the plain read would: the zone stays the one meant for the note.
Wt makeWt(int table, float position, float previousPosition, int samples, float increment, float limit,
          const UserWavetableBank::Pinned* user, float plainIncrement = 0.0f, double outputRate = 0.0)
{
    const auto& bank = WavetableBank::instance();
    const int t = std::clamp(table, 0, kTotalTables - 1);
    // The frame pair is chosen at the middle of the chunk and the mix ramps from the previous position to the new
    // one, so the morph is smooth (the pair is allowed to extrapolate by a fraction of a frame at its ends).
    const float last = static_cast<float>(WavetableBank::numFrames - 1);
    const float framePos0 = std::clamp(previousPosition, 0.0f, 1.0f) * last;
    const float framePos1 = std::clamp(position, 0.0f, 1.0f) * last;
    const int frameA = std::min(static_cast<int>(0.5f * (framePos0 + framePos1)), WavetableBank::numFrames - 2);
    int mip = WavetableBank::mipForIncrement(increment, limit);
    const UserWavetable* u = t >= WavetableBank::numTables && user != nullptr
                                 ? (*user)[static_cast<size_t>(t - WavetableBank::numTables)]
                                 : nullptr;
    if (u != nullptr && u->zoneCount() >= 2 && plainIncrement > 0.0f)
        mip = std::max(mip, WavetableBank::mipForIncrement(plainIncrement, WavetableBank::outputBandLimit(outputRate)));
    Wt w;
    w.mip = mip;
    if (t < WavetableBank::numTables)
    {
        // A factory table that no control-thread call has generated yet is silent rather than generated here.
        const bool ready = bank.resident(t);
        w.a = ready ? bank.frame(t, mip, frameA) : silentFrame();
        w.b = ready ? bank.frame(t, mip, frameA + 1) : silentFrame();
    }
    else
    {
        w.a = u != nullptr ? u->frame(mip, frameA) : silentFrame();
        w.b = u != nullptr ? u->frame(mip, frameA + 1) : silentFrame();
    }
    w.isize = WavetableBank::mipSize(mip);
    w.size = static_cast<float>(w.isize);
    w.mix = framePos0 - static_cast<float>(frameA);
    w.mixStep = (framePos1 - framePos0) / static_cast<float>(std::max(samples, 1));
    w.blend = w.mix != 0.0f || w.mixStep != 0.0f;
    return w;
}

struct WarpCtx
{
    float p0 = 0.0f;
    float p1 = 0.0f;
    float p2 = 0.0f;
    // Largest factor by which the warp speeds up the wavetable read; selects the mip level.
    float rateScale = 1.0f;
    float ratio = 1.0f;
    int oversample = 1;
};

WarpCtx makeWarpCtx(WarpMode mode, float a, float b)
{
    WarpCtx c;
    c.oversample = 2;
    switch (mode)
    {
    case WarpMode::Fm:
        c.p0 = a * 2.0f;
        c.ratio = 1.0f + std::round(b * 7.0f);
        c.rateScale = std::min(1.0f + 6.2832f * c.p0 * c.ratio, 64.0f);
        c.oversample = 4;
        break;
    case WarpMode::Ring:
        c.p0 = a;
        c.ratio = 1.0f + std::round(b * 7.0f);
        c.oversample = 4;
        break;
    case WarpMode::SyncHard:
    case WarpMode::SyncSoft:
        c.p0 = 1.0f + a * 7.0f;
        c.p1 = b;
        c.rateScale = c.p0;
        break;
    case WarpMode::BendPlus:
        c.p0 = std::exp(a * 1.6f);
        c.p1 = b;
        c.rateScale = std::min(c.p0, 8.0f);
        break;
    case WarpMode::BendMinus:
        c.p0 = std::exp(-a * 1.6f);
        c.p1 = b;
        c.rateScale = std::min(1.0f / c.p0, 8.0f);
        break;
    case WarpMode::Mirror:
        c.p0 = 1.0f + a * 3.0f;
        c.p1 = b;
        c.rateScale = c.p0;
        break;
    case WarpMode::Pwm:
        c.p0 = 0.5f - 0.45f * a;
        c.p1 = b;
        c.rateScale = std::max(0.5f / c.p0, 0.5f / (1.0f - c.p0));
        break;
    case WarpMode::Quantize:
        c.p0 = 0.5f * (2.0f + 126.0f * (1.0f - a) * (1.0f - a));
        c.p1 = 8.0f + 4088.0f * (1.0f - b) * (1.0f - b);
        c.oversample = 4;
        break;
    case WarpMode::Asym:
        c.p0 = 1.0f + 3.0f * a;
        c.p1 = 1.0f + 3.0f * b;
        c.oversample = 4;
        break;
    case WarpMode::Flip:
        c.p0 = 1.0f - 0.95f * a;
        c.p1 = 0.01f + 0.2f * b;
        c.oversample = 4;
        break;
    case WarpMode::Formant:
        c.p0 = 1.0f + a * 15.0f;
        c.p1 = std::min(1.0f, a * 4.0f);
        c.p2 = b;
        c.rateScale = 1.0f + (c.p0 - 1.0f) * c.p1;
        break;
    case WarpMode::Smear:
        c.p0 = a * 0.3f / 3.0f;
        c.p1 = b;
        break;
    case WarpMode::Fold:
        c.p0 = 1.0f + a * 8.0f;
        c.p1 = b;
        c.oversample = 4;
        break;
    case WarpMode::Off:
    case WarpMode::Count:
        c.oversample = 1;
        break;
    }
    return c;
}

float foldWave(float x)
{
    x += 1.0f;
    x -= 4.0f * std::floor(x * 0.25f);
    if (x > 2.0f)
        x = 4.0f - x;
    return x - 1.0f;
}

// Reflects x back and forth between 0 and 1. Once the phase actually reaches a fold point the corner is
// rounded (quadratic blend over k) so the warped waveform has no slope discontinuity to alias.
float triFold(float x, bool smooth)
{
    const float h = x * 0.5f;
    const float u = (h - std::floor(h)) * 2.0f;
    const float d = u < 1.0f ? u : 2.0f - u;
    if (!smooth)
        return d;
    constexpr float k = 0.08f;
    if (d < k)
        return d * d / (2.0f * k) + 0.5f * k;
    if (d > 1.0f - k)
    {
        const float e = 1.0f - d;
        return 1.0f - (e * e / (2.0f * k) + 0.5f * k);
    }
    return d;
}

template <WarpMode M> inline float warpRead(const Wt& w, float ph, const WarpCtx& c, float mod)
{
    if constexpr (M == WarpMode::Off || M == WarpMode::SyncHard || M == WarpMode::SyncSoft)
    {
        return w.read(ph);
    }
    else if constexpr (M == WarpMode::Fm)
    {
        return w.read(ph + c.p0 * mod);
    }
    else if constexpr (M == WarpMode::BendPlus || M == WarpMode::BendMinus)
    {
        const float pw = std::pow(ph, c.p0);
        if (c.p1 <= 0.0f)
            return w.read(pw);
        const float sym = ph < 0.5f ? 0.5f * std::pow(2.0f * ph, c.p0) : 1.0f - 0.5f * std::pow(2.0f - 2.0f * ph, c.p0);
        return w.read(pw + c.p1 * (sym - pw));
    }
    else if constexpr (M == WarpMode::Mirror)
    {
        return w.read(triFold(ph * c.p0 + c.p1, c.p0 > 1.0001f || c.p1 > 0.0f));
    }
    else if constexpr (M == WarpMode::Pwm)
    {
        float p2 = ph + c.p1;
        p2 -= std::floor(p2);
        const float d = c.p0;
        return w.read(p2 < d ? 0.5f * p2 / d : 0.5f + 0.5f * (p2 - d) / (1.0f - d));
    }
    else if constexpr (M == WarpMode::Quantize)
    {
        const float q = std::floor(ph * c.p1) / c.p1;
        return std::round(w.read(q) * c.p0) / c.p0;
    }
    else if constexpr (M == WarpMode::Asym)
    {
        const float s = std::clamp(w.read(ph), -1.5f, 1.5f);
        return s >= 0.0f ? std::pow(s, c.p0) : -std::pow(-s, c.p1);
    }
    else if constexpr (M == WarpMode::Flip)
    {
        const float x = std::clamp((ph - c.p0) / c.p1, 0.0f, 1.0f);
        return (1.0f - 2.0f * (x * x * (3.0f - 2.0f * x))) * w.read(ph);
    }
    else if constexpr (M == WarpMode::Formant)
    {
        const float dry = w.read(ph);
        if (c.p1 <= 0.0f)
            return dry;
        const float win = 0.5f - 0.5f * std::cos(2.0f * kPi * ph);
        const float shaped = win + (win * win - win) * c.p2;
        const float wet = w.read(ph * c.p0) * shaped;
        return dry + (wet - dry) * c.p1;
    }
    else if constexpr (M == WarpMode::Smear)
    {
        if (c.p0 <= 0.0f)
            return w.read(ph);
        const float t = c.p1;
        const float w0 = 0.25f + (0.55f - 0.25f) * t;
        const float w1 = 0.25f + (0.30f - 0.25f) * t;
        const float w2 = 0.25f + (0.12f - 0.25f) * t;
        const float w3 = 0.25f + (0.03f - 0.25f) * t;
        return w0 * w.read(ph) + w1 * w.read(ph + c.p0) + w2 * w.read(ph + 2.0f * c.p0) + w3 * w.read(ph + 3.0f * c.p0);
    }
    else if constexpr (M == WarpMode::Ring)
    {
        return w.read(ph) * (1.0f - c.p0 + c.p0 * mod);
    }
    else if constexpr (M == WarpMode::Fold)
    {
        return foldWave(c.p0 * w.read(ph) + c.p1);
    }
    else
    {
        return w.read(ph);
    }
}

struct UnisonRun
{
    Wt wt;
    float inc = 0.0f;
    float gl = 0.0f;
    float gr = 0.0f;
    WarpCtx ctx;
};

template <WarpMode M>
void runUnison(const UnisonRun& u, float& phase, float& prev, MinBlepBuffer* blep, float* outL, float* outR, int m,
               const float* modBuf)
{
    Wt wt = u.wt;
    float ph = phase;
    if constexpr (M == WarpMode::SyncHard || M == WarpMode::SyncSoft)
    {
        // The slave reads the table at r times the master phase and restarts with the master. The step and the
        // slope break at each restart are corrected with a minBLEP / minBLAMP.
        const float r = u.ctx.p0;
        const float off = u.ctx.p1;
        const auto slave = [&](float master)
        {
            if constexpr (M == WarpMode::SyncHard)
                return wt.read(master * r + off);
            else
                return wt.read(triFold(master * r + off, true));
        };
        const float e = 0.5f / wt.size;
        const auto slaveSlope = [&](float master) { return (slave(master + e) - slave(master - e)) / (2.0f * e) * u.inc; };
        for (int i = 0; i < m; ++i)
        {
            ph += u.inc;
            const bool wrapped = ph >= 1.0f;
            if (wrapped)
            {
                ph -= 1.0f;
                const float d = ph / u.inc;
                blep->addStep(slave(0.0f) - slave(1.0f), d);
                blep->addKink(slaveSlope(0.0f) - slaveSlope(1.0f), d);
            }
            const float emit = std::clamp(slave(ph) + blep->tick(), -4.0f, 4.0f);
            outL[i] += emit * u.gl;
            outR[i] += emit * u.gr;
            wt.advance();
        }
        (void)prev;
    }
    else
    {
        for (int i = 0; i < m; ++i)
        {
            const float s = std::clamp(warpRead<M>(wt, ph, u.ctx, modBuf != nullptr ? modBuf[i] : 0.0f), -4.0f, 4.0f);
            outL[i] += s * u.gl;
            outR[i] += s * u.gr;
            wt.advance();
            ph += u.inc;
            if (ph >= 1.0f)
                ph -= 1.0f;
        }
    }
    phase = ph;
}

#if CRUCIBLE_SIMD_NATIVE
using simd::Float4;
using simd::Int4;

// Four unison voices in the lanes of one vector. Every lane has its own table level (pointers and sizes may differ);
// the frame mix is shared. The 6-point weights, the table dot products, the phase update, the warp arithmetic and
// the pan gains run in the lanes, only the table reads are gathered.
struct Lanes4
{
    const float* a[4];
    const float* b[4];
    Float4 size;
    Int4 maxN;
    float mix;
    float mixStep;
    bool blend;
};

struct Lagrange4
{
    Float4 c0 = Float4::splat(-1.0f / 120.0f), c1 = Float4::splat(1.0f / 24.0f), c2 = Float4::splat(-1.0f / 12.0f),
           c3 = Float4::splat(1.0f / 12.0f), c4 = Float4::splat(-1.0f / 24.0f), c5 = Float4::splat(1.0f / 120.0f);
    Float4 one = Float4::splat(1.0f), two = Float4::splat(2.0f), three = Float4::splat(3.0f), zero = Float4::splat(0.0f);
};

// Table value at phase ph (cycles, in [0, 1]) in every lane.
template <bool UseB> CRUCIBLE_ALWAYS_INLINE Float4 readLanes(const Lanes4& L, const Lagrange4& k, Float4 ph, Float4 mix)
{
    const Float4 x = ph * L.size;
    const Int4 n = Int4::min(x.truncate(), L.maxN);
    const Float4 f = x - Float4::fromInt(n);
    const Float4 d0 = f + k.two, d1 = f + k.one, d2 = f, d3 = f - k.one, d4 = f - k.two, d5 = f - k.three;
    const Float4 s4 = d4 * d5, s3 = d3 * s4, s2 = d2 * s3, s1 = d1 * s2;
    const Float4 p2 = d0 * d1, p3 = p2 * d2, p4 = p3 * d3, p5 = p4 * d4;
    const Float4 w0 = s1 * k.c0, w1 = (d0 * s2) * k.c1, w2 = (p2 * s3) * k.c2, w3 = (p3 * s4) * k.c3, w4 = (p4 * d5) * k.c4,
                 w5 = p5 * k.c5;
    Float4 sa = w0 * Float4::gather(L.a, n, -2);
    sa = Float4::mulAdd(sa, w1, Float4::gather(L.a, n, -1));
    sa = Float4::mulAdd(sa, w2, Float4::gather(L.a, n, 0));
    sa = Float4::mulAdd(sa, w3, Float4::gather(L.a, n, 1));
    sa = Float4::mulAdd(sa, w4, Float4::gather(L.a, n, 2));
    sa = Float4::mulAdd(sa, w5, Float4::gather(L.a, n, 3));
    if constexpr (!UseB)
    {
        return sa;
    }
    else
    {
        Float4 sb = w0 * Float4::gather(L.b, n, -2);
        sb = Float4::mulAdd(sb, w1, Float4::gather(L.b, n, -1));
        sb = Float4::mulAdd(sb, w2, Float4::gather(L.b, n, 0));
        sb = Float4::mulAdd(sb, w3, Float4::gather(L.b, n, 1));
        sb = Float4::mulAdd(sb, w4, Float4::gather(L.b, n, 2));
        sb = Float4::mulAdd(sb, w5, Float4::gather(L.b, n, 3));
        return Float4::mulAdd(sa, sb - sa, mix);
    }
}

CRUCIBLE_ALWAYS_INLINE Float4 wrapPhase(Float4 x)
{
    return x - Float4::floor(x);
}

CRUCIBLE_ALWAYS_INLINE Float4 foldWave4(Float4 x, const Lagrange4& k)
{
    x = x + k.one;
    x = x - Float4::splat(4.0f) * Float4::floor(x * Float4::splat(0.25f));
    x = Float4::select(Float4::gt(x, k.two), Float4::splat(4.0f) - x, x);
    return x - k.one;
}

// Scalar helpers of the hard sync kernel: the slave's table position for a master phase.
struct SyncLanes
{
    MinBlepBuffer* blep[4];
    Wt wt[4];
    float inc[4];
};

template <WarpMode M, bool UseB>
void runGroup4(const Lanes4& L, const WarpCtx& c, const float* inc, const float* gl, const float* gr, float* phase,
               float* outL, float* outR, int m, const float* modBuf, SyncLanes* sync)
{
    const Lagrange4 k;
    Float4 ph = Float4::load(phase);
    const Float4 vinc = Float4::load(inc), vgl = Float4::load(gl), vgr = Float4::load(gr);
    const Float4 hi = Float4::splat(4.0f), lo = Float4::splat(-4.0f);
    float mixS = L.mix;
    Float4 vmix = Float4::splat(L.mix);
    const Float4 vmixStep = Float4::splat(L.mixStep);
    const Float4 p0 = Float4::splat(c.p0), p1 = Float4::splat(c.p1);
    for (int i = 0; i < m; ++i)
    {
        Float4 s;
        if constexpr (M == WarpMode::SyncHard)
        {
            ph = ph + vinc;
            const Float4 wrapped = Float4::geq(ph, k.one);
            if (Float4::anyTrue(wrapped))
            {
                // The slave restarts with the master: correct the step and the slope break with a minBLEP / minBLAMP.
                float flag[4], phs[4];
                Float4::select(wrapped, k.one, k.zero).store(flag);
                (ph - Float4::select(wrapped, k.one, k.zero)).store(phs);
                for (int l = 0; l < 4; ++l)
                    if (flag[l] != 0.0f)
                    {
                        Wt w = sync->wt[l];
                        w.mix = mixS;
                        const float r = c.p0, off = c.p1;
                        const auto slave = [&](float master) { return w.read(master * r + off); };
                        const float e = 0.5f / w.size;
                        const auto slope = [&](float master)
                        { return (slave(master + e) - slave(master - e)) / (2.0f * e) * sync->inc[l]; };
                        const float d = phs[l] / sync->inc[l];
                        sync->blep[l]->addStep(slave(0.0f) - slave(1.0f), d);
                        sync->blep[l]->addKink(slope(0.0f) - slope(1.0f), d);
                    }
                ph = Float4::load(phs);
            }
            float tk[4];
            for (int l = 0; l < 4; ++l)
                tk[l] = sync->blep[l]->tick();
            s = readLanes<UseB>(L, k, wrapPhase(ph * p0 + p1), vmix) + Float4::load(tk);
        }
        else
        {
            Float4 rp = ph;
            if constexpr (M == WarpMode::Fm)
                rp = wrapPhase(ph + p0 * Float4::splat(modBuf[i]));
            s = readLanes<UseB>(L, k, rp, vmix);
            if constexpr (M == WarpMode::Ring)
                s = s * Float4::splat(1.0f - c.p0 + c.p0 * modBuf[i]);
            else if constexpr (M == WarpMode::Fold)
                s = foldWave4(p0 * s + p1, k);
        }
        s = Float4::min(Float4::max(s, lo), hi);
        outL[i] += (s * vgl).sum();
        outR[i] += (s * vgr).sum();
        if constexpr (M != WarpMode::SyncHard)
        {
            ph = ph + vinc;
            ph = ph - Float4::select(Float4::geq(ph, k.one), k.one, k.zero);
        }
        if constexpr (UseB)
        {
            vmix = vmix + vmixStep;
            mixS += L.mixStep;
        }
    }
    ph.store(phase);
}

template <WarpMode M>
void runGroup4Blend(const Lanes4& L, const WarpCtx& c, const float* inc, const float* gl, const float* gr, float* phase,
                    float* outL, float* outR, int m, const float* modBuf, SyncLanes* sync)
{
    if (L.blend)
        runGroup4<M, true>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
    else
        runGroup4<M, false>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
}

// Whether a warp has a four-lane kernel.
constexpr bool hasGroupKernel(WarpMode mode)
{
    return mode == WarpMode::Off || mode == WarpMode::Fm || mode == WarpMode::Ring || mode == WarpMode::SyncHard ||
           mode == WarpMode::Fold;
}

void dispatchGroup4(WarpMode mode, const Lanes4& L, const WarpCtx& c, const float* inc, const float* gl, const float* gr,
                    float* phase, float* outL, float* outR, int m, const float* modBuf, SyncLanes* sync)
{
    switch (mode)
    {
    case WarpMode::Fm:
        runGroup4Blend<WarpMode::Fm>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
        break;
    case WarpMode::Ring:
        runGroup4Blend<WarpMode::Ring>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
        break;
    case WarpMode::Fold:
        runGroup4Blend<WarpMode::Fold>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
        break;
    case WarpMode::SyncHard:
        runGroup4Blend<WarpMode::SyncHard>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
        break;
    default:
        runGroup4Blend<WarpMode::Off>(L, c, inc, gl, gr, phase, outL, outR, m, modBuf, sync);
        break;
    }
}
#endif

void dispatchUnison(WarpMode mode, const UnisonRun& u, float& phase, float& prev, MinBlepBuffer* blep, float* l, float* r,
                    int m, const float* modBuf)
{
    switch (mode)
    {
#define LT_CASE(name)                                                                                                       \
    case WarpMode::name:                                                                                                    \
        runUnison<WarpMode::name>(u, phase, prev, blep, l, r, m, modBuf);                                                   \
        break
        LT_CASE(Fm);
        LT_CASE(SyncHard);
        LT_CASE(SyncSoft);
        LT_CASE(BendPlus);
        LT_CASE(BendMinus);
        LT_CASE(Mirror);
        LT_CASE(Pwm);
        LT_CASE(Quantize);
        LT_CASE(Asym);
        LT_CASE(Flip);
        LT_CASE(Formant);
        LT_CASE(Smear);
        LT_CASE(Ring);
        LT_CASE(Fold);
#undef LT_CASE
    case WarpMode::Off:
    case WarpMode::Count:
        runUnison<WarpMode::Off>(u, phase, prev, blep, l, r, m, modBuf);
        break;
    }
}

float polyBlep(float t, float dt)
{
    if (t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.0f;
    }
    if (t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

// Hard sync to an external master: the carrier restarts its cycle whenever the master wraps. The jump at each
// restart is corrected with a minBLEP. events are master wrap times in carrier samples, ascending.
void runSyncExternal(const UnisonRun& u, float& phase, MinBlepBuffer& blep, float* outL, float* outR, int m,
                     const float* events, int numEvents)
{
    Wt wt = u.wt;
    float ph = phase;
    int e = 0;
    const float zero = wt.read(0.0f);
    const float zeroSlope = wt.slope(0.0f) * u.inc;
    for (int i = 0; i < m; ++i)
    {
        const float s = std::clamp(wt.read(ph) + blep.tick(), -4.0f, 4.0f);
        outL[i] += s * u.gl;
        outR[i] += s * u.gr;
        float t0 = static_cast<float>(i);
        while (e < numEvents && events[e] <= static_cast<float>(i + 1))
        {
            const float te = std::max(events[e], t0);
            float atEvent = ph + u.inc * (te - t0);
            atEvent -= std::floor(atEvent);
            const float d = static_cast<float>(i + 1) - te;
            blep.addStep(zero - wt.read(atEvent), d);
            // The slope breaks too: the cycle restarts in the middle of the waveform's own edge.
            const float slopeBefore = wt.slope(atEvent) * u.inc;
            blep.addKink(zeroSlope - slopeBefore, d);
            ph = 0.0f;
            t0 = te;
            ++e;
        }
        ph += u.inc * (static_cast<float>(i + 1) - t0);
        ph -= std::floor(ph);
        wt.advance();
    }
    phase = ph;
}
} // namespace

void computeUnisonLayout(const OscParams& p, float detuneCents, const std::array<float, kMaxUnison>& randA,
                         const std::array<float, kMaxUnison>& randB, float voiceGain,
                         std::array<UnisonVoice, kMaxUnison>& out)
{
    static constexpr std::array<float, 7> superOffsets = {-1.0f, -0.5715f, -0.1775f, 0.0f, 0.1810f, 0.5651f, 0.9768f};
    static constexpr std::array<std::array<float, 8>, 4> chords = {{
        {{0.0f, 12.0f, -12.0f, 24.0f, -24.0f, 36.0f, -36.0f, 48.0f}},
        {{0.0f, 7.0f, 12.0f, 19.0f, 24.0f, 31.0f, -5.0f, -12.0f}},
        {{0.0f, 3.0f, 7.0f, 12.0f, 15.0f, 19.0f, 24.0f, 27.0f}},
        {{0.0f, 4.0f, 7.0f, 12.0f, 16.0f, 19.0f, 24.0f, 28.0f}},
    }};

    const int n = std::clamp(p.unisonVoices, 1, kMaxUnison);
    std::array<float, kMaxUnison> weight{};
    float norm = 0.0f;
    for (int u = 0; u < n; ++u)
    {
        const float lin = n > 1 ? static_cast<float>(u) / static_cast<float>(n - 1) * 2.0f - 1.0f : 0.0f;
        float detuneRel = lin;
        float semis = 0.0f;
        switch (p.uniMode)
        {
        case 1:
        {
            const float x = n > 1 ? static_cast<float>(u) / static_cast<float>(n - 1) * 6.0f : 3.0f;
            const int i = std::min(static_cast<int>(x), 5);
            const float f = x - static_cast<float>(i);
            detuneRel = superOffsets[static_cast<size_t>(i)] * (1.0f - f) + superOffsets[static_cast<size_t>(i + 1)] * f;
            break;
        }
        case 2:
            detuneRel = lin * std::abs(lin);
            break;
        case 3:
        case 4:
        case 5:
        case 6:
            semis = chords[static_cast<size_t>(p.uniMode - 3)][static_cast<size_t>(u)];
            detuneRel = lin * 0.3f;
            break;
        case 7:
            detuneRel = randA[static_cast<size_t>(u)];
            break;
        default:
            break;
        }
        semis += detuneRel * detuneCents * 0.01f;
        semis += randA[static_cast<size_t>(u)] * p.uniScatter * 0.3f;

        const float w = 0.15f + 0.85f * (1.0f - std::abs(lin));
        weight[static_cast<size_t>(u)] = n > 1 ? w + (1.0f - w) * p.uniBlend : 1.0f;
        norm += weight[static_cast<size_t>(u)] * weight[static_cast<size_t>(u)];

        const float pan = std::clamp(lin * p.spread + randB[static_cast<size_t>(u)] * p.uniScatter * 0.5f, -1.0f, 1.0f);
        const float angle = (pan + 1.0f) * 0.78539816f;
        out[static_cast<size_t>(u)].semitones = semis;
        out[static_cast<size_t>(u)].gainL = std::cos(angle) * kSqrt2;
        out[static_cast<size_t>(u)].gainR = std::sin(angle) * kSqrt2;
    }
    const float g = voiceGain / std::sqrt(std::max(norm, 1.0e-6f));
    for (int u = 0; u < n; ++u)
    {
        out[static_cast<size_t>(u)].gainL *= g * weight[static_cast<size_t>(u)];
        out[static_cast<size_t>(u)].gainR *= g * weight[static_cast<size_t>(u)];
    }
}

// Highest partial * phase increment (cycles per oscillator sample) allowed: at the output rate partials up to
// (rate - 20 kHz) are kept, so what folds over Nyquist lands above 20 kHz. Above 48 kHz the table is also kept to
// 30 kHz (never less than 0.2 cycles per sample): partials of a low note beyond that would reach the table's read
// resolution, and their interpolation images fold back into the audible band (measured -50 dB for C2 at 96 kHz).
float Voice::bandLimit(int os) const
{
    // Oversampled (warped) oscillators generate partials of their own above the table's, so they stay within the
    // Nyquist of their own rate.
    if (os != 1)
        return 0.5f;
    return WavetableBank::outputBandLimit(sampleRate);
}

const float* Voice::spectralFrame(OscState& st, const spectral::SpectralTable& table, spectral::SpectralSynth& synth,
                                  const OscParams& p, float amount, float position, float previousPosition, int mip)
{
    if (st.specData.size() < static_cast<size_t>(OscState::kSpecCache) * kSpecStride)
        return nullptr;
    const float last = static_cast<float>(WavetableBank::numFrames - 1);
    const float fp0 = std::clamp(previousPosition, 0.0f, 1.0f) * last, fp1 = std::clamp(position, 0.0f, 1.0f) * last;
    // The position of the chunk's middle, quantised to 1/16 of a frame so that a slowly moving position reuses frames.
    const float pq = std::clamp(std::round(0.5f * (fp0 + fp1) * 16.0f) / 16.0f, 0.0f, last);
    const int frame = std::min(static_cast<int>(pq), WavetableBank::numFrames - 2);
    const float mix = std::clamp(pq - static_cast<float>(frame), 0.0f, 1.0f);
    OscState::SpecKey key;
    key.analysis = table.id();
    key.table = p.table;
    key.mip = mip;
    key.position = static_cast<int>(std::lround(pq * 16.0f));
    key.interp = p.wtInterp;
    key.warp = p.specWarp;
    key.amount = static_cast<int>(std::lround(std::clamp(amount, -1.0f, 1.0f) * 256.0f));
    ++st.specTick;
    int victim = 0;
    for (int e = 0; e < OscState::kSpecCache; ++e)
    {
        if (st.specKey[static_cast<size_t>(e)] == key)
        {
            st.specUsed[static_cast<size_t>(e)] = st.specTick;
            return st.specData.data() + static_cast<size_t>(e) * kSpecStride + 2;
        }
        if (st.specUsed[static_cast<size_t>(e)] < st.specUsed[static_cast<size_t>(victim)])
            victim = e;
    }
    float* dst = st.specData.data() + static_cast<size_t>(victim) * kSpecStride + 2;
    spectral::WarpSettings warp;
    warp.mode = static_cast<spectral::Warp>(std::clamp(p.specWarp, 0, static_cast<int>(spectral::Warp::Count) - 1));
    warp.amount = amount;
    const auto interp = p.wtInterp != 0 ? spectral::Interp::Sweep : spectral::Interp::Crossfade;
    if (!synth.render(table, frame, mix, interp, warp, WavetableBank::mipHarmonics(mip), WavetableBank::mipSize(mip), dst))
    {
        st.specKey[static_cast<size_t>(victim)] = {};
        return nullptr;
    }
    st.specKey[static_cast<size_t>(victim)] = key;
    st.specUsed[static_cast<size_t>(victim)] = st.specTick;
    return dst;
}

void Voice::configure(int oscillators, int filterSlots, const ModuleRegistry& registry)
{
    extra = std::make_unique<Extra>();
    filters.assign(static_cast<size_t>(std::clamp(filterSlots, 1, kMaxFilterSlots)), FilterUnit{});
    bankModel.fill(-1);
    for (auto& f : filters)
        f.prepare(sampleRate, static_cast<Quality>(std::clamp(quality, 0, 2)));
    model = &registry.parameters();
    const auto types = registry.oscillatorTypes();
    oscTypes = std::max(1, static_cast<int>(types.size()));
    instances.clear();
    instances.resize(static_cast<size_t>(std::clamp(oscillators, 1, kMaxOscSlots) * oscTypes));
    for (int slot = 0; slot < std::clamp(oscillators, 1, kMaxOscSlots); ++slot)
        osc[static_cast<size_t>(slot)].specData.assign(static_cast<size_t>(OscState::kSpecCache) * kSpecStride, 0.0f);
    for (int slot = 0; slot < std::clamp(oscillators, 1, kMaxOscSlots); ++slot)
        for (int t = 1; t < oscTypes; ++t)
            if (types[static_cast<size_t>(t)] != nullptr && types[static_cast<size_t>(t)]->createOscillator)
                instances[static_cast<size_t>(slot * oscTypes + t)] =
                    types[static_cast<size_t>(t)]->createOscillator(slot, *model);
}

OscillatorInstance* Voice::oscillatorInstance(int slot, int type) const
{
    if (type < 1 || type >= oscTypes || slot < 0 || static_cast<size_t>(slot * oscTypes + type) >= instances.size())
        return nullptr;
    return instances[static_cast<size_t>(slot * oscTypes + type)].get();
}

void Voice::prepare(double rate, int qualityLevel)
{
    sampleRate = rate;
    quality = qualityLevel;
    MinBlep::instance();
    FineBlep::instance();
    for (auto& inst : instances)
        if (inst)
            inst->prepare(rate, quality);
    for (auto& e : env)
        e.prepare(rate);
    for (auto& f : filters)
        f.prepare(rate, static_cast<Quality>(std::clamp(qualityLevel, 0, 2)));
    kill();
}

void Voice::noteOn(int note, float vel, const VoiceParams& params, std::uint64_t age, float startNote, float detuneSemitones)
{
    // A voice that still sounds and is struck again (mono / legato retrigger) keeps its oscillator state, so the
    // waveform stays continuous under the envelope's "From Level" restart. A fresh, finished or fading-out voice starts
    // its oscillators anew.
    const bool keepOsc = active && !fading;
    pending.on = false;
    staticDetune = detuneSemitones;
    fading = false;
    fadeDone = 0;
    fadeLength = std::max(16, static_cast<int>(0.005 * sampleRate));
    midiNote = note;
    curNote = startNote;
    velocity = std::clamp(vel, 0.0f, 1.0f);
    if (keepOsc)
    {
        velGainRamp = std::max(1, static_cast<int>(0.002 * sampleRate));
        velGainStep = (velocity - velGain) / static_cast<float>(velGainRamp);
    }
    else
    {
        velGain = velocity;
        velGainRamp = 0;
    }
    noteAge = age;
    rng.state = (static_cast<std::uint32_t>(age * 2654435761U + static_cast<std::uint64_t>(note)) ^ params.seed) | 1U;

    for (int o = 0; o < params.numOsc; ++o)
    {
        auto& st = osc[static_cast<size_t>(o)];
        const auto& op = params.osc[static_cast<size_t>(o)];
        const float base = op.phaseDegrees / 360.0f;
        for (int u = 0; u < kMaxUnison; ++u)
        {
            // Every unison voice, the first included, starts at a random phase scaled by "Random Phase" (0 = reset
            // to the Phase knob), so two voices of the same pitch do not hold a fixed phase relation.
            const float offset = rng.unipolar() * op.uniRand;
            const float ra = rng.bipolar();
            const float rb = rng.bipolar();
            if (keepOsc)
                continue;
            st.phase[static_cast<size_t>(u)] = std::fmod(base + offset, 1.0f);
            st.prev[static_cast<size_t>(u)] = 0.0f;
            st.randA[static_cast<size_t>(u)] = ra;
            st.randB[static_cast<size_t>(u)] = rb;
        }
        if (keepOsc)
            continue;
        st.modPhase = 0.0f;
        st.lastPosition = -1.0f;
        for (auto& b : st.blep)
            b.reset();
        st.up1.reset();
        st.up2.reset();
        refPhase[static_cast<size_t>(o)] = base;
        for (auto& d : st.decimator)
            d.reset();
        for (auto& d : st.decimator2)
            d.reset();
    }
    if (!keepOsc)
    {
        subPhase = 0.0f;
        drift = 0.0f;
        driftClock = kChunkSize - 1; // the first call takes a step, as it always did
    }
    noteSeed = static_cast<std::uint32_t>(age * 2246822519ULL + static_cast<std::uint64_t>(note) * 3266489917ULL) ^
               params.seed ^ 0x9e3779b9U;
    noteTime = 0.0;
    // Module oscillators run on through a retrigger as the built-in ones do (only continuous ones get here, see
    // continuesOnRetrigger()); a fresh voice starts them anew.
    if (!keepOsc)
        startedType.fill(-1);

    if (!active)
    {
        bankModel.fill(-1);
        for (auto& f : filters)
            f.reset();
        for (auto& e : env)
            e.reset();
        pink0 = pink1 = pink2 = brown = crackle = {};
    }
    ensureCard(params);
    setEnvelopes(params);
    for (auto& e : env)
        e.noteOn(velocity, note);
    for (size_t l = 0; l < lfos.size(); ++l)
        lfos[l].reset(rng.next(), params.lfo[l].settings.startPhase);
    lfoRateMod.fill(0.0f);
    noteRandom = rng.bipolar();
    // The modulator bank and the audio bus restart with the note (no draw from rng: the streams above stay as they were).
    bankRestart = true;
    for (size_t l = 0; l < extra->audioLfos.size(); ++l)
        extra->audioLfos[l].reset(params.lfo[l].settings.startPhase);
    extra->audio.lastMean.fill(0.0f);
    mixPrevCount = 0;
    active = true;
}

void Voice::ensureCard(const VoiceParams& p)
{
    cardAmounts = p.card;
    cardOn = p.card.any();
    cardTune = 0.0f;
    cardGain = 1.0f;
    if (!cardOn)
        return;
    if (!cardBuilt || card.seed != p.seed || card.index != voiceIndex)
    {
        card = VoiceCard::make(p.seed, voiceIndex);
        cardBuilt = true;
        for (size_t f = 0; f < filters.size() && f < static_cast<size_t>(kCardFilters); ++f)
            filters[f].setSeed(card.filterSeed[f]);
    }
    cardTune = card.pitchSemitones(p.card.osc);
    cardGain = card.gain(p.card.level);
}

void Voice::setEnvelopes(const VoiceParams& p)
{
    for (size_t e = 0; e < env.size(); ++e)
    {
        float timeScale = 1.0f, sustainOffset = 0.0f;
        if (cardOn && p.card.env > 0.0f)
        {
            timeScale = card.envTimeScale(static_cast<int>(e), p.card.env);
            sustainOffset = card.envSustainOffset(static_cast<int>(e), p.card.env);
        }
        env[e].setParams(p.env[e], p.envShape[e], timeScale, sustainOffset);
    }
}

void Voice::fadeOut()
{
    if (!active || fading)
        return;
    fading = true;
    fadeDone = 0;
}

bool Voice::continuesOnRetrigger() const
{
    for (int slot = 0; slot < kMaxOscSlots; ++slot)
    {
        const int type = startedType[static_cast<size_t>(slot)];
        if (type < 1)
            continue; // not started, or the built-in wavetable (its state is kept)
        const OscillatorInstance* inst = oscillatorInstance(slot, type);
        if (inst != nullptr && !inst->continuesOnRetrigger())
            return false;
    }
    return true;
}

void Voice::noteOnAfterFade(int note, float vel, const VoiceParams& params, std::uint64_t age, float startNote,
                            float detuneSemitones)
{
    if (!active)
    {
        noteOn(note, vel, params, age, startNote, detuneSemitones);
        return;
    }
    pending = {true, false, note, vel, &params, age, startNote, detuneSemitones};
    noteAge = age; // the newest note is the last to be stolen
    fadeOut();
}

void Voice::glideTo(int note)
{
    midiNote = note;
}

void Voice::noteOff()
{
    if (pending.on)
    {
        pending.released = true; // applied once the note has started
        return;
    }
    for (auto& e : env)
        e.noteOff();
    for (auto& inst : instances)
        if (inst)
            inst->noteOff();
}

void Voice::kill()
{
    pending.on = false;
    active = false;
    fading = false;
    for (auto& e : env)
        e.reset();
    bankModel.fill(-1);
    for (auto& f : filters)
        f.reset();
}

float Voice::noiseSample(int ch, int type)
{
    const auto c = static_cast<size_t>(ch);
    const float white = rng.bipolar();
    switch (type)
    {
    case 1:
    {
        pink0[c] = 0.99765f * pink0[c] + white * 0.0990460f;
        pink1[c] = 0.96300f * pink1[c] + white * 0.2965164f;
        pink2[c] = 0.57000f * pink2[c] + white * 1.0526913f;
        return (pink0[c] + pink1[c] + pink2[c] + white * 0.1848f) * 0.25f;
    }
    case 2:
    {
        brown[c] = std::clamp(brown[c] * 0.998f + white * 0.04f, -1.0f, 1.0f);
        return brown[c] * 3.0f;
    }
    case 3:
    {
        if (rng.unipolar() < 0.0015f)
            crackle[c] = rng.bipolar() * 1.5f;
        crackle[c] *= 0.7f;
        return crackle[c];
    }
    default:
        return white;
    }
}

void Voice::renderOscillator(int index, const VoiceParams& vp, const std::array<float, 64>& mod,
                             const std::array<SlotMod, kMaxOscSlots>& slotMod, float pitchSemis, int n, float* outL,
                             float* outR, float* monoOut, float levelOverride)
{
    const OscParams& p = vp.osc[static_cast<size_t>(index)];
    const SlotMod& sm = slotMod[static_cast<size_t>(index)];
    const auto at = [&mod](ModDest d) { return mod[static_cast<size_t>(d)]; };

    float level = std::clamp(p.level + sm.level, 0.0f, 1.0f);
    if (level <= 0.0f && levelOverride > 0.0f)
        level = levelOverride;
    if (level <= 0.0f)
        return;

    const float position = std::clamp(p.position + sm.position, 0.0f, 1.0f);
    auto& morph = osc[static_cast<size_t>(index)];
    const float previousPosition = morph.lastPosition >= 0.0f ? morph.lastPosition : position;
    morph.lastPosition = position;
    auto mode = static_cast<WarpMode>(std::clamp(p.warp, 0, static_cast<int>(WarpMode::Count) - 1));
    const float warpA = std::clamp(p.warpA + sm.warpA, 0.0f, 1.0f);
    const float warpB = std::clamp(p.warpB + sm.warpB, 0.0f, 1.0f);

    // Cross modulation from an earlier slot: FM and ring replace the warp's own modulator by that slot's output
    // (when the warp is off or the same kind), hard sync restarts the cycle at the other slot's wraps (warp off).
    const int xsrc = p.xsrc;
    const bool xvalid = p.xmode > 0 && xsrc >= 0 && xsrc < index;
    const float xamt = std::clamp(p.xamt + sm.xamt, 0.0f, 1.0f);
    bool externalMod = false;
    bool externalSync = false;
    if (xvalid && xamt > 0.0f)
    {
        if (p.xmode == 1 && (mode == WarpMode::Off || mode == WarpMode::Fm))
            externalMod = true, mode = WarpMode::Fm;
        else if (p.xmode == 2 && (mode == WarpMode::Off || mode == WarpMode::Ring))
            externalMod = true, mode = WarpMode::Ring;
        else if (p.xmode == 3 && mode == WarpMode::Off)
            externalSync = true;
    }
    WarpCtx ctx = externalMod ? makeWarpCtx(mode, xamt, 0.0f) : makeWarpCtx(mode, warpA, warpB);
    if (externalSync)
        ctx.oversample = 2;
    // Quality policy: Draft caps the oversampling at 2x, High adds a stage to 2x modes.
    if (quality == 0)
        ctx.oversample = std::min(ctx.oversample, 2);
    else if (quality == 2 && ctx.oversample == 2)
        ctx.oversample = 4;
    const int os = ctx.oversample;
    const int m = n * os;

    const float centre = curNote + vp.transpose + p.semitones + p.fineCents * 0.01f + pitchSemis +
                         24.0f * (sm.pitch + at(ModDest::Pitch)) + (drift * vp.driftAmount + staticDetune) + cardTune;
    const float detune = std::clamp(p.detuneCents + sm.uniDetune + 50.0f * at(ModDest::UnisonDetune), 0.0f, 200.0f);

    auto& st = osc[static_cast<size_t>(index)];
    // Spectral wavetable path (opt-in, see Spectral.h): the analysis of the slot's table, or null for the legacy path. The
    // analysis is made on a control thread; a table that has none yet plays through the legacy crossfade.
    const spectral::SpectralTable* spec = nullptr;
    const float specAmount = std::clamp(p.specAmount + sm.specAmount, -1.0f, 1.0f);
    if (vp.spectralSynth != nullptr && !externalSync && (p.wtInterp != 0 || (p.specWarp != 0 && specAmount != 0.0f)))
    {
        const int tableIndex = std::clamp(p.table, 0, kTotalTables - 1);
        if (tableIndex < WavetableBank::numTables)
            spec = spectral::FactoryTables::instance().get(tableIndex);
        else if (vp.userTables != nullptr)
        {
            const UserWavetable* u = (*vp.userTables)[static_cast<size_t>(tableIndex - WavetableBank::numTables)];
            spec = u != nullptr ? u->spectral() : nullptr;
        }
    }
    const auto wtFor = [&](float increment, float limit, float plainIncrement)
    {
        Wt w = makeWt(p.table, position, previousPosition, m, increment, limit, vp.userTables, plainIncrement,
                      sampleRate);
        if (spec != nullptr)
            if (const float* f = spectralFrame(st, *spec, *vp.spectralSynth, p, specAmount, position, previousPosition,
                                               w.mip))
            {
                w.a = w.b = f;
                w.mix = w.mixStep = 0.0f;
                w.blend = false;
            }
        return w;
    };
    std::array<UnisonVoice, kMaxUnison> layout{};
    if (sm.uniSpread != 0.0f || sm.uniBlend != 0.0f || sm.uniScatter != 0.0f)
    {
        OscParams stack = p;
        stack.spread = std::clamp(p.spread + sm.uniSpread, 0.0f, 1.0f);
        stack.uniBlend = std::clamp(p.uniBlend + sm.uniBlend, 0.0f, 1.0f);
        stack.uniScatter = std::clamp(p.uniScatter + sm.uniScatter, 0.0f, 1.0f);
        computeUnisonLayout(stack, detune, st.randA, st.randB, level, layout);
    }
    else
        computeUnisonLayout(p, detune, st.randA, st.randB, level, layout);
    const int voices = std::clamp(p.unisonVoices, 1, kMaxUnison);

    float bufL[4 * kChunkSize] = {};
    float bufR[4 * kChunkSize] = {};

    // Modulator for FM and ring modulation: a single wavetable read at ratio times the centre pitch.
    float modBuf[4 * kChunkSize];
    const float* modPtr = nullptr;
    if (externalMod)
    {
        // The other slot's output, interpolated from the base rate to the oversampled rate (os is 4 here).
        float mid[2 * kChunkSize];
        for (int i = 0; i < n; ++i)
            st.up1.process(modSignal[static_cast<size_t>(xsrc)][static_cast<size_t>(i)], mid[2 * i], mid[2 * i + 1]);
        if (os >= 4)
            for (int j = 0; j < 2 * n; ++j)
                st.up2.process(mid[j], modBuf[2 * j], modBuf[2 * j + 1]);
        else
            for (int j = 0; j < 2 * n; ++j)
                modBuf[j] = mid[j];
        modPtr = modBuf;
    }
    else if (mode == WarpMode::Fm || mode == WarpMode::Ring)
    {
        const int src = std::clamp(p.warpSource, 0, vp.numOsc - 1);
        const auto& so = vp.osc[static_cast<size_t>(src)];
        const float srcPos = std::clamp(so.position + slotMod[static_cast<size_t>(src)].position, 0.0f, 1.0f);
        const float inc =
            std::min(noteToHz(centre) * ctx.ratio / static_cast<float>(sampleRate) / static_cast<float>(os), 0.49f);
        const Wt wt = makeWt(so.table, srcPos, srcPos, m, inc, bandLimit(os), vp.userTables,
                             inc * static_cast<float>(os), sampleRate);
        float mp = st.modPhase;
        for (int i = 0; i < m; ++i)
        {
            modBuf[i] = wt.read(mp);
            mp += inc;
            if (mp >= 1.0f)
                mp -= 1.0f;
        }
        st.modPhase = mp;
        modPtr = modBuf;
    }

    int firstScalar = 0;
#if CRUCIBLE_SIMD_NATIVE
    // Groups of up to four unison voices share the lanes of one vector; the remaining single voice runs scalar.
    if (simd::kernelsEnabled() && !externalSync && hasGroupKernel(mode) && voices >= 2)
    {
        std::array<UnisonRun, kMaxUnison> runs;
        for (int u = 0; u < voices; ++u)
        {
            auto& run = runs[static_cast<size_t>(u)];
            const float hz = noteToHz(centre + layout[static_cast<size_t>(u)].semitones);
            run.inc = std::min(hz / static_cast<float>(sampleRate) / static_cast<float>(os), 0.49f);
            run.gl = layout[static_cast<size_t>(u)].gainL;
            run.gr = layout[static_cast<size_t>(u)].gainR;
        }
        MinBlepBuffer padBlep;
        while (voices - firstScalar >= 2)
        {
            const int u0 = firstScalar;
            const int count = std::min(4, voices - u0);
            float inc4[4], gl4[4], gr4[4], ph4[4], size4[4];
            std::int32_t maxN4[4];
            Lanes4 lanes{};
            SyncLanes sync{};
            // Resolve the cycles of this group only now: the spectral cache keeps at least the four of a group.
            for (int l = 0; l < count; ++l)
            {
                auto& run = runs[static_cast<size_t>(u0 + l)];
                run.wt = wtFor(run.inc * ctx.rateScale, bandLimit(os), run.inc * static_cast<float>(os));
            }
            for (int l = 0; l < 4; ++l)
            {
                const bool live = l < count;
                const auto& r = runs[static_cast<size_t>(live ? u0 + l : u0)];
                lanes.a[l] = r.wt.a;
                lanes.b[l] = r.wt.b;
                size4[l] = r.wt.size;
                maxN4[l] = r.wt.isize - 1;
                inc4[l] = live ? r.inc : 0.0f;
                gl4[l] = live ? r.gl : 0.0f;
                gr4[l] = live ? r.gr : 0.0f;
                ph4[l] = live ? st.phase[static_cast<size_t>(u0 + l)] : 0.0f;
                sync.wt[l] = r.wt;
                sync.inc[l] = r.inc;
                sync.blep[l] = live ? &st.blep[static_cast<size_t>(u0 + l)] : &padBlep;
            }
            lanes.size = Float4::load(size4);
            lanes.maxN = Int4::load(maxN4);
            const auto& r0 = runs[static_cast<size_t>(u0)];
            lanes.mix = r0.wt.mix;
            lanes.mixStep = r0.wt.mixStep;
            lanes.blend = r0.wt.blend;
            dispatchGroup4(mode, lanes, ctx, inc4, gl4, gr4, ph4, bufL, bufR, m, modPtr, &sync);
            for (int l = 0; l < count; ++l)
                st.phase[static_cast<size_t>(u0 + l)] = ph4[l];
            firstScalar += count;
        }
    }
#endif

    for (int u = firstScalar; u < voices; ++u)
    {
        UnisonRun run;
        const float hz = noteToHz(centre + layout[static_cast<size_t>(u)].semitones);
        run.inc = std::min(hz / static_cast<float>(sampleRate) / static_cast<float>(os), 0.49f);
        run.wt = wtFor(run.inc * ctx.rateScale, externalSync ? 0.25f : bandLimit(os),
                       run.inc * static_cast<float>(os));
        run.gl = layout[static_cast<size_t>(u)].gainL;
        run.gr = layout[static_cast<size_t>(u)].gainR;
        run.ctx = ctx;
        if (externalSync)
        {
            std::array<float, 128> ev{};
            const auto& w = wraps[static_cast<size_t>(xsrc)];
            for (int k = 0; k < w.count; ++k)
                ev[static_cast<size_t>(k)] = w.time[static_cast<size_t>(k)] * static_cast<float>(os);
            runSyncExternal(run, st.phase[static_cast<size_t>(u)], st.blep[static_cast<size_t>(u)], bufL, bufR, m, ev.data(),
                            w.count);
        }
        else
        {
            dispatchUnison(mode, run, st.phase[static_cast<size_t>(u)], st.prev[static_cast<size_t>(u)],
                           &st.blep[static_cast<size_t>(u)], bufL, bufR, m, modPtr);
        }
    }

    const float pan = std::clamp(p.pan + sm.pan, -1.0f, 1.0f);
    const float angle = (pan + 1.0f) * 0.78539816f;
    const float panL = std::cos(angle) * kSqrt2;
    const float panR = std::sin(angle) * kSqrt2;

    float preL[kChunkSize], preR[kChunkSize];
    if (os == 1)
    {
        for (int i = 0; i < n; ++i)
        {
            preL[i] = bufL[i];
            preR[i] = bufR[i];
        }
    }
    else if (os == 2)
    {
        st.decimator[0].processBlock(bufL, preL, n);
        st.decimator[1].processBlock(bufR, preR, n);
    }
    else
    {
        float midL[2 * kChunkSize], midR[2 * kChunkSize];
        st.decimator[0].processBlock(bufL, midL, 2 * n);
        st.decimator[1].processBlock(bufR, midR, 2 * n);
        st.decimator2[0].processBlock(midL, preL, n);
        st.decimator2[1].processBlock(midR, preR, n);
    }
    for (int i = 0; i < n; ++i)
    {
        outL[i] += preL[i] * panL;
        outR[i] += preR[i] * panR;
    }
    if (monoOut != nullptr)
        for (int i = 0; i < n; ++i)
            monoOut[i] = 0.5f * (preL[i] + preR[i]) / level;
}

void Voice::renderModule(int index, const VoiceParams& vp, const std::array<SlotMod, kMaxOscSlots>& slotMod,
                         const float* targetMod, float pitchSemis, int n, float* outL, float* outR, float* monoOut,
                         float levelOverride, const OscAudio* audioMod)
{
    const OscParams& p = vp.osc[static_cast<size_t>(index)];
    const SlotMod& sm = slotMod[static_cast<size_t>(index)];
    const int type = p.type;
    auto* inst = oscillatorInstance(index, type);
    if (inst == nullptr)
        return;
    float level = std::clamp(p.level + sm.level, 0.0f, 1.0f);
    int& started = startedType[static_cast<size_t>(index)];
    if (started != type)
    {
        OscNoteInfo info;
        info.note = static_cast<float>(midiNote);
        info.velocity = velocity;
        info.seed = noteSeed + static_cast<std::uint32_t>(index) * 0x85ebca6bU;
        info.restart = started >= 0;
        if (cardOn && vp.card.osc > 0.0f && index < kCardOscSlots)
        {
            info.card = card.osc[static_cast<size_t>(index)].data();
            info.cardAmount = vp.card.osc;
        }
        inst->noteOn(info);
        started = type;
    }
    if (level <= 0.0f && levelOverride > 0.0f)
        level = levelOverride;
    if (level <= 0.0f)
        return;

    OscRender r;
    r.values = vp.values;
    r.targetMod = targetMod;
    r.pitch = curNote + vp.transpose + p.semitones + p.fineCents * 0.01f + pitchSemis + 24.0f * sm.pitch +
              (drift * vp.driftAmount + staticDetune) + (inst->usesVoiceCard() ? 0.0f : cardTune);
    r.sampleRate = sampleRate;
    r.numSamples = n;
    float bufL[kChunkSize] = {};
    float bufR[kChunkSize] = {};
    r.left = bufL;
    r.right = bufR;
    r.sample = vp.sample[static_cast<size_t>(index)];
    r.slices = vp.slices[static_cast<size_t>(index)];
    r.zones = vp.zones[static_cast<size_t>(index)];
    r.cloud = vp.cloud[static_cast<size_t>(index)];
    r.noteTime = noteTime;
    if (audioMod != nullptr && inst->supportsAudioRate())
    {
        r.pitchMod = audioMod->pitch;
        r.fmAmountMod = audioMod->fm;
        r.widthMod = audioMod->pw;
    }
    inst->render(r);

    // Ring modulation by an earlier slot (FM and sync need the wavetable oscillator).
    const float xamt = std::clamp(p.xamt + sm.xamt, 0.0f, 1.0f);
    if (p.xmode == 2 && p.xsrc >= 0 && p.xsrc < index && xamt > 0.0f)
        for (int i = 0; i < n; ++i)
        {
            const float g = 1.0f - xamt + xamt * modSignal[static_cast<size_t>(p.xsrc)][static_cast<size_t>(i)];
            bufL[i] *= g;
            bufR[i] *= g;
        }
    if (monoOut != nullptr)
        for (int i = 0; i < n; ++i)
            monoOut[i] = 0.5f * (bufL[i] + bufR[i]);

    const float pan = std::clamp(p.pan + sm.pan, -1.0f, 1.0f);
    const float angle = (pan + 1.0f) * 0.78539816f;
    const float panL = std::cos(angle) * kSqrt2 * level;
    const float panR = std::sin(angle) * kSqrt2 * level;
    for (int i = 0; i < n; ++i)
    {
        outL[i] += bufL[i] * panL;
        outR[i] += bufR[i] * panR;
    }
}

void Voice::renderChunk(const VoiceParams& p, const GlobalMod& g, float pitchBend, int numSamples, float* left, float* right)
{
    if (!beginChunk(p, g, pitchBend, numSamples, nullptr, 0))
        return;
    for (int f = 0; f < chunk.numFilters; ++f)
        filterIn(f);
    endChunk(left, right);
}

bool Voice::beginChunk(const VoiceParams& p, const GlobalMod& g, float pitchBend, int numSamples, FilterBank* bank,
                       int bankVoice)
{
    if (!active)
        return false;

    ensureCard(p);
    setEnvelopes(p);
    auto& audio = extra->audio;
    auto& scratch = extra->scratch;

    if (p.glide > 0.0f)
    {
        const float a = 1.0f - std::exp(-static_cast<float>(numSamples) * 3.0f / (static_cast<float>(sampleRate) * p.glide));
        curNote += (static_cast<float>(midiNote) - curNote) * a;
    }
    else
    {
        curNote = static_cast<float>(midiNote);
    }
    // The analog drift is a random walk with one step per 32 samples of the voice's time, however the calls are cut.
    driftClock += numSamples;
    for (; driftClock >= kChunkSize; driftClock -= kChunkSize)
        drift += rng.bipolar() * 0.02f - drift * 0.01f;

    // The modulator bank, the route set and the audio bus are idle (null / zero) for a patch that uses none of them.
    const RouteSet* const rs = p.routeSet;
    const ModulatorSet* const mods = p.modulators != nullptr && p.modulators->needed != 0U ? p.modulators : nullptr;
    std::uint32_t audioMask = rs != nullptr ? rs->audioMask : 0U;
    for (int f = 0; f < p.numFilter; ++f)
        if (p.filter[static_cast<size_t>(f)].fmSource > 0 && p.filter[static_cast<size_t>(f)].fmAmount != 0.0f)
            audioMask |= 1U << (p.filter[static_cast<size_t>(f)].fmSource - 1);
    const bool arOn = audioMask != 0U;
    if (arOn)
    {
        audio.slotOf.fill(-1);
        audio.filled.fill(false);
        audio.used = 0;
        for (int a = 0; a < kNumAudioSources; ++a)
            if ((audioMask >> a & 1U) != 0U && audio.used < kAudioBusSlots)
                audio.slotOf[static_cast<size_t>(a)] = static_cast<signed char>(audio.used++);
    }

    // Which sources do the routed slots need? Unused LFOs are not advanced.
    std::array<bool, kNumModSources> used{};
    for (const auto& slot : p.slots)
        if (slot.source > 0 && slot.source < kNumModSources && slot.destination > 0 && slot.amount != 0.0f)
            used[static_cast<size_t>(slot.source)] = true;
    for (int r = 0; r < p.numRoutes; ++r)
        if (p.routes[r].source > 0 && p.routes[r].source < kNumModSources)
            used[static_cast<size_t>(p.routes[r].source)] = true;
    for (int r = 0; r < p.numTargetRoutes; ++r)
        if (p.targetRoutes[r].source > 0 && p.targetRoutes[r].source < kNumModSources)
            used[static_cast<size_t>(p.targetRoutes[r].source)] = true;
    if (rs != nullptr)
        for (const auto& m : rs->amountMods)
            if (m.slot.source > 0 && m.slot.source < kNumModSources)
                used[static_cast<size_t>(m.slot.source)] = true;
    if (mods != nullptr)
        for (int n = 0; n < kNumBankModulators; ++n)
            if (mods->neededVoice(n))
            {
                const auto& c = mods->mod[static_cast<size_t>(n)];
                for (const int sel : {c.inA, c.inB})
                    if (sel > 0 && sel < kNumModSources)
                        used[static_cast<size_t>(sel)] = true;
            }

    // The ModSources start at zero; the modulator and audio-rate entries only matter (and are only zeroed) when the bank or
    // the bus is in use.
    std::array<float, kNumSourcesExt> sources;
    std::fill(sources.begin(), sources.begin() + kNumModSources, 0.0f);
    if (mods != nullptr || arOn || rs != nullptr)
        std::fill(sources.begin() + kNumModSources, sources.end(), 0.0f);
    // Envelopes used on the audio bus run one sample at a time; the others advance by the chunk, as always.
    const auto advanceEnvelope = [&](size_t e)
    {
        const int a = kAudioEnvFirst + static_cast<int>(e) - 1;
        if (arOn && (audioMask >> a & 1U) != 0U && audio.slotOf[static_cast<size_t>(a)] >= 0)
        {
            auto& buf = audio.buf[static_cast<size_t>(audio.slotOf[static_cast<size_t>(a)])];
            for (int i = 0; i < numSamples; ++i)
                buf[static_cast<size_t>(i)] = env[e].tick();
            audio.filled[static_cast<size_t>(a)] = true;
            return buf[static_cast<size_t>(numSamples - 1)];
        }
        return env[e].advance(numSamples);
    };
    const bool audioEnvelopes = arOn && (audioMask >> kAudioEnvFirst) != 0U;
    const float modEnvValue = audioEnvelopes ? advanceEnvelope(1) : env[1].advance(numSamples);
    sources[static_cast<size_t>(ModSource::ModEnv)] = modEnvValue;
    sources[static_cast<size_t>(ModSource::AmpEnv)] = env[0].value();
    sources[static_cast<size_t>(ModSource::Env3)] = audioEnvelopes ? advanceEnvelope(2) : env[2].advance(numSamples);
    sources[static_cast<size_t>(ModSource::Env4)] = audioEnvelopes ? advanceEnvelope(3) : env[3].advance(numSamples);
    for (size_t l = 0; l < kNumLfos; ++l)
    {
        const auto id = static_cast<size_t>(kLfoSources[l]);
        if (!used[id])
            continue;
        const auto& lp = p.lfo[l];
        float value;
        if (lp.settings.trigger == LfoTrigger::Free)
        {
            value = g.freeLfo[l];
        }
        else
        {
            LfoSettings st = lp.settings;
            st.rateHz *= std::exp2(2.0f * lfoRateMod[l]);
            value = lfos[l].advance(st, numSamples, sampleRate, g.customShape[l]) * lp.amount;
        }
        sources[id] = value;
    }
    sources[static_cast<size_t>(ModSource::Velocity)] = velocity;
    sources[static_cast<size_t>(ModSource::Key)] = std::clamp((static_cast<float>(midiNote) - 60.0f) / 60.0f, -1.0f, 1.0f);
    sources[static_cast<size_t>(ModSource::ModWheel)] = g.modWheel;
    sources[static_cast<size_t>(ModSource::Aftertouch)] = g.aftertouch;
    sources[static_cast<size_t>(ModSource::Random)] = noteRandom;
    for (size_t m = 0; m < kNumMacros; ++m)
        sources[static_cast<size_t>(macroSource(static_cast<int>(m)))] = g.macro[m];
    sources[static_cast<size_t>(ModSource::StripL)] = g.strip[0];
    sources[static_cast<size_t>(ModSource::StripR)] = g.strip[1];
    sources[static_cast<size_t>(ModSource::SceneMorph)] = g.morph;
    sources[static_cast<size_t>(ModSource::PitchBend)] = g.pitchBend;

    if (arOn)
    {
        // Audio-rate LFOs and the chunk means of the audio sources (an oscillator's output is the previous chunk's mean).
        for (int l = 0; l < kNumLfos; ++l)
        {
            const int a = kAudioLfoFirst + l;
            const signed char slot = audio.slotOf[static_cast<size_t>(a)];
            if (slot < 0)
                continue;
            auto& buf = audio.buf[static_cast<size_t>(slot)];
            const auto& lp = p.lfo[static_cast<size_t>(l)];
            if (lp.audioMode == 0)
                std::fill(buf.begin(), buf.begin() + numSamples, 0.0f);
            else
            {
                const double hz = lp.audioMode == 1
                                      ? static_cast<double>(lp.audioHz)
                                      : static_cast<double>(noteToHz(curNote + p.transpose + pitchBend)) * lp.audioRatio;
                extra->audioLfos[static_cast<size_t>(l)].render(lp.settings.shape, hz / sampleRate, lp.amount, numSamples,
                                                                buf.data());
            }
            audio.filled[static_cast<size_t>(a)] = true;
        }
        for (int a = 0; a < kNumAudioSources; ++a)
        {
            const signed char slot = audio.slotOf[static_cast<size_t>(a)];
            if (slot < 0)
                continue;
            float mean = audio.lastMean[static_cast<size_t>(a)];
            if (audio.filled[static_cast<size_t>(a)])
            {
                mean = 0.0f;
                for (int i = 0; i < numSamples; ++i)
                    mean += audio.buf[static_cast<size_t>(slot)][static_cast<size_t>(i)];
                mean /= static_cast<float>(numSamples);
            }
            sources[static_cast<size_t>(kExtAudioFirst + a)] = mean;
        }
    }
    if (mods != nullptr)
    {
        for (int n = 0; n < kNumBankModulators; ++n)
            if (mods->mod[static_cast<size_t>(n)].scope != ModulatorScope::Voice)
                sources[static_cast<size_t>(kExtModulatorFirst + n)] = g.bank[static_cast<size_t>(n)];
        if (bankRestart)
            restartVoiceModulators(*mods, extra->bankState.data());
        ModulatorContext ctx;
        ctx.numSamples = numSamples;
        ctx.sampleRate = sampleRate;
        ctx.voice = true;
        ctx.gate = active && !fading && !env[0].isReleasing();
        ctx.velocity = velocity;
        ctx.note = curNote;
        ctx.noteOn = bankRestart;
        ctx.audio = extra->mixPrev.data();
        ctx.audioCount = mixPrevCount;
        evaluateModulators(*mods, extra->bankState.data(), ctx, sources.data());
        for (size_t n = 0; n < static_cast<size_t>(kNumBankModulators); ++n)
            readoutBank[n] = sources[static_cast<size_t>(kExtModulatorFirst) + n];
        readoutBankLive = true;
    }
    else if (readoutBankLive)
    {
        readoutBank.fill(0.0f);
        readoutBankLive = false;
    }
    bankRestart = false;

    // Route amounts that other routes move (modulation of modulation), in dependency order.
    auto& amountDelta = scratch.amountDelta;
    const bool haveDelta = rs != nullptr && !rs->amountMods.empty();
    const auto extBipolar = [&](int src) { return extSourceBipolar(p.modulators, src); };
    const auto routeValue = [&](const ModSlotParams& slot)
    { return evaluateSlot(slot, sources[static_cast<size_t>(slot.source)], extBipolar(slot.source)); };
    const auto withDelta = [&](ModSlotParams slot, int routeIndex)
    {
        if (haveDelta)
            slot.amount = std::clamp(slot.amount + amountDelta[static_cast<size_t>(routeIndex)], -1.0f, 1.0f);
        return slot;
    };
    if (haveDelta)
    {
        amountDelta.fill(0.0f);
        for (const auto& m : rs->amountMods)
            amountDelta[static_cast<size_t>(m.target)] += routeValue(withDelta(m.slot, m.route));
    }

    std::array<float, 64> mod{};
    accumulateModulation(p.slots, sources.data(), mod.data());
    for (int r = 0; r < p.numRoutes; ++r)
    {
        const auto& route = p.routes[r];
        if (route.source > 0 && route.source < kNumSourcesExt && route.destination > 0 && route.destination < kNumModDests)
            mod[static_cast<size_t>(route.destination)] +=
                routeValue(haveDelta ? withDelta(route, rs->voiceSlotRoute[static_cast<size_t>(r)]) : route);
    }
    if (p.globalExtra != nullptr)
        for (size_t d = 0; d < static_cast<size_t>(kNumModDests); ++d)
            mod[d] += p.globalExtra[d];
    for (size_t l = 0; l < kNumLfos; ++l)
        lfoRateMod[l] = mod[static_cast<size_t>(ModDest::Lfo1Rate) + l];

    // Per-voice modulation of module parameters: matrix slots aimed at generic targets and assignable routes.
    const auto& voiceTargets = model->voiceTargets();
    const int numVoiceTargets = static_cast<int>(voiceTargets.size());
    std::array<float, ParameterMap::kMaxVoiceTargets> targetMod;
    bool anyTargetMod = false;
    if (numVoiceTargets > 0)
    {
        std::fill(targetMod.begin(), targetMod.begin() + numVoiceTargets, 0.0f);
        for (const auto& slot : p.slots)
            if (slot.source > 0 && slot.source < kNumModSources && slot.destination >= kNumModDests && slot.amount != 0.0f)
            {
                const int t = model->voiceTargetOfModTarget(slot.destination - kNumModDests);
                if (t >= 0)
                {
                    targetMod[static_cast<size_t>(t)] += evaluateSlot(slot, sources[static_cast<size_t>(slot.source)]);
                    anyTargetMod = true;
                }
            }
        for (int r = 0; r < p.numTargetRoutes; ++r)
        {
            const auto& route = p.targetRoutes[r];
            targetMod[static_cast<size_t>(route.destination)] +=
                routeValue(haveDelta ? withDelta(route, rs->voiceTargetSlotRoute[static_cast<size_t>(r)]) : route);
            anyTargetMod = true;
        }
        if (p.globalExtraTargets != nullptr)
        {
            for (int t = 0; t < numVoiceTargets; ++t)
                targetMod[static_cast<size_t>(t)] += p.globalExtraTargets[t];
            anyTargetMod = true;
        }
    }
    readoutDest = mod;
    if (numVoiceTargets > 0)
    {
        if (anyTargetMod)
            std::copy(targetMod.begin(), targetMod.begin() + numVoiceTargets, readoutTarget.begin());
        else
            std::fill(readoutTarget.begin(), readoutTarget.begin() + numVoiceTargets, 0.0f);
    }
    // Offset of a per-voice target in units of its range (0 without modulation), and the raw modulation amount.
    const auto targetRaw = [&](int t) { return t >= 0 && anyTargetMod ? targetMod[static_cast<size_t>(t)] : 0.0f; };
    const auto targetDelta = [&](int t)
    {
        return t >= 0 && anyTargetMod ? targetMod[static_cast<size_t>(t)] * voiceTargets[static_cast<size_t>(t)].range
                                      : 0.0f;
    };

    // Per-slot modulation offsets. The first three oscillators and two filters have fixed matrix destinations.
    std::array<SlotMod, kMaxOscSlots> slotMod{};
    {
        static constexpr std::array<std::array<ModDest, 6>, kNumOscs> dests = {{
            {ModDest::Osc1Position, ModDest::Osc1Pitch, ModDest::Osc1Level, ModDest::Osc1WarpA, ModDest::Osc1WarpB,
             ModDest::Osc1Pan},
            {ModDest::Osc2Position, ModDest::Osc2Pitch, ModDest::Osc2Level, ModDest::Osc2WarpA, ModDest::Osc2WarpB,
             ModDest::Osc2Pan},
            {ModDest::Osc3Position, ModDest::Osc3Pitch, ModDest::Osc3Level, ModDest::Osc3WarpA, ModDest::Osc3WarpB,
             ModDest::Osc3Pan},
        }};
        for (int o = 0; o < std::min(p.numOsc, kNumOscs); ++o)
        {
            const auto& d = dests[static_cast<size_t>(o)];
            auto& m = slotMod[static_cast<size_t>(o)];
            const auto at = [&mod](ModDest x) { return mod[static_cast<size_t>(x)]; };
            m = {at(d[0]), at(d[1]), at(d[2]), at(d[3]), at(d[4]), at(d[5]), 0.0f};
        }
        for (int o = kNumOscs; o < p.numOsc; ++o)
        {
            const auto& t = p.osc[static_cast<size_t>(o)].targets;
            slotMod[static_cast<size_t>(o)] = {targetDelta(t[0]),
                                               targetDelta(t[1]) / 24.0f,
                                               targetDelta(t[2]),
                                               targetDelta(t[3]),
                                               targetDelta(t[4]),
                                               targetDelta(t[5]),
                                               0.0f};
        }
        for (int o = 0; o < p.numOsc; ++o)
        {
            slotMod[static_cast<size_t>(o)].xamt = targetDelta(p.osc[static_cast<size_t>(o)].xamtTarget);
            const auto& ut = p.osc[static_cast<size_t>(o)].uniTargets;
            auto& m = slotMod[static_cast<size_t>(o)];
            m.uniDetune = targetDelta(ut[0]);
            m.uniSpread = targetDelta(ut[1]);
            m.uniBlend = targetDelta(ut[2]);
            m.uniScatter = targetDelta(ut[3]);
            m.specAmount = targetDelta(p.osc[static_cast<size_t>(o)].specAmountTarget);
        }
    }

    // Oscillators that cannot take per-sample input are given the chunk mean of their audio-rate routes instead.
    std::array<bool, kMaxOscSlots> takesPitch, takesFm, takesCross, takesWidth, takesPosition; // set below when arOn
    if (arOn && rs != nullptr)
    {
        for (int o = 0; o < p.numOsc; ++o)
        {
            const int type = p.osc[static_cast<size_t>(o)].type;
            const auto so = static_cast<size_t>(o);
            takesPitch[so] = takesFm[so] = takesCross[so] = takesWidth[so] = takesPosition[so] = false;
            if (type == 0)
            {
                takesPitch[so] = takesCross[so] = takesPosition[so] = true;
                // The wavetable oscillator has no pulse width and no FM amount of its own: such a route has nothing to move.
                takesWidth[so] = takesFm[so] = true;
            }
            else
            {
                const auto* inst = oscillatorInstance(o, type);
                takesPitch[so] = takesFm[so] = takesWidth[so] = inst != nullptr && inst->supportsAudioRate();
            }
        }
        for (const auto& ar : rs->audioRoutes)
        {
            if (ar.target == AudioTarget::FilterCutoff || ar.target == AudioTarget::Amp)
                continue;
            const int sourceSlot = audio.slotOf[static_cast<size_t>(ar.source)];
            if (sourceSlot < 0)
                continue;
            const float amount = haveDelta ? withDelta({0, 0, ar.amount, 0, 0.0f}, ar.route).amount : ar.amount;
            const float mean = sources[static_cast<size_t>(kExtAudioFirst + ar.source)];
            for (int o = 0; o < p.numOsc; ++o)
            {
                if (ar.slot >= 0 && ar.slot != o)
                    continue;
                const auto so = static_cast<size_t>(o);
                const bool takes = ar.target == AudioTarget::OscPitch         ? takesPitch[so]
                                   : ar.target == AudioTarget::OscFmAmount    ? takesFm[so]
                                   : ar.target == AudioTarget::OscCrossAmount ? takesCross[so]
                                   : ar.target == AudioTarget::OscPulseWidth  ? takesWidth[so]
                                                                              : takesPosition[so];
                if (takes)
                    continue;
                if (ar.target == AudioTarget::OscPitch)
                    slotMod[so].pitch += amount * mean;
                else if (ar.voiceTarget >= 0 && numVoiceTargets > 0)
                {
                    targetMod[static_cast<size_t>(ar.voiceTarget)] += amount * mean;
                    anyTargetMod = true;
                }
            }
        }
    }

    // Buses: one input per filter and the dry path.
    const int numFilters = std::min(p.numFilter, static_cast<int>(filters.size()));
    chunk.samples = numSamples;
    chunk.numFilters = numFilters;
    chunk.bank = bank;
    chunk.bankVoice = bankVoice;
    chunk.banked.fill(false);
    chunk.hasCutoffMod.fill(false);
    chunk.hasAmpMod = false;
    std::memset(chunk.bus, 0, sizeof(chunk.bus));
    std::memset(chunk.dry, 0, sizeof(chunk.dry));
    std::memset(chunk.carry, 0, sizeof(chunk.carry));
    std::memset(chunk.sum, 0, sizeof(chunk.sum));
    auto& bus = chunk.bus;
    float* const dryL = chunk.dry[0];
    float* const dryR = chunk.dry[1];

    // Serial: everything enters filter 1. Parallel: everything enters every filter. Split: the source's route
    // names a filter, all filters (index modelFilters) or the dry path (modelFilters + 1).
    const auto sendToBuses = [&](int route, const float* l, const float* r)
    {
        const int mode = p.filterRoute;
        const bool toDry = mode == 2 && route == p.modelFilters + 1;
        for (int f = 0; f < numFilters; ++f)
        {
            const bool to = mode == 0 ? f == 0 : (mode == 1 || route == f || route == p.modelFilters);
            if (!to)
                continue;
            for (int i = 0; i < numSamples; ++i)
            {
                bus[f][0][i] += l[i];
                bus[f][1][i] += r[i];
            }
        }
        if (toDry)
            for (int i = 0; i < numSamples; ++i)
            {
                dryL[i] += l[i];
                dryR[i] += r[i];
            }
    };

    // Cross modulation: which slots are listened to, and where the master of a hard sync wraps in this chunk.
    std::array<bool, kMaxOscSlots> needSignal{}, needWraps{};
    for (int o = 0; o < p.numOsc; ++o)
    {
        const auto& op = p.osc[static_cast<size_t>(o)];
        if (op.xmode > 0 && op.xsrc >= 0 && op.xsrc < o)
            (op.xmode == 3 ? needWraps : needSignal)[static_cast<size_t>(op.xsrc)] = true;
    }
    for (int o = 0; o < p.numOsc; ++o)
    {
        if (!needWraps[static_cast<size_t>(o)])
            continue;
        const auto& so = p.osc[static_cast<size_t>(o)];
        const float centre =
            curNote + p.transpose + so.semitones + so.fineCents * 0.01f + pitchBend +
            24.0f * (slotMod[static_cast<size_t>(o)].pitch + mod[static_cast<size_t>(ModDest::Pitch)]) +
            (drift * p.driftAmount + staticDetune) +
            (so.type > 0 && oscillatorInstance(o, so.type) != nullptr && oscillatorInstance(o, so.type)->usesVoiceCard()
                 ? 0.0f
                 : cardTune);
        const double inc = static_cast<double>(noteToHz(centre)) / sampleRate;
        auto& w = wraps[static_cast<size_t>(o)];
        w.count = 0;
        double ph = refPhase[static_cast<size_t>(o)];
        const double total = ph + inc * numSamples;
        const int crossings = static_cast<int>(std::floor(total));
        for (int j = 1; j <= crossings && w.count < 64; ++j)
            w.time[static_cast<size_t>(w.count++)] =
                static_cast<float>(std::min((static_cast<double>(j) - ph) / inc, static_cast<double>(numSamples)));
        refPhase[static_cast<size_t>(o)] = total - crossings;
    }

    // Oscillator outputs that the audio bus carries need the slot's mono signal.
    if (arOn)
        for (int o = 0; o < p.numOsc; ++o)
            if (audio.slotOf[static_cast<size_t>(kAudioOscFirst + o)] >= 0)
                needSignal[static_cast<size_t>(o)] = true;
    const bool trackAudio = mods != nullptr && mods->needsAudio;
    if (trackAudio)
        std::fill(extra->mixNow.begin(), extra->mixNow.begin() + numSamples, 0.0f);
    const auto& voiceTargets_ = model->voiceTargets();

    for (int o = 0; o < p.numOsc; ++o)
    {
        float oL[kChunkSize] = {}, oR[kChunkSize] = {};
        const auto& op = p.osc[static_cast<size_t>(o)];
        float* mono = needSignal[static_cast<size_t>(o)] ? modSignal[static_cast<size_t>(o)].data() : nullptr;
        if (mono != nullptr)
            std::fill(mono, mono + kChunkSize, 0.0f);
        const float override = mono != nullptr ? 1.0f : -1.0f;

        // Audio-rate modulation of this slot from the bus: sums of the routes' sources, per sample.
        float* const pitchBuf = scratch.osc[0].data();
        float* const fmBuf = scratch.osc[1].data();
        float* const crossBuf = scratch.osc[2].data();
        float* const pwBuf = scratch.osc[3].data();
        float* const posBuf = scratch.osc[4].data();
        bool hasPitch = false, hasFm = false, hasCross = false, hasPw = false, hasPos = false;
        if (arOn && rs != nullptr)
        {
            const auto sum = [&](AudioTarget kind, float* dest)
            {
                bool any = false;
                for (const auto& ar : rs->audioRoutes)
                {
                    if (ar.target != kind || !(ar.slot == o || (kind == AudioTarget::OscPitch && ar.slot < 0)))
                        continue;
                    const int bufSlot = audio.slotOf[static_cast<size_t>(ar.source)];
                    if (bufSlot < 0 || !audio.filled[static_cast<size_t>(ar.source)])
                        continue; // not yet rendered (a later oscillator's output) or not carried: reads as zero
                    float scale = 1.0f;
                    if (kind == AudioTarget::OscPitch)
                        scale = 24.0f;
                    else if (kind == AudioTarget::OscPulseWidth && ar.voiceTarget >= 0)
                        scale = voiceTargets_[static_cast<size_t>(ar.voiceTarget)].range;
                    const float amount =
                        (haveDelta ? withDelta({0, 0, ar.amount, 0, 0.0f}, ar.route).amount : ar.amount) * scale;
                    if (!any)
                    {
                        std::fill(dest, dest + numSamples, 0.0f);
                        any = true;
                    }
                    const auto& b = audio.buf[static_cast<size_t>(bufSlot)];
                    for (int i = 0; i < numSamples; ++i)
                        dest[i] += amount * b[static_cast<size_t>(i)];
                }
                return any;
            };
            const auto so = static_cast<size_t>(o);
            hasPitch = takesPitch[so] && sum(AudioTarget::OscPitch, pitchBuf);
            hasFm = takesFm[so] && op.type > 0 && sum(AudioTarget::OscFmAmount, fmBuf);
            hasCross = takesCross[so] && op.type == 0 && sum(AudioTarget::OscCrossAmount, crossBuf);
            hasPw = takesWidth[so] && op.type > 0 && sum(AudioTarget::OscPulseWidth, pwBuf);
            hasPos = takesPosition[so] && op.type == 0 && sum(AudioTarget::OscPosition, posBuf);
        }

        if (op.type > 0 && p.values != nullptr)
        {
            OscAudio oa;
            oa.pitch = hasPitch ? pitchBuf : nullptr;
            oa.fm = hasFm ? fmBuf : nullptr;
            oa.pw = hasPw ? pwBuf : nullptr;
            renderModule(o, p, slotMod, anyTargetMod ? targetMod.data() : nullptr, pitchBend, numSamples, oL, oR, mono,
                         override, hasPitch || hasFm || hasPw ? &oa : nullptr);
        }
        else if (hasPitch || hasCross || hasPos)
        {
            // The wavetable oscillator takes audio-rate modulation in groups of four samples (a 12 kHz control rate at
            // 48 kHz), with the phase running on, except where hard sync from another slot needs the whole chunk.
            const bool grouped = !(op.xmode == 3 && op.xsrc >= 0 && op.xsrc < o);
            const auto mean = [](const float* x, int from, int count)
            {
                float m = 0.0f;
                for (int i = 0; i < count; ++i)
                    m += x[from + i];
                return m / static_cast<float>(count);
            };
            auto local = slotMod;
            auto& lm = local[static_cast<size_t>(o)];
            if (grouped)
            {
                for (int i0 = 0; i0 < numSamples; i0 += 4)
                {
                    const int m = std::min(4, numSamples - i0);
                    lm = slotMod[static_cast<size_t>(o)];
                    if (hasPitch)
                        lm.pitch += mean(pitchBuf, i0, m) / 24.0f;
                    if (hasCross)
                        lm.xamt += mean(crossBuf, i0, m);
                    if (hasPos)
                        lm.position += mean(posBuf, i0, m);
                    renderOscillator(o, p, mod, local, pitchBend, m, oL + i0, oR + i0, mono != nullptr ? mono + i0 : nullptr,
                                     override);
                }
            }
            else
            {
                if (hasPitch)
                    lm.pitch += mean(pitchBuf, 0, numSamples) / 24.0f;
                if (hasCross)
                    lm.xamt += mean(crossBuf, 0, numSamples);
                if (hasPos)
                    lm.position += mean(posBuf, 0, numSamples);
                renderOscillator(o, p, mod, local, pitchBend, numSamples, oL, oR, mono, override);
            }
        }
        else
            renderOscillator(o, p, mod, slotMod, pitchBend, numSamples, oL, oR, mono, override);
        if (arOn)
        {
            const int a = kAudioOscFirst + o;
            const signed char bufSlot = audio.slotOf[static_cast<size_t>(a)];
            if (bufSlot >= 0 && mono != nullptr)
            {
                auto& b = audio.buf[static_cast<size_t>(bufSlot)];
                float m = 0.0f;
                for (int i = 0; i < numSamples; ++i)
                {
                    b[static_cast<size_t>(i)] = mono[i];
                    m += mono[i];
                }
                audio.filled[static_cast<size_t>(a)] = true;
                audio.lastMean[static_cast<size_t>(a)] = m / static_cast<float>(numSamples);
            }
        }
        if (std::clamp(op.level + slotMod[static_cast<size_t>(o)].level, 0.0f, 1.0f) > 0.0f)
        {
            sendToBuses(op.route, oL, oR);
            if (trackAudio)
                for (int i = 0; i < numSamples; ++i)
                    extra->mixNow[static_cast<size_t>(i)] += 0.5f * (oL[i] + oR[i]);
        }
    }
    if (trackAudio)
    {
        extra->mixPrev = extra->mixNow;
        mixPrevCount = numSamples;
    }

    const float subLevel = std::clamp(p.subLevel + mod[static_cast<size_t>(ModDest::SubLevel)], 0.0f, 1.0f);
    if (subLevel > 0.0f)
    {
        float sL[kChunkSize], sR[kChunkSize];
        const float hz = noteToHz(curNote + p.transpose + pitchBend + 24.0f * mod[static_cast<size_t>(ModDest::Pitch)] +
                                  (drift * p.driftAmount + staticDetune) + cardTune) *
                         (p.subOctave == 0 ? 0.5f : 0.25f);
        const float inc = std::min(hz / static_cast<float>(sampleRate), 0.49f);
        for (int i = 0; i < numSamples; ++i)
        {
            float s;
            switch (p.subWave)
            {
            case 1:
                s = subPhase < 0.5f ? 4.0f * subPhase - 1.0f : 3.0f - 4.0f * subPhase;
                break;
            case 2:
                s = (subPhase < 0.5f ? 1.0f : -1.0f) + polyBlep(subPhase, inc) -
                    polyBlep(std::fmod(subPhase + 0.5f, 1.0f), inc);
                break;
            case 3:
                s = 2.0f * subPhase - 1.0f - polyBlep(subPhase, inc);
                break;
            default:
                s = std::sin(kTwoPi * subPhase);
                break;
            }
            sL[i] = sR[i] = s * subLevel;
            subPhase += inc;
            if (subPhase >= 1.0f)
                subPhase -= 1.0f;
        }
        sendToBuses(p.subRoute, sL, sR);
    }

    const float noiseLevel = std::clamp(p.noiseLevel + mod[static_cast<size_t>(ModDest::NoiseLevel)], 0.0f, 1.0f);
    if (noiseLevel > 0.0f)
    {
        float nL[kChunkSize], nR[kChunkSize];
        for (int i = 0; i < numSamples; ++i)
        {
            nL[i] = noiseSample(0, p.noiseType) * noiseLevel * 0.5f;
            nR[i] = noiseSample(1, p.noiseType) * noiseLevel * 0.5f;
        }
        sendToBuses(p.noiseRoute, nL, nR);
    }

    auto& run = chunk.run;
    static constexpr std::array<ModDest, 2> cutoffDest = {ModDest::Cutoff, ModDest::Cutoff2};
    static constexpr std::array<ModDest, 2> resDest = {ModDest::Resonance, ModDest::Resonance2};
    static constexpr std::array<ModDest, 2> driveDest = {ModDest::Drive, ModDest::Drive2};
    static constexpr std::array<ModDest, 2> mixDest = {ModDest::Filter1Mix, ModDest::Filter2Mix};
    static constexpr std::array<ModDest, 2> panDest = {ModDest::Filter1Pan, ModDest::Filter2Pan};
    for (size_t f = 0; f < static_cast<size_t>(numFilters); ++f)
    {
        const auto& fp = p.filter[f];
        const bool fixed = f < kNumFilters;
        const auto at = [&](const std::array<ModDest, 2>& d) { return fixed ? mod[static_cast<size_t>(d[f])] : 0.0f; };
        const auto& tg = fp.targets;
        const float cutoffMod = fixed ? at(cutoffDest) : targetRaw(tg[0]);
        const float octaves = fp.keytrack * (static_cast<float>(midiNote) - 60.0f) / 12.0f +
                              fp.envAmount * 7.0f * modEnvValue + 5.0f * cutoffMod;
        FilterSettings fs;
        fs.model = fp.model;
        fs.cutoffHz = std::clamp(fp.cutoff * std::exp2(octaves), 20.0f, 20000.0f);
        fs.resonance = std::clamp(fp.resonance + (fixed ? at(resDest) : targetDelta(tg[1])), 0.0f, 1.0f);
        fs.character = fp.character;
        fs.highpass = fp.highpass;
        fs.formants = f < p.formants.size() ? p.formants[f] : nullptr;
        if (cardOn && p.card.filter > 0.0f && f < static_cast<size_t>(kCardFilters))
            fs.tolerance = card.filterTolerance(static_cast<int>(f), p.card.filter);
        const float drive = std::clamp(fp.drive + (fixed ? at(driveDest) : targetDelta(tg[2])), 0.0f, 1.0f);
        const int modelIndex = std::clamp(fp.model, 0, static_cast<int>(FilterModel::Count) - 1);
        if (bank != nullptr && bank->ready() && static_cast<int>(f) < bank->slots() &&
            FilterChannel::isAnalogModel(modelIndex))
        {
            // The nonlinear models of all sounding voices run together in the bank; the voice only keeps its own
            // channels' model bookkeeping (a model change resets, as in FilterChannel::configure).
            const int analogIndex = modelIndex - static_cast<int>(FilterModel::TransistorLadder);
            filters[f].trackModel(modelIndex);
            bank->seedVoice(static_cast<int>(f), analogIndex, bankVoice, cardOn && f < static_cast<size_t>(kCardFilters),
                            card.filterSeed[std::min(f, static_cast<size_t>(kCardFilters - 1))]);
            if (bankModel[f] != analogIndex)
            {
                bank->resetVoice(static_cast<int>(f), analogIndex, bankVoice);
                bankModel[f] = analogIndex;
            }
            if (fp.on)
            {
                bank->configure(static_cast<int>(f), analogIndex, bankVoice, fs, drive);
                chunk.banked[f] = true;
            }
        }
        else
        {
            bankModel[f] = -1;
            filters[f].configure(fs, drive, sampleRate);
        }
        if (arOn && fp.on)
        {
            // Audio-rate cutoff modulation, 5 octaves per unit like the matrix: routes and the filter's own FM source.
            float* cm = extra->cutoffMod[f];
            bool any = false;
            const auto addSource = [&](int src, float amount)
            {
                const int bufSlot = audio.slotOf[static_cast<size_t>(src)];
                if (bufSlot < 0 || !audio.filled[static_cast<size_t>(src)])
                    return;
                if (!any)
                {
                    std::fill(cm, cm + numSamples, 0.0f);
                    any = true;
                }
                const auto& b = audio.buf[static_cast<size_t>(bufSlot)];
                for (int i = 0; i < numSamples; ++i)
                    cm[i] += amount * 5.0f * b[static_cast<size_t>(i)];
            };
            if (rs != nullptr)
                for (const auto& ar : rs->audioRoutes)
                    if (ar.target == AudioTarget::FilterCutoff && ar.slot == static_cast<int>(f))
                        addSource(ar.source, haveDelta ? withDelta({0, 0, ar.amount, 0, 0.0f}, ar.route).amount : ar.amount);
            if (fp.fmSource > 0 && fp.fmAmount != 0.0f)
                addSource(fp.fmSource - 1, fp.fmAmount);
            if (any)
            {
                chunk.hasCutoffMod[f] = true;
                if (chunk.banked[f])
                    bank->setCutoffMod(static_cast<int>(f), bankVoice, cm, numSamples);
            }
        }
        const float pan = std::clamp(fp.pan + (fixed ? at(panDest) : targetDelta(tg[4])), -1.0f, 1.0f);
        const float angle = (pan + 1.0f) * 0.78539816f;
        run[f] = {fp.on, std::clamp(fp.mix + (fixed ? at(mixDest) : targetDelta(tg[3])), 0.0f, 1.0f),
                  std::cos(angle) * kSqrt2, std::sin(angle) * kSqrt2};
    }
    chunk.ampMod = std::clamp(1.0f + mod[static_cast<size_t>(ModDest::AmpLevel)], 0.0f, 2.0f);
    if (arOn && rs != nullptr)
    {
        // Audio-rate amplitude modulation (ring / amplitude modulation by an oscillator or an audio-rate LFO).
        for (const auto& ar : rs->audioRoutes)
        {
            if (ar.target != AudioTarget::Amp)
                continue;
            const int bufSlot = audio.slotOf[static_cast<size_t>(ar.source)];
            if (bufSlot < 0 || !audio.filled[static_cast<size_t>(ar.source)])
                continue;
            if (!chunk.hasAmpMod)
            {
                std::fill(extra->ampBuf, extra->ampBuf + numSamples, 0.0f);
                chunk.hasAmpMod = true;
            }
            const float amount = haveDelta ? withDelta({0, 0, ar.amount, 0, 0.0f}, ar.route).amount : ar.amount;
            const auto& b = audio.buf[static_cast<size_t>(bufSlot)];
            for (int i = 0; i < numSamples; ++i)
                extra->ampBuf[i] += amount * b[static_cast<size_t>(i)];
        }
        if (chunk.hasAmpMod)
            for (int i = 0; i < numSamples; ++i)
                extra->ampBuf[i] = std::clamp(1.0f + extra->ampBuf[i], 0.0f, 2.0f);
    }
    chunk.serial = p.filterRoute == 0;
    return true;
}

// A filter stage's output joins the chain: serial stages feed the next one, parallel and split stages add up.
void Voice::emitStage(int slot, const float* wetL, const float* wetR)
{
    const auto f = static_cast<size_t>(slot);
    const FilterRun& r = chunk.run[f];
    const float (*in)[kChunkSize] = chunk.in[f];
    float* const outL = chunk.serial ? chunk.carry[0] : chunk.sum[0];
    float* const outR = chunk.serial ? chunk.carry[1] : chunk.sum[1];
    for (int i = 0; i < chunk.samples; ++i)
    {
        float oL = in[0][i], oR = in[1][i];
        if (r.on)
        {
            oL = (in[0][i] + (wetL[i] - in[0][i]) * r.mix) * r.panL;
            oR = (in[1][i] + (wetR[i] - in[1][i]) * r.mix) * r.panR;
        }
        if (chunk.serial)
        {
            outL[i] = oL;
            outR[i] = oR;
        }
        else
        {
            outL[i] += oL;
            outR[i] += oR;
        }
    }
}

// Input of the filter in a slot: its bus, plus in serial routing the previous stage's output. A filter that runs in the
// voice is processed here and its output joins the chain; one that runs in the bank only receives its input, and
// filterOut() collects the result once the bank has processed the slot.
void Voice::filterIn(int slot)
{
    const auto f = static_cast<size_t>(slot);
    const bool carryIn = chunk.serial && slot > 0;
    for (int i = 0; i < chunk.samples; ++i)
    {
        float inL = chunk.bus[f][0][i], inR = chunk.bus[f][1][i];
        if (carryIn)
        {
            inL += chunk.carry[0][i];
            inR += chunk.carry[1][i];
        }
        chunk.in[f][0][i] = inL;
        chunk.in[f][1][i] = inR;
    }
    if (!chunk.run[f].on)
    {
        emitStage(slot, chunk.in[f][0], chunk.in[f][1]);
        return;
    }
    if (chunk.banked[f])
    {
        std::memcpy(chunk.bank->lane(slot, chunk.bankVoice, 0), chunk.in[f][0],
                    sizeof(float) * static_cast<size_t>(chunk.samples));
        std::memcpy(chunk.bank->lane(slot, chunk.bankVoice, 1), chunk.in[f][1],
                    sizeof(float) * static_cast<size_t>(chunk.samples));
        return;
    }
    float wetL[kChunkSize], wetR[kChunkSize];
    if (chunk.hasCutoffMod[f])
    {
        std::memcpy(wetL, chunk.in[f][0], sizeof(float) * static_cast<size_t>(chunk.samples));
        std::memcpy(wetR, chunk.in[f][1], sizeof(float) * static_cast<size_t>(chunk.samples));
        filters[f].processBlock(wetL, wetR, chunk.samples, extra->cutoffMod[f]);
        emitStage(slot, wetL, wetR);
        return;
    }
    std::memcpy(wetL, chunk.in[f][0], sizeof(float) * static_cast<size_t>(chunk.samples));
    std::memcpy(wetR, chunk.in[f][1], sizeof(float) * static_cast<size_t>(chunk.samples));
    filters[f].processBlock(wetL, wetR, chunk.samples, nullptr);
    emitStage(slot, wetL, wetR);
}

void Voice::filterOut(int slot)
{
    if (!chunk.banked[static_cast<size_t>(slot)])
        return;
    emitStage(slot, chunk.bank->lane(slot, chunk.bankVoice, 0), chunk.bank->lane(slot, chunk.bankVoice, 1));
}

void Voice::endChunk(float* left, float* right)
{
    const float* const sumL = chunk.serial ? chunk.carry[0] : chunk.sum[0];
    const float* const sumR = chunk.serial ? chunk.carry[1] : chunk.sum[1];
    // Audio-rate amplitude modulation, when a route asks for it, multiplies per sample; the common case has no such term.
    const float* const ampBuf = chunk.hasAmpMod ? extra->ampBuf : nullptr;
    for (int i = 0; i < chunk.samples; ++i)
    {
        if (velGainRamp > 0) [[unlikely]]
            velGain = --velGainRamp > 0 ? velGain + velGainStep : velocity;
        float ampLevel = env[0].tick() * velGain * chunk.ampMod;
        if (ampBuf != nullptr) [[unlikely]]
            ampLevel *= ampBuf[i];
        if (cardGain != 1.0f)
            ampLevel *= cardGain;
        if (fading)
        {
            const float x = static_cast<float>(fadeDone) / static_cast<float>(fadeLength);
            ampLevel *= x >= 1.0f ? 0.0f : 0.5f + 0.5f * std::cos(3.14159265f * x);
            ++fadeDone;
        }
        left[i] += (sumL[i] + chunk.dry[0][i]) * ampLevel;
        right[i] += (sumR[i] + chunk.dry[1][i]) * ampLevel;
    }
    finishChunk();
}

void Voice::finishChunk()
{
    noteTime += static_cast<double>(chunk.samples) / sampleRate;
    if (env[0].isIdle() || (fading && fadeDone >= fadeLength))
    {
        if (pending.on)
        {
            // The stolen voice has faded out: the note that took it over starts now.
            const Pending p = pending;
            for (auto& e : env)
                e.reset(); // silent now: the new note's envelope attacks from zero, not from the old level
            noteOn(p.note, p.velocity, *p.params, p.age, p.startNote, p.detune);
            if (p.released)
                noteOff();
            return;
        }
        active = false;
        fading = false;
    }
}

} // namespace crucible
