// SPDX-License-Identifier: MIT
// fx.phaser2: a phaser made of first-order all-pass stages in the zero-delay-feedback (TPT) form.
//
//   * 2, 4, 6, 8 or 12 identical stages. A stage with break frequency fc (g = tan(pi fc / fs)) is the bilinear transform of
//     the first-order all-pass, so the phase of N stages is N x 2 atan(tan(pi f / fs) / g) and, mixed 50 / 50
//     with the dry signal, the notches fall where that phase is an odd multiple of pi: f_k = fs / pi x atan(g x tan((2k + 1)
//     pi / 2N)). The tests check this prediction.
//   * Sweep: fc = Center x 2^(Range x u) with u in [-1, 1] from a sine or triangle oscillator, from the envelope of the
//     input, or from the envelope of the sidechain input (the input itself when none is connected). The two channels use
//     oscillators Spread x half a cycle apart.
//   * Stage type: Linear, OTA (each stage integrates tanh(k x) / k, a symmetric soft limit) or JFET (the same, saturating at
//   1 on
//     one side and at 2 on the other with the same slope at zero, so it adds even harmonics). Each stage is a nonlinear
//     resistor in series with a capacitor (a passive one-port: its gain never exceeds 1), solved for every stage and sample
//     with one Newton step from the linear solution. k follows Drive; the small-signal response equals the linear one.
//   * Feedback: the last stage feeds the first one a sample later, through a soft saturation (Feedback Saturation), positive
//   or
//     negative.
//   * Barber-pole: the stages are arranged as N / 2 parallel pairs, each pair a two-stage all-pass that makes one notch. The
//     pairs are spread evenly over the sweep range (Center x 2^+-Range) and all travel the same way, up or down, for ever: a
//     pair enters at the bottom of the range, leaves at the top, and its contribution is weighted by sin^2 of its position,
//     so it fades in and out and the wrap is never heard. The wet signal is x + sum of w (pair(x) - x).
//   * The feedback path is high-passed (about 12 Hz) before the saturation: an asymmetric stage makes DC, which the all-pass
//     chain passes at unity gain.
//
// Rate, center, range, feedback, drive and spread are smoothed, and the coefficients are interpolated inside each chunk of
// 16 samples. Changing the stage count, the stage type, the sweep source or the barber-pole mode fades the wet signal out,
// clears the stages and fades it back in.
#include "FxModuleSupport.h"

#include <crucible/module/LatencyReporter.h>

#include <atomic>
#include <cstdint>

namespace crucible::modules::fx
{
namespace
{
constexpr int kChunk = 16;
constexpr int kMaxStages = 12;
constexpr double kTwoPi = 6.283185307179586;
constexpr double kPi = 3.141592653589793;
constexpr int kStageChoices[] = {2, 4, 6, 8, 12};

// Rational tanh, exact at 0 and 3 (saturated beyond).
inline float tanhA(float x)
{
    x = x < -3.0f ? -3.0f : (x > 3.0f ? 3.0f : x);
    const float x2 = x * x;
    return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// The JFET stage's characteristic saturates at 1 on one side and at kJfetAsym on the other, with the same slope at zero.
constexpr float kJfetAsym = 2.0f;

struct Block
{
    int stages = 4, type = 0, barber = 0, source = 0, shape = 0;
    double center = 800.0, range = 1.5, rate = 0.4;
    float mix = 0.5f, feedback = 0.3f, fbSat = 0.0f, spread = 0.5f, driveDb = 6.0f, envSens = 0.0f, envAttack = 5.0f,
          envRelease = 120.0f;
};

struct Config
{
    int stages = 4, type = 0, barber = 0, source = 0;
    bool operator==(const Config& o) const
    {
        return stages == o.stages && type == o.type && barber == o.barber && source == o.source;
    }
    bool operator!=(const Config& o) const { return !(*this == o); }
};

// Per-sample constants of a saturating stage, worked out off the audio path: G = g / (1 + g), iP = 1 - G, kk = k (1 - G)
// (the argument of the tanh is kk x (a - s) at the linear solution), gj = g, ci = 1 / k, p = 1 + g. The JFET stage uses kkN
// = kk / kJfetAsym and ciN = kJfetAsym / k for negative drive.
struct StageK
{
    float G, iP, kk, gj, ci, p, kkN, ciN;
};

template <int Type> inline StageK stageK(float g, float G, float k, float invK)
{
    StageK q;
    q.G = G;
    q.iP = 1.0f - G;
    q.kk = k * (1.0f - G);
    q.gj = g;
    q.ci = invK;
    q.p = 1.0f + g;
    q.kkN = Type == 2 ? q.kk * (1.0f / kJfetAsym) : q.kk;
    q.ciN = Type == 2 ? invK * kJfetAsym : invK;
    return q;
}

// One stage: input a, state s. Returns the all-pass output. The stage is a series nonlinear resistor and capacitor seen as a
// wave one-port: with the resistor current y = tanhA(k z) / k (z = a - s - v is its voltage, v = g y the capacitor step),
// the output is a - 2 y and the state moves by 2 v. A passive resistor and a capacitor make a passive one-port, so |output|
// never exceeds |input| in the energy sense whatever the saturation (the linear stage is the case y = z). The integrator
// equation v = g tanhA(k (u - v)) / k (u = a - s) takes one Newton step from the linear solution v0 = G u; with the rational
// tanh t = N / D written out, 1 - t^2 = (D^2 - N^2) / D^2, so the step is (v0 D - c N) D / (p D^2 - gj N^2) with c = gj / k:
// one division. The step is worked out as r = step / g, so that y = (1 - G) u - r / 2 needs no division by g.
template <int Type> inline float allpass(float a, float& s, const StageK& q)
{
    const float u = a - s;
    float w = a - 2.0f * q.iP * u;
    float s2 = s + 2.0f * q.G * u;
    if constexpr (Type != 0)
    {
        float x = q.kk * u;
        float ci = q.ci;
        if constexpr (Type == 2)
            if (u < 0.0f)
            {
                x = q.kkN * u;
                ci = q.ciN;
            }
        if (x < -3.0f || x > 3.0f) [[unlikely]]
            x = x < 0.0f ? -3.0f : 3.0f;
        const float x2 = x * x;
        const float num = x * (27.0f + x2), den = 27.0f + 9.0f * x2;
        const float top = u * q.iP * den - ci * num;
        const float r = top * (2.0f * den) / (q.p * den * den - q.gj * num * num);
        w += r;
        s2 -= q.gj * r;
    }
    s = s2;
    return w;
}

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_phaser2_mix")), stages(localIndex(d, "fx_phaser2_stages")),
          center(localIndex(d, "fx_phaser2_center")), range(localIndex(d, "fx_phaser2_range")),
          rate(localIndex(d, "fx_phaser2_rate")), shape(localIndex(d, "fx_phaser2_shape")),
          feedback(localIndex(d, "fx_phaser2_feedback")), fbSat(localIndex(d, "fx_phaser2_fb_sat")),
          spread(localIndex(d, "fx_phaser2_spread")), type(localIndex(d, "fx_phaser2_type")),
          drive(localIndex(d, "fx_phaser2_drive")), source(localIndex(d, "fx_phaser2_source")),
          envSens(localIndex(d, "fx_phaser2_env_sens")), envAttack(localIndex(d, "fx_phaser2_env_attack")),
          envRelease(localIndex(d, "fx_phaser2_env_release")), barber(localIndex(d, "fx_phaser2_barber"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        sampleRate = c.sampleRate;
        fadeStep = static_cast<float>(1.0 / (0.015 * sampleRate));
        smoothMix = static_cast<float>(1.0 - std::exp(-1.0 / (0.005 * sampleRate)));
        alpha = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.02 * sampleRate));
        alpha40 = 1.0 - std::exp(-static_cast<double>(kChunk) / (0.04 * sampleRate));
        dcCoef = static_cast<float>(1.0 - std::exp(-kTwoPi * 12.0 / sampleRate));
        reset();
    }

    void reset() override
    {
        clearStages();
        needInit = true;
        switching = false;
        fade = 1.0f;
        mixSmooth = mixPre = 0.0f;
        phase = 0.0;
        barberPhase = 0.0;
        env = 0.0;
        cleared = true;
    }

    void deactivate() override
    {
        if (!cleared)
            reset();
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        cleared = false;
        const Block b = readBlock(in);
        const Config want{b.stages, b.type, b.barber, b.source};
        if (needInit)
        {
            active = want;
            centerS = b.center;
            rangeS = b.range;
            rateS = b.rate;
            fbS = b.feedback;
            fbNow = b.feedback;
            fbSatS = b.fbSat;
            spreadS = b.spread;
            kS = kStart = static_cast<float>(std::pow(10.0, b.driveDb / 20.0));
            mixSmooth = mixPre = b.mix;
            for (int c = 0; c < 2; ++c)
                coefficients(c, 0.0, true);
            needInit = false;
        }
        if (!switching && want != active)
            switching = true;
        const double lp = std::abs(static_cast<double>(b.feedback));
        tail.store(0.05 + (lp > 0.01 ? std::min(10.0, 0.01 * std::log(1.0e-3) / std::log(std::min(lp, 0.999))) : 0.0),
                   std::memory_order_relaxed);

        const float* auxL = nullptr;
        const float* auxR = nullptr;
        if (b.source == 2 && in.hasAux())
        {
            auxL = in.auxLeft();
            auxR = in.auxRight();
        }
        for (int pos = 0; pos < n; pos += kChunk)
        {
            const int m = std::min(kChunk, n - pos);
            const float fade0 = fade;
            bool applied = false;
            if (switching)
            {
                fade -= static_cast<float>(m) * fadeStep;
                if (fade <= 0.0f)
                {
                    fade = 0.0f;
                    applied = true;
                }
            }
            else if (fade < 1.0f)
                fade = std::min(1.0f, fade + static_cast<float>(m) * fadeStep);
            chunk(l + pos, r + pos, m, b, fade0, fade, auxL ? auxL + pos : nullptr, auxR ? auxR + pos : nullptr);
            if (applied)
            {
                active = want;
                clearStages();
                switching = false;
                for (int c = 0; c < 2; ++c)
                    coefficients(c, 0.0, true);
            }
        }
    }

    int latencySamples() const override { return 0; }
    double tailSeconds() const override { return tail.load(std::memory_order_relaxed); }

private:
    Block readBlock(const EffectInputs& in) const
    {
        Block b;
        b.stages = kStageChoices[std::clamp(in.choice(stages), 0, 4)];
        b.type = std::clamp(in.choice(type), 0, 2);
        b.barber = std::clamp(in.choice(barber), 0, 2);
        b.source = std::clamp(in.choice(source), 0, 2);
        b.shape = std::clamp(in.choice(shape), 0, 1);
        b.center = static_cast<double>(in.value(center));
        b.range = static_cast<double>(in.value(range));
        b.rate = static_cast<double>(in.value(rate));
        b.mix = in.value(mix);
        b.feedback = in.value(feedback);
        b.fbSat = in.value(fbSat);
        b.spread = in.value(spread);
        b.driveDb = in.value(drive);
        b.envSens = in.value(envSens);
        b.envAttack = in.value(envAttack);
        b.envRelease = in.value(envRelease);
        return b;
    }

    void clearStages()
    {
        for (auto& ch : s)
            for (float& v : ch)
                v = 0.0f;
        fbPrev[0] = fbPrev[1] = 0.0f;
        fbDc[0] = fbDc[1] = outDc[0] = outDc[1] = 0.0f;
    }

    static double wave(int shapeIndex, double p)
    {
        p -= std::floor(p);
        if (shapeIndex == 0)
            return std::sin(kTwoPi * p);
        return p < 0.25 ? 4.0 * p : (p < 0.75 ? 2.0 - 4.0 * p : 4.0 * p - 4.0);
    }

    // The saturating stages solve their integrator with one Newton step from the linear solution; with the break frequency
    // near Nyquist (g above about 2.5) and 24 dB of drive that step leaves the passive region and a fast sweep with feedback
    // 0.95 diverges, so the saturating types stop the sweep at 0.3 fs (g = 2.0).
    double clampFc(double fc) const { return std::clamp(fc, 10.0, (active.type == 0 ? 0.45 : 0.3) * sampleRate); }

    // Sets the coefficients of channel c for the sweep position u (or the barber conveyor phase) and, when `jump` is set,
    // makes them the current ones; otherwise they are the targets the chunk ramps to.
    void coefficients(int c, double u, bool jump)
    {
        if (active.barber == 0)
        {
            const double g = std::tan(kPi * clampFc(centerS * std::exp2(rangeS * u)) / sampleRate);
            tg[c][0] = static_cast<float>(g);
            tG[c][0] = static_cast<float>(g / (1.0 + g));
            tw[c][0] = 1.0f;
        }
        else
        {
            const double lo = clampFc(centerS * std::exp2(-rangeS)), hi = clampFc(centerS * std::exp2(rangeS));
            const double ratio = std::log2(hi / lo);
            const double off = c == 0 ? 0.0 : 0.5 * static_cast<double>(spreadS);
            const int pairs = active.stages / 2;
            // The pair weights sum to pairs / 2; above 1 the dry term (1 - sum) turns negative and the wet sum can exceed
            // unity gain, so the weights are scaled to a sum of at most 1 (a convex mix of all-passes, gain at most 1 at
            // every frequency).
            double sum = 0.0;
            for (int i = 0; i < pairs; ++i)
            {
                double p = u + off + static_cast<double>(i) / static_cast<double>(pairs);
                p -= std::floor(p);
                const double sn = std::sin(kPi * p);
                sum += sn * sn;
            }
            const double norm = sum > 1.0 ? 1.0 / sum : 1.0;
            for (int i = 0; i < pairs; ++i)
            {
                double p = u + off + static_cast<double>(i) / static_cast<double>(pairs);
                p -= std::floor(p);
                const double g = std::tan(kPi * clampFc(lo * std::exp2(ratio * p)) / sampleRate);
                const double sn = std::sin(kPi * p);
                tg[c][i] = static_cast<float>(g);
                tG[c][i] = static_cast<float>(g / (1.0 + g));
                tw[c][i] = static_cast<float>(sn * sn * norm);
            }
        }
        if (jump)
            for (int i = 0; i < kMaxStages; ++i)
            {
                cg[c][i] = tg[c][active.barber == 0 ? 0 : i];
                cG[c][i] = tG[c][active.barber == 0 ? 0 : i];
                cw[c][i] = tw[c][active.barber == 0 ? 0 : i];
            }
    }

    // What the next sample feeds back: the stage output without its DC (a one-pole high-pass; an asymmetric stage makes DC
    // and the all-pass chain passes it at unity gain, so the loop would otherwise build it up), then the soft saturation.
    float feedbackSample(int c, float x)
    {
        const float d = x - fbDc[c];
        fbDc[c] += dcCoef * d;
        const float y = (1.0f - dcCoef) * d; // x minus the updated DC estimate, one operation after x
        if (fbSatF == 0.0f)
            return y;
        return y + fbSatF * (0.5f * tanhA(2.0f * y) - y);
    }

    // Linear stage in the form out = c a + (1 - c) s with c = 2G - 1 (G = g / (1 + g)): one multiply-add on the path from
    // input to output, so a long chain is not limited by the latency of the zero-delay-feedback solution. The stage count is
    // a template argument, so the chain is unrolled.
    template <int N>
    void linearLoop(const float* inL, const float* inR, float* wl, float* wr, int m, float fb, float dfb, float c0, float c1,
                    float d0, float d1)
    {
        float* sl = s[0];
        float* sr = s[1];
        for (int i = 0; i < m; ++i)
        {
            fb += dfb;
            c0 += d0;
            c1 += d1;
            const float e0 = 1.0f - c0, e1 = 1.0f - c1;
            float a = inL[i] + fb * fbPrev[0];
            float b = inR[i] + fb * fbPrev[1];
            for (int k = 0; k < N; ++k)
            {
                const float p0 = e0 * sl[k], p1 = e1 * sr[k];
                const float o0 = c0 * a + p0, o1 = c1 * b + p1;
                sl[k] = o0 + (a - sl[k]);
                sr[k] = o1 + (b - sr[k]);
                a = o0;
                b = o1;
            }
            fbPrev[0] = feedbackSample(0, a);
            fbPrev[1] = feedbackSample(1, b);
            wl[i] = a;
            wr[i] = b;
        }
    }

    template <int Type> void runShared(const float* inL, const float* inR, float* wl, float* wr, int m, float fb)
    {
        const float inv = 1.0f / static_cast<float>(m);
        const int count = active.stages;
        if constexpr (Type == 0)
        {
            const float dfb = (fbEnd - fb) * inv;
            float c0 = 2.0f * cG[0][0] - 1.0f, c1 = 2.0f * cG[1][0] - 1.0f;
            const float d0 = (2.0f * tG[0][0] - 1.0f - c0) * inv, d1 = (2.0f * tG[1][0] - 1.0f - c1) * inv;
            switch (count)
            {
            case 2:
                linearLoop<2>(inL, inR, wl, wr, m, fb, dfb, c0, c1, d0, d1);
                break;
            case 4:
                linearLoop<4>(inL, inR, wl, wr, m, fb, dfb, c0, c1, d0, d1);
                break;
            case 6:
                linearLoop<6>(inL, inR, wl, wr, m, fb, dfb, c0, c1, d0, d1);
                break;
            case 8:
                linearLoop<8>(inL, inR, wl, wr, m, fb, dfb, c0, c1, d0, d1);
                break;
            default:
                linearLoop<12>(inL, inR, wl, wr, m, fb, dfb, c0, c1, d0, d1);
                break;
            }
        }
        else
        {
            float* sl = s[0];
            float* sr = s[1];
            const float invK0 = 1.0f / kStart, dInvK = (1.0f / kS - invK0) * inv, dK = (kS - kStart) * inv;
            float kc = kStart, ic = invK0;
            float g0 = cg[0][0], G0 = cG[0][0], g1 = cg[1][0], G1 = cG[1][0];
            const float dg0 = (tg[0][0] - g0) * inv, dG0 = (tG[0][0] - G0) * inv, dg1 = (tg[1][0] - g1) * inv,
                        dG1 = (tG[1][0] - G1) * inv;
            const float dfb = (fbEnd - fb) * inv;
            for (int i = 0; i < m; ++i)
            {
                fb += dfb;
                g0 += dg0;
                G0 += dG0;
                g1 += dg1;
                G1 += dG1;
                kc += dK;
                ic += dInvK;
                const StageK q0 = stageK<Type>(g0, G0, kc, ic), q1 = stageK<Type>(g1, G1, kc, ic);
                float a = inL[i] + fb * fbPrev[0];
                float b = inR[i] + fb * fbPrev[1];
                for (int k = 0; k < count; ++k)
                {
                    a = allpass<Type>(a, sl[k], q0);
                    b = allpass<Type>(b, sr[k], q1);
                }
                fbPrev[0] = feedbackSample(0, a);
                fbPrev[1] = feedbackSample(1, b);
                wl[i] = a;
                wr[i] = b;
            }
        }
        for (int c = 0; c < 2; ++c)
        {
            cg[c][0] = tg[c][0];
            cG[c][0] = tG[c][0];
        }
    }

    template <int Type> void runBarber(const float* inL, const float* inR, float* wl, float* wr, int m, float fb)
    {
        const float inv = 1.0f / static_cast<float>(m);
        const float invK0 = 1.0f / kStart, dInvK = (1.0f / kS - invK0) * inv, dK = (kS - kStart) * inv;
        float kc = kStart, ic = invK0;
        constexpr int kPairs = kMaxStages / 2;
        float g[2][kPairs], G[2][kPairs], w[2][kPairs], dg[2][kPairs], dG[2][kPairs], dw[2][kPairs];
        const int pairs = active.stages / 2;
        for (int c = 0; c < 2; ++c)
            for (int k = 0; k < pairs; ++k)
            {
                g[c][k] = cg[c][k];
                G[c][k] = cG[c][k];
                w[c][k] = cw[c][k];
                dg[c][k] = (tg[c][k] - g[c][k]) * inv;
                dG[c][k] = (tG[c][k] - G[c][k]) * inv;
                dw[c][k] = (tw[c][k] - w[c][k]) * inv;
            }
        const float dfb = (fbEnd - fb) * inv;
        for (int i = 0; i < m; ++i)
        {
            fb += dfb;
            kc += dK;
            ic += dInvK;
            const float xa = inL[i] + fb * fbPrev[0];
            const float xb = inR[i] + fb * fbPrev[1];
            float sa = xa, sb = xb;
            for (int k = 0; k < pairs; ++k)
            {
                g[0][k] += dg[0][k];
                G[0][k] += dG[0][k];
                w[0][k] += dw[0][k];
                g[1][k] += dg[1][k];
                G[1][k] += dG[1][k];
                w[1][k] += dw[1][k];
                const StageK qa = stageK<Type>(g[0][k], G[0][k], kc, ic), qb = stageK<Type>(g[1][k], G[1][k], kc, ic);
                float ya = allpass<Type>(xa, s[0][2 * k], qa);
                float yb = allpass<Type>(xb, s[1][2 * k], qb);
                ya = allpass<Type>(ya, s[0][2 * k + 1], qa);
                yb = allpass<Type>(yb, s[1][2 * k + 1], qb);
                sa += w[0][k] * (ya - xa);
                sb += w[1][k] * (yb - xb);
            }
            fbPrev[0] = feedbackSample(0, sa);
            fbPrev[1] = feedbackSample(1, sb);
            wl[i] = sa;
            wr[i] = sb;
        }
        for (int c = 0; c < 2; ++c)
            for (int k = 0; k < pairs; ++k)
            {
                cg[c][k] = tg[c][k];
                cG[c][k] = tG[c][k];
                cw[c][k] = tw[c][k];
            }
    }

    void chunk(float* l, float* r, int m, const Block& b, float fade0, float fade1, const float* auxL, const float* auxR)
    {
        const double a = m == kChunk ? alpha : 1.0 - std::exp(-static_cast<double>(m) / (0.02 * sampleRate));
        const double a40 = m == kChunk ? alpha40 : 1.0 - std::exp(-static_cast<double>(m) / (0.04 * sampleRate));
        centerS += (b.center - centerS) * a;
        rangeS += (b.range - rangeS) * a;
        rateS += (b.rate - rateS) * a;
        fbS += (static_cast<double>(b.feedback) - fbS) * a40;
        fbSatS += (static_cast<double>(b.fbSat) - fbSatS) * a;
        spreadS += (static_cast<double>(b.spread) - spreadS) * a;
        const double kTarget = std::pow(10.0, static_cast<double>(b.driveDb) / 20.0);
        kStart = kS;
        kS += static_cast<float>((kTarget - static_cast<double>(kS)) * a);
        fbSatF = static_cast<float>(fbSatS);
        fbStart = fbNow;
        fbEnd = static_cast<float>(fbS);
        fbNow = fbEnd;

        // Envelope: peak of the chunk of the detector signal (the sidechain or the input), attack and release smoothing. It
        // always runs, so that switching the source to it finds it settled.
        double u0 = 0.0;
        {
            const float* sl = active.source == 2 && auxL ? auxL : l;
            const float* sr = active.source == 2 && auxR ? auxR : r;
            float pk = 0.0f;
            for (int i = 0; i < m; ++i)
                pk = std::max(pk, std::max(std::abs(sl[i]), std::abs(sr[i])));
            const double tau = 0.001 * static_cast<double>(pk > env ? b.envAttack : b.envRelease);
            env += (static_cast<double>(pk) - env) *
                   (1.0 - std::exp(-static_cast<double>(m) / (std::max(tau, 1.0e-4) * sampleRate)));
            if (active.source != 0)
            {
                const double e = env * std::pow(10.0, static_cast<double>(b.envSens) / 20.0);
                u0 = 2.0 * e / (e + 0.05) - 1.0;
            }
        }
        if (active.barber == 0)
            phase += rateS * static_cast<double>(m) / sampleRate;
        else
            barberPhase += (active.barber == 1 ? 1.0 : -1.0) * rateS * static_cast<double>(m) / sampleRate;
        phase -= std::floor(phase);
        barberPhase -= std::floor(barberPhase);
        for (int c = 0; c < 2; ++c)
        {
            const double off = c == 0 ? 0.0 : 0.5 * spreadS;
            const double u = active.barber != 0 ? barberPhase : (active.source == 0 ? wave(b.shape, phase + off) : u0);
            coefficients(c, u, false);
        }

        float wl[kChunk], wr[kChunk];
        const float fb = fbStart;
        if (active.barber == 0)
        {
            if (active.type == 0)
                runShared<0>(l, r, wl, wr, m, fb);
            else if (active.type == 1)
                runShared<1>(l, r, wl, wr, m, fb);
            else
                runShared<2>(l, r, wl, wr, m, fb);
        }
        else if (active.type == 0)
            runBarber<0>(l, r, wl, wr, m, fb);
        else if (active.type == 1)
            runBarber<1>(l, r, wl, wr, m, fb);
        else
            runBarber<2>(l, r, wl, wr, m, fb);

        if (active.type == 2) // the JFET stage's asymmetry leaves DC on the wet signal
            for (int i = 0; i < m; ++i)
            {
                outDc[0] += dcCoef * (wl[i] - outDc[0]);
                outDc[1] += dcCoef * (wr[i] - outDc[1]);
                wl[i] -= outDc[0];
                wr[i] -= outDc[1];
            }
        const bool mixMoving = std::abs(b.mix - mixSmooth) > 1.0e-6f || std::abs(b.mix - mixPre) > 1.0e-6f;
        if (!mixMoving)
            mixSmooth = mixPre = b.mix;
        const float fgStep = (fade1 - fade0) / static_cast<float>(m);
        float gF = fade0;
        for (int i = 0; i < m; ++i)
        {
            if (mixMoving)
            {
                mixPre += (b.mix - mixPre) * smoothMix; // two poles: the onset of a mix change has no kink
                mixSmooth += (mixPre - mixSmooth) * smoothMix;
            }
            gF += fgStep;
            const float k = mixSmooth * gF;
            l[i] += (wl[i] - l[i]) * k;
            r[i] += (wr[i] - r[i]) * k;
        }
    }

    const int mix, stages, center, range, rate, shape, feedback, fbSat, spread, type, drive, source, envSens, envAttack,
        envRelease, barber;

    double sampleRate = 48000.0;
    Config active;
    bool needInit = true, switching = false, cleared = true;
    float fade = 1.0f, fadeStep = 0.001f, mixSmooth = 0.0f, mixPre = 0.0f, smoothMix = 0.01f;
    double alpha = 0.02, alpha40 = 0.01, phase = 0.0, barberPhase = 0.0, env = 0.0;
    double centerS = 800.0, rangeS = 1.5, rateS = 0.4, fbS = 0.0, fbSatS = 0.0, spreadS = 0.5;
    float kS = 2.0f, kStart = 2.0f, fbSatF = 0.0f, fbStart = 0.0f, fbEnd = 0.0f, fbNow = 0.0f;
    float s[2][kMaxStages] = {};
    float fbPrev[2] = {}, fbDc[2] = {}, outDc[2] = {}, dcCoef = 0.0016f;
    float cg[2][kMaxStages] = {}, cG[2][kMaxStages] = {}, cw[2][kMaxStages] = {};
    float tg[2][kMaxStages] = {}, tG[2][kMaxStages] = {}, tw[2][kMaxStages] = {};
    std::atomic<double> tail{0.05};
};
} // namespace

ModuleDescriptor phaser2Effect()
{
    auto d =
        effectDescriptor("fx.phaser2", "Phaser 2", "Modulation", CpuClass::Light,
                         "Phaser of 2, 4, 6, 8 or 12 first-order all-pass stages (zero-delay-feedback form) swept by an "
                         "oscillator, the envelope of the input or the envelope of the sidechain input. The stages can be "
                         "linear or have an OTA-style or JFET-style saturating integrator, the feedback is positive or "
                         "negative with a saturation, and the barber-pole mode makes the notches travel up or down for "
                         "ever. Linear, no latency.");
    auto& p = d.params;
    addOn(p, "phaser2", "Phaser 2", "fx_phaser2", 0.5f);
    p.push_back(params::choice("fx_phaser2_stages", "Phaser 2 Stages", "fx_phaser2", {"2", "4", "6", "8", "12"}, 1));
    addHz(p, "phaser2_center", "Phaser 2 Center", "fx_phaser2", 100.0f, 6000.0f, 800.0f);
    addF(p, "phaser2_range", "Phaser 2 Range", "fx_phaser2", 0.0f, 3.0f, 1.5f, "oct");
    addF(p, "phaser2_rate", "Phaser 2 Rate", "fx_phaser2_2", 0.02f, 10.0f, 0.4f, "Hz", 0.4f);
    p.push_back(params::choice("fx_phaser2_shape", "Phaser 2 Shape", "fx_phaser2_2", {"Sine", "Triangle"}, 0));
    addF(p, "phaser2_feedback", "Phaser 2 Feedback", "fx_phaser2_2", -0.95f, 0.95f, 0.3f);
    addF(p, "phaser2_fb_sat", "Phaser 2 Feedback Saturation", "fx_phaser2_2", 0.0f, 1.0f, 0.0f);
    addF(p, "phaser2_spread", "Phaser 2 Spread", "fx_phaser2_2", 0.0f, 1.0f, 0.5f);
    p.push_back(params::choice("fx_phaser2_type", "Phaser 2 Stage Type", "fx_phaser2_2", {"Linear", "OTA", "JFET"}, 0));
    addF(p, "phaser2_drive", "Phaser 2 Drive", "fx_phaser2_3", 0.0f, 24.0f, 6.0f, "dB");
    p.push_back(params::choice("fx_phaser2_source", "Phaser 2 Source", "fx_phaser2_3", {"LFO", "Envelope", "Sidechain"}, 0));
    addF(p, "phaser2_env_sens", "Phaser 2 Envelope Sensitivity", "fx_phaser2_3", -40.0f, 40.0f, 0.0f, "dB");
    addF(p, "phaser2_env_attack", "Phaser 2 Envelope Attack", "fx_phaser2_3", 0.5f, 200.0f, 5.0f, "ms", 0.4f);
    addF(p, "phaser2_env_release", "Phaser 2 Envelope Release", "fx_phaser2_3", 5.0f, 2000.0f, 120.0f, "ms", 0.4f);
    p.push_back(params::choice("fx_phaser2_barber", "Phaser 2 Barber-pole", "fx_phaser2_3", {"Off", "Up", "Down"}, 0));
    p.push_back(params::integer("fx_pos_phaser2", "Phaser 2 Slot", "fx_phaser2", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_phaser2_on";
    d.positionParam = "fx_pos_phaser2";
    d.capabilities = {"latency"};
    d.pages.push_back(
        {"fx_phaser2", "Phaser 2", "fx", "Phaser 2", "On, mix, stage count, sweep center and range, and the chain slot."});
    d.pages.push_back(
        {"fx_phaser2_2", "Phaser 2 2", "fx", "Phaser 2 Sweep", "Rate, shape, feedback, saturation, spread and stage type."});
    d.pages.push_back({"fx_phaser2_3", "Phaser 2 3", "fx", "Phaser 2 Modulation",
                       "Stage drive, sweep source, envelope follower and the barber-pole mode."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.phaser2")); };
    return d;
}

} // namespace crucible::modules::fx
