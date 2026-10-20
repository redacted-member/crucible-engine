// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "ModalBankAvx.h"
#include "OscSupport.h"

#if CRUCIBLE_MODAL_AVX
#include <intrin.h>
#endif

#include <crucible/Dsp.h>
#include <crucible/Simd.h>

// stmlib selects its host code paths (instead of ARM Cortex-M inline assembly) when TEST is defined.
#define TEST 1
#include <rings/resources.h>
#include <stmlib/dsp/cosine_oscillator.h>
#include <stmlib/dsp/dsp.h>
#undef TEST

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

// Modal / physical oscillator (P13). Resonators, strings and exciters follow the designs of the Mutable Instruments
// Rings and Elements modules (Emilie Gillet, MIT licence; the stiffness table, the cosine pickup and the exciter models
// come from there, see third_party/mutable/ and THIRD_PARTY.md). The ports differ where the originals are tied to a
// fixed 32 or 48 kHz clock:
//  * every mode is a zero-delay-feedback state-variable filter whose pole is placed exactly: its coefficients come from
//    the z-plane pole (radius from the T60, angle from the frequency) through the bilinear map, so the mode frequency
//    and the decay time are exact at every sample rate instead of following the approximations of the lookup tables;
//  * the string computes the group of delays of its loop (damping filters, dispersion all-pass, Hermite read) at the
//    fundamental and the loop gain that gives the requested T60, again at any sample rate;
//  * exciter times are in seconds, the generator is the engine's own (seeded per note), nothing is global;
//  * no mode is placed above kBandLimit x fs, and the output is a function of the note seed only.
// "Tolerance" draws a card per note (pitch, mode frequencies, decay, level) from the note seed, like the voice card of
// the analog oscillator; at 0 the oscillator is exact.

namespace crucible::modules
{
namespace
{
constexpr int kMaxModes = 64;
constexpr int kBowModes = 8;
constexpr double kPiD = 3.141592653589793;
constexpr double kLn1000 = 6.907755278982137; // amplitude falls by 60 dB over the T60
// Highest mode frequency in cycles per sample. Modes above it are dropped, so with a decaying (band-limited) excitation
// nothing sits near Nyquist; the figure is checked by tests/EnginesTests.cpp.
constexpr double kBandLimit = 0.40;
constexpr float kMinFundamental = 20.0f;

#if CRUCIBLE_MODAL_AVX
// AVX in the CPU and its register state saved by the OS (CPUID.1:ECX bits 27 and 28, XCR0 bits 1 and 2). Evaluated once,
// when the library is loaded.
bool cpuHasAvx()
{
    int regs[4];
    __cpuid(regs, 1);
    const bool osSaves = (regs[2] & (1 << 27)) != 0, avx = (regs[2] & (1 << 28)) != 0;
    return osSaves && avx && (_xgetbv(0) & 6U) == 6U;
}
const bool kBankAvx = cpuHasAvx();
#endif

enum Model
{
    modelModal,
    modelString
};
enum ExciterKind
{
    excStrike,
    excPluck,
    excParticles,
    excBlow,
    excNoise,
    excBow
};

float gauss(Rng& rng)
{
    float s = 0.0f;
    for (int i = 0; i < 12; ++i)
        s += rng.unipolar();
    return s - 6.0f;
}

struct ModeDesign
{
    float g, rg, h, scale;
};

// Coefficients of the zero-delay-feedback state-variable filter (g, r, h as in the TPT structure) whose poles are
// rho e^(+-j theta): theta = 2 pi f, rho from the T60 in samples. scale makes the low-pass impulse response peak at 1.
ModeDesign designMode(double cyclesPerSample, double t60Samples)
{
    const double theta = 2.0 * kPiD * cyclesPerSample;
    const double rho = std::exp(-kLn1000 / std::max(t60Samples, 2.0));
    // s = (z - 1) / (z + 1) for z = rho e^(i theta), written out: |z + 1|^2 = d.
    const double c = std::cos(theta), sn = std::sin(theta);
    const double d = 1.0 + 2.0 * rho * c + rho * rho;
    const double x = (rho * rho - 1.0) / d, y = 2.0 * rho * sn / d;
    const double g = std::sqrt(x * x + y * y);
    const double r = -2.0 * x / g;
    const double h = 1.0 / (1.0 + r * g + g * g);
    // Low-pass impulse response: g^2 h (1 + z^-1)^2 over the pole pair; amplitude |B(1/p)| / sin(theta), with
    // |1 + 1/z|^2 = d / rho^2.
    const double amp = g * g * h * d / (rho * rho) / std::max(sn, 1.0e-6);
    return {static_cast<float>(g), static_cast<float>(r + g), static_cast<float>(h),
            static_cast<float>(1.0 / std::max(amp, 1.0e-9))};
}

struct Svf
{
    float g = 0.0f, r = 1.0f, h = 1.0f;
    float s1 = 0.0f, s2 = 0.0f;
    void setFq(double cyclesPerSample, double q)
    {
        g = static_cast<float>(std::tan(kPiD * std::min(cyclesPerSample, 0.49)));
        r = static_cast<float>(1.0 / q);
        h = 1.0f / (1.0f + r * g + g * g);
    }
    float lowPass(float in)
    {
        const float hp = (in - r * s1 - g * s1 - s2) * h;
        const float bp = g * hp + s1;
        s1 = g * hp + bp;
        const float lp = g * bp + s2;
        s2 = g * bp + lp;
        return lp;
    }
    float bandPassNormalised(float in)
    {
        const float hp = (in - r * s1 - g * s1 - s2) * h;
        const float bp = g * hp + s1;
        s1 = g * hp + bp;
        const float lp = g * bp + s2;
        s2 = g * bp + lp;
        return bp * r;
    }
    void reset() { s1 = s2 = 0.0f; }
};

// ---- Exciters (after the Elements exciter models: mallet, plectrum, particles, flow, noise) --------------------------
class Exciter
{
public:
    void prepare(double rate) { sr = rate; }
    void start(int kindIn, std::uint32_t seed, float velocity)
    {
        kind = kindIn;
        rng.state = (seed * 2654435761U + 0x7f4a7c15U) | 1U;
        lp.reset();
        lp2.reset();
        flowState = 0.5f;
        particleState = 0.5f + 0.5f * rng.unipolar();
        particleRange = 1.0f;
        particleDelay = 0;
        pluckDelay = 0;
        fresh = true;
        strength = 0.2f + 0.8f * velocity;
        signature = rng.unipolar();
    }
    // Writes n samples; gate is the key state, level the sustain level of the continuous exciters.
    void process(float* out, int n, bool gate, float parameter, float timbre, float level)
    {
        std::fill(out, out + n, 0.0f);
        // Filter: a critically damped two-pole low-pass (noise: resonance from the parameter), cutoff from the timbre.
        const double cutoff = 0.4 * sr * std::pow(2.0, 8.0 * (static_cast<double>(timbre) - 1.0)) + 40.0;
        const double cps = std::min(cutoff / sr, 0.33);
        if (kind == excNoise)
            lp.setFq(cps, 0.5 + static_cast<double>(parameter) * 12.0);
        else
            lp.setFq(cps, 0.5);
        lp2.setFq(cps, 0.5);
        const float amp = strength;
        switch (kind)
        {
        case excStrike:
            if (fresh)
                out[0] = amp * 2.0f;
            break;
        case excPluck:
        {
            if (fresh)
            {
                out[0] = -amp * (0.05f + signature * 0.2f);
                pluckDelay = static_cast<int>((4096.0 * parameter * parameter + 64.0) * sr / 32000.0);
            }
            for (int i = 0; i < n; ++i)
                if (pluckDelay > 0 && --pluckDelay == 0)
                    out[i] += amp * 2.0f;
            break;
        }
        case excParticles:
            if (fresh)
            {
                particleState = 1.0f - 0.6f * particleState * particleState;
                particleDelay = 0;
                particleRange = 1.0f;
            }
            if (gate)
                for (int i = 0; i < n; ++i)
                {
                    if (particleDelay > 0)
                    {
                        --particleDelay;
                        continue;
                    }
                    const float r0 = rng.unipolar();
                    const float amount = 1.05f + 0.5f * r0 * r0;
                    if (rng.unipolar() > 0.7f)
                    {
                        particleState = std::min(particleState * amount, particleRange + 0.25f);
                    }
                    else if (rng.unipolar() < 0.3f)
                    {
                        particleState = std::max(particleState / amount, 0.02f);
                    }
                    particleDelay = static_cast<int>(particleState * 0.15 * sr);
                    float gain = 1.0f - particleRange;
                    gain *= gain;
                    out[i] = particleState * amp * 2.0f * (1.0f - gain) * level;
                    const float decay = 1.0f - parameter;
                    particleRange *= 1.0f - decay * decay * 0.5f;
                }
            break;
        case excBlow:
        {
            const float scale = parameter * parameter * parameter * parameter;
            const float threshold = 0.0001f + scale * 0.125f;
            for (int i = 0; i < n; ++i)
            {
                const float s = rng.unipolar();
                if (s < threshold)
                    flowState = -flowState;
                out[i] = (flowState + (s - 0.5f - flowState) * scale) * level * 0.4f;
            }
            break;
        }
        case excNoise:
        case excBow:
        {
            // Local copies (written back below) keep the generator and the filter states in registers: MSVC does not
            // use type-based aliasing and would store and reload them around every write to out.
            Rng g = rng;
            for (int i = 0; i < n; ++i)
                out[i] = (g.unipolar() - 0.5f) * level * (kind == excBow ? 0.25f : 0.5f);
            rng = g;
            break;
        }
        default:
            break;
        }
        fresh = false;
        // Two stages: the band limit of every exciter (the double zero of each stage sits at Nyquist).
        Svf a = lp, b = lp2;
        for (int i = 0; i < n; ++i)
            out[i] = b.lowPass(a.lowPass(out[i]));
        lp = a;
        lp2 = b;
    }

private:
    int kind = excStrike;
    double sr = 48000.0;          // audio thread: the rate the current design was made for
    double prepareRate = 48000.0; // control thread: set by prepare(), read by arm(); the audio thread never writes it
    Rng rng;
    Svf lp, lp2;
    float flowState = 0.5f, particleState = 0.5f, particleRange = 1.0f, strength = 1.0f, signature = 0.0f;
    int particleDelay = 0, pluckDelay = 0;
    bool fresh = true;
};

// ---- The modal resonator ---------------------------------------------------------------------------------------------
struct ModeSet
{
    int count = 0;
    std::array<double, kMaxModes> freq{}; // cycles per sample
    std::array<double, kMaxModes> t60{};  // seconds
};

// Mode frequencies and decay times from the Rings / Elements laws: stiffness (geometry) stretches the partials, the
// brightness sets how much faster each partial decays than the one below, and the geometry flattens that fall.
void designModes(double f0, float geometry, float brightness, double decaySeconds, int resolution, ModeSet& out)
{
    // The table has 257 entries and the interpolation reads one past the index: stay below 1.
    geometry = std::clamp(geometry, 0.0f, 0.9999f);
    float stiffness = stmlib::Interpolate(rings::lut_stiffness, geometry, 256.0f);
    double stretch = 1.0;
    double harmonic = f0;
    double t60 = decaySeconds;
    const float bright = std::clamp(brightness, 0.0f, 1.0f);
    const float g = geometry;
    float brightnessAttenuation = 1.0f - g;
    brightnessAttenuation *= brightnessAttenuation;
    brightnessAttenuation *= brightnessAttenuation;
    brightnessAttenuation *= brightnessAttenuation;
    const float b = bright * (1.0f - 0.2f * brightnessAttenuation);
    float qLoss = b * (2.0f - b) * 0.85f + 0.15f;
    const float lossRate = g * (2.0f - g) * 0.1f;
    out.count = 0;
    for (int i = 0; i < std::min(kMaxModes, resolution); ++i)
    {
        const double f = harmonic * stretch;
        if (f >= kBandLimit || f <= 0.0)
            break;
        out.freq[static_cast<size_t>(i)] = f;
        out.t60[static_cast<size_t>(i)] = t60;
        out.count = i + 1;
        stretch += stiffness;
        stiffness *= stiffness < 0.0f ? 0.93f : 0.98f;
        qLoss += lossRate * (1.0f - qLoss);
        harmonic += f0;
        t60 *= qLoss;
    }
}

class Bank
{
public:
    void reset()
    {
        s1.fill(0.0f);
        s2.fill(0.0f);
        used = 0;
        padded = 0;
    }

    // Applies a design: coefficients per mode, rounded up to a multiple of four with silent filler modes.
    void apply(const ModeSet& m, double rate, const std::array<float, kMaxModes>& freqCard,
               const std::array<float, kMaxModes>& decayCard)
    {
        used = m.count;
        padded = (used + 3) & ~3;
        for (int i = 0; i < padded; ++i)
        {
            const auto k = static_cast<size_t>(i);
            if (i < used)
            {
                const double f = m.freq[k] * (1.0 + static_cast<double>(freqCard[k]));
                const double t60 = m.t60[k] * (1.0 + static_cast<double>(decayCard[k])) * rate;
                const auto d = designMode(std::min(f, kBandLimit), t60);
                g[k] = d.g;
                rg[k] = d.rg;
                h[k] = d.h;
                scale[k] = d.scale;
            }
            else
            {
                g[k] = 0.0f;
                rg[k] = 1.0f;
                h[k] = 1.0f;
                scale[k] = 0.0f;
                s1[k] = s2[k] = 0.0f;
            }
        }
    }

    // Weights for the two pickups for pickup position p and the side pickup offset q (cosine comb, as in Rings).
    void pickups(float p, float q, std::array<float, kMaxModes>& wc, std::array<float, kMaxModes>& ws) const
    {
        stmlib::CosineOscillator a, b;
        a.Init<stmlib::COSINE_OSCILLATOR_APPROXIMATE>(std::clamp(p, 0.0f, 1.0f));
        b.Init<stmlib::COSINE_OSCILLATOR_APPROXIMATE>(std::clamp(q, 0.0f, 1.0f));
        a.Start();
        b.Start();
        for (int i = 0; i < kMaxModes; ++i)
        {
            const auto k = static_cast<size_t>(i);
            const float wa = a.Next(), wb = b.Next();
            wc[k] = i < used ? wa * scale[k] : 0.0f;
            ws[k] = i < used ? wb * scale[k] : 0.0f;
        }
    }

    // Runs n samples of input x through all modes; the two pickups (weights ramp from *0 to *1) add into c and s.
    void process(const float* x, int n, const float* wc0, const float* wc1, const float* ws0, const float* ws1, float* c,
                 float* s)
    {
#if CRUCIBLE_MODAL_AVX
        // The same arithmetic with eight-lane filters (ModalBankAvx.h): bit-identical, about a third faster.
        if (kBankAvx)
        {
            modal::bankProcessAvx(x, n, padded, g.data(), rg.data(), h.data(), s1.data(), s2.data(), wc0, wc1, ws0, ws1, c,
                                  s);
            return;
        }
#endif
        using simd::Float4;
        const float inv = 1.0f / static_cast<float>(n);
        for (int t = 0; t < n; ++t)
        {
            const Float4 in = Float4::splat(x[t]);
            Float4 c0 = Float4::splat(0.0f), c1 = c0, d0 = c0, d1 = c0;
            for (int i = 0; i < padded; i += 4)
            {
                const Float4 gv = Float4::load(&g[static_cast<size_t>(i)]);
                const Float4 st1 = Float4::load(&s1[static_cast<size_t>(i)]);
                const Float4 st2 = Float4::load(&s2[static_cast<size_t>(i)]);
                const Float4 hp =
                    (in - Float4::load(&rg[static_cast<size_t>(i)]) * st1 - st2) * Float4::load(&h[static_cast<size_t>(i)]);
                const Float4 ghp = gv * hp;
                const Float4 bp = ghp + st1;
                (ghp + bp).store(&s1[static_cast<size_t>(i)]);
                const Float4 gbp = gv * bp;
                const Float4 lp = gbp + st2;
                (gbp + lp).store(&s2[static_cast<size_t>(i)]);
                c0 = Float4::mulAdd(c0, Float4::load(wc0 + i), lp);
                c1 = Float4::mulAdd(c1, Float4::load(wc1 + i), lp);
                d0 = Float4::mulAdd(d0, Float4::load(ws0 + i), lp);
                d1 = Float4::mulAdd(d1, Float4::load(ws1 + i), lp);
            }
            const float f = (static_cast<float>(t) + 1.0f) * inv;
            const float a0 = c0.sum(), a1 = c1.sum(), b0 = d0.sum(), b1 = d1.sum();
            c[t] += a0 + (a1 - a0) * f;
            s[t] += b0 + (b1 - b0) * f;
        }
    }

    int modeCount() const { return used; }
    float modeG(int i) const { return g[static_cast<size_t>(i)]; }

private:
    alignas(16) std::array<float, kMaxModes> g{}, rg{}, h{}, s1{}, s2{}, scale{};
    int used = 0, padded = 0;
};

// Banded waveguide bow (after Elements): the first modes get a delay loop each, the friction curve couples them. The eight
// band-pass filters run as two four-lane vectors.
class BowedModes
{
public:
    static constexpr size_t kLine = 1024; // a power of two above the longest delay (the period is halved below 1024)

    void prepare(double rate)
    {
        sr = rate;
        lines.assign(kLine * kBowModes, 0.0f);
        reset();
    }
    void reset()
    {
        std::fill(lines.begin(), lines.end(), 0.0f);
        pos = 0;
        s1.fill(0.0f);
        s2.fill(0.0f);
        bowSignal = 0.0f;
    }
    void configure(const ModeSet& m)
    {
        count = std::min(kBowModes, m.count);
        for (int i = 0; i < kBowModes; ++i)
        {
            const auto k = static_cast<size_t>(i);
            if (i < count)
            {
                int period = static_cast<int>(1.0 / m.freq[k]);
                while (period >= 1024)
                    period >>= 1;
                delay[k] = static_cast<size_t>(std::max(period, 2));
                Svf f;
                f.setFq(m.freq[k], 1.0 + m.freq[k] * sr * 1500.0 / 32000.0);
                g[k] = f.g;
                rg[k] = f.r + f.g;
                h[k] = f.h;
                r[k] = f.r;
            }
            else
            {
                g[k] = 0.0f;
                rg[k] = 1.0f;
                h[k] = 1.0f;
                r[k] = 0.0f;
                delay[k] = 2;
            }
        }
    }
    CRUCIBLE_ALWAYS_INLINE static float bowTable(float x, float velocity)
    {
        x = 0.13f * velocity - x;
        float bow = x * 6.0f;
        bow = std::abs(bow) + 0.75f;
        bow *= bow;
        bow *= bow;
        bow = 0.25f / bow;
        // std::clamp(bow, 0.0025f, 0.245f), written out: MSVC compiles the clamp through references in memory.
        bow = bow < 0.0025f ? 0.0025f : bow;
        bow = 0.245f < bow ? 0.245f : bow;
        return x * bow;
    }
    // One block of the bowed modes: out[i] += gain * (bowed output for the input x[i] * inScale). Per sample: read the eight
    // delays, filter the eight modes (input plus the friction signal of the previous sample), write the delays, sum with the
    // weights. The filter states, the write position and the friction signal stay in registers over the block (MSVC ran the
    // per-sample version as an out-of-line call that stored and reloaded all of them).
    void addBlock(const float* x, float inScale, float* out, float gain, float velocity, const float* weights, int n)
    {
        using simd::Float4;
        float* base = lines.data();
        const Float4 loss = Float4::splat(0.99f);
        const Float4 gLo = Float4::load(&g[0]), gHi = Float4::load(&g[4]), rgLo = Float4::load(&rg[0]),
                     rgHi = Float4::load(&rg[4]);
        const Float4 hLo = Float4::load(&h[0]), hHi = Float4::load(&h[4]), rLo = Float4::load(&r[0]),
                     rHi = Float4::load(&r[4]);
        Float4 s1Lo = Float4::load(&s1[0]), s1Hi = Float4::load(&s1[4]), s2Lo = Float4::load(&s2[0]),
               s2Hi = Float4::load(&s2[4]);
        size_t at = pos;
        float signal = bowSignal;
        for (int i = 0; i < n; ++i)
        {
            const float input = x[i] * inScale;
            alignas(16) float delayed[kBowModes];
            for (size_t k = 0; k < kBowModes; ++k)
                delayed[k] = base[((at + delay[k]) & (kLine - 1)) * kBowModes + k];
            // 0.99 x per mode, then summed in mode order (the same operations as one mode at a time).
            const Float4 dLo = loss * Float4::load(delayed), dHi = loss * Float4::load(delayed + 4);
            dLo.store(delayed);
            dHi.store(delayed + 4);
            float bow = 0.0f;
            for (size_t k = 0; k < kBowModes; ++k)
                bow += delayed[k];
            const Float4 in = Float4::splat(input + signal);
            at = (at + kLine - 1) & (kLine - 1);
            float* row = base + at * kBowModes;
            alignas(16) float y[kBowModes];
            {
                const Float4 xv = in + dLo;
                const Float4 hp = (xv - rgLo * s1Lo - s2Lo) * hLo;
                const Float4 ghp = gLo * hp;
                const Float4 bp = ghp + s1Lo;
                s1Lo = ghp + bp;
                const Float4 gbp = gLo * bp;
                s2Lo = gbp + gbp + s2Lo;
                // lp = gbp + s2 is not needed: the normalised band-pass output is bp * r.
                const Float4 o = bp * rLo;
                o.store(row);
                o.store(y);
            }
            {
                const Float4 xv = in + dHi;
                const Float4 hp = (xv - rgHi * s1Hi - s2Hi) * hHi;
                const Float4 ghp = gHi * hp;
                const Float4 bp = ghp + s1Hi;
                s1Hi = ghp + bp;
                const Float4 gbp = gHi * bp;
                s2Hi = gbp + gbp + s2Hi;
                const Float4 o = bp * rHi;
                o.store(row + 4);
                o.store(y + 4);
            }
            float sum = 0.0f;
            for (size_t k = 0; k < kBowModes; ++k)
                sum += y[k] * weights[k];
            signal = bowTable(bow, velocity);
            const float result = sum * 8.0f;
            out[i] += gain * result;
        }
        s1Lo.store(&s1[0]);
        s1Hi.store(&s1[4]);
        s2Lo.store(&s2[0]);
        s2Hi.store(&s2[4]);
        pos = at;
        bowSignal = signal;
    }
    int modes() const { return count; }

private:
    double sr = 48000.0;
    // The eight delay lines interleaved: lines[t * kBowModes + k] is mode k at ring position t. All lines share the write
    // position (they always moved together).
    std::vector<float> lines;
    size_t pos = 0;
    std::array<size_t, kBowModes> delay{};
    alignas(16) std::array<float, kBowModes> g{}, rg{}, h{}, r{}, s1{}, s2{};
    float bowSignal = 0.0f;
    int count = 0;
};

// ---- The string (after the Rings / Elements string: comb with Hermite read, dispersion all-pass, damping filter) -----
// One pass around the loop is: Hermite read, dispersion all-pass, the exciter input, a two-pole low-pass. The read delay
// is the period minus the exact phase delays of the low-pass and the all-pass at the fundamental, and the loop gain is set
// from the gain of the low-pass at the fundamental, so pitch and T60 are exact (the cubic read is corrected the same way).
class StringModel
{
public:
    void prepare(double rate)
    {
        sr = rate;
        int size = 64;
        while (size < static_cast<int>(rate / kMinFundamental) + 16)
            size <<= 1;
        line.assign(static_cast<size_t>(size), 0.0f);
        stretchLine.assign(static_cast<size_t>(size / 2), 0.0f);
        reset();
    }
    void reset()
    {
        std::fill(line.begin(), line.end(), 0.0f);
        std::fill(stretchLine.begin(), stretchLine.end(), 0.0f);
        wp = swp = 0;
        lp.reset();
        noiseState = 0.0f;
        dcX = dcY = 0.0f;
    }

    struct Control
    {
        double f0 = 0.0; // cycles per sample
        float dispersion = 0.0f;
        float brightness = 0.5f;
        float position = 0.5f;
        double t60 = 1.0; // seconds
    };

    void set(const Control& c, Rng* r)
    {
        rng = r;
        const double period = 1.0 / std::max(c.f0, 1.0e-6);
        const double w = 2.0 * kPiD * c.f0;
        const double b = std::clamp(static_cast<double>(c.brightness), 0.0, 1.0);
        // Loss filter: two-pole low-pass (critically damped) at 32 to 400 times the fundamental.
        const double fc = std::min(c.f0 * 32.0 * std::exp2(b * 3.64), 0.45);
        lp.setFq(fc, 0.5);
        const double g = static_cast<double>(lp.g), rr = static_cast<double>(lp.r);
        const double t = std::tan(w * 0.5);
        const std::complex<double> hLp = g * g / std::complex<double>(g * g - t * t, rr * g * t);
        // A DC blocker in the loop (corner at 2 % of the fundamental) keeps the offset of impulsive or one-signed excitation
        // from circulating forever; its gain and phase at the fundamental join the low-pass in the design.
        const double blockerR = std::exp(-2.0 * kPiD * std::max(0.02 * c.f0, 1.0 / sr));
        const std::complex<double> zi = std::polar(1.0, -w);
        const std::complex<double> hBlock = (1.0 - zi) / (1.0 - blockerR * zi);
        dcR = static_cast<float>(blockerR);
        const std::complex<double> hLoop = hLp * hBlock;
        const double lpGain = std::abs(hLoop);
        const double lpDelay = -std::arg(hLoop) / w;
        const float d = std::clamp(c.dispersion, 0.0f, 1.0f);
        apDelayInt = 0;
        apGain = 0.0f;
        double apPhaseDelay = 0.0;
        if (d > 0.0f)
        {
            const float stretchPoint = d * (2.0f - d) * 0.475f;
            const int apDelay = static_cast<int>(period * static_cast<double>(stretchPoint));
            if (apDelay >= 4 && static_cast<size_t>(apDelay) + 4 < stretchLine.size() && period - apDelay >= 8.0)
            {
                apDelayInt = apDelay;
                apGain = -0.618f * d / (0.15f + std::abs(d));
                const double th = apDelayInt * w;
                const double gg = static_cast<double>(apGain);
                apPhaseDelay = apDelayInt + 2.0 * std::atan2(gg * std::sin(th), 1.0 - gg * std::cos(th)) / w;
            }
        }
        noiseAmount = d > 0.75f ? 4.0f * (d - 0.75f) : 0.0f;
        noiseAmount = noiseAmount * noiseAmount * 0.025f;
        noiseFilter = static_cast<float>(std::exp2((b - 1.0) * 4.0));
        const double wantedDelay = std::max(4.0, period - lpDelay - apPhaseDelay);
        mainDelay = wantedDelay + hermiteBias(wantedDelay, w);
        const double perPass = std::pow(10.0, -3.0 * period / (std::max(c.t60, 0.001) * sr));
        damping = static_cast<float>(std::min(perPass / std::max(lpGain, 1.0e-6), 1.0));
        combFraction = 0.5 - 0.98 * std::abs(static_cast<double>(std::clamp(c.position, 0.0f, 1.0f)) - 0.5);
    }

    // Adds n samples into out (bridge) and aux (comb pickup).
    void process(const float* in, float* out, float* aux, int n)
    {
        const size_t mask = line.size() - 1;
        const size_t smask = stretchLine.size() - 1;
        for (int i = 0; i < n; ++i)
        {
            float delay = static_cast<float>(mainDelay);
            if (noiseAmount > 0.0f)
            {
                const float noise = (2.0f * rng->unipolar() - 1.0f) / (0.2f + noiseFilter);
                noiseState += noiseFilter * (noise - noiseState);
                delay *= 1.0f + noiseState * noiseAmount;
            }
            float s = readHermite(delay);
            if (apDelayInt > 0)
            {
                const float tap = stretchLine[(swp + static_cast<size_t>(apDelayInt)) & smask];
                const float write = s + apGain * tap;
                stretchLine[swp] = write;
                swp = (swp + smask) & smask;
                s = -write * apGain + tap;
            }
            s += in[i];
            const float filtered = lp.lowPass(s) * damping;
            float y = filtered - dcX + dcR * dcY;
            dcX = filtered;
            if (!(std::abs(y) < 1.0e6f))
                y = 0.0f;
            dcY = y;
            line[wp] = y;
            wp = (wp + mask) & mask;
            out[i] += y;
            aux[i] += readLinear(static_cast<float>(mainDelay * combFraction));
        }
    }

private:
    float readHermite(float delay) const
    {
        const size_t mask = line.size() - 1;
        const int di = static_cast<int>(delay);
        const float f = delay - static_cast<float>(di);
        const size_t t = wp + static_cast<size_t>(di) + line.size();
        const float xm1 = line[(t - 1) & mask];
        const float x0 = line[t & mask];
        const float x1v = line[(t + 1) & mask];
        const float x2v = line[(t + 2) & mask];
        const float c = (x1v - xm1) * 0.5f;
        const float v = x0 - x1v;
        const float w = c + v;
        const float a = w + v + (x2v - x0) * 0.5f;
        const float bNeg = w + a;
        return (((a * f) - bNeg) * f + c) * f + x0;
    }
    float readLinear(float delay) const
    {
        const size_t mask = line.size() - 1;
        const int di = static_cast<int>(delay);
        const float f = delay - static_cast<float>(di);
        const float a = line[(wp + static_cast<size_t>(di)) & mask];
        const float b = line[(wp + static_cast<size_t>(di) + 1) & mask];
        return a + (b - a) * f;
    }
    // Phase delay of the cubic Hermite read of a fractional delay at w, relative to the requested delay (so that adding
    // the returned value to the request makes the actual delay the request).
    static double hermiteBias(double delay, double w)
    {
        const int di = static_cast<int>(delay);
        const double f = delay - di;
        const double wts[4] = {-0.5 * f * (1.0 - f) * (1.0 - f), 1.0 - 2.5 * f * f + 1.5 * f * f * f,
                               0.5 * f * (1.0 + 4.0 * f - 3.0 * f * f), -0.5 * f * f * (1.0 - f)};
        std::complex<double> h = 0.0;
        for (int k = 0; k < 4; ++k)
            h += wts[k] * std::polar(1.0, -w * (k - 1));
        const double actual = di - std::arg(h) / w;
        return delay - actual;
    }

    double sr = 48000.0;
    std::vector<float> line, stretchLine;
    size_t wp = 0, swp = 0;
    float damping = 1.0f, dcX = 0.0f, dcY = 0.0f, dcR = 0.999f;
    Svf lp;
    Rng* rng = nullptr;
    int apDelayInt = 0;
    float apGain = 0.0f, noiseAmount = 0.0f, noiseFilter = 1.0f, noiseState = 0.0f;
    double mainDelay = 100.0, combFraction = 0.5;
};

// What only the bowed resonator and the string need (about 50 KB): built by arm(), because every voice owns an instance of
// every oscillator type.
struct Heavy
{
    BowedModes bowed;
    StringModel string;
};

class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        modelP = sp.bind("modal_model");
        exciterP = sp.bind("modal_exciter");
        geometry = sp.bind("modal_geometry");
        brightness = sp.bind("modal_brightness");
        position = sp.bind("modal_position");
        decay = sp.bind("modal_decay");
        contour = sp.bind("modal_contour");
        timbre = sp.bind("modal_timbre");
        sustain = sp.bind("modal_sustain");
        spread = sp.bind("modal_spread");
        tolerance = sp.bind("modal_tol");
        modesP = sp.bind("modal_modes");
    }

    ~Instance() override { delete heavy.load(); }

    void prepare(double rate, int) override
    {
        sr = rate;
        prepareRate = rate;
        exciter.prepare(rate);
        bank.reset();
        delete heavy.exchange(nullptr); // re-armed by the engine (audio is stopped while it prepares)
        idle = true;
    }

    void arm() override
    {
        if (heavy.load(std::memory_order_acquire) != nullptr || prepareRate <= 0.0)
            return;
        auto* h = new Heavy;
        h->bowed.prepare(prepareRate);
        h->string.prepare(prepareRate);
        heavy.store(h, std::memory_order_release);
    }

    void noteOn(const OscNoteInfo& info) override
    {
        rng.state = (info.seed * 2246822519U + 0x5bd1e995U) | 1U;
        Rng cardRng;
        cardRng.state = (info.seed * 3266489917U + 0x9e3779b9U) | 1U;
        cardPitch = gauss(cardRng);
        cardLevel = gauss(cardRng);
        for (int i = 0; i < kMaxModes; ++i)
        {
            freqCard[static_cast<size_t>(i)] = gauss(cardRng);
            decayCard[static_cast<size_t>(i)] = gauss(cardRng);
        }
        bank.reset();
        if (Heavy* h = heavy.load(std::memory_order_acquire))
        {
            h->bowed.reset();
            h->string.reset();
        }
        gate = true;
        velocity = info.velocity;
        fresh = true;
        idle = false;
        designed = false;
        stringDesigned = false;
        quietChunks = 0;
        envelope = 0.0f;
        lfoPhase = 0.0f;
        seed = info.seed;
        weightsValid = false;
    }

    void noteOff() override { gate = false; }

    void render(const OscRender& whole) override
    {
        // The voice renders chunks of at most 32 samples; longer calls (a graph node, a test) are cut into such chunks.
        for (int done = 0; done < whole.numSamples; done += 32)
        {
            OscRender part = whole;
            part.numSamples = std::min(32, whole.numSamples - done);
            part.left = whole.left + done;
            part.right = whole.right + done;
            part.noteTime = whole.noteTime + static_cast<double>(done) / whole.sampleRate;
            renderChunk(part);
        }
    }

private:
    void renderChunk(const OscRender& r)
    {
        Heavy* heavyState = heavy.load(std::memory_order_acquire);
        if (idle || r.numSamples <= 0 || heavyState == nullptr)
            return;
        BowedModes& bowed = heavyState->bowed;
        StringModel& string = heavyState->string;
        const int n = r.numSamples;
        const auto v = [&](int p) { return sp.value(r.values, r.targetMod, p); };
        const int mdl = std::clamp(static_cast<int>(v(modelP)), 0, 1);
        const int exc = std::clamp(static_cast<int>(v(exciterP)), 0, 5);
        const float geom = v(geometry), bright = v(brightness), pos = v(position);
        const float t60 = v(decay), param = v(contour), tim = v(timbre), sus = v(sustain), width = v(spread);
        const float tol = v(tolerance);
        const int resolution = std::clamp(static_cast<int>(v(modesP)), 4, kMaxModes);

        if (!(r.pitch > -200.0f && r.pitch < 300.0f))
            return;
        const double hzNominal = 440.0 * std::pow(2.0, (static_cast<double>(r.pitch) - 69.0) / 12.0);
        const double hz =
            std::max(hzNominal * std::exp2(static_cast<double>(cardPitch) * 5.0 * static_cast<double>(tol) / 1200.0),
                     static_cast<double>(kMinFundamental));
        const double f0 = hz / r.sampleRate;

        // Exciter (restarts when the kind changes while a note sounds).
        if (fresh || exc != lastExciter)
        {
            exciter.start(exc, seed, velocity);
            lastExciter = exc;
        }
        std::array<float, 32> x{};
        const bool continuous = exc >= excParticles;
        const float target = gate ? (continuous ? sus : 0.0f) : 0.0f;
        // Envelope of the continuous exciters: ~30 ms attack, ~150 ms release.
        const float coef =
            1.0f - std::exp(-static_cast<float>(n) / static_cast<float>(r.sampleRate) / (target > envelope ? 0.03f : 0.15f));
        envelope += (target - envelope) * coef;
        // A resonator driven continuously builds up in proportion to its decay time: keep the steady level the same for any
        // T60.
        const float loudness = continuous ? std::clamp(std::sqrt(1.2f / std::max(t60, 0.02f)), 0.15f, 4.0f) : 1.0f;
        const float level = (continuous ? envelope * loudness : 1.0f) * (1.0f + cardLevel * 0.1f * tol);
        exciter.process(x.data(), n, gate, param, tim, level);

        std::array<float, 32> centre{}, side{};
        const float lfoStep = static_cast<float>(n) / static_cast<float>(r.sampleRate) * 0.5f;
        lfoPhase += lfoStep;
        lfoPhase -= std::floor(lfoPhase);
        const float lfo = lfoPhase > 0.5f ? 1.0f - lfoPhase : lfoPhase;

        if (mdl == modelModal)
        {
            // Redesign only when something that shapes the modes moved.
            // A redesign costs about as much as 25 samples of the bank: while the pitch or a shape control moves, it is
            // refreshed every fourth chunk (under 3 ms), at once for a jump of more than 3.5 cents.
            const bool changed = std::abs(f0 - lastF0) > f0 * 2.0e-6 || geom != lastGeom || bright != lastBright ||
                                 t60 != lastDecay || resolution != lastRes || tol != lastTol || sr != r.sampleRate;
            const bool jumped =
                std::abs(f0 - lastF0) > f0 * 2.0e-3 || resolution != lastRes || tol != lastTol || sr != r.sampleRate;
            ++sinceDesign;
            const bool moved = !designed || (changed && (sinceDesign >= 4 || jumped));
            if (moved)
            {
                designModes(f0, geom, bright, static_cast<double>(t60), resolution, modes);
                std::array<float, kMaxModes> fc{}, dc{};
                for (int i = 0; i < kMaxModes; ++i)
                {
                    fc[static_cast<size_t>(i)] = freqCard[static_cast<size_t>(i)] * 0.002f * tol; // up to ~3.5 cents
                    dc[static_cast<size_t>(i)] = std::clamp(decayCard[static_cast<size_t>(i)] * 0.1f * tol, -0.5f, 0.5f);
                }
                bank.apply(modes, r.sampleRate, fc, dc);
                if (exc == excBow)
                    bowed.configure(modes);
                lastF0 = f0;
                lastGeom = geom;
                lastBright = bright;
                lastDecay = t60;
                lastRes = resolution;
                lastTol = tol;
                sr = r.sampleRate;
                designed = true;
                sinceDesign = 0;
                weightsValid = false;
            }
            if (exc == excBow && bowed.modes() == 0)
                bowed.configure(modes);
            std::array<float, kMaxModes> wc1{}, ws1{};
            bank.pickups(pos, std::clamp(pos + width * (0.1f + 0.4f * lfo), 0.0f, 1.0f), wc1, ws1);
            if (!weightsValid)
            {
                wc0 = wc1;
                ws0 = ws1;
                weightsValid = true;
            }
            // Input scale: the strike excitation is an impulse of area ~strength; keep the level comparable across kinds.
            std::array<float, 32> in = x;
            for (int i = 0; i < n; ++i)
                in[static_cast<size_t>(i)] *= kModalTrim[exc];
            bank.process(in.data(), n, wc0.data(), wc1.data(), ws0.data(), ws1.data(), centre.data(), side.data());
            if (exc == excBow)
                bowed.addBlock(x.data(), 0.125f, centre.data(), kBowOut, envelope * 0.5f, wc1.data(), n);
            wc0 = wc1;
            ws0 = ws1;
            for (int i = 0; i < n; ++i)
            {
                const float c = centre[static_cast<size_t>(i)];
                const float s = side[static_cast<size_t>(i)] - c;
                centre[static_cast<size_t>(i)] = c;
                side[static_cast<size_t>(i)] = s;
            }
        }
        else
        {
            StringModel::Control c;
            c.f0 = f0;
            c.dispersion = geom;
            c.brightness = bright;
            c.position = pos;
            c.t60 = static_cast<double>(t60);
            // The loop design (exact phase delays and gains at the fundamental) is refreshed when a control moved, every
            // fourth chunk at most, at once for a jump of more than 3.5 cents or a first chunk.
            ++sinceStringDesign;
            const bool stringChanged = !stringDesigned || std::abs(f0 - lastString.f0) > f0 * 2.0e-6 ||
                                       geom != lastString.dispersion || bright != lastString.brightness ||
                                       pos != lastString.position || static_cast<double>(t60) != lastString.t60 ||
                                       sr != r.sampleRate;
            const bool stringJump = !stringDesigned || std::abs(f0 - lastString.f0) > f0 * 2.0e-3 || sr != r.sampleRate;
            if (stringChanged && (stringJump || sinceStringDesign >= 4))
            {
                string.set(c, &rng);
                lastString = c;
                stringDesigned = true;
                sinceStringDesign = 0;
                sr = r.sampleRate;
            }
            std::array<float, 32> in = x;
            for (int i = 0; i < n; ++i)
                in[static_cast<size_t>(i)] *= kStringTrim[exc];
            string.process(in.data(), centre.data(), side.data(), n);
            for (int i = 0; i < n; ++i)
            {
                const float s = side[static_cast<size_t>(i)] - centre[static_cast<size_t>(i)];
                side[static_cast<size_t>(i)] = s;
            }
        }
        fresh = false;

        float peak = 0.0f;
        for (int i = 0; i < n; ++i)
        {
            const float c = centre[static_cast<size_t>(i)] * kOutputGain;
            const float s = side[static_cast<size_t>(i)] * kOutputGain * width;
            float l = c + s, rr = c - s;
            // Guard: a bounded output whatever the settings (the bow loop and the string are the only feedback paths).
            l = std::clamp(l, -4.0f, 4.0f);
            rr = std::clamp(rr, -4.0f, 4.0f);
            r.left[i] += l;
            r.right[i] += rr;
            peak = std::max(peak, std::max(std::abs(l), std::abs(rr)));
        }
        // Silent after release: stop work until the next note.
        if (!gate && peak < 1.0e-7f && !(exciterActive(exc)))
        {
            if (++quietChunks > 8)
            {
                bank.reset();
                bowed.reset();
                string.reset();
                designed = false;
                stringDesigned = false;
                idle = true;
            }
        }
        else
        {
            quietChunks = 0;
        }
    }

    bool exciterActive(int exc) const { return exc >= excParticles && envelope > 1.0e-4f; }

    // Input trims per exciter so that every kind plays at a comparable level (a velocity-1 strike peaks near 0.7, the
    // sustained kinds sit near 0.15 rms).
    static constexpr float kModalTrim[6] = {0.0875f, 0.085f, 0.15f, 0.0145f, 0.0268f, 0.0167f};
    static constexpr float kStringTrim[6] = {5.95f, 5.95f, 9.0f, 0.9f, 0.8f, 4.5f};
    static constexpr float kBowOut = 0.05f;
    static constexpr float kOutputGain = 0.5f;

    SlotParams sp;
    int modelP = 0, exciterP = 0, geometry = 0, brightness = 0, position = 0, decay = 0, contour = 0, timbre = 0,
        sustain = 0, spread = 0, tolerance = 0, modesP = 0;
    double sr = 48000.0;          // audio thread: the rate the current design was made for
    double prepareRate = 48000.0; // set by prepare(), read by arm(); the audio thread never writes it
    Rng rng;
    Exciter exciter;
    Bank bank;
    std::atomic<Heavy*> heavy{nullptr};
    ModeSet modes;
    std::array<float, kMaxModes> freqCard{}, decayCard{}, wc0{}, ws0{};
    float cardPitch = 0.0f, cardLevel = 0.0f, envelope = 0.0f, lfoPhase = 0.0f, velocity = 1.0f;
    double lastF0 = 0.0;
    float lastGeom = -1.0f, lastBright = -1.0f, lastDecay = -1.0f, lastTol = -1.0f;
    int lastRes = -1, lastExciter = -1, quietChunks = 0, sinceDesign = 0, sinceStringDesign = 0;
    StringModel::Control lastString;
    bool stringDesigned = false;
    std::uint32_t seed = 1;
    bool gate = false, fresh = true, idle = true, designed = false, weightsValid = false;
};
} // namespace

ModuleDescriptor modalOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.modal";
    d.name = "Modal";
    d.kind = ModuleKind::Oscillator;
    d.category = "Physical";
    d.cpu = CpuClass::Light;
    d.description = "Modal resonator (up to 64 modes with exact frequency and T60 at any sample rate) or comb string, "
                    "excited by a strike, pluck, particles, blow, noise or bow; resonator, string and exciter designs "
                    "follow the Mutable Instruments Rings and Elements modules (MIT).";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto label = slotLabel(slot);
        const auto main = addSlotPage(d, slot, "modal", "Modal", "LCD shows the mode spectrum and its decay.");
        const auto exc = addSlotPage(d, slot, "modal_x", "Modal Exciter", "LCD shows the exciter and the pickup positions.");
        const auto add = [&d](ParamDef p) { d.params.push_back(std::move(p)); };
        add(params::choice(slotId(slot, "modal_model"), label + " Modal Model", main, {"Resonator", "String"}, 0));
        add(params::choice(slotId(slot, "modal_exciter"), label + " Modal Exciter", main,
                           {"Strike", "Pluck", "Particles", "Blow", "Noise", "Bow"}, 0));
        add(voiceMod(params::floating(slotId(slot, "modal_geometry"), label + " Modal Geometry", main, 0.0f, 1.0f, 0.25f)));
        add(voiceMod(
            params::floating(slotId(slot, "modal_brightness"), label + " Modal Brightness", main, 0.0f, 1.0f, 0.5f)));
        add(voiceMod(params::floating(slotId(slot, "modal_position"), label + " Modal Position", main, 0.0f, 1.0f, 0.25f)));
        add(voiceMod(params::floating(slotId(slot, "modal_decay"), label + " Modal Decay (T60)", main, 0.02f, 30.0f, 1.2f,
                                      "s", 0.3f)));
        add(voiceMod(
            params::floating(slotId(slot, "modal_contour"), label + " Modal Exciter Contour", exc, 0.0f, 1.0f, 0.3f)));
        add(voiceMod(
            params::floating(slotId(slot, "modal_timbre"), label + " Modal Exciter Timbre", exc, 0.0f, 1.0f, 0.7f)));
        add(voiceMod(
            params::floating(slotId(slot, "modal_sustain"), label + " Modal Exciter Sustain", exc, 0.0f, 1.0f, 0.8f)));
        add(voiceMod(params::floating(slotId(slot, "modal_spread"), label + " Modal Stereo Spread", exc, 0.0f, 1.0f, 0.3f)));
        add(params::floating(slotId(slot, "modal_tol"), label + " Modal Tolerance", exc, 0.0f, 1.0f, 0.0f));
        add(params::integer(slotId(slot, "modal_modes"), label + " Modal Modes", exc, 4, kMaxModes, kMaxModes));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

const char* modalOscillatorName()
{
    return "Modal";
}

} // namespace crucible::modules
