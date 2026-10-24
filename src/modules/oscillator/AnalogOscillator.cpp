// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Dsp.h>
#include <crucible/FineBlep.h>
#include <crucible/MinBlep.h>
#include <crucible/Simd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

// Virtual analog oscillator (P9). One slot renders up to eight unison voices; each voice is a double-precision phase
// that drives saw, pulse, triangle and sine components. Every discontinuity (a step) and every slope break (a corner)
// of the output is located at its exact sub-sample position and corrected with the 32-zero-crossing minBLEP /
// minBLAMP of MinBlep.h: wraps, pulse edges (also while the width moves), triangle corners, the finite reset of the
// saw-core character, hard-sync resets, soft-sync reversals and every crossing of a through-zero FM phase. Above fs/4
// the output crossfades over half an octave into the one-harmonic band-limited table of the same shape (the mip level
// that holds only the fundamental). The pulse is DC blocked at 5 Hz.
//
// Characters model circuit principles: the saw-core VCO charges a capacitor (curved ramp), resets in a finite time and
// derives a comparator pulse and a folded, softly shaped triangle from the ramp; the triangle-core VCO switches the
// integrator slope for its saw and shapes a tanh sine from the triangle; the DCO is exact in pitch, its saw level falls
// with pitch unless compensated and it has a divider sub.
//
// Cross modulation uses an internal modulator phase at a ratio of the pitch (hard and soft sync, through-zero linear
// FM, exponential FM, ring). The OscillatorInstance interface renders one slot in isolation, so another slot's output
// is not reachable: PWM from a slot goes through the generic modulation system (the pulse width is a voice-modulation
// target), and the cross modulation of the slot (osc2_xmod_*) stays the voice's.

namespace crucible::modules
{
namespace
{
constexpr int kMaxUnison = 8;
constexpr int kMaxAudioBlock = 64; // longest block with per-sample modulation (the voice renders 32)
constexpr double kTwoPiD = 6.283185307179586;
constexpr double kPiD = 3.141592653589793;
constexpr double kEdgeEps = 1.0e-9;
constexpr double kMaxStep = 0.45; // largest phase advance per sample (cycles)

enum Wave
{
    waveSaw,
    wavePulse,
    waveTriangle,
    waveSine,
    waveSawPulse,
    waveMorph
};
enum Character
{
    charIdeal,
    charSawCore,
    charTriCore,
    charDco
};
enum Mode
{
    modeOff,
    modeHardSync,
    modeSoftSync,
    modeLinearFm,
    modeExpFm,
    modeRing
};

using crucible::BlepBuffer;
using crucible::FineBlep;
constexpr int kLag = crucible::kBlepLag;

// floor() for the phase arithmetic. MSVC calls into the C runtime for floor() without SSE4.1, so there the common case
// (|x| well inside the int64 range) truncates and corrects; the result is the same value as std::floor.
CRUCIBLE_ALWAYS_INLINE double floorFast(double x)
{
#if defined(_MSC_VER) && !defined(__clang__)
    if (std::abs(x) < 4.0e15)
    {
        const double t = static_cast<double>(static_cast<long long>(x));
        return t > x ? t - 1.0 : t;
    }
#endif
    return std::floor(x);
}

// sin(2 pi x) for any x, accurate to about 1e-10 (Taylor series on the folded quarter period).
inline double sinTurns(double x)
{
    double y = x - std::floor(x + 0.5);
    if (y > 0.25)
        y = 0.5 - y;
    else if (y < -0.25)
        y = -0.5 - y;
    const double t = y * kTwoPiD;
    const double t2 = t * t;
    return t * (1.0 +
                t2 * (-1.0 / 6.0 +
                      t2 * (1.0 / 120.0 + t2 * (-1.0 / 5040.0 + t2 * (1.0 / 362880.0 + t2 * (-1.0 / 39916800.0 +
                                                                                             t2 * (1.0 / 6227020800.0)))))));
}
inline double cosTurns(double x)
{
    return sinTurns(x + 0.25);
}

struct Weights
{
    double saw = 0.0, tri = 0.0, sine = 0.0, pulse = 0.0;
};

inline Weights lerp(const Weights& a, const Weights& b, double t)
{
    return {a.saw + (b.saw - a.saw) * t, a.tri + (b.tri - a.tri) * t, a.sine + (b.sine - a.sine) * t,
            a.pulse + (b.pulse - a.pulse) * t};
}

// The shapes of one voice for one chunk: values and slopes (per cycle) of the saw, triangle and sine components
// of the chosen character, and the comparator pulse.
struct Core
{
    int character = charIdeal;
    double kappa = 0.0; // ramp curvature: c(u) = (1 - exp(-kappa u)) * norm
    double norm = 1.0;
    double rho = 0.0; // share of the cycle taken by the reset (fall)
    double riseEnd = 1.0;
    double invRise = 1.0;
    double invRho = 0.0;
    double sawGain = 1.0;
    double triMid = 0.5; // where the folded triangle turns
    double softK = 0.8;  // saw-core triangle shaper
    double softNorm = 1.0;
    double sineK = 1.3; // triangle-core tanh sine
    double sineNorm = 1.0;

    void setup(int ch, double curvature, double resetShare)
    {
        character = ch;
        kappa = curvature;
        norm = kappa > 1.0e-6 ? 1.0 / (1.0 - std::exp(-kappa)) : 1.0;
        rho = resetShare;
        riseEnd = 1.0 - rho;
        invRise = 1.0 / riseEnd;
        invRho = rho > 0.0 ? 1.0 / rho : 0.0;
        triMid = 0.5;
        if (ch == charSawCore && kappa > 1.0e-6)
            triMid = -std::log(1.0 - 0.5 / norm) / kappa;
        if (ch == charSawCore)
            softNorm = 1.0 / std::tanh(softK);
        else if (ch == charTriCore)
            sineNorm = 1.0 / std::tanh(sineK);
    }

    double curve(double u) const { return kappa > 1.0e-6 ? (1.0 - std::exp(-kappa * u)) * norm : u; }
    double curveSlope(double u) const { return kappa > 1.0e-6 ? kappa * std::exp(-kappa * u) * norm : 1.0; }

    double sawV(double p) const
    {
        if (p < riseEnd)
            return sawGain * (2.0 * curve(p * invRise) - 1.0);
        return sawGain * (1.0 - 2.0 * (p - riseEnd) * invRho);
    }
    double sawD(double p) const
    {
        if (p < riseEnd)
            return sawGain * 2.0 * curveSlope(p * invRise) * invRise;
        return sawGain * -2.0 * invRho;
    }

    double triRaw(double p) const
    {
        if (character == charSawCore)
            return 2.0 * std::abs(2.0 * curve(p) - 1.0) - 1.0;
        return p < 0.5 ? 4.0 * p - 1.0 : 3.0 - 4.0 * p;
    }
    double triRawD(double p) const
    {
        if (character == charSawCore)
        {
            const double s = 2.0 * curve(p) - 1.0;
            return (s < 0.0 ? -4.0 : 4.0) * curveSlope(p);
        }
        return p < 0.5 ? 4.0 : -4.0;
    }
    double triV(double p) const
    {
        const double raw = triRaw(p);
        return character == charSawCore ? std::tanh(softK * raw) * softNorm : raw;
    }
    double triD(double p) const
    {
        const double raw = triRaw(p);
        if (character != charSawCore)
            return triRawD(p);
        const double th = std::tanh(softK * raw);
        return softK * (1.0 - th * th) * softNorm * triRawD(p);
    }
    double sineV(double p) const
    {
        if (character == charTriCore)
            return std::tanh(sineK * triRaw(p)) * sineNorm;
        return sinTurns(p);
    }
    double sineD(double p) const
    {
        if (character == charTriCore)
        {
            const double th = std::tanh(sineK * triRaw(p));
            return sineK * (1.0 - th * th) * sineNorm * triRawD(p);
        }
        return kTwoPiD * cosTurns(p);
    }
};

inline double otherV(const Core& c, const Weights& w, double p)
{
    double v = 0.0;
    if (w.saw != 0.0)
        v += w.saw * c.sawV(p);
    if (w.tri != 0.0)
        v += w.tri * c.triV(p);
    if (w.sine != 0.0)
        v += w.sine * c.sineV(p);
    return v;
}
inline double otherD(const Core& c, const Weights& w, double p)
{
    double v = 0.0;
    if (w.saw != 0.0)
        v += w.saw * c.sawD(p);
    if (w.tri != 0.0)
        v += w.tri * c.triD(p);
    if (w.sine != 0.0)
        v += w.sine * c.sineD(p);
    return v;
}
// +1 while the phase is inside [a, b) (wrapping when a > b), else -1.
inline double pulseV(double p, double a, double b)
{
    const bool high = a < b ? (p >= a && p < b) : (p >= a || p < b);
    return high ? 1.0 : -1.0;
}

// Everything that is constant over one chunk of one unison voice.
using EdgeGrid = std::array<double, 5>;

// Work space of one render() with an audio-rate width: kept out of the stack frame (render() runs for every voice and slot
// of every chunk, with or without it). Each instance owns one, allocated when it is created on the control thread.
struct AudioScratch
{
    std::array<EdgeGrid, kMaxAudioBlock> gridA, gridB;
    std::array<double, kMaxAudioBlock + 3> widths;
};

struct Chunk
{
    Core core;
    Weights w0, w1;
    double a0 = 0.0, a1 = 0.0, b0 = 0.5, b1 = 0.5; // pulse edges: rising at a, falling at b
    double subLevel = 0.0;
    int mode = modeOff;
    double ratio = 1.0;
    double amt = 0.0;
    double inc = 0.0; // phase advance per sample at the voice pitch
    std::array<double, 4> other{};
    int numOther = 0;
    bool useCross = false;
    double cross = 0.0;
    double dcCoef = 0.0;
    double gain = 1.0;
    bool pulseStream = false; // the pulse (or sub) stream is live in this chunk
    bool sawOnly = false;     // only the saw component of an ideal or DCO core sounds
    // Audio-rate modulation of the chunk (null: none). pitchMod: semitones per sample; fmMod: added to the cross-modulation
    // amount; gridA / gridB: the pulse edges of every sample at five instants (0, 1/4, 1/2, 3/4, 1 of the sample) on the
    // smooth curve of the modulated width (null when the width does not move per sample).
    const float* pitchMod = nullptr;
    const float* fmMod = nullptr;
    const EdgeGrid* gridA = nullptr;
    const EdgeGrid* gridB = nullptr;
};

struct Uni
{
    double ph = 0.0;
    double mph = 0.0;
    double dir = 1.0;
    double ou = 0.0;
    double sub = 1.0;
    double dcLp = 0.0;
    double pmPrev = 0.0; // pitch modulation (semitones) at the end of the previous chunk
    bool pmFresh = true;
    std::array<double, 3> wHist{}; // the last three widths of the previous chunk, for the audio-rate width curve
    bool wFresh = true;
    bool fresh = true;
    float cardPitch = 0.0f, cardWidth = 0.0f, cardLevel = 0.0f, cardCurve = 0.0f;
    BlepBuffer bufO, bufP;
    std::array<double, kLag> lagO{}, lagP{}, lagF{};
    int lagPos = 0;
    int pulseTail = 0; // samples the pulse buffer still has to be drained for
    bool pulseLive = false, crossLive = false;
    bool lagFresh = true;
};

struct SampleCtx
{
    Weights w;
    double aS = 0.0, aE = 0.0, bS = 0.0, bE = 0.0;
    // Edges at 0, 1/4, 1/2, 3/4 and 1 of the sample when the width follows an audio-rate curve (else null: straight lines).
    const EdgeGrid* gridA = nullptr;
    const EdgeGrid* gridB = nullptr;
    // Position of an edge (e = 0 rising, 1 falling) at time tau of the sample.
    double edge(int e, double tau) const
    {
        if (gridA == nullptr)
        {
            const double pS = e == 0 ? aS : bS, pE = e == 0 ? aE : bE;
            return pS + (pE - pS) * tau;
        }
        const EdgeGrid& g = e == 0 ? *gridA : *gridB;
        const double x = std::clamp(tau, 0.0, 1.0) * 4.0;
        const int i = std::min(static_cast<int>(x), 3);
        return g[static_cast<size_t>(i)] + (g[static_cast<size_t>(i) + 1] - g[static_cast<size_t>(i)]) * (x - i);
    }
};

class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        wave = sp.bind("an_wave");
        shape = sp.bind("an_shape");
        pw = sp.bind("an_pw");
        character = sp.bind("an_char");
        sub = sp.bind("an_sub");
        comp = sp.bind("an_dco_comp");
        xmode = sp.bind("an_xmod_mode");
        xratio = sp.bind("an_xmod_ratio");
        xamt = sp.bind("an_xmod_amt");
        unison = sp.bind("an_uni");
        detune = sp.bind("an_detune");
        spread = sp.bind("an_spread");
        drift = sp.bind("an_drift");
        tau = sp.bind("an_drift_tau");
        tolerance = sp.bind("an_tol");
        FineBlep::instance(); // builds the tables on this (control) thread
        scratch = std::make_unique<AudioScratch>();
    }

    void prepare(double rate, int) override { sr = rate; }
    bool usesVoiceCard() const override { return true; }
    bool supportsAudioRate() const override { return true; }
    bool continuesOnRetrigger() const override { return true; }

    void noteOn(const OscNoteInfo& info) override
    {
        rng.state = (info.seed * 2654435761U) | 1U;
        // The tolerances of every unison voice (pitch, width, level and curvature) come from the voice card when the voice
        // has one (VoiceCard.h: fixed per voice, the same note after note). Without a card they are drawn from the note
        // seed, so every note is a new but reproducible "voice" (the legacy source, VoiceCard::oscillatorFromSeed).
        std::array<OscCard, kCardUnison> fromSeed;
        const OscCard* cards = info.card;
        if (cards == nullptr)
        {
            VoiceCard::oscillatorFromSeed(info.seed, fromSeed.data());
            cards = fromSeed.data();
        }
        cardAmount = info.card != nullptr ? static_cast<double>(info.cardAmount) : 0.0;
        for (int k = 0; k < kMaxUnison; ++k)
        {
            auto& u = uni[static_cast<size_t>(k)];
            u.cardPitch = cards[k].pitch;
            u.cardWidth = cards[k].width;
            u.cardLevel = cards[k].level;
            u.cardCurve = cards[k].curve;
        }
        for (int k = 0; k < kMaxUnison; ++k)
        {
            auto& u = uni[static_cast<size_t>(k)];
            u.ph = k == 0 ? 0.0 : static_cast<double>(rng.unipolar());
            u.mph = 0.0;
            u.dir = 1.0;
            u.ou = static_cast<double>(gauss(rng));
            u.sub = 1.0;
            u.fresh = true;
            u.pmFresh = true;
            u.wFresh = true;
            u.lagFresh = true;
            u.lagPos = 0;
            u.pulseTail = 0;
            u.bufO.reset();
            u.bufP.reset();
        }
        first = true;
    }

    void render(const OscRender& r) override
    {
        if (sr != r.sampleRate)
            sr = r.sampleRate;
        const int wv = static_cast<int>(sp.value(r.values, r.targetMod, wave));
        const double sh = static_cast<double>(sp.value(r.values, r.targetMod, shape));
        const double width = static_cast<double>(sp.value(r.values, r.targetMod, pw));
        const int ch = static_cast<int>(sp.value(r.values, r.targetMod, character));
        const double subLevel = static_cast<double>(sp.value(r.values, r.targetMod, sub));
        const bool compensated = sp.value(r.values, r.targetMod, comp) > 0.5f;
        const int mode = static_cast<int>(sp.value(r.values, r.targetMod, xmode));
        const double ratio = mode != modeOff ? static_cast<double>(sp.value(r.values, r.targetMod, xratio)) : 1.0;
        const double amount = mode != modeOff ? static_cast<double>(sp.value(r.values, r.targetMod, xamt)) : 0.0;
        const int voices = std::clamp(static_cast<int>(std::lround(sp.value(r.values, r.targetMod, unison))), 1, kMaxUnison);
        const double detuneCents = voices > 1 ? static_cast<double>(sp.value(r.values, r.targetMod, detune)) : 0.0;
        const double spreadAmt = voices > 1 ? static_cast<double>(sp.value(r.values, r.targetMod, spread)) : 0.0;
        const double driftAmt = static_cast<double>(sp.value(r.values, r.targetMod, drift));
        const double driftTau = driftAmt > 0.0 ? static_cast<double>(sp.value(r.values, r.targetMod, tau)) : 1.0;
        // The slot's own tolerance, or the voice card's amount when that is larger.
        const double tol = std::max(static_cast<double>(sp.value(r.values, r.targetMod, tolerance)), cardAmount);

        const int n = r.numSamples;
        if (n <= 0)
            return;
        const double dt = static_cast<double>(n) / sr;
        if (dcRate != sr)
        {
            dcRate = sr;
            dcCoefCached = 1.0 - std::exp(-kTwoPiD * 5.0 / sr);
        }
        const double dcCoef = dcCoefCached;
        const double voiceGain = voices > 1 ? 1.0 / std::sqrt(static_cast<double>(voices)) : 1.0;

        Weights target = weightsFor(wv, sh);
        const bool dco = ch == charDco;
        for (int k = 0; k < voices; ++k)
        {
            auto& u = uni[static_cast<size_t>(k)];
            Chunk c;
            // Pitch: unison spread, drift (Ornstein-Uhlenbeck, std drift * 6 cents) and the tolerance of the card.
            double cents = 0.0;
            if (voices > 1)
                cents += (2.0 * k / (voices - 1) - 1.0) * detuneCents;
            if (driftAmt > 0.0)
            {
                const double a = std::exp(-dt / std::max(driftTau, 0.05));
                u.ou = a * u.ou + std::sqrt(1.0 - a * a) * static_cast<double>(gauss(rng));
                cents += u.ou * driftAmt * 6.0;
            }
            if (!dco)
                cents += static_cast<double>(u.cardPitch) * 5.0 * tol;
            double hz = 440.0 * std::exp2((static_cast<double>(r.pitch) - 69.0 + cents * 0.01) / 12.0);
            // A NaN pitch would put NaN into the phase for good (and floor(NaN) cast to an integer is undefined).
            if (!std::isfinite(hz))
                hz = hz > 0.0 ? 0.49 * sr : 0.0;
            c.inc = std::min(hz / sr, 0.49);
            const double widthCard = static_cast<double>(u.cardWidth) * 0.015 * tol;
            const double widthEff = std::clamp(width + widthCard, 0.02, 0.98);

            // Core of the character.
            double curvature = 0.0, resetShare = 0.0;
            if (ch == charSawCore)
            {
                curvature = std::max(0.0, 0.35 * (1.0 + static_cast<double>(u.cardCurve) * 0.25 * tol));
                resetShare = std::clamp(6.0e-6 * hz, 1.0e-5, 0.15);
            }
            else if (ch == charTriCore)
            {
                resetShare = 1.0 / 16.0;
            }
            c.core.setup(ch, curvature, resetShare);
            if (dco && !compensated)
                c.core.sawGain = 1.0 / (1.0 + hz / 2000.0);

            // Weights and pulse edges: previous chunk -> this chunk, so nothing steps at a chunk boundary.
            Weights tw = target;
            tw.saw *= 1.0 + static_cast<double>(u.cardLevel) * 0.06 * tol;
            tw.tri *= 1.0 + static_cast<double>(u.cardLevel) * 0.06 * tol;
            tw.sine *= 1.0 + static_cast<double>(u.cardLevel) * 0.06 * tol;
            tw.pulse *= 1.0 + static_cast<double>(u.cardLevel) * 0.06 * tol;
            double a1 = 0.0, b1 = widthEff;
            pulseEdges(c.core, ch, widthEff, a1, b1);
            // Audio-rate pulse width: the edges of every sample at five instants on the smooth (cubic Hermite) curve through
            // the widths. The curve needs the sample after the one it describes, so it runs one sample late: the edges of
            // sample i follow the widths of samples i - 2 to i - 1, with the three before this chunk kept in `wHist`.
            if (r.widthMod != nullptr && n <= kMaxAudioBlock)
            {
                auto& gridA = scratch->gridA;
                auto& gridB = scratch->gridB;
                auto& w = scratch->widths;
                for (int i = 0; i < n; ++i)
                    w[static_cast<size_t>(i) + 3] =
                        std::clamp(width + widthCard + static_cast<double>(r.widthMod[i]), 0.02, 0.98);
                for (size_t j = 0; j < 3; ++j)
                    w[j] = u.wFresh ? w[3] : u.wHist[j];
                u.wFresh = false;
                for (int i = 0; i < n; ++i)
                {
                    const auto si = static_cast<size_t>(i);
                    const double p0 = w[si + 1], p1 = w[si + 2], m0 = 0.5 * (w[si + 2] - w[si]),
                                 m1 = 0.5 * (w[si + 3] - w[si + 1]);
                    for (size_t q = 0; q < 5; ++q)
                    {
                        const double t = 0.25 * static_cast<double>(q), t2 = t * t, t3 = t2 * t;
                        const double wq = (2.0 * t3 - 3.0 * t2 + 1.0) * p0 + (t3 - 2.0 * t2 + t) * m0 +
                                          (3.0 * t2 - 2.0 * t3) * p1 + (t3 - t2) * m1;
                        pulseEdges(c.core, ch, std::clamp(wq, 0.02, 0.98), gridA[si][q], gridB[si][q]);
                    }
                }
                for (size_t j = 0; j < 3; ++j)
                    u.wHist[j] = w[static_cast<size_t>(n) + j];
                a1 = gridA[static_cast<size_t>(n) - 1][4];
                b1 = gridB[static_cast<size_t>(n) - 1][4];
                c.gridA = gridA.data();
                c.gridB = gridB.data();
            }
            else
                u.wFresh = true;
            if (r.pitchMod != nullptr && n <= kMaxAudioBlock)
                c.pitchMod = r.pitchMod;
            if (r.fmAmountMod != nullptr && n <= kMaxAudioBlock && mode != modeOff)
                c.fmMod = r.fmAmountMod;
            if (first || u.fresh)
            {
                prevW[static_cast<size_t>(k)] = tw;
                prevA[static_cast<size_t>(k)] = a1;
                prevB[static_cast<size_t>(k)] = b1;
            }
            c.w0 = prevW[static_cast<size_t>(k)];
            c.w1 = tw;
            c.a0 = prevA[static_cast<size_t>(k)];
            c.a1 = a1;
            c.b0 = prevB[static_cast<size_t>(k)];
            c.b1 = b1;
            prevW[static_cast<size_t>(k)] = tw;
            prevA[static_cast<size_t>(k)] = a1;
            prevB[static_cast<size_t>(k)] = b1;

            c.subLevel = subLevel;
            c.mode = mode;
            c.ratio = ratio;
            c.amt = amount;
            c.dcCoef = dcCoef;
            c.gain = voiceGain;
            buildBoundaries(c);
            if (mode == modeOff || mode == modeRing)
            {
                const double x = c.inc > 0.25 ? std::log2(c.inc / 0.25) / 0.5 : 0.0;
                c.cross = std::clamp(x, 0.0, 1.0);
                c.useCross = c.cross > 0.0;
            }

            double gl = 1.0, gr = 1.0;
            if (voices > 1)
            {
                const double pos = (2.0 * k / (voices - 1) - 1.0) * spreadAmt;
                const double angle = (pos + 1.0) * 0.78539816339744831;
                gl = std::cos(angle) * 1.4142135623730951;
                gr = std::sin(angle) * 1.4142135623730951;
            }
            c.pulseStream = tw.pulse != 0.0 || c.w0.pulse != 0.0 || subLevel > 0.0;
            c.sawOnly = c.core.character != charSawCore && c.core.character != charTriCore && tw.tri == 0.0 &&
                        tw.sine == 0.0 && c.w0.tri == 0.0 && c.w0.sine == 0.0;
            runVoice(u, c, n, gl, gr, r.left, r.right);
        }
        first = false;
    }

private:
    static float gauss(Rng& g)
    {
        float s = -6.0f;
        for (int i = 0; i < 12; ++i)
            s += g.unipolar();
        return s;
    }

    static Weights weightsFor(int wv, double sh)
    {
        Weights w;
        switch (wv)
        {
        case wavePulse:
            w.pulse = 1.0;
            break;
        case waveTriangle:
            w.tri = 1.0;
            break;
        case waveSine:
            w.sine = 1.0;
            break;
        case waveSawPulse:
            w.saw = 1.0 - sh;
            w.pulse = sh;
            break;
        case waveMorph:
        {
            // sine -> triangle -> saw -> pulse, one third of the control each
            const double x = std::clamp(sh, 0.0, 1.0) * 3.0;
            const int seg = std::min(2, static_cast<int>(x));
            const double f = x - seg;
            if (seg == 0)
            {
                w.sine = 1.0 - f;
                w.tri = f;
            }
            else if (seg == 1)
            {
                w.tri = 1.0 - f;
                w.saw = f;
            }
            else
            {
                w.saw = 1.0 - f;
                w.pulse = f;
            }
            break;
        }
        default:
            w.saw = 1.0;
            break;
        }
        return w;
    }

    // Rising edge a and falling edge b of the pulse (high on [a, b), wrapping).
    static void pulseEdges(const Core& core, int ch, double width, double& a, double& b)
    {
        if (ch == charSawCore)
        {
            // Comparator on the ramp: high while the ramp is below the threshold (plus the comparator's offset).
            const double thr = std::clamp(2.0 * width - 1.0 + 0.03, -0.98, 0.98);
            const double cLevel = (thr + 1.0) * 0.5;
            double u = cLevel;
            if (core.kappa > 1.0e-6)
                u = -std::log(1.0 - cLevel / core.norm) / core.kappa;
            b = u * core.riseEnd;
            a = core.riseEnd + core.rho * (1.0 - thr) * 0.5; // the reset sweeps the ramp down through the threshold
            if (a >= 1.0)
                a = 0.0;
        }
        else if (ch == charTriCore)
        {
            a = 0.5 - width * 0.5;
            b = 0.5 + width * 0.5;
        }
        else
        {
            a = 0.0;
            b = width;
        }
    }

    static void buildBoundaries(Chunk& c)
    {
        const auto add = [&c](double p)
        {
            for (int i = 0; i < c.numOther; ++i)
                if (std::abs(c.other[static_cast<size_t>(i)] - p) < 1.0e-12)
                    return;
            c.other[static_cast<size_t>(c.numOther++)] = p;
        };
        const bool saw = c.w0.saw != 0.0 || c.w1.saw != 0.0;
        const bool tri = c.w0.tri != 0.0 || c.w1.tri != 0.0;
        const bool sine = c.w0.sine != 0.0 || c.w1.sine != 0.0;
        if (saw)
        {
            add(0.0);
            if (c.core.rho > 0.0)
                add(c.core.riseEnd);
        }
        if (tri)
        {
            add(0.0);
            add(c.core.triMid);
        }
        if (sine && c.core.character == charTriCore)
        {
            add(0.0);
            add(0.5);
        }
    }

    void emitOther(Uni& u, const Chunk& c, const Weights& w, double p, bool up, double dphi, double tt)
    {
        const double before = (up ? p - kEdgeEps : p + kEdgeEps);
        const double after = (up ? p + kEdgeEps : p - kEdgeEps);
        const double pb = before - std::floor(before);
        const double pa = after - std::floor(after);
        const double step = otherV(c.core, w, pa) - otherV(c.core, w, pb);
        const double kink = (otherD(c.core, w, pa) - otherD(c.core, w, pb)) * dphi;
        const auto d = static_cast<float>(1.0 - tt);
        if (std::abs(step) > 1.0e-12)
            u.bufO.addStep(static_cast<float>(step), d);
        if (std::abs(kink) > 1.0e-12)
            u.bufO.addKink(static_cast<float>(kink), d);
    }

    // Phase path of a sample: P(sigma) = sum k[i] sigma^i (sigma in samples, i = 1..5). A segment starting at sigma = o sees
    // phase(s) = P(o + s) - P(o).
    struct Path
    {
        std::array<double, 6> k{};
        double o = 0.0;
        Path() = default;
        explicit Path(double slope0) { k[1] = slope0; }
        double eval(double sg) const { return sg * (k[1] + sg * (k[2] + sg * (k[3] + sg * (k[4] + sg * k[5])))); }
        double deriv(double sg) const
        {
            return k[1] + sg * (2.0 * k[2] + sg * (3.0 * k[3] + sg * (4.0 * k[4] + sg * 5.0 * k[5])));
        }
        double at(double s) const { return eval(o + s) - eval(o); }
        double slope(double s) const { return deriv(o + s); }
        bool curved() const { return k[2] != 0.0 || k[3] != 0.0 || k[4] != 0.0 || k[5] != 0.0; }
    };

    // Phase path through the phase advance per sample f(sigma) sampled at 0, 1/4, 1/2, 3/4 and 1: the integral of the
    // quartic through the five values (Boole's rule at the sample instants, so the phase there is that of the continuous
    // modulation to about 1e-9 cycles, and crossings are found on a curve that follows it).
    static Path pathFrom(const std::array<double, 5>& f)
    {
        Path path;
        const double c1 = (-25.0 * f[0] + 48.0 * f[1] - 36.0 * f[2] + 16.0 * f[3] - 3.0 * f[4]) / 3.0;
        const double c2 = (70.0 * f[0] - 208.0 * f[1] + 228.0 * f[2] - 112.0 * f[3] + 22.0 * f[4]) / 3.0;
        const double c3 = (-80.0 * f[0] + 288.0 * f[1] - 384.0 * f[2] + 224.0 * f[3] - 48.0 * f[4]) / 3.0;
        const double c4 = (32.0 * f[0] - 128.0 * f[1] + 192.0 * f[2] - 128.0 * f[3] + 32.0 * f[4]) / 3.0;
        path.k[1] = f[0];
        path.k[2] = 0.5 * c1;
        path.k[3] = c2 / 3.0;
        path.k[4] = 0.25 * c3;
        path.k[5] = 0.2 * c4;
        return path;
    }

    // Time (sigma, 0..len) at which phase0 + path crosses position(sg) + shift. fa and fb are the signed distances at both
    // ends (opposite signs), so the secant is the exact answer for a straight path and the start of a Newton refinement
    // for a curved one.
    static double crossing(const Path& path, double phase0, double pos0, double dpos, double shift, double len, double fa,
                           double fb)
    {
        double sg = len * fa / (fa - fb);
        if (!path.curved() && dpos == 0.0)
            return sg;
        for (int it = 0; it < 5; ++it)
        {
            const double g = phase0 + path.at(sg) - (pos0 + dpos * sg + shift);
            const double dg = path.slope(sg) - dpos;
            if (std::abs(dg) < 1.0e-12)
                break;
            sg = std::clamp(sg - g / dg, 0.0, len);
        }
        return sg;
    }

    // Splits a curved path where its slope changes sign, so that no boundary is crossed twice within one piece.
    void segmentMonotone(Uni& u, const Chunk& c, const SampleCtx& s, const Path& path, double tA, double tB)
    {
        const double len = tB - tA;
        std::array<double, 6> cut{};
        int cuts = 0;
        if (path.curved())
        {
            constexpr int probes = 16;
            double previous = path.slope(0.0);
            double previousAt = 0.0;
            for (int i = 1; i <= probes && cuts < 4; ++i)
            {
                const double at = len * i / probes;
                const double v = path.slope(at);
                if ((previous < 0.0) != (v < 0.0))
                {
                    double lo = previousAt, hi = at;
                    const bool loNegative = previous < 0.0;
                    for (int it = 0; it < 40; ++it)
                    {
                        const double mid = 0.5 * (lo + hi);
                        if ((path.slope(mid) < 0.0) == loNegative)
                            lo = mid;
                        else
                            hi = mid;
                    }
                    cut[static_cast<size_t>(cuts++)] = 0.5 * (lo + hi);
                }
                previous = v;
                previousAt = at;
            }
        }
        double from = 0.0;
        for (int i = 0; i <= cuts; ++i)
        {
            const double to = i < cuts ? cut[static_cast<size_t>(i)] : len;
            if (to <= from + 1.0e-9)
                continue;
            if (s.gridA == nullptr)
            {
                Path piece = path;
                piece.o = path.o + from;
                segment(u, c, s, piece, tA + from, tA + to);
            }
            else
            {
                // A width on a curve: when an edge is crossed, every quarter of the sample is a piece of its own (the edges
                // move in straight lines inside a piece), so the placement error of the moving edges is a sixteenth of the
                // sample's. The edge moves slower than the phase, so a crossing shows at the ends of the sample; most
                // samples have none and take one piece.
                Path whole = path;
                whole.o = path.o + from;
                const double span = to - from;
                const double ph0 = u.ph, ph1 = ph0 + (whole.curved() ? whole.at(span) : whole.k[1] * span);
                bool crossed = false;
                for (int e = 0; e < 2 && !crossed; ++e)
                    crossed = std::floor(ph0 - s.edge(e, tA + from)) != std::floor(ph1 - s.edge(e, tA + to));
                if (!crossed)
                {
                    segment(u, c, s, whole, tA + from, tA + to);
                }
                else
                {
                    double a = from;
                    while (a < to - 1.0e-9)
                    {
                        const double quarter = std::floor((tA + a) * 4.0 + 1.0e-9) / 4.0 + 0.25 - tA;
                        const double b = std::min(to, quarter);
                        Path piece = path;
                        piece.o = path.o + a;
                        segment(u, c, s, piece, tA + a, tA + b);
                        a = b;
                    }
                }
            }
            from = to;
        }
    }

    // The same as segment() for a straight path (constant slope) when nothing is crossed in the piece (the usual sample):
    // no boundary work and no Path. Anything else falls through to segment(), which redoes the checks from scratch.
    CRUCIBLE_ALWAYS_INLINE void segmentStraight(Uni& u, const Chunk& c, const SampleCtx& s, double slope, double tA, double tB)
    {
        const double len = tB - tA;
        const double ph0 = u.ph;
        const double ph1 = ph0 + slope * len;
        bool quiet = s.gridA == nullptr;
        if (quiet)
        {
            for (int k = 0; k < c.numOther; ++k)
            {
                const double p = c.other[static_cast<size_t>(k)];
                if (floorFast(ph0 - p) != floorFast(ph1 - p))
                {
                    quiet = false;
                    break;
                }
            }
        }
        if (quiet && (c.w0.pulse != 0.0 || c.w1.pulse != 0.0))
        {
            const double p0a = s.aS + (s.aE - s.aS) * tA, p1a = s.aS + (s.aE - s.aS) * tB;
            const double p0b = s.bS + (s.bE - s.bS) * tA, p1b = s.bS + (s.bE - s.bS) * tB;
            quiet = floorFast(ph0 - p0a) == floorFast(ph1 - p1a) && floorFast(ph0 - p0b) == floorFast(ph1 - p1b);
        }
        if (quiet && c.subLevel > 0.0)
            quiet = floorFast(ph0) == floorFast(ph1);
        if (!quiet)
        {
            segment(u, c, s, Path(slope), tA, tB);
            return;
        }
        u.ph = ph1 - floorFast(ph1);
    }

    // Moves the phase over [tA, tB] of the sample along `path` (sigma = t - tA), emitting the corrections of every boundary
    // crossed.
    CRUCIBLE_ALWAYS_INLINE void segment(Uni& u, const Chunk& c, const SampleCtx& s, const Path& path, double tA, double tB)
    {
        const double len = tB - tA;
        const double ph0 = u.ph;
        const double ph1 = ph0 + (path.curved() ? path.at(len) : path.k[1] * len);
        for (int k = 0; k < c.numOther; ++k)
        {
            const double p = c.other[static_cast<size_t>(k)];
            const double uu = ph0 - p, vv = ph1 - p;
            const double fu = std::floor(uu), fv = std::floor(vv);
            if (fu == fv)
                continue;
            const bool up = vv > uu;
            const int lo = static_cast<int>(up ? fu + 1.0 : fv + 1.0), hi = static_cast<int>(up ? fv : fu);
            for (int sft = lo; sft <= hi; ++sft)
            {
                const double sg = crossing(path, ph0, p, 0.0, sft, len, uu - sft, vv - sft);
                emitOther(u, c, s.w, p, up, path.slope(sg), tA + sg);
            }
        }
        const bool pulseOn = c.w0.pulse != 0.0 || c.w1.pulse != 0.0;
        if (pulseOn)
        {
            for (int e = 0; e < 2; ++e)
            {
                const double pS = e == 0 ? s.aS : s.bS, pE = e == 0 ? s.aE : s.bE;
                const double p0 = s.edge(e, tA), p1 = s.edge(e, tB);
                // Speed of the edge per sample: the straight line of the sample, or the chord of this piece of the curve.
                const double edgeSpeed = s.gridA == nullptr ? pE - pS : (p1 - p0) / len;
                const double uu = ph0 - p0, vv = ph1 - p1;
                const double fu = std::floor(uu), fv = std::floor(vv);
                if (fu == fv)
                    continue;
                const bool up = vv > uu;
                const int lo = static_cast<int>(up ? fu + 1.0 : fv + 1.0), hi = static_cast<int>(up ? fv : fu);
                for (int sft = lo; sft <= hi; ++sft)
                {
                    const double sg = crossing(path, ph0, p0, edgeSpeed, sft, len, uu - sft, vv - sft);
                    const double sign = (e == 0) == up ? 1.0 : -1.0;
                    u.bufP.addStep(static_cast<float>(2.0 * sign * s.w.pulse), static_cast<float>(1.0 - (tA + sg)));
                }
            }
        }
        if (c.subLevel > 0.0)
        {
            const double fu = std::floor(ph0), fv = std::floor(ph1);
            if (fu != fv)
            {
                const bool up = ph1 > ph0;
                const int lo = static_cast<int>(up ? fu + 1.0 : fv + 1.0), hi = static_cast<int>(up ? fv : fu);
                for (int sft = lo; sft <= hi; ++sft)
                {
                    const double sg = crossing(path, ph0, 0.0, 0.0, sft, len, ph0 - sft, ph1 - sft);
                    const double old = u.sub;
                    u.sub = -old;
                    u.bufP.addStep(static_cast<float>(-2.0 * old * c.subLevel), static_cast<float>(1.0 - (tA + sg)));
                }
            }
        }
        u.ph = ph1 - std::floor(ph1);
    }

    // Output stage: the naive components at the new phase enter the lag ring, the sample from kLag samples ago leaves it and
    // receives the corrections of this sample (minBLEP / minBLAMP buffers), the pulse is DC blocked, and above fs/4 the
    // one-harmonic table takes over. The template flags drop the pulse and table work for the common saw / triangle / sine.
    template <bool Pulse, bool Cross>
    CRUCIBLE_ALWAYS_INLINE double finishSample(Uni& u, const Chunk& c, const Weights& w, double aE, double bE,
                                               double ringGain)
    {
        const double p = u.ph;
        const double oNew = c.sawOnly ? w.saw * c.core.sawGain * (2.0 * p - 1.0) : otherV(c.core, w, p);
        double pNew = 0.0, fNew = 0.0;
        if constexpr (Pulse)
            pNew = c.pulseStream ? w.pulse * pulseV(p, aE, bE) + c.subLevel * u.sub : 0.0;
        if constexpr (Cross)
        {
            // The one-harmonic table of the same shape (fundamental of the saw, triangle, sine and pulse mix).
            const double aS = w.sine * 1.0 - w.saw * c.core.sawGain * (2.0 / kPiD);
            const double bC = -w.tri * (8.0 / (kPiD * kPiD));
            const double pa = aE * kTwoPiD, pb = bE * kTwoPiD;
            const double sinC = aS + w.pulse * (2.0 / kPiD) * (std::cos(pa) - std::cos(pb));
            const double cosC = bC + w.pulse * (2.0 / kPiD) * (std::sin(pb) - std::sin(pa));
            fNew = sinC * sinTurns(p) + cosC * cosTurns(p);
        }
        if (u.lagFresh) [[unlikely]]
        {
            u.lagO.fill(oNew);
            u.lagP.fill(pNew);
            u.lagF.fill(fNew);
            u.lagFresh = false;
        }
        const auto lp = static_cast<size_t>(u.lagPos);
        u.lagPos = (u.lagPos + 1) & (kLag - 1);
        const double oOld = u.lagO[lp];
        u.lagO[lp] = oNew;
        double y = oOld + static_cast<double>(u.bufO.tick());
        if constexpr (Pulse)
        {
            const double pOld = u.lagP[lp];
            u.lagP[lp] = pNew;
            u.pulseTail = c.pulseStream ? FineBlep::length + kLag : u.pulseTail - 1;
            const double x = pOld + static_cast<double>(u.bufP.tick());
            u.dcLp += c.dcCoef * (x - u.dcLp);
            y += x - u.dcLp;
        }
        if constexpr (Cross)
        {
            const double fOld = u.lagF[lp];
            u.lagF[lp] = fNew;
            y = y + (fOld - y) * c.cross;
        }
        return y * ringGain * c.gain;
    }

    // Constant shape, steady pitch, no cross modulation: the boundary corrections are the same at every event, so they
    // are worked out once per chunk and the phase moves in a straight line.
    template <bool Pulse, bool Cross>
    void runStatic(Uni& u, const Chunk& c, int n, double gl, double gr, float* left, float* right)
    {
        const Weights& w = c.w1;
        const double inc = c.inc, invInc = 1.0 / inc;
        struct Event
        {
            double p, step, kink; // kink: slope change per sample
            int target;           // 0 other, 1 pulse, 2 sub toggle
        };
        std::array<Event, 8> events{};
        int count = 0;
        for (int k = 0; k < c.numOther; ++k)
        {
            const double p = c.other[static_cast<size_t>(k)];
            const double before = p - kEdgeEps, after = p + kEdgeEps;
            const double pb = before - std::floor(before), pa = after - std::floor(after);
            events[static_cast<size_t>(count++)] = {p, otherV(c.core, w, pa) - otherV(c.core, w, pb),
                                                    (otherD(c.core, w, pa) - otherD(c.core, w, pb)) * inc, 0};
        }
        if (w.pulse != 0.0)
        {
            events[static_cast<size_t>(count++)] = {c.a1, 2.0 * w.pulse, 0.0, 1};
            events[static_cast<size_t>(count++)] = {c.b1, -2.0 * w.pulse, 0.0, 1};
        }
        if (c.subLevel > 0.0)
            events[static_cast<size_t>(count++)] = {0.0, 0.0, 0.0, 2};
        for (int i = 0; i < n; ++i)
        {
            const double ph0 = u.ph;
            const double nw = ph0 + inc;
            for (int k = 0; k < count; ++k)
            {
                const Event& e = events[static_cast<size_t>(k)];
                const double q = e.p > ph0 ? e.p : e.p + 1.0;
                if (q > nw)
                    continue;
                const auto d = static_cast<float>(1.0 - (q - ph0) * invInc);
                if (e.target == 0)
                {
                    if (std::abs(e.step) > 1.0e-12)
                        u.bufO.addStep(static_cast<float>(e.step), d);
                    if (std::abs(e.kink) > 1.0e-12)
                        u.bufO.addKink(static_cast<float>(e.kink), d);
                }
                else if (e.target == 1)
                {
                    u.bufP.addStep(static_cast<float>(e.step), d);
                }
                else
                {
                    const double old = u.sub;
                    u.sub = -old;
                    u.bufP.addStep(static_cast<float>(-2.0 * old * c.subLevel), d);
                }
            }
            u.ph = nw >= 1.0 ? nw - 1.0 : nw;
            const double y = finishSample<Pulse, Cross>(u, c, w, c.a1, c.b1, 1.0);
            left[i] += static_cast<float>(y * gl);
            right[i] += static_cast<float>(y * gr);
        }
    }

    void runVoice(Uni& u, const Chunk& c, int n, double gl, double gr, float* left, float* right)
    {
        const int mode = c.mode;
        const bool sync = mode == modeHardSync || mode == modeSoftSync;
        const bool modulated = mode == modeLinearFm || mode == modeExpFm || mode == modeRing;
        const double incMaster = sync ? c.inc : c.inc * c.ratio;
        const double incSlave = sync ? std::min(c.inc * c.ratio, kMaxStep) : c.inc;
        // Without audio-rate pitch the multiplier is exactly 1: the clamped steps are the same every sample.
        const double slaveStep0 = std::min(incSlave, kMaxStep), masterStep0 = std::min(incMaster, kMaxStep);
        const double pulseMean0 = [&]
        {
            const double width = c.b1 > c.a1 ? c.b1 - c.a1 : 1.0 - c.a1 + c.b1;
            return 2.0 * width - 1.0;
        }();
        if (u.fresh)
        {
            u.dcLp = c.w0.pulse * pulseMean0;
            u.fresh = false;
        }
        if (mode == modeOff && c.w0.saw == c.w1.saw && c.w0.tri == c.w1.tri && c.w0.sine == c.w1.sine &&
            c.w0.pulse == c.w1.pulse && c.a0 == c.a1 && c.b0 == c.b1 && c.gridA == nullptr && c.pitchMod == nullptr)
        {
            const bool pulse = c.pulseStream || u.pulseTail > 0;
            if (pulse && !u.pulseLive)
                u.lagP.fill(0.0);
            if (c.useCross && !u.crossLive)
                u.lagF.fill(0.0);
            u.pulseLive = pulse;
            u.crossLive = c.useCross;
            if (c.useCross)
            {
                if (pulse)
                    runStatic<true, true>(u, c, n, gl, gr, left, right);
                else
                    runStatic<false, true>(u, c, n, gl, gr, left, right);
            }
            else if (pulse)
            {
                runStatic<true, false>(u, c, n, gl, gr, left, right);
            }
            else
            {
                runStatic<false, false>(u, c, n, gl, gr, left, right);
            }
            return;
        }
        const double fmIndex = c.amt * 8.0;
        const double fmOctaves = c.amt * 2.0;
        // Exponential FM keeps the average phase advance (the pitch) by dividing out the mean of 2^(x sin): I0(x ln 2).
        double expNorm = 1.0;
        if (mode == modeExpFm)
        {
            const double h = 0.5 * fmOctaves * 0.6931471805599453;
            double term = 1.0, sum = 1.0;
            for (int k = 1; k < 12; ++k)
            {
                term *= h * h / (static_cast<double>(k) * k);
                sum += term;
            }
            expNorm = 1.0 / sum;
        }
        const double invN = 1.0 / static_cast<double>(n);
        const bool steady = c.w0.saw == c.w1.saw && c.w0.tri == c.w1.tri && c.w0.sine == c.w1.sine &&
                            c.w0.pulse == c.w1.pulse && c.a0 == c.a1 && c.b0 == c.b1 && c.gridA == nullptr;
        if (c.pitchMod != nullptr && u.pmFresh)
        {
            u.pmPrev = static_cast<double>(c.pitchMod[0]);
            u.pmFresh = false;
        }
        if (!u.pulseLive)
            u.lagP.fill(0.0);
        if (c.useCross && !u.crossLive)
            u.lagF.fill(0.0);
        u.pulseLive = true;
        u.crossLive = c.useCross;

        for (int i = 0; i < n; ++i)
        {
            const double t0 = i * invN, t1 = (i + 1) * invN;
            SampleCtx s;
            if (steady)
            {
                s.w = c.w1;
                s.aS = s.aE = c.a1;
                s.bS = s.bE = c.b1;
            }
            else if (c.gridA != nullptr)
            {
                s.w = lerp(c.w0, c.w1, t1);
                s.gridA = &c.gridA[i];
                s.gridB = &c.gridB[i];
                s.aS = (*s.gridA)[0];
                s.aE = (*s.gridA)[4];
                s.bS = (*s.gridB)[0];
                s.bE = (*s.gridB)[4];
            }
            else
            {
                s.w = lerp(c.w0, c.w1, t1);
                s.aS = c.a0 + (c.a1 - c.a0) * t0;
                s.aE = c.a0 + (c.a1 - c.a0) * t1;
                s.bS = c.b0 + (c.b1 - c.b0) * t0;
                s.bE = c.b0 + (c.b1 - c.b0) * t1;
            }
            // Audio-rate pitch: the multiplier of the phase advance at the end of the sample.
            double pmNow = 0.0;
            double pmFactorEnd = 1.0;
            if (c.pitchMod != nullptr)
            {
                pmNow = static_cast<double>(c.pitchMod[i]);
                pmFactorEnd = std::exp2(pmNow / 12.0);
            }

            double ringGain = 1.0;
            if (sync)
            {
                // Both phases advance with the pitch multiplier, clamped like the unmodulated steps; the crossing time uses
                // the modulated master step, so it always lies inside the sample.
                const double slaveStep = c.pitchMod == nullptr ? slaveStep0 : std::min(incSlave * pmFactorEnd, kMaxStep);
                const double masterStep = c.pitchMod == nullptr ? masterStep0 : std::min(incMaster * pmFactorEnd, kMaxStep);
                double dphi = u.dir * slaveStep;
                const double mold = u.mph;
                const double mnew = mold + masterStep;
                if (mnew >= 1.0)
                {
                    const double a = std::clamp((1.0 - mold) / masterStep, 0.0, 1.0);
                    u.mph = mnew - 1.0;
                    segmentStraight(u, c, s, dphi, 0.0, a);
                    if (mode == modeHardSync)
                    {
                        const double pb = u.ph;
                        const double step = otherV(c.core, s.w, 0.0) - otherV(c.core, s.w, pb);
                        const double kink = (otherD(c.core, s.w, 0.0) - otherD(c.core, s.w, pb)) * dphi;
                        const auto d = static_cast<float>(1.0 - a);
                        if (std::abs(step) > 1.0e-12)
                            u.bufO.addStep(static_cast<float>(step), d);
                        if (std::abs(kink) > 1.0e-12)
                            u.bufO.addKink(static_cast<float>(kink), d);
                        if (s.w.pulse != 0.0)
                        {
                            const double ea = s.aS + (s.aE - s.aS) * a, eb = s.bS + (s.bE - s.bS) * a;
                            const double ps = (pulseV(0.0, ea, eb) - pulseV(pb, ea, eb)) * s.w.pulse;
                            if (ps != 0.0)
                                u.bufP.addStep(static_cast<float>(ps), d);
                        }
                        u.ph = 0.0;
                    }
                    else
                    {
                        const double kink = -2.0 * otherD(c.core, s.w, u.ph) * dphi;
                        if (std::abs(kink) > 1.0e-12)
                            u.bufO.addKink(static_cast<float>(kink), static_cast<float>(1.0 - a));
                        u.dir = -u.dir;
                        dphi = u.dir * slaveStep;
                    }
                    segmentStraight(u, c, s, dphi, a, 1.0);
                }
                else
                {
                    u.mph = mnew;
                    segmentStraight(u, c, s, dphi, 0.0, 1.0);
                }
            }
            else
            {
                Path path(c.inc);
                if (modulated || c.pitchMod != nullptr)
                {
                    // The pitch multiplier at the five instants of the sample (linear between the sample values).
                    std::array<double, 5> pf{1.0, 1.0, 1.0, 1.0, 1.0};
                    if (c.pitchMod != nullptr)
                        for (size_t q = 0; q < 5; ++q)
                            pf[q] = std::exp2((u.pmPrev + (pmNow - u.pmPrev) * 0.25 * static_cast<double>(q)) / 12.0);
                    if (modulated)
                    {
                        // Audio-rate FM amount: the amount of this sample (the other samples' values are not needed).
                        double amt = c.amt, index = fmIndex, octaves = fmOctaves, norm = expNorm;
                        if (c.fmMod != nullptr)
                        {
                            amt = std::clamp(c.amt + static_cast<double>(c.fmMod[i]), 0.0, 1.0);
                            index = amt * 8.0;
                            octaves = amt * 2.0;
                            if (mode == modeExpFm)
                            {
                                const double h = 0.5 * octaves * 0.6931471805599453;
                                double term = 1.0, sum = 1.0;
                                for (int k = 1; k < 12; ++k)
                                {
                                    term *= h * h / (static_cast<double>(k) * k);
                                    sum += term;
                                }
                                norm = 1.0 / sum;
                            }
                        }
                        // The modulator moves with the pitch like the carrier: its step is scaled by the mean multiplier of
                        // the sample (1 without audio-rate pitch).
                        const double masterStep = incMaster * (0.5 * (pf[0] + pf[4]));
                        const double next = u.mph + masterStep;
                        if (mode == modeRing)
                        {
                            ringGain = (1.0 - amt) + amt * sinTurns(next);
                            if (c.pitchMod != nullptr)
                            {
                                std::array<double, 5> f;
                                for (size_t q = 0; q < 5; ++q)
                                    f[q] = std::clamp(c.inc * pf[q], -kMaxStep, kMaxStep);
                                path = pathFrom(f);
                            }
                        }
                        else
                        {
                            // Instantaneous phase advance at five instants of the sample; the path through them is the
                            // integral of their quartic interpolant.
                            std::array<double, 5> f;
                            for (size_t q = 0; q < 5; ++q)
                            {
                                const double mod = sinTurns(u.mph + masterStep * 0.25 * static_cast<double>(q));
                                const double v = mode == modeLinearFm ? c.inc + index * incMaster * mod
                                                                      : c.inc * norm * std::exp2(octaves * mod);
                                f[q] = std::clamp(v * pf[q], -kMaxStep, kMaxStep);
                            }
                            path = pathFrom(f);
                        }
                        u.mph = next - std::floor(next);
                    }
                    else
                    {
                        std::array<double, 5> f;
                        for (size_t q = 0; q < 5; ++q)
                            f[q] = std::clamp(c.inc * pf[q], -kMaxStep, kMaxStep);
                        path = pathFrom(f);
                    }
                }
                segmentMonotone(u, c, s, path, 0.0, 1.0);
            }
            if (c.pitchMod != nullptr)
                u.pmPrev = pmNow;

            const double y = c.useCross ? finishSample<true, true>(u, c, s.w, s.aE, s.bE, ringGain)
                                        : finishSample<true, false>(u, c, s.w, s.aE, s.bE, ringGain);
            left[i] += static_cast<float>(y * gl);
            right[i] += static_cast<float>(y * gr);
        }
    }

    SlotParams sp;
    int wave = 0, shape = 0, pw = 0, character = 0, sub = 0, comp = 0, xmode = 0, xratio = 0, xamt = 0, unison = 0;
    int detune = 0, spread = 0, drift = 0, tau = 0, tolerance = 0;
    double cardAmount = 0.0; // amount of the voice card's tolerances, 0 without a card
    std::unique_ptr<AudioScratch> scratch;
    double sr = 48000.0;
    double dcRate = 0.0, dcCoefCached = 0.0;
    bool first = true;
    Rng rng;
    std::array<Uni, kMaxUnison> uni;
    std::array<Weights, kMaxUnison> prevW{};
    std::array<double, kMaxUnison> prevA{}, prevB{};
};
} // namespace

const char* analogOscillatorName()
{
    return "Analog (VA)";
}

ModuleDescriptor analogOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.analog";
    d.name = analogOscillatorName();
    d.kind = ModuleKind::Oscillator;
    d.category = "Analog";
    d.cpu = CpuClass::Light;
    d.description = "Virtual analog oscillator: saw, pulse with PWM, triangle, sine, saw + pulse mix and a shape morph, "
                    "minBLEP / minBLAMP band-limited at exact sub-sample positions, with hard and soft sync, "
                    "through-zero and exponential FM, ring modulation, unison, drift and voice-card tolerance, and "
                    "saw-core, triangle-core and DCO characters.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto label = slotLabel(slot);
        const auto wavePage = addSlotPage(d, slot, "analog", "Analog", "LCD shows the oscillator waveform.");
        const auto xmodPage =
            addSlotPage(d, slot, "analog_x", "Analog Sync / Unison", "LCD shows the sync and unison settings.");
        const auto driftPage = addSlotPage(d, slot, "analog_drift", "Analog Drift", "LCD shows the drift and tolerance.");
        const auto add = [&d](ParamDef p) { d.params.push_back(std::move(p)); };
        add(params::choice(slotId(slot, "an_wave"), label + " Analog Wave", wavePage,
                           {"Saw", "Pulse", "Triangle", "Sine", "Saw + Pulse", "Morph"}, 0));
        add(voiceMod(params::floating(slotId(slot, "an_shape"), label + " Analog Shape", wavePage, 0.0f, 1.0f, 0.0f)));
        add(voiceMod(params::floating(slotId(slot, "an_pw"), label + " Analog Pulse Width", wavePage, 0.02f, 0.98f, 0.5f)));
        add(params::choice(slotId(slot, "an_char"), label + " Analog Character", wavePage,
                           {"Ideal", "Saw-core VCO", "Triangle-core VCO", "DCO"}, 0));
        add(voiceMod(params::floating(slotId(slot, "an_sub"), label + " Analog Divider Sub", wavePage, 0.0f, 1.0f, 0.0f)));
        add(params::toggle(slotId(slot, "an_dco_comp"), label + " DCO Level Compensation", wavePage, false));
        add(params::choice(slotId(slot, "an_xmod_mode"), label + " Analog Cross Mod", xmodPage,
                           {"Off", "Hard Sync", "Soft Sync", "Linear FM (TZ)", "Exp FM", "Ring"}, 0));
        add(voiceMod(params::floating(slotId(slot, "an_xmod_ratio"), label + " Analog Mod Ratio", xmodPage, 0.25f, 16.0f,
                                      2.0f, {}, 0.4f)));
        add(voiceMod(
            params::floating(slotId(slot, "an_xmod_amt"), label + " Analog Mod Amount", xmodPage, 0.0f, 1.0f, 0.5f)));
        add(params::integer(slotId(slot, "an_uni"), label + " Analog Unison", xmodPage, 1, 8, 1));
        add(voiceMod(
            params::floating(slotId(slot, "an_detune"), label + " Analog Detune", xmodPage, 0.0f, 100.0f, 12.0f, "ct")));
        add(voiceMod(
            params::floating(slotId(slot, "an_spread"), label + " Analog Stereo Spread", xmodPage, 0.0f, 1.0f, 0.5f)));
        add(voiceMod(params::floating(slotId(slot, "an_drift"), label + " Analog Drift", driftPage, 0.0f, 1.0f, 0.0f)));
        add(params::floating(slotId(slot, "an_drift_tau"), label + " Analog Drift Time", driftPage, 0.5f, 5.0f, 2.0f, "s",
                             0.5f));
        add(params::floating(slotId(slot, "an_tol"), label + " Analog Tolerance", driftPage, 0.0f, 1.0f, 0.0f));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

} // namespace crucible::modules
