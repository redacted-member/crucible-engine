// SPDX-License-Identifier: MIT
#pragma once

// The analog filter interface (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 2.6): nonlinear zero-delay-feedback
// filter models behind one block interface. A model owns its state for up to four lanes, so one object can run four
// voices in lock step (processQuad) with the same arithmetic as four mono objects but vectorised.
//
// Threading: prepare() and the constructor allocate and belong to the control thread (the first prepare() of a model
// and tier also measures its pitch-correction table, a few tens of milliseconds). Everything else is allocation free
// and lock free and may run on the audio thread, with denormals flushed as in the rest of the engine. An object is
// used through one entry point: process() for lane 0, or processQuad() for all four lanes (an object prepared with
// one lane runs lane 0 only).
//
// Tiers (Quality): Draft runs at the host rate, Standard at twice the rate, High at four times the rate (section 2.7).
// Draft and Standard solve each sample by a linearisation at the previous state followed by one Newton step; High
// iterates Newton's method to 1e-6, at most four times.

#include <crucible/module/Module.h>

#include <complex>
#include <cstdint>
#include <memory>

namespace crucible
{

// How the nonlinear solve of one sample is done.
enum class SolverTier
{
    Draft,    // 1x; linearised at the previous state plus one Newton step (a linearised solve alone is unstable)
    Standard, // 2x; linearised at the previous state plus one Newton step
    High      // 4x; Newton to 1e-6, at most four iterations
};

inline SolverTier solverTierFor(Quality q)
{
    return q == Quality::Draft ? SolverTier::Draft : (q == Quality::Standard ? SolverTier::Standard : SolverTier::High);
}

// log2 of the oversampling factor of a tier: 1x, 2x, 4x.
inline int oversamplingStagesFor(Quality q)
{
    return q == Quality::Draft ? 0 : (q == Quality::Standard ? 1 : 2);
}

class AnalogFilter
{
public:
    static constexpr int kMaxLanes = 4;

    virtual ~AnalogFilter() = default;
    virtual std::unique_ptr<AnalogFilter> clone() const = 0;

    // Control thread, allocates. lanes: 1 (mono) or 4 (processQuad). maxBlock: the longest block passed to process().
    virtual void prepare(double sampleRate, Quality quality, int lanes = 1, int maxBlock = 512) = 0;
    virtual void reset() = 0;
    // Returns one lane to the state reset() gives it while the other lanes keep running: a voice that starts on a quad
    // shared with sounding voices. Controls of the lane are set again afterwards (setLaneControls). A one-lane model
    // resets entirely.
    virtual void resetLane(int lane)
    {
        if (lanes() == 1 && lane == 0)
            reset();
    }

    // Seed of the part-to-part tolerances and the start-up noise; lane l draws from (seed, l).
    virtual void setSeed(std::uint32_t seed) = 0;
    // Draws of one lane from (seed, draw) instead of (seed, lane). draw 0 gives the lane the values a one-lane model has
    // under `seed`; setSeed() restores draw = lane for all lanes.
    virtual void setLaneSeed(int lane, std::uint32_t seed, int draw)
    {
        (void)lane;
        (void)seed;
        (void)draw;
    }
    // The quoted noise injection (about -120 dB) that starts self-oscillation from silence. Off by default.
    virtual void setStartupNoise(bool on) = 0;
    // The optional high-pass in front of the model, in four steps (0 = off, 1 to 3 = increasingly high corners:
    // 60 / 180 / 540 Hz, one passive RC section); models without one ignore it. The OTA cascade, the Sallen-Key
    // low-pass, the OTA SVF and the OTA 4-pole have it; the transistor and diode ladders, the Sallen-Key high-pass
    // and the passive high-pass do not.
    virtual void setHighpassStep(int step) { (void)step; }

    // cutoffHz: the frequency at which the filter self-oscillates at full resonance (the dial frequency); for the
    // models that never oscillate, the corner or centre frequency of the response.
    // resonance, drive, character, tolerance: 0..1. character blends in gain compensation (0 = the circuit's own bass
    // loss, 1 = unity DC gain) for the ladders and OTA filters; the models without bass loss use it for their own voicing:
    // the Sallen-Key's diode clipper hardness, the OTA SVF's low-pass / notch / high-pass morph (0 / 0.5 / 1), nothing for
    // the passive high-pass. tolerance scales the seeded component spread (cutoff +-4 %, feedback +-3 %, each
    // stage's capacitor +-1 % at 1).
    virtual void setControls(float cutoffHz, float resonance, float drive, float character, float tolerance) = 0;
    virtual void setLaneControls(int lane, float cutoffHz, float resonance, float drive, float character,
                                 float tolerance) = 0;

    // In place. cutoffOctMod: optional per-sample cutoff modulation in octaves (n values), applied on top of
    // the cutoff and followed with a short ramp (16-sample control rate).
    virtual void process(float* io, int n, const float* cutoffOctMod = nullptr) = 0;
    // Four lanes at once, in place; io[l] is lane l. cutoffOctMod may be null or hold null entries.
    virtual void processQuad(float* const io[4], int n, const float* const cutoffOctMod[4] = nullptr) = 0;

    virtual int lanes() const = 0;
    // Latency of the oversampler in host-rate samples (0 at Draft).
    virtual float latency() const = 0;
    // The loop gain k a resonance setting maps to; the model self-oscillates when the linear loop gain reaches
    // selfOscillationGain() (infinity for the models that cannot). For the Sallen-Key k is the gain Ka of the feedback
    // stage (1 .. 3.2, oscillation from 3), for the OTA SVF the quality factor Q (0.5 .. 20), for the passive high-pass
    // the share of the second section.
    virtual float feedbackForResonance(float resonance) const = 0;
    virtual float selfOscillationGain() const = 0;
    // Linear gain at zero frequency for small signals (the bass loss of the circuit and what character gives back).
    virtual float dcGain(float resonance, float character) const = 0;
    // Small-signal magnitude response in dB at n frequencies (Hz) for the controls given last to setControls() (lane 0)
    // and setHighpassStep(), with nominal parts (tolerance 0) and the gain of the drive stage. Pure arithmetic on the
    // settings: no audio state is read and nothing is allocated, so a thread that owns the object may call it while no
    // other thread changes the controls. A loop that would self-oscillate is shown at 99.5 % of its critical gain.
    virtual void responseDb(const float* freqHz, int n, float* outDb) const = 0;
};

// The models. Each call allocates a prepared-for-48-kHz-Standard single-lane instance; call prepare() to change that.
std::unique_ptr<AnalogFilter> makeTransistorLadder();
std::unique_ptr<AnalogFilter> makeOtaCascade();
std::unique_ptr<AnalogFilter> makeDiodeLadder();
std::unique_ptr<AnalogFilter> makeSallenKeyLowpass();
std::unique_ptr<AnalogFilter> makeSallenKeyHighpass();
std::unique_ptr<AnalogFilter> makeSvfOta();
std::unique_ptr<AnalogFilter> makeOta4Pole();
std::unique_ptr<AnalogFilter> makePassiveHp();

} // namespace crucible
