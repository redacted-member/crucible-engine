// SPDX-License-Identifier: MIT
#pragma once

// The analog envelope (docs/plans/ENGINE-COMMUNITY-PLAN.md, section 2.9). One class wraps the engine's digital ADSR
// (Dsp.h) and an RC model of the circuit that analog envelope generators are built on, and adds the retrigger modes and
// the velocity and key time scaling that apply to both.
//
// Digital: the legacy Adsr, untouched. With the shape at its defaults (From Level, no scaling, no tolerance) the output is
// bit-identical to a bare Adsr.
//
// Analog RC: a capacitor charged and discharged through resistors.
//   * Attack charges toward 1 + overshoot (the supply is above the peak the comparator trips at) and the level crosses 1
//     exactly at the attack time when it starts from zero: tau = attack / ln((1 + overshoot) / overshoot). Started from a
//     higher level the time constant is the same, so the peak comes sooner, as in the circuit.
//   * The attack curve (AdsrParams::attackCurve) bends the charge: the level is expm1(k u) / expm1(k) of the RC state u
//     (k = 8 curve), so a negative curve starts faster, a positive one slower, and the peak is still at the attack time.
//     Curve 0 is the plain recursion, unchanged.
//   * Decay and release are exponential. The time set is the time to fall within 1 % (-40 dB) of the target, so
//     tau = time / ln(100).
//   * Sustain is the decay's target. The decay and the sustain stage are one recursion, so a sustain change (or a sustain
//     modulated while the key is held) moves the level through the same RC with the decay's time constant.
//   * A release ends when the level is below 2e-5 (-94 dB); the step to silence is smaller than that.
//
// Retrigger: From Level starts the attack from where the level is (continuous), Reset takes the level to zero through a
// 0.5 ms smooth fade so the old level is not cut off, Legato ignores a note-on while the key is still held (the
// envelope keeps running), ADR has no sustain: the decay runs to zero while the key is held and a note-off releases
// whatever level the envelope is at.
//
// All of this runs on the audio thread without allocation; prepare() only stores the rate.

#include <crucible/Dsp.h>

#include <algorithm>
#include <cmath>

namespace crucible
{

enum class EnvMode : int
{
    Digital = 0,
    AnalogRc = 1
};

enum class Retrigger : int
{
    FromLevel = 0,
    Reset = 1,
    Legato = 2,
    Adr = 3
};

struct EnvShape
{
    EnvMode mode = EnvMode::Digital;
    Retrigger retrigger = Retrigger::FromLevel;
    float overshoot = 0.35f;   // analog attack target is 1 + overshoot
    float velocityTime = 0.0f; // -1..1: full velocity halves the times at +1, doubles them at -1
    float keyTime = 0.0f;      // -1..1: an octave up halves the times at +1
};

class AnalogEnvelope
{
public:
    static constexpr float kEnd = 2.0e-5f;
    static constexpr double kSettle = 4.605170185988092; // ln(100)
    static constexpr double kDeclickSeconds = 0.0005;

    void prepare(double rate)
    {
        fs = rate;
        digital.prepare(rate);
        declickLength = std::max(1, static_cast<int>(kDeclickSeconds * rate));
    }

    // timeScale and sustainOffset carry the voice card's tolerance (1 and 0: none). Call before every chunk; the mode
    // takes effect at the next note-on.
    void setParams(const AdsrParams& p, const EnvShape& s, float timeScale = 1.0f, float sustainOffset = 0.0f)
    {
        raw = p;
        shape = s;
        tolerance = timeScale;
        sustainShift = sustainOffset;
        if (idleNow())
            active = s.mode;
        apply();
    }

    void noteOn(float velocity, int note)
    {
        noteScale = 1.0f;
        if (shape.velocityTime != 0.0f || shape.keyTime != 0.0f)
        {
            const float octaves = -shape.velocityTime * (2.0f * std::clamp(velocity, 0.0f, 1.0f) - 1.0f) -
                                  shape.keyTime * (static_cast<float>(note) - 60.0f) / 12.0f;
            noteScale = std::exp2(std::clamp(octaves, -4.0f, 4.0f));
        }
        const bool running = !idleNow();
        const bool changed = running && active != shape.mode;
        if (running && !changed && shape.retrigger == Retrigger::Legato && !releasingNow())
        {
            apply();
            return;
        }
        if (shape.retrigger == Retrigger::Reset || changed)
        {
            declickFrom = out;
            declickLeft = out != 0.0f ? declickLength : 0;
            digital.reset();
            level = 0.0;
            stage = Stage::Idle;
        }
        active = shape.mode;
        apply();
        if (active == EnvMode::Digital)
            digital.noteOn();
        else
        {
            stage = Stage::Attack;
            holdCount = 0;
            if (attackK != 0.0)
                chargeU = level >= 1.0 ? 1.0 : std::log1p(level * attackDen) / attackK; // inverse of the curve
        }
    }

    void noteOff()
    {
        if (active == EnvMode::Digital)
            digital.noteOff();
        else if (stage != Stage::Idle)
            stage = Stage::Release;
    }

    void reset()
    {
        digital.reset();
        stage = Stage::Idle;
        level = 0.0;
        out = 0.0f;
        declickLeft = 0;
    }

    bool isIdle() const { return idleNow() && declickLeft == 0; }
    bool isReleasing() const { return releasingNow(); }
    bool isSustaining() const { return active == EnvMode::Digital ? digital.isSustaining() : stage == Stage::Sustain; }
    float value() const { return out; }

    float tick()
    {
        float v;
        if (active == EnvMode::Digital)
        {
            v = digital.tick();
            if (shape.retrigger == Retrigger::Adr && digital.isSustaining())
            {
                digital.reset();
                v = 0.0f;
            }
        }
        else
        {
            v = static_cast<float>(rcTick());
        }
        if (declickLeft > 0)
        {
            const float x = static_cast<float>(declickLength - declickLeft) / static_cast<float>(declickLength);
            // Quintic smoothstep, one to zero: slope and curvature vanish at both ends, so the fade adds no corner.
            v += declickFrom * (1.0f - x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f));
            --declickLeft;
        }
        out = v;
        return v;
    }

    float advance(int numSamples)
    {
        float v = out;
        for (int i = 0; i < numSamples; ++i)
            v = tick();
        return v;
    }

private:
    enum class Stage
    {
        Idle,
        Attack,
        Hold,
        Decay, // also the sustain stage's recursion until the level has settled
        Sustain,
        Release
    };

    bool idleNow() const { return active == EnvMode::Digital ? digital.isIdle() : stage == Stage::Idle; }
    bool releasingNow() const { return active == EnvMode::Digital ? digital.isReleasing() : stage == Stage::Release; }

    void apply()
    {
        const float k = noteScale * tolerance;
        AdsrParams a = raw;
        a.attack *= k;
        a.decay *= k;
        a.release *= k;
        a.sustain = std::clamp(a.sustain + sustainShift, 0.0f, 1.0f);
        if (shape.retrigger == Retrigger::Adr)
            a.sustain = 0.0f;
        if (active == EnvMode::Digital)
        {
            digital.setParams(a);
            return;
        }
        const double rate = fs;
        const double overshoot = std::clamp(static_cast<double>(shape.overshoot), 0.02, 2.0);
        const double attack = std::max(static_cast<double>(a.attack), 0.0005);
        const double decay = std::max(static_cast<double>(a.decay), 0.0005);
        const double release = std::max(static_cast<double>(a.release), 0.0005);
        target = 1.0 + overshoot;
        const double kc = static_cast<double>(std::clamp(raw.attackCurve, -1.0f, 1.0f)) * 8.0;
        attackK = std::abs(kc) < 1.0e-3 ? 0.0 : kc;
        attackDen = attackK == 0.0 ? 1.0 : std::expm1(attackK);
        if (stage == Stage::Attack && attackK != 0.0)
            chargeU = level >= 1.0 ? 1.0 : std::log1p(level * attackDen) / attackK; // keep the level, re-aim the charge
        attackCoef = std::exp(-std::log((1.0 + overshoot) / overshoot) / (attack * rate));
        decayCoef = std::exp(-kSettle / (decay * rate));
        releaseCoef = std::exp(-kSettle / (release * rate));
        sustainLevel = shape.retrigger == Retrigger::Adr ? 0.0 : static_cast<double>(a.sustain);
        holdSamples = static_cast<int>(std::max(a.hold, 0.0f) * rate);
    }

    double rcTick()
    {
        switch (stage)
        {
        case Stage::Idle:
            break;
        case Stage::Attack:
            if (attackK == 0.0)
            {
                level = target - (target - level) * attackCoef;
            }
            else
            {
                // The RC charge runs on its own state and the attack curve bends what it drives, so the peak is still
                // reached at the attack time (the charge crosses 1 there) and a start from a level continues smoothly.
                chargeU = target - (target - chargeU) * attackCoef;
                level = chargeU >= 1.0 ? 1.0 : std::expm1(attackK * chargeU) / attackDen;
            }
            if (level >= 1.0)
            {
                level = 1.0;
                holdCount = holdSamples;
                stage = holdSamples > 0 ? Stage::Hold : Stage::Decay;
            }
            break;
        case Stage::Hold:
            level = 1.0;
            if (--holdCount <= 0)
                stage = Stage::Decay;
            break;
        case Stage::Decay:
        case Stage::Sustain:
            level = sustainLevel + (level - sustainLevel) * decayCoef;
            if (sustainLevel == 0.0 && level < static_cast<double>(kEnd))
            {
                level = 0.0;
                if (shape.retrigger == Retrigger::Adr)
                    stage = Stage::Idle;
            }
            else if (stage == Stage::Decay && std::abs(level - sustainLevel) <= 0.01 * (1.0 - sustainLevel) + 1.0e-6)
            {
                stage = Stage::Sustain;
            }
            break;
        case Stage::Release:
            level *= releaseCoef;
            if (level < static_cast<double>(kEnd))
            {
                level = 0.0;
                stage = Stage::Idle;
            }
            break;
        }
        return level;
    }

    Adsr digital;
    EnvMode active = EnvMode::Digital;
    EnvShape shape;
    AdsrParams raw;
    float tolerance = 1.0f;
    float sustainShift = 0.0f;
    float noteScale = 1.0f;
    double fs = 44100.0;

    Stage stage = Stage::Idle;
    double level = 0.0;
    double target = 1.35;
    double attackCoef = 0.999;
    double attackK = 0.0;   // 8 * attack curve, 0 when the curve is off
    double attackDen = 1.0; // expm1(attackK)
    double chargeU = 0.0;   // the RC charge while a curved attack runs
    double decayCoef = 0.999;
    double releaseCoef = 0.999;
    double sustainLevel = 0.8;
    int holdSamples = 0;
    int holdCount = 0;

    float out = 0.0f;
    float declickFrom = 0.0f;
    int declickLeft = 0;
    int declickLength = 24;
};

} // namespace crucible
