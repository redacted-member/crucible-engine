// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/DelayLine.h>
#include <crucible/FxExtra.h>
#include <crucible/HalfBand.h>
#include <crucible/Oversampler.h>
#include <crucible/fx/Lookahead.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace crucible
{

// The rack-style built-in effects (fixed parameter pages "fx_<id>_..." and a slot "fx_pos_<id>"). The first-generation
// drive, chorus, flanger, phaser, delay, reverb, compressor, EQ, parametric EQ, dimension, gate and convolver were removed
// before v1; the parameter order was renumbered once for that, which is allowed before
// v1 and never after.
enum class FxId
{
    Limiter,
    Crusher,
    Width,
    Filter,
    Multiband,
    TranceGate,
    Tremolo,
    RingShift,
    Count
};

constexpr int kNumFx = static_cast<int>(FxId::Count);
// Effects that exist only as modules: the vocoder, then the dynamics of FX3 (fx.comp_clean,
// fx.comp_vca, fx.gate2), FX4's fx.maximizer, fx.clipper and util.meter, then the pro EQ of FX5 (fx.eq_pro) FX6's
// fx.reverb_algo, FX7's fx.convolution and FX10's fx.delay_pro, then FX13's fx.transient and fx.deesser and FX11's
// fx.ensemble, fx.phaser2 and fx.flanger2, then FX13's fx.exciter, then FX13's fx.saturator, then fx.shaper, then
// fx.multiband_drive, then fx.cabinet. The effect chain has kNumModuleFx slots.
constexpr int kNumModuleOnlyFx = 21;
constexpr int kNumModuleFx = kNumFx + kNumModuleOnlyFx;

struct FxLimiterParams
{
    bool on = false;
    float mix = 1.0f;
    float ceilingDb = -0.3f;
    float releaseMs = 80.0f;
    float gainDb = 0.0f;
    float lookaheadMs = 1.5f;
};

struct FxCrusherParams
{
    bool on = false;
    float mix = 1.0f;
    float bits = 8.0f;
    float rateHz = 12000.0f;
    float jitter = 0.0f;
    float dither = 0.0f;
    // Note tracking (opt-in): with track set and noteHz > 0 the rate is noteHz x ratio x 2^(fineCents / 1200) and
    // rateHz is ignored; the rate glides (about 10 ms) when the note changes. Off: rateHz, exactly as before.
    bool track = false;
    float noteHz = 0.0f;
    float ratio = 1.0f;
    float fineCents = 0.0f;
};

struct FxWidthParams
{
    bool on = false;
    float mix = 1.0f;
    float width = 1.0f;
    float bassMonoHz = 20.0f;
    float pan = 0.0f;
};

class FxLimiter
{
public:
    // Called by the rack while the effect is bypassed so the next activation starts at the target mix.
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxLimiterParams& p);

private:
    static constexpr int maxLook = 1024;
    // The audio delay; a lookahead change crossfades between two taps instead of clearing it (BFX5).
    fx::Lookahead audio;
    std::array<float, maxLook> required{};
    std::array<float, maxLook> minimum{};
    int pos = 0;
    int lastLook = -1;
    double avgSum = 0.0;
    float gain = 1.0f;
    float mixSmooth = 1.0f;
    bool primed = false;
    double sr = 48000.0;
};

class FxCrusher
{
public:
    // Called by the rack while the effect is bypassed so the next activation starts at the target mix.
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxCrusherParams& p);

private:
    std::array<float, 2> held{};
    float acc = 1.0f;
    float mixSmooth = 1.0f;
    float stepSmooth = 0.25f;
    bool tracking = false;
    bool primed = false;
    std::uint32_t rngState = 0x2468ace1U;
    double sr = 48000.0;
};

class FxWidth
{
public:
    // Called by the rack while the effect is bypassed so the next activation starts at the target mix.
    void deactivate() { primed = false; }
    void prepare(double sampleRate);
    void reset();
    void process(float* l, float* r, int n, const FxWidthParams& p);

private:
    float sideLp = 0.0f;
    float mixSmooth = 1.0f;
    bool primed = false;
    double sr = 48000.0;
};

} // namespace crucible
