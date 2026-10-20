// SPDX-License-Identifier: MIT
// fx.maximizer: lookahead true-peak limiter and maximiser on include/crucible/fx/Maximizer.h, with a loudness readout.
#include "FxModuleSupport.h"

#include <crucible/fx/Loudness.h>
#include <crucible/fx/Maximizer.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/MeterReadout.h>

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public GainReductionMeter, public LatencyReporter, public MeterReadout
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : gain(localIndex(d, "fx_mx_gain")), ceiling(localIndex(d, "fx_mx_ceiling")),
          release(localIndex(d, "fx_mx_release")), lookahead(localIndex(d, "fx_mx_lookahead")),
          relMode(localIndex(d, "fx_mx_relmode")), hold(localIndex(d, "fx_mx_hold")),
          autoHold(localIndex(d, "fx_mx_autohold")), link(localIndex(d, "fx_mx_link")),
          stereo(localIndex(d, "fx_mx_stereo")), clip(localIndex(d, "fx_mx_clip")), knee(localIndex(d, "fx_mx_knee")),
          oversample(localIndex(d, "fx_mx_os")), dither(localIndex(d, "fx_mx_dither")), bits(localIndex(d, "fx_mx_bits")),
          resetMeter(localIndex(d, "fx_mx_mreset"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        mx.prepare(c.sampleRate, c.maxBlockSize, c.quality, c.oversampling, c.seed);
        meter.prepare(c.sampleRate, 2, false);
        dirty = false;
    }
    void reset() override
    {
        mx.reset();
        meter.reset();
        dirty = false;
    }
    // Switched off: forget the lookahead buffer and the gain so that switching on replays nothing.
    void deactivate() override
    {
        if (dirty)
        {
            mx.reset();
            dirty = false;
        }
    }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        using namespace crucible::fx;
        MaximizerSettings s;
        LimiterSettings& ls = s.limiter;
        ls.gainDb = in.value(gain);
        ls.ceilingDb = in.value(ceiling);
        ls.releaseMs = in.value(release);
        ls.lookaheadMs = in.value(lookahead);
        ls.releaseMode = static_cast<ReleaseMode>(std::clamp(in.choice(relMode), 0, 2));
        ls.holdMs = in.value(hold);
        ls.autoHold = in.on(autoHold);
        ls.link = in.value(link) * 0.01f;
        ls.sideBias = in.choice(stereo) == 1;
        ls.clipShare = in.value(clip) * 0.01f;
        ls.clipSoftness = in.value(knee) * 0.01f;
        s.oversample = in.choice(oversample) == 1;
        s.dither = static_cast<DitherMode>(std::clamp(in.choice(dither), 0, 2));
        s.ditherBits = 16 + 4 * std::clamp(in.choice(bits), 0, 2);
        if (in.on(resetMeter) && !resetHeld)
            meter.resetStatistics();
        resetHeld = in.on(resetMeter);
        dirty = true;
        mx.process(l, r, n, s);
        meter.process(l, r, n);
    }

    int meterBands() const override { return 1; }
    float gainReductionDb(int) const override { return mx.gainReductionDb(); }
    int latencySamples() const override { return mx.latencySamples(); }
    bool takeLatencyChanged() override { return mx.takeLatencyChanged(); }
    void loudness(float* out) const override
    {
        out[0] = meter.momentaryLufs();
        out[1] = meter.shortTermLufs();
        out[2] = meter.integratedLufs();
        out[3] = meter.loudnessRange();
        out[4] = meter.samplePeakDb();
        out[5] = meter.maxMomentaryLufs();
        out[6] = meter.rangeLowLufs();
        out[7] = meter.rangeHighLufs();
    }

private:
    const int gain, ceiling, release, lookahead, relMode, hold, autoHold, link, stereo, clip, knee, oversample, dither, bits,
        resetMeter;
    crucible::fx::Maximizer mx;
    crucible::fx::LoudnessMeter meter;
    bool dirty = false, resetHeld = false;
};
} // namespace

ModuleDescriptor maximizerEffect()
{
    auto d = effectDescriptor(
        "fx.maximizer", "Maximizer", "Dynamics", CpuClass::Medium,
        "Lookahead true-peak limiter and maximiser: output ceiling in dBTP that is never exceeded (4x "
        "interpolated sidechain, or the whole audio path at 4x), lookahead 0.5 to 10 ms (reported as latency), "
        "hold adapted to the period of the bass, fixed, dual-stage or crest-adaptive release, stereo link and a "
        "mid / side bias that turns the side down more, an optional ADAA soft-clip stage that takes a share of "
        "the peak reduction, TPDF or noise-shaped dither to 16, 20 or 24 bits and a loudness readout (momentary, "
        "short-term, integrated, range) of the output with a target-loudness gain suggestion. Gain reduction "
        "is published for meters.");
    auto& p = d.params;
    addOn(p, "mx", "Maximizer", "fx_mx", 1.0f, false);
    addF(p, "mx_gain", "Maximizer Gain", "fx_mx", 0.0f, 36.0f, 0.0f, "dB");
    addF(p, "mx_ceiling", "Maximizer Ceiling", "fx_mx", -12.0f, 0.0f, -1.0f, "dBTP");
    addF(p, "mx_release", "Maximizer Release", "fx_mx", 10.0f, 2000.0f, 150.0f, "ms", 0.35f);
    addF(p, "mx_lookahead", "Maximizer Lookahead", "fx_mx", 0.5f, 10.0f, 3.0f, "ms");
    p.push_back(params::choice("fx_mx_relmode", "Maximizer Release Mode", "fx_mx_2", {"Fixed", "Dual", "Adaptive"}, 2));
    addF(p, "mx_hold", "Maximizer Hold", "fx_mx_2", 0.0f, 50.0f, 8.0f, "ms", 0.5f);
    p.push_back(params::toggle("fx_mx_autohold", "Maximizer Auto Hold", "fx_mx_2", true));
    addF(p, "mx_link", "Maximizer Link", "fx_mx_2", 0.0f, 100.0f, 100.0f, "%");
    p.push_back(params::choice("fx_mx_stereo", "Maximizer Stereo", "fx_mx_2", {"L/R", "M/S bias"}, 0));
    addF(p, "mx_clip", "Maximizer Clip", "fx_mx_3", 0.0f, 100.0f, 0.0f, "%");
    addF(p, "mx_knee", "Maximizer Clip Knee", "fx_mx_3", 0.0f, 100.0f, 50.0f, "%");
    p.push_back(params::choice("fx_mx_os", "Maximizer Oversampling", "fx_mx_3", {"Off", "4x"}, 0));
    p.push_back(params::choice("fx_mx_dither", "Maximizer Dither", "fx_mx_3", {"Off", "TPDF", "Shaped"}, 0));
    p.push_back(params::choice("fx_mx_bits", "Maximizer Bits", "fx_mx_3", {"16", "20", "24"}, 0));
    p.push_back(params::integer("fx_pos_mx", "Maximizer Slot", "fx_mx_3", 1, kNumModuleFx, kNumFx + 5));
    p.push_back(params::toggle("fx_mx_mreset", "Maximizer Reset Meter", "fx_mx_4", false));
    addF(p, "mx_target", "Maximizer Target Loudness", "fx_mx_4", -30.0f, -5.0f, -14.0f, "LUFS");
    d.onParam = "fx_mx_on";
    d.positionParam = "fx_pos_mx";
    d.capabilities = {"gain_reduction", "latency", "loudness"};
    d.pages.push_back({"fx_mx", "Maximizer", "fx", "Maximizer", "Gain, ceiling (dBTP), release and lookahead."});
    d.pages.push_back({"fx_mx_2", "Maximizer 2", "fx", "Maximizer Release", "Release mode, hold, stereo link and mode."});
    d.pages.push_back({"fx_mx_3", "Maximizer 3", "fx", "Maximizer Output",
                       "Soft clip, oversampled audio path, dither and the chain slot."});
    d.pages.push_back({"fx_mx_4", "Maximizer 4", "fx", "Maximizer Loudness",
                       "Meter reset and the target loudness of the gain suggestion."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.maximizer")); };
    return d;
}

} // namespace crucible::modules::fx
