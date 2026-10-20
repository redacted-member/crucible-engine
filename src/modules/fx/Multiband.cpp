// SPDX-License-Identifier: MIT
#include "FxModuleSupport.h"

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public GainReductionMeter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : on(localIndex(d, "fx_multiband_on")), mix(localIndex(d, "fx_multiband_mix")),
          xlow(localIndex(d, "fx_multiband_xlow")), xhigh(localIndex(d, "fx_multiband_xhigh")),
          attack(localIndex(d, "fx_multiband_attack")), release(localIndex(d, "fx_multiband_release")),
          ratio(localIndex(d, "fx_multiband_ratio")), lowThresh(localIndex(d, "fx_multiband_low_thresh")),
          midThresh(localIndex(d, "fx_multiband_mid_thresh")), highThresh(localIndex(d, "fx_multiband_high_thresh")),
          lowGain(localIndex(d, "fx_multiband_low_gain")), midGain(localIndex(d, "fx_multiband_mid_gain")),
          highGain(localIndex(d, "fx_multiband_high_gain"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxMultibandParams q;
        q.on = in.on(on);
        q.mix = in.value(mix);
        q.lowCrossHz = in.value(xlow);
        q.highCrossHz = in.value(xhigh);
        q.attackMs = in.value(attack);
        q.releaseMs = in.value(release);
        q.ratio = in.value(ratio);
        q.thresholdDb = {in.value(lowThresh), in.value(midThresh), in.value(highThresh)};
        q.gainDb = {in.value(lowGain), in.value(midGain), in.value(highGain)};
        fx.process(l, r, n, q);
    }

    int meterBands() const override { return 3; }
    float gainReductionDb(int band) const override { return fx.gainReductionDb(band); }

private:
    const int on, mix, xlow, xhigh, attack, release, ratio, lowThresh, midThresh, highThresh, lowGain, midGain, highGain;
    FxMultiband fx;
};
} // namespace

ModuleDescriptor multibandEffect()
{
    auto d = effectDescriptor("fx.multiband", "Multiband", "Dynamics", CpuClass::Medium,
                              "Three-band compressor with LR4 crossovers and gain-reduction meters.");
    auto& p = d.params;
    // Multiband compressor
    addOn(p, "multiband", "Multiband", "fx_multiband", 1.0f);
    addHz(p, "multiband_xlow", "Multiband Low X-Over", "fx_multiband", 40.0f, 1000.0f, 200.0f, 0.4f);
    addHz(p, "multiband_xhigh", "Multiband High X-Over", "fx_multiband", 800.0f, 12000.0f, 3000.0f, 0.4f);
    addF(p, "multiband_attack", "Multiband Attack", "fx_multiband", 0.1f, 200.0f, 10.0f, "ms", 0.4f);
    addF(p, "multiband_release", "Multiband Release", "fx_multiband", 5.0f, 2000.0f, 150.0f, "ms", 0.4f);
    addF(p, "multiband_ratio", "Multiband Ratio", "fx_multiband_2", 1.0f, 20.0f, 4.0f, ":1", 0.5f);
    addF(p, "multiband_low_thresh", "Multiband Low Threshold", "fx_multiband_2", -60.0f, 0.0f, -24.0f, "dB");
    addF(p, "multiband_mid_thresh", "Multiband Mid Threshold", "fx_multiband_2", -60.0f, 0.0f, -24.0f, "dB");
    addF(p, "multiband_high_thresh", "Multiband High Threshold", "fx_multiband_2", -60.0f, 0.0f, -24.0f, "dB");
    addF(p, "multiband_low_gain", "Multiband Low Gain", "fx_multiband_3", -24.0f, 24.0f, 0.0f, "dB");
    addF(p, "multiband_mid_gain", "Multiband Mid Gain", "fx_multiband_3", -24.0f, 24.0f, 0.0f, "dB");
    addF(p, "multiband_high_gain", "Multiband High Gain", "fx_multiband_3", -24.0f, 24.0f, 0.0f, "dB");
    addPosition(p, "multiband", "Multiband", "fx_order_2", 5);
    d.onParam = "fx_multiband_on";
    d.positionParam = "fx_pos_multiband";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.multiband")); };
    return d;
}

} // namespace crucible::modules::fx
