// SPDX-License-Identifier: MIT
#include "FxModuleSupport.h"

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : on(localIndex(d, "fx_filter_on")), mix(localIndex(d, "fx_filter_mix")), model(localIndex(d, "fx_filter_model")),
          cutoff(localIndex(d, "fx_filter_cutoff")), res(localIndex(d, "fx_filter_res")),
          drive(localIndex(d, "fx_filter_drive")), lfoRate(localIndex(d, "fx_filter_lfo_rate")),
          lfoSync(localIndex(d, "fx_filter_lfo_sync")), lfoDepth(localIndex(d, "fx_filter_lfo_depth"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxFilterParams q;
        q.on = in.on(on);
        q.mix = in.value(mix);
        q.model = in.rounded(model);
        q.cutoffHz = in.value(cutoff);
        q.resonance = in.value(res);
        q.drive = in.value(drive);
        q.lfoRateHz = in.value(lfoRate);
        q.lfoSyncBeats = lfoSyncBeats(in.rounded(lfoSync));
        q.lfoDepth = in.value(lfoDepth);
        q.bpm = in.bpm();
        fx.process(l, r, n, q);
    }

private:
    const int on, mix, model, cutoff, res, drive, lfoRate, lfoSync, lfoDepth;
    FxFilter fx;
};
} // namespace

ModuleDescriptor filterFxEffect()
{
    auto d = effectDescriptor("fx.filter", "Filter", "Filter", CpuClass::Medium,
                              "Any of the 17 voice filter models on the bus, with drive and a synced LFO sweep.");
    auto& p = d.params;
    // Filter FX
    addOn(p, "filter", "Filter FX", "fx_filter", 1.0f);
    p.push_back(params::choice("fx_filter_model", "Filter FX Model", "fx_filter", filterModelNames(), 0));
    addHz(p, "filter_cutoff", "Filter FX Cutoff", "fx_filter", 20.0f, 20000.0f, 2000.0f);
    addF(p, "filter_res", "Filter FX Resonance", "fx_filter", 0.0f, 1.0f, 0.3f);
    addF(p, "filter_drive", "Filter FX Drive", "fx_filter", 0.0f, 1.0f, 0.0f);
    addF(p, "filter_lfo_rate", "Filter FX LFO Rate", "fx_filter_2", 0.02f, 20.0f, 0.5f, "Hz", 0.4f, Smoothing::None);
    p.push_back(params::choice("fx_filter_lfo_sync", "Filter FX LFO Sync", "fx_filter_2", params::tempoSyncNames(), 0));
    addF(p, "filter_lfo_depth", "Filter FX LFO Depth", "fx_filter_2", 0.0f, 4.0f, 0.0f, "oct");
    addPosition(p, "filter", "Filter", "fx_order_2", 4);
    d.onParam = "fx_filter_on";
    d.positionParam = "fx_pos_filter";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.filter")); };
    return d;
}

} // namespace crucible::modules::fx
