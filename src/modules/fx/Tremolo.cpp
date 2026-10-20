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
        : on(localIndex(d, "fx_tremolo_on")), rate(localIndex(d, "fx_tremolo_rate")), sync(localIndex(d, "fx_tremolo_sync")),
          depth(localIndex(d, "fx_tremolo_depth")), shape(localIndex(d, "fx_tremolo_shape")),
          phase(localIndex(d, "fx_tremolo_phase"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxTremoloParams q;
        q.on = in.on(on);
        q.rateHz = in.value(rate);
        q.syncBeats = lfoSyncBeats(in.rounded(sync));
        q.depth = in.value(depth);
        q.shape = in.rounded(shape);
        q.stereoPhaseDeg = in.value(phase);
        q.bpm = in.bpm();
        fx.process(l, r, n, q);
    }

private:
    const int on, rate, sync, depth, shape, phase;
    FxTremolo fx;
};
} // namespace

ModuleDescriptor tremoloEffect()
{
    auto d = effectDescriptor("fx.tremolo", "Tremolo", "Modulation", CpuClass::Light,
                              "Tremolo and auto-pan with five shapes and tempo sync.");
    auto& p = d.params;
    // Tremolo / auto-pan
    addOn(p, "tremolo", "Tremolo", "fx_tremolo", 1.0f, false);
    addF(p, "tremolo_rate", "Tremolo Rate", "fx_tremolo", 0.05f, 30.0f, 4.0f, "Hz", 0.4f, Smoothing::None);
    p.push_back(params::choice("fx_tremolo_sync", "Tremolo Sync", "fx_tremolo", params::tempoSyncNames(), 0));
    addF(p, "tremolo_depth", "Tremolo Depth", "fx_tremolo", 0.0f, 1.0f, 0.5f);
    p.push_back(params::choice("fx_tremolo_shape", "Tremolo Shape", "fx_tremolo", tremoloShapeNames(), 0));
    addF(p, "tremolo_phase", "Tremolo Stereo Phase", "fx_tremolo", 0.0f, 180.0f, 0.0f, "deg");
    addPosition(p, "tremolo", "Tremolo", "fx_order_3", 7);
    d.onParam = "fx_tremolo_on";
    d.positionParam = "fx_pos_tremolo";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.tremolo")); };
    return d;
}

} // namespace crucible::modules::fx
