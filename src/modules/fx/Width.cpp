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
        : on(localIndex(d, "fx_width_on")), mix(localIndex(d, "fx_width_mix")), amt(localIndex(d, "fx_width_amt")),
          bass(localIndex(d, "fx_width_bass")), pan(localIndex(d, "fx_width_pan"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxWidthParams q{in.on(on), in.value(mix), in.value(amt), in.value(bass), in.value(pan)};
        fx.process(l, r, n, q);
    }

private:
    const int on, mix, amt, bass, pan;
    FxWidth fx;
};
} // namespace

ModuleDescriptor widthEffect()
{
    auto d = effectDescriptor("fx.width", "Width", "Stereo", CpuClass::Light, "Mid/side width, bass mono and output pan.");
    auto& p = d.params;
    addOn(p, "width", "Width", "fx_width", 1.0f);
    addF(p, "width_amt", "Stereo Width", "fx_width", 0.0f, 2.0f, 1.0f);
    addF(p, "width_bass", "Bass Mono Below", "fx_width", 20.0f, 500.0f, 20.0f, "Hz", 0.4f);
    addF(p, "width_pan", "Output Pan", "fx_width", -1.0f, 1.0f, 0.0f);
    addPosition(p, "width", "Width", "fx_order_1", 2);
    d.onParam = "fx_width_on";
    d.positionParam = "fx_pos_width";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.width")); };
    return d;
}

} // namespace crucible::modules::fx
