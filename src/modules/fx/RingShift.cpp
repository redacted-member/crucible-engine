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
        : on(localIndex(d, "fx_ring_on")), mix(localIndex(d, "fx_ring_mix")), mode(localIndex(d, "fx_ring_mode")),
          freq(localIndex(d, "fx_ring_freq")), spread(localIndex(d, "fx_ring_spread"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxRingShiftParams q{in.on(on), in.value(mix), in.rounded(mode), in.value(freq), in.value(spread)};
        fx.process(l, r, n, q);
    }

private:
    const int on, mix, mode, freq, spread;
    FxRingShift fx;
};
} // namespace

ModuleDescriptor ringShiftEffect()
{
    auto d = effectDescriptor("fx.ring", "Ring / Shift", "Modulation", CpuClass::Medium,
                              "Ring modulator and single-sideband frequency shifter.");
    auto& p = d.params;
    // Ring modulator / frequency shifter
    addOn(p, "ring", "Ring / Shift", "fx_ring", 0.5f);
    p.push_back(params::choice("fx_ring_mode", "Ring / Shift Mode", "fx_ring", {"Ring", "Shift Up", "Shift Down"}, 0));
    addHz(p, "ring_freq", "Ring / Shift Freq", "fx_ring", 0.1f, 5000.0f, 200.0f, 0.25f);
    addF(p, "ring_spread", "Ring / Shift Spread", "fx_ring", 0.0f, 1.0f, 0.0f);
    addPosition(p, "ring", "Ring / Shift", "fx_order_3", 8);
    d.onParam = "fx_ring_on";
    d.positionParam = "fx_pos_ring";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.ring")); };
    return d;
}

} // namespace crucible::modules::fx
