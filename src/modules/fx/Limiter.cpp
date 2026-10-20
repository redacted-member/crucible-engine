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
        : on(localIndex(d, "fx_limit_on")), mix(localIndex(d, "fx_limit_mix")), ceiling(localIndex(d, "fx_limit_ceiling")),
          release(localIndex(d, "fx_limit_release")), gain(localIndex(d, "fx_limit_gain")),
          look(localIndex(d, "fx_limit_look"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxLimiterParams q{in.on(on), in.value(mix), in.value(ceiling), in.value(release), in.value(gain), in.value(look)};
        fx.process(l, r, n, q);
    }

private:
    const int on, mix, ceiling, release, gain, look;
    FxLimiter fx;
};
} // namespace

ModuleDescriptor limiterEffect()
{
    auto d = effectDescriptor("fx.limiter", "Limiter", "Dynamics", CpuClass::Light, "Lookahead peak limiter.");
    auto& p = d.params;
    addOn(p, "limit", "Limiter", "fx_limit", 1.0f);
    addF(p, "limit_ceiling", "Limiter Ceiling", "fx_limit", -24.0f, 0.0f, -0.3f, "dB");
    addF(p, "limit_release", "Limiter Release", "fx_limit", 5.0f, 1000.0f, 80.0f, "ms", 0.4f);
    addF(p, "limit_gain", "Limiter Gain", "fx_limit", 0.0f, 24.0f, 0.0f, "dB");
    addF(p, "limit_look", "Limiter Lookahead", "fx_limit", 0.1f, 5.0f, 1.5f, "ms");
    addPosition(p, "limiter", "Limiter", "fx_order_1", 3);
    d.onParam = "fx_limit_on";
    d.positionParam = "fx_pos_limiter";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.limiter")); };
    return d;
}

} // namespace crucible::modules::fx
