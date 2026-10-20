// SPDX-License-Identifier: MIT
#include "FxModuleSupport.h"

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public StepPosition
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : on(localIndex(d, "fx_trance_on")), mix(localIndex(d, "fx_trance_mix")), rate(localIndex(d, "fx_trance_rate")),
          pattern(localIndex(d, "fx_trance_pattern")), steps(localIndex(d, "fx_trance_steps")),
          depth(localIndex(d, "fx_trance_depth")), attack(localIndex(d, "fx_trance_attack")),
          release(localIndex(d, "fx_trance_release"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxTranceGateParams q;
        q.on = in.on(on);
        q.mix = in.value(mix);
        q.stepBeats = tranceGateRateBeats(in.rounded(rate));
        q.pattern = static_cast<std::uint32_t>(std::max(in.rounded(pattern), 0));
        q.steps = in.rounded(steps);
        q.depth = in.value(depth);
        q.attackMs = in.value(attack);
        q.releaseMs = in.value(release);
        q.bpm = in.bpm();
        fx.process(l, r, n, q);
    }

    int currentStep() const override { return fx.currentStep(); }

private:
    const int on, mix, rate, pattern, steps, depth, attack, release;
    FxTranceGate fx;
};
} // namespace

ModuleDescriptor tranceGateEffect()
{
    auto d = effectDescriptor("fx.trance", "Trance Gate", "Modulation", CpuClass::Light,
                              "Tempo-synced 16-step gate with shaped edges.");
    auto& p = d.params;
    // Trance gate
    addOn(p, "trance", "Trance Gate", "fx_trance", 1.0f);
    p.push_back(params::choice("fx_trance_rate", "Trance Gate Rate", "fx_trance", tranceGateRateNames(), 2));
    p.push_back(params::integer("fx_trance_pattern", "Trance Gate Pattern", "fx_trance", 0, 65535, 0xB6DB));
    p.push_back(params::integer("fx_trance_steps", "Trance Gate Steps", "fx_trance", 1, 16, 16));
    addF(p, "trance_depth", "Trance Gate Depth", "fx_trance", 0.0f, 1.0f, 1.0f);
    addF(p, "trance_attack", "Trance Gate Attack", "fx_trance_2", 0.1f, 50.0f, 2.0f, "ms", 0.4f);
    addF(p, "trance_release", "Trance Gate Release", "fx_trance_2", 0.1f, 200.0f, 20.0f, "ms", 0.4f);
    addPosition(p, "trance", "Trance Gate", "fx_order_2", 6);
    d.onParam = "fx_trance_on";
    d.positionParam = "fx_pos_trance";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.trance")); };
    return d;
}

} // namespace crucible::modules::fx
