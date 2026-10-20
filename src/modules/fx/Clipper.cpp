// SPDX-License-Identifier: MIT
// fx.clipper: hard / soft / morphing clipper with ADAA, oversampling and a true-peak guard
// (include/crucible/fx/ClipperEffect.h).
#include "FxModuleSupport.h"

#include <crucible/fx/ClipperEffect.h>
#include <crucible/module/LatencyReporter.h>

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_clp_mix")), drive(localIndex(d, "fx_clp_drive")), ceiling(localIndex(d, "fx_clp_ceiling")),
          shape(localIndex(d, "fx_clp_shape")), adaa(localIndex(d, "fx_clp_adaa")), oversample(localIndex(d, "fx_clp_os")),
          guard(localIndex(d, "fx_clp_guard"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        clipper.prepare(c.sampleRate, c.maxBlockSize, c.quality, c.oversampling);
        quality = c.quality;
        dirty = false;
    }
    void reset() override
    {
        clipper.reset();
        dirty = false;
    }
    void deactivate() override
    {
        if (dirty)
        {
            clipper.reset();
            dirty = false;
        }
    }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        crucible::fx::ClipperSettings s;
        s.driveDb = in.value(drive);
        s.ceilingDb = in.value(ceiling);
        s.softness = in.value(shape) * 0.01f;
        s.adaaOrder = std::clamp(in.choice(adaa), 0, 2);
        // Auto follows the quality policy of the plan (section 3.9): 1x for drafts, 2x as standard, 4x for renders.
        const int choice = in.choice(oversample);
        s.oversampleStages = choice == 0 ? (quality == Quality::Draft ? 0 : (quality == Quality::High ? 2 : 1)) : choice - 1;
        s.guard = in.on(guard);
        s.mix = in.value(mix);
        dirty = true;
        clipper.process(l, r, n, s);
    }

    int latencySamples() const override { return clipper.latencySamples(); }
    bool takeLatencyChanged() override { return clipper.takeLatencyChanged(); }

private:
    const int mix, drive, ceiling, shape, adaa, oversample, guard;
    crucible::fx::ClipperEffect clipper;
    Quality quality = Quality::Standard;
    bool dirty = false;
};
} // namespace

ModuleDescriptor clipperEffect()
{
    auto d = effectDescriptor(
        "fx.clipper", "Clipper", "Distortion", CpuClass::Medium,
        "Clipper with a ceiling in dBTP: drive into a curve that morphs from a hard clip to tanh, first-order "
        "first- or second-order antiderivative anti-aliasing (or none) at 1x, 2x, 4x or 8x oversampling (Auto follows the "
        "quality "
        "policy), a true-peak guard that trims the inter-sample overs of the band-limited result with a one "
        "millisecond lookahead, and a latency-aligned parallel mix. Latency is the oversampling filters plus the "
        "guard, reported.");
    auto& p = d.params;
    addOn(p, "clp", "Clipper", "fx_clp", 1.0f);
    addF(p, "clp_drive", "Clipper Drive", "fx_clp", 0.0f, 36.0f, 0.0f, "dB");
    addF(p, "clp_ceiling", "Clipper Ceiling", "fx_clp", -24.0f, 0.0f, -1.0f, "dBTP");
    addF(p, "clp_shape", "Clipper Shape", "fx_clp", 0.0f, 100.0f, 40.0f, "%");
    p.push_back(params::choice("fx_clp_adaa", "Clipper ADAA", "fx_clp", {"Off", "1st order", "2nd order"}, 2));
    p.push_back(params::choice("fx_clp_os", "Clipper Oversampling", "fx_clp_2", {"Auto", "1x", "2x", "4x", "8x"}, 0));
    p.push_back(params::toggle("fx_clp_guard", "Clipper True-Peak Guard", "fx_clp_2", true));
    p.push_back(params::integer("fx_pos_clp", "Clipper Slot", "fx_clp_2", 1, kNumModuleFx, kNumFx + 6));
    d.onParam = "fx_clp_on";
    d.positionParam = "fx_pos_clp";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_clp", "Clipper", "fx", "Clipper", "Mix, drive, ceiling (dBTP), shape and ADAA."});
    d.pages.push_back(
        {"fx_clp_2", "Clipper 2", "fx", "Clipper Quality", "Oversampling, true-peak guard and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.clipper")); };
    return d;
}

} // namespace crucible::modules::fx
