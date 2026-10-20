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
        : on(localIndex(d, "fx_crush_on")), mix(localIndex(d, "fx_crush_mix")), bits(localIndex(d, "fx_crush_bits")),
          rate(localIndex(d, "fx_crush_rate")), jitter(localIndex(d, "fx_crush_jitter")),
          dither(localIndex(d, "fx_crush_dither")), track(localIndex(d, "fx_crush_track")),
          ratio(localIndex(d, "fx_crush_ratio")), fine(localIndex(d, "fx_crush_fine"))
    {
    }

    void prepare(const ModuleContext& c) override { fx.prepare(c.sampleRate); }
    void reset() override { fx.reset(); }
    void deactivate() override { fx.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        FxCrusherParams q{in.on(on), in.value(mix), in.value(bits), in.value(rate), in.value(jitter), in.value(dither)};
        // Note tracking: the rate follows the engine's last played note (no note yet: the fixed rate).
        q.track = in.on(track) && in.hasNote();
        if (q.track)
        {
            q.noteHz = 440.0f * std::exp2((in.note() - 69.0f) * (1.0f / 12.0f));
            q.ratio = in.value(ratio);
            q.fineCents = in.value(fine);
        }
        fx.process(l, r, n, q);
    }

private:
    const int on, mix, bits, rate, jitter, dither, track, ratio, fine;
    FxCrusher fx;
};
} // namespace

ModuleDescriptor crusherEffect()
{
    auto d = effectDescriptor("fx.crusher", "Crusher", "Distortion", CpuClass::Light,
                              "Bit depth and sample rate reduction with jitter and dither. Track makes the rate follow the last played "
                              "note (note frequency x ratio, fine in cents).");
    auto& p = d.params;
    addOn(p, "crush", "Crusher", "fx_crush", 1.0f);
    addF(p, "crush_bits", "Crusher Bits", "fx_crush", 1.0f, 16.0f, 8.0f, "bit");
    addF(p, "crush_rate", "Crusher Rate", "fx_crush", 500.0f, 48000.0f, 12000.0f, "Hz", 0.4f);
    addF(p, "crush_jitter", "Crusher Jitter", "fx_crush", 0.0f, 1.0f, 0.0f);
    addF(p, "crush_dither", "Crusher Dither", "fx_crush", 0.0f, 1.0f, 0.0f);
    addPosition(p, "crusher", "Crusher", "fx_order_1", 1);
    // Note tracking (version 2): flagged appended so the frozen parameter order does not move.
    const size_t firstNew = p.size();
    p.push_back(params::toggle("fx_crush_track", "Crusher Track", "fx_crush_track", false));
    addF(p, "crush_ratio", "Crusher Track Ratio", "fx_crush_track", 0.25f, 16.0f, 1.0f, "x", 0.4f);
    addF(p, "crush_fine", "Crusher Track Fine", "fx_crush_track", -100.0f, 100.0f, 0.0f, "ct");
    for (size_t i = firstNew; i < p.size(); ++i)
    {
        p[i].since = 2;
        p[i].appended = 1;
    }
    // The crusher page already holds six knobs, so the tracking knobs get a page of their own.
    d.pages.push_back({"fx_crush_track", "Crusher Track", "fx", "Crusher Note Tracking",
                       "Makes the crusher rate follow the last played note."});
    d.version = 2;
    d.onParam = "fx_crush_on";
    d.positionParam = "fx_pos_crusher";
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.crusher")); };
    return d;
}

} // namespace crucible::modules::fx
