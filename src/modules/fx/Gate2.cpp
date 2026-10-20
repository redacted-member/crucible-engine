// SPDX-License-Identifier: MIT
// fx.gate2: expander, gate and ducker on the dynamics core (include/crucible/fx/Dynamics.h).
#include "FxModuleSupport.h"

#include <crucible/fx/Dynamics.h>
#include <crucible/module/LatencyReporter.h>

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public GainReductionMeter, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_g2_mix")), mode(localIndex(d, "fx_g2_mode")), thresh(localIndex(d, "fx_g2_thresh")),
          ratio(localIndex(d, "fx_g2_ratio")), range(localIndex(d, "fx_g2_range")), hyst(localIndex(d, "fx_g2_hyst")),
          attack(localIndex(d, "fx_g2_attack")), hold(localIndex(d, "fx_g2_hold")), release(localIndex(d, "fx_g2_release")),
          knee(localIndex(d, "fx_g2_knee")), lookahead(localIndex(d, "fx_g2_lookahead")),
          detector(localIndex(d, "fx_g2_detector")), hp(localIndex(d, "fx_g2_schp")), lp(localIndex(d, "fx_g2_sclp")),
          key(localIndex(d, "fx_g2_key")), listen(localIndex(d, "fx_g2_listen")), channels(localIndex(d, "fx_g2_channels")),
          link(localIndex(d, "fx_g2_link"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        proc.prepare(c.sampleRate);
        dirty = false;
    }
    void reset() override
    {
        proc.reset();
        dirty = false;
    }
    void deactivate() override
    {
        if (dirty)
        {
            proc.reset();
            dirty = false;
        }
    }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        using namespace crucible::fx;
        DynamicsSettings s;
        switch (in.choice(mode))
        {
        case 1:
            s.kind = DynamicsKind::Expander;
            break;
        case 2:
            s.kind = DynamicsKind::Ducker;
            break;
        default:
            s.kind = DynamicsKind::Gate;
            break;
        }
        s.thresholdDb = in.value(thresh);
        const float ratioValue = in.value(ratio);
        s.ratio = ratioValue >= 99.5f ? 1000.0f : ratioValue;
        s.rangeDb = std::abs(in.value(range));
        s.hysteresisDb = in.value(hyst);
        s.kneeDb = in.value(knee);
        s.attackMs = in.value(attack);
        s.releaseMs = in.value(release);
        s.holdMs = in.value(hold);
        s.lookaheadMs = in.value(lookahead);
        s.detector = static_cast<DetectorMode>(std::clamp(in.choice(detector), 0, 2));
        s.rmsWindowMs = 4.0f;
        s.peakHoldMs = 1.0f;
        s.scHighpassHz = in.value(hp);
        s.scLowpassHz = in.value(lp);
        s.listen = in.on(listen);
        s.channels = static_cast<ChannelMode>(std::clamp(in.choice(channels), 0, static_cast<int>(ChannelMode::Count) - 1));
        s.link = in.value(link) * 0.01f;
        s.mix = in.value(mix);
        dirty = true;
        proc.process(l, r, n, in.auxLeft(), in.auxRight(), s, in.choice(key) == 1 && in.hasAux());
    }

    int meterBands() const override { return 1; }
    float gainReductionDb(int) const override { return proc.gainReductionDb(); }
    int latencySamples() const override { return proc.latencySamples(); }
    bool takeLatencyChanged() override { return proc.takeLatencyChanged(); }

private:
    const int mix, mode, thresh, ratio, range, hyst, attack, hold, release, knee, lookahead, detector, hp, lp, key, listen,
        channels, link;
    crucible::fx::DynamicsProcessor proc;
    bool dirty = false;
};
} // namespace

ModuleDescriptor gate2Effect()
{
    auto d = effectDescriptor(
        "fx.gate2", "Gate Expander Ducker", "Dynamics", CpuClass::Light,
        "Expander, gate and ducker on the shared dynamics core: ratio 1:1 to 1:infinity, range, hysteresis, hold, "
        "attack (opening) and release (closing) as times to 63 % of a step of gain, lookahead up to 10 ms "
        "(reported as latency) so a fast attack does not click, sidechain high-pass and low-pass, external key "
        "from the host's auxiliary input, listen, channel mode with link. Ducker mode lowers the gain by the "
        "range while the key is above the threshold. Gain reduction is published for meters.");
    auto& p = d.params;
    addOn(p, "g2", "Gate2", "fx_g2", 1.0f);
    p.push_back(params::choice("fx_g2_mode", "Gate2 Mode", "fx_g2", {"Gate", "Expander", "Ducker"}, 0));
    addF(p, "g2_thresh", "Gate2 Threshold", "fx_g2", -90.0f, 0.0f, -45.0f, "dB");
    addF(p, "g2_ratio", "Gate2 Ratio", "fx_g2", 1.0f, 100.0f, 100.0f, ":1", 0.4f);
    addF(p, "g2_range", "Gate2 Range", "fx_g2", -90.0f, 0.0f, -80.0f, "dB");
    addF(p, "g2_hyst", "Gate2 Hysteresis", "fx_g2_2", 0.0f, 24.0f, 6.0f, "dB");
    addF(p, "g2_attack", "Gate2 Attack", "fx_g2_2", 0.02f, 100.0f, 0.5f, "ms", 0.4f);
    addF(p, "g2_hold", "Gate2 Hold", "fx_g2_2", 0.0f, 1000.0f, 30.0f, "ms", 0.5f);
    addF(p, "g2_release", "Gate2 Release", "fx_g2_2", 1.0f, 3000.0f, 120.0f, "ms", 0.4f);
    addF(p, "g2_knee", "Gate2 Knee", "fx_g2_2", 0.0f, 24.0f, 0.0f, "dB");
    addF(p, "g2_lookahead", "Gate2 Lookahead", "fx_g2_2", 0.0f, 10.0f, 0.0f, "ms");
    p.push_back(params::choice("fx_g2_detector", "Gate2 Detector", "fx_g2_3", {"Peak", "RMS", "Hybrid"}, 0));
    addHz(p, "g2_schp", "Gate2 Sidechain HP", "fx_g2_3", 20.0f, 1000.0f, 20.0f, 0.4f);
    addHz(p, "g2_sclp", "Gate2 Sidechain LP", "fx_g2_3", 1000.0f, 20000.0f, 20000.0f, 0.4f);
    p.push_back(params::choice("fx_g2_key", "Gate2 Key", "fx_g2_3", {"Internal", "External"}, 0));
    p.push_back(params::toggle("fx_g2_listen", "Gate2 Listen", "fx_g2_3", false));
    p.push_back(params::integer("fx_pos_g2", "Gate2 Slot", "fx_g2_3", 1, kNumModuleFx, kNumFx + 4));
    {
        std::vector<std::string> names;
        for (int i = 0; i < static_cast<int>(crucible::fx::ChannelMode::Count); ++i)
            names.emplace_back(crucible::fx::channelModeName(static_cast<crucible::fx::ChannelMode>(i)));
        p.push_back(params::choice("fx_g2_channels", "Gate2 Channels", "fx_g2_4", names, 0));
    }
    addF(p, "g2_link", "Gate2 Link", "fx_g2_4", 0.0f, 100.0f, 100.0f, "%");
    d.onParam = "fx_g2_on";
    d.positionParam = "fx_pos_g2";
    d.capabilities = {"gain_reduction", "latency"};
    d.pages.push_back({"fx_g2", "Gate2", "fx", "Gate Expander Ducker", "Mode, threshold, ratio and range."});
    d.pages.push_back({"fx_g2_2", "Gate2 2", "fx", "Gate Expander Ducker Timing",
                       "Hysteresis, attack, hold, release, knee and lookahead."});
    d.pages.push_back({"fx_g2_3", "Gate2 3", "fx", "Gate Expander Ducker Sidechain",
                       "Detector, sidechain filters, key, listen and the chain slot."});
    d.pages.push_back({"fx_g2_4", "Gate2 4", "fx", "Gate Expander Ducker Stereo", "Channel mode and link."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.gate2")); };
    return d;
}

} // namespace crucible::modules::fx
