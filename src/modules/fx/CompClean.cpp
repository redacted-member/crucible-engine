// SPDX-License-Identifier: MIT
// fx.comp_clean: the clean digital compressor on the dynamics core (include/crucible/fx/Dynamics.h).
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
        : mix(localIndex(d, "fx_cc_mix")), mode(localIndex(d, "fx_cc_mode")), thresh(localIndex(d, "fx_cc_thresh")),
          ratio(localIndex(d, "fx_cc_ratio")), attack(localIndex(d, "fx_cc_attack")),
          release(localIndex(d, "fx_cc_release")), knee(localIndex(d, "fx_cc_knee")), makeup(localIndex(d, "fx_cc_makeup")),
          autoMakeup(localIndex(d, "fx_cc_amakeup")), range(localIndex(d, "fx_cc_range")), hold(localIndex(d, "fx_cc_hold")),
          autoRelease(localIndex(d, "fx_cc_auto")), detector(localIndex(d, "fx_cc_detector")),
          rms(localIndex(d, "fx_cc_rms")), truePeak(localIndex(d, "fx_cc_truepeak")),
          topology(localIndex(d, "fx_cc_topology")), lookahead(localIndex(d, "fx_cc_lookahead")),
          hp(localIndex(d, "fx_cc_schp")), lp(localIndex(d, "fx_cc_sclp")), bellHz(localIndex(d, "fx_cc_scbf")),
          bellDb(localIndex(d, "fx_cc_scbg")), bellQ(localIndex(d, "fx_cc_scbq")), key(localIndex(d, "fx_cc_key")),
          listen(localIndex(d, "fx_cc_listen")), channels(localIndex(d, "fx_cc_channels")), link(localIndex(d, "fx_cc_link"))
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
    // Switched off: forget the lookahead buffer and the gain so that switching on replays nothing.
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
        s.kind = in.choice(mode) == 1 ? DynamicsKind::Upward : DynamicsKind::Compressor;
        s.thresholdDb = in.value(thresh);
        const float ratioValue = in.value(ratio);
        s.ratio = ratioValue >= 99.5f ? 1.0e6f : ratioValue;
        s.kneeDb = in.value(knee);
        s.rangeDb = in.value(range);
        s.detector = static_cast<DetectorMode>(std::clamp(in.choice(detector), 0, 2));
        s.truePeak = in.on(truePeak);
        s.feedback = in.choice(topology) == 1;
        s.rmsWindowMs = in.value(rms);
        s.attackMs = in.value(attack);
        s.releaseMs = in.value(release);
        s.peakHoldMs = std::clamp(0.04f * s.releaseMs, 0.5f, 5.0f);
        s.holdMs = in.value(hold);
        s.autoRelease = in.on(autoRelease);
        s.scHighpassHz = in.value(hp);
        s.scLowpassHz = in.value(lp);
        s.scBellHz = in.value(bellHz);
        s.scBellDb = in.value(bellDb);
        s.scBellQ = in.value(bellQ);
        s.listen = in.on(listen);
        s.channels = static_cast<ChannelMode>(std::clamp(in.choice(channels), 0, static_cast<int>(ChannelMode::Count) - 1));
        s.link = in.value(link) * 0.01f;
        s.lookaheadMs = in.value(lookahead);
        s.makeupDb = in.value(makeup);
        s.autoMakeup = in.on(autoMakeup);
        s.mix = in.value(mix);
        dirty = true;
        proc.process(l, r, n, in.auxLeft(), in.auxRight(), s, in.choice(key) == 1 && in.hasAux());
    }

    int meterBands() const override { return 1; }
    float gainReductionDb(int) const override { return proc.gainReductionDb(); }
    int latencySamples() const override { return proc.latencySamples(); }
    bool takeLatencyChanged() override { return proc.takeLatencyChanged(); }

private:
    const int mix, mode, thresh, ratio, attack, release, knee, makeup, autoMakeup, range, hold, autoRelease, detector, rms,
        truePeak, topology, lookahead, hp, lp, bellHz, bellDb, bellQ, key, listen, channels, link;
    crucible::fx::DynamicsProcessor proc;
    bool dirty = false;
};
} // namespace

ModuleDescriptor compCleanEffect()
{
    auto d = effectDescriptor(
        "fx.comp_clean", "Clean Compressor", "Dynamics", CpuClass::Light,
        "Clean digital compressor on the shared dynamics core: peak, RMS or hybrid detection (optionally a 4x "
        "true-peak reading), feed-forward or feedback, soft knee up to 24 dB, ratio 1:1 to infinity, downward or "
        "upward compression, attack 0.02 to 300 ms and release 5 ms to 5 s (times to 63 % of a step of gain) or an "
        "automatic dual-stage release, hold, lookahead up to 10 ms (reported as latency), sidechain high-pass, "
        "low-pass and bell, external key from the host's auxiliary input (the internal key is used when none is "
        "supplied), listen, channel mode with link, range, parallel mix and automatic makeup. Gain reduction is "
        "published for meters.");
    auto& p = d.params;
    addOn(p, "cc", "Clean Comp", "fx_cc", 1.0f);
    addF(p, "cc_thresh", "Clean Comp Threshold", "fx_cc", -60.0f, 0.0f, -18.0f, "dB");
    addF(p, "cc_ratio", "Clean Comp Ratio", "fx_cc", 1.0f, 100.0f, 4.0f, ":1", 0.4f);
    addF(p, "cc_attack", "Clean Comp Attack", "fx_cc", 0.02f, 300.0f, 10.0f, "ms", 0.35f);
    addF(p, "cc_release", "Clean Comp Release", "fx_cc", 5.0f, 5000.0f, 120.0f, "ms", 0.35f);
    addF(p, "cc_knee", "Clean Comp Knee", "fx_cc_2", 0.0f, 24.0f, 6.0f, "dB");
    addF(p, "cc_makeup", "Clean Comp Makeup", "fx_cc_2", 0.0f, 18.0f, 0.0f, "dB");
    p.push_back(params::toggle("fx_cc_amakeup", "Clean Comp Auto Makeup", "fx_cc_2", false));
    addF(p, "cc_range", "Clean Comp Range", "fx_cc_2", 0.0f, 60.0f, 60.0f, "dB");
    addF(p, "cc_hold", "Clean Comp Hold", "fx_cc_2", 0.0f, 500.0f, 0.0f, "ms", 0.5f);
    p.push_back(params::toggle("fx_cc_auto", "Clean Comp Auto Release", "fx_cc_2", false));
    p.push_back(params::choice("fx_cc_detector", "Clean Comp Detector", "fx_cc_3", {"Peak", "RMS", "Hybrid"}, 1));
    addF(p, "cc_rms", "Clean Comp RMS Window", "fx_cc_3", 1.0f, 100.0f, 20.0f, "ms", 0.5f);
    p.push_back(params::toggle("fx_cc_truepeak", "Clean Comp True Peak", "fx_cc_3", false));
    p.push_back(params::choice("fx_cc_topology", "Clean Comp Topology", "fx_cc_3", {"Feed-forward", "Feedback"}, 0));
    addF(p, "cc_lookahead", "Clean Comp Lookahead", "fx_cc_3", 0.0f, 10.0f, 0.0f, "ms");
    p.push_back(params::integer("fx_pos_cc", "Clean Comp Slot", "fx_cc_3", 1, kNumModuleFx, kNumFx + 2));
    addHz(p, "cc_schp", "Clean Comp Sidechain HP", "fx_cc_4", 20.0f, 1000.0f, 20.0f, 0.4f);
    addHz(p, "cc_sclp", "Clean Comp Sidechain LP", "fx_cc_4", 1000.0f, 20000.0f, 20000.0f, 0.4f);
    addHz(p, "cc_scbf", "Clean Comp Sidechain Bell Freq", "fx_cc_4", 100.0f, 10000.0f, 1000.0f, 0.4f);
    addF(p, "cc_scbg", "Clean Comp Sidechain Bell Gain", "fx_cc_4", -18.0f, 18.0f, 0.0f, "dB");
    addF(p, "cc_scbq", "Clean Comp Sidechain Bell Q", "fx_cc_4", 0.3f, 8.0f, 1.0f, {}, 0.5f);
    p.push_back(params::choice("fx_cc_key", "Clean Comp Key", "fx_cc_4", {"Internal", "External"}, 0));
    p.push_back(params::toggle("fx_cc_listen", "Clean Comp Listen", "fx_cc_5", false));
    {
        std::vector<std::string> names;
        for (int i = 0; i < static_cast<int>(crucible::fx::ChannelMode::Count); ++i)
            names.emplace_back(crucible::fx::channelModeName(static_cast<crucible::fx::ChannelMode>(i)));
        p.push_back(params::choice("fx_cc_channels", "Clean Comp Channels", "fx_cc_5", names, 0));
    }
    addF(p, "cc_link", "Clean Comp Link", "fx_cc_5", 0.0f, 100.0f, 100.0f, "%");
    p.push_back(params::choice("fx_cc_mode", "Clean Comp Mode", "fx_cc_5", {"Downward", "Upward"}, 0));
    d.onParam = "fx_cc_on";
    d.positionParam = "fx_pos_cc";
    d.capabilities = {"gain_reduction", "latency"};
    d.pages.push_back({"fx_cc", "Clean Comp", "fx", "Clean Compressor",
                       "Threshold, ratio, attack and release; the LCD draws the transfer curve."});
    d.pages.push_back(
        {"fx_cc_2", "Clean Comp 2", "fx", "Clean Compressor Gain", "Knee, makeup, range, hold and automatic release."});
    d.pages.push_back({"fx_cc_3", "Clean Comp 3", "fx", "Clean Compressor Detector",
                       "Detector, topology, lookahead (latency) and the chain slot."});
    d.pages.push_back(
        {"fx_cc_4", "Clean Comp 4", "fx", "Clean Compressor Sidechain", "Sidechain filters and the key source."});
    d.pages.push_back({"fx_cc_5", "Clean Comp 5", "fx", "Clean Compressor Stereo", "Listen, channel mode, link and mode."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.comp_clean")); };
    return d;
}

} // namespace crucible::modules::fx
