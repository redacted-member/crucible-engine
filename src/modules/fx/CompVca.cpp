// SPDX-License-Identifier: MIT
// fx.comp_vca: VCA bus compressor on the dynamics core (include/crucible/fx/Dynamics.h). Designed from public circuit
// descriptions of VCA bus compressors: feed-forward, smoothed-RMS detector, stepped ratio / attack / release, a release
// that follows the programme, a sidechain high-pass, a clean gain cell with a slight third harmonic that grows with gain
// reduction.
#include "FxModuleSupport.h"

#include <crucible/Saturation.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/module/LatencyReporter.h>

namespace crucible::modules::fx
{
namespace
{
constexpr float kRatios[] = {1.5f, 2.0f, 4.0f, 10.0f};
constexpr float kAttackMs[] = {0.1f, 0.3f, 1.0f, 3.0f, 10.0f, 30.0f};
constexpr float kReleaseMs[] = {100.0f, 300.0f, 600.0f, 1200.0f};

class Instance final : public EffectModule, public GainReductionMeter, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_cv_mix")), thresh(localIndex(d, "fx_cv_thresh")), ratio(localIndex(d, "fx_cv_ratio")),
          attack(localIndex(d, "fx_cv_attack")), release(localIndex(d, "fx_cv_release")),
          makeup(localIndex(d, "fx_cv_makeup")), knee(localIndex(d, "fx_cv_knee")), hp(localIndex(d, "fx_cv_schp")),
          key(localIndex(d, "fx_cv_key")), channels(localIndex(d, "fx_cv_channels")), link(localIndex(d, "fx_cv_link")),
          console(localIndex(d, "fx_cv_console")), drive(localIndex(d, "fx_cv_drive"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        proc.prepare(c.sampleRate);
        rate = c.sampleRate;
        driveKnown = false;
        saturation::Settings st;
        st.model = saturation::Model::Transformer;
        st.cornerHz = 14.0f;
        st.driveDb = 0.0f;
        st.dcCornerHz = 5.0f;
        for (auto& sat : stage)
        {
            sat.prepare(c.sampleRate);
            sat.set(st);
        }
        dirty = false;
    }
    void reset() override
    {
        proc.reset();
        driveKnown = false;
        for (auto& sat : stage)
            sat.reset();
        dirty = false;
    }
    void deactivate() override
    {
        if (dirty)
            reset();
    }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        using namespace crucible::fx;
        DynamicsSettings s;
        s.kind = DynamicsKind::Compressor;
        s.thresholdDb = in.value(thresh);
        s.ratio = kRatios[std::clamp(in.choice(ratio), 0, 3)];
        s.kneeDb = in.value(knee);
        s.rangeDb = 40.0f;
        s.detector = DetectorMode::Rms;
        s.rmsWindowMs = 6.0f;
        s.attackMs = kAttackMs[std::clamp(in.choice(attack), 0, 5)];
        const int rel = in.choice(release);
        s.autoRelease = rel >= 4;
        s.releaseMs = kReleaseMs[std::clamp(rel, 0, 3)];
        s.autoFastMs = 60.0f;
        s.autoSlowMs = 1500.0f;
        s.autoChargeMs = 600.0f;
        s.scHighpassHz = in.value(hp);
        s.channels = static_cast<ChannelMode>(std::clamp(in.choice(channels), 0, static_cast<int>(ChannelMode::Count) - 1));
        s.link = in.value(link) * 0.01f;
        s.makeupDb = in.value(makeup);
        s.mix = in.value(mix);
        s.cellColour = 1.0f;
        dirty = true;
        proc.process(l, r, n, in.auxLeft(), in.auxRight(), s, in.choice(key) == 1 && in.hasAux());
        if (!in.on(console))
        {
            driveKnown = false;
            return;
        }
        const float target = in.value(drive);
        if (!driveKnown)
        {
            driveKnown = true;
            driveA = driveB = target;
            applyDrive(target);
        }
        // The console drive glides (two cascaded one-poles on the dB value, applied in short chunks) so a moving
        // control never steps the stage's gain; once it has arrived the block is one call, as for a constant setting.
        int done = 0;
        while (done < n)
        {
            if (driveB == target && driveA == target)
            {
                stage[0].process(l + done, l + done, n - done);
                stage[1].process(r + done, r + done, n - done);
                break;
            }
            const int len = std::min(kDriveChunk, n - done);
            const float coef = 1.0f - std::exp(-static_cast<float>(len) / (kDriveGlideMs * 0.001f * static_cast<float>(rate)));
            driveA += (target - driveA) * coef;
            driveB += (driveA - driveB) * coef;
            if (std::abs(target - driveB) < 1.0e-3f && std::abs(target - driveA) < 1.0e-3f)
                driveA = driveB = target;
            applyDrive(driveB);
            stage[0].process(l + done, l + done, len);
            stage[1].process(r + done, r + done, len);
            done += len;
        }
    }

    int meterBands() const override { return 1; }
    float gainReductionDb(int) const override { return proc.gainReductionDb(); }
    int latencySamples() const override { return proc.latencySamples(); }
    bool takeLatencyChanged() override { return proc.takeLatencyChanged(); }

private:
    static constexpr int kDriveChunk = 8;
    static constexpr float kDriveGlideMs = 20.0f;
    void applyDrive(float db)
    {
        saturation::Settings st = stage[0].settings();
        st.driveDb = db;
        stage[0].set(st);
        stage[1].set(st);
    }
    const int mix, thresh, ratio, attack, release, makeup, knee, hp, key, channels, link, console, drive;
    crucible::fx::DynamicsProcessor proc;
    saturation::Saturator stage[2];
    bool dirty = false;
    bool driveKnown = false;
    double rate = 48000.0;
    float driveA = 0.0f, driveB = 0.0f;
};
} // namespace

ModuleDescriptor compVcaEffect()
{
    auto d = effectDescriptor(
        "fx.comp_vca", "VCA Bus Compressor", "Dynamics", CpuClass::Light,
        "VCA bus compressor: feed-forward with a smoothed-RMS detector, ratios 1.5, 2, 4 and 10:1, attack 0.1 to "
        "30 ms, release 0.1 to 1.2 s or auto (fast for transients, slow for sustained material), sidechain "
        "high-pass, external key from the host's auxiliary input, stereo link, parallel mix and makeup. The gain "
        "cell is clean with a slight third harmonic that grows with gain reduction; the optional console output "
        "stage is a transformer model.");
    auto& p = d.params;
    addOn(p, "cv", "VCA Comp", "fx_cv", 1.0f);
    addF(p, "cv_thresh", "VCA Comp Threshold", "fx_cv", -40.0f, 10.0f, -12.0f, "dB");
    p.push_back(params::choice("fx_cv_ratio", "VCA Comp Ratio", "fx_cv", {"1.5:1", "2:1", "4:1", "10:1"}, 1));
    p.push_back(params::choice("fx_cv_attack", "VCA Comp Attack", "fx_cv",
                               {"0.1 ms", "0.3 ms", "1 ms", "3 ms", "10 ms", "30 ms"}, 3));
    p.push_back(
        params::choice("fx_cv_release", "VCA Comp Release", "fx_cv", {"0.1 s", "0.3 s", "0.6 s", "1.2 s", "Auto"}, 1));
    addF(p, "cv_makeup", "VCA Comp Makeup", "fx_cv_2", 0.0f, 18.0f, 0.0f, "dB");
    addF(p, "cv_knee", "VCA Comp Knee", "fx_cv_2", 0.0f, 12.0f, 3.0f, "dB");
    addHz(p, "cv_schp", "VCA Comp Sidechain HP", "fx_cv_2", 20.0f, 500.0f, 20.0f, 0.4f);
    p.push_back(params::choice("fx_cv_key", "VCA Comp Key", "fx_cv_2", {"Internal", "External"}, 0));
    {
        std::vector<std::string> names;
        for (int i = 0; i < static_cast<int>(crucible::fx::ChannelMode::Count); ++i)
            names.emplace_back(crucible::fx::channelModeName(static_cast<crucible::fx::ChannelMode>(i)));
        p.push_back(params::choice("fx_cv_channels", "VCA Comp Channels", "fx_cv_2", names, 0));
    }
    addF(p, "cv_link", "VCA Comp Link", "fx_cv_2", 0.0f, 100.0f, 100.0f, "%");
    p.push_back(params::toggle("fx_cv_console", "VCA Comp Console Output", "fx_cv_3", false));
    addF(p, "cv_drive", "VCA Comp Console Drive", "fx_cv_3", 0.0f, 24.0f, 6.0f, "dB");
    p.push_back(params::integer("fx_pos_cv", "VCA Comp Slot", "fx_cv_3", 1, kNumModuleFx, kNumFx + 3));
    d.onParam = "fx_cv_on";
    d.positionParam = "fx_pos_cv";
    d.capabilities = {"gain_reduction", "latency"};
    d.pages.push_back({"fx_cv", "VCA Comp", "fx", "VCA Bus Compressor", "Threshold, ratio, attack and release."});
    d.pages.push_back(
        {"fx_cv_2", "VCA Comp 2", "fx", "VCA Bus Compressor Sidechain", "Makeup, knee, sidechain, key and stereo link."});
    d.pages.push_back(
        {"fx_cv_3", "VCA Comp 3", "fx", "VCA Bus Compressor Output", "Console output stage and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.comp_vca")); };
    return d;
}

} // namespace crucible::modules::fx
