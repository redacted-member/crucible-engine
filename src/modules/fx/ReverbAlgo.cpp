// SPDX-License-Identifier: MIT
// fx.reverb_algo: Room, Hall, Chamber, Plate and Ambience reverbs on the FDN, early reflection and plate cores
// (include/crucible/fx/Fdn.h).
#include "FxModuleSupport.h"

#include <crucible/fx/Fdn.h>
#include <crucible/module/LatencyReporter.h>

namespace crucible::modules::fx
{
namespace
{
class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_ra_mix")), algorithm(localIndex(d, "fx_ra_algo")), decay(localIndex(d, "fx_ra_decay")),
          size(localIndex(d, "fx_ra_size")), predelay(localIndex(d, "fx_ra_predelay")),
          density(localIndex(d, "fx_ra_density")), early(localIndex(d, "fx_ra_early")),
          lowMult(localIndex(d, "fx_ra_lomult")), highMult(localIndex(d, "fx_ra_himult")),
          lowHz(localIndex(d, "fx_ra_lofreq")), highHz(localIndex(d, "fx_ra_hifreq")),
          modDepth(localIndex(d, "fx_ra_moddepth")), modRate(localIndex(d, "fx_ra_modrate")),
          width(localIndex(d, "fx_ra_width")), lowCut(localIndex(d, "fx_ra_lowcut")),
          highCut(localIndex(d, "fx_ra_highcut")), freeze(localIndex(d, "fx_ra_freeze")), duck(localIndex(d, "fx_ra_duck")),
          duckRelease(localIndex(d, "fx_ra_duckrel"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        reverb.prepare(c.sampleRate, c.quality == Quality::High ? 32 : 16, c.seed);
    }
    void reset() override { reverb.reset(); }
    // Switched off: forget the tail a bounded number of samples per block, so that switching on replays nothing.
    void deactivate() override { reverb.deactivate(); }
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        using namespace crucible::fx;
        ReverbSettings s;
        s.algorithm =
            static_cast<ReverbAlgorithm>(std::clamp(in.choice(algorithm), 0, static_cast<int>(ReverbAlgorithm::Count) - 1));
        s.mix = in.value(mix);
        s.decaySeconds = in.value(decay);
        s.size = in.value(size);
        s.predelayMs = in.value(predelay);
        s.density = in.value(density);
        s.early = in.value(early);
        s.lowMult = in.value(lowMult);
        s.highMult = in.value(highMult);
        s.lowHz = in.value(lowHz);
        s.highHz = in.value(highHz);
        s.modDepth = in.value(modDepth);
        s.modRate = in.value(modRate);
        s.width = in.value(width);
        s.lowCutHz = in.value(lowCut);
        s.highCutHz = in.value(highCut);
        s.freeze = in.on(freeze);
        s.duck = in.value(duck);
        s.duckReleaseMs = in.value(duckRelease);
        reverb.process(l, r, n, s);
    }

    double tailSeconds() const override { return reverb.tailSeconds(); }
    bool tailInfinite() const override { return reverb.tailInfinite(); }

private:
    const int mix, algorithm, decay, size, predelay, density, early, lowMult, highMult, lowHz, highHz, modDepth, modRate,
        width, lowCut, highCut, freeze, duck, duckRelease;
    crucible::fx::AlgorithmicReverb reverb;
};
} // namespace

ModuleDescriptor reverbAlgoEffect()
{
    auto d = effectDescriptor(
        "fx.reverb_algo", "Algorithmic Reverb", "Reverb", CpuClass::Medium,
        "Five algorithmic reverbs. Room, Hall, Chamber and Ambience are a 16-line feedback delay network "
        "(32 lines at High quality) with Jot's frequency-dependent absorption, so the decay time is the set "
        "one at low, middle and high frequencies, behind an all-pass input diffuser and early reflections "
        "(image-source shoebox taps for Room and Chamber, velvet-noise taps for Hall and Ambience); Plate "
        "is Dattorro's figure-eight all-pass tank with the same absorption. Smooth random modulation of "
        "the delays, pre-delay, size, density, low and high decay multipliers with their crossovers, "
        "freeze (the tail is held at constant energy), ducking, stereo width, wet low and high cut and "
        "mix. Switching the algorithm fades out, clears and fades in. The tail length is reported.");
    auto& p = d.params;
    addOn(p, "ra", "Algo Reverb", "fx_ra", 0.3f);
    p.push_back(
        params::choice("fx_ra_algo", "Algo Reverb Algorithm", "fx_ra", {"Room", "Hall", "Chamber", "Plate", "Ambience"}, 1));
    addF(p, "ra_decay", "Algo Reverb Decay", "fx_ra", 0.1f, 60.0f, 2.5f, "s", 0.35f);
    addF(p, "ra_size", "Algo Reverb Size", "fx_ra", 0.0f, 1.0f, 0.5f);
    addF(p, "ra_predelay", "Algo Reverb Predelay", "fx_ra", 0.0f, 500.0f, 15.0f, "ms", 0.5f);
    addF(p, "ra_density", "Algo Reverb Density", "fx_ra_2", 0.0f, 1.0f, 0.7f);
    addF(p, "ra_early", "Algo Reverb Early Level", "fx_ra_2", 0.0f, 1.0f, 0.5f);
    addF(p, "ra_lomult", "Algo Reverb Low Decay x", "fx_ra_2", 0.1f, 4.0f, 1.0f, "x", 0.5f);
    addF(p, "ra_himult", "Algo Reverb High Decay x", "fx_ra_2", 0.05f, 2.0f, 0.7f, "x", 0.5f);
    addHz(p, "ra_lofreq", "Algo Reverb Low Crossover", "fx_ra_2", 50.0f, 1000.0f, 250.0f, 0.4f);
    addHz(p, "ra_hifreq", "Algo Reverb Damping Frequency", "fx_ra_2", 1000.0f, 16000.0f, 4000.0f, 0.4f);
    addF(p, "ra_moddepth", "Algo Reverb Modulation Depth", "fx_ra_3", 0.0f, 1.0f, 0.3f);
    addF(p, "ra_modrate", "Algo Reverb Modulation Rate", "fx_ra_3", 0.05f, 5.0f, 1.0f, "Hz", 0.5f);
    addF(p, "ra_width", "Algo Reverb Width", "fx_ra_3", 0.0f, 1.0f, 1.0f);
    addHz(p, "ra_lowcut", "Algo Reverb Low Cut", "fx_ra_3", 20.0f, 1000.0f, 80.0f, 0.4f);
    addHz(p, "ra_highcut", "Algo Reverb High Cut", "fx_ra_3", 1000.0f, 20000.0f, 20000.0f, 0.4f);
    p.push_back(params::toggle("fx_ra_freeze", "Algo Reverb Freeze", "fx_ra_4", false));
    addF(p, "ra_duck", "Algo Reverb Ducking", "fx_ra_4", 0.0f, 1.0f, 0.0f);
    addF(p, "ra_duckrel", "Algo Reverb Duck Release", "fx_ra_4", 20.0f, 2000.0f, 250.0f, "ms", 0.5f);
    p.push_back(params::integer("fx_pos_ra", "Algo Reverb Slot", "fx_ra_4", 1, kNumModuleFx, kNumFx + 5));
    d.onParam = "fx_ra_on";
    d.positionParam = "fx_pos_ra";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_ra", "Algo Reverb", "fx", "Algorithmic Reverb", "Algorithm, decay, size, pre-delay and mix."});
    d.pages.push_back(
        {"fx_ra_2", "Algo Reverb 2", "fx", "Algorithmic Reverb Character",
         "Density, early reflections and the low and high decay multipliers with their crossover frequencies."});
    d.pages.push_back({"fx_ra_3", "Algo Reverb 3", "fx", "Algorithmic Reverb Output",
                       "Modulation, stereo width and the wet low and high cut."});
    d.pages.push_back(
        {"fx_ra_4", "Algo Reverb 4", "fx", "Algorithmic Reverb Freeze", "Freeze, ducking and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.reverb_algo")); };
    return d;
}

} // namespace crucible::modules::fx
