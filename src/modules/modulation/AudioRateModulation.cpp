// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/Modulators.h>
#include <crucible/module/ParamBuilder.h>

#include <string>

// Audio-rate modulation (P12, section 2.10): `lfoN_audio` turns an LFO into a band-limited audio-rate oscillator on the
// voice's audio bus (a route source, kSourceAudio + n), and `filterN_fm_src` / `filterN_fm_amt` feed an audio source
// straight into the cutoff of a filter, one sample at a time. Registered last; every default leaves the voice as it was.

namespace crucible::modules
{

ModuleDescriptor audioRateModulation(const VoiceGraphShape& shape)
{
    ModuleDescriptor d;
    d.id = "mod.audio_rate";
    d.name = "Audio-Rate Modulation";
    d.kind = ModuleKind::Voice;
    d.category = "Modulation";
    d.cpu = CpuClass::Light;
    d.sectionHint = "mod";
    d.description =
        "Band-limited audio-rate LFOs (sine, triangle, saw, square, pulse) on the voice's audio bus and per-sample "
        "filter cutoff modulation from an audio source. Audio sources used by routes to pitch, FM amount, pulse "
        "width, wavetable position, filter cutoff or amplitude run per sample; other targets get the chunk mean.";
    for (int l = 1; l <= kNumLfos; ++l)
    {
        const std::string p = "lfo" + std::to_string(l) + "_audio";
        const std::string label = "LFO " + std::to_string(l);
        d.pages.push_back(
            {p, label + " Audio Rate", "mod", label + " Audio Rate",
             "Runs the LFO's shape as a band-limited audio-rate oscillator on the audio bus: at a fixed frequency "
             "or as a ratio of the note's frequency. Sine, triangle, saw up, square, saw down and 25 % pulse "
             "shapes are band-limited; other shapes play as sine."});
        d.params.push_back(params::choice(p, label + " Audio Rate", p, {"Off", "Frequency", "Key Ratio"}, 0));
        d.params.push_back(
            params::floating(p + "_hz", label + " Audio Frequency", p, 1.0f, 20000.0f, 440.0f, "Hz", 0.3f, Smoothing::None));
        d.params.push_back(
            params::floating(p + "_ratio", label + " Key Ratio", p, 0.125f, 16.0f, 1.0f, {}, 0.4f, Smoothing::None));
    }
    std::vector<std::string> sources = {"Off"};
    for (int l = 1; l <= kNumLfos; ++l)
        sources.push_back("LFO " + std::to_string(l) + " Audio");
    for (int o = 1; o <= 8; ++o)
        sources.push_back("Osc " + std::to_string(o));
    sources.push_back("Mod Env");
    sources.push_back("Env 3");
    sources.push_back("Env 4");
    for (int f = 1; f <= shape.filters; ++f)
    {
        const std::string prefix = f == 1 ? std::string("filter") : "filter" + std::to_string(f);
        const std::string label = "Filter " + std::to_string(f);
        const std::string page = prefix + "_fm";
        d.pages.push_back(
            {page, label + " FM", "filter", label + " FM",
             "Audio-rate cutoff modulation: the chosen audio source moves the cutoff by amount x 5 octaves per "
             "unit of signal, one sample at a time (the nonlinear models at a control rate of 4 samples)."});
        d.params.push_back(params::choice(prefix + "_fm_src", label + " FM Source", page, sources, 0));
        d.params.push_back(
            params::floating(prefix + "_fm_amt", label + " FM Amount", page, -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    }
    return d;
}

} // namespace crucible::modules
