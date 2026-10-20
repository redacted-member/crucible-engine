// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <algorithm>
#include <crucible/Filters.h>
#include <crucible/ModMatrix.h>
#include <crucible/UserWavetables.h>
#include <crucible/module/ParamBuilder.h>

// The wavetable voice: three warpable wavetable oscillators with unison, sub, noise, two filters with routing,
// four envelopes, four LFOs, the sixteen-slot matrix and voice behaviour. The DSP is Voice / SynthEngine; this
// file declares the voice's parameters.

namespace crucible::modules
{
namespace
{
const std::vector<std::string>& warpNames()
{
    static const std::vector<std::string> names = {"Off",    "FM",      "Sync Hard", "Sync Soft", "Bend +",
                                                   "Bend -", "Mirror",  "PWM",       "Quantize",  "Asym",
                                                   "Flip",   "Formant", "Smear",     "Ring",      "Fold"};
    return names;
}

// Where a source goes in split routing: each filter, all of them, or past the filters. For two filters this is
// the original "Filter 1, Filter 2, Both, Dry".
std::vector<std::string> routeChoices(int filters)
{
    std::vector<std::string> out;
    for (int f = 1; f <= filters; ++f)
        out.push_back("Filter " + std::to_string(f));
    out.push_back(filters == 2 ? "Both" : "All");
    out.push_back("Dry");
    return out;
}

std::vector<std::string> oscNames(int count)
{
    std::vector<std::string> out;
    for (int o = 1; o <= count; ++o)
        out.push_back("Osc " + std::to_string(o));
    return out;
}

void addWarp(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page,
             int defaultRoute, int defaultSource, const VoiceGraphShape& shape)
{
    out.push_back(params::choice(prefix + "_warp", label + " Warp", page, warpNames(), 0));
    out.push_back(params::floating(prefix + "_warp_a", label + " Warp A", page, 0.0f, 1.0f, 0.0f));
    out.push_back(params::floating(prefix + "_warp_b", label + " Warp B", page, 0.0f, 1.0f, 0.0f));
    out.push_back(params::floating(prefix + "_pan", label + " Pan", page, -1.0f, 1.0f, 0.0f));
    out.push_back(params::choice(prefix + "_route", label + " Route", page, routeChoices(shape.filters), defaultRoute));
    // The default must be a valid choice: a one-oscillator layout has only "Osc 1".
    const int source = std::clamp(defaultSource, 0, std::max(shape.oscillators - 1, 0));
    out.push_back(params::choice(prefix + "_warp_src", label + " Warp Source", page, oscNames(shape.oscillators), source));
}

void addStack(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page)
{
    out.push_back(params::choice(prefix + "_uni_mode", label + " Stack Mode", page,
                                 {"Linear", "Super", "Exp", "Octaves", "Fifths", "Minor", "Major", "Random"}, 0));
    out.push_back(params::floating(prefix + "_uni_blend", label + " Unison Blend", page, 0.0f, 1.0f, 1.0f));
    out.push_back(
        params::floating(prefix + "_uni_rand", label + " Random Phase", page, 0.0f, 1.0f, 1.0f, {}, 1.0f, Smoothing::None));
    out.push_back(params::floating(prefix + "_uni_scatter", label + " Voice Scatter", page, 0.0f, 1.0f, 0.0f));
}

void addOscillator(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page,
                   float defaultLevel)
{
    out.push_back(params::choice(prefix + "_table", label + " Table", page, allTableNames(), 0));
    out.push_back(params::floating(prefix + "_pos", label + " Position", page, 0.0f, 1.0f, 0.0f));
    out.push_back(params::integer(prefix + "_semi", label + " Semitone", page, -24, 24, 0, "st"));
    out.push_back(params::floating(prefix + "_fine", label + " Fine", page, -100.0f, 100.0f, 0.0f, "ct"));
    out.push_back(params::floating(prefix + "_level", label + " Level", page, 0.0f, 1.0f, defaultLevel));
    out.push_back(
        params::floating(prefix + "_phase", label + " Phase", page, 0.0f, 360.0f, 0.0f, "deg", 1.0f, Smoothing::None));
}

void addUnison(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page)
{
    out.push_back(params::integer(prefix + "_voices", label + " Unison Voices", page, 1, 8, 1));
    out.push_back(params::floating(prefix + "_detune", label + " Unison Detune", page, 0.0f, 100.0f, 20.0f, "ct", 0.6f));
    out.push_back(params::floating(prefix + "_spread", label + " Unison Spread", page, 0.0f, 1.0f, 1.0f));
}

void addEnvelope(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page,
                 float decay, float sustain, float release)
{
    out.push_back(
        params::floating(prefix + "_attack", label + " Attack", page, 0.001f, 10.0f, 0.005f, "s", 0.3f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_decay", label + " Decay", page, 0.001f, 10.0f, decay, "s", 0.3f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_sustain", label + " Sustain", page, 0.0f, 1.0f, sustain, {}, 1.0f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_release", label + " Release", page, 0.001f, 10.0f, release, "s", 0.3f, Smoothing::None));
    out.push_back(params::floating(prefix + "_hold", label + " Hold", page, 0.0f, 10.0f, 0.0f, "s", 0.3f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_curve", label + " Curve", page, -1.0f, 1.0f, -0.8f, {}, 1.0f, Smoothing::None));
    // The attack has its own curve: 0 (linear) reaches the peak at the attack time.
    auto attackCurve = params::floating(prefix + "_attack_curve", label + " Attack Curve", page + "_attack", -1.0f, 1.0f,
                                        0.0f, {}, 1.0f, Smoothing::None);
    attackCurve.since = 2;
    out.push_back(attackCurve);
}

void addLfo(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page)
{
    out.push_back(
        params::floating(prefix + "_rate", label + " Rate", page, 0.02f, 30.0f, 2.0f, "Hz", 0.4f, Smoothing::None));
    out.push_back(params::choice(prefix + "_shape", label + " Shape", page, params::lfoShapeNames(), 0));
    out.push_back(params::floating(prefix + "_amount", label + " Amount", page, 0.0f, 1.0f, 1.0f));
}

void addLfoExt(std::vector<ParamDef>& out, const std::string& prefix, const std::string& label, const std::string& page)
{
    out.push_back(params::choice(prefix + "_trig", label + " Trigger", page, {"Retrigger", "Free", "One-Shot"}, 0));
    out.push_back(params::choice(prefix + "_sync", label + " Sync", page, params::tempoSyncNames(), 0));
    out.push_back(
        params::floating(prefix + "_fade", label + " Fade In", page, 0.0f, 10.0f, 0.0f, "s", 0.4f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_delay", label + " Delay", page, 0.0f, 10.0f, 0.0f, "s", 0.4f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_phase", label + " Start Phase", page, 0.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    out.push_back(
        params::floating(prefix + "_smooth", label + " Smooth", page, 0.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
}

void addModSlot(std::vector<ParamDef>& out, int slot, const std::string& page, const std::string& shapePage,
                const std::vector<std::string>& sourceNames)
{
    const auto prefix = "mod" + std::to_string(slot);
    const auto label = "Mod " + std::to_string(slot);
    out.push_back(params::choice(prefix + "_src", label + " Source", page, sourceNames, 0));
    auto dst = params::choice(prefix + "_dst", label + " Destination", page, legacyModDestNames(), 0);
    dst.destinationChoice = true; // extended with every registered modulation target when the map is built
    out.push_back(dst);
    out.push_back(params::floating(prefix + "_amt", label + " Amount", page, -1.0f, 1.0f, 0.0f));
    out.push_back(params::choice(prefix + "_pol", label + " Polarity", shapePage, {"Native", "Unipolar", "Bipolar"}, 0));
    out.push_back(params::floating(prefix + "_curve", label + " Curve", shapePage, -1.0f, 1.0f, 0.0f));
}

} // namespace

ModuleDescriptor wavetableVoice(const VoiceGraphShape& shape, int macros)
{
    ModuleDescriptor d;
    d.id = "voice.wavetable";
    d.name = "Wavetable Voice";
    d.kind = ModuleKind::Voice;
    d.category = "Voice";
    d.version = 2; // 2: attack curves, retuned filters and attacks (docs/SOUND_CHANGES.md); patches need no migration
    d.cpu = CpuClass::Heavy;
    d.description = "Three wavetable oscillators (warps, unison stacks), sub, noise, two filters with routing, "
                    "four envelopes, four LFOs and a sixteen-slot modulation matrix.";
    auto& p = d.params;

    addOscillator(p, "osc1", "Osc 1", "osc1", 0.8f);
    addOscillator(p, "osc2", "Osc 2", "osc2", 0.0f);

    addUnison(p, "osc1_uni", "Osc 1", "unison");
    addUnison(p, "osc2_uni", "Osc 2", "unison");

    p.push_back(params::floating("sub_level", "Sub Level", "mix", 0.0f, 1.0f, 0.0f));
    p.push_back(params::choice("sub_octave", "Sub Octave", "mix", {"-1 Oct", "-2 Oct"}, 0));
    p.push_back(params::floating("noise_level", "Noise Level", "mix", 0.0f, 1.0f, 0.0f));
    p.push_back(params::floating("master_gain", "Master Gain", "mix", -60.0f, 6.0f, -6.0f, "dB", 2.5f));
    p.push_back(params::choice("sub_wave", "Sub Wave", "mix", {"Sine", "Triangle", "Square", "Saw"}, 0));
    p.push_back(params::choice("noise_type", "Noise Type", "mix", {"White", "Pink", "Brown", "Crackle"}, 0));

    p.push_back(params::choice("filter_type", "Filter Type", "filter", filterModelNames(), 0));
    p.push_back(params::floating("filter_cutoff", "Cutoff", "filter", 20.0f, 20000.0f, 8000.0f, "Hz", 0.3f, Smoothing::Log));
    p.push_back(params::floating("filter_res", "Resonance", "filter", 0.0f, 1.0f, 0.15f));
    p.push_back(params::floating("filter_drive", "Drive", "filter", 0.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter_env", "Filter Env Amount", "filter", -1.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter_key", "Filter Keytrack", "filter", 0.0f, 1.0f, 0.0f));

    addEnvelope(p, "amp", "Amp", "amp_env", 0.3f, 0.8f, 0.25f);
    addEnvelope(p, "menv", "Mod Env", "mod_env", 0.4f, 0.5f, 0.3f);

    addLfo(p, "lfo1", "LFO 1", "lfos");
    addLfo(p, "lfo2", "LFO 2", "lfos");
    addLfo(p, "lfo3", "LFO 3", "lfo_3_4");
    addLfo(p, "lfo4", "LFO 4", "lfo_3_4");
    addLfoExt(p, "lfo1", "LFO 1", "lfo1_ext");
    addLfoExt(p, "lfo2", "LFO 2", "lfo2_ext");
    addLfoExt(p, "lfo3", "LFO 3", "lfo3_ext");
    addLfoExt(p, "lfo4", "LFO 4", "lfo4_ext");
    addEnvelope(p, "env3", "Env 3", "env3", 0.4f, 0.5f, 0.3f);
    addEnvelope(p, "env4", "Env 4", "env4", 0.4f, 0.5f, 0.3f);
    for (int slot = 1; slot <= kNumModSlots; ++slot)
    {
        // Slots pair up on pages "mod_1_2" .. "mod_15_16"; polarity and curve sit three per "mod_shape_N" page.
        const int pair = (slot - 1) / 2;
        const std::string page = "mod_" + std::to_string(2 * pair + 1) + "_" + std::to_string(2 * pair + 2);
        addModSlot(p, slot, page, "mod_shape_" + std::to_string((slot - 1) / 3 + 1), modSourceNamesFor(macros));
    }

    // Third oscillator, warp, stacking and voice pages.
    addWarp(p, "osc1", "Osc 1", "osc1_warp", 0, 1, shape);
    addWarp(p, "osc2", "Osc 2", "osc2_warp", 1, 0, shape);
    addOscillator(p, "osc3", "Osc 3", "osc3", 0.0f);
    addWarp(p, "osc3", "Osc 3", "osc3_warp", 2, 0, shape);
    addUnison(p, "osc3_uni", "Osc 3", "unison3");
    addStack(p, "osc1", "Osc 1", "stack1");
    addStack(p, "osc2", "Osc 2", "stack2");
    addStack(p, "osc3", "Osc 3", "stack3");

    // Filters
    p.push_back(params::toggle("filter_on", "Filter 1 On", "filter1_out", true));
    p.push_back(params::floating("filter_pan", "Filter 1 Pan", "filter1_out", -1.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter_mix", "Filter 1 Mix", "filter1_out", 0.0f, 1.0f, 1.0f));
    p.push_back(params::choice("filter2_type", "Filter 2 Type", "filter2", filterModelNames(), 0));
    p.push_back(params::floating("filter2_cutoff", "Filter 2 Cutoff", "filter2", 20.0f, 20000.0f, 8000.0f, "Hz", 0.3f,
                                 Smoothing::Log));
    p.push_back(params::floating("filter2_res", "Filter 2 Resonance", "filter2", 0.0f, 1.0f, 0.15f));
    p.push_back(params::floating("filter2_drive", "Filter 2 Drive", "filter2", 0.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter2_env", "Filter 2 Env Amount", "filter2", -1.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter2_key", "Filter 2 Keytrack", "filter2", 0.0f, 1.0f, 0.0f));
    p.push_back(params::toggle("filter2_on", "Filter 2 On", "filter2_out", false));
    p.push_back(params::floating("filter2_pan", "Filter 2 Pan", "filter2_out", -1.0f, 1.0f, 0.0f));
    p.push_back(params::floating("filter2_mix", "Filter 2 Mix", "filter2_out", 0.0f, 1.0f, 1.0f));
    p.push_back(params::choice("filter_route", "Filter Routing", "routing", {"Serial", "Parallel", "Split"}, 0));
    p.push_back(params::choice("sub_route", "Sub Route", "routing", routeChoices(shape.filters), shape.filters));
    p.push_back(params::choice("noise_route", "Noise Route", "routing", routeChoices(shape.filters), shape.filters));

    p.push_back(params::choice("voice_mode", "Voice Mode", "voice", {"Poly", "Mono", "Legato"}, 0));
    p.push_back(params::floating("voice_glide", "Glide", "voice", 0.0f, 2.0f, 0.0f, "s", 0.4f, Smoothing::None));
    p.push_back(params::floating("voice_bend", "Bend Range", "voice", 0.0f, 24.0f, 2.0f, "st", 1.0f, Smoothing::None));
    p.push_back(params::integer("voice_poly", "Polyphony", "voice", 1, 16, 16));
    p.push_back(params::floating("voice_drift", "Analog Drift", "voice", 0.0f, 1.0f, 0.0f));
    p.push_back(params::integer("voice_transpose", "Transpose", "voice", -24, 24, 0, "st"));

    for (const auto& [id, name] :
         std::vector<std::pair<std::string, std::string>>{{"amp_env_attack", "Amp Attack Curve"},
                                                          {"mod_env_attack", "Mod Env Attack Curve"},
                                                          {"env3_attack", "Env 3 Attack Curve"},
                                                          {"env4_attack", "Env 4 Attack Curve"}})
        d.pages.push_back(
            {id, name, "env", name,
             "LCD draws the attack segment; the strip bends it (linear reaches the peak at the attack time)."});

    // Slots beyond the original three oscillators and two filters, same parameter layout per slot.
    const auto page = [&d](const std::string& id, const std::string& name, const std::string& section,
                           const std::string& title, const std::string& text)
    { d.pages.push_back({id, name, section, title, text}); };
    // Slots without fixed matrix destinations are modulated per voice through the generic targets.
    for (int o = 4; o <= shape.oscillators; ++o)
    {
        const size_t first = p.size();
        const auto n = std::to_string(o);
        const auto prefix = "osc" + n;
        const auto label = "Osc " + n;
        page(prefix, label, "osc2", "Oscillator " + n, "LCD shows the live wavetable frame at the current position.");
        page(prefix + "_warp", label + " Warp", "osc2", "Oscillator " + n + " Warp", "LCD shows the warped waveform.");
        page("unison" + n, "Unison " + n, "voice", "Unison, Oscillator " + n, "LCD plots the detuned voice stack.");
        page("stack" + n, label + " Stack", "osc2", "Oscillator " + n + " Unison Stack", "LCD plots the stack offsets.");
        addOscillator(p, prefix, label, prefix, 0.0f);
        addWarp(p, prefix, label, prefix + "_warp", shape.filters, 0, shape);
        addUnison(p, prefix + "_uni", label, "unison" + n);
        addStack(p, prefix, label, "stack" + n);
        for (size_t i = first; i < p.size(); ++i)
        {
            const auto& id = p[i].id;
            for (const char* suffix : {"_pos", "_semi", "_level", "_warp_a", "_warp_b", "_pan"})
                if (id == prefix + suffix)
                    p[i] = params::voiceModulatable(std::move(p[i]));
        }
    }
    for (int f = 3; f <= shape.filters; ++f)
    {
        const auto n = std::to_string(f);
        const auto prefix = "filter" + n;
        const auto label = "Filter " + n;
        const size_t first = p.size();
        page(prefix, label, "filter", "Filter " + n, "LCD draws the filter magnitude response.");
        page(prefix + "_out", label + " Out", "filter", "Filter " + n + " Output",
             "LCD shows the stereo placement and mix.");
        p.push_back(params::choice(prefix + "_type", label + " Type", prefix, filterModelNames(), 0));
        p.push_back(params::floating(prefix + "_cutoff", label + " Cutoff", prefix, 20.0f, 20000.0f, 8000.0f, "Hz", 0.3f,
                                     Smoothing::Log));
        p.push_back(params::floating(prefix + "_res", label + " Resonance", prefix, 0.0f, 1.0f, 0.15f));
        p.push_back(params::floating(prefix + "_drive", label + " Drive", prefix, 0.0f, 1.0f, 0.0f));
        p.push_back(params::floating(prefix + "_env", label + " Env Amount", prefix, -1.0f, 1.0f, 0.0f));
        p.push_back(params::floating(prefix + "_key", label + " Keytrack", prefix, 0.0f, 1.0f, 0.0f));
        p.push_back(params::toggle(prefix + "_on", label + " On", prefix + "_out", false));
        p.push_back(params::floating(prefix + "_pan", label + " Pan", prefix + "_out", -1.0f, 1.0f, 0.0f));
        p.push_back(params::floating(prefix + "_mix", label + " Mix", prefix + "_out", 0.0f, 1.0f, 1.0f));
        for (size_t i = first; i < p.size(); ++i)
            for (const char* suffix : {"_cutoff", "_res", "_drive", "_mix", "_pan"})
                if (p[i].id == prefix + suffix)
                    p[i] = params::voiceModulatable(std::move(p[i]));
    }

    return d;
}

} // namespace crucible::modules
