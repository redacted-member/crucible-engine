// SPDX-License-Identifier: MIT
#include <crucible/Modulators.h>
#include <crucible/SynthEngine.h>
#include <crucible/graph/Compiler.h>
#include <crucible/module/ModuleRack.h>
#include "TestSupport.h"

#include <chrono>
#include <ctime>
#include <cstdio>
#include <functional>

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Config
{
    const char* name;
    int notes;
    std::function<void(std::vector<float>&)> setup;
};

void allFx(std::vector<float>& v)
{
    for (const auto& p : ParameterMap::params())
        if (p.id.rfind("fx_", 0) == 0 && p.id.size() > 3 && p.id.compare(p.id.size() - 3, 3, "_on") == 0)
            setValue(v, p.id, 1.0f);
}

void unison(std::vector<float>& v, int voices, int oscs)
{
    for (int o = 1; o <= 3; ++o)
    {
        const auto p = "osc" + std::to_string(o);
        setValue(v, p + "_uni_voices", static_cast<float>(voices));
        setValue(v, p + "_level", o <= oscs ? 0.5f : 0.0f);
        setValue(v, p + "_uni_detune", 30.0f);
    }
}

// The voice configuration that stands for the reference sound in the plan's CPU table.
constexpr const char* kReferenceRow = "16 voices, 1 osc x 8 unison, no FX";

// CPU targets of docs/plans/ENGINE-COMMUNITY-PLAN.md section 2.12 (48 kHz, 128-sample blocks, Apple M-class core;
// this bench runs 512-sample blocks). A row without a measurement names the package that makes it measurable; the
// run fails (exit code 1) when a measured row is above its target + 20 %.
struct PlanRow
{
    const char* name;
    double targetPercent;
    const char* pendingUntil; // empty = measured today
    double measuredPercent;   // negative = not measured
};

// The plan's analog voice rows: 16 voices of two osc.analog slots, the transistor ladder, analog RC envelopes and voice
// cards (oscillator, filter, envelope and amplifier tolerances), best of two runs of 5 s.
double analogVoicePercent(Quality quality)
{
    auto v = ParameterMap::defaultValues();
    for (int o = 1; o <= 2; ++o)
    {
        const auto p = "osc" + std::to_string(o);
        setValue(v, p + "_type", 5.0f);
        setValue(v, p + "_level", 0.4f);
        setValue(v, p + "_fine", o == 1 ? -6.0f : 6.0f);
    }
    setValue(v, "osc3_level", 0.0f);
    setValue(v, "filter_on", 1.0f);
    setValue(v, "filter_type", 17.0f);
    setValue(v, "filter_cutoff", 2500.0f);
    setValue(v, "filter_res", 0.4f);
    setValue(v, "filter_env", 0.3f);
    setValue(v, "amp_sustain", 1.0f);
    for (const char* p : {"amp", "menv"})
        setValue(v, std::string(p) + "_mode", 1.0f);
    for (const char* p : {"voice_card_osc", "voice_card_filter", "voice_card_env", "voice_card_level"})
        setValue(v, p, 1.0f);
    double best = 1.0e9;
    for (int rep = 0; rep < 2; ++rep)
    {
        EngineConfig cfg;
        cfg.quality = quality;
        CRUCIBLE_HEAP(SynthEngine, engine, cfg);
        prepare(engine, v);
        for (int n = 0; n < 16; ++n)
            engine.noteOn(36 + n * 3, 1.0f);
        renderBlocks(engine, 4800);
        const auto start = std::chrono::steady_clock::now();
        renderBlocks(engine, static_cast<int>(kRate * 5.0));
        best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
    }
    return 100.0 * best / 5.0;
}

int printPlanTable(std::vector<PlanRow>& rows)
{
    std::printf("\nPlan CPU targets (section 2.12), %% of one core; the bench fails at target + 20 %%\n");
    std::printf("%-72s %8s %9s  %s\n", "configuration", "target", "fails at", "measured");
    int failed = 0;
    for (const auto& r : rows)
    {
        std::printf("%-72s %7.1f%% %8.1f%%  ", r.name, r.targetPercent, r.targetPercent * 1.2);
        if (r.measuredPercent < 0.0)
            std::printf("pending (%s)\n", r.pendingUntil);
        else
        {
            const bool over = r.measuredPercent > r.targetPercent * 1.2;
            failed += over ? 1 : 0;
            std::printf("%.2f%%  %s\n", r.measuredPercent, over ? "OVER" : "ok");
        }
    }
    return failed == 0 ? 0 : 1;
}
} // namespace

int main(int argc, char** argv)
{
    // Optional first argument: only run the voice configurations whose name contains it (skips the effect table);
    // "plan" runs just the configuration behind the plan's CPU table and prints that table (exit code 1 when over).
    const std::string only = argc > 1 ? argv[1] : "";
    std::vector<PlanRow> plan = {
        {"16 voices: 2 osc.analog, ladder at 2x, analog envelopes, voice cards", 6.0, "needs P7, P9, P11", -1.0},
        {"the same at High quality", 12.0, "needs P7, P9, P11", -1.0},
        {"Reference patch 16 x 8 unison via classic.voice (today: the hard-wired voice)", 4.0, "", -1.0},
        {"graph of 32 fine-grained nodes per voice, 16 voices", 10.0, "needs P4", -1.0},
        {"each extra FX instance (today: heaviest single module)", 1.0, "run the bench without arguments", -1.0},
    };
    const ScopedFlushDenormals noDenormals;
    if (only == "mod")
    {
        // The cost of the modulator bank and of audio-rate modulation: the plan's analog voice (16 voices, two osc.analog
        // slots, transistor ladder at Standard, analog RC envelopes, voice cards) with modulators and audio-rate routes
        // added, best of three runs of 4 s each, as percent of one core. Modulators are routed to the cutoff, so that they
        // run.
        struct Case
        {
            const char* name;
            std::function<void(std::vector<float>&, SynthEngine*)> setup;
        };
        const auto baseline = [](std::vector<float>& v)
        {
            for (int o = 1; o <= 2; ++o)
            {
                const auto p = "osc" + std::to_string(o);
                setValue(v, p + "_type", 5.0f);
                setValue(v, p + "_level", 0.4f);
                setValue(v, p + "_fine", o == 1 ? -6.0f : 6.0f);
            }
            setValue(v, "osc3_level", 0.0f);
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_type", 17.0f);
            setValue(v, "filter_cutoff", 2500.0f);
            setValue(v, "filter_res", 0.4f);
            setValue(v, "filter_env", 0.3f);
            setValue(v, "amp_sustain", 1.0f);
            for (const char* p : {"amp", "menv"})
                setValue(v, std::string(p) + "_mode", 1.0f);
            for (const char* p : {"voice_card_osc", "voice_card_filter", "voice_card_env", "voice_card_level"})
                setValue(v, p, 1.0f);
        };
        static const ModulatorType kinds[8] = {
            ModulatorType::Lag,   ModulatorType::Combine,  ModulatorType::SampleHold, ModulatorType::Turing,
            ModulatorType::Chaos, ModulatorType::Follower, ModulatorType::LfoTable,   ModulatorType::PitchTracker};
        const auto bank = [&](int count)
        {
            return [count](std::vector<float>& v, SynthEngine* e)
            {
                for (int n = 0; n < count; ++n)
                {
                    const std::string p = "bank" + std::to_string(n + 1);
                    setValue(v, p + "_type", static_cast<float>(kinds[n]));
                    setValue(v, p + "_in_a", static_cast<float>(ModSource::Lfo1));
                    setValue(v, p + "_mode", n == 4 ? 1.0f : 0.0f);
                    setValue(v, p + "_rate", 20.0f);
                    setValue(v, p + "_rise", 0.01f);
                    setValue(v, p + "_fall", 0.05f);
                }
                if (e != nullptr)
                    for (int n = 0; n < count; ++n)
                        e->modRoutes().add({kSourceModulator + n, "filter_cutoff", 0.1f, 2, 0.0f, 0.0f});
            };
        };
        const std::vector<Case> cases = {
            {"baseline (the bank is present, every modulator off)", [](auto&, SynthEngine*) {}},
            {"1 modulator (Lag of LFO 1) to the cutoff", bank(1)},
            {"2 modulators (+ Combine)", bank(2)},
            {"4 modulators (+ S&H, Turing)", bank(4)},
            {"6 modulators (+ Chaos (Lorenz), Follower)", bank(6)},
            {"8 modulators (+ LFO table, Pitch tracker)", bank(8)},
            {"audio-rate LFO (saw, 440 Hz) to the ladder cutoff, batched per sample",
             [](auto& v, SynthEngine* e)
             {
                 setValue(v, "lfo1_audio", 1.0f);
                 setValue(v, "lfo1_shape", 2.0f);
                 if (e != nullptr)
                     e->modRoutes().add({kSourceAudio + kAudioLfoFirst, "filter_cutoff", 0.1f, 0, 0.0f, 0.0f});
             }},
            {"oscillator 1 output to the pulse width of oscillator 2, per sample",
             [](auto& v, SynthEngine* e)
             {
                 setValue(v, "osc1_an_wave", 3.0f);
                 setValue(v, "osc2_an_wave", 1.0f);
                 if (e != nullptr)
                     e->modRoutes().add({kSourceAudio + kAudioOscFirst, "osc2_an_pw", 0.2f, 0, 0.0f, 0.0f});
             }},
            {"oscillator 1 output to the pitch of oscillator 2, per sample",
             [](auto& v, SynthEngine* e)
             {
                 setValue(v, "osc1_an_wave", 3.0f);
                 if (e != nullptr)
                     e->modRoutes().add({kSourceAudio + kAudioOscFirst, "osc2_semi", 0.02f, 0, 0.0f, 0.0f});
             }},
            {"audio-rate LFO to the amplifier (AM), per sample",
             [](auto& v, SynthEngine* e)
             {
                 setValue(v, "lfo1_audio", 1.0f);
                 if (e != nullptr)
                     e->modRoutes().add({kSourceAudio + kAudioLfoFirst, kTargetVoiceAmp, 0.3f, 0, 0.0f, 0.0f});
             }},
        };
        std::printf("Modulator bank and audio-rate modulation, 16 voices of the analog voice (2 x osc.analog, ladder, RC "
                    "envelopes, cards), "
                    "Standard, 48 kHz, 512-sample blocks; percent of one core\n");
        std::printf("%-76s %9s %12s\n", "configuration", "% of core", "vs baseline");
        double base = 0.0;
        for (const auto& c : cases)
        {
            double best = 1.0e9;
            for (int rep = 0; rep < 3; ++rep)
            {
                auto v = ParameterMap::defaultValues();
                baseline(v);
                CRUCIBLE_HEAP(SynthEngine, engine);
                c.setup(v, nullptr);
                prepare(engine, v);
                c.setup(v, &engine);
                for (int n = 0; n < 16; ++n)
                    engine.noteOn(36 + n * 3, 1.0f);
                renderBlocks(engine, 4800);
                // Process CPU time, so that other work on the machine does not count.
                const std::clock_t start = std::clock();
                renderBlocks(engine, static_cast<int>(kRate * 4.0));
                best = std::min(best, static_cast<double>(std::clock() - start) / CLOCKS_PER_SEC);
            }
            const double percent = 100.0 * best / 4.0;
            if (base == 0.0)
                base = percent;
            std::printf("%-76s %8.3f%% %+11.3f%%\n", c.name, percent, percent - base);
        }
        return 0;
    }
    if (only == "graphfilter")
    {
        // Per-voice cost of filter.analog nodes in a graph product (oscillator, filter, output): the product with the
        // filter minus the same product without it, best of three runs, with the quad batching on and off.
        const auto compile = [](bool withFilter, int model, int voices)
        {
            const std::string filter =
                withFilter ? R"(, { "id": "filt", "type": "filter.analog", "params": { "model": )" + std::to_string(model) +
                                 R"(, "cutoff": 2000.0, "res": 0.7, "drive": 0.2, "tolerance": 0.3 } })"
                           : "";
            const std::string edges =
                withFilter ? R"({ "from": "osc.out", "to": "filt.in" }, { "from": "filt.out", "to": "out.in" })"
                           : R"({ "from": "osc.out", "to": "out.in" })";
            const std::string text = R"({"format": "crucible-product", "version": 1, "id": "bench.graphfilter",
                "voice": { "layers": [ { "id": "main", "voices": )" +
                                     std::to_string(voices) +
                                     R"(, "nodes": [ { "id": "osc", "type": "osc.analog", "slot": 1 })" + filter +
                                     R"(, { "id": "out", "type": "mix.out" } ], "edges": [ )" + edges + R"( ] } ] },
                "defaults": { "osc1_level": 0.3 } })";
            graph::ProductDef def;
            std::string error;
            if (!graph::productFromText(text, def, error))
                std::printf("product: %s\n", error.c_str());
            auto compiled = graph::compile(def, error);
            if (!compiled)
                std::printf("compile: %s\n", error.c_str());
            return compiled;
        };
        const auto run = [&](const std::shared_ptr<const graph::CompiledProduct>& compiled, int voices, bool batch)
        {
            double best = 1.0e9;
            for (int rep = 0; rep < 3; ++rep)
            {
                EngineConfig cfg;
                cfg.compiled = compiled;
                cfg.batchAnalogFilters = batch;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                const auto values = compiled->defaultValues();
                engine.prepare(kRate);
                engine.setParameterValues(values.data(), static_cast<int>(values.size()));
                for (int n = 0; n < voices; ++n)
                    engine.noteOn(36 + n * 3, 1.0f);
                renderBlocks(engine, 4800);
                const auto start = std::chrono::steady_clock::now();
                renderBlocks(engine, static_cast<int>(kRate * 3.0));
                best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
            }
            return best;
        };
        static const char* const names[] = {"Transistor ladder", "OTA cascade", "Coupled diode ladder", "Sallen-Key LP",
                                            "Sallen-Key HP",     "OTA SVF",     "OTA 4-pole",           "Passive HP"};
        std::printf("filter.analog in a graph product at Standard, 48 kHz, 512-sample blocks; percent of one core per voice "
                    "(stereo)\n");
        std::printf("%-22s %7s %14s %14s\n", "model", "voices", "single", "batched");
        for (int voices : {4, 8})
            for (int m : {0, 2, 3})
            {
                const auto without = compile(false, m, voices), with = compile(true, m, voices);
                const double base = run(without, voices, true);
                const double single = 100.0 * (run(with, voices, false) - base) / 3.0 / voices;
                const double batched = 100.0 * (run(with, voices, true) - base) / 3.0 / voices;
                std::printf("%-22s %7d %13.3f%% %13.3f%%\n", names[m], voices, single, batched);
            }
        return 0;
    }
    if (only == "analog")
    {
        // Per-voice cost of the analog filter models (FilterModel 17-24) at Standard through the engine: the same patch with
        // the filter switched on and off, best of three runs each. One stereo voice runs two filter lanes; the plan's
        // targets (section 2.6: ladders 0.12 %, diode ladder 0.18 %, 2-pole models 0.08 %) are per mono lane.
        static const char* const names[] = {"Transistor ladder", "OTA cascade", "Coupled diode ladder", "Sallen-Key LP",
                                            "Sallen-Key HP",     "OTA SVF",     "OTA 4-pole",           "Passive HP"};
        static const double targets[] = {0.12, 0.12, 0.18, 0.08, 0.08, 0.08, 0.12, 0.08};
        const auto cost = [](int voices, int model, bool on)
        {
            double best = 1.0e9;
            for (int rep = 0; rep < 3; ++rep)
            {
                auto values = ParameterMap::defaultValues();
                unison(values, 1, 1);
                setValue(values, "amp_sustain", 1.0f);
                setValue(values, "filter_on", on ? 1.0f : 0.0f);
                setValue(values, "filter_type", static_cast<float>(model));
                setValue(values, "filter_cutoff", 2000.0f);
                setValue(values, "filter_res", 0.7f);
                setValue(values, "filter_drive", 0.2f);
                CRUCIBLE_HEAP(SynthEngine, engine);
                prepare(engine, values);
                for (int n = 0; n < voices; ++n)
                    engine.noteOn(36 + n * 3, 1.0f);
                renderBlocks(engine, 4800);
                const auto start = std::chrono::steady_clock::now();
                renderBlocks(engine, static_cast<int>(kRate * 3.0));
                best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
            }
            return best;
        };
        std::printf("Analog filter models at Standard, 48 kHz, 512-sample blocks; percent of one core, filter on minus "
                    "filter off\n");
        std::printf("%-22s %7s %14s %14s %13s %13s\n", "model", "voices", "% per voice", "% per lane", "plan target",
                    "fails at");
        int failed = 0;
        for (int voices : {8, 16})
            for (int m = 0; m < 8; ++m)
            {
                const double off = cost(voices, 17 + m, false);
                const double on = cost(voices, 17 + m, true);
                const double perVoice = 100.0 * (on - off) / 3.0 / voices;
                const bool over = perVoice / 2.0 > targets[m] * 1.2;
                failed += over ? 1 : 0;
                std::printf("%-22s %7d %13.3f%% %13.3f%% %12.2f%% %12.2f%%  %s\n", names[m], voices, perVoice,
                            perVoice / 2.0, targets[m], targets[m] * 1.2, over ? "OVER" : "ok");
            }
        return failed == 0 ? 0 : 1;
    }
    const std::vector<Config> configs = {
        {"1 voice, 1 osc, no FX", 1, [](auto& v) { unison(v, 1, 1); }},
        {"8 voices, 1 osc x 4 unison, no FX", 8, [](auto& v) { unison(v, 4, 1); }},
        {"16 voices, 1 osc x 8 unison, no FX", 16, [](auto& v) { unison(v, 8, 1); }},
        {"16 voices, 3 osc x 8 unison, no FX", 16, [](auto& v) { unison(v, 8, 3); }},
        {"16 voices, 3 osc x 8 unison, ladder + comb filters", 16,
         [](auto& v)
         {
             unison(v, 8, 3);
             setValue(v, "filter_type", 8.0f);
             setValue(v, "filter2_on", 1.0f);
             setValue(v, "filter2_type", 12.0f);
             setValue(v, "filter_route", 1.0f);
         }},
        {"16 voices, 3 osc x 4 unison, FM + hard sync + fold warps (2x/4x oversampled)", 16,
         [](auto& v)
         {
             unison(v, 4, 3);
             setValue(v, "osc1_warp", 1.0f);
             setValue(v, "osc1_warp_a", 0.4f);
             setValue(v, "osc2_warp", 2.0f);
             setValue(v, "osc2_warp_a", 0.4f);
             setValue(v, "osc3_warp", 14.0f);
             setValue(v, "osc3_warp_a", 0.4f);
         }},
        {"warp detail: 16 voices, 3 osc x 4 unison, FM on all (4x)", 16,
         [](auto& v)
         {
             unison(v, 4, 3);
             for (const char* o : {"osc1", "osc2", "osc3"})
             {
                 setValue(v, std::string(o) + "_warp", 1.0f);
                 setValue(v, std::string(o) + "_warp_a", 0.4f);
             }
         }},
        {"warp detail: 16 voices, 3 osc x 4 unison, hard sync on all (2x)", 16,
         [](auto& v)
         {
             unison(v, 4, 3);
             for (const char* o : {"osc1", "osc2", "osc3"})
             {
                 setValue(v, std::string(o) + "_warp", 2.0f);
                 setValue(v, std::string(o) + "_warp_a", 0.4f);
             }
         }},
        {"warp detail: 16 voices, 3 osc x 4 unison, fold on all (4x)", 16,
         [](auto& v)
         {
             unison(v, 4, 3);
             for (const char* o : {"osc1", "osc2", "osc3"})
             {
                 setValue(v, std::string(o) + "_warp", 14.0f);
                 setValue(v, std::string(o) + "_warp_a", 0.4f);
             }
         }},
        {"16 voices, 1 osc x 4 unison, all 20 FX on (convolver included)", 16,
         [](auto& v)
         {
             unison(v, 4, 1);
             allFx(v);
         }},
        {"worst case: 16 voices, 3 osc x 8 unison, 3 warps, 2 filters, all FX, 16 mod routes", 16,
         [](auto& v)
         {
             unison(v, 8, 3);
             setValue(v, "osc1_warp", 1.0f);
             setValue(v, "osc1_warp_a", 0.4f);
             setValue(v, "osc2_warp", 2.0f);
             setValue(v, "osc3_warp", 14.0f);
             setValue(v, "filter_type", 8.0f);
             setValue(v, "filter2_on", 1.0f);
             setValue(v, "filter2_type", 11.0f);
             setValue(v, "filter_route", 1.0f);
             for (int s = 1; s <= 16; ++s)
             {
                 setValue(v, "mod" + std::to_string(s) + "_src", static_cast<float>(1 + s % 2));
                 setValue(v, "mod" + std::to_string(s) + "_dst", static_cast<float>(1 + s % 12));
                 setValue(v, "mod" + std::to_string(s) + "_amt", 0.3f);
             }
             allFx(v);
         }},
    };

    std::printf("Crucible CPU benchmark, 48 kHz, 512-sample blocks, 5 s of audio, single thread\n");
    std::printf("real-time factor = audio seconds rendered per wall-clock second (higher is better, > 1 is real time)\n\n");
    for (const auto& c : configs)
    {
        if (only == "plan" ? std::string(c.name) != kReferenceRow
                           : (!only.empty() && std::string(c.name).find(only) == std::string::npos))
            continue;
        auto values = ParameterMap::defaultValues();
        c.setup(values);
        setValue(values, "amp_sustain", 1.0f);
        CRUCIBLE_HEAP(SynthEngine, engine);
        prepare(engine, values);
        for (int n = 0; n < c.notes; ++n)
            engine.noteOn(36 + n * 3, 1.0f);
        renderBlocks(engine, 4800); // warm up
        const int total = static_cast<int>(kRate * 5.0);
        const auto start = std::chrono::steady_clock::now();
        const auto r = renderBlocks(engine, total);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        std::printf("%-86s  RTF %7.1f   (CPU %5.1f%% of one core)  %s\n", c.name, 5.0 / seconds, 100.0 * seconds / 5.0,
                    r.finite ? "" : "NON-FINITE");
        if (std::string(c.name) == kReferenceRow)
            plan[2].measuredPercent = 100.0 * seconds / 5.0;
    }

    if (only.empty() || std::string("vector field").find(only) != std::string::npos)
    {
        // Cost of the vector overlay: 24 voices with and without the morph (four scenes that differ a little in every
        // scene-managed float so the voices cost the same, orbit and drift running so the puck moves in every chunk).
        const auto run = [&](bool vector, int voices, bool moving = true)
        {
            EngineConfig cfg;
            cfg.maxVoices = voices;
            SceneBank scenes;
            auto values = ParameterMap::defaultValues();
            unison(values, 4, 3);
            setValue(values, "amp_sustain", 1.0f);
            const auto& defs = ParameterMap::params();
            for (int k = 0; k < 4; ++k)
            {
                auto v = values;
                for (size_t i = 0; i < defs.size(); ++i)
                    if (defs[i].scene && defs[i].type == ParamType::Float)
                        v[i] = std::clamp(values[i] + 0.03f * (static_cast<float>(k) - 1.5f) * (defs[i].max - defs[i].min),
                                          defs[i].min, defs[i].max);
                scenes.store(k, v);
            }
            if (!vector)
                scenes.vectorMorph({0, 1, 2, 3}, 0.5f, 0.5f, values); // the same sound, set directly
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.attachScenes(&scenes);
            if (vector)
            {
                setValue(values, "vector_on", 1.0f);
                setValue(values, "vector_orbit_depth", moving ? 0.4f : 0.0f);
                setValue(values, "vector_orbit_rate", 3.0f);
                setValue(values, "vector_drift", moving ? 0.2f : 0.0f);
                setValue(values, "vector_smooth", 0.02f);
            }
            prepare(engine, values);
            for (int n = 0; n < voices; ++n)
                engine.noteOn(36 + n * 2, 1.0f);
            renderBlocks(engine, 4800);
            const auto start = std::chrono::steady_clock::now();
            renderBlocks(engine, static_cast<int>(kRate * 5.0));
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        };
        double off = 1e9, on = 1e9, still = 1e9;
        for (int rep = 0; rep < 6; ++rep) // best of six
        {
            off = std::min(off, run(false, 24));
            on = std::min(on, run(true, 24));
            still = std::min(still, run(true, 24, false));
        }
        std::printf("\nVector field, puck standing still: %.1f%% of one core (%+.2f%% of the render time)",
                    100.0 * still / 5.0, 100.0 * (still - off) / off);
        std::printf("\nVector field, 24 voices, 3 osc x 4 unison: off %.1f%% of one core, on %.1f%%  -> morph adds %.2f%% "
                    "of the render time\n",
                    100.0 * off / 5.0, 100.0 * on / 5.0, 100.0 * (on - off) / off);
    }

    if (only.empty() || only == "plan")
    {
        plan[0].measuredPercent = analogVoicePercent(Quality::Standard);
        plan[1].measuredPercent = analogVoicePercent(Quality::High);
    }
    if (only == "plan")
        return printPlanTable(plan);
    if (!only.empty())
        return 0;

    // Per effect module: CPU per stereo sample at the module's defaults (switched on) and the non-harmonic energy
    // of a 1 kHz sine at -6 dBFS (aliasing, noise and modulation sidebands; harmonics are excluded).
    std::printf("\nEffect modules (defaults, switched on), 48 kHz, 32-sample control blocks\n");
    std::printf("%-14s %-11s %10s %10s %14s\n", "module", "cpu class", "ns/sample", "% core", "1k non-harm dB");
    const auto& defs = ParameterMap::params();
    std::vector<float> legacy(static_cast<size_t>(kNumModDests), 0.0f);
    ModuleContext ctx;
    ctx.sampleRate = kRate;
    double heaviestFx = 0.0;
    for (int m = 0;; ++m)
    {
        ModuleRack rack(ModuleRegistry::instance());
        if (m >= rack.size())
            break;
        rack.prepare(ctx);
        auto values = ParameterMap::defaultValues();
        for (const auto& p : defs)
            if (p.id.size() > 3 && p.id.compare(p.id.size() - 3, 3, "_on") == 0 && p.id.rfind("fx_", 0) == 0 &&
                p.id.rfind("fx_ep_b", 0) != 0)
                setValue(values, p.id, 0.0f);
        setValue(values, rack.descriptor(m).onParam, 1.0f);
        const auto run = [&](std::vector<float>& l, std::vector<float>& r)
        {
            for (size_t pos = 0; pos < l.size(); pos += 32)
                rack.process(l.data() + pos, r.data() + pos, 32, values.data(), legacy.data(), nullptr, 120.0);
        };
        const int n = static_cast<int>(kRate * 2.0);
        std::vector<float> l(static_cast<size_t>(n)), r(static_cast<size_t>(n));
        Rng rng;
        for (int i = 0; i < n; ++i)
        {
            l[static_cast<size_t>(i)] = 0.3f * rng.bipolar();
            r[static_cast<size_t>(i)] = 0.3f * rng.bipolar();
        }
        const auto start = std::chrono::steady_clock::now();
        run(l, r);
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        rack.reset();
        std::vector<float> sl(static_cast<size_t>(n)), sr(static_cast<size_t>(n));
        for (int i = 0; i < n; ++i)
            sl[static_cast<size_t>(i)] = sr[static_cast<size_t>(i)] =
                0.5f * static_cast<float>(std::sin(2.0 * 3.14159265358979 * 1000.0 * i / kRate));
        run(sl, sr);
        const double thd = inharmonicEnergyDb(sl, 1000.0, kRate, n - 20000);
        heaviestFx = std::max(heaviestFx, 100.0 * seconds / 2.0);
        static const char* cpu[] = {"light", "medium", "heavy"};
        std::printf("%-14s %-11s %10.1f %10.2f %14.1f\n", rack.descriptor(m).id.c_str(),
                    cpu[static_cast<int>(rack.descriptor(m).cpu)], seconds * 1.0e9 / n, 100.0 * seconds / 2.0, thd);
    }
    plan[4].measuredPercent = heaviestFx;
    return printPlanTable(plan);
}
