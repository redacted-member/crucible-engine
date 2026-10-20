// SPDX-License-Identifier: MIT
// Voices whose filter slot holds one of the nonlinear models (FilterModel 17 and up) run four lanes at a time in the
// engine's FilterBank. These tests render the same playing with the bank and voice by voice (EngineConfig::
// batchAnalogFilters) and compare, through note starts at odd sample positions, steals, mono mode, unison, every filter
// routing, model switches and cutoff modulation; and they count allocations with many voices.

#include "AllocCounter.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Filters.h>
#include <crucible/graph/Compiler.h>
#include <crucible/Simd.h>

#include <chrono>
#include <cstring>
#include <cstdio>
#include <functional>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define CRUCIBLE_TEST_SANITIZED 1
#endif
#ifndef CRUCIBLE_TEST_SANITIZED
#define CRUCIBLE_TEST_SANITIZED 0
#endif

using namespace crucible;
using namespace crucible::testing;

namespace
{
// Largest difference between the two renders relative to the larger peak of the pair (but at least 1). The two paths run
// the same arithmetic in a different order (a lane of a vector against a scalar, the 16-sample ramps advanced in
// different steps), so they agree to float rounding: 1.5e-6 was the largest difference measured, the bound is 1e-5.
constexpr double kTolerance = 1.0e-5;
// Full resonance at full drive self-oscillates in a nonlinear loop that magnifies a rounding difference of 1e-7 in the
// integrator gain into a small phase and amplitude error (the largest measured is 1.4 % of the peak on arm64 NEON, 3.5 % in
// the scalar and SSE builds, the diode ladder).
constexpr double kHardTolerance = 8.0e-2;

struct Event
{
    int call; // before this render call
    int note; // negative: note off of -note
    float velocity;
};

struct Playing
{
    std::vector<float> values;
    int maxVoices = 16;
    std::vector<Event> events;
    // Parameter changes applied before a call: {call, id, value}.
    struct Change
    {
        int call;
        const char* id;
        float value;
    };
    std::vector<Change> changes;
};

Rendered play(const Playing& p, bool batched, std::shared_ptr<const graph::CompiledProduct> compiled = nullptr,
              std::uint64_t* quadBlocks = nullptr)
{
    EngineConfig cfg;
    cfg.maxVoices = p.maxVoices;
    cfg.batchAnalogFilters = batched;
    cfg.compiled = std::move(compiled);
    CRUCIBLE_HEAP(SynthEngine, engine, cfg);
    auto values = p.values;
    prepare(engine, values);
    // Odd block lengths cut the 32-sample control grid at different places, so voices start and end inside a cell.
    static const int sizes[] = {37, 100, 64, 5, 211, 29, 32, 160};
    Rendered all;
    std::vector<float> l(512), r(512);
    for (int call = 0; call < 60; ++call)
    {
        for (const auto& c : p.changes)
            if (c.call == call)
            {
                setValue(values, c.id, c.value);
                engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            }
        for (const auto& e : p.events)
            if (e.call == call)
            {
                if (e.note >= 0)
                    engine.noteOn(e.note, e.velocity);
                else
                    engine.noteOff(-e.note);
            }
        const int n = sizes[call % 8];
        engine.render(l.data(), r.data(), n);
        for (int i = 0; i < n; ++i)
        {
            all.left.push_back(l[static_cast<size_t>(i)]);
            all.right.push_back(r[static_cast<size_t>(i)]);
            for (const float s : {l[static_cast<size_t>(i)], r[static_cast<size_t>(i)]})
            {
                all.finite = all.finite && std::isfinite(s);
                all.peak = std::max(all.peak, std::abs(s));
            }
        }
    }
    if (quadBlocks != nullptr)
        *quadBlocks = engine.batchedFilterBlocks();
    return all;
}

struct Difference
{
    double worst = 0.0;
    double peak = 0.0;
    size_t first = 0; // first sample that differs by more than 1e-3 (0 = none)
};

Difference compare(const Playing& p, const std::shared_ptr<const graph::CompiledProduct>& compiled = nullptr,
                   std::uint64_t* quadBlocks = nullptr)
{
    const auto a = play(p, true, compiled, quadBlocks), b = play(p, false, compiled);
    Difference d;
    d.peak = std::max(a.peak, b.peak);
    if (!a.finite || !b.finite || a.left.size() != b.left.size())
    {
        d.worst = 1.0e9;
        return d;
    }
    for (size_t i = 0; i < a.left.size(); ++i)
    {
        if (std::abs(a.left[i] - b.left[i]) > 1.0e-3f && d.first == 0)
            d.first = i;
        d.worst = std::max({d.worst, static_cast<double>(std::abs(a.left[i] - b.left[i])),
                            static_cast<double>(std::abs(a.right[i] - b.right[i]))});
    }
    return d;
}

std::vector<float> patch(int model, float res = 0.7f, float drive = 0.2f)
{
    auto v = ParameterMap::defaultValues();
    setValue(v, "amp_sustain", 0.8f);
    setValue(v, "amp_release", 0.15f);
    setValue(v, "filter_on", 1.0f);
    setValue(v, "filter_type", static_cast<float>(model));
    setValue(v, "filter_cutoff", 1500.0f);
    setValue(v, "filter_res", res);
    setValue(v, "filter_drive", drive);
    setValue(v, "filter_character", 0.3f);
    setValue(v, "filter_env", 0.4f);
    setValue(v, "filter_key", 0.5f);
    setValue(v, "osc1_uni_voices", 3.0f);
    setValue(v, "osc1_uni_detune", 20.0f);
    setValue(v, "osc1_uni_spread", 0.8f);
    return v;
}

// A graph product: oscillator, filter.analog (model, optionally with an audio-rate cutoff), vca and an envelope that also
// opens the cutoff; six voices, so the last pair is full and an odd count is not needed to cover a pair with one voice.
std::shared_ptr<const graph::CompiledProduct> graphProduct(int model, bool audioRate, int voices)
{
    const std::string fm = audioRate ? R"(, { "from": "lfo.out", "to": "filt.cutoff_fm", "amount": 0.2 })" : "";
    const std::string lfo = audioRate ? R"(, { "id": "lfo", "type": "lfo.audio", "params": { "rate": 375.0 } })" : "";
    const std::string text = R"({"format": "crucible-product", "version": 1, "id": "test.quad",
        "voice": { "layers": [ { "id": "main", "voices": )" +
                             std::to_string(voices) + R"(,
            "nodes": [ { "id": "osc", "type": "osc.analog", "slot": 1 }, { "id": "env", "type": "env.adsr" },
                       { "id": "filt", "type": "filter.analog", "params": { "model": )" +
                             std::to_string(model) + R"(, "cutoff": 1500.0, "res": 0.7,
                           "drive": 0.2, "character": 0.3, "tolerance": 0.4, "key": 0.5 } },
                       { "id": "amp", "type": "amp.vca" }, { "id": "out", "type": "mix.out" })" +
                             lfo + R"( ],
            "edges": [ { "from": "osc.out", "to": "filt.in" }, { "from": "env.out", "to": "filt.cutoff", "amount": 0.3 },
                       { "from": "filt.out", "to": "amp.in" }, { "from": "env.out", "to": "amp.gain" },
                       { "from": "amp.out", "to": "out.in" })" +
                             fm + R"( ] } ] },
        "defaults": { "osc1_level": 0.4 } })";
    graph::ProductDef def;
    std::string error;
    if (!graph::productFromText(text, def, error))
        return nullptr;
    return graph::compile(def, error);
}

// Five notes at staggered, off-grid starts with note-offs between.
std::vector<Event> chord()
{
    return {{0, 48, 1.0f}, {2, 55, 0.8f}, {3, 60, 0.7f}, {5, 64, 1.0f},
            {9, 67, 0.9f}, {14, -55, 0},  {20, -48, 0},  {26, 52, 0.6f}};
}

// Percent of one core per voice that the filter adds, with `voices` stereo voices playing: the same patch with the filter
// switched on and off, best of three runs each, 512-sample blocks.
double filterCost(int voices, int model)
{
    const auto run = [&](bool on)
    {
        double best = 1.0e9;
        for (int rep = 0; rep < 3; ++rep)
        {
            auto values = ParameterMap::defaultValues();
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
            renderBlocks(engine, static_cast<int>(kRate * 1.5));
            best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        }
        return best;
    };
    return 100.0 * (run(true) - run(false)) / 1.5 / voices;
}

class VoiceQuadTests : public UnitTest
{
public:
    VoiceQuadTests() : UnitTest("VoiceQuad", "Engine") {}

    void runTest() override
    {
        static const char* const names[] = {"transistor ladder", "OTA cascade", "diode ladder", "Sallen-Key LP",
                                            "Sallen-Key HP",     "OTA SVF",     "OTA 4-pole",   "passive HP"};
        double overall = 0.0;

        beginTest(
            "batched equals voice by voice for each model: staggered notes, note-offs, unison, envelope and key tracking");
        for (int m = 0; m < 8; ++m)
        {
            Playing p;
            p.values = patch(17 + m);
            p.events = chord();
            const auto d = compare(p);
            overall = std::max(overall, d.worst / std::max(d.peak, 1.0e-6));
            logMessage(String(names[m]) + ": largest difference " + String(d.worst) + " at peak " + String(d.peak));
            expect(d.peak > 1.0e-3, String(names[m]) + " is audible");
            expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String(names[m]) + " difference " + String(d.worst));
        }

        beginTest("graph filter.analog nodes: voice pairs run as quads and match voice by voice, for each model, with and "
                  "without an audio-rate cutoff");
        for (const bool audioRate : {false, true})
            for (int m = 0; m < 8; ++m)
            {
                const auto compiled = graphProduct(m, audioRate, 6);
                expect(compiled != nullptr, String(names[m]) + " product compiles");
                if (!compiled)
                    continue;
                Playing p;
                p.values = compiled->defaultValues();
                p.events = chord();
                std::uint64_t blocks = 0;
                const auto d = compare(p, compiled, &blocks);
                const String what = String(names[m]) + (audioRate ? " with audio-rate cutoff" : "");
                logMessage(what + ": largest difference " + String(d.worst) + " at peak " + String(d.peak) + ", " +
                           String(static_cast<double>(blocks)) + " quad blocks");
                expect(d.peak > 1.0e-3, what + " is audible");
                expect(blocks > 0, what + " took the quad path");
                expect(d.worst <= kTolerance * std::max(d.peak, 1.0), what + " difference " + String(d.worst));
            }

        beginTest(
            "voice card filter tolerance: the bank lanes draw the voice's own component values, as the single path does");
        for (int m = 0; m < 8; ++m)
        {
            Playing p;
            p.values = patch(17 + m);
            setValue(p.values, "voice_card_filter", 1.0f);
            p.events = chord();
            const auto d = compare(p);
            logMessage(String(names[m]) + " with card tolerance: largest difference " + String(d.worst) + " at peak " +
                       String(d.peak));
            expect(d.peak > 1.0e-3, String(names[m]) + " is audible");
            expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String(names[m]) + " difference " + String(d.worst));
        }

        beginTest("hard settings: full resonance, full drive, character and the high-pass step");
        for (int m = 0; m < 8; ++m)
        {
            Playing p;
            p.values = patch(17 + m, 1.0f, 1.0f);
            setValue(p.values, "filter_character", 1.0f);
            setValue(p.values, "filter_hp", 2.0f);
            p.events = chord();
            const auto d = compare(p);
            overall = std::max(overall, d.worst / std::max(d.peak, 1.0e-6));
            logMessage(String(names[m]) + ": largest difference " + String(d.worst) + " at peak " + String(d.peak) +
                       ", first at sample " + String(static_cast<double>(d.first)));
            expect(d.worst <= kHardTolerance * std::max(d.peak, 1.0), String(names[m]) + " difference " + String(d.worst));
        }

        beginTest("a full pool steals the oldest voice; the spare voices fade the victim");
        {
            Playing p;
            p.values = patch(17);
            p.maxVoices = 4;
            p.events = {{0, 40, 1.0f}, {1, 47, 1.0f}, {2, 52, 1.0f}, {3, 59, 1.0f},
                        {4, 64, 1.0f}, {6, 67, 1.0f}, {8, 71, 1.0f}};
            const auto d = compare(p);
            expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String("difference ") + String(d.worst));
        }

        beginTest("mono and legato mode with glide");
        for (const float mode : {1.0f, 2.0f})
        {
            Playing p;
            p.values = patch(18);
            setValue(p.values, "voice_mode", mode);
            setValue(p.values, "voice_glide", 0.1f);
            p.events = {{0, 48, 1.0f}, {4, 55, 1.0f}, {8, 60, 1.0f}, {12, -60, 0}, {16, 52, 1.0f}};
            const auto d = compare(p);
            expect(d.worst <= kTolerance * std::max(d.peak, 1.0),
                   String("mode ") + String(mode) + " difference " + String(d.worst));
        }

        beginTest("two filter slots: nonlinear models in both, one of each, in serial, parallel and split routing");
        for (int route = 0; route < 3; ++route)
            for (int combo = 0; combo < 3; ++combo)
            {
                Playing p;
                p.values = patch(combo == 2 ? 8 : 17);
                setValue(p.values, "filter_route", static_cast<float>(route));
                setValue(p.values, "filter2_on", 1.0f);
                setValue(p.values, "filter2_type", combo == 1 ? 8.0f : (combo == 2 ? 20.0f : 19.0f));
                setValue(p.values, "filter2_cutoff", 900.0f);
                setValue(p.values, "filter2_res", 0.6f);
                setValue(p.values, "filter_pan", -0.4f);
                setValue(p.values, "filter2_pan", 0.4f);
                setValue(p.values, "filter2_mix", 0.8f);
                setValue(p.values, "osc2_level", 0.5f);
                setValue(p.values, "osc2_route", 1.0f);
                setValue(p.values, "sub_level", 0.3f);
                setValue(p.values, "noise_level", 0.05f);
                p.events = chord();
                const auto d = compare(p);
                expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String("route ") + String(route) + " combination " +
                                                                          String(combo) + " difference " + String(d.worst));
            }

        beginTest("switching the model while notes sound, and switching a slot off and on");
        {
            const std::vector<std::pair<const char*, std::vector<Playing::Change>>> cases = {
                {"nonlinear to nonlinear", {{6, "filter_type", 20.0f}, {14, "filter_type", 22.0f}}},
                {"nonlinear to legacy and back", {{6, "filter_type", 8.0f}, {14, "filter_type", 18.0f}}},
                {"slot off and on", {{6, "filter_on", 0.0f}, {14, "filter_on", 1.0f}}},
                {"all of them",
                 {{6, "filter_type", 20.0f},
                  {10, "filter_type", 8.0f},
                  {14, "filter_type", 22.0f},
                  {18, "filter_on", 0.0f},
                  {22, "filter_on", 1.0f},
                  {30, "filter_type", 19.0f}}},
            };
            for (const auto& c : cases)
            {
                Playing p;
                p.values = patch(17);
                p.events = chord();
                p.changes = c.second;
                const auto d = compare(p);
                logMessage(String(c.first) + ": largest difference " + String(d.worst) + " at peak " + String(d.peak) +
                           ", first at sample " + String(static_cast<double>(d.first)));
                expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String(c.first) + " difference " + String(d.worst));
            }
        }

        beginTest("per-voice cutoff modulation from the matrix (key velocity and envelope) is kept");
        {
            Playing p;
            p.values = patch(17);
            setValue(p.values, "mod1_src", 3.0f);
            setValue(p.values, "mod1_dst", 7.0f);
            setValue(p.values, "mod1_amt", 0.6f);
            setValue(p.values, "mod2_src", 2.0f);
            setValue(p.values, "mod2_dst", 7.0f);
            setValue(p.values, "mod2_amt", 0.4f);
            p.events = chord();
            const auto d = compare(p);
            expect(d.worst <= kTolerance * std::max(d.peak, 1.0), String("difference ") + String(d.worst));
            // The notes differ in velocity and pitch, so their cutoffs differ: the render must not equal one with the
            // modulation removed.
            Playing flat = p;
            setValue(flat.values, "mod1_amt", 0.0f);
            setValue(flat.values, "mod2_amt", 0.0f);
            const auto a = play(p, true), b = play(flat, true);
            double diff = 0.0;
            for (size_t i = 0; i < a.left.size(); ++i)
                diff = std::max(diff, static_cast<double>(std::abs(a.left[i] - b.left[i])));
            expect(diff > 0.01, "the modulation is audible through the bank");
        }

        beginTest("resetLane returns one lane to a fresh filter and leaves the other lanes running");
        {
            static std::unique_ptr<AnalogFilter> (*const factories[])() = {
                makeTransistorLadder,  makeOtaCascade, makeDiodeLadder, makeSallenKeyLowpass,
                makeSallenKeyHighpass, makeSvfOta,     makeOta4Pole,    makePassiveHp};
            for (int m = 0; m < 8; ++m)
            {
                auto a = factories[m](), b = factories[m](), fresh = factories[m]();
                for (auto* f : {a.get(), b.get()})
                {
                    f->prepare(kRate, Quality::Standard, 4, 64);
                    for (int l = 0; l < 4; ++l)
                        f->setLaneControls(l, 600.0f * static_cast<float>(l + 1), 0.5f, 0.1f, 0.3f, 0.0f);
                }
                fresh->prepare(kRate, Quality::Standard, 1, 64);
                Rng rng;
                std::vector<std::vector<float>> x(4, std::vector<float>(64)), y, z;
                const auto fill = [&]
                {
                    for (auto& v : x)
                        for (auto& t : v)
                            t = 0.3f * rng.bipolar();
                };
                const auto runQuad = [&](AnalogFilter& f, std::vector<std::vector<float>>& io)
                {
                    float* p4[4] = {io[0].data(), io[1].data(), io[2].data(), io[3].data()};
                    f.processQuad(p4, 64);
                };
                for (int blockIndex = 0; blockIndex < 20; ++blockIndex)
                {
                    fill();
                    y = x;
                    z = x;
                    runQuad(*a, y);
                    runQuad(*b, z);
                }
                a->resetLane(2);
                a->setLaneControls(2, 900.0f, 0.4f, 0.2f, 0.5f, 0.0f);
                fresh->setControls(900.0f, 0.4f, 0.2f, 0.5f, 0.0f);
                double others = 0.0, lane = 0.0;
                for (int blockIndex = 0; blockIndex < 20; ++blockIndex)
                {
                    fill();
                    y = x;
                    z = x;
                    std::vector<float> one = x[2];
                    runQuad(*a, y);
                    runQuad(*b, z);
                    fresh->process(one.data(), 64);
                    for (size_t i = 0; i < 64; ++i)
                    {
                        for (const size_t l : {0u, 1u, 3u})
                            others = std::max(others, static_cast<double>(std::abs(y[l][i] - z[l][i])));
                        lane = std::max(lane, static_cast<double>(std::abs(y[2][i] - one[i])));
                    }
                }
                expect(others < 1.0e-4, String(names[m]) + " other lanes differ by " + String(others));
                expect(lane < 1.0e-4, String(names[m]) + " the reset lane differs from a fresh filter by " + String(lane));
            }
        }

        beginTest("legacy models are untouched: a patch without a nonlinear model renders identically either way");
        {
            for (const int model : {0, 5, 8, 11})
            {
                Playing p;
                p.values = patch(model);
                p.events = chord();
                const auto a = play(p, true), b = play(p, false);
                expect(a.left == b.left && a.right == b.right, String("model ") + String(model));
            }
        }

        beginTest("no allocation with 16 voices on every nonlinear model, with model switches and note changes");
        {
            EngineConfig cfg;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            auto v = patch(17);
            setValue(v, "filter2_on", 1.0f);
            setValue(v, "filter2_type", 21.0f);
            setValue(v, "filter_route", 1.0f);
            prepare(engine, v);
            for (int n = 0; n < 16; ++n)
                engine.noteOn(36 + n * 3, 0.9f);
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 10; ++i)
                engine.render(l.data(), r.data(), 256);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int model = 17; model < 25; ++model)
            {
                setValue(v, "filter_type", static_cast<float>(model));
                setValue(v, "filter2_type", static_cast<float>(41 - model));
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                for (int i = 0; i < 10; ++i)
                {
                    if (i % 4 == 0)
                    {
                        engine.noteOff(36 + (i % 16) * 3);
                        engine.noteOn(40 + i, 0.8f);
                    }
                    engine.render(l.data(), r.data(), i % 2 == 0 ? 256 : 77);
                }
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with 16 voices on the filter bank");
        }
        beginTest("no allocation in the graph quad path with note changes and audio-rate cutoff");
        {
            EngineConfig cfg;
            cfg.compiled = graphProduct(2, true, 6);
            expect(cfg.compiled != nullptr, "product compiles");
            if (cfg.compiled)
            {
                CRUCIBLE_HEAP(SynthEngine, graphEngine, cfg);
                const auto values = cfg.compiled->defaultValues();
                prepare(graphEngine, values);
                for (int n = 0; n < 6; ++n)
                    graphEngine.noteOn(40 + n * 4, 0.9f);
                std::vector<float> l(256), r(256);
                for (int i = 0; i < 10; ++i)
                    graphEngine.render(l.data(), r.data(), 256);
                gAllocations = 0;
                gTrackAllocations = true;
                for (int i = 0; i < 20; ++i)
                {
                    if (i % 4 == 0)
                    {
                        graphEngine.noteOff(40 + (i % 6) * 4);
                        graphEngine.noteOn(44 + i, 0.8f);
                    }
                    graphEngine.render(l.data(), r.data(), i % 2 == 0 ? 256 : 77);
                }
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "allocations in the graph quad path");
                expect(graphEngine.batchedFilterBlocks() > 0, "the graph quad path ran");
            }
        }

        beginTest("CPU: eight voices through the engine cost the plan's figure per mono lane (within 20 %)");
        {
            // The absolute numbers are those of an Apple M-class core (the plan's reference machine); other hosts only
            // record them. A stereo voice runs two lanes. Before the bank every voice ran alone at 0.43 to 0.60 % per lane.
#if defined(NDEBUG) && !CRUCIBLE_TEST_SANITIZED && CRUCIBLE_SIMD_NEON
            struct Row
            {
                int model;
                double target;
            };
            for (const Row row : {Row{17, 0.12}, Row{19, 0.18}, Row{20, 0.08}, Row{23, 0.12}})
            {
                double lane = 1.0e9;
                for (int attempt = 0; attempt < 2 && lane > row.target * 1.2; ++attempt)
                    lane = std::min(lane, filterCost(8, row.model) / 2.0);
                logMessage(String("model ") + String(row.model) + ": " + String(lane) + " % per lane, plan target " +
                           String(row.target) + " %");
                expectCpu(lane <= row.target * 1.2,
                       String("model ") + String(row.model) + " costs " + String(lane) + " % per lane");
            }
#endif
        }

        beginTest("FilterUnit::processBlock equals the per-sample process() loop: bit for bit at constant controls, within "
                  "rounding as the cutoff moves (models 17-24, Standard and High)");
        for (const float cutoffStep : {1.0f, 1.07f})
            for (const Quality quality : {Quality::Standard, Quality::High})
                for (int m = 0; m < 8; ++m)
                {
                    FilterUnit perSample, blockwise;
                    for (FilterUnit* u : {&perSample, &blockwise})
                    {
                        u->prepare(kRate, quality);
                        u->setSeed(11);
                        u->reset();
                    }
                    FilterSettings s;
                    s.model = 17 + m;
                    s.cutoffHz = 1800.0f;
                    s.resonance = 0.7f;
                    s.character = 0.3f;
                    s.tolerance = 0.6f;
                    s.highpass = 1;
                    bool identical = true;
                    float movingWorst = 0.0f;
                    std::uint32_t noise = 12345;
                    int done = 0;
                    for (const int n : {32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32})
                    {
                        s.cutoffHz *= cutoffStep;
                        perSample.configure(s, 0.2f, kRate);
                        blockwise.configure(s, 0.2f, kRate);
                        std::vector<float> l(static_cast<size_t>(n)), r(static_cast<size_t>(n));
                        for (int i = 0; i < n; ++i, ++done)
                        {
                            noise = noise * 1664525U + 1013904223U;
                            l[static_cast<size_t>(i)] = 0.4f * std::sin(0.05f * static_cast<float>(done)) +
                                                        0.1f * (static_cast<float>(noise >> 8) / 8388608.0f - 1.0f);
                            r[static_cast<size_t>(i)] = -0.7f * l[static_cast<size_t>(i)];
                        }
                        auto bl = l, br = r;
                        for (int i = 0; i < n; ++i)
                            perSample.process(l[static_cast<size_t>(i)], r[static_cast<size_t>(i)]);
                        blockwise.processBlock(bl.data(), br.data(), n, nullptr);
                        for (int i = 0; i < n; ++i)
                            movingWorst =
                                std::max({movingWorst, std::abs(l[static_cast<size_t>(i)] - bl[static_cast<size_t>(i)]),
                                          std::abs(r[static_cast<size_t>(i)] - br[static_cast<size_t>(i)])});
                        identical = identical &&
                                    std::memcmp(l.data(), bl.data(), sizeof(float) * static_cast<size_t>(n)) == 0 &&
                                    std::memcmp(r.data(), br.data(), sizeof(float) * static_cast<size_t>(n)) == 0;
                    }
                    const String what = String(names[m]) + (quality == Quality::High ? " (High)" : " (Standard)");
                    if (cutoffStep == 1.0f)
                        expect(identical, what + " block path differs from process() at constant controls");
                    else
                    {
                        logMessage(what + " moving cutoff: largest difference " + String(movingWorst));
                        expect(movingWorst <= kTolerance, what + " moving-cutoff difference " + String(movingWorst));
                    }
                }

        logMessage(String("largest relative difference over the model scenarios: ") + String(overall));
    }
};

static VoiceQuadTests voiceQuadTests;
} // namespace
