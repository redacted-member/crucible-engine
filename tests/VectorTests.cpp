// SPDX-License-Identifier: MIT
// Vector field (perf.vector): four-corner scene morph as an audio-thread overlay, clip transport and two-lane
// motion recording.

#include "GraphSupport.h"

#include <crucible/Engine.h>
#include <crucible/Patch.h>

using namespace crucible;
using namespace crucible::testing;

namespace
{
// Four scenes that differ in a float with a skew (filter_cutoff), a plain float (osc1_level), a choice
// (osc1_table) and a bool (osc1_on is not present on every build, so fx_ensemble_on).
struct Corners
{
    std::array<float, 4> cutoff{300.0f, 2000.0f, 800.0f, 9000.0f};
    std::array<float, 4> level{0.2f, 0.5f, 0.8f, 1.0f};
    std::array<float, 4> table{1.0f, 2.0f, 3.0f, 4.0f};
};

void fillScenes(Engine& e, const Corners& c)
{
    for (int k = 0; k < 4; ++k)
    {
        e.setParam("filter_cutoff", c.cutoff[static_cast<size_t>(k)]);
        e.setParam("osc1_level", c.level[static_cast<size_t>(k)]);
        e.setParam("osc1_table", c.table[static_cast<size_t>(k)]);
        e.setParam("fx_ensemble_on", k % 2 == 1 ? 1.0f : 0.0f);
        e.storeScene(k);
    }
    e.setParam("filter_cutoff", 5000.0f);
    e.setParam("osc1_level", 0.3f);
    e.setParam("osc1_table", 0.0f);
    e.setParam("fx_ensemble_on", 0.0f);
}

void run(Engine& e, int blocks, int block = 128)
{
    std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
    for (int i = 0; i < blocks; ++i)
        e.process(l.data(), r.data(), block);
}

void puck(Engine& e, float x, float y)
{
    e.setParam("vector_x", x);
    e.setParam("vector_y", y);
}
} // namespace

class VectorTests : public UnitTest
{
public:
    VectorTests() : UnitTest("Vector field and clip transport", "Product") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("parameters: off by default, not part of scenes, pages within six knobs");
        {
            for (const char* id :
                 {"vector_on", "vector_x", "vector_y", "vector_a", "vector_b", "vector_c", "vector_d", "vector_smooth",
                  "vector_orbit_rate", "vector_orbit_depth", "vector_orbit_shape", "vector_drift", "vector_drift_rate"})
            {
                const int i = ParameterMap::indexOf(id);
                expect(i >= 0, id);
                if (i >= 0)
                    expect(!ParameterMap::params()[static_cast<size_t>(i)].scene, id);
            }
            expect(ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("vector_on"))].defaultValue == 0.0f);
            for (size_t page = 0; page < ParameterMap::pages().size(); ++page)
                expect(ParameterMap::paramsOnPage(static_cast<int>(page)).size() <= 6);
            const auto w = vectorWeights(0.5f, 0.5f);
            for (const float v : w.w)
                expectWithinAbsoluteError(v, 0.25f, 1.0e-7f);
            const auto a = vectorWeights(0.0f, 1.0f);
            expect(a.w[0] == 1.0f && a.w[1] == 0.0f && a.w[2] == 0.0f && a.w[3] == 0.0f, "corner A is top left");
            const auto d = vectorWeights(1.0f, 0.0f);
            expect(d.w[3] == 1.0f, "corner D is bottom right");
        }

        beginTest("corners give exactly the stored scene values, the centre is the mean in normalised space");
        {
            Engine e;
            e.prepare(kRate, 128);
            const Corners c;
            fillScenes(e, c);
            e.setParam("vector_smooth", 0.0f);
            e.setParam("vector_on", 1.0f);
            const float xs[4] = {0.0f, 1.0f, 0.0f, 1.0f};
            const float ys[4] = {1.0f, 1.0f, 0.0f, 0.0f};
            for (int k = 0; k < 4; ++k)
            {
                puck(e, xs[k], ys[k]);
                run(e, 80);
                const auto ks = static_cast<size_t>(k);
                expect(e.modulatedValue("filter_cutoff") == c.cutoff[ks], "cutoff at corner " + std::to_string(k));
                expect(e.modulatedValue("osc1_level") == c.level[ks], "level at corner " + std::to_string(k));
                expect(e.modulatedValue("osc1_table") == c.table[ks], "table at corner " + std::to_string(k));
                expect(e.modulatedValue("fx_ensemble_on") == (k % 2 == 1 ? 1.0f : 0.0f),
                       "bool at corner " + std::to_string(k));
                float px = -1.0f, py = -1.0f;
                e.vectorPosition(px, py);
                expect(px == xs[k] && py == ys[k], "puck position");
            }
            puck(e, 0.5f, 0.5f);
            run(e, 80);
            for (const char* id : {"filter_cutoff", "osc1_level"})
            {
                const auto& def = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf(id))];
                float mean = 0.0f;
                const auto& stored = id[0] == 'f' ? c.cutoff : c.level;
                for (const float v : stored)
                    mean += ParameterMap::toNormalised(def, v) * 0.25f;
                expectWithinAbsoluteError(ParameterMap::toNormalised(def, e.modulatedValue(id)), mean, 2.0e-4f, id);
            }
            // The parameters themselves are untouched by the overlay.
            expectEquals(e.getParam(ParameterMap::indexOf("filter_cutoff")), 5000.0f);
            // The offline reference (control thread) agrees with the audio-thread overlay.
            std::vector<float> ref = e.getAllParams();
            expect(e.scenes().vectorMorph({0, 1, 2, 3}, 0.5f, 0.5f, ref));
            expectWithinAbsoluteError(ref[static_cast<size_t>(ParameterMap::indexOf("filter_cutoff"))],
                                      e.modulatedValue("filter_cutoff"), 1.0f);
        }

        beginTest("choices take the heaviest corner, off returns the host values");
        {
            Engine e;
            e.prepare(kRate, 128);
            const Corners c;
            fillScenes(e, c);
            e.setParam("vector_smooth", 0.0f);
            e.setParam("vector_on", 1.0f);
            puck(e, 0.7f, 0.6f); // B has the most weight
            run(e, 60);
            expectEquals(e.modulatedValue("osc1_table"), c.table[1]);
            puck(e, 0.1f, 0.2f); // C
            run(e, 60);
            expectEquals(e.modulatedValue("osc1_table"), c.table[2]);
            puck(e, 0.5f, 0.5f); // a tie goes to corner A
            run(e, 60);
            expectEquals(e.modulatedValue("osc1_table"), c.table[0]);
            e.setParam("vector_on", 0.0f);
            run(e, 80);
            expectWithinAbsoluteError(e.modulatedValue("filter_cutoff"), 5000.0f, 0.01f);
            expectEquals(e.modulatedValue("osc1_table"), 0.0f);
        }

        beginTest("corners with an empty scene use the host value; no stored scene means no overlay");
        {
            Engine e;
            e.prepare(kRate, 128);
            e.setParam("filter_cutoff", 1234.0f);
            e.setParam("vector_on", 1.0f);
            e.setParam("vector_smooth", 0.0f);
            run(e, 40);
            expectWithinAbsoluteError(e.modulatedValue("filter_cutoff"), 1234.0f, 0.01f);
            e.setParam("filter_cutoff", 400.0f);
            e.storeScene(0); // corner A only
            e.setParam("filter_cutoff", 1234.0f);
            puck(e, 0.0f, 1.0f);
            run(e, 80);
            expectEquals(e.modulatedValue("filter_cutoff"), 400.0f);
            puck(e, 1.0f, 1.0f); // B is empty: the host value
            run(e, 80);
            expectWithinAbsoluteError(e.modulatedValue("filter_cutoff"), 1234.0f, 0.01f);
        }

        beginTest("locked parameters keep the host value while the vector is live; clearing the locks hands them back");
        {
            SceneBank scenes;
            auto base = ParameterMap::defaultValues();
            setValue(base, "vector_on", 1.0f);
            setValue(base, "vector_smooth", 0.0f);
            setValue(base, "vector_x", 1.0f);
            setValue(base, "vector_y", 1.0f);
            const float cutoffs[4] = {300.0f, 2000.0f, 800.0f, 9000.0f};
            const float levels[4] = {0.2f, 0.5f, 0.8f, 1.0f};
            for (int k = 0; k < 4; ++k)
            {
                auto v = base;
                setValue(v, "filter_cutoff", cutoffs[k]);
                setValue(v, "osc1_level", levels[k]);
                scenes.store(k, v);
            }
            SynthEngine engine;
            engine.attachScenes(&scenes);
            setValue(base, "filter_cutoff", 1234.0f);
            prepare(engine, base);
            std::vector<char> locks(ParameterMap::params().size(), 0);
            locks[static_cast<size_t>(ParameterMap::indexOf("filter_cutoff"))] = 1;
            engine.setVectorExclusions(locks.data());
            const int cutoff = ParameterMap::indexOf("filter_cutoff"), level = ParameterMap::indexOf("osc1_level");
            renderBlocks(engine, 4096);
            expectWithinAbsoluteError(engine.modulatedValue(cutoff), 1234.0f, 0.01f);
            expectWithinAbsoluteError(engine.modulatedValue(level), 0.5f, 1.0e-4f); // corner B
            locks[static_cast<size_t>(cutoff)] = 0;
            renderBlocks(engine, 4096);
            expectWithinAbsoluteError(engine.modulatedValue(cutoff), 2000.0f, 1.0f);
            // Locking it again stops the overlay writing it; with no lock mask at all the overlay owns everything.
            locks[static_cast<size_t>(cutoff)] = 1;
            setValue(base, "vector_x", 0.0f);
            engine.setParameterValues(base.data(), static_cast<int>(base.size()));
            renderBlocks(engine, 4096);
            expectWithinAbsoluteError(engine.modulatedValue(cutoff), 2000.0f, 1.0f);
            engine.setVectorExclusions(nullptr);
            renderBlocks(engine, 4096);
            expectWithinAbsoluteError(engine.modulatedValue(cutoff), 300.0f, 1.0f);
        }

        beginTest("deterministic, and identical across block sizes on the 32-sample grid (orbit, drift, smoothing)");
        {
            const auto render = [](int block)
            {
                EngineConfig cfg;
                cfg.seed = 7;
                Engine e(cfg);
                e.prepare(kRate, 512);
                fillScenes(e, Corners{});
                e.setParam("amp_sustain", 1.0f);
                e.setParam("vector_on", 1.0f);
                e.setParam("vector_smooth", 0.1f);
                e.setParam("vector_orbit_depth", 0.3f);
                e.setParam("vector_orbit_rate", 1.7f);
                e.setParam("vector_orbit_shape", 1.0f);
                e.setParam("vector_drift", 0.2f);
                e.setParam("vector_drift_rate", 2.0f);
                puck(e, 0.4f, 0.6f);
                e.noteOn(48, 0.9f);
                e.noteOn(55, 0.9f);
                std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
                std::vector<float> outL, outR;
                for (int done = 0; done < 49152; done += block)
                {
                    e.process(l.data(), r.data(), block);
                    outL.insert(outL.end(), l.begin(), l.end());
                    outR.insert(outR.end(), r.begin(), r.end());
                }
                return std::make_pair(outL, outR);
            };
            const auto a = render(512), b = render(512), c = render(128), d = render(32);
            expect(a == b, "same input, same output");
            expect(a == c && a == d, "block sizes 32, 128, 512");
            float peak = 0.0f;
            for (const float s : a.first)
                peak = std::max(peak, std::abs(s));
            expect(peak > 0.01f && std::isfinite(peak), "it sounds");
        }

        beginTest("with the vector off nothing changes, whatever the corner parameters say");
        {
            const auto render = [](bool withScenes)
            {
                Engine e;
                e.prepare(kRate, 128);
                if (withScenes)
                    fillScenes(e, Corners{});
                else
                {
                    e.setParam("filter_cutoff", 5000.0f);
                    e.setParam("osc1_level", 0.3f);
                    e.setParam("osc1_table", 0.0f);
                }
                e.setParam("vector_orbit_depth", 0.4f);
                e.noteOn(50, 1.0f);
                std::vector<float> l(128), r(128), out;
                for (int i = 0; i < 100; ++i)
                {
                    e.process(l.data(), r.data(), 128);
                    out.insert(out.end(), l.begin(), l.end());
                }
                return out;
            };
            expect(render(true) == render(false));
        }

        beginTest("the puck is a modulation target: a route moves it");
        {
            Engine e;
            e.prepare(kRate, 128);
            const Corners c;
            fillScenes(e, c);
            e.setParam("vector_smooth", 0.0f);
            e.setParam("vector_on", 1.0f);
            puck(e, 0.0f, 1.0f);
            ModRoute route;
            route.source = static_cast<int>(ModSource::ModWheel);
            route.target = "vector_x";
            route.amount = 1.0f;
            expect(e.modRoutes().add(route));
            run(e, 60);
            expectEquals(e.modulatedValue("osc1_level"), c.level[0]);
            e.modWheel(1.0f);
            run(e, 60);
            expectEquals(e.modulatedValue("osc1_level"), c.level[1]);
        }

        beginTest("clip transport: stop fades the modulation out and parks the head, start restarts the clip");
        {
            Engine e;
            e.prepare(kRate, 128);
            ClipBank bank;
            Clip clip;
            clip.trigger = ClipTrigger::GlobalFree;
            clip.length = 8.0;
            clip.loopEnd = 8.0;
            ClipLane lane;
            lane.points = {{0.0, 1.0f}, {8.0, 1.0f}};
            clip.lanes = {lane};
            bank.clips.push_back(clip);
            bank.clips.push_back(clip);
            e.clips().replace(bank);
            e.modRoutes().add({clipLaneSource(0, 0), "fx_ra_mix", 0.5f, 2, 0.0f, 0.0f});
            e.setParam("fx_ra_mix", 0.1f);
            const auto& def = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("fx_ra_mix"))];
            const float full = std::min(def.max, 0.1f + 0.5f * (def.max - def.min));
            run(e, 40);
            expect(e.isClipPlaying(0) && e.isClipPlaying(1) && !e.isClipPlaying(2), "state per clip (only loaded clips)");
            expectWithinAbsoluteError(e.modulatedValue("fx_ra_mix"), full, 1.0e-4f);
            std::array<double, kMaxClips> pos{};
            e.stopClip(0);
            expect(!e.isClipPlaying(0) && e.isClipPlaying(1));
            run(e, 10);
            expectEquals(e.modulatedValue("fx_ra_mix"), 0.1f);
            e.clipPositions(pos);
            const double parked = pos[0];
            run(e, 100);
            e.clipPositions(pos);
            expectEquals(pos[0], parked);
            expect(pos[1] > parked, "the other clip keeps running");
            e.startClip(0);
            run(e, 2);
            e.clipPositions(pos);
            expect(pos[0] < 0.2, "restarted from the beginning");
            run(e, 20);
            expectWithinAbsoluteError(e.modulatedValue("fx_ra_mix"), full, 1.0e-4f);
            e.stopClip(7); // not loaded: harmless
            e.startClip(99);
            expect(!e.isClipPlaying(99));
        }

        beginTest("clip run state is stored in patches; patches without it load with every clip running");
        {
            Engine e;
            e.prepare(kRate, 128);
            ClipBank bank;
            Clip clip;
            clip.trigger = ClipTrigger::GlobalFree;
            clip.length = clip.loopEnd = 8.0;
            ClipLane lane;
            lane.points = {{0.0, 1.0f}, {8.0, 1.0f}};
            clip.lanes = {lane};
            bank.clips.assign(3, clip);
            e.clips().replace(bank);
            const std::string allRunning = patchToJson(capturePatch(e));
            expect(allRunning.find("clipsStopped") == std::string::npos, "all running: the document is unchanged");
            e.stopClip(1);
            e.stopClip(5); // no such clip: not stored
            const Patch saved = capturePatch(e);
            expectEquals(static_cast<int>(saved.clipsStopped), 2);
            Patch loaded;
            std::string error;
            expect(patchFromJson(patchToJson(saved), loaded, error), error);
            expectEquals(static_cast<int>(loaded.clipsStopped), 2);
            Engine f;
            f.prepare(kRate, 128);
            applyPatch(f, loaded);
            expect(f.isClipPlaying(0) && !f.isClipPlaying(1) && f.isClipPlaying(2), "stopped clip restored");
            // A patch with everything running starts what a previous patch had stopped.
            Patch plain;
            expect(patchFromJson(allRunning, plain, error), error);
            applyPatch(f, plain);
            expect(f.isClipPlaying(0) && f.isClipPlaying(1) && f.isClipPlaying(2), "running restored");
        }

        beginTest("motion recorder: two parameters into two lanes of one clip (a vector path)");
        {
            Engine e;
            e.prepare(kRate, 128);
            e.setParam("vector_x", 0.2f);
            e.setParam("vector_y", 0.8f);
            std::vector<float> l(128), r(128);
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 120.0;
            const auto proc = [&](int blocks)
            {
                for (int i = 0; i < blocks; ++i)
                {
                    e.process(l.data(), r.data(), 128, nullptr, 0, &host);
                    e.motion().drain();
                }
            };
            proc(10);
            MotionSettings ms;
            ms.clip = 2;
            ms.lane = 0;
            ms.param = "vector_x";
            ms.param2 = "vector_y";
            ms.lane2 = 1;
            expect(e.recordMotion(ms), "arms");
            expect(e.modRoutes().routesFor("vector_x").size() == 1 && e.modRoutes().routesFor("vector_y").size() == 1,
                   "one route each");
            proc(30);
            for (int step = 1; step <= 100; ++step)
            {
                e.setParam("vector_x", 0.2f + 0.6f * static_cast<float>(step) / 100.0f); // right
                e.setParam("vector_y", 0.8f - 0.6f * static_cast<float>(step) / 100.0f); // down
                proc(3);
            }
            proc(40);
            e.motion().stop();
            e.motion().drain();
            const auto bank = e.clips().snapshot();
            expect(bank.clips.size() == 3 && bank.clips[2].lanes.size() == 2, "clip with two lanes");
            const auto& px = bank.clips[2].lanes[0].points;
            const auto& py = bank.clips[2].lanes[1].points;
            expect(px.size() >= 2 && py.size() >= 2, "both lanes have points");
            expect(px.back().y > 0.75f && py.back().y < 0.25f,
                   "x goes up, y goes down: " + String(px.back().y) + " " + String(py.back().y));
            expectEquals(e.motion().dropped(), 0);

            // A bad second parameter is refused and does not change the first take.
            MotionSettings bad = ms;
            bad.param2 = "vector_x";
            expect(!e.recordMotion(bad), "same parameter twice");
            bad.param2 = "nope";
            expect(!e.recordMotion(bad), "unknown parameter");
            bad.param2 = "vector_y";
            bad.lane2 = 0;
            expect(!e.recordMotion(bad), "same lane twice");
        }

        beginTest("an oscillator type that only the scenes hold (Modal) is armed before the vector pad selects it");
        {
            const int typeIndex = ParameterMap::indexOf("osc2_type");
            const auto& choices = ParameterMap::params()[static_cast<size_t>(typeIndex)].choices;
            for (const char* name : {"Modal"})
            {
                const auto found = std::find(choices.begin(), choices.end(), std::string(name));
                expect(found != choices.end(), std::string(name) + " is an oscillator type");
                if (found == choices.end())
                    continue;
                const float type = static_cast<float>(found - choices.begin());
                // Stored while the host type is still the default, and before and after prepare().
                for (const bool storeBeforePrepare : {false, true})
                {
                    Engine e;
                    e.setParam("osc1_level", 0.0f);
                    e.setParam("osc2_level", 1.0f);
                    e.setParam("amp_sustain", 1.0f);
                    auto values = e.getAllParams();
                    values[static_cast<size_t>(typeIndex)] = type;
                    if (storeBeforePrepare)
                        for (int k = 0; k < 4; ++k)
                            e.scenes().store(k, values);
                    e.prepare(kRate, 128);
                    if (!storeBeforePrepare)
                        for (int k = 0; k < 4; ++k)
                            e.scenes().store(k, values);
                    e.setParam("vector_smooth", 0.0f);
                    e.setParam("vector_on", 1.0f);
                    e.noteOn(48, 1.0f);
                    double energy = 0.0;
                    std::vector<float> l(128), r(128);
                    for (int i = 0; i < 60; ++i)
                    {
                        e.process(l.data(), r.data(), 128);
                        for (int k = 0; k < 128; ++k)
                            energy += static_cast<double>(l[static_cast<size_t>(k)]) * l[static_cast<size_t>(k)];
                    }
                    expect(e.modulatedValue("osc2_type") == type, std::string(name) + " reaches the slot");
                    expect(energy > 1.0e-6, std::string(name) + " sounds when only the scenes select it (" +
                                                (storeBeforePrepare ? "stored before prepare" : "stored after prepare") +
                                                ")");
                }
            }
        }

        beginTest("a wavetable that only the scenes select has its spectral analysis built when the scene is stored");
        {
            Engine e;
            e.prepare(kRate, 128);
            const int tableIndex = ParameterMap::indexOf("osc3_table");
            auto values = e.getAllParams();
            values[static_cast<size_t>(tableIndex)] = static_cast<float>(WavetableBank::numTables - 1);
            e.scenes().store(0, values);
            expect(spectral::FactoryTables::instance().get(WavetableBank::numTables - 1) != nullptr);
        }
    }
};

static VectorTests vectorTests;
