// SPDX-License-Identifier: MIT
// Parameter change poll with sources, engine latency, tail and reset.
#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/crucible.h>
#include "AllocCounter.h"
#include "TestSupport.h"

#include <algorithm>
#include <atomic>
#include <map>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

namespace
{
std::map<int, int> poll(Engine& e)
{
    std::vector<int> index(4096), source(4096);
    const int n = e.paramChanges(index.data(), source.data(), static_cast<int>(index.size()));
    std::map<int, int> out;
    for (int i = 0; i < n; ++i)
        out[index[static_cast<size_t>(i)]] = source[static_cast<size_t>(i)];
    return out;
}

float peak(Engine& e, int samples)
{
    std::vector<float> l(512), r(512);
    float p = 0.0f;
    for (int done = 0; done < samples; done += 512)
    {
        e.process(l.data(), r.data(), 512);
        for (size_t i = 0; i < 512; ++i)
            p = std::max({p, std::abs(l[i]), std::abs(r[i])});
    }
    return p;
}
} // namespace

class HostPlumbingTests : public UnitTest
{
public:
    HostPlumbingTests() : UnitTest("Host plumbing", "Engine") {}

    void runTest() override
    {
        beginTest("parameter changes: host, MIDI CC, patch load and scene recall carry their sources");
        {
            Engine e;
            e.prepare(48000.0, 512);
            expect(poll(e).empty(), "nothing changed after construction");
            const int cutoff = e.model().indexOf("filter_cutoff");
            const int res = e.model().indexOf("filter_res");
            const int drive = e.model().indexOf("fx_ra_mix");

            e.setParam(cutoff, 1234.0f);
            auto changes = poll(e);
            expectEquals(static_cast<int>(changes.size()), 1);
            expectEquals(changes[cutoff], 0);
            expect(poll(e).empty(), "drained");

            e.setParamRt(res, 0.5f);
            changes = poll(e);
            expectEquals(changes[res], 0);

            e.mapCC(74, cutoff);
            expect(e.midi(0xb0, 74, 100)); // control thread form
            changes = poll(e);
            expectEquals(changes[cutoff], 1);
            e.setParam(cutoff, 500.0f); // the scope ended: back to host
            expectEquals(poll(e)[cutoff], 0);

            const MidiInput cc{0, 0xb0, 74, 20}; // audio thread form
            std::vector<float> l(256), r(256);
            e.process(l.data(), r.data(), 256, &cc, 1);
            changes = poll(e);
            expectEquals(changes[cutoff], 1);

            expect(e.ccMap().armLearn("fx_ra_mix"));
            expect(e.midi(0xb0, 21, 64));
            expectEquals(poll(e)[drive], 1);

            e.setParam(drive, 0.9f);
            Patch p = capturePatch(e);
            poll(e);
            e.setParam(cutoff, 3000.0f);
            poll(e);
            applyPatch(e, p);
            changes = poll(e);
            expect(changes.size() > 100, "a patch load marks every parameter");
            expectEquals(changes[cutoff], 2);
            expectEquals(changes[drive], 2);
            for (const auto& [index, source] : changes)
                expectEquals(source, 2);

            e.storeScene(3);
            e.setParam(cutoff, 777.0f);
            poll(e);
            expect(e.recallScene(3));
            changes = poll(e);
            expectEquals(changes[cutoff], 2);
            e.morphScenes(3, 3, 0.5f);
            expectEquals(poll(e)[cutoff], 2);

            // seq_pattern coupling reports seq_length as internal
            const int pattern = e.model().indexOf("seq_pattern");
            const int length = e.model().indexOf("seq_length");
            e.sequencer().edit([](SeqBank& b) { b.pattern[2].lengthBars = 3; });
            e.setParam(pattern, 2.0f);
            changes = poll(e);
            expectEquals(changes[pattern], 0);
            expectEquals(changes[length], 2);
        }

        beginTest("parameter changes: cap, index order and no allocation");
        {
            Engine e;
            e.prepare(48000.0, 512);
            for (int i = 5; i >= 0; --i)
                e.setParam(i, e.model().params()[static_cast<size_t>(i)].max);
            int index[4] = {}, source[4] = {};
            expectEquals(e.paramChanges(index, source, 4), 4);
            for (int i = 0; i < 4; ++i)
                expectEquals(index[i], i);
            expectEquals(e.paramChanges(index, nullptr, 4), 2);
            expectEquals(index[0], 4);
            expectEquals(e.paramChanges(index, nullptr, 4), 0);
            expectEquals(e.paramChanges(nullptr, nullptr, 4), 0);

            const int cutoff = e.model().indexOf("filter_cutoff");
            std::vector<float> l(256), r(256);
            const MidiInput cc{0, 0xb0, 74, 20};
            e.mapCC(74, cutoff);
            gTrackAllocations = true;
            gAllocations = 0;
            e.setParamRt(cutoff, 900.0f);
            e.process(l.data(), r.data(), 256, &cc, 1);
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "marking from the audio thread allocates nothing");
        }

        beginTest("servicePending: true after setParamRt of a table / type parameter until service() has run");
        {
            Engine e;
            e.prepare(48000.0, 512);
            expect(!e.servicePending());
            e.setParamRt(e.model().indexOf("filter_cutoff"), 900.0f);
            expect(!e.servicePending(), "a plain parameter needs no service");
            e.setParamRt(e.model().indexOf("osc1_table"), 14.0f);
            expect(e.servicePending());
            e.service();
            expect(!e.servicePending());
            e.setParamRt(e.model().indexOf("osc2_type"), 3.0f);
            expect(e.servicePending());
            e.service();
            expect(!e.servicePending());
        }

        beginTest("parameter changes: hidden (non-exposed) parameters are never reported");
        {
            // A product that lists its surface hides every other parameter from hosts.
            std::string error;
            auto engine = Engine::fromProduct(R"({"format":"crucible-product","version":1,"id":"hid",
                "surface":{"expose":"listed","pages":[{"id":"p","params":["filter_cutoff","filter_res"]}]}})",
                                              error);
            expect(engine != nullptr, error);
            if (engine == nullptr)
                return;
            Engine& e = *engine;
            e.prepare(48000.0, 512);
            int hidden = -1;
            for (int i = 0; i < e.model().size() && hidden < 0; ++i)
                if (!e.model().params()[static_cast<size_t>(i)].exposed)
                    hidden = i;
            expect(hidden >= 0, "a listed surface hides the rest");
            if (hidden >= 0)
            {
                poll(e);
                e.setParam(hidden, e.model().params()[static_cast<size_t>(hidden)].max);
                e.setParam("filter_cutoff", 1500.0f);
                const auto changes = poll(e);
                expect(changes.count(hidden) == 0, "the hidden parameter is not in the poll");
                expect(changes.count(e.model().indexOf("filter_cutoff")) == 1, "the exposed one is");
                expect(poll(e).empty(), "and the mark of the hidden one was dropped, not kept for later");
            }
        }

        beginTest("latency: the engine total equals what the modules report, and a change is announced by the poll");
        {
            Engine e;
            e.prepare(48000.0, 512);
            poll(e);
            expectEquals(e.latencySamples(), 0);
            e.setParam("fx_mx_on", 1.0f);
            e.setParam("fx_mx_lookahead", 5.0f);
            e.prepare(48000.0, 512); // prepare publishes the latency before any block runs
            auto* mx = e.coreEngine().engine().effects().findCapability<LatencyReporter>("fx.maximizer");
            expect(mx != nullptr);
            const int reported = mx != nullptr ? mx->latencySamples() : -1;
            expect(reported > 0, "the maximiser looks ahead");
            expectEquals(e.latencySamples(), reported);
            std::vector<int> index(4096), source(4096);
            int n = e.paramChanges(index.data(), source.data(), 4096);
            expectEquals(index[0], Engine::kLatencyChanged);
            expectEquals(source[0], 2);
            expect(n > 1);
            n = e.paramChanges(index.data(), source.data(), 4096);
            expectEquals(n, 0);
            e.setParam("fx_mx_on", 0.0f);
            e.prepare(48000.0, 512);
            expectEquals(e.latencySamples(), 0);
            n = e.paramChanges(index.data(), source.data(), 4096);
            expect(n >= 1 && index[0] == Engine::kLatencyChanged);
        }

        beginTest("tail: the longest tail of the enabled modules");
        {
            Engine e;
            e.prepare(48000.0, 512);
            expectEquals(e.tailSeconds(), 0.0);
            e.setParam("fx_ra_on", 1.0f);
            e.setParam("fx_ra_decay", 4.0f);
            e.setParam("fx_ra_predelay", 0.0f);
            peak(e, 16384); // modules publish their settings while processing
            expectWithinAbsoluteError(e.tailSeconds(), 4.0, 0.05);
            e.setParam("fx_ra_on", 0.0f);
            e.prepare(48000.0, 512);
            expectEquals(e.tailSeconds(), 0.0);
            e.setParam("fx_delay_pro_on", 1.0f);
            e.setParam("fx_delay_pro_sync", 0.0f);
            e.setParam("fx_delay_pro_time", 100.0f);
            e.setParam("fx_delay_pro_feedback", 0.5f);
            peak(e, 16384);                                       // the parameters are smoothed
            expectWithinAbsoluteError(e.tailSeconds(), 1.1, 0.1); // about 11 repeats of 100 ms to fall 60 dB
        }

        beginTest("tail: a change is announced by the poll as the tail pseudo index, after the latency entry");
        {
            Engine e;
            e.prepare(48000.0, 512);
            std::vector<int> index(64), source(64);
            expectEquals(e.paramChanges(index.data(), source.data(), 64), 0, "nothing changed after prepare");
            e.setParam("fx_ra_on", 1.0f);
            e.setParam("fx_ra_decay", 4.0f);
            peak(e, 16384);
            expect(e.tailSeconds() > 1.0);
            int n = e.paramChanges(index.data(), source.data(), 64);
            int tailAt = -1;
            for (int i = 0; i < n; ++i)
                if (index[static_cast<size_t>(i)] == Engine::kTailChanged)
                    tailAt = i;
            expect(tailAt >= 0, "the tail change is reported");
            expect(tailAt >= 0 && source[static_cast<size_t>(tailAt)] == 2, "with the internal source");
            expectEquals(Engine::kTailChanged, -2);
            n = e.paramChanges(index.data(), source.data(), 64);
            for (int i = 0; i < n; ++i)
                expect(index[static_cast<size_t>(i)] != Engine::kTailChanged, "reported once");
            // A buffer that only fits the latency entry leaves the tail change for the next call.
            e.setParam("fx_ra_decay", 1.0f);
            peak(e, 16384);
            std::vector<int> one(1);
            n = e.paramChanges(one.data(), nullptr, 1);
            expectEquals(n, 1);
            expectEquals(one[0], Engine::kTailChanged);
        }

        beginTest("setParamRt rounds int and choice values exactly as setParam does");
        {
            Engine e;
            e.prepare(48000.0, 512);
            const int type = e.model().indexOf("osc1_type");
            expect(type >= 0);
            e.setParam(type, 2.7f);
            const float viaSet = e.getParam(type);
            expectEquals(viaSet, 3.0f);
            e.setParam(type, 0.0f);
            e.setParamRt(type, 2.7f);
            expectEquals(e.getParam(type), viaSet);
            e.setParamRt(type, 2.4f);
            expectEquals(e.getParam(type), 2.0f);
            const int cutoff = e.model().indexOf("filter_cutoff");
            e.setParamRt(cutoff, 1234.5f);
            expectEquals(e.getParam(cutoff), 1234.5f); // floats are not rounded
        }

        beginTest("reset: a ringing reverb is silent in the next block, without allocating");
        {
            Engine e;
            e.prepare(48000.0, 512);
            e.setParam("fx_ra_on", 1.0f);
            e.setParam("fx_ra_mix", 1.0f);
            e.setParam("fx_ra_decay", 8.0f);
            e.setParam("fx_delay_pro_on", 1.0f);
            e.setParam("fx_delay_pro_feedback", 0.8f);
            e.noteOn(60, 1.0f);
            peak(e, 4096);
            e.noteOff(60);
            expect(peak(e, 4096) > 1.0e-4f, "still ringing before reset");
            e.noteOn(64, 1.0f);
            peak(e, 512);
            gTrackAllocations = true;
            gAllocations = 0;
            e.reset();
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "reset allocates nothing");
            expectEquals(e.activeVoiceCount(), 0);
            // The algorithmic reverb keeps an anti-denormal offset of about 1e-19 (-370 dB); anything above -240 dB is a leak.
            expect(peak(e, 2048) < 1.0e-12f, "silent after reset");
            e.noteOn(60, 1.0f);
            expect(peak(e, 2048) > 0.0f, "plays again");
        }

        beginTest("reset while another thread plays notes: the queue is drained from the consumer side and nothing replays");
        {
            Engine e;
            e.prepare(48000.0, 512);
            std::atomic<bool> stop{false};
            std::thread player(
                [&]
                {
                    int note = 40;
                    while (!stop.load())
                    {
                        e.noteOn(note, 0.8f);
                        e.noteOff(note);
                        note = 40 + (note + 1) % 40;
                    }
                });
            std::vector<float> l(64), r(64);
            for (int i = 0; i < 3000; ++i)
            {
                e.reset();
                e.process(l.data(), r.data(), 64);
            }
            stop.store(true);
            player.join();
            e.reset();
            expectEquals(peak(e, 2048), 0.0f, "silent after the final reset");
            expectEquals(e.activeVoiceCount(), 0);
        }

        beginTest("a block with more than 256 sorted MIDI events delivers every one of them (600 here), the closing "
                  "note-offs included");
        {
            Engine e;
            e.prepare(48000.0, 512);
            e.setParam("amp_release", 0.05f);
            std::vector<MidiInput> events;
            events.push_back({0, 0x90, 60, 100});
            for (int i = 1; i < 599; ++i)
                events.push_back({i % 400, 0xb0, 1, static_cast<std::uint8_t>(i % 100)});
            std::stable_sort(events.begin(), events.end(),
                             [](const MidiInput& a, const MidiInput& b) { return a.position < b.position; });
            events.push_back({500, 0x80, 60, 0});
            std::vector<float> l(512), r(512);
            e.process(l.data(), r.data(), 512, events.data(), static_cast<int>(events.size()));
            expectEquals(static_cast<int>(events.size()), 600);
            for (int i = 0; i < 40; ++i)
                e.process(l.data(), r.data(), 512);
            expectEquals(e.activeVoiceCount(), 0, "the note-off at event 600 arrived");
            expectEquals(peak(e, 2048), 0.0f, "silent");
        }

        beginTest("string_view ids: a long literal never allocates in indexOf, setParam, getParam or setParamText");
        {
            Engine e;
            e.prepare(48000.0, 512);
            const std::string_view longId = "fx_reverb_predelay_sync_mode_long_name";
            gTrackAllocations = true;
            gAllocations = 0;
            const int a = e.model().indexOf("fx_delay_feedback_filter_cutoff_long_literal");
            const int b = e.model().indexOf(longId);
            const bool setLong = e.setParam("fx_delay_feedback_filter_cutoff_long_literal", 1.0f);
            const float got = e.getParam("fx_delay_feedback_filter_cutoff_long_literal");
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "no allocation");
            expectEquals(a, -1);
            expectEquals(b, -1);
            expect(!setLong, "unknown long id is refused");
            expectEquals(got, 0.0f);
            const std::string_view known = "filter_cutoff";
            expectEquals(e.model().indexOf(known), e.model().indexOf(std::string("filter_cutoff")));
            expect(e.model().indexOf(known) >= 0, "view of a known id resolves");
            expectEquals(e.model().indexOf(std::string_view("filter_cutoffX", 13)), e.model().indexOf("filter_cutoff"),
                         "a view stops at its length");
        }

        beginTest("a patch that selects a non-default factory table: the first process allocates nothing");
        {
            const char* ids[] = {"osc1_table", "osc2_table"};
            const float tables[] = {9.0f, 6.0f};
            // set before prepare (Engine), then the same through the C API
            {
                Engine e;
                for (int k = 0; k < 2; ++k)
                    e.setParam(ids[k], tables[k]);
                e.prepare(48000.0, 512);
                std::vector<float> l(512), r(512);
                e.noteOn(60, 1.0f);
                gTrackAllocations = true;
                gAllocations = 0;
                e.process(l.data(), r.data(), 512);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "Engine::setParam before prepare");
                float p = 0.0f;
                for (float v : l)
                    p = std::max(p, std::abs(v));
                expect(p > 0.0f, "sounds");
            }
            {
                CrucibleEngine* c = crucible_create(nullptr);
                const int i0 = crucible_param_index(c, "osc1_table");
                const int i1 = crucible_param_index(c, "osc2_table");
                crucible_set_param(c, i0, tables[0]);
                crucible_set_param(c, i1, tables[1]);
                crucible_prepare(c, 48000.0, 512);
                std::vector<float> l(512), r(512);
                crucible_note_on(c, 60, 1.0f);
                gTrackAllocations = true;
                gAllocations = 0;
                crucible_process(c, l.data(), r.data(), 512, nullptr, 0);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "crucible_set_param before prepare");
                crucible_destroy(c);
            }
            // crucible_param_index is [any]: no allocation, even for an id too long for a small string
            {
                CrucibleEngine* c = crucible_create(nullptr);
                const char* longId = "an_unknown_parameter_id_longer_than_any_small_string_buffer";
                int found = 0;
                int missing = 0;
                gTrackAllocations = true;
                gAllocations = 0;
                found = crucible_param_index(c, "osc1_table");
                missing = crucible_param_index(c, longId) + crucible_param_index(c, nullptr);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "crucible_param_index allocates");
                expect(found >= 0, "known id found");
                expectEquals(missing, -2, "unknown id and NULL give -1");
                crucible_destroy(c);
            }
            // a patch load
            {
                Engine src;
                src.prepare(48000.0, 512);
                for (int k = 0; k < 2; ++k)
                    src.setParam(ids[k], tables[k]);
                const Patch patch = capturePatch(src);
                Engine e;
                e.prepare(48000.0, 512);
                applyPatch(e, patch);
                std::vector<float> l(512), r(512);
                e.noteOn(60, 1.0f);
                gTrackAllocations = true;
                gAllocations = 0;
                e.process(l.data(), r.data(), 512);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "applyPatch");
                expectEquals(e.getParam(ids[0]), tables[0]);
            }
            {
                CrucibleEngine* a = crucible_create(nullptr);
                crucible_prepare(a, 48000.0, 512);
                crucible_set_param(a, crucible_param_index(a, "osc1_table"), tables[0]);
                const int need = crucible_save_patch(a, "t", nullptr, 0);
                std::vector<char> json(static_cast<size_t>(need) + 1);
                crucible_save_patch(a, "t", json.data(), json.size());
                crucible_destroy(a);
                CrucibleEngine* c = crucible_create(nullptr);
                crucible_prepare(c, 48000.0, 512);
                char err[256] = {};
                expectEquals(crucible_load_patch(c, json.data(), err, sizeof err), 1, err);
                std::vector<float> l(512), r(512);
                crucible_note_on(c, 60, 1.0f);
                gTrackAllocations = true;
                gAllocations = 0;
                crucible_process(c, l.data(), r.data(), 512, nullptr, 0);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "crucible_load_patch");
                crucible_destroy(c);
            }
        }
    }
};

static HostPlumbingTests hostPlumbingTests;
