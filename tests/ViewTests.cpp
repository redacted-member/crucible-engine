// SPDX-License-Identifier: MIT
#include <crucible/Cloud.h>
#include <crucible/Engine.h>
#include <crucible/Sample.h>
#include <crucible/Views.h>
#include "AllocCounter.h"
#include "TestSupport.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
#define CRUCIBLE_VIEW_TESTS_SANITISED 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
#define CRUCIBLE_VIEW_TESTS_SANITISED 1
#endif
#endif

namespace
{
// Timing claims mean nothing under the sanitizers' instrumentation; the checks still run, the thresholds do not apply.
#ifdef CRUCIBLE_VIEW_TESTS_SANITISED
constexpr bool kSanitised = true;
#else
constexpr bool kSanitised = false;
#endif
constexpr double kTwoPi = 6.283185307179586;

// A producer whose payload is self-checking: every element equals the number of the publication that wrote it.
struct Counter
{
    std::atomic<std::uint64_t> published{0};
    std::uint64_t next = 0;
};

void produceCounter(void* context, const ViewArgs& args, ViewWriter& out)
{
    auto* c = static_cast<Counter*>(context);
    const float value = static_cast<float>(++c->next);
    if (args.op == 0)
        out.scalar(value);
    else
    {
        const int n = args.index;
        for (int i = 0; i < n; ++i)
            out.push(value);
    }
    c->published.store(c->next);
}

int addCounterView(ViewHub& hub, Counter& c, const char* id, ViewKind kind, int size)
{
    ViewHub::Spec spec;
    spec.info = {"test", id, "", kind, size, 1, true};
    spec.produce = &produceCounter;
    spec.context = &c;
    spec.args = {kind == ViewKind::Scalar ? 0 : 1, size, 0};
    return hub.add(std::move(spec));
}

AudioData sineAudio(int frames, double freq, double rate = 48000.0)
{
    AudioData a;
    a.sampleRate = rate;
    a.channels.assign(1, std::vector<float>(static_cast<size_t>(frames)));
    for (int i = 0; i < frames; ++i)
        a.channels[0][static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(kTwoPi * freq * i / rate));
    return a;
}

void run(Engine& e, int blocks, int size = 256)
{
    std::vector<float> l(static_cast<size_t>(size)), r(static_cast<size_t>(size));
    for (int b = 0; b < blocks; ++b)
        e.process(l.data(), r.data(), size);
}

// Reads a view into a vector; empty when nothing was published.
std::vector<float> readView(const Engine& e, int view, std::uint64_t* sequence = nullptr)
{
    std::vector<float> buffer(static_cast<size_t>(e.views().info(view).maxElements));
    std::uint64_t seq = 0;
    const int n = e.views().read(view, buffer.data(), static_cast<int>(buffer.size()), &seq);
    if (sequence != nullptr)
        *sequence = seq;
    buffer.resize(static_cast<size_t>(std::max(n, 0)));
    return buffer;
}

int viewOf(Engine& e, const std::string& instance, const std::string& id)
{
    return e.views().find(instance, id);
}

float scalarOf(const Engine& e, int view)
{
    const auto v = readView(e, view);
    return v.empty() ? -12345.0f : v[0];
}

bool sameBits(float a, float b)
{
    return std::memcmp(&a, &b, sizeof a) == 0;
}
} // namespace

class ViewTests : public UnitTest
{
public:
    ViewTests() : UnitTest("Views", "Engine") {}

    void runTest() override
    {
        beginTest("hub: payload kinds, sequence numbers, subscriptions and rate limits");
        {
            ViewHub hub;
            hub.prepare(48000.0);
            Counter cs, cv;
            const int scalar = addCounterView(hub, cs, "scalar", ViewKind::Scalar, 1);
            const int vector = addCounterView(hub, cv, "vector", ViewKind::Vector, 5);
            expectEquals(hub.count(), 2);
            expectEquals(hub.find("test", "vector"), vector);
            expectEquals(hub.find("test", "nope"), -1);
            expectEquals(addCounterView(hub, cs, "scalar", ViewKind::Scalar, 1), -1, "no duplicates");
            float out[8];
            std::uint64_t seq = 99;
            expectEquals(hub.read(vector, out, 8, &seq), 0, "nothing published without a subscriber");
            expectEquals(static_cast<int>(seq), 0);
            expectEquals(hub.read(77, out, 8, &seq), -1, "unknown view");
            for (int i = 0; i < 10; ++i)
                hub.publish(64);
            expectEquals(static_cast<int>(cv.published.load()), 0, "an unsubscribed view is never produced");

            const int sub = hub.subscribe(vector, 0.0f);
            expect(sub > 0);
            expectEquals(hub.subscribers(vector), 1);
            hub.publish(64);
            hub.publish(64);
            expectEquals(hub.read(vector, out, 8, &seq), 5);
            expectEquals(static_cast<int>(seq), 2);
            expect(out[0] == 2.0f && out[4] == 2.0f);
            expectEquals(hub.read(vector, out, 3, &seq), -2, "capacity too small");
            expectEquals(static_cast<int>(cs.published.load()), 0, "the scalar was not asked for");

            const int sub2 = hub.subscribe(scalar, 0.0f);
            hub.publish(64);
            expectEquals(hub.read(scalar, out, 8, &seq), 1);
            expect(out[0] == 1.0f && seq == 1);
            hub.unsubscribe(sub2);
            hub.publish(64);
            hub.read(scalar, out, 8, &seq);
            expect(out[0] == 1.0f && seq == 1, "unsubscribed again: it stops");
            hub.unsubscribe(sub);

            // Rate limit: 100 Hz at 48 kHz is one publication per 480 samples, whatever the block size.
            Counter slow;
            const int limited = addCounterView(hub, slow, "limited", ViewKind::Vector, 4);
            const int s100 = hub.subscribe(limited, 100.0f);
            for (int i = 0; i < 1000; ++i)
                hub.publish(48); // one second
            expect(std::abs(static_cast<int>(slow.published.load()) - 100) <= 1,
                   "published " + String(static_cast<int>(slow.published.load())));
            // The fastest subscriber wins, an unlimited one wins over any rate.
            const int s400 = hub.subscribe(limited, 400.0f);
            const auto before = slow.published.load();
            for (int i = 0; i < 1000; ++i)
                hub.publish(48);
            expect(std::abs(static_cast<int>(slow.published.load() - before) - 400) <= 2, "two subscribers");
            hub.unsubscribe(s400);
            hub.unsubscribe(s100);

            // A mask passes its bits through untouched.
            struct Bits
            {
                static void produce(void*, const ViewArgs&, ViewWriter& out)
                {
                    out.pushWord(0x7fc00001U); // a NaN payload must survive
                    out.pushWord(0x80000000U);
                    out.pushWord(1U);
                }
            };
            ViewHub::Spec spec;
            spec.info = {"test", "mask", "", ViewKind::Mask, 4, 1, true};
            spec.produce = &Bits::produce;
            const int mask = hub.add(std::move(spec));
            hub.subscribe(mask);
            hub.publish(1);
            std::uint32_t words[4] = {};
            expectEquals(hub.read(mask, reinterpret_cast<float*>(words), 4, &seq), 3);
            expect(words[0] == 0x7fc00001U && words[1] == 0x80000000U && words[2] == 1U, "mask words");
        }

        beginTest("hub: no tearing under a writer and several readers (vectors, scalars, a mask-sized payload)");
        {
            ViewHub hub;
            hub.prepare(48000.0);
            Counter big, small, one;
            const int vecBig = addCounterView(hub, big, "big", ViewKind::Vector, 4096);
            const int vecSmall = addCounterView(hub, small, "small", ViewKind::Vector, 7);
            const int scalar = addCounterView(hub, one, "one", ViewKind::Scalar, 1);
            hub.subscribe(vecBig);
            hub.subscribe(vecSmall);
            hub.subscribe(scalar);
            std::atomic<bool> stop{false};
            std::atomic<int> torn{0}, backwards{0}, reads{0};
            std::vector<std::thread> readers;
            for (int t = 0; t < 4; ++t)
                readers.emplace_back(
                    [&, t]
                    {
                        std::vector<float> buffer(4096);
                        std::uint64_t lastBig = 0, lastSmall = 0, lastOne = 0;
                        while (!stop.load())
                        {
                            std::uint64_t seq = 0;
                            int n = hub.read(vecBig, buffer.data(), 4096, &seq);
                            if (n == 4096)
                            {
                                for (int i = 1; i < n; ++i)
                                    if (buffer[static_cast<size_t>(i)] != buffer[0])
                                    {
                                        torn.fetch_add(1);
                                        break;
                                    }
                                if (static_cast<std::uint64_t>(buffer[0]) != seq || seq < lastBig)
                                    backwards.fetch_add(1);
                                lastBig = seq;
                            }
                            n = hub.read(vecSmall, buffer.data(), 4096, &seq);
                            if (n == 7)
                            {
                                for (int i = 1; i < n; ++i)
                                    if (buffer[static_cast<size_t>(i)] != buffer[0])
                                        torn.fetch_add(1);
                                if (static_cast<std::uint64_t>(buffer[0]) != seq || seq < lastSmall)
                                    backwards.fetch_add(1);
                                lastSmall = seq;
                            }
                            n = hub.read(scalar, buffer.data(), 4096, &seq);
                            if (n == 1)
                            {
                                if (static_cast<std::uint64_t>(buffer[0]) != seq || seq < lastOne)
                                    backwards.fetch_add(1);
                                lastOne = seq;
                            }
                            reads.fetch_add(1);
                            if (t == 3)
                                std::this_thread::yield(); // one slow reader keeps slots pinned longer
                        }
                    });
            const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
            std::uint64_t publishes = 0;
            while (std::chrono::steady_clock::now() < end)
            {
                for (int i = 0; i < 64; ++i)
                {
                    hub.publish(32);
                    ++publishes;
                }
            }
            stop = true;
            for (auto& r : readers)
                r.join();
            expectEquals(torn.load(), 0, "torn snapshots");
            expectEquals(backwards.load(), 0, "payload and sequence agree and never go backwards");
            expect(reads.load() > 100, "readers ran " + String(reads.load()));
            logMessage("publications " + std::to_string(publishes) + ", reads " + std::to_string(reads.load()) +
                       ", skipped " + std::to_string(hub.dropped(vecBig)));
        }

        beginTest("engine: master views equal the meter, loudness, scope, spectrum and sounding-notes accessors");
        {
            Engine e;
            e.prepare(kRate, 512);
            const int meter = viewOf(e, "master", "meter");
            const int loud = viewOf(e, "master", "loudness");
            const int scope = viewOf(e, "master", "scope");
            const int notes = viewOf(e, "engine", "sounding_notes");
            expect(meter >= 0 && loud >= 0 && scope >= 0 && notes >= 0);
            expect(readView(e, meter).empty(), "silent until subscribed");
            std::vector<int> subs;
            for (const int v : {meter, loud, scope, notes})
                subs.push_back(e.views().subscribe(v, 0.0f));
            std::vector<int> spectra;
            for (const int size : {256, 512, 1024, 2048, 4096})
            {
                spectra.push_back(viewOf(e, "master", "spectrum_" + std::to_string(size)));
                expect(spectra.back() >= 0);
                e.views().subscribe(spectra.back(), 0.0f);
            }
            e.noteOn(57, 1.0f);
            e.noteOn(64, 0.9f);
            run(e, 60);
            const auto m = readView(e, meter);
            const auto ref = e.meter();
            expectEquals(m.size(), size_t(3));
            expect(sameBits(m[0], ref.peakLeftDb) && sameBits(m[1], ref.peakRightDb) && (m[2] != 0.0f) == ref.clip, "meter");
            expect(m[0] > -60.0f, "the meter sees the notes");
            const auto l = readView(e, loud);
            const auto lref = e.loudness();
            expectEquals(l.size(), size_t(5));
            expect(sameBits(l[0], lref.momentaryLufs) && sameBits(l[1], lref.shortTermLufs) &&
                       sameBits(l[2], lref.truePeakDb) && sameBits(l[3], lref.maxTruePeakDb) &&
                       sameBits(l[4], lref.correlation),
                   "loudness");
            const auto sc = readView(e, scope);
            std::vector<float> sl, sr;
            e.analyzer().scope(sl, sr, MasterAnalyzer::scopeSize);
            expectEquals(sc.size(), size_t(2 * MasterAnalyzer::scopeSize));
            expect(std::equal(sl.begin(), sl.end(), sc.begin()) &&
                       std::equal(sr.begin(), sr.end(), sc.begin() + MasterAnalyzer::scopeSize),
                   "scope");
            int size = 256;
            for (const int s : spectra)
            {
                const auto view = readView(e, s);
                const auto old = e.analyzer().spectrum(size);
                expect(view == old, "spectrum " + String(size));
                size *= 2;
            }
            const auto mask = readView(e, notes);
            expectEquals(mask.size(), size_t(4));
            std::array<std::uint64_t, 2> bits{};
            e.coreEngine().soundingNotes(bits);
            std::uint32_t w[4];
            std::memcpy(w, mask.data(), sizeof w);
            expect((static_cast<std::uint64_t>(w[1]) << 32 | w[0]) == bits[0] &&
                       (static_cast<std::uint64_t>(w[3]) << 32 | w[2]) == bits[1],
                   "sounding notes");
            for (const int s : subs)
                e.views().unsubscribe(s);
        }

        beginTest(
            "engine: every parameter's modulated value, voice value and band equal the accessors; chosen voices and bands");
        {
            Engine e;
            e.prepare(kRate, 512);
            e.setParam("lfo1_rate", 6.0f);
            e.setParam("osc1_pos", 0.4f);
            e.setParam("filter_cutoff", 1200.0f);
            expect(e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_pos", 0.3f, 0, 0.0f, 0.0f}));
            expect(e.modRoutes().add({static_cast<int>(ModSource::Key), "osc1_pos", 0.5f, 0, 0.0f, 0.0f}));
            expect(e.modRoutes().add({static_cast<int>(ModSource::Key), "filter_cutoff", 0.4f, 0, 0.0f, 0.0f}));
            const auto& params = e.model().params();
            std::vector<int> mod(params.size()), voice(params.size()), band(params.size());
            for (size_t i = 0; i < params.size(); ++i)
            {
                mod[i] = viewOf(e, "params", params[i].id + ".mod");
                voice[i] = viewOf(e, "params", params[i].id + ".voice");
                band[i] = viewOf(e, "params", params[i].id + ".band");
                expect(mod[i] >= 0 && voice[i] >= 0 && band[i] >= 0, params[i].id);
                e.views().subscribe(mod[i]);
                e.views().subscribe(voice[i]);
                e.views().subscribe(band[i]);
            }
            const int pos = ParameterMap::indexOf("osc1_pos");
            const int cut = ParameterMap::indexOf("filter_cutoff");
            const int low = viewOf(e, "params", "osc1_pos.voice@0");
            const int high = viewOf(e, "params", "osc1_pos.voice@1");
            const int far = viewOf(e, "params", "osc1_pos.voice@15");
            expect(low >= 0 && high >= 0 && far >= 0);
            expectEquals(viewOf(e, "params", "osc1_pos.voice@999"), -1);
            expectEquals(viewOf(e, "params", "nope.voice@1"), -1);
            e.views().subscribe(low);
            e.views().subscribe(high);
            e.views().subscribe(far);
            auto& synth = e.coreEngine().engine();
            int idleChecks = 0;
            for (size_t i = 0; i < params.size(); ++i)
                if (sameBits(scalarOf(e, mod[i]), -12345.0f))
                    ++idleChecks;
            expectEquals(idleChecks, static_cast<int>(params.size()), "nothing before the first render");

            // Idle: voice value equals the modulated value, the band collapses and counts no voice.
            run(e, 4);
            for (size_t i = 0; i < params.size(); ++i)
            {
                expect(sameBits(scalarOf(e, mod[i]), synth.modulatedValue(static_cast<int>(i))),
                       params[i].id + " mod (idle)");
                expect(sameBits(scalarOf(e, voice[i]), synth.voiceModulatedValue(static_cast<int>(i))),
                       params[i].id + " voice (idle)");
            }
            const auto idle = readView(e, band[static_cast<size_t>(pos)]);
            expect(idle.size() == 3 && idle[0] == idle[1] && idle[2] == 0.0f);

            e.noteOn(36, 1.0f);
            e.noteOn(96, 1.0f);
            float lo = 9.0f, hi = -9.0f;
            for (int b = 0; b < 300; ++b)
            {
                run(e, 1);
                for (size_t i = 0; i < params.size(); ++i)
                {
                    const int pi = static_cast<int>(i);
                    expect(sameBits(scalarOf(e, mod[i]), synth.modulatedValue(pi)), params[i].id + " mod");
                    expect(sameBits(scalarOf(e, voice[i]), synth.voiceModulatedValue(pi)), params[i].id + " voice");
                }
                const auto bp = readView(e, band[static_cast<size_t>(pos)]);
                const auto bc = readView(e, band[static_cast<size_t>(cut)]);
                expect(bp.size() == 3 && bc.size() == 3 && bp[2] == 2.0f && bc[2] == 2.0f, "two voices");
                const float newest = synth.voiceModulatedValue(pos);
                expect(bp[0] <= newest && newest <= bp[1], "the newest voice is inside the band");
                expect(bp[0] <= bp[1] && bc[0] <= bc[1]);
                lo = std::min(lo, bp[1] - bp[0]);
                hi = std::max(hi, bp[1] - bp[0]);
                // The chosen voices are the band's ends (the low note has the lower key modulation).
                const float a = scalarOf(e, low), bhi = scalarOf(e, high);
                expect(std::min(a, bhi) == bp[0] && std::max(a, bhi) == bp[1], "voices 0 and 1 are the band's ends");
                expect(sameBits(scalarOf(e, far), synth.modulatedValue(pos)), "an idle voice reads the unmodulated value");
                if (hi > 0.2f && b > 20)
                    break;
            }
            expect(hi > 0.2f, "the band is open: key tracking differs per voice, width " + String(hi, 3));
            // Chosen voice N is the value of voice N: the newest one equals the .voice view.
            const float newest = scalarOf(e, voice[static_cast<size_t>(pos)]);
            expect(newest == scalarOf(e, high) || newest == scalarOf(e, low), "the newest voice is one of them");
        }

        beginTest("engine: clips, vector field, sequencer and voice count views");
        {
            Engine e;
            e.prepare(kRate, 512);
            const int clips = viewOf(e, "clips", "positions");
            const int vec = viewOf(e, "engine", "vector_position");
            const int voices = viewOf(e, "engine", "active_voices");
            const int step = viewOf(e, "sequencer", "step");
            const int running = viewOf(e, "sequencer", "running");
            for (const int v : {clips, vec, voices, step, running})
            {
                expect(v >= 0);
                e.views().subscribe(v);
            }
            e.noteOn(60, 1.0f);
            run(e, 20);
            std::array<double, kMaxClips> pos{};
            e.clipPositions(pos);
            const auto c = readView(e, clips);
            expectEquals(c.size(), size_t(kMaxClips));
            for (size_t i = 0; i < pos.size(); ++i)
                expect(c[i] == static_cast<float>(pos[i]), "clip " + String(static_cast<int>(i)));
            float x = 0, y = 0;
            e.vectorPosition(x, y);
            const auto vp = readView(e, vec);
            expect(vp.size() == 3 && vp[0] == x && vp[1] == y);
            expectEquals(static_cast<int>(scalarOf(e, voices)), e.activeVoiceCount());
            expectEquals(static_cast<int>(scalarOf(e, step)), e.sequencerStep());
            expectEquals(scalarOf(e, running) != 0.0f, e.sequencerRunning());
        }

        beginTest("engine: sample and cloud views equal sampleView and cloudView");
        {
            Engine e;
            e.prepare(kRate, 256);
            std::string error;
            expect(e.loadSample(0, sineAudio(48000, 440.0), "sine", error), error);
            e.setParam("osc1_type", 1.0f);
            e.setParam("osc1_smp_mode", 1.0f);
            e.setParam("osc1_grain_density", 60.0f);
            e.setParam("osc1_grain_size", 60.0f);
            e.setParam("osc2_type", 2.0f);
            e.setParam("osc2_level", 0.5f);
            CloudModel cloud = CloudModel::random(40, 7);
            e.setCloud(1, cloud);
            std::vector<Zone> zones(2);
            zones[0].keyHi = 59;
            zones[1].keyLo = 60;
            zones[1].root = 72.0f;
            zones[1].gainDb = -3.0f;
            e.setZones(0, zones);
            e.coreEngine().engine().sampleBank().setSlices(0, {0.25f, 0.5f});

            const char* ids[] = {"sample_info",      "sample_peaks",  "sample_slices",      "sample_zones",
                                 "sample_playheads", "sample_grains", "sample_zones_active"};
            std::vector<int> osc1;
            for (const char* id : ids)
            {
                osc1.push_back(viewOf(e, "osc1", id));
                expect(osc1.back() >= 0, id);
                e.views().subscribe(osc1.back());
            }
            const int points = viewOf(e, "osc2", "cloud_points");
            const int energy = viewOf(e, "osc2", "cloud_energy");
            const int state = viewOf(e, "osc2", "cloud_state");
            for (const int v : {points, energy, state})
            {
                expect(v >= 0);
                e.views().subscribe(v);
            }
            expect(readView(e, viewOf(e, "osc3", "sample_info")).empty(), "no sample in slot 3");
            e.noteOn(69, 1.0f);
            e.noteOn(48, 1.0f);
            run(e, 80);

            SynthEngine::SampleView sv;
            expect(e.sampleView(0, sv));
            const auto info = readView(e, osc1[0]);
            expect(info.size() == 3 && info[0] == static_cast<float>(sv.frames) &&
                       info[1] == static_cast<float>(sv.sampleRate) && info[2] == sv.detectedNote,
                   "info");
            const auto peaks = readView(e, osc1[1]);
            expectEquals(peaks.size(), sv.peakMin.size() + sv.peakMax.size());
            expect(
                std::equal(sv.peakMin.begin(), sv.peakMin.end(), peaks.begin()) &&
                    std::equal(sv.peakMax.begin(), sv.peakMax.end(), peaks.begin() + static_cast<long>(sv.peakMin.size())),
                "peaks");
            expect(readView(e, osc1[2]) == sv.slices && sv.slices.size() == 2, "slices");
            const auto z = readView(e, osc1[3]);
            expectEquals(z.size(), size_t(24));
            expect(z[2] == 59.0f && z[12 + 1] == 60.0f && z[12 + 5] == 72.0f && z[12 + 6] == -3.0f, "zones");
            expect(readView(e, osc1[4]) == sv.visual.playheads && !sv.visual.playheads.empty(), "playheads");
            const auto grains = readView(e, osc1[5]);
            expectEquals(grains.size(), sv.visual.grains.size() * 4);
            expect(!grains.empty(), "grains");
            for (size_t i = 0; i < sv.visual.grains.size(); ++i)
                expect(grains[4 * i] == sv.visual.grains[i].position && grains[4 * i + 1] == sv.visual.grains[i].width &&
                           grains[4 * i + 2] == sv.visual.grains[i].amplitude &&
                           grains[4 * i + 3] == sv.visual.grains[i].pan,
                       "grain");
            const auto activeZones = readView(e, osc1[6]);
            expectEquals(activeZones.size(), sv.visual.zones.size());

            CloudSnapshot cs;
            expect(e.cloudView(1, cs));
            const auto pts = readView(e, points);
            expectEquals(pts.size(), cs.points.size() * 6);
            for (size_t i = 0; i < cs.points.size(); ++i)
                expect(pts[6 * i] == cs.points[i].x && pts[6 * i + 3] == cs.points[i].phase &&
                           pts[6 * i + 5] == cs.points[i].random,
                       "point");
            expect(readView(e, energy) == cs.energy && !cs.energy.empty(), "cloud energy");
            const auto st = readView(e, state);
            expect(st.size() == 2 && st[0] == cs.position && static_cast<int>(st[1]) == cs.activeNotes &&
                       cs.activeNotes == 2,
                   "cloud state");

            // Pulled views advance their sequence only when the content changes.
            std::uint64_t s1 = 0, s2 = 0;
            readView(e, osc1[3], &s1);
            readView(e, osc1[3], &s2);
            expectEquals(static_cast<int>(s1), static_cast<int>(s2));
            zones[1].gainDb = -6.0f;
            e.setZones(0, zones);
            readView(e, osc1[3], &s2);
            expect(s2 > s1, "a changed zone list is a new sequence");
        }

        beginTest("engine: readers on other threads see consistent views while the audio thread renders");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("lfo1_rate", 9.0f);
            expect(e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_pos", 0.3f, 0, 0.0f, 0.0f}));
            const int scope = viewOf(e, "master", "scope");
            const int band = viewOf(e, "params", "osc1_pos.band");
            const int meter = viewOf(e, "master", "meter");
            const int spectrum = viewOf(e, "master", "spectrum_1024");
            const int one = viewOf(e, "params", "osc1_pos.voice@0");
            for (const int v : {scope, band, meter, spectrum, one})
                e.views().subscribe(v, 0.0f);
            e.noteOn(60, 1.0f);
            std::atomic<bool> stop{false};
            std::atomic<int> bad{0};
            std::vector<std::thread> readers;
            for (int t = 0; t < 3; ++t)
                readers.emplace_back(
                    [&]
                    {
                        std::vector<float> buffer(8192);
                        std::uint64_t last = 0;
                        while (!stop.load())
                        {
                            std::uint64_t seq = 0;
                            const int n = e.views().read(band, buffer.data(), 8192, &seq);
                            if (n == 3 && (buffer[0] > buffer[1] || seq < last))
                                bad.fetch_add(1);
                            last = std::max(last, seq);
                            e.views().read(scope, buffer.data(), 8192, &seq);
                            e.views().read(meter, buffer.data(), 8192, &seq);
                            e.views().read(spectrum, buffer.data(), 8192, &seq);
                            e.views().read(one, buffer.data(), 8192, &seq);
                        }
                    });
            run(e, 600);
            stop = true;
            for (auto& r : readers)
                r.join();
            expectEquals(bad.load(), 0);
        }

        beginTest("engine: pulled sample and cloud views and the voice count are safe to read while the control thread "
                  "swaps data");
        {
            Engine e;
            e.prepare(kRate, 256);
            std::string error;
            e.loadSample(0, sineAudio(24000, 440.0), "sine", error);
            e.setParam("osc1_type", 1.0f);
            e.setParam("osc2_type", 2.0f);
            e.setCloud(1, CloudModel::random(60, 5));
            std::vector<Zone> zones(2);
            zones[0].keyHi = 59;
            zones[1].keyLo = 60;
            e.setZones(0, zones);
            const char* sampleIds[] = {"sample_info", "sample_peaks", "sample_slices", "sample_zones"};
            std::vector<int> pulled;
            for (const char* id : sampleIds)
                pulled.push_back(viewOf(e, "osc1", id));
            pulled.push_back(viewOf(e, "osc2", "cloud_points"));
            e.noteOn(60, 1.0f);
            std::atomic<bool> stop{false};
            std::atomic<long> reads{0};
            std::vector<std::thread> readers;
            for (int t = 0; t < 2; ++t)
                readers.emplace_back(
                    [&]
                    {
                        std::vector<float> buffer(16384);
                        while (!stop.load())
                        {
                            for (const int v : pulled)
                                e.views().read(v, buffer.data(), 16384, nullptr);
                            if (e.activeVoiceCount() < 0)
                                reads.fetch_add(1000000);
                            reads.fetch_add(1);
                        }
                    });
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 60; ++i)
            {
                e.loadSample(0, sineAudio(12000 + 100 * i, 330.0 + i), "swap", error);
                e.coreEngine().engine().sampleBank().setSlices(0, {0.25f, 0.5f});
                e.setCloud(1, CloudModel::random(20 + i, static_cast<unsigned>(i + 1)));
                e.noteOn(48 + i % 12, 0.8f);
                e.process(l.data(), r.data(), 256);
                e.coreEngine().engine().sampleBank().collectGarbage();
            }
            stop = true;
            for (auto& r2 : readers)
                r2.join();
            expect(reads.load() > 0 && reads.load() < 1000000);
        }

        beginTest("engine: publishing allocates nothing on the audio thread with every live view subscribed");
        {
            Engine e;
            e.prepare(kRate, 256);
            std::string error;
            e.loadSample(0, sineAudio(48000, 440.0), "sine", error);
            e.setParam("osc1_type", 1.0f);
            e.setParam("osc1_smp_mode", 1.0f);
            e.setParam("osc1_grain_density", 80.0f);
            e.setParam("osc2_type", 2.0f);
            e.setParam("osc3_type", 1.0f);
            e.setCloud(1, CloudModel::random(200, 3));
            e.setParam("lfo1_rate", 5.0f);
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_pos", 0.3f, 0, 0.0f, 0.0f});
            int live = 0;
            for (int v = 0; v < e.views().count(); ++v)
                if (e.views().info(v).live && e.views().subscribe(v, 0.0f) > 0)
                    ++live;
            expect(live > 1000, "subscribed " + String(live));
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 4; ++i)
            {
                e.noteOn(40 + 7 * i, 0.9f);
                e.process(l.data(), r.data(), 256);
            }
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 400; ++i)
                e.process(l.data(), r.data(), 256);
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with all views live");
            std::vector<float> buffer(8192);
            std::uint64_t seq = 0;
            expect(e.views().read(viewOf(e, "master", "scope"), buffer.data(), 8192, &seq) == 8192 && seq > 300);
        }

        beginTest("cost: an unsubscribed view costs nothing, a subscribed set stays far below the render time");
        {
            ViewHub idle;
            idle.prepare(kRate);
            Counter c;
            addCounterView(idle, c, "v", ViewKind::Vector, 64);
            const auto t0 = std::chrono::steady_clock::now();
            const int calls = kSanitised ? 200000 : 20000000;
            for (int i = 0; i < calls; ++i)
                idle.publish(256);
            const double ns =
                std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() / calls;
            logMessage("publish() with nothing subscribed: " + std::to_string(ns) + " ns per render call");
            expect(kSanitised || ns < 20.0, "publish with no subscription is one atomic load, " + String(ns, 2) + " ns");
            expectEquals(static_cast<int>(c.published.load()), 0);

            const auto timeRender = [](Engine& e, int blocks)
            {
                std::vector<float> l(256), r(256);
                for (int i = 0; i < 20; ++i)
                    e.process(l.data(), r.data(), 256);
                double best = 1.0e9;
                for (int rep = 0; rep < 5; ++rep)
                {
                    const auto a = std::chrono::steady_clock::now();
                    for (int i = 0; i < blocks; ++i)
                        e.process(l.data(), r.data(), 256);
                    best = std::min(best,
                                    std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - a).count() /
                                        blocks);
                }
                return best;
            };
            Engine e;
            e.prepare(kRate, 256);
            for (int i = 0; i < 8; ++i)
                e.noteOn(40 + 5 * i, 0.9f);
            const double base = timeRender(e, 300);
            int live = 0;
            for (int v = 0; v < e.views().count(); ++v)
                if (e.views().info(v).live && e.views().subscribe(v, 60.0f) > 0)
                    ++live;
            const double all = timeRender(e, 300);
            logMessage("render " + std::to_string(base) + " us per 256-sample block; with " + std::to_string(live) +
                       " live views at 60 Hz " + std::to_string(all) + " us");
            expect(kSanitised || all < base * 1.5 + 5.0,
                   "views at 60 Hz stay cheap: " + String(all, 2) + " vs " + String(base, 2) + " us");
        }
    }
};

static ViewTests viewTests;
