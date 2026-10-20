// SPDX-License-Identifier: MIT
#include <crucible/module/ConvolutionIr.h>
#include <crucible/Engine.h>
#include <crucible/filters/FilterModel.h>
#include <crucible/EngineCore.h>
#include <crucible/Meter.h>
#include <crucible/Scenes.h>
#include <crucible/Tuning.h>
#include <crucible/Sequencer.h>
#include "AllocCounter.h"
#include "TestSupport.h"

#include <atomic>
#include <limits>
#include <thread>

#include <cstdlib>
#include <new>

// Allocation counter for the "no allocation on the audio thread" test.
namespace crucible::testing
{
thread_local bool gTrackAllocations = false;
thread_local long gAllocations = 0;
} // namespace crucible::testing
using crucible::testing::gAllocations;
using crucible::testing::gTrackAllocations;

void* operator new(std::size_t n)
{
    if (gTrackAllocations)
        ++gAllocations;
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n)
{
    if (gTrackAllocations)
        ++gAllocations;
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete[](void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept
{
    std::free(p);
}

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Hit
{
    long long sample;
    SeqEvent::Type type;
    int a;
    float value;
};

std::vector<Hit> runSequencer(Sequencer& seq, const SeqBank& bank, const SeqSettings& s, double totalSeconds, int block,
                              const std::vector<SeqHostTime>* hostPerBlock = nullptr)
{
    std::vector<Hit> hits;
    std::vector<SeqEvent> ev(Sequencer::maxEvents);
    const long long total = static_cast<long long>(totalSeconds * kRate);
    int blockIndex = 0;
    for (long long pos = 0; pos < total; pos += block, ++blockIndex)
    {
        const int n = static_cast<int>(std::min<long long>(block, total - pos));
        SeqHostTime host;
        if (hostPerBlock != nullptr && blockIndex < static_cast<int>(hostPerBlock->size()))
            host = (*hostPerBlock)[static_cast<size_t>(blockIndex)];
        const int count = seq.process(bank, s, host, n, ev.data(), static_cast<int>(ev.size()));
        for (int i = 0; i < count; ++i)
            hits.push_back({pos + ev[static_cast<size_t>(i)].offset, ev[static_cast<size_t>(i)].type,
                            ev[static_cast<size_t>(i)].a, ev[static_cast<size_t>(i)].value});
    }
    return hits;
}

std::vector<Hit> only(const std::vector<Hit>& h, SeqEvent::Type t)
{
    std::vector<Hit> out;
    for (const auto& x : h)
        if (x.type == t)
            out.push_back(x);
    return out;
}

std::unique_ptr<SeqBank> allStepsOn(int bars = 1, float gate = 0.5f)
{
    auto bp = std::make_unique<SeqBank>();
    SeqBank& b = *bp;
    for (auto& p : b.pattern)
    {
        p.lengthBars = bars;
        for (auto& s : p.steps)
        {
            s.on = true;
            s.note = 60;
            s.gate = gate;
        }
    }
    return bp;
}

std::vector<float> defaultsWith(std::initializer_list<std::pair<const char*, float>> kv)
{
    auto v = ParameterMap::defaultValues();
    for (const auto& [k, x] : kv)
        setValue(v, k, x);
    return v;
}

} // namespace

class SequencerTests : public UnitTest
{
public:
    SequencerTests() : UnitTest("Sequencer", "Engine") {}

    void runTest() override
    {
        beginTest("step scheduling is sample accurate across tempo, rate and block size");
        {
            for (const double bpm : {90.0, 120.0, 133.3, 174.0})
                for (const int rate : {0, 1, 2, 3, 4, 5})
                    for (const int block : {64, 477, 2048})
                    {
                        Sequencer seq;
                        seq.prepare(kRate);
                        SeqSettings s;
                        s.run = true;
                        s.tempo = static_cast<float>(bpm);
                        s.rate = rate;
                        const auto bankHeap = allStepsOn(1);
                        const SeqBank& bank = *bankHeap;
                        const auto hits = only(runSequencer(seq, bank, s, 4.0, block), SeqEvent::Type::NoteOn);
                        const double stepSamples = seqRateBeats(rate) * 60.0 / bpm * kRate;
                        bool ok = hits.size() > 4;
                        String detail;
                        for (size_t k = 0; k < hits.size() && ok; ++k)
                        {
                            const double expected = std::floor(static_cast<double>(k) * stepSamples);
                            ok = std::abs(static_cast<double>(hits[k].sample) - expected) <= 1.0;
                            if (!ok)
                                detail = " step " + String(static_cast<int>(k)) + " got " + String(hits[k].sample) +
                                         " expected " + String(expected);
                        }
                        if (!ok)
                            for (size_t q = 0; q < std::min<size_t>(6, hits.size()); ++q)
                                logMessage("hit " + String(static_cast<int>(q)) + " at " + String(hits[q].sample));
                        expect(ok, "bpm " + String(bpm) + " rate " + String(rate) + " block " + String(block) + detail);
                    }
        }

        beginTest("swing delays odd steps; gates produce note-offs");
        {
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings s;
            s.run = true;
            s.tempo = 120.0f;
            s.swingPercent = 50.0f;
            const auto bankHeap = allStepsOn(1, 0.5f);
            const SeqBank& bank = *bankHeap;
            const auto all = runSequencer(seq, bank, s, 2.0, 256);
            const auto ons = only(all, SeqEvent::Type::NoteOn);
            const auto offs = only(all, SeqEvent::Type::NoteOff);
            // 16th at 120 BPM = 6000 samples; 50% swing delays odd steps by 3000.
            expect(std::abs(ons[0].sample - 0) <= 1 && std::abs(ons[1].sample - (6000 + 3000)) <= 1 &&
                   std::abs(ons[2].sample - 12000) <= 1);
            expect(std::abs(offs[0].sample - 3000) <= 1, "gate off at " + String(offs[0].sample));
            expect(offs.size() >= ons.size() - 1);
        }

        beginTest("ties extend the note and skip the retrigger");
        {
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings s;
            s.run = true;
            CRUCIBLE_HEAP(SeqBank, bank);
            auto& p = bank.pattern[0];
            p.steps[0] = {true, 60, 100, 1, 1.5f, 1.0f, {}};
            p.steps[1] = {true, 60, 100, 1, 0.5f, 1.0f, {}};
            p.steps[2] = {true, 64, 100, 1, 0.5f, 1.0f, {}};
            const auto all = runSequencer(seq, bank, s, 0.5, 512);
            const auto ons = only(all, SeqEvent::Type::NoteOn);
            const auto offs = only(all, SeqEvent::Type::NoteOff);
            expectEquals(static_cast<int>(ons.size()), 2); // step 0 and step 2; step 1 is tied
            expect(ons[0].a == 60 && ons[1].a == 64);
            // The held note ends half a step into step 1.
            expect(std::abs(offs[0].sample - (6000 + 3000)) <= 1, "tie off at " + String(offs[0].sample));
        }

        beginTest("ratchets subdivide the step");
        {
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings s;
            s.run = true;
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0] = {true, 62, 90, 3, 0.5f, 1.0f, {}};
            const auto ons = only(runSequencer(seq, bank, s, 0.2, 128), SeqEvent::Type::NoteOn);
            expectEquals(static_cast<int>(ons.size()), 3);
            expect(std::abs(ons[1].sample - 2000) <= 1 && std::abs(ons[2].sample - 4000) <= 1);
        }

        beginTest("probability is deterministic for a seed and statistically right");
        {
            CRUCIBLE_HEAP(SeqBank, bank, *allStepsOn(16));
            for (auto& st : bank.pattern[0].steps)
                st.probability = 0.5f;
            const auto count = [&](std::uint32_t seed)
            {
                Sequencer seq;
                seq.prepare(kRate);
                seq.setSeed(seed);
                SeqSettings s;
                s.run = true;
                s.tempo = 300.0f;
                s.rate = 2;
                return runSequencer(seq, bank, s, 6.0, 512);
            };
            const auto a = count(11), b = count(11), c = count(99);
            expectEquals(a.size(), b.size());
            bool same = true;
            for (size_t i = 0; i < a.size(); ++i)
                same = same && a[i].sample == b[i].sample && a[i].type == b[i].type;
            expect(same, "same seed, same events");
            bool differs = a.size() != c.size();
            for (size_t i = 0; i < std::min(a.size(), c.size()) && !differs; ++i)
                differs = a[i].sample != c[i].sample;
            expect(differs, "different seed differs");
            const auto played = only(a, SeqEvent::Type::NoteOn).size();
            const auto positions = only(a, SeqEvent::Type::Position).size();
            const double ratio = static_cast<double>(played) / static_cast<double>(positions);
            expect(ratio > 0.4 && ratio < 0.6, "ratio " + String(ratio));

            for (auto& st : bank.pattern[0].steps)
                st.probability = 0.0f;
            expectEquals(static_cast<int>(only(count(1), SeqEvent::Type::NoteOn).size()), 0);
        }

        beginTest("a product exclusion mask is kept next to the sequencer's parameter locks, not instead of them");
        {
            const auto cutoff = static_cast<size_t>(ParameterMap::indexOf("filter_cutoff"));
            const auto res = static_cast<size_t>(ParameterMap::indexOf("filter_res"));
            const auto level = static_cast<size_t>(ParameterMap::indexOf("osc1_level"));
            std::vector<char> product(ParameterMap::params().size(), 0);
            product[res] = 1;
            CRUCIBLE_HEAP(EngineCore, core);
            core.setProductExclusions(product.data());
            core.prepare(kRate);
            auto v = defaultsWith({{"seq_run", 1.0f}, {"seq_tempo", 120.0f}});
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0] = {true, 60, 110, 1, 0.5f, 1.0f, {}};
            bank.pattern[0].steps[0].locks[0] = {static_cast<std::int16_t>(cutoff), 500.0f};
            core.store().replace(bank);
            const char* mask = core.engine().vectorExclusions();
            expect(mask != nullptr && mask != product.data());
            expect(mask[res] == 1 && mask[cutoff] == 0);
            std::vector<float> l(512), r(512);
            core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            expect(mask[cutoff] == 1); // locked by the sequencer
            expect(mask[res] == 1);    // still the product's
            expect(mask[level] == 0);
        }

        beginTest("parameter locks are applied at the step start and restored after");
        {
            const int cutoff = ParameterMap::indexOf("filter_cutoff");
            const int res = ParameterMap::indexOf("filter_res");
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings s;
            s.run = true;
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0].on = true;
            bank.pattern[0].steps[0].locks[0] = {static_cast<std::int16_t>(cutoff), 500.0f};
            bank.pattern[0].steps[0].locks[1] = {static_cast<std::int16_t>(res), 0.9f};
            bank.pattern[0].steps[1].on = true;
            bank.pattern[0].steps[1].locks[0] = {static_cast<std::int16_t>(cutoff), 900.0f};
            const auto all = runSequencer(seq, bank, s, 0.3, 256);
            const auto sets = only(all, SeqEvent::Type::LockSet);
            const auto clears = only(all, SeqEvent::Type::LockClear);
            expectEquals(static_cast<int>(sets.size()), 3);
            expect(sets[0].a == cutoff && sets[0].value == 500.0f && sets[0].sample == 0);
            // Step 1 re-locks the cutoff (no clear) and releases the resonance.
            bool resClearedAtStep1 = false;
            for (const auto& c : clears)
                if (c.a == res && std::abs(c.sample - 6000) <= 1)
                    resClearedAtStep1 = true;
            expect(resClearedAtStep1);
            bool cutoffClearedAtStep2 = false;
            for (const auto& c : clears)
                if (c.a == cutoff && std::abs(c.sample - 12000) <= 1)
                    cutoffClearedAtStep2 = true;
            expect(cutoffClearedAtStep2);
        }

        beginTest("direction, chaining and stop behaviour");
        {
            CRUCIBLE_HEAP(SeqBank, bank, *allStepsOn(1));
            SeqSettings s;
            s.direction = 1;
            expectEquals(Sequencer::positionFor(bank, s, 0, 1).index, 15);
            expectEquals(Sequencer::positionFor(bank, s, 3, 1).index, 12);
            s.direction = 2;
            expectEquals(Sequencer::positionFor(bank, s, 15, 1).index, 15);
            expectEquals(Sequencer::positionFor(bank, s, 16, 1).index, 14);
            expectEquals(Sequencer::positionFor(bank, s, 30, 1).index, 0);
            expectEquals(Sequencer::positionFor(bank, s, 31, 1).index, 1);
            s.direction = 3;
            bool inRange = true;
            for (int k = 0; k < 200; ++k)
                inRange = inRange && Sequencer::positionFor(bank, s, k, 7).index < 16;
            expect(inRange);
            expectEquals(Sequencer::positionFor(bank, s, 5, 7).index, Sequencer::positionFor(bank, s, 5, 7).index);

            bank.pattern[1].lengthBars = 2;
            s = SeqSettings();
            s.chain = 1; // slots 1-2
            expectEquals(Sequencer::positionFor(bank, s, 15, 1).slot, 0);
            expectEquals(Sequencer::positionFor(bank, s, 16, 1).slot, 1);
            expectEquals(Sequencer::positionFor(bank, s, 16 + 31, 1).slot, 1);
            expectEquals(Sequencer::positionFor(bank, s, 16 + 32, 1).slot, 0); // wraps (16 + 32 steps in the chain)

            // Stop flushes sounding notes.
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings run;
            run.run = true;
            CRUCIBLE_HEAP(SeqBank, g, *allStepsOn(1, 4.0f));
            std::vector<SeqEvent> ev(Sequencer::maxEvents);
            seq.process(g, run, {}, 512, ev.data(), static_cast<int>(ev.size()));
            run.run = false;
            const int n = seq.process(g, run, {}, 512, ev.data(), static_cast<int>(ev.size()));
            expect(n >= 1 && ev[0].type == SeqEvent::Type::NoteOff);
            expect(!seq.isActive());
        }

        beginTest("host sync follows ppq, stops with the transport and flushes on a jump");
        {
            Sequencer seq;
            seq.prepare(kRate);
            SeqSettings s;
            s.run = true;
            s.hostSync = true;
            const auto bankHeap = allStepsOn(1, 0.5f);
            const SeqBank& bank = *bankHeap;
            std::vector<SeqHostTime> hosts;
            const double bps = 128.0 / 60.0 / kRate;
            for (int b = 0; b < 100; ++b)
            {
                SeqHostTime h;
                h.hasPpq = true;
                h.playing = b < 60;
                h.ppq = 8.0 + b * 512 * bps;
                h.hasBpm = true;
                h.bpm = 128.0;
                hosts.push_back(h);
            }
            const auto all = runSequencer(seq, bank, s, 100 * 512 / kRate, 512, &hosts);
            const auto ons = only(all, SeqEvent::Type::NoteOn);
            // The first trigger is at ppq 8.0 = the first sample; spacing is a 16th at 128 BPM.
            const double step = 0.25 * 60.0 / 128.0 * kRate;
            expect(std::abs(ons[0].sample - 0) <= 1 && std::abs(ons[3].sample - std::floor(3 * step)) <= 1);
            expect(ons.back().sample < 60 * 512 + 1, "no triggers after the transport stopped");
        }

        beginTest("patterns serialise and the store publishes edits safely");
        {
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[3].lengthBars = 4;
            bank.pattern[3].steps[17] = {true, 71, 33, 2, 1.75f, 0.4f, {}};
            bank.pattern[3].steps[17].locks[2] = {static_cast<std::int16_t>(ParameterMap::indexOf("filter_cutoff")),
                                                  1234.5f};
            bank.pattern[11].steps[255] = {true, 12, 127, 4, 0.1f, 1.0f, {}};
            const auto records = seqBankSerialise(bank);
            expectEquals(static_cast<int>(records.size()), kSeqSlots);
            CRUCIBLE_HEAP(SeqBank, restored);
            seqBankDeserialise(records, restored);
            expectEquals(restored.pattern[3].lengthBars, 4);
            const auto& r = restored.pattern[3].steps[17];
            expect(r.on && r.note == 71 && r.velocity == 33 && r.ratchet == 2);
            expectWithinAbsoluteError(r.gate, 1.75f, 1.0e-3f);
            expectWithinAbsoluteError(r.probability, 0.4f, 1.0e-3f);
            expect(r.locks[0].param == ParameterMap::indexOf("filter_cutoff") &&
                   std::abs(r.locks[0].value - 1234.5f) < 0.01f);
            expect(restored.pattern[11].steps[255].note == 12 && restored.pattern[11].steps[255].ratchet == 4);
            expect(!restored.pattern[0].steps[0].on);

            SeqStore store;
            store.edit([](SeqBank& b) { b.pattern[2].steps[5].on = true; });
            expect(store.snapshotPattern(2).steps[5].on);
            const SeqBank* held = store.acquire();
            store.edit([](SeqBank& b) { b.pattern[2].steps[6].on = true; });
            expect(held->pattern[2].steps[5].on && !held->pattern[2].steps[6].on); // pinned bank is untouched
            store.release();
            store.collectGarbage();
            expect(store.snapshotPattern(2).steps[6].on);
            store.replace(bank);
            expectEquals(store.snapshotPattern(3).lengthBars, 4);
        }

        beginTest("concurrent edits while the audio side reads never tear");
        {
            SeqStore store;
            std::atomic<bool> stop{false};
            std::atomic<int> bad{0};
            std::thread reader(
                [&]
                {
                    while (!stop.load())
                    {
                        const SeqBank* b = store.acquire();
                        // The writer keeps steps 0 and 1 equal; a torn read would break that.
                        if (b->pattern[0].steps[0].note != b->pattern[0].steps[1].note)
                            ++bad;
                        store.release();
                    }
                });
            for (int i = 0; i < 2000; ++i)
                store.edit(
                    [i](SeqBank& b)
                    {
                        b.pattern[0].steps[0].note = static_cast<std::uint8_t>(i % 100);
                        b.pattern[0].steps[1].note = static_cast<std::uint8_t>(i % 100);
                    });
            stop = true;
            reader.join();
            expectEquals(bad.load(), 0);
        }
    }
};

class CoreTests : public UnitTest
{
public:
    CoreTests() : UnitTest("Core", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("sequencer drives the voice allocator and produces audio; locks change the sound");
        {
            const auto render = [&](bool lock)
            {
                CRUCIBLE_HEAP(EngineCore, core);
                core.prepare(kRate);
                auto v = defaultsWith({{"seq_run", 1.0f}, {"seq_tempo", 120.0f}, {"amp_release", 0.05f}});
                CRUCIBLE_HEAP(SeqBank, bank);
                bank.pattern[0].steps[0] = {true, 60, 110, 1, 0.5f, 1.0f, {}};
                if (lock)
                    bank.pattern[0].steps[0].locks[0] = {static_cast<std::int16_t>(ParameterMap::indexOf("osc1_level")),
                                                         0.0f};
                core.store().replace(bank);
                std::vector<float> l(512), r(512);
                float peak = 0.0f;
                for (int b = 0; b < 12; ++b)
                {
                    core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
                    if (b >= 4) // skip the 30 ms parameter smoothing that follows the lock
                        for (const float x : l)
                            peak = std::max(peak, std::abs(x));
                }
                return peak;
            };
            expect(render(false) > 0.01f);
            expect(render(true) < 0.001f);
        }

        beginTest("external events (OSC keyboard), MIDI, bend, wheel and sustain reach the engine");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            std::vector<float> l(512), r(512);
            ExternalEvent on;
            on.type = ExternalEvent::Type::NoteOn;
            on.note = 64;
            on.value = 0.8f;
            expect(core.pushExternal(on));
            core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            expectEquals(core.engine().activeVoiceCount(), 1);
            MidiInput midi[2] = {{100, 0x90, 67, 100}, {300, 0x80, 64, 0}};
            core.process(l.data(), r.data(), 512, v.data(), midi, 2, {});
            expectEquals(core.engine().activeVoiceCount(), 2); // 64 releasing, 67 sounding
            ExternalEvent off;
            off.type = ExternalEvent::Type::NoteOff;
            off.note = 67;
            core.pushExternal(off);
            for (int i = 0; i < 200; ++i)
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            expectEquals(core.engine().activeVoiceCount(), 0);
            for (const auto type :
                 {ExternalEvent::Type::PitchBend, ExternalEvent::Type::ModWheel, ExternalEvent::Type::Sustain})
            {
                ExternalEvent e;
                e.type = type;
                e.value = 0.5f;
                expect(core.pushExternal(e));
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            }
        }

        beginTest("step record captures incoming notes");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = defaultsWith({{"seq_rec", 1.0f}});
            std::vector<float> l(256), r(256);
            MidiInput midi[1] = {{10, 0x90, 55, 90}};
            core.process(l.data(), r.data(), 256, v.data(), midi, 1, {});
            RecordEvent rec;
            expect(core.popRecord(rec));
            expect(rec.note == 55 && rec.velocity == 90 && rec.step == -1);
            expect(!core.popRecord(rec));
        }

        beginTest("no allocation on the audio thread with sequencer, 16 voices and every effect running");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = defaultsWith(
                {{"seq_run", 1.0f},       {"seq_tempo", 200.0f},  {"osc1_uni_voices", 8.0f}, {"osc1_warp", 2.0f},
                 {"osc1_warp_a", 0.5f},   {"filter2_on", 1.0f},   {"fx_saturator_on", 1.0f}, {"fx_ensemble_on", 1.0f},
                 {"fx_flanger2_on", 1.0f}, {"fx_phaser2_on", 1.0f}, {"fx_delay_pro_on", 1.0f}, {"fx_ra_on", 1.0f},
                 {"fx_cc_on", 1.0f},    {"fx_ep_on", 1.0f},     {"fx_limit_on", 1.0f},     {"fx_crush_on", 1.0f},
                 {"fx_width_on", 1.0f},   {"mod1_src", 1.0f},     {"mod1_dst", 7.0f},        {"mod1_amt", 0.5f}});
            CRUCIBLE_HEAP(SeqBank, bank, *allStepsOn(1, 0.9f));
            for (auto& s : bank.pattern[0].steps)
                s.locks[0] = {static_cast<std::int16_t>(ParameterMap::indexOf("filter_cutoff")), 2000.0f};
            core.store().replace(bank);
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 20; ++i)
                core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {}); // warm-up
            MidiInput midi[3] = {{5, 0x90, 40, 100}, {50, 0x90, 52, 100}, {120, 0x80, 40, 0}};
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 400; ++i)
            {
                ExternalEvent e;
                e.type = ExternalEvent::Type::ModWheel;
                e.value = 0.3f;
                core.pushExternal(e);
                core.process(l.data(), r.data(), 256, v.data(), i % 7 == 0 ? midi : nullptr, i % 7 == 0 ? 3 : 0, {});
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations on the audio thread");
        }

        beginTest("no allocation while the saturation saturator models run and are switched");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            setValue(v, "fx_saturator_on", 1.0f);
            setValue(v, "fx_saturator_drive", 32.0f);
            std::vector<float> l(256), r(256);
            MidiInput midi[1] = {{0, 0x90, 48, 100}};
            core.process(l.data(), r.data(), 256, v.data(), midi, 1, {});
            for (int mode = 0; mode < 16; ++mode)
            {
                setValue(v, "fx_saturator_model", static_cast<float>(mode));
                core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            }
            // The id is built outside the counted window: it is longer than MSVC's small-string buffer.
            const std::string saturatorModel = "fx_saturator_model";
            gAllocations = 0;
            gTrackAllocations = true;
            for (int mode = 0; mode < 16; ++mode)
            {
                setValue(v, saturatorModel, static_cast<float>(mode));
                for (int i = 0; i < 20; ++i)
                    core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the saturation modes");
        }

        beginTest(
            "no allocation while the nonlinear ladder filters run and are switched (voices, filter effect, interface)");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_res", 0.85f);
            setValue(v, "filter_drive", 0.5f);
            setValue(v, "fx_filter_on", 1.0f);
            setValue(v, "fx_filter_res", 0.7f);
            setValue(v, "fx_filter_lfo_depth", 2.0f);
            std::vector<float> l(256), r(256);
            MidiInput midi[2] = {{0, 0x90, 48, 100}, {0, 0x90, 55, 100}};
            core.process(l.data(), r.data(), 256, v.data(), midi, 2, {});
            // Parameter ids are built outside the counted window: a std::string temporary of an id longer than the
            // small-string buffer (15 chars on MSVC's STL, 22 on libc++) would be counted although it is test code.
            const std::string filterType = "filter_type", filterCharacter = "filter_character", filterHp = "filter_hp",
                              fxFilterModel = "fx_filter_model";
            gAllocations = 0;
            gTrackAllocations = true;
            for (int model = 17; model < 25; ++model)
            {
                setValue(v, filterType, static_cast<float>(model));
                setValue(v, filterCharacter, static_cast<float>(model % 3) * 0.5f);
                setValue(v, filterHp, static_cast<float>(model % 4));
                setValue(v, fxFilterModel, static_cast<float>(model));
                for (int i = 0; i < 30; ++i)
                    core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the nonlinear filter models");

            // The interface itself, mono and four lanes, with and without a cutoff modulation array.
            for (auto make : {makeTransistorLadder, makeOtaCascade, makeDiodeLadder, makeSallenKeyLowpass,
                              makeSallenKeyHighpass, makeSvfOta, makeOta4Pole, makePassiveHp})
            {
                auto mono = make();
                mono->prepare(kRate, Quality::Standard, 1, 256);
                auto quad = make();
                quad->prepare(kRate, Quality::High, 4, 256);
                std::vector<float> a(256, 0.1f), b(256, 0.1f), c(256, 0.1f), d(256, 0.1f), mod(256, 0.5f);
                float* lanes[4] = {a.data(), b.data(), c.data(), d.data()};
                const float* mods[4] = {mod.data(), nullptr, mod.data(), nullptr};
                gAllocations = 0;
                gTrackAllocations = true;
                for (int i = 0; i < 20; ++i)
                {
                    mono->setControls(500.0f + static_cast<float>(i), 0.9f, 0.3f, 0.2f, 0.5f);
                    mono->process(a.data(), 256, i % 2 == 0 ? mod.data() : nullptr);
                    quad->setLaneControls(i % 4, 800.0f, 0.7f, 0.4f, 0.1f, 0.2f);
                    quad->processQuad(lanes, 256, mods);
                }
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "allocations in AnalogFilter::process / processQuad");
            }
        }

        beginTest("no allocation with every second-generation effect running (user IR swapped mid-run)");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            std::vector<std::pair<const char*, float>> on;
            auto v = ParameterMap::defaultValues();
            for (const char* id : {"fx_ep_on", "fx_filter_on", "fx_ensemble_on", "fx_multiband_on", "fx_g2_on",
                                   "fx_trance_on", "fx_tremolo_on", "fx_ring_on", "fx_cx_on", "fx_saturator_on"})
                setValue(v, id, 1.0f);
            setValue(v, "fx_filter_lfo_depth", 2.0f);
            setValue(v, "fx_saturator_filter_mode", 1.0f);
            setValue(v, "arp_on", 1.0f);
            setValue(v, "arp_grid", 1.0f);
            setValue(v, "arp_ratchet", 3.0f);
            core.arpPattern().edit([](ArpPattern& p) { p.steps[2].tie = true; });
            std::vector<float> ir(4800, 0.0f);
            ir[10] = 1.0f;
            ImpulseResponse irData;
            irData.sampleRate = kRate;
            irData.channels = {ir};
            core.engine().effects().findCapability<ConvolutionIr>()->setImpulseResponse(irData, "a");
            std::vector<float> l(256), r(256);
            MidiInput midi[2] = {{0, 0x90, 48, 100}, {0, 0x90, 55, 100}};
            core.process(l.data(), r.data(), 256, v.data(), midi, 2, {});
            for (int i = 0; i < 20; ++i)
                core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 200; ++i)
                core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the new effects");
            // Publishing a new IR happens on this (control) thread; the audio side then picks it up allocation free.
            irData.channels = {std::vector<float>(ir.begin(), ir.begin() + 2400)};
            core.engine().effects().findCapability<ConvolutionIr>()->setImpulseResponse(irData, "b");
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 20; ++i)
                core.process(l.data(), r.data(), 256, v.data(), nullptr, 0, {});
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations after an IR swap");
            float peak = 0.0f;
            for (const float x : l)
                peak = std::max(peak, std::abs(x));
            expect(std::isfinite(peak));
        }

        beginTest("no allocation with clips, voice playheads, routes and the motion recorder running");
        {
            Engine e;
            e.prepare(kRate, 128);
            ClipBank bank;
            for (const auto trigger :
                 {ClipTrigger::Voice, ClipTrigger::GlobalRetrigger, ClipTrigger::GlobalFree, ClipTrigger::Transport})
            {
                Clip c;
                c.trigger = trigger;
                c.mode = ClipPlayMode::PingPong;
                c.sustainLoop = true;
                c.loopStart = 1.0;
                ClipLane curve;
                for (int i = 0; i < 40; ++i)
                    curve.points.push_back({i * 0.1, (i % 5) * 0.2f, 0.2f, SegmentShape::SCurve});
                curve.humanise = 0.5f;
                curve.randomise = 0.2f;
                curve.smoothingMs = 5.0f;
                ClipLane steps;
                steps.kind = ClipLaneKind::Steps;
                steps.steps = {0.1f, 0.9f, 0.4f, 0.7f};
                steps.stepGlide = 0.5f;
                c.lanes = {curve, steps};
                bank.clips.push_back(c);
            }
            e.clips().replace(bank);
            for (int clip = 0; clip < 4; ++clip)
            {
                e.modRoutes().add({clipLaneSource(clip, 0), "osc1_level", 0.3f, 2, 0.1f, 20.0f});
                e.modRoutes().add({clipLaneSource(clip, 1), "filter_cutoff", 0.3f, 0, 0.0f, 0.0f});
                e.modRoutes().add({clipLaneSource(clip, 1), "fx_ra_size", 0.3f, 2, 0.0f, 10.0f});
            }
            e.setParam("fx_ra_on", 1.0f);
            e.setParam("mpe_on", 1.0f);
            e.setParam("voice_glide", 0.2f);
            e.setParam("voice_glide_type", 1.0f);
            e.modRoutes().add({kSourceMpePressure, "osc1_level", 0.4f, 2, 0.0f, 10.0f});
            e.modRoutes().add({kSourceMpeSlide, "filter_cutoff", 0.4f});
            e.modRoutes().add({kSourceMpeBend, "fx_ra_size", 0.4f, 0, 0.0f, 5.0f});
            Tuning quarter;
            std::string scl = "q\n24\n";
            for (int k = 1; k <= 24; ++k)
                scl += std::to_string(50.0 * k) + "\n";
            std::string tuneError;
            quarter.loadScl(scl, tuneError);
            e.tuning().set(quarter);
            e.attachKeyBus(KeyBus::createPrivate());
            e.setParam("key_mode", 1.0f);
            e.setParam("key_scale", 2.0f);
            e.setParam("key_sync", 2.0f);
            MotionSettings ms;
            ms.clip = 3;
            ms.lane = 0;
            ms.param = "fx_ra_mix";
            ms.overdub = true;
            std::vector<float> l(128), r(128);
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 140.0;
            host.hasPpq = true;
            host.playing = true;
            MidiInput midi[2] = {{3, 0x90, 40, 100}, {60, 0x90, 52, 100}};
            for (int i = 0; i < 40; ++i)
            {
                host.ppq = i * 0.05;
                e.process(l.data(), r.data(), 128, i == 0 ? midi : nullptr, i == 0 ? 2 : 0, &host);
            }
            e.recordMotion(ms);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 600; ++i)
            {
                host.ppq = 2.0 + i * 0.05;
                if (i % 40 == 7)
                    e.setParam("key_root", static_cast<float>(i % 12));
                if (i % 50 == 10)
                    e.setParamNormalised(e.model().indexOf("fx_ra_mix"), (i % 100) < 50 ? 0.9f : 0.1f);
                const auto ch = static_cast<std::uint8_t>(i % 4);
                MidiInput m[4] = {{static_cast<int>(i % 7), static_cast<std::uint8_t>((i % 3 == 0 ? 0x80 : 0x90) | ch),
                                   static_cast<std::uint8_t>(40 + i % 12), 90},
                                  {10, static_cast<std::uint8_t>(0xe0 | ch), static_cast<std::uint8_t>(i % 128), 70},
                                  {20, static_cast<std::uint8_t>(0xd0 | ch), static_cast<std::uint8_t>(i % 100), 0},
                                  {30, static_cast<std::uint8_t>(0xb0 | ch), 74, static_cast<std::uint8_t>(i % 90)}};
                e.process(l.data(), r.data(), 128, i % 5 == 0 ? m : nullptr, i % 5 == 0 ? 4 : 0, &host);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with clips and the recorder");
            e.motion().stop();
            expect(e.motion().drain() >= 0);
            float peak = 0.0f;
            for (const float x : l)
                peak = std::max(peak, std::abs(x));
            expect(std::isfinite(peak));
        }

        beginTest("no allocation with cloud oscillators, moving spectral drawbars and a route on a band");
        {
            Engine e;
            e.prepare(kRate, 128);
            e.setParam("osc1_type", 2.0f);
            e.setParam("osc2_type", 2.0f);
            e.setParam("osc2_level", 0.5f);
            e.modRoutes().add({static_cast<int>(ModSource::Velocity), "osc1_cloud_band4", -0.5f});
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc2_cloud_band2", 0.5f});
            std::vector<float> l(128), r(128);
            MidiInput midi[2] = {{3, 0x90, 40, 100}, {60, 0x90, 52, 100}};
            e.process(l.data(), r.data(), 128, midi, 2);
            // Look the ids up outside the counted window (a 16-char std::string temporary allocates on MSVC's STL).
            const int band1 = e.model().indexOf("osc1_cloud_band1"), band7 = e.model().indexOf("osc2_cloud_band7");
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 300; ++i)
            {
                e.setParamNormalised(band1, (i % 40) / 40.0f);
                e.setParamNormalised(band7, (i % 30) / 30.0f);
                e.process(l.data(), r.data(), 128, nullptr, 0);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with drawbars");
        }

        beginTest("no allocation with the vector field (orbit, drift, scenes republished) and clip transport running");
        {
            Engine e;
            e.prepare(kRate, 128);
            for (int k = 0; k < 4; ++k)
            {
                e.setParam("filter_cutoff", 300.0f + 700.0f * static_cast<float>(k));
                e.setParam("osc1_table", static_cast<float>(k));
                e.setParam("fx_ra_mix", 0.1f * static_cast<float>(k));
                e.storeScene(k);
            }
            e.setParam("vector_on", 1.0f);
            e.setParam("vector_orbit_depth", 0.3f);
            e.setParam("vector_drift", 0.2f);
            e.setParam("vector_smooth", 0.05f);
            ClipBank bank;
            Clip clip;
            clip.trigger = ClipTrigger::GlobalFree;
            ClipLane lane;
            lane.points = {{0.0, 0.0f}, {2.0, 1.0f}, {4.0, 0.0f}};
            clip.lanes = {lane};
            bank.clips.push_back(clip);
            e.clips().replace(bank);
            e.modRoutes().add({clipLaneSource(0, 0), "vector_x", 0.5f});
            e.setParam("fx_ra_on", 1.0f);
            std::vector<float> l(128), r(128);
            MidiInput midi[2] = {{3, 0x90, 40, 100}, {60, 0x90, 52, 100}};
            e.process(l.data(), r.data(), 128, midi, 2);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 400; ++i)
            {
                if (i % 50 == 7)
                    e.stopClip(0);
                if (i % 50 == 17)
                    e.startClip(0);
                e.setParam("vector_y", (i % 60) / 60.0f);
                e.process(l.data(), r.data(), 128, i % 90 == 0 ? midi : nullptr, i % 90 == 0 ? 2 : 0);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the vector field");
        }

        beginTest("no allocation in the Engine facade's process(), including block splitting and CC mapping");
        {
            Engine e;
            e.prepare(kRate, 128);
            e.mapCC(74, ParameterMap::indexOf("filter_cutoff"));
            e.modRoutes().add({kSourceCC + 71, "filter_res", 0.4f, 0, 0.0f, 20.0f});
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "fx_ra_size", 0.3f});
            e.modRoutes().add({static_cast<int>(ModSource::AmpEnv), "osc1_pos", 0.5f});
            e.setParam("fx_ra_on", 1.0f);
            std::vector<float> l(1000), r(1000);
            MidiInput midi[3] = {{5, 0x90, 40, 100}, {300, 0xb0, 74, 30}, {900, 0x80, 40, 0}};
            e.process(l.data(), r.data(), 1000, midi, 3); // warm-up
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 100; ++i)
            {
                e.noteOn(50 + i % 12, 0.8f);
                e.process(l.data(), r.data(), 1000, midi, 3);
                e.noteOff(50 + i % 12);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations in Engine::process");
        }

        beginTest(
            "no allocation with sample, granular, cloud, noise and sub oscillators (sample and cloud swapped mid-run)");
        {
            Engine e;
            e.prepare(kRate, 128);
            AudioData audio;
            audio.sampleRate = 44100.0;
            audio.channels.assign(2, std::vector<float>(44100, 0.0f));
            for (int i = 0; i < 44100; ++i)
                audio.channels[0][static_cast<size_t>(i)] = audio.channels[1][static_cast<size_t>(i)] =
                    0.3f * std::sin(0.05f * static_cast<float>(i)) + 0.2f * std::sin(0.31f * static_cast<float>(i));
            std::string error;
            expect(e.loadSample(0, audio, "a", error));
            expect(e.loadSample(1, audio, "a", error));
            e.setCloud(2, CloudModel::random(40, 7));
            for (const auto& [id, v] : std::vector<std::pair<const char*, float>>{{"osc1_type", 1.0f},
                                                                                  {"osc1_level", 0.5f},
                                                                                  {"osc1_smp_loop", 1.0f},
                                                                                  {"osc1_smp_xfade", 0.3f},
                                                                                  {"osc2_type", 1.0f},
                                                                                  {"osc2_level", 0.5f},
                                                                                  {"osc2_smp_mode", 1.0f},
                                                                                  {"osc2_grain_density", 120.0f},
                                                                                  {"osc3_type", 2.0f},
                                                                                  {"osc3_level", 0.4f},
                                                                                  {"sub_level", 0.2f},
                                                                                  {"osc3_cloud_pos", 0.4f},
                                                                                  {"filter2_on", 1.0f},
                                                                                  {"filter_route", 1.0f}})
                e.setParam(id, v);
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc2_grain_pos", 0.5f});
            e.modRoutes().add({static_cast<int>(ModSource::Velocity), "osc3_cloud_pos", 0.5f});
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 10; ++i)
            {
                e.noteOn(48 + i, 0.8f);
                e.process(l.data(), r.data(), 256);
            }
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 200; ++i)
            {
                if (i % 5 == 0)
                    e.noteOn(40 + i % 24, 0.7f);
                e.process(l.data(), r.data(), 256);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the oscillator modules");
            // Publishing new data is a control thread job; the audio thread then picks it up allocation free.
            expect(e.loadSample(0, audio, "b", error));
            e.setCloud(2, CloudModel::random(60, 9));
            e.setParam("osc1_type", 3.0f);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 40; ++i)
                e.process(l.data(), r.data(), 256);
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations after publishing new sample data");
        }

        beginTest("no allocation with multisample zones (layers, crossfades, round robin, zones republished mid-run)");
        {
            Engine e;
            e.prepare(kRate, 128);
            AudioData audio;
            audio.sampleRate = 44100.0;
            audio.channels.assign(2, std::vector<float>(22050, 0.0f));
            for (int i = 0; i < 22050; ++i)
                audio.channels[0][static_cast<size_t>(i)] = audio.channels[1][static_cast<size_t>(i)] =
                    0.3f * std::sin(0.05f * static_cast<float>(i));
            std::string error;
            expect(e.loadSample(0, audio, "own", error));
            expect(e.addZoneSample(0, audio, "two", error) == 1);
            expect(e.addZoneSample(0, audio, "three", error) == 2);
            Zone a, b, c;
            a.keyHi = 63;
            a.sampleId = 0;
            b.keyLo = 56;
            b.sampleId = 1;
            b.loopMode = 1;
            c.sampleId = 2;
            c.group = 1;
            expect(e.setZones(0, {a, b, c}));
            for (const auto& [id, v] : std::vector<std::pair<const char*, float>>{{"osc1_type", 1.0f},
                                                                                  {"osc1_level", 0.5f},
                                                                                  {"osc1_smp_zone_mode", 1.0f},
                                                                                  {"osc1_smp_zone_xfade_key", 6.0f},
                                                                                  {"osc1_smp_zone_xfade_vel", 0.2f},
                                                                                  {"osc1_smp_zone_rr", 2.0f},
                                                                                  {"osc1_smp_uni_voices", 3.0f}})
                e.setParam(id, v);
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 6; ++i)
            {
                e.noteOn(50 + i * 3, 0.6f);
                e.process(l.data(), r.data(), 256);
            }
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 200; ++i)
            {
                if (i % 5 == 0)
                    e.noteOn(36 + i % 40, 0.2f + 0.005f * static_cast<float>(i));
                e.process(l.data(), r.data(), 256);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with zones");
            expect(e.setZones(0, {b, c}));
            e.setParam("osc1_smp_mode", 1.0f);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 40; ++i)
            {
                if (i % 4 == 0)
                    e.noteOn(48 + i, 0.7f);
                e.process(l.data(), r.data(), 256);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations after republishing zones");
        }
    }
};

class SceneTests : public UnitTest
{
public:
    SceneTests() : UnitTest("Scenes", "Engine") {}

    void runTest() override
    {
        const auto& defs = ParameterMap::params();
        const auto idx = [](const char* id) { return static_cast<size_t>(ParameterMap::indexOf(id)); };

        beginTest("store and recall");
        {
            SceneBank bank;
            auto a = ParameterMap::defaultValues();
            a[idx("filter_cutoff")] = 1234.0f;
            a[idx("osc1_table")] = 5.0f;
            expect(!bank.isFilled(3));
            bank.store(3, a);
            expect(bank.isFilled(3));
            auto out = ParameterMap::defaultValues();
            expect(bank.recall(3, out));
            expectEquals(out[idx("filter_cutoff")], 1234.0f);
            expectEquals(out[idx("osc1_table")], 5.0f);
            expect(!bank.recall(4, out));
            bank.clear(3);
            expect(!bank.isFilled(3));
        }

        beginTest("morph interpolates continuous parameters and snaps discrete ones at 0.5");
        {
            SceneBank bank;
            auto a = ParameterMap::defaultValues();
            auto b = a;
            a[idx("filter_cutoff")] = 100.0f;
            b[idx("filter_cutoff")] = 10000.0f;
            a[idx("master_gain")] = -30.0f;
            b[idx("master_gain")] = 0.0f;
            a[idx("osc1_table")] = 1.0f;
            b[idx("osc1_table")] = 9.0f;
            a[idx("filter_on")] = 0.0f;
            b[idx("filter_on")] = 1.0f;
            a[idx("osc1_uni_voices")] = 2.0f;
            b[idx("osc1_uni_voices")] = 8.0f;
            bank.store(0, a);
            bank.store(1, b);
            auto out = ParameterMap::defaultValues();
            expect(bank.morph(0, 1, 0.0f, out));
            expectWithinAbsoluteError(out[idx("filter_cutoff")], 100.0f, 0.01f);
            expect(bank.morph(0, 1, 1.0f, out));
            expectWithinAbsoluteError(out[idx("filter_cutoff")], 10000.0f, 0.5f);
            expect(bank.morph(0, 1, 0.5f, out));
            // Skewed range: the midpoint is taken in normalised space, so it is not the arithmetic mean.
            const float mid = out[idx("filter_cutoff")];
            expect(mid > 100.0f && mid < 5050.0f, "mid cutoff " + String(mid));
            const auto& d = defs[idx("filter_cutoff")];
            const float expected =
                d.min + (d.max - d.min) * std::pow(0.5f * (std::pow((100.0f - d.min) / (d.max - d.min), d.skew) +
                                                           std::pow((10000.0f - d.min) / (d.max - d.min), d.skew)),
                                                   1.0f / d.skew);
            expectWithinAbsoluteError(mid, expected, 0.5f);
            // Discrete: below 0.5 keeps A, from 0.5 takes B.
            bank.morph(0, 1, 0.49f, out);
            expect(out[idx("osc1_table")] == 1.0f && out[idx("filter_on")] == 0.0f && out[idx("osc1_uni_voices")] == 2.0f);
            bank.morph(0, 1, 0.5f, out);
            expect(out[idx("osc1_table")] == 9.0f && out[idx("filter_on")] == 1.0f && out[idx("osc1_uni_voices")] == 8.0f);
            // Monotonic in t.
            float previous = -1.0f;
            bool monotonic = true;
            for (int i = 0; i <= 20; ++i)
            {
                bank.morph(0, 1, static_cast<float>(i) / 20.0f, out);
                monotonic = monotonic && out[idx("filter_cutoff")] >= previous;
                previous = out[idx("filter_cutoff")];
            }
            expect(monotonic);
            // An empty scene cannot be a morph target.
            expect(!bank.morph(0, 5, 0.5f, out));
        }

        beginTest("performance controls are excluded from scenes");
        {
            SceneBank bank;
            auto a = ParameterMap::defaultValues();
            a[idx("strip_l")] = 0.9f;
            a[idx("seq_run")] = 1.0f;
            a[idx("scene_morph")] = 0.7f;
            bank.store(0, a);
            auto out = ParameterMap::defaultValues();
            out[idx("strip_l")] = 0.1f;
            out[idx("seq_run")] = 0.0f;
            out[idx("scene_morph")] = 0.2f;
            bank.recall(0, out);
            expectEquals(out[idx("strip_l")], 0.1f);
            expectEquals(out[idx("seq_run")], 0.0f);
            expectEquals(out[idx("scene_morph")], 0.2f);
        }

        beginTest("scenes round-trip through their serialised form and survive parameter additions");
        {
            SceneBank bank;
            auto a = ParameterMap::defaultValues();
            a[idx("filter_cutoff")] = 777.0f;
            a[idx("fx_delay_pro_time")] = 321.0f;
            bank.store(7, a);
            const auto records = bank.serialise();
            expectEquals(static_cast<int>(records.size()), 1);
            SceneBank restored;
            restored.deserialise(records);
            expect(restored.isFilled(7) && !restored.isFilled(6));
            auto out = ParameterMap::defaultValues();
            restored.recall(7, out);
            expectWithinAbsoluteError(out[idx("filter_cutoff")], 777.0f, 0.01f);
            expectWithinAbsoluteError(out[idx("fx_delay_pro_time")], 321.0f, 0.01f);

            // An old state that lacks newer ids still loads, with defaults for the missing ones.
            SceneBank fromOld;
            fromOld.deserialise({{0, "filter_cutoff=555;does_not_exist=1"}});
            auto o2 = ParameterMap::defaultValues();
            fromOld.recall(0, o2);
            expectWithinAbsoluteError(o2[idx("filter_cutoff")], 555.0f, 0.01f);
            expectEquals(o2[idx("osc1_level")], defs[idx("osc1_level")].defaultValue);
        }

        beginTest("scene morph is usable as a mod source with a morph parameter");
        {
            expect(ParameterMap::indexOf("scene_morph") >= 0);
            const auto& names = ParameterMap::modSourceNames();
            expect(std::find(names.begin(), names.end(), "Scene Morph") != names.end());
        }
    }
};

class MeterTests : public UnitTest
{
public:
    MeterTests() : UnitTest("Meter", "Engine") {}

    void runTest() override
    {
        beginTest("silence reads -120 dB and nothing is NaN");
        {
            OutputMeter m;
            m.prepare(kRate);
            std::vector<float> z(480, 0.0f);
            m.process(z.data(), z.data(), 480);
            auto s = m.snapshot();
            expect(s.peakLeftDb == -120.0f && s.peakRightDb == -120.0f && !s.clip);
            std::vector<float> bad(480, std::nanf(""));
            m.process(bad.data(), bad.data(), 480);
            s = m.snapshot();
            expect(std::isfinite(s.peakLeftDb) && s.peakLeftDb >= -120.0f && s.peakLeftDb <= 12.0f);
            std::vector<float> inf(480, std::numeric_limits<float>::infinity());
            m.process(inf.data(), inf.data(), 480);
            s = m.snapshot();
            expect(std::isfinite(s.peakLeftDb) && s.peakLeftDb <= 12.0f && s.clip);
        }

        beginTest("instant attack and about 300 ms release");
        {
            OutputMeter m;
            m.prepare(kRate);
            std::vector<float> loud(64, 0.5f), quiet(64, 0.0f);
            m.process(loud.data(), loud.data(), 64);
            expectWithinAbsoluteError(m.snapshot().peakLeftDb, -6.02f, 0.05f);
            // After 300 ms the held level has fallen by one time constant (8.7 dB).
            for (int i = 0; i < 225; ++i)
                m.process(quiet.data(), quiet.data(), 64);
            expectWithinAbsoluteError(m.snapshot().peakLeftDb, -6.02f - 8.69f, 0.6f);
            // Channels are independent.
            std::vector<float> l(64, 0.25f);
            m.prepare(kRate);
            m.process(l.data(), quiet.data(), 64);
            const auto s = m.snapshot();
            expect(s.peakLeftDb > -13.0f && s.peakRightDb == -120.0f);
        }

        beginTest("clip latches on |x| >= 1.0 and holds for 2 s");
        {
            OutputMeter m;
            m.prepare(kRate);
            std::vector<float> ok(480, 0.99f), over(480, 0.0f), quiet(480, 0.0f);
            m.process(ok.data(), ok.data(), 480);
            expect(!m.snapshot().clip);
            over[100] = -1.0f;
            m.process(over.data(), quiet.data(), 480);
            expect(m.snapshot().clip);
            for (int i = 0; i < 190; ++i) // 1.9 s
                m.process(quiet.data(), quiet.data(), 480);
            expect(m.snapshot().clip);
            for (int i = 0; i < 20; ++i) // past 2 s
                m.process(quiet.data(), quiet.data(), 480);
            expect(!m.snapshot().clip);
            // A new over restarts the hold; reset clears it.
            m.process(over.data(), quiet.data(), 480);
            expect(m.snapshot().clip);
            m.resetClip();
            expect(!m.snapshot().clip);
            // Hold 0 latches until reset.
            m.setHoldSeconds(0.0f);
            m.process(over.data(), quiet.data(), 480);
            for (int i = 0; i < 2000; ++i)
                m.process(quiet.data(), quiet.data(), 480);
            expect(m.snapshot().clip);
            m.resetClip();
            expect(!m.snapshot().clip);
        }

        beginTest("full-scale engine output through the core clips and meters");
        {
            CRUCIBLE_HEAP(EngineCore, core);
            core.prepare(kRate);
            auto v = ParameterMap::defaultValues();
            setValue(v, "master_gain", 6.0f);
            setValue(v, "osc1_level", 1.0f);
            setValue(v, "osc1_uni_voices", 8.0f);
            setValue(v, "osc1_uni_spread", 0.0f);
            setValue(v, "osc1_pos", 1.0f);
            setValue(v, "sub_level", 1.0f);
            setValue(v, "filter_cutoff", 20000.0f);
            std::vector<float> l(512), r(512);
            MidiInput midi[4] = {{0, 0x90, 40, 127}, {0, 0x90, 47, 127}, {0, 0x90, 52, 127}, {0, 0x90, 59, 127}};
            core.process(l.data(), r.data(), 512, v.data(), midi, 4, {});
            for (int i = 0; i < 40; ++i)
                core.process(l.data(), r.data(), 512, v.data(), nullptr, 0, {});
            const auto s = core.meter().snapshot();
            expect(s.peakLeftDb > -40.0f && s.peakLeftDb <= 12.0f, "peak " + String(s.peakLeftDb));
        }
    }
};

static SequencerTests sequencerTests;
static CoreTests coreTests;
static SceneTests sceneTests;
static MeterTests meterTests;
