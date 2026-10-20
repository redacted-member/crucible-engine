// SPDX-License-Identifier: MIT
#include <crucible/module/ConvolutionIr.h>
#include <crucible/Engine.h>
#include <crucible/crucible.h>
#include <crucible/SmoothedValue.h>
#include <crucible/SpscRing.h>
#include "AllocCounter.h"
#include "TestSupport.h"

#include <cstdio>
#include <thread>

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Block
{
    float peak = 0.0f;
    bool finite = true;
};

Block run(Engine& e, int samples, int block = 512, const MidiInput* midi = nullptr, int numMidi = 0)
{
    Block b;
    std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
    for (int done = 0; done < samples; done += block)
    {
        const int n = std::min(block, samples - done);
        e.process(l.data(), r.data(), n, done == 0 ? midi : nullptr, done == 0 ? numMidi : 0);
        for (int i = 0; i < n; ++i)
        {
            b.finite = b.finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
            b.peak = std::max({b.peak, std::abs(l[static_cast<size_t>(i)]), std::abs(r[static_cast<size_t>(i)])});
        }
    }
    return b;
}
} // namespace

class FacadeTests : public UnitTest
{
public:
    FacadeTests() : UnitTest("Engine facade", "Engine") {}

    void runTest() override
    {
        beginTest("parameters: lookup, clamping, normalised and CC mapping");
        {
            Engine e;
            const int cutoff = e.model().indexOf("filter_cutoff");
            expect(cutoff >= 0 && e.model().indexOf("nope") == -1);
            expectEquals(e.getParam(cutoff), e.model().params()[static_cast<size_t>(cutoff)].defaultValue);
            expect(e.setParam("filter_cutoff", 1.0e9f));
            expectEquals(e.getParam(cutoff), 20000.0f);
            expect(!e.setParam(-1, 1.0f) && !e.setParam(e.model().size(), 1.0f));
            expect(e.setParamNormalised(cutoff, 0.0f));
            expectEquals(e.getParam(cutoff), 20.0f);
            const int table = e.model().indexOf("osc1_table");
            e.setParamNormalised(table, 0.5f);
            expectEquals(e.getParam(table), std::round(e.getParam(table)));

            e.mapCC(74, cutoff);
            expectEquals(e.ccMapping(74), cutoff);
            e.mapCC(1, cutoff); // reserved for the mod wheel
            expectEquals(e.ccMapping(1), -1);
            expect(e.midi(0xb3, 74, 127)); // any channel
            expectEquals(e.getParam(cutoff), 20000.0f);
            e.mapCC(74, -1);
            expect(!e.midi(0xb0, 74, 0));
            expectEquals(e.getParam(cutoff), 20000.0f);
            e.resetParamsToDefaults();
            expectEquals(e.getParam(cutoff), e.model().params()[static_cast<size_t>(cutoff)].defaultValue);
        }

        beginTest("setParamRt: allocation free; service() arms the oscillator and syncs the sequencer");
        {
            Engine e;
            e.prepare(48000.0, 512);
            const int type = e.model().indexOf("osc1_type");
            const int table = e.model().indexOf("osc1_table");
            const int pattern = e.model().indexOf("seq_pattern");
            const int cutoff = e.model().indexOf("filter_cutoff");
            expect(type >= 0 && table >= 0 && pattern >= 0 && cutoff >= 0);
            gAllocations = 0;
            gTrackAllocations = true;
            const bool ok = e.setParamRt(type, 8.0f) && e.setParamRt(table, 7.0f) && e.setParamRt(pattern, 3.0f) &&
                            e.setParamRt(cutoff, 1.0e9f);
            const bool bad =
                e.setParamRt(-1, 1.0f) || e.setParamRt(e.model().size(), 1.0f) || e.setParamRt(cutoff, std::nanf(""));
            gTrackAllocations = false;
            expect(ok && !bad);
            expectEquals(static_cast<int>(gAllocations), 0, "setParamRt allocations");
            expectEquals(e.getParam(type), 8.0f);
            expectEquals(e.getParam(cutoff), 20000.0f);

            e.sequencer().edit([](SeqBank& b) { b.pattern[3].lengthBars = 2; });
            e.service();
            expectEquals(e.getParam("seq_length"), 2.0f);
            // Spectral needs a sample to sound; Modal (6) sounds on its own.
            expect(e.setParamRt(type, 6.0f));
            e.service();
            e.noteOn(60, 1.0f);
            const Block b = run(e, 8192);
            expect(b.finite && b.peak > 0.0f, "voice sounds after service()");
        }

        beginTest("process(): a CC mapped to an oscillator type allocates nothing; service() arms it");
        {
            Engine e;
            e.prepare(48000.0, 512);
            const int type = e.model().indexOf("osc1_type");
            e.mapCC(20, type);
            const auto& d = e.model().params()[static_cast<size_t>(type)];
            const int cc = static_cast<int>(std::lround(8.0f / (d.max - d.min) * 127.0f)); // Spectral
            std::vector<float> l(256), r(256);
            const MidiInput ev[1] = {{0, 0xb0, 20, static_cast<std::uint8_t>(cc)}};
            gAllocations = 0;
            gTrackAllocations = true;
            e.process(l.data(), r.data(), 256, ev, 1);
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "process allocations with a mapped CC");
            expectEquals(e.getParam(type), 8.0f);
            e.service();
            e.process(l.data(), r.data(), 256);
        }

        beginTest("modulatedValue(id) resolves the id in this engine's own product");
        {
            ProductLayout product;
            product.id = "facade-product";
            product.sections = {{"sound", "Sound"}, {"fx", "FX"}};
            product.voice = {4, 2};
            product.macros = 8;
            EngineConfig c;
            c.product = product;
            Engine e(c);
            e.prepare(kRate, 256);
            const Engine defaults;
            expect(e.model().indexOf("macro1") != defaults.model().indexOf("macro1"), "the product moves the indices");
            expect(e.setParam("macro1", 0.75f));
            run(e, 256);
            expectWithinAbsoluteError(e.modulatedValue("macro1"), 0.75f, 1.0e-3f);
        }

        beginTest("C API: crucible_process_events with a param event allocates nothing; crucible_service applies it");
        {
            CrucibleConfig config{};
            config.struct_size = sizeof(config);
            CrucibleEngine* ce = crucible_create(&config);
            expect(ce != nullptr);
            if (ce != nullptr)
            {
                crucible_prepare(ce, 48000.0, 512);
                const int type = crucible_param_index(ce, "osc1_type");
                const int cutoff = crucible_param_index(ce, "filter_cutoff");
                std::vector<float> l(256), r(256);
                const CrucibleMidiEvent note[1] = {{0, 0x90, 60, 100}};
                const CrucibleParamEvent changes[2] = {{0, type, 6.0f}, {0, cutoff, 900.0f}};
                CrucibleTransport transport{};
                transport.struct_size = sizeof(transport);
                transport.bpm = 120.0;
                transport.ppq = 4.0;
                transport.playing = 1;
                crucible_process_events(ce, l.data(), r.data(), 256, nullptr, 0, nullptr, 0, nullptr); // warm up
                gAllocations = 0;
                gTrackAllocations = true;
                crucible_process_events(ce, l.data(), r.data(), 256, note, 1, changes, 2, &transport);
                gTrackAllocations = false;
                expectEquals(static_cast<int>(gAllocations), 0, "crucible_process_events allocations");
                expectEquals(crucible_get_param(ce, type), 6.0f);
                expectEquals(crucible_get_param(ce, cutoff), 900.0f);
                crucible_service(ce);
                float peak = 0.0f;
                for (int block = 0; block < 16; ++block)
                {
                    crucible_process_events(ce, l.data(), r.data(), 256, nullptr, 0, nullptr, 0, nullptr);
                    for (float v : l)
                        peak = std::max(peak, std::abs(v));
                }
                expect(peak > 0.0f, "the voice sounds after crucible_service");
                crucible_destroy(ce);
            }
        }

        beginTest("notes from the input thread render, release to silence; raw MIDI works");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("amp_release", 0.05f);
            expect(run(e, 4096).peak == 0.0f);
            expect(e.noteOn(60, 0.9f));
            const auto on = run(e, 9600);
            expect(on.finite && on.peak > 0.01f, "peak " + String(on.peak));
            expect(e.activeVoiceCount() >= 1);
            expect(e.noteOff(60));
            run(e, 24000);
            expect(run(e, 4800).peak < 1.0e-4f);

            expect(e.midi(0x92, 64, 100)); // note on, channel 3
            expect(run(e, 4800).peak > 0.01f);
            expect(e.midi(0x92, 64, 0)); // velocity 0 = note off
            run(e, 24000);
            expect(run(e, 4800).peak < 1.0e-4f);
            expect(!e.noteOn(128, 1.0f));
        }

        beginTest("sample-accurate MIDI, blocks larger than maxBlockSize are split");
        {
            Engine e;
            e.prepare(kRate, 64);
            std::vector<float> l(1000, 1.0f), r(1000, 1.0f);
            MidiInput midi[1] = {{700, 0x90, 60, 127}};
            e.process(l.data(), r.data(), 1000, midi, 1);
            float before = 0.0f, after = 0.0f;
            for (int i = 0; i < 700; ++i)
                before = std::max(before, std::abs(l[static_cast<size_t>(i)]));
            for (int i = 700; i < 1000; ++i)
                after = std::max(after, std::abs(l[static_cast<size_t>(i)]));
            expect(before == 0.0f, "silent before the note");
            expect(after > 0.0f, "sound after the note");
        }

        beginTest("pitch bend, mod wheel, aftertouch and sustain reach the engine");
        {
            Engine e;
            e.prepare(kRate, 512);
            e.setParam("amp_release", 0.02f);
            expect(e.sustain(true));
            e.noteOn(48, 1.0f);
            run(e, 2048);
            e.noteOff(48);
            run(e, 9600);
            expect(e.activeVoiceCount() == 1, "sustain holds the note");
            e.sustain(false);
            run(e, 9600);
            expectEquals(e.activeVoiceCount(), 0);
            expect(e.pitchBend(0.5f) && e.modWheel(0.3f) && e.aftertouch(0.7f) && e.allNotesOff());
            expect(run(e, 512).finite);
        }

        beginTest("scenes store, recall and morph through the facade");
        {
            Engine e;
            e.setParam("filter_cutoff", 500.0f);
            e.storeScene(0);
            e.setParam("filter_cutoff", 8000.0f);
            e.storeScene(1);
            expect(e.recallScene(0));
            expectWithinAbsoluteError(e.getParam("filter_cutoff"), 500.0f, 0.01f);
            expect(e.morphScenes(0, 1, 0.5f));
            const float mid = e.getParam("filter_cutoff");
            expect(mid > 500.0f && mid < 8000.0f);
            expect(!e.recallScene(5));
        }

        beginTest("sequencer runs from the facade with host tempo");
        {
            Engine e;
            e.prepare(kRate, 512);
            CRUCIBLE_HEAP(SeqBank, bank);
            bank.pattern[0].steps[0] = {true, 60, 110, 1, 0.5f, 1.0f, {}};
            e.sequencer().replace(bank);
            e.setParam("seq_run", 1.0f);
            SeqHostTime host;
            host.hasBpm = true;
            host.bpm = 120.0;
            std::vector<float> l(512), r(512);
            float peak = 0.0f;
            for (int b = 0; b < 10; ++b)
            {
                e.process(l.data(), r.data(), 512, nullptr, 0, &host);
                for (const float x : l)
                    peak = std::max(peak, std::abs(x));
            }
            expect(peak > 0.01f && e.sequencerRunning());
        }

        beginTest("user content: WAV wavetable, cloud wavetable and impulse response load through the facade");
        {
            Engine e;
            e.prepare(kRate, 256);
            AudioData wav;
            wav.sampleRate = kRate;
            wav.channels.resize(1);
            for (int i = 0; i < 4096; ++i) // two frames: sine then saw
                wav.channels[0].push_back(i < 2048 ? std::sin(6.2831853f * static_cast<float>(i) / 2048.0f)
                                                   : 1.0f - 2.0f * static_cast<float>(i - 2048) / 2048.0f);
            std::string error;
            const auto path = std::string("crucible_engine_test_wave.wav");
            expect(writeWavFile(path, wav));
            expect(e.loadWavetableFile(0, path, WavetableImport{}, error), error);
            std::remove(path.c_str());
            expect(e.userWavetables().table(0)->sourceFrameCount() == 2);
            expect(!e.loadWavetableFile(1, "/does/not/exist.wav", WavetableImport{}, error));
            expect(e.loadCloudWavetable(1, CloudModel::harmonicMorph(32)));
            expectEquals(e.userWavetables().name(1), std::string("Harmonic Morph"));
            e.setParam("osc1_table", static_cast<float>(WavetableBank::numTables + 1));
            e.noteOn(57, 1.0f);
            const auto played = run(e, 4800);
            expect(played.finite && played.peak > 0.05f);

            ImpulseResponse ir;
            ir.sampleRate = 44100.0;
            ir.channels = {std::vector<float>(4410, 0.0f)};
            ir.channels[0][0] = 1.0f;
            expect(e.loadConvolutionIr(ir, "click"));
            expect(e.coreEngine().engine().effects().findCapability<ConvolutionIr>()->hasImpulseResponse());
            e.prepare(44100.0, 256); // the response is rebuilt for the new rate
            expect(e.coreEngine().engine().effects().findCapability<ConvolutionIr>()->hasImpulseResponse());
        }

        beginTest("SPSC ring: order, capacity N - 1, concurrent producer and consumer");
        {
            SpscRing<int, 4> ring;
            expect(ring.push(1) && ring.push(2) && ring.push(3) && !ring.push(4));
            int v = 0;
            expect(ring.pop(v) && v == 1 && ring.size() == 2);
            ring.reset();
            expect(!ring.pop(v));

            SpscRing<int, 64> big;
            constexpr int count = 200000;
            std::thread producer(
                [&]
                {
                    for (int i = 0; i < count;)
                        if (big.push(i))
                            ++i;
                });
            int expected = 0;
            bool ordered = true;
            while (expected < count)
            {
                int x;
                if (big.pop(x))
                {
                    ordered = ordered && x == expected;
                    ++expected;
                }
            }
            producer.join();
            expect(ordered);
        }

        beginTest("linear smoother ramps, ignores equal targets and snaps with zero steps");
        {
            LinearSmoothedValue s;
            s.reset(100.0, 0.04); // 4 steps
            s.setCurrentAndTargetValue(0.0f);
            s.setTargetValue(1.0f);
            expectWithinAbsoluteError(s.getNextValue(), 0.25f, 1.0e-6f);
            expectWithinAbsoluteError(s.getNextValue(), 0.5f, 1.0e-6f);
            s.setTargetValue(1.0f); // same target: the ramp continues
            expectWithinAbsoluteError(s.getNextValue(), 0.75f, 1.0e-6f);
            expectEquals(s.getNextValue(), 1.0f);
            expect(!s.isSmoothing());
            s.reset(0);
            s.setTargetValue(3.0f);
            expectEquals(s.getNextValue(), 3.0f);
        }

        beginTest("FFT: Accelerate and portable paths agree and round-trip");
        {
            const Fft fft(10);
            std::vector<float> a(2048, 0.0f);
            for (int i = 0; i < 1024; ++i)
                a[static_cast<size_t>(i)] =
                    std::sin(0.05f * static_cast<float>(i)) + 0.3f * std::cos(0.7f * static_cast<float>(i * i % 97));
            auto b = a;
            const auto original = a;
            fft.forward(a.data());
            fft.forwardPortable(b.data());
            float maxDiff = 0.0f;
            for (int k = 0; k <= 1024; ++k)
                maxDiff = std::max(maxDiff, std::abs(a[static_cast<size_t>(k)] - b[static_cast<size_t>(k)]));
            expect(maxDiff < 1.0e-3f, "forward diff " + String(maxDiff));
            fft.inverse(a.data());
            fft.inversePortable(b.data());
            float rt = 0.0f, rtp = 0.0f;
            for (int i = 0; i < 1024; ++i)
            {
                rt = std::max(rt, std::abs(a[static_cast<size_t>(i)] - original[static_cast<size_t>(i)]));
                rtp = std::max(rtp, std::abs(b[static_cast<size_t>(i)] - original[static_cast<size_t>(i)]));
            }
            expect(rt < 1.0e-4f && rtp < 1.0e-4f, "round trip " + String(rt) + " / " + String(rtp));
        }
    }
};

static FacadeTests facadeTests;
