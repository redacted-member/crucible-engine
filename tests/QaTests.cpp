// SPDX-License-Identifier: MIT
// Regression and robustness: golden-render fingerprints with tolerance, block-size and API parity, and fuzzing
// of parameters and MIDI.
//
// Golden fingerprints live in tests/golden/renders.txt. After an intended change of sound, regenerate them with
//   CRUCIBLE_UPDATE_GOLDEN=1 build/crucible_engine_tests Golden
// and review the diff.

#include <crucible/Engine.h>
#include <crucible/crucible.h>
#include "TestSupport.h"

#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>

#ifndef CRUCIBLE_ENGINE_GOLDEN_DIR
#define CRUCIBLE_ENGINE_GOLDEN_DIR "tests/golden"
#endif

using namespace crucible;
using namespace crucible::testing;

namespace
{
struct Scenario
{
    const char* name;
    std::vector<std::pair<const char*, float>> params;
    std::vector<int> notes;
};

const std::vector<Scenario>& scenarios()
{
    static const std::vector<Scenario> list = {
        {"default_chord", {}, {48, 55, 60}},
        {"warps_filters_matrix",
         {{"osc1_warp", 1.0f},
          {"osc1_warp_a", 0.5f},
          {"osc1_uni_voices", 4.0f},
          {"osc2_level", 0.6f},
          {"osc2_warp", 6.0f},
          {"filter_type", 9.0f},
          {"filter_res", 0.6f},
          {"filter_cutoff", 1200.0f},
          {"filter2_on", 1.0f},
          {"filter2_type", 12.0f},
          {"filter_route", 1.0f},
          {"mod1_src", 1.0f},
          {"mod1_dst", 7.0f},
          {"mod1_amt", 0.4f}},
         {45, 57}},
        {"classic_fx",
         {{"osc1_pos", 1.0f},
          // The first-generation effects of this scenario (drive, chorus, phaser, delay, reverb, comp, eq) became
          // their successors at the closest settings when they were removed (docs/SOUND_CHANGES.md).
          {"fx_saturator_on", 1.0f},
          {"fx_saturator_drive", 12.0f},
          {"fx_saturator_autogain", 0.0f},
          {"fx_saturator_output", -4.2f},
          {"fx_ensemble_on", 1.0f},
          {"fx_ensemble_noise", -100.0f},
          {"fx_ensemble_compander", 0.0f},
          {"fx_phaser2_on", 1.0f},
          {"fx_phaser2_stages", 3.0f},
          {"fx_phaser2_range", 1.8f},
          {"fx_delay_pro_on", 1.0f},
          {"fx_delay_pro_saturation", 0.0f},
          {"fx_ra_on", 1.0f},
          {"fx_ra_algo", 3.0f},
          {"fx_ra_mix", 0.25f},
          {"fx_cc_on", 1.0f},
          {"fx_ep_on", 1.0f},
          {"fx_ep_b1_on", 1.0f},
          {"fx_ep_b1_type", 1.0f},
          {"fx_ep_b1_freq", 120.0f},
          {"fx_ep_b2_on", 1.0f},
          {"fx_ep_b2_freq", 1000.0f},
          {"fx_ep_b3_on", 1.0f},
          {"fx_ep_b3_type", 2.0f},
          {"fx_ep_b3_freq", 6000.0f},
          {"fx_limit_on", 1.0f},
          {"fx_width_on", 1.0f}},
         {52, 59}},
        {"new_fx",
         {{"osc1_pos", 1.0f},
          {"fx_ep_on", 1.0f},
          {"fx_ep_b1_on", 1.0f},
          {"fx_ep_b1_freq", 800.0f},
          {"fx_ep_b1_gain", 6.0f},
          {"fx_ep_b1_q", 0.707f},
          {"fx_filter_on", 1.0f},
          {"fx_filter_lfo_depth", 2.0f},
          {"fx_ensemble_on", 1.0f},
          {"fx_ensemble_mode", 2.0f},
          {"fx_ensemble_compander", 0.0f},
          {"fx_multiband_on", 1.0f},
          {"fx_trance_on", 1.0f},
          {"fx_tremolo_on", 1.0f},
          {"fx_ring_on", 1.0f},
          {"fx_ring_mix", 0.3f}},
         {50, 62}},
        {"arp_and_sequencer",
         {{"arp_on", 1.0f},
          {"arp_sync", 0.0f},
          {"arp_mode", 2.0f},
          {"arp_octaves", 2.0f},
          {"seq_run", 1.0f},
          {"seq_tempo", 120.0f},
          {"amp_release", 0.05f}},
         {60, 64, 67}},
    };
    return list;
}

// RMS per 2400-sample window (50 ms), left and right, in dB.
std::vector<double> fingerprint(const Scenario& s, int block)
{
    Engine e;
    e.prepare(kRate, 512);
    for (const auto& [id, v] : s.params)
        e.setParam(id, v);
    CRUCIBLE_HEAP(SeqBank, bank);
    bank.pattern[0].steps[0] = {true, 36, 110, 1, 0.5f, 1.0f, {}};
    bank.pattern[0].steps[6] = {true, 43, 90, 2, 0.5f, 1.0f, {}};
    e.sequencer().replace(bank);
    const int total = 48000;
    std::vector<float> l(static_cast<size_t>(total)), r(static_cast<size_t>(total));
    std::vector<MidiInput> midi;
    for (const int n : s.notes)
        midi.push_back({0, 0x90, static_cast<std::uint8_t>(n), 100});
    std::vector<MidiInput> offs;
    for (int pos = 0; pos < total; pos += block)
    {
        const int n = std::min(block, total - pos);
        const bool first = pos == 0;
        const bool release = pos <= 30016 && pos + n > 30016;
        offs.clear();
        for (const int note : s.notes)
            offs.push_back({30016 - pos, 0x80, static_cast<std::uint8_t>(note), 0});
        e.process(l.data() + pos, r.data() + pos, n, first ? midi.data() : (release ? offs.data() : nullptr),
                  first ? static_cast<int>(midi.size()) : (release ? static_cast<int>(offs.size()) : 0));
    }
    std::vector<double> fp;
    for (int w = 0; w < total; w += 2400)
        for (const auto* ch : {&l, &r})
        {
            double sum = 0.0;
            for (int i = w; i < w + 2400; ++i)
                sum += static_cast<double>((*ch)[static_cast<size_t>(i)]) * (*ch)[static_cast<size_t>(i)];
            fp.push_back(10.0 * std::log10(std::max(sum / 2400.0, 1.0e-20)));
        }
    return fp;
}

std::map<std::string, std::vector<double>> readGolden(const std::string& path)
{
    std::map<std::string, std::vector<double>> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream is(line);
        is.imbue(std::locale::classic());
        std::string name;
        is >> name;
        double v;
        while (is >> v)
            out[name].push_back(v);
    }
    return out;
}
} // namespace

class QaTests : public UnitTest
{
public:
    QaTests() : UnitTest("Golden renders and fuzzing", "Engine") {}

    void runTest() override
    {
        const std::string path = std::string(CRUCIBLE_ENGINE_GOLDEN_DIR) + "/renders.txt";
        beginTest("golden-render fingerprints match within 0.05 dB");
        {
            const char* update = std::getenv("CRUCIBLE_UPDATE_GOLDEN");
            if (update != nullptr && std::string(update) == "1")
            {
                std::ofstream f(path);
                f << "# RMS in dB per 50 ms window (left, right), 1 s per scenario; see tests/QaTests.cpp\n";
                f.imbue(std::locale::classic());
                f.precision(6);
                for (const auto& s : scenarios())
                {
                    f << s.name;
                    for (const double v : fingerprint(s, 256))
                        f << ' ' << std::fixed << v;
                    f << '\n';
                }
                logMessage("golden fingerprints rewritten: " + path);
            }
            const auto golden = readGolden(path);
            expect(golden.size() == scenarios().size(), "golden file " + path);
            for (const auto& s : scenarios())
            {
                const auto it = golden.find(s.name);
                if (it == golden.end())
                {
                    expect(false, std::string("missing golden ") + s.name);
                    continue;
                }
                const auto now = fingerprint(s, 256);
                expect(now.size() == it->second.size(), s.name);
                double worst = 0.0;
                for (size_t i = 0; i < std::min(now.size(), it->second.size()); ++i)
                    if (it->second[i] > -150.0 || now[i] > -150.0)
                        worst = std::max(worst, std::abs(now[i] - it->second[i]));
                expect(worst < 0.05, String(s.name) + " deviates by " + String(worst) + " dB");
            }
        }

        beginTest("parity: block size does not change the sound");
        {
            // Rendering runs in 32-sample control chunks that restart at every event, so blocks that are multiples
            // of 32 with events on that grid are bit-identical; events off the grid (the arpeggiator's steps)
            // shift chunk boundaries and differ only by control-rate rounding.
            for (const auto& s : scenarios())
            {
                const auto a = fingerprint(s, 64);
                const auto b = fingerprint(s, 512);
                if (std::string(s.name) != "arp_and_sequencer")
                    expect(a == b, s.name);
                double worst = 0.0;
                for (size_t i = 0; i < a.size(); ++i)
                    if (a[i] > -150.0)
                        worst = std::max(worst, std::abs(a[i] - b[i]));
                expect(worst < 0.05, String(s.name) + " block-size deviation " + String(worst) + " dB");
            }
        }

        beginTest("parity: the C API renders exactly what the C++ facade renders");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("osc1_pos", 0.4f);
            CrucibleEngine* c = crucible_create(nullptr);
            crucible_prepare(c, kRate, 256);
            crucible_set_param_id(c, "osc1_pos", 0.4f);
            std::vector<float> l1(256), r1(256), l2(256), r2(256);
            MidiInput m{3, 0x90, 57, 99};
            CrucibleMidiEvent cm{3, 0x90, 57, 99};
            bool same = true;
            for (int b = 0; b < 20; ++b)
            {
                e.process(l1.data(), r1.data(), 256, b == 0 ? &m : nullptr, b == 0 ? 1 : 0);
                crucible_process(c, l2.data(), r2.data(), 256, b == 0 ? &cm : nullptr, b == 0 ? 1 : 0);
                same = same && l1 == l2 && r1 == r2;
            }
            crucible_destroy(c);
            expect(same);
        }

        beginTest("determinism: same seed, same samples; a different seed changes per-note randomness only");
        {
            const auto run = [](std::uint32_t seed)
            {
                EngineConfig cfg;
                cfg.seed = seed;
                Engine e(cfg);
                e.prepare(kRate, 256);
                e.setParam("osc1_uni_voices", 6.0f);
                e.setParam("osc1_uni_rand", 1.0f);
                e.setParam("voice_drift", 1.0f);
                std::vector<float> out, l(256), r(256);
                MidiInput m{0, 0x90, 50, 100};
                for (int b = 0; b < 30; ++b)
                {
                    e.process(l.data(), r.data(), 256, b == 0 ? &m : nullptr, b == 0 ? 1 : 0);
                    out.insert(out.end(), l.begin(), l.end());
                }
                return out;
            };
            expect(run(7) == run(7));
            expect(run(7) != run(8));
        }

        beginTest("fuzz: random parameters and random MIDI never produce NaN, infinity or runaway levels");
        {
            Rng rng;
            rng.state = 0x1234567U;
            const auto& defs = ParameterMap::params();
            int failures = 0;
            for (int round = 0; round < 24; ++round)
            {
                EngineConfig cfg;
                cfg.maxVoices = 1 + static_cast<int>(rng.unipolar() * 24.0f);
                Engine e(cfg);
                e.prepare(round % 3 == 0 ? 44100.0 : (round % 3 == 1 ? 48000.0 : 96000.0), 256);
                for (size_t i = 0; i < defs.size(); ++i)
                {
                    const auto& d = defs[i];
                    if (d.id == "fx_cx_on" || d.id == "seq_rec")
                        continue; // the convolver is covered elsewhere and slow; recording only fills a queue
                    if (rng.unipolar() < 0.5f)
                        e.setParamNormalised(static_cast<int>(i), rng.unipolar());
                }
                std::vector<float> l(256), r(256);
                float peak = 0.0f;
                bool finite = true;
                for (int b = 0; b < 40; ++b)
                {
                    std::vector<MidiInput> midi;
                    const int count = static_cast<int>(rng.unipolar() * 6.0f);
                    for (int k = 0; k < count; ++k)
                    {
                        static const std::uint8_t kinds[] = {0x90, 0x80, 0xE0, 0xB0, 0xD0, 0x90};
                        const auto kind = kinds[static_cast<int>(rng.unipolar() * 5.99f)];
                        midi.push_back({static_cast<int>(rng.unipolar() * 255.0f), kind,
                                        static_cast<std::uint8_t>(rng.unipolar() * 127.0f),
                                        static_cast<std::uint8_t>(rng.unipolar() * 127.0f)});
                    }
                    std::sort(midi.begin(), midi.end(),
                              [](const MidiInput& a, const MidiInput& b) { return a.position < b.position; });
                    e.process(l.data(), r.data(), 256, midi.data(), static_cast<int>(midi.size()));
                    for (int i = 0; i < 256; ++i)
                    {
                        finite =
                            finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
                        peak = std::max({peak, std::abs(l[static_cast<size_t>(i)]), std::abs(r[static_cast<size_t>(i)])});
                    }
                }
                if (!finite || peak > 1000.0f)
                {
                    ++failures;
                    logMessage("round " + String(round) + (finite ? "" : " non-finite") + " peak " + String(peak));
                }
            }
            expectEquals(failures, 0);
        }
    }
};

class AnalyzerTests : public UnitTest
{
public:
    AnalyzerTests() : UnitTest("Master analyzer", "Engine") {}

    void runTest() override
    {
        const auto feed = [](MasterAnalyzer& a, double hz, float amp, double phase, bool invertRight, int seconds)
        {
            std::vector<float> l(480), r(480);
            long long n = 0;
            for (int b = 0; b < seconds * 100; ++b)
            {
                for (int i = 0; i < 480; ++i, ++n)
                {
                    l[static_cast<size_t>(i)] =
                        amp * static_cast<float>(std::sin(2.0 * 3.14159265358979 * hz * n / kRate + phase));
                    r[static_cast<size_t>(i)] = invertRight ? -l[static_cast<size_t>(i)] : l[static_cast<size_t>(i)];
                }
                a.process(l.data(), r.data(), 480);
            }
        };

        beginTest("loudness: a 1 kHz stereo sine at -20 dBFS reads -20 LUFS momentary and short-term");
        {
            MasterAnalyzer a;
            a.prepare(kRate);
            feed(a, 1000.0, 0.1f, 0.0, false, 4);
            const auto s = a.loudness();
            expectWithinAbsoluteError(s.momentaryLufs, -20.0f, 0.15f);
            expectWithinAbsoluteError(s.shortTermLufs, -20.0f, 0.15f);
            expectWithinAbsoluteError(s.correlation, 1.0f, 1.0e-3f);
            MasterAnalyzer b;
            b.prepare(kRate);
            feed(b, 1000.0, 0.1f, 0.0, true, 1);
            expectWithinAbsoluteError(b.loudness().correlation, -1.0f, 1.0e-3f);
            MasterAnalyzer quiet;
            quiet.prepare(kRate);
            feed(quiet, 1000.0, 0.0f, 0.0, false, 1);
            expect(quiet.loudness().momentaryLufs <= -120.0f && quiet.loudness().truePeakDb <= -120.0f);
        }

        beginTest("true peak finds inter-sample peaks the sample values miss");
        {
            MasterAnalyzer a;
            a.prepare(kRate);
            // fs / 4 with a 45 degree phase: samples sit at 0.707 of the waveform peak.
            feed(a, kRate / 4.0, 0.5f, 3.14159265358979 / 4.0, false, 1);
            const auto s = a.loudness();
            expect(s.maxTruePeakDb > 20.0f * std::log10(0.5f * 0.7072f) + 2.0f, "true peak " + String(s.maxTruePeakDb));
            expectWithinAbsoluteError(s.maxTruePeakDb, 20.0f * std::log10(0.5f), 0.6f);
            a.resetMax();
            expect(a.loudness().maxTruePeakDb <= -120.0f);
        }

        beginTest("scope and spectrum snapshots; disabled analyzer does no work");
        {
            MasterAnalyzer a;
            a.prepare(kRate);
            feed(a, 1500.0, 0.25f, 0.0, false, 1);
            std::vector<float> l, r;
            a.scope(l, r, 1024);
            expect(l.size() == 1024 && std::abs(*std::max_element(l.begin(), l.end()) - 0.25f) < 0.01f);
            const auto spec = a.spectrum(2048);
            expectEquals(static_cast<int>(spec.size()), 1025);
            const auto peakBin = std::max_element(spec.begin(), spec.end()) - spec.begin();
            expectWithinAbsoluteError(static_cast<double>(peakBin) * kRate / 2048.0, 1500.0, kRate / 2048.0);
            expectWithinAbsoluteError(spec[static_cast<size_t>(peakBin)], 20.0f * std::log10(0.25f), 1.5f);
            MasterAnalyzer off;
            off.prepare(kRate);
            off.setEnabled(false);
            feed(off, 1000.0, 0.5f, 0.0, false, 1);
            expect(off.loudness().momentaryLufs <= -120.0f && off.loudness().truePeakDb <= -120.0f);
        }
    }
};

static QaTests qaTests;
static AnalyzerTests analyzerTests;
