// SPDX-License-Identifier: MIT
// Step sequencer Keys trigger (seq_trigger = Keys): the pattern plays only while keys are held, starts from step 1 at the
// first key, is transposed by the newest key relative to C4, and the keys themselves do not sound. Free (the default)
// runs as before.

#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Sequencer.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace crucible;
using crucible::test::UnitTest;
using crucible::testing::kRate;

namespace
{
constexpr int kBlock = 256;

/// Renders `seconds` and returns the peak of each 50 ms window; keys (note, on time, off time) are played as MIDI.
struct Key
{
    int note;
    double on, off;
};

std::vector<double> render(float trigger, const std::vector<Key>& keys, double seconds)
{
    Engine e;
    e.prepare(kRate, kBlock);
    e.setParam("seq_run", 1.0f);
    e.setParam("seq_sync", 0.0f);
    e.setParam("seq_tempo", 120.0f);
    e.setParam("seq_trigger", trigger);
    e.setParam("amp_release", 0.02f);
    CRUCIBLE_HEAP(SeqBank, bank);
    for (int s = 0; s < 16; s += 2)
        bank.pattern[0].steps[static_cast<size_t>(s)] = {true, 48, 110, 1, 0.5f, 1.0f, {}};
    e.sequencer().replace(bank);
    e.service();

    const int total = static_cast<int>(seconds * kRate);
    const int window = static_cast<int>(0.05 * kRate);
    std::vector<double> peaks(static_cast<size_t>(total / window + 1), 0.0);
    std::vector<float> l(kBlock), r(kBlock);
    std::vector<MidiInput> midi;
    for (int pos = 0; pos < total; pos += kBlock)
    {
        const int n = std::min(kBlock, total - pos);
        midi.clear();
        for (const Key& k : keys)
        {
            const int on = static_cast<int>(k.on * kRate), off = static_cast<int>(k.off * kRate);
            if (on >= pos && on < pos + n)
                midi.push_back({on - pos, 0x90, static_cast<std::uint8_t>(k.note), 100});
            if (off >= pos && off < pos + n)
                midi.push_back({off - pos, 0x80, static_cast<std::uint8_t>(k.note), 0});
        }
        std::sort(midi.begin(), midi.end(), [](const MidiInput& a, const MidiInput& b) { return a.position < b.position; });
        e.process(l.data(), r.data(), n, midi.data(), static_cast<int>(midi.size()));
        for (int i = 0; i < n; ++i)
        {
            double& p = peaks[static_cast<size_t>((pos + i) / window)];
            p = std::max(p, static_cast<double>(std::max(std::fabs(l[static_cast<size_t>(i)]), std::fabs(r[static_cast<size_t>(i)]))));
        }
    }
    return peaks;
}

double maxOf(const std::vector<double>& v, double from, double to)
{
    double m = 0.0;
    for (size_t k = static_cast<size_t>(from / 0.05); k < v.size() && static_cast<double>(k) * 0.05 < to; ++k)
        m = std::max(m, v[k]);
    return m;
}

/// The strongest frequency (Hz) of a held Keys-triggered pattern with the key `note`: the pattern note 48 moved by note - 60.
double pitchWithKey(int note)
{
    Engine e;
    e.prepare(kRate, kBlock);
    e.setParam("seq_run", 1.0f);
    e.setParam("seq_tempo", 120.0f);
    e.setParam("seq_trigger", 1.0f);
    e.setParam("osc1_table", 0.0f);
    e.setParam("osc1_pos", 0.0f); // Sine-Saw at position 0: a sine
    e.setParam("filter_on", 0.0f);
    CRUCIBLE_HEAP(SeqBank, bank);
    bank.pattern[0].steps[0] = {true, 48, 110, 1, 3.5f, 1.0f, {}};
    e.sequencer().replace(bank);
    e.service();
    const int total = static_cast<int>(0.4 * kRate);
    std::vector<float> l(static_cast<size_t>(total)), r(static_cast<size_t>(total));
    const MidiInput on{0, 0x90, static_cast<std::uint8_t>(note), 100};
    for (int pos = 0; pos < total; pos += kBlock)
        e.process(l.data() + pos, r.data() + pos, std::min(kBlock, total - pos), pos == 0 ? &on : nullptr, pos == 0 ? 1 : 0);
    // Zero crossings over the last 0.2 s.
    int crossings = 0;
    const int from = total / 2;
    for (int i = from + 1; i < total; ++i)
        if ((l[static_cast<size_t>(i - 1)] < 0.0f) != (l[static_cast<size_t>(i)] < 0.0f))
            ++crossings;
    return crossings / 2.0 / ((total - from) / kRate);
}
} // namespace

class SeqTriggerTests : public UnitTest
{
public:
    SeqTriggerTests() : UnitTest("Seq trigger", "Sequencer") {}

    void runTest() override
    {
        beginTest("Keys: silent with no key, plays while a key is held, stops after release; the key does not sound");
        {
            const auto peaks = render(1.0f, {{60, 0.5, 1.5}}, 2.5);
            // nothing before the key (the pattern starts with the block the key falls in: up to one block early)
            expect(maxOf(peaks, 0.0, 0.45) < 1.0e-6);
            expect(maxOf(peaks, 0.55, 1.5) > 0.01);   // the pattern plays
            expect(maxOf(peaks, 1.8, 2.5) < 1.0e-4);  // and stops with the release (amp release 20 ms)
        }

        beginTest("Free (the default) runs without keys, as before");
        {
            const auto peaks = render(0.0f, {}, 1.0);
            expect(maxOf(peaks, 0.0, 1.0) > 0.01);
        }

        beginTest("Keys: the newest key transposes the pattern relative to C4");
        {
            const double at60 = pitchWithKey(60); // pattern note 48: C3, 130.8 Hz
            const double at72 = pitchWithKey(72); // an octave up
            expectWithinAbsoluteError(at60, 130.81, 4.0);
            expectWithinAbsoluteError(at72 / at60, 2.0, 0.06);
        }
    }
};

static SeqTriggerTests seqTriggerTests;
