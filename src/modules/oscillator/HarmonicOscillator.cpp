// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Cloud.h>

#include <algorithm>
#include <cmath>

// Harmonic (additive) oscillator (P13): a harmonic layer on the cloud renderer. Every control computes a partial
// law for the note, a CloudModel of at most kMaxPartials points that lives inside the instance, and the cloud renderer
// plays it (its rotator bank, the 0.45 x fs band limit and the normalisation are the ones of osc.cloud).
//
// The law for partial n = 1, 2, ... (frequency ratio x_n, level y_n):
//   x_n = n ^ (1 + 0.2 stretch)                             stretch -1 .. 1: 0 is exactly harmonic
//   y_n = odd/even(n) * n ^ (tilt / 6.0206)                 tilt in dB per octave of partial number
//         * 10 ^ (formantGain / 20 * exp(-0.5 (log2(f_n / fc) / sigma)^2))   a Gaussian bell in log frequency at the
//                                                          partial's frequency f_n = x_n f0 (fixed in Hz, not in n)
//         * exp(-decay * 0.5 * (n - 1) * t)                 per-partial decay with the time t since the note started
//   odd/even: +1 removes the even partials, -1 the odd ones, 0 keeps both (linear in between).
// The levels are relative; the cloud renderer normalises their sum, so ratios between partials are exactly the law.

namespace crucible::modules
{
namespace
{
constexpr int kMaxPartials = 128;
constexpr float kFloor = 1.0e-7f; // keeps the rotators of silent partials running so that they stay phase locked

class Instance final : public OscillatorInstance, public CloudOscillatorView
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        count = sp.bind("harm_count");
        oddEven = sp.bind("harm_oddeven");
        stretch = sp.bind("harm_stretch");
        tilt = sp.bind("harm_tilt");
        decay = sp.bind("harm_decay");
        formantHz = sp.bind("harm_formant");
        formantWidth = sp.bind("harm_fwidth");
        formantGain = sp.bind("harm_fgain");
        cloud.points.reserve(kMaxPartials);
        cloud.spread = 1.0f;
    }

    void prepare(double rate, int) override { osc.prepare(rate); }

    void noteOn(const OscNoteInfo& info) override
    {
        osc.noteOn(0.0f, info.seed);
        used = 0;
        first = true;
    }

    void render(const OscRender& r) override
    {
        const auto v = [&](int p) { return sp.value(r.values, r.targetMod, p); };
        const float hz = 440.0f * std::exp2((r.pitch - 69.0f) / 12.0f);
        if (!(hz > 0.0f) || !std::isfinite(hz))
            return;
        const int n = std::clamp(static_cast<int>(v(count)), 1, kMaxPartials);
        used = std::max(used, n);
        // The first chunk of a note covers every rotator, so that a partial that joins later starts from a known state.
        fillLaw(cloud, first ? kMaxPartials : used, n, hz, v(oddEven), v(stretch), v(tilt), v(decay), v(formantHz),
                v(formantWidth), v(formantGain), static_cast<float>(r.noteTime));
        osc.renderModel(&cloud, r.left, r.right, r.numSamples, 0.0f, hz, 1.0f, 0.0f);
        first = false;
    }

    CloudOscillator& cloudOscillator() override { return osc; }

    // The partial law (also used by the tests as the oracle for a given set of controls). Points beyond `active` keep a
    // negligible level.
    static void fillLaw(CloudModel& m, int total, int active, float f0, float oddEvenValue, float stretchValue, float tiltDb,
                        float decayValue, float formantHzValue, float formantOctaves, float formantDb, float noteTime)
    {
        m.points.resize(static_cast<size_t>(total));
        const float oe = std::clamp(oddEvenValue, -1.0f, 1.0f);
        const float sigma = std::max(formantOctaves, 0.05f) / 2.3548f;
        const float fc = std::max(formantHzValue, 10.0f);
        for (int i = 0; i < total; ++i)
        {
            const float nn = static_cast<float>(i + 1);
            const float x = std::pow(nn, 1.0f + 0.2f * std::clamp(stretchValue, -1.0f, 1.0f));
            float y = 1.0f;
            const bool odd = ((i + 1) & 1) != 0;
            y *= odd ? 1.0f + std::min(oe, 0.0f) : 1.0f - std::max(oe, 0.0f);
            y *= std::pow(nn, tiltDb / 6.0206f);
            if (formantDb != 0.0f)
            {
                const float octaves = std::log2(std::max(x * f0, 1.0f) / fc);
                y *= std::pow(10.0f, formantDb / 20.0f * std::exp(-0.5f * (octaves / sigma) * (octaves / sigma)));
            }
            y *= std::exp(-decayValue * 0.5f * (nn - 1.0f) * noteTime);
            auto& p = m.points[static_cast<size_t>(i)];
            p.x = std::min(x, kCloudMaxPartial);
            p.y = i < active && std::isfinite(y) ? std::max(y, kFloor) : kFloor;
            p.z = 0.0f;
            p.phase = 0.0f;
            p.pan = 0.0f;
            p.random = 0.0f;
        }
    }

private:
    SlotParams sp;
    int count = 0, oddEven = 0, stretch = 0, tilt = 0, decay = 0, formantHz = 0, formantWidth = 0, formantGain = 0;
    int used = 0;
    bool first = true;
    CloudModel cloud;
    CloudOscillator osc;
};
} // namespace

ModuleDescriptor harmonicOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.harmonic";
    d.name = "Harmonic";
    d.kind = ModuleKind::Oscillator;
    d.category = "Additive";
    d.cpu = CpuClass::Heavy;
    d.description = "Harmonic additive layer on the cloud renderer: partial count, odd / even balance, stretch, spectral "
                    "tilt, a formant bell fixed in Hz and per-partial decay over the note.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto label = slotLabel(slot);
        const auto main = addSlotPage(d, slot, "harm", "Harmonic", "LCD shows the partial levels.");
        const auto form =
            addSlotPage(d, slot, "harm_x", "Harmonic Formant", "LCD shows the formant bell over the partials.");
        const auto add = [&d](ParamDef p) { d.params.push_back(std::move(p)); };
        add(params::integer(slotId(slot, "harm_count"), label + " Harmonic Count", main, 1, kMaxPartials, 16));
        add(voiceMod(
            params::floating(slotId(slot, "harm_oddeven"), label + " Harmonic Odd / Even", main, -1.0f, 1.0f, 0.0f)));
        add(voiceMod(params::floating(slotId(slot, "harm_stretch"), label + " Harmonic Stretch", main, -1.0f, 1.0f, 0.0f)));
        add(voiceMod(
            params::floating(slotId(slot, "harm_tilt"), label + " Harmonic Tilt", main, -24.0f, 12.0f, -6.0f, "dB/oct")));
        add(voiceMod(
            params::floating(slotId(slot, "harm_decay"), label + " Harmonic Partial Decay", main, 0.0f, 1.0f, 0.0f)));
        add(voiceMod(params::floating(slotId(slot, "harm_formant"), label + " Harmonic Formant Frequency", form, 100.0f,
                                      8000.0f, 1000.0f, "Hz", 0.35f)));
        add(voiceMod(params::floating(slotId(slot, "harm_fwidth"), label + " Harmonic Formant Width", form, 0.1f, 3.0f, 1.0f,
                                      "oct")));
        add(voiceMod(params::floating(slotId(slot, "harm_fgain"), label + " Harmonic Formant Gain", form, -24.0f, 24.0f,
                                      0.0f, "dB")));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

const char* harmonicOscillatorName()
{
    return "Harmonic";
}

} // namespace crucible::modules
