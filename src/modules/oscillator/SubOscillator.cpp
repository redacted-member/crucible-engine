// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Dsp.h>

#include <algorithm>
#include <cmath>

// Sub oscillator: sine, triangle, square, saw and pulse, one to three octaves below the voice pitch, with a drive
// stage. Edges use polyBLEP (square, saw, pulse) so the waveform is band-limited at every pitch.

namespace crucible::modules
{
namespace
{
float polyBlep(float t, float dt)
{
    if (t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.0f;
    }
    if (t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

float wrap(float x)
{
    return x - std::floor(x);
}

class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        wave = sp.bind("sub_wave");
        octave = sp.bind("sub_octave");
        drive = sp.bind("sub_drive");
        pulse = sp.bind("sub_pulse");
    }

    bool continuesOnRetrigger() const override { return true; }
    void noteOn(const OscNoteInfo&) override { phase = 0.0f; }

    void render(const OscRender& r) override
    {
        const int shape = static_cast<int>(sp.value(r.values, r.targetMod, wave));
        const int oct = static_cast<int>(sp.value(r.values, r.targetMod, octave));
        const float amount = sp.value(r.values, r.targetMod, drive);
        const float width = std::clamp(sp.value(r.values, r.targetMod, pulse), 0.05f, 0.95f);
        const float hz = 440.0f * std::exp2((r.pitch - 69.0f - 12.0f * static_cast<float>(oct + 1)) / 12.0f);
        const float inc = std::min(hz / static_cast<float>(r.sampleRate), 0.49f);
        const float gain = 1.0f + 6.0f * amount;
        const float comp = amount > 0.0f ? 1.0f / std::tanh(gain) : 1.0f;
        for (int i = 0; i < r.numSamples; ++i)
        {
            float s;
            switch (shape)
            {
            case 1:
                s = phase < 0.5f ? 4.0f * phase - 1.0f : 3.0f - 4.0f * phase;
                break;
            case 2:
                s = (phase < 0.5f ? 1.0f : -1.0f) + polyBlep(phase, inc) - polyBlep(wrap(phase + 0.5f), inc);
                break;
            case 3:
                s = 2.0f * phase - 1.0f - polyBlep(phase, inc);
                break;
            case 4:
                s = (phase < width ? 1.0f : -1.0f) + polyBlep(phase, inc) - polyBlep(wrap(phase + 1.0f - width), inc) -
                    (2.0f * width - 1.0f);
                break;
            default:
                s = std::sin(kTwoPi * phase);
                break;
            }
            if (amount > 0.0f)
                s = std::tanh(gain * s) * comp;
            r.left[i] += s;
            r.right[i] += s;
            phase += inc;
            if (phase >= 1.0f)
                phase -= 1.0f;
        }
    }

private:
    SlotParams sp;
    int wave = 0, octave = 0, drive = 0, pulse = 0;
    float phase = 0.0f;
};
} // namespace

ModuleDescriptor subOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.sub";
    d.name = "Sub";
    d.kind = ModuleKind::Oscillator;
    d.category = "Sub";
    d.cpu = CpuClass::Light;
    d.description = "Sub oscillator one to three octaves below the voice pitch: sine, triangle, square, saw, pulse.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = addSlotPage(d, slot, "sub", "Sub", "LCD shows the sub waveform.");
        const auto label = slotLabel(slot);
        d.params.push_back(params::choice(slotId(slot, "sub_wave"), label + " Sub Wave", page,
                                          {"Sine", "Triangle", "Square", "Saw", "Pulse"}, 0));
        d.params.push_back(
            params::choice(slotId(slot, "sub_octave"), label + " Sub Octave", page, {"-1 Oct", "-2 Oct", "-3 Oct"}, 0));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "sub_drive"), label + " Sub Drive", page, 0.0f, 1.0f, 0.0f)));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "sub_pulse"), label + " Sub Pulse Width", page, 0.05f, 0.95f, 0.25f)));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

} // namespace crucible::modules
