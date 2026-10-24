// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Dsp.h>

#include <algorithm>
#include <array>
#include <cmath>

// Noise oscillator: white, pink, brown, blue, violet, velvet and crackle, a tilt control and stereo width.

namespace crucible::modules
{
namespace
{
class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        type = sp.bind("noise_type");
        color = sp.bind("noise_color");
        width = sp.bind("noise_width");
        density = sp.bind("noise_density");
    }

    void prepare(double rate, int) override { sr = rate; }

    bool continuesOnRetrigger() const override { return true; }
    void noteOn(const OscNoteInfo& info) override
    {
        rng.state = (info.seed * 3266489917U) | 1U;
        st = State{};
    }

    void render(const OscRender& r) override
    {
        const int kind = static_cast<int>(sp.value(r.values, r.targetMod, type));
        const float tilt = sp.value(r.values, r.targetMod, color);
        const float w = sp.value(r.values, r.targetMod, width);
        const float dens = sp.value(r.values, r.targetMod, density);
        const float c = 1.0f - w;
        const float cross = std::sqrt(std::max(0.0f, 1.0f - c * c));
        const float a = std::abs(tilt);
        // Tilt: a one-pole low-pass (tilt < 0) whose corner falls from 20 kHz to 160 Hz with |tilt|, or its complementary
        // high-pass (tilt > 0) whose corner rises from 20 Hz to 2.56 kHz, so the sound gets steadily brighter both ways.
        const float fc = tilt < 0.0f ? 20000.0f * std::exp2(-7.0f * a) : 20.0f * std::exp2(7.0f * a);
        const float coef = 1.0f - std::exp(-6.2831853f * std::min(fc, 0.45f * static_cast<float>(r.sampleRate)) /
                                           static_cast<float>(r.sampleRate));
        const float velvetGain = std::min(1.0f, std::sqrt(2000.0f / std::max(dens, 1.0f)));
        const float velvetProb = dens / static_cast<float>(r.sampleRate);
        for (int i = 0; i < r.numSamples; ++i)
        {
            const float nl = sample(0, kind, velvetProb, velvetGain);
            const float ni = sample(1, kind, velvetProb, velvetGain);
            float l = nl;
            float rr = cross * ni + c * nl;
            if (a > 0.01f)
            {
                lp[0] += coef * (l - lp[0]);
                lp[1] += coef * (rr - lp[1]);
                const float mixAmt = std::min(1.0f, a * 4.0f);
                const float fl = tilt < 0.0f ? lp[0] : l - lp[0];
                const float fr = tilt < 0.0f ? lp[1] : rr - lp[1];
                l += (fl - l) * mixAmt;
                rr += (fr - rr) * mixAmt;
            }
            r.left[i] += l * 0.5f;
            r.right[i] += rr * 0.5f;
        }
    }

private:
    struct State
    {
        std::array<float, 2> pink0{}, pink1{}, pink2{}, brown{}, crackle{}, prev{};
    };

    float sample(int ch, int kind, float velvetProb, float velvetGain)
    {
        const auto c = static_cast<size_t>(ch);
        const float white = rng.bipolar();
        switch (kind)
        {
        case 1:
            st.pink0[c] = 0.99765f * st.pink0[c] + white * 0.0990460f;
            st.pink1[c] = 0.96300f * st.pink1[c] + white * 0.2965164f;
            st.pink2[c] = 0.57000f * st.pink2[c] + white * 1.0526913f;
            return (st.pink0[c] + st.pink1[c] + st.pink2[c] + white * 0.1848f) * 0.25f;
        case 2:
            st.brown[c] = std::clamp(st.brown[c] * 0.998f + white * 0.04f, -1.0f, 1.0f);
            return st.brown[c] * 3.0f;
        case 3: // blue: differentiated white
        {
            const float out = (white - st.prev[c]) * 0.5f;
            st.prev[c] = white;
            return out;
        }
        case 4: // violet: second difference
        {
            const float d = white - st.prev[c];
            const float out = (d - st.crackle[c]) * 0.25f;
            st.prev[c] = white;
            st.crackle[c] = d;
            return out;
        }
        case 5: // velvet: sparse bipolar impulses
            return rng.unipolar() < velvetProb ? (rng.unipolar() < 0.5f ? -1.0f : 1.0f) * velvetGain : 0.0f;
        case 6:
            if (rng.unipolar() < 0.0015f)
                st.crackle[c] = rng.bipolar() * 1.5f;
            st.crackle[c] *= 0.7f;
            return st.crackle[c];
        default:
            return white;
        }
    }

    SlotParams sp;
    int type = 0, color = 0, width = 0, density = 0;
    double sr = 48000.0;
    Rng rng;
    State st;
    std::array<float, 2> lp{};
};
} // namespace

ModuleDescriptor noiseOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.noise";
    d.name = "Noise";
    d.kind = ModuleKind::Oscillator;
    d.category = "Noise";
    d.cpu = CpuClass::Light;
    d.description = "White, pink, brown, blue, violet, velvet and crackle noise with a tilt filter and stereo width.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = addSlotPage(d, slot, "noise", "Noise", "LCD shows the noise spectrum tilt.");
        const auto label = slotLabel(slot);
        d.params.push_back(params::choice(slotId(slot, "noise_type"), label + " Noise Type", page,
                                          {"White", "Pink", "Brown", "Blue", "Violet", "Velvet", "Crackle"}, 0));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "noise_color"), label + " Noise Tilt", page, -1.0f, 1.0f, 0.0f)));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "noise_width"), label + " Noise Width", page, 0.0f, 1.0f, 1.0f)));
        d.params.push_back(voiceMod(params::floating(slotId(slot, "noise_density"), label + " Velvet Density", page, 10.0f,
                                                     20000.0f, 2000.0f, "Hz", 0.3f)));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

} // namespace crucible::modules
