// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Cloud.h>

#include <array>
#include <cmath>

// The 3D cloud as a voice oscillator: additive rendering of the slot's cloud model (Engine::setCloud) at the voice
// pitch, with the morph position and width as parameters. Partials above 0.45 x the sample rate are skipped, so
// the oscillator is band-limited at every pitch.

namespace crucible::modules
{
namespace
{
class Instance final : public OscillatorInstance, public CloudOscillatorView
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        pos = sp.bind("cloud_pos");
        width = sp.bind("cloud_width");
        pan = sp.bind("cloud_pan");
        for (int b = 0; b < kCloudBands; ++b)
            band[static_cast<size_t>(b)] = sp.bind("cloud_band" + std::to_string(b + 1));
    }

    void prepare(double rate, int) override { osc.prepare(rate); }
    bool continuesOnRetrigger() const override { return true; }
    void noteOn(const OscNoteInfo& info) override { osc.noteOn(0.0f, info.seed); }
    void noteOff() override {}

    void render(const OscRender& r) override
    {
        if (r.cloud == nullptr)
            return;
        const float hz = 440.0f * std::exp2((r.pitch - 69.0f) / 12.0f);
        std::array<float, kCloudBands> gains;
        for (size_t b = 0; b < gains.size(); ++b)
            gains[b] = sp.value(r.values, r.targetMod, band[b]);
        osc.renderModel(r.cloud, r.left, r.right, r.numSamples, sp.value(r.values, r.targetMod, pos), hz,
                        sp.value(r.values, r.targetMod, width), sp.value(r.values, r.targetMod, pan), gains.data());
    }

    CloudOscillator& cloudOscillator() override { return osc; }

private:
    SlotParams sp;
    int pos = 0, width = 0, pan = 0;
    std::array<int, kCloudBands> band{};
    CloudOscillator osc;
};
} // namespace

ModuleDescriptor cloudOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.cloud";
    d.name = "Cloud";
    d.kind = ModuleKind::Oscillator;
    d.category = "Additive";
    d.version = 2; // 2: renders the spectral drawbars (see cloudBands); patches without them load with every band at 1
    d.cpu = CpuClass::Heavy;
    d.description = "Direct additive rendering of a 3D point cloud of partials (inharmonic spectra allowed), morphed "
                    "along the cloud's z axis.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = addSlotPage(d, slot, "cloud", "Cloud", "LCD shows the cloud slice at the morph position.");
        const auto label = slotLabel(slot);
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "cloud_pos"), label + " Cloud Position", page, 0.0f, 1.0f, 0.0f)));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "cloud_width"), label + " Cloud Width", page, 0.02f, 1.0f, 0.15f)));
        d.params.push_back(
            voiceMod(params::floating(slotId(slot, "cloud_pan"), label + " Cloud Stereo", page, 0.0f, 1.0f, 1.0f)));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

// The spectral drawbars of every cloud slot. A module of their own, registered after everything else, so that the
// generic modulation destinations of all earlier parameters keep their numbers in existing patches.
ModuleDescriptor cloudBands(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "voice.cloud_bands";
    d.name = "Cloud Spectral Drawbars";
    d.kind = ModuleKind::Voice;
    d.category = "Additive";
    d.cpu = CpuClass::Light;
    d.description = "Eight octave-band gains over the partials of the cloud oscillator.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto page = slotId(slot, "cloud");
        const auto label = slotLabel(slot);
        // Bands 1-3 fill the cloud page, bands 4-8 sit on a second page (a page holds six).
        const auto pageB = addSlotPage(d, slot, "cloud_b", "Cloud Bands", "LCD shows the spectrum with eight octave bands.");
        static const char* const bandNames[kCloudBands] = {"1", "2", "3-4", "5-8", "9-16", "17-32", "33-64", "65+"};
        for (int b = 0; b < kCloudBands; ++b)
            d.params.push_back(voiceMod(params::floating(slotId(slot, "cloud_band" + std::to_string(b + 1)),
                                                         label + " Cloud Band " + bandNames[b], b < 3 ? page : pageB, 0.0f,
                                                         2.0f, 1.0f, "", 1.0f, Smoothing::Log)));
        for (size_t k = d.params.size() - kCloudBands; k < d.params.size(); ++k)
            d.params[k].uiHint = UiHint::Drawbar;
    }
    d.perOscillator = true;
    return d;
}

} // namespace crucible::modules
