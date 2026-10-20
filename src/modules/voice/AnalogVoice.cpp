// SPDX-License-Identifier: MIT
#include "../Modules.h"

#include <crucible/module/ParamBuilder.h>

#include <string>

// The analog behaviour of the voice (P11), as parameters of the existing voice: for each of the four envelopes the
// circuit (`Digital`, the default, or `Analog RC`), the retrigger mode and the velocity and key time scaling; the
// voice card (part-to-part tolerances of the oscillators, filters, envelopes and amplifier, derived from the engine seed
// and the voice index, see VoiceCard.h); and the optional noise floor and mains hum. Everything defaults to the legacy
// voice: digital envelopes, no card, no noise. The module is registered after everything else so that the indices of the
// existing parameters do not move.

namespace crucible::modules
{

ModuleDescriptor analogVoice(const VoiceGraphShape&)
{
    ModuleDescriptor d;
    d.id = "voice.analog";
    d.name = "Analog Voice";
    d.kind = ModuleKind::Voice;
    d.category = "Voice";
    d.cpu = CpuClass::Light;
    d.description =
        "Analog RC envelopes with retrigger modes and velocity and key time scaling, voice cards with "
        "part-to-part tolerances for oscillators, filters, envelopes and amplifier, and an optional seeded noise "
        "floor and mains hum.";

    struct Env
    {
        const char* prefix;
        const char* label;
    };
    for (const Env& e : {Env{"amp", "Amp"}, Env{"menv", "Mod Env"}, Env{"env3", "Env 3"}, Env{"env4", "Env 4"}})
    {
        const std::string prefix = e.prefix;
        const std::string label = e.label;
        const std::string page = prefix + "_analog";
        d.pages.push_back({page, label + " Circuit", "env", label + " Circuit",
                           "Digital or analog RC envelope, retrigger mode, and velocity and key scaling of the times."});
        d.params.push_back(params::choice(prefix + "_mode", label + " Mode", page, {"Digital", "Analog RC"}, 0));
        d.params.push_back(
            params::choice(prefix + "_retrig", label + " Retrigger", page, {"From Level", "Reset", "Legato", "ADR"}, 0));
        d.params.push_back(params::floating(prefix + "_overshoot", label + " Overshoot", page, 0.02f, 1.0f, 0.35f, {}, 1.0f,
                                            Smoothing::None));
        d.params.push_back(params::floating(prefix + "_vel_time", label + " Velocity Time", page, -1.0f, 1.0f, 0.0f, {},
                                            1.0f, Smoothing::None));
        d.params.push_back(
            params::floating(prefix + "_key_time", label + " Key Time", page, -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    }

    d.pages.push_back({"voice_card", "Voice Card", "voice", "Voice Card",
                       "Part-to-part tolerances of each voice; the values are fixed per voice and engine seed."});
    d.params.push_back(params::floating("voice_card_osc", "Card Oscillator Tolerance", "voice_card", 0.0f, 1.0f, 0.0f));
    d.params.push_back(params::floating("voice_card_filter", "Card Filter Tolerance", "voice_card", 0.0f, 1.0f, 0.0f));
    d.params.push_back(params::floating("voice_card_env", "Card Envelope Tolerance", "voice_card", 0.0f, 1.0f, 0.0f));
    d.params.push_back(params::floating("voice_card_level", "Card Amp Tolerance", "voice_card", 0.0f, 1.0f, 0.0f));

    d.pages.push_back({"voice_noise", "Noise and Hum", "voice", "Noise Floor and Hum",
                       "Seeded noise floor and mains hum added after the voices, as RMS levels in dBFS."});
    d.params.push_back(params::toggle("voice_noise_on", "Noise Floor", "voice_noise", false));
    d.params.push_back(
        params::floating("voice_noise_db", "Noise Level", "voice_noise", -120.0f, -60.0f, -96.0f, "dB", 1.0f));
    d.params.push_back(params::toggle("voice_hum_on", "Hum", "voice_noise", false));
    d.params.push_back(params::floating("voice_hum_db", "Hum Level", "voice_noise", -120.0f, -60.0f, -84.0f, "dB", 1.0f));
    d.params.push_back(params::choice("voice_hum_hz", "Hum Frequency", "voice_noise", {"50 Hz", "60 Hz"}, 0));
    return d;
}

} // namespace crucible::modules
