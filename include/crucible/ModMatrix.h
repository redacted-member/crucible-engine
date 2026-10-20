// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/ParameterMap.h>

#include <array>
#include <cmath>

namespace crucible
{

constexpr int kNumModSources = static_cast<int>(ModSource::Count);
constexpr int kNumModDests = static_cast<int>(ModDest::Count);
constexpr int kMaxRoutes = 128; // assignable routes (Modulation.h)

constexpr std::array<ModSource, 4> kLfoSources = {ModSource::Lfo1, ModSource::Lfo2, ModSource::Lfo3, ModSource::Lfo4};

enum class ModPolarity
{
    Native,
    Unipolar,
    Bipolar
};

struct ModSlotParams
{
    int source = 0;
    int destination = 0;
    float amount = 0.0f;
    int polarity = 0;
    float curve = 0.0f;
};

// Sources whose natural range is -1..1; everything else is 0..1.
constexpr bool isBipolarSource(ModSource s)
{
    switch (s)
    {
    case ModSource::Lfo1:
    case ModSource::Lfo2:
    case ModSource::Lfo3:
    case ModSource::Lfo4:
    case ModSource::Key:
    case ModSource::Random:
    case ModSource::PitchBend:
        return true;
    case ModSource::None:
    case ModSource::ModEnv:
    case ModSource::Velocity:
    case ModSource::AmpEnv:
    case ModSource::Env3:
    case ModSource::Env4:
    case ModSource::ModWheel:
    case ModSource::Aftertouch:
    case ModSource::Macro1:
    case ModSource::Macro2:
    case ModSource::Macro3:
    case ModSource::Macro4:
    case ModSource::Macro5:
    case ModSource::Macro6:
    case ModSource::Macro7:
    case ModSource::Macro8:
    case ModSource::StripL:
    case ModSource::StripR:
    case ModSource::SceneMorph:
    case ModSource::Count:
        return false;
    }
    return false;
}

// Applies polarity, curve and amount to a raw source value. nativeBipolar: whether the source is naturally -1..1 (the
// modulator bank and audio-rate sources, which have extended source indices, say so themselves).
inline float evaluateSlot(const ModSlotParams& slot, float raw, bool nativeBipolar)
{
    float v = raw;
    if (slot.polarity == static_cast<int>(ModPolarity::Bipolar) && !nativeBipolar)
        v = 2.0f * raw - 1.0f;
    else if (slot.polarity == static_cast<int>(ModPolarity::Unipolar) && nativeBipolar)
        v = 0.5f * (raw + 1.0f);

    if (slot.curve != 0.0f)
    {
        const float exponent = std::exp2(slot.curve * 2.0f);
        v = std::copysign(std::pow(std::abs(v), exponent), v);
    }
    return v * slot.amount;
}

inline float evaluateSlot(const ModSlotParams& slot, float raw)
{
    return evaluateSlot(slot, raw, isBipolarSource(static_cast<ModSource>(slot.source)));
}

// Sums every active slot into dest (kNumModDests entries). sources has kNumModSources entries.
inline void accumulateModulation(const std::array<ModSlotParams, kNumModSlots>& slots, const float* sources, float* dest)
{
    for (const auto& slot : slots)
    {
        if (slot.source <= 0 || slot.destination <= 0 || slot.source >= kNumModSources || slot.destination >= kNumModDests ||
            slot.amount == 0.0f)
            continue;
        dest[slot.destination] += evaluateSlot(slot, sources[slot.source]);
    }
}

} // namespace crucible
