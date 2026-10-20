// SPDX-License-Identifier: MIT
#include <crucible/FilterBank.h>

#include <algorithm>
#include <cstring>

namespace crucible
{

void FilterBank::prepare(double sampleRate, Quality quality, int voices, int slots, std::uint32_t seed)
{
    rate = sampleRate;
    baseSeed = seed;
    voiceCount = std::max(voices, 0);
    slotCount = std::max(slots, 0);
    pairs = (voiceCount + 1) / 2;
    paddedVoices = pairs * 2;
    quads.clear();
    if (voiceCount == 0 || slotCount == 0)
    {
        slotCount = 0;
        return;
    }
    using Factory = std::unique_ptr<AnalogFilter> (*)();
    static constexpr Factory factories[kModels] = {makeTransistorLadder, makeOtaCascade,        makeDiodeLadder,
                                                   makeSallenKeyLowpass, makeSallenKeyHighpass, makeSvfOta,
                                                   makeOta4Pole,         makePassiveHp};
    quads.resize(static_cast<size_t>(slotCount) * kModels * static_cast<size_t>(pairs));
    for (int s = 0; s < slotCount; ++s)
        for (int m = 0; m < kModels; ++m)
            for (int p = 0; p < pairs; ++p)
            {
                auto q = factories[m]();
                q->prepare(sampleRate, quality, 4, kBlock);
                q->setSeed(seed + static_cast<std::uint32_t>(m) + 16U * static_cast<std::uint32_t>(p));
                quads[quadIndex(s, m, p)] = std::move(q);
            }
    highpassStep.assign(quads.size(), 0);
    buffers.assign(static_cast<size_t>(slotCount) * static_cast<size_t>(paddedVoices) * 2 * kBlock, 0.0f);
    fed.assign(static_cast<size_t>(slotCount) * static_cast<size_t>(paddedVoices), 0);
    modFed.assign(fed.size(), 0);
    cutoffMod.assign(fed.size() * kBlock, 0.0f);
}

void FilterBank::beginChunk()
{
    std::fill(fed.begin(), fed.end(), static_cast<unsigned char>(0));
    std::fill(modFed.begin(), modFed.end(), static_cast<unsigned char>(0));
}

void FilterBank::setCutoffMod(int slot, int voice, const float* octaves, int n)
{
    const size_t v = static_cast<size_t>(slot) * static_cast<size_t>(paddedVoices) + static_cast<size_t>(voice);
    std::memcpy(cutoffMod.data() + v * kBlock, octaves, sizeof(float) * static_cast<size_t>(std::min(n, kBlock)));
    modFed[v] = 1;
}

void FilterBank::resetVoice(int slot, int model, int voice)
{
    auto& q = *quads[quadIndex(slot, model, voice / 2)];
    const int first = (voice & 1) * 2;
    q.resetLane(first);
    q.resetLane(first + 1);
}

void FilterBank::seedVoice(int slot, int model, int voice, bool useCard, std::uint32_t cardSeed)
{
    const int pair = voice / 2;
    auto& q = *quads[quadIndex(slot, model, pair)];
    const int first = (voice & 1) * 2;
    for (int c = 0; c < 2; ++c)
    {
        const int lane = first + c;
        if (useCard)
            q.setLaneSeed(lane, cardSeed + 16U * static_cast<std::uint32_t>(c) + static_cast<std::uint32_t>(model), 0);
        else
            q.setLaneSeed(lane, baseSeed + static_cast<std::uint32_t>(model) + 16U * static_cast<std::uint32_t>(pair), lane);
    }
}

void FilterBank::configure(int slot, int model, int voice, const FilterSettings& s, float drive)
{
    const auto index = quadIndex(slot, model, voice / 2);
    auto& q = *quads[index];
    const float fc = std::clamp(s.cutoffHz, 20.0f, 0.45f * static_cast<float>(rate));
    const float res = std::clamp(s.resonance, 0.0f, 1.0f);
    const int first = (voice & 1) * 2;
    q.setLaneControls(first, fc, res, drive, s.character, s.tolerance);
    q.setLaneControls(first + 1, fc, res, drive, s.character, s.tolerance);
    if (highpassStep[index] != s.highpass)
    {
        q.setHighpassStep(s.highpass);
        highpassStep[index] = s.highpass;
    }
    fed[static_cast<size_t>(slot) * static_cast<size_t>(paddedVoices) + static_cast<size_t>(voice)] = 1;
}

void FilterBank::process(int slot, int model, int n)
{
    const size_t base = static_cast<size_t>(slot) * static_cast<size_t>(paddedVoices);
    for (int p = 0; p < pairs; ++p)
    {
        const bool a = fed[base + static_cast<size_t>(2 * p)] != 0, b = fed[base + static_cast<size_t>(2 * p + 1)] != 0;
        if (!a && !b)
            continue;
        // A lane whose voice does not sound runs on silence.
        if (!a)
        {
            std::memset(lane(slot, 2 * p, 0), 0, sizeof(float) * kBlock);
            std::memset(lane(slot, 2 * p, 1), 0, sizeof(float) * kBlock);
        }
        if (!b)
        {
            std::memset(lane(slot, 2 * p + 1, 0), 0, sizeof(float) * kBlock);
            std::memset(lane(slot, 2 * p + 1, 1), 0, sizeof(float) * kBlock);
        }
        float* io[4] = {lane(slot, 2 * p, 0), lane(slot, 2 * p, 1), lane(slot, 2 * p + 1, 0), lane(slot, 2 * p + 1, 1)};
        ++quadBlocks;
        const bool ma = modFed[base + static_cast<size_t>(2 * p)] != 0,
                   mb = modFed[base + static_cast<size_t>(2 * p + 1)] != 0;
        if (ma || mb)
        {
            // Both channels of a voice share one modulation; a lane without it is given null.
            const float* modA = ma ? cutoffMod.data() + (base + static_cast<size_t>(2 * p)) * kBlock : nullptr;
            const float* modB = mb ? cutoffMod.data() + (base + static_cast<size_t>(2 * p + 1)) * kBlock : nullptr;
            const float* const mods[4] = {modA, modA, modB, modB};
            quads[quadIndex(slot, model, p)]->processQuad(io, n, mods);
        }
        else
            quads[quadIndex(slot, model, p)]->processQuad(io, n, nullptr);
    }
}

} // namespace crucible
