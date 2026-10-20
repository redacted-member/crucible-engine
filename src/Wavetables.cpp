// SPDX-License-Identifier: MIT
#include <crucible/Wavetables.h>

#include <crucible/Fft.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <memory>

namespace crucible
{
namespace
{
constexpr float kPi = 3.14159265358979323846f;
constexpr int kMaxHarmonic = WavetableBank::tableSize / 2;

float hash01(unsigned n)
{
    n = (n << 13U) ^ n;
    n = n * (n * n * 15731U + 789221U) + 1376312589U;
    return static_cast<float>(n & 0xffffffU) / static_cast<float>(0x1000000U);
}

using Coefficient = std::complex<float>;

// Spectral generators: coefficient of harmonic h (>= 1) for morph position t in [0, 1].
Coefficient sineToSaw(float t, int h)
{
    const auto a = h == 1 ? 1.0f : t / static_cast<float>(h);
    return {0.0f, a};
}

Coefficient pwm(float t, int h)
{
    const auto width = 0.5f - 0.45f * t;
    const auto hf = static_cast<float>(h);
    return {2.0f / (kPi * hf) * std::sin(kPi * hf * width), 0.0f};
}

Coefficient formant(float t, int h)
{
    const auto centre = 2.0f + 38.0f * t;
    const auto width = 1.5f + 0.05f * centre;
    const auto d = (static_cast<float>(h) - centre) / width;
    const auto a = 2.0f * std::exp(-0.5f * d * d) + 0.4f / static_cast<float>(h);
    return {0.0f, a};
}

Coefficient spectral(float t, int h)
{
    const auto limit = 3.0f + t * 61.0f;
    const auto hf = static_cast<float>(h);
    if (hf > limit)
        return {0.0f, 0.0f};
    const auto slot = t * 8.0f;
    const auto k = static_cast<unsigned>(slot);
    const auto frac = slot - static_cast<float>(k);
    const auto gate0 = hash01(static_cast<unsigned>(h) * 7U + k * 131U);
    const auto gate1 = hash01(static_cast<unsigned>(h) * 7U + (k + 1U) * 131U);
    const auto gate = 0.3f + 0.7f * (gate0 + (gate1 - gate0) * frac);
    const auto a = gate / std::pow(hf, 1.3f - 0.8f * t);
    const auto phase = 2.0f * kPi * hash01(static_cast<unsigned>(h) * 977U);
    return {a * std::cos(phase), a * std::sin(phase)};
}

// Saw with a resonant low-pass sweep from a near-sine to the full sawtooth.
Coefficient analogSaw(float t, int h)
{
    const auto hf = static_cast<float>(h);
    const auto fc = 1.5f * std::pow(600.0f, t);
    const auto r = hf / fc;
    const auto lp = 1.0f / std::sqrt(1.0f + r * r * r * r);
    const auto bump = 1.0f + 1.2f * (1.0f - t) * std::exp(-0.5f * (r - 1.0f) * (r - 1.0f) * 8.0f);
    return {0.0f, lp * bump / hf};
}

// Square with the same kind of sweep; odd harmonics only.
Coefficient analogSquare(float t, int h)
{
    if (h % 2 == 0)
        return {0.0f, 0.0f};
    const auto hf = static_cast<float>(h);
    const auto fc = 1.5f * std::pow(500.0f, t);
    const auto r = hf / fc;
    const auto lp = 1.0f / std::sqrt(1.0f + r * r * r * r);
    return {0.0f, lp / hf};
}

// Triangle with variable symmetry: rising time d goes from 0.5 to 0.97 (towards a ramp).
Coefficient analogTri(float t, int h)
{
    const auto d = 0.5f + 0.47f * t;
    const auto hf = static_cast<float>(h);
    return {0.0f, 2.0f * std::sin(kPi * hf * d) / (kPi * kPi * hf * hf * d * (1.0f - d))};
}

// Vowels A, E, I, O, U with three formants each (Hz, relative gain).
Coefficient vocal(float t, int h)
{
    struct Vowel
    {
        float f[3];
        float g[3];
    };
    static constexpr std::array<Vowel, 5> vowels = {{
        {{800.0f, 1150.0f, 2900.0f}, {1.0f, 0.5f, 0.25f}},
        {{400.0f, 1600.0f, 2700.0f}, {1.0f, 0.6f, 0.3f}},
        {{350.0f, 1700.0f, 2700.0f}, {1.0f, 0.55f, 0.3f}},
        {{450.0f, 800.0f, 2830.0f}, {1.0f, 0.6f, 0.2f}},
        {{325.0f, 700.0f, 2530.0f}, {1.0f, 0.4f, 0.2f}},
    }};
    const auto pos = t * 4.0f;
    const auto i = std::min(static_cast<int>(pos), 3);
    const auto frac = pos - static_cast<float>(i);
    constexpr float f0 = 110.0f;
    const auto freq = static_cast<float>(h) * f0;
    float sum = 0.0f;
    for (int k = 0; k < 3; ++k)
    {
        const auto fc = vowels[static_cast<size_t>(i)].f[k] * (1.0f - frac) + vowels[static_cast<size_t>(i + 1)].f[k] * frac;
        const auto g = vowels[static_cast<size_t>(i)].g[k] * (1.0f - frac) + vowels[static_cast<size_t>(i + 1)].g[k] * frac;
        const auto bw = 70.0f + 0.08f * fc;
        const auto d = (freq - fc) / bw;
        sum += g / (1.0f + d * d);
    }
    return {0.0f, sum / std::pow(static_cast<float>(h), 0.6f)};
}

// A travelling group of partials over a faint 1/h background.
Coefficient additive(float t, int h)
{
    const auto hf = static_cast<float>(h);
    const auto centre = 1.0f + 45.0f * t;
    const auto w = 1.5f + 0.25f * centre;
    const auto d = (hf - centre) / w;
    const auto alt = (h % 2 == 1) ? 1.0f : 0.6f;
    return {0.0f, alt * (std::exp(-0.5f * d * d) + 0.12f) / std::pow(hf, 0.7f)};
}

// Randomised magnitudes (key-framed, interpolated) with fixed random phases.
Coefficient noiseTexture(float t, int h)
{
    const auto slot = t * 6.0f;
    const auto k = std::min(static_cast<unsigned>(slot), 5U);
    const auto frac = slot - static_cast<float>(k);
    const auto r0 = hash01(static_cast<unsigned>(h) * 31U + k * 7919U);
    const auto r1 = hash01(static_cast<unsigned>(h) * 31U + (k + 1U) * 7919U);
    const auto r = r0 + (r1 - r0) * frac;
    const auto slope = 0.5f + 0.7f * (1.0f - t);
    const auto a = (0.05f + r * r) / std::pow(static_cast<float>(h), slope);
    const auto phase = 2.0f * kPi * hash01(static_cast<unsigned>(h) * 4099U + 17U);
    return {a * std::cos(phase), a * std::sin(phase)};
}

Coefficient organ(float t, int h)
{
    static constexpr std::array<std::array<float, 8>, 8> presets = {{
        {{1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 0.8f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 0.8f, 0.6f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 0.0f, 0.7f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f}},
        {{1.0f, 0.7f, 0.5f, 0.7f, 0.0f, 0.5f, 0.0f, 0.3f}},
        {{0.6f, 1.0f, 0.4f, 0.8f, 0.3f, 0.6f, 0.0f, 0.5f}},
        {{0.3f, 0.5f, 1.0f, 0.5f, 0.7f, 0.4f, 0.6f, 0.4f}},
        {{1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}},
    }};
    // Drawbar harmonics 1,2,3,4,5,6,8,10 -> index mapping.
    static constexpr std::array<int, 8> harmonics = {1, 2, 3, 4, 5, 6, 8, 10};
    const auto pos = t * 7.0f;
    const auto i = std::min(static_cast<int>(pos), 6);
    const auto frac = pos - static_cast<float>(i);
    for (size_t k = 0; k < harmonics.size(); ++k)
        if (harmonics[k] == h)
            return {0.0f,
                    presets[static_cast<size_t>(i)][k] * (1.0f - frac) + presets[static_cast<size_t>(i + 1)][k] * frac};
    return {0.0f, 0.0f};
}

// Time-domain generators write one cycle of tableSize samples.
using CycleGenerator = void (*)(float, float*);

float foldWave(float x)
{
    x += 1.0f;
    x -= 4.0f * std::floor(x * 0.25f);
    if (x > 2.0f)
        x = 4.0f - x;
    return x - 1.0f;
}

void fmBells(float t, float* out)
{
    const auto index = 7.0f * t;
    for (int i = 0; i < WavetableBank::tableSize; ++i)
    {
        const auto ph = 2.0f * kPi * static_cast<float>(i) / WavetableBank::tableSize;
        out[i] = std::sin(ph + index * std::sin(3.0f * ph) + 0.4f * index * std::sin(7.0f * ph));
    }
}

void digitalHarsh(float t, float* out)
{
    const auto gain = 1.0f + 12.0f * t * t;
    const auto phaseSteps = static_cast<float>(1 << (11 - static_cast<int>(t * 7.0f)));
    const auto levels = std::max(3.0f, 256.0f * (1.0f - t) * (1.0f - t));
    for (int i = 0; i < WavetableBank::tableSize; ++i)
    {
        const auto ph = static_cast<float>(i) / WavetableBank::tableSize;
        const auto q = std::floor(ph * phaseSteps) / phaseSteps;
        auto y = foldWave(gain * std::sin(2.0f * kPi * q));
        y = std::round(y * levels * 0.5f) / (levels * 0.5f);
        out[i] = y;
    }
}

void syncSaw(float t, float* out)
{
    const auto ratio = 1.0f + 7.0f * t;
    for (int i = 0; i < WavetableBank::tableSize; ++i)
    {
        const auto ph = static_cast<float>(i) / WavetableBank::tableSize * ratio;
        out[i] = 2.0f * (ph - std::floor(ph)) - 1.0f;
    }
}

struct TableDef
{
    const char* name;
    Coefficient (*spectral)(float, int);
    CycleGenerator cycle;
};

constexpr std::array<TableDef, WavetableBank::numTables> tableDefs = {{
    {"Sine-Saw", sineToSaw, nullptr},
    {"PWM", pwm, nullptr},
    {"Formant", formant, nullptr},
    {"Spectral", spectral, nullptr},
    {"Analog Saw", analogSaw, nullptr},
    {"Analog Square", analogSquare, nullptr},
    {"Analog Tri", analogTri, nullptr},
    {"Vocal", vocal, nullptr},
    {"FM Bells", nullptr, fmBells},
    {"Additive", additive, nullptr},
    {"Noise Texture", noiseTexture, nullptr},
    {"Digital Harsh", nullptr, digitalHarsh},
    {"Organ", organ, nullptr},
    {"Sync Saw", nullptr, syncSaw},
}};
} // namespace

const std::vector<std::string>& WavetableBank::tableNames()
{
    static const std::vector<std::string> names = []
    {
        std::vector<std::string> n;
        for (const auto& d : tableDefs)
            n.emplace_back(d.name);
        return n;
    }();
    return names;
}

const WavetableBank& WavetableBank::instance()
{
    static const WavetableBank bank;
    return bank;
}

void WavetableBank::ensure(int table) const
{
    if (table < 0 || table >= numTables)
        return;
    auto& slot = slots[static_cast<size_t>(table)];
    if (slot.data.load(std::memory_order_acquire) != nullptr)
        return;
    std::lock_guard<std::mutex> lock(generateMutex);
    if (slot.data.load(std::memory_order_relaxed) != nullptr)
        return;
    auto storage = std::make_unique<float[]>(perTable());
    generate(table, storage.get());
    slot.storage = std::move(storage);
    slot.data.store(slot.storage.get(), std::memory_order_release);
}

void WavetableBank::ensureAll() const
{
    for (int t = 0; t < numTables; ++t)
        ensure(t);
}

size_t WavetableBank::residentBytes() const
{
    size_t n = 0;
    for (int t = 0; t < numTables; ++t)
        if (resident(t))
            n += bytesPerTable();
    return n;
}

void WavetableBank::generate(int table, float* out)
{
    std::fill(out, out + perTable(), 0.0f);

    const Fft fft(11);
    std::vector<std::unique_ptr<Fft>> levelFft(13);
    for (const auto& lv : levels)
    {
        int order = 0;
        while ((1 << order) < lv.size)
            ++order;
        if (!levelFft[static_cast<size_t>(order)])
            levelFft[static_cast<size_t>(order)] = std::make_unique<Fft>(order);
    }
    std::vector<float> spectrum(static_cast<size_t>(tableSize) * 2);
    std::vector<float> work(static_cast<size_t>(maxMipSize) * 2);

    {
        const auto& def = tableDefs[static_cast<size_t>(table)];
        for (int fr = 0; fr < numFrames; ++fr)
        {
            const auto t = static_cast<float>(fr) / static_cast<float>(numFrames - 1);

            std::fill(spectrum.begin(), spectrum.end(), 0.0f);
            if (def.cycle != nullptr)
            {
                std::fill(work.begin(), work.end(), 0.0f);
                def.cycle(t, work.data());
                fft.forward(work.data());
                // Forward output is unnormalised; scale so it matches the inverse's convention.
                for (int h = 1; h <= kMaxHarmonic - 1; ++h)
                {
                    spectrum[static_cast<size_t>(2 * h)] = work[static_cast<size_t>(2 * h)] / tableSize;
                    spectrum[static_cast<size_t>(2 * h + 1)] = work[static_cast<size_t>(2 * h + 1)] / tableSize;
                }
            }
            else
            {
                for (int h = 1; h <= kMaxHarmonic - 1; ++h)
                {
                    const auto c = def.spectral(t, h);
                    spectrum[static_cast<size_t>(2 * h)] = c.real();
                    spectrum[static_cast<size_t>(2 * h + 1)] = c.imag();
                }
            }

            // Loudness: the peak of the full-band cycle (1023 partials, 2048 samples) normalises the frame.
            float scale = 1.0f;
            {
                std::fill(work.begin(), work.begin() + 2 * tableSize, 0.0f);
                for (int h = 1; h <= kMaxHarmonic - 1; ++h)
                {
                    work[static_cast<size_t>(2 * h)] = spectrum[static_cast<size_t>(2 * h)];
                    work[static_cast<size_t>(2 * h + 1)] = spectrum[static_cast<size_t>(2 * h + 1)];
                }
                fft.inverse(work.data());
                float peak = 0.0f;
                for (int i = 0; i < tableSize; ++i)
                    peak = std::max(peak, std::abs(work[static_cast<size_t>(i)]));
                scale = peak > 1.0e-9f ? 1.0f / peak : 1.0f;
            }
            for (int mip = 0; mip < numMips; ++mip)
            {
                const int size = mipSize(mip);
                int order = 0;
                while ((1 << order) < size)
                    ++order;
                std::fill(work.begin(), work.begin() + 2 * size, 0.0f);
                const float gain = static_cast<float>(size) / static_cast<float>(tableSize);
                for (int h = 1; h <= mipHarmonics(mip); ++h)
                {
                    work[static_cast<size_t>(2 * h)] = spectrum[static_cast<size_t>(2 * h)] * gain;
                    work[static_cast<size_t>(2 * h + 1)] = spectrum[static_cast<size_t>(2 * h + 1)] * gain;
                }
                levelFft[static_cast<size_t>(order)]->inverse(work.data());

                auto* dst = out + mipOffset(mip) +
                            static_cast<size_t>(fr) * static_cast<size_t>(size + guardBefore + guardAfter) + guardBefore;
                for (int i = 0; i < size; ++i)
                    dst[i] = work[static_cast<size_t>(i)] * scale;
                dst[-2] = dst[size - 2];
                dst[-1] = dst[size - 1];
                dst[size] = dst[0];
                dst[size + 1] = dst[1];
                dst[size + 2] = dst[2];
            }
        }
    }
}

} // namespace crucible
