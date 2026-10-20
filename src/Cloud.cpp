// SPDX-License-Identifier: MIT
#include <crucible/Cloud.h>

#include "TextFormat.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace crucible
{
namespace
{
constexpr double kTwoPi = 6.283185307179586;

float rand01(std::uint32_t& s)
{
    s ^= s << 13U;
    s ^= s >> 17U;
    s ^= s << 5U;
    return static_cast<float>(s >> 8U) / 16777216.0f;
}

std::uint32_t mix(std::uint32_t a, std::uint32_t b)
{
    std::uint32_t h = a * 0x9e3779b9U ^ (b + 0x7f4a7c15U + (a << 6U) + (a >> 2U));
    return h == 0 ? 1U : h;
}
} // namespace

void CloudModel::sanitise()
{
    if (points.size() > static_cast<size_t>(kCloudMaxPoints))
        points.resize(static_cast<size_t>(kCloudMaxPoints));
    for (auto& p : points)
    {
        p.x = std::isfinite(p.x) ? std::clamp(p.x, 1.0f, kCloudMaxPartial) : 1.0f;
        p.y = std::isfinite(p.y) ? std::clamp(p.y, 0.0f, 1.0f) : 0.0f;
        p.z = std::isfinite(p.z) ? std::clamp(p.z, 0.0f, 1.0f) : 0.0f;
        p.phase = std::isfinite(p.phase) ? p.phase - std::floor(p.phase) : 0.0f;
        p.pan = std::isfinite(p.pan) ? std::clamp(p.pan, -1.0f, 1.0f) : 0.0f;
        p.random = std::isfinite(p.random) ? std::clamp(p.random, 0.0f, 1.0f) : 0.0f;
    }
    spread = std::isfinite(spread) ? std::clamp(spread, 0.01f, 2.0f) : 0.15f;
}

float CloudModel::weightWithSpread(const CloudPoint& pt, float position, float spreadValue) const
{
    const float d = (pt.z - position) / std::max(spreadValue, 0.01f);
    return pt.y * std::exp(-0.5f * d * d);
}

std::string CloudModel::serialise() const
{
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os << "cloud 1 " << text::fixed(spread, 6) << ' ' << name << '\n';
    for (const auto& p : points)
        os << text::fixed(p.x, 6) << ' ' << text::fixed(p.y, 6) << ' ' << text::fixed(p.z, 6) << ' '
           << text::fixed(p.phase, 6) << ' ' << text::fixed(p.pan, 6) << ' ' << text::fixed(p.random, 6) << '\n';
    return os.str();
}

bool CloudModel::deserialise(const std::string& textIn, CloudModel& out)
{
    std::istringstream is(textIn);
    is.imbue(std::locale::classic());
    std::string tag;
    int version = 0;
    CloudModel m;
    if (!(is >> tag >> version >> m.spread) || tag != "cloud" || version != 1)
        return false;
    std::getline(is, m.name);
    if (!m.name.empty() && m.name[0] == ' ')
        m.name.erase(0, 1);
    CloudPoint p;
    while (is >> p.x >> p.y >> p.z >> p.phase >> p.pan >> p.random)
        m.points.push_back(p);
    m.sanitise();
    out = std::move(m);
    return true;
}

CloudModel CloudModel::harmonicMorph(int partials)
{
    CloudModel m;
    m.name = "Harmonic Morph";
    partials = std::clamp(partials, 1, kCloudMaxPoints / 2);
    // z = 0: a saw-like series; z = 1: sparse, slightly inharmonic bell partials.
    for (int h = 1; h <= partials; ++h)
        m.points.push_back({static_cast<float>(h), 1.0f / static_cast<float>(h), 0.0f, 0.0f, 0.0f, 0.0f});
    const float bell[] = {1.0f, 2.76f, 5.40f, 8.93f, 13.34f, 18.64f};
    for (size_t i = 0; i < 6; ++i)
        m.points.push_back({bell[i], 0.8f / static_cast<float>(i + 1), 1.0f, 0.0f, (i % 2 == 0 ? -0.4f : 0.4f), 0.1f});
    m.spread = 0.3f;
    m.sanitise();
    return m;
}

CloudModel CloudModel::random(int count, std::uint32_t seed)
{
    CloudModel m;
    m.name = "Random Cloud";
    std::uint32_t s = seed == 0 ? 1U : seed;
    count = std::clamp(count, 0, kCloudMaxPoints);
    for (int i = 0; i < count; ++i)
    {
        CloudPoint p;
        p.x = 1.0f + std::floor(std::pow(rand01(s), 2.0f) * 63.0f);
        p.y = 0.2f + 0.8f * rand01(s);
        p.z = rand01(s);
        p.phase = rand01(s);
        p.pan = 2.0f * rand01(s) - 1.0f;
        p.random = 0.3f * rand01(s);
        m.points.push_back(p);
    }
    m.sanitise();
    return m;
}

std::vector<std::vector<float>> cloudToCycles(const CloudModel& model)
{
    constexpr int frames = WavetableBank::numFrames;
    constexpr int size = WavetableBank::tableSize;
    std::vector<std::vector<float>> cycles(static_cast<size_t>(frames), std::vector<float>(static_cast<size_t>(size), 0.0f));
    for (int f = 0; f < frames; ++f)
    {
        const float pos = static_cast<float>(f) / static_cast<float>(frames - 1);
        auto& c = cycles[static_cast<size_t>(f)];
        for (const auto& p : model.points)
        {
            const int h = static_cast<int>(std::lround(p.x));
            const float a = model.weight(p, pos);
            if (h < 1 || h >= size / 2 || a < 1.0e-6f)
                continue;
            for (int i = 0; i < size; ++i)
                c[static_cast<size_t>(i)] +=
                    a * static_cast<float>(std::sin(kTwoPi * (static_cast<double>(h) * i / size + p.phase)));
        }
    }
    return cycles;
}

// ---------------------------------------------------------------------------------------------------------------
void CloudOscillator::prepare(double sampleRate)
{
    sr = sampleRate;
    active = false;
}

void CloudOscillator::setModel(const CloudModel& m)
{
    auto copy = std::make_unique<CloudModel>(m);
    copy->sanitise();
    shared.publish(std::move(copy));
}

void CloudOscillator::noteOn(float, std::uint32_t seed)
{
    noteSeed = seed == 0 ? 1U : seed;
    active = true;
    fresh = true;
}

void CloudOscillator::render(float* left, float* right, int numSamples, float position, float frequencyHz)
{
    const CloudModel* m = shared.acquire();
    renderModel(m, left, right, numSamples, position, frequencyHz, 0.0f, 1.0f);
    shared.release();
}

void CloudOscillator::energies(std::vector<float>& out, size_t count) const
{
    out.resize(count);
    for (size_t i = 0; i < count && i < energy.size(); ++i)
        out[i] = energy[i].load(std::memory_order_relaxed);
}

void CloudOscillator::renderModel(const CloudModel* m, float* left, float* right, int numSamples, float position,
                                  float frequencyHz, float spreadOverride, float panScale, const float* bandGains)
{
    if (m == nullptr || !active || numSamples <= 0)
        return;
    const float spreadValue = spreadOverride > 0.0f ? spreadOverride : m->spread;
    position = std::clamp(position, 0.0f, 1.0f);
    lastPosition.store(position, std::memory_order_relaxed);
    const size_t count = std::min(m->points.size(), static_cast<size_t>(kCloudMaxPoints));
    if (fresh || m != lastModel)
    {
        for (size_t i = 0; i < count; ++i)
        {
            const auto& p = m->points[i];
            std::uint32_t s = mix(noteSeed, static_cast<std::uint32_t>(i + 1));
            auto& q = partials[i];
            const double ph = kTwoPi * (p.phase + p.random * rand01(s));
            q.re = std::cos(ph);
            q.im = std::sin(ph);
            q.jitterLevel = 1.0f - p.random * 0.5f * rand01(s);
            q.detune = std::exp2(p.random * (rand01(s) - 0.5f) * 0.2f / 12.0f); // up to +/- 0.1 semitone
        }
        fresh = false;
        lastModel = m;
    }

    // Level per partial at this position; the sum is normalised so the output peak stays at or below 1.
    float sum = 0.0f;
    std::array<float, kCloudMaxPoints> amp{};
    std::array<double, kCloudMaxPoints> stepRe{}, stepIm{};
    const double nyquistGuard = 0.45 * sr;
    for (size_t i = 0; i < count; ++i)
    {
        const auto& p = m->points[i];
        const double f = static_cast<double>(frequencyHz) * p.x * partials[i].detune;
        if (f >= nyquistGuard || f <= 0.0)
        {
            amp[i] = 0.0f;
            continue;
        }
        amp[i] = m->weightWithSpread(p, position, spreadValue) * partials[i].jitterLevel;
        if (bandGains != nullptr)
            amp[i] *= bandGains[cloudBandOf(p.x)];
        const float angle = (std::clamp(p.pan * panScale, -1.0f, 1.0f) + 1.0f) * 0.25f * 3.14159265f;
        partials[i].gainL = std::cos(angle);
        partials[i].gainR = std::sin(angle);
        sum += amp[i];
        const double w = kTwoPi * f / sr;
        stepRe[i] = std::cos(w);
        stepIm[i] = std::sin(w);
    }
    const float norm = 1.0f / std::max(sum, 1.0f);
    for (size_t i = 0; i < count; ++i)
        energy[i].store(amp[i] * norm, std::memory_order_relaxed);
    for (size_t i = count; i < static_cast<size_t>(kCloudMaxPoints); ++i)
        energy[i].store(0.0f, std::memory_order_relaxed);

    for (size_t i = 0; i < count; ++i)
    {
        if (amp[i] <= 0.0f)
            continue;
        auto& q = partials[i];
        const float gl = amp[i] * norm * q.gainL * 1.41421356f;
        const float gr = amp[i] * norm * q.gainR * 1.41421356f;
        double re = q.re, im = q.im;
        const double sr0 = stepRe[i], si0 = stepIm[i];
        for (int n = 0; n < numSamples; ++n)
        {
            const auto s = static_cast<float>(im);
            left[n] += s * gl;
            right[n] += s * gr;
            const double nr = re * sr0 - im * si0;
            im = re * si0 + im * sr0;
            re = nr;
        }
        // Renormalise the rotator so rounding never grows or shrinks it.
        const double mag = std::sqrt(re * re + im * im);
        q.re = re / mag;
        q.im = im / mag;
    }
}

CloudSnapshot CloudOscillator::snapshot() const
{
    CloudSnapshot s;
    if (const auto* m = shared.latest())
        s.points = m->points;
    s.position = lastPosition.load(std::memory_order_relaxed);
    s.energy.resize(s.points.size());
    for (size_t i = 0; i < s.points.size(); ++i)
        s.energy[i] = energy[i].load(std::memory_order_relaxed);
    s.activeNotes = active ? 1 : 0;
    return s;
}

CloudBank::CloudBank()
{
    for (size_t i = 0; i < slots.size(); ++i)
        reset(static_cast<int>(i));
}

void CloudBank::set(int slot, const CloudModel& model)
{
    if (slot < 0 || slot >= static_cast<int>(slots.size()))
        return;
    auto copy = std::make_unique<CloudModel>(model);
    copy->sanitise();
    slots[static_cast<size_t>(slot)].publish(std::move(copy));
    user[static_cast<size_t>(slot)] = true;
}

void CloudBank::reset(int slot)
{
    if (slot < 0 || slot >= static_cast<int>(slots.size()))
        return;
    slots[static_cast<size_t>(slot)].publish(std::make_unique<CloudModel>(CloudModel::harmonicMorph()));
    user[static_cast<size_t>(slot)] = false;
}

bool CloudBank::isUser(int slot) const
{
    return slot >= 0 && slot < static_cast<int>(user.size()) && user[static_cast<size_t>(slot)];
}

std::vector<CloudBank::Record> CloudBank::serialise() const
{
    std::vector<Record> out;
    for (size_t i = 0; i < slots.size(); ++i)
        if (user[i] && slots[i].latest() != nullptr)
            out.push_back({static_cast<int>(i), slots[i].latest()->serialise()});
    return out;
}

void CloudBank::deserialise(const std::vector<Record>& records)
{
    for (size_t i = 0; i < slots.size(); ++i)
        reset(static_cast<int>(i));
    for (const auto& r : records)
    {
        CloudModel m;
        if (CloudModel::deserialise(r.text, m))
            set(r.slot, m);
    }
}

const CloudModel* CloudBank::model(int slot) const
{
    return slot >= 0 && slot < static_cast<int>(slots.size()) ? slots[static_cast<size_t>(slot)].latest() : nullptr;
}

void CloudBank::collectGarbage()
{
    for (auto& s : slots)
        s.collectGarbage();
}

void CloudBank::acquire(Pinned& out) noexcept
{
    for (size_t i = 0; i < slots.size(); ++i)
        out[i] = slots[i].acquire();
}

void CloudBank::release() noexcept
{
    for (auto& s : slots)
        s.release();
}

} // namespace crucible
