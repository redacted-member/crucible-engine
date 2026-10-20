// SPDX-License-Identifier: MIT
#include <crucible/KeyDetect.h>

#include <crucible/MidiFile.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace crucible
{
namespace
{
constexpr std::array<double, 12> kMajor = {6.35, 2.23, 3.48, 2.33, 4.38, 4.09, 2.52, 5.19, 2.39, 3.66, 2.29, 2.88};
constexpr std::array<double, 12> kMinor = {6.33, 2.68, 3.52, 5.38, 2.60, 3.53, 2.54, 4.75, 3.98, 2.69, 3.34, 3.17};
const char* kNames[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

double pearson(const std::array<double, 12>& a, const std::array<double, 12>& b)
{
    double ma = 0, mb = 0;
    for (int i = 0; i < 12; ++i)
    {
        ma += a[static_cast<size_t>(i)];
        mb += b[static_cast<size_t>(i)];
    }
    ma /= 12;
    mb /= 12;
    double num = 0, da = 0, db = 0;
    for (int i = 0; i < 12; ++i)
    {
        const double x = a[static_cast<size_t>(i)] - ma, y = b[static_cast<size_t>(i)] - mb;
        num += x * y;
        da += x * x;
        db += y * y;
    }
    return da <= 1.0e-12 || db <= 1.0e-12 ? 0.0 : num / std::sqrt(da * db);
}
} // namespace

void KeyDetector::noteOn(int note, float velocity01) noexcept
{
    if (note < 0 || note > 127)
        return;
    const float w = std::clamp(velocity01, 0.05f, 1.0f);
    auto& h = histogram[static_cast<size_t>(note % 12)];
    h.store(h.load(std::memory_order_relaxed) + w, std::memory_order_relaxed);
    held[static_cast<size_t>(note)].store(w, std::memory_order_relaxed);
}

void KeyDetector::noteOff(int note) noexcept
{
    if (note >= 0 && note < 128)
        held[static_cast<size_t>(note)].store(0.0f, std::memory_order_relaxed);
}

void KeyDetector::tick(float seconds) noexcept
{
    for (int n = 0; n < 128; ++n)
    {
        const float w = held[static_cast<size_t>(n)].load(std::memory_order_relaxed);
        if (w > 0.0f)
        {
            auto& h = histogram[static_cast<size_t>(n % 12)];
            h.store(h.load(std::memory_order_relaxed) + w * seconds, std::memory_order_relaxed);
        }
    }
}

void KeyDetector::reset() noexcept
{
    for (auto& h : histogram)
        h.store(0.0f, std::memory_order_relaxed);
    for (auto& h : held)
        h.store(0.0f, std::memory_order_relaxed);
}

std::vector<KeyProposal> KeyDetector::analyse(const std::array<float, 12>& histogram)
{
    std::array<double, 12> h{};
    double total = 0;
    for (int i = 0; i < 12; ++i)
    {
        h[static_cast<size_t>(i)] = histogram[static_cast<size_t>(i)];
        total += h[static_cast<size_t>(i)];
    }
    std::vector<KeyProposal> out;
    for (int root = 0; root < 12; ++root)
        for (int minor = 0; minor < 2; ++minor)
        {
            const auto& profile = minor != 0 ? kMinor : kMajor;
            std::array<double, 12> rotated{};
            for (int i = 0; i < 12; ++i)
                rotated[static_cast<size_t>(i)] = profile[static_cast<size_t>((i - root + 12) % 12)];
            KeyProposal p;
            p.root = root;
            p.minor = minor != 0;
            p.scale = minor != 0 ? 2 : 1;
            p.correlation = static_cast<float>(pearson(h, rotated));
            p.evidence = static_cast<float>(total);
            p.name = std::string(kNames[root]) + (minor != 0 ? " minor" : " major");
            out.push_back(p);
        }
    std::stable_sort(out.begin(), out.end(),
                     [](const KeyProposal& a, const KeyProposal& b) { return a.correlation > b.correlation; });
    // Confidence: margin over the best key that is not the relative of the winner, scaled by the evidence.
    if (!out.empty())
    {
        const auto& best = out.front();
        const int relative = best.minor ? (best.root + 3) % 12 : (best.root + 9) % 12;
        float rival = -1.0f;
        for (size_t i = 1; i < out.size(); ++i)
            if (!(out[i].root == relative && out[i].minor != best.minor))
            {
                rival = out[i].correlation;
                break;
            }
        const float margin = std::clamp((best.correlation - rival) / 0.25f, 0.0f, 1.0f);
        const float evidence = static_cast<float>(1.0 - std::exp(-total / 4.0));
        const float fit = std::clamp((best.correlation - 0.2f) / 0.6f, 0.0f, 1.0f);
        out.front().confidence = total <= 0.0 ? 0.0f : margin * evidence * fit;
        for (size_t i = 1; i < out.size(); ++i)
            out[i].confidence = 0.0f;
    }
    return out;
}

std::vector<KeyProposal> KeyDetector::candidates() const
{
    std::array<float, 12> h{};
    for (size_t i = 0; i < 12; ++i)
        h[i] = histogram[i].load(std::memory_order_relaxed);
    return analyse(h);
}

KeyProposal KeyDetector::propose() const
{
    const auto c = candidates();
    return c.empty() ? KeyProposal{} : c.front();
}

void KeyDetector::addMidiFile(const std::vector<TimedMidi>& events)
{
    std::array<double, 128> start{};
    std::array<float, 128> vel{};
    start.fill(-1.0);
    for (const auto& e : events)
    {
        const int type = e.status & 0xf0;
        const int n = e.data1 & 0x7f;
        if (type == 0x90 && e.data2 > 0)
        {
            if (start[static_cast<size_t>(n)] >= 0.0)
                continue;
            start[static_cast<size_t>(n)] = e.seconds;
            vel[static_cast<size_t>(n)] = static_cast<float>(e.data2) / 127.0f;
        }
        else if (type == 0x80 || type == 0x90)
        {
            if (start[static_cast<size_t>(n)] < 0.0)
                continue;
            const float duration = static_cast<float>(std::max(0.0, e.seconds - start[static_cast<size_t>(n)]));
            auto& h = histogram[static_cast<size_t>(n % 12)];
            h.store(h.load(std::memory_order_relaxed) + vel[static_cast<size_t>(n)] * (0.25f + std::min(duration, 4.0f)),
                    std::memory_order_relaxed);
            start[static_cast<size_t>(n)] = -1.0;
        }
    }
}

} // namespace crucible
