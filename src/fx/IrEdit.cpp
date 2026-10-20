// SPDX-License-Identifier: MIT
#include <crucible/fx/IrEdit.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Section
{
    double b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    double z1 = 0, z2 = 0;
    double tick(double x)
    {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
};

Section designSection(bool highPass, double hz, double q, double rate)
{
    const double w = 2.0 * kPi * hz / rate;
    const double c = std::cos(w), s = std::sin(w), alpha = s / (2.0 * q);
    const double a0 = 1.0 + alpha;
    Section o;
    if (highPass)
    {
        o.b0 = (1.0 + c) * 0.5 / a0;
        o.b1 = -(1.0 + c) / a0;
        o.b2 = o.b0;
    }
    else
    {
        o.b0 = (1.0 - c) * 0.5 / a0;
        o.b1 = (1.0 - c) / a0;
        o.b2 = o.b0;
    }
    o.a1 = -2.0 * c / a0;
    o.a2 = (1.0 - alpha) / a0;
    return o;
}

// Fourth-order Butterworth.
void cut(std::vector<float>& x, bool highPass, double hz, double rate)
{
    Section s1 = designSection(highPass, hz, 0.5411961001461970, rate);
    Section s2 = designSection(highPass, hz, 1.3065629648763766, rate);
    for (float& v : x)
        v = static_cast<float>(s2.tick(s1.tick(static_cast<double>(v))));
}

double smoothstep(double t)
{
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

size_t samplesFor(double ms, double rate)
{
    return ms <= 0.0 ? 0 : static_cast<size_t>(std::llround(ms * 1.0e-3 * rate));
}
} // namespace

ImpulseResponse applyIrEdit(const ImpulseResponse& in, const IrEditSettings& e)
{
    ImpulseResponse out;
    out.sampleRate = in.sampleRate;
    out.layout = in.layout;
    const double rate = in.sampleRate;
    const size_t cap = static_cast<size_t>(kMaxEditedImpulseResponseSeconds * rate);
    for (const auto& channel : in.channels)
    {
        std::vector<float> x = channel;
        if (x.empty())
            x.push_back(0.0f);
        // Trim start, length.
        const size_t start = std::min(samplesFor(e.trimMs, rate), x.size() - 1);
        x.erase(x.begin(), x.begin() + static_cast<std::ptrdiff_t>(start));
        const double keep = std::clamp(e.lengthPercent, 0.0, 100.0) / 100.0;
        x.resize(std::max<size_t>(1, static_cast<size_t>(std::llround(keep * static_cast<double>(x.size())))));
        if (e.reverse)
            std::reverse(x.begin(), x.end());
        if (std::abs(e.stretch - 1.0) > 1.0e-6)
            x = resampleKaiser(x, 1.0, std::clamp(e.stretch, 0.1, 10.0));
        if (x.empty())
            x.push_back(0.0f);
        if (x.size() > cap)
            x.resize(cap);
        // Fades.
        const size_t fadeIn = std::min(samplesFor(e.fadeInMs, rate), x.size());
        for (size_t i = 0; i < fadeIn; ++i)
            x[i] *= static_cast<float>(0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(fadeIn)));
        const size_t fadeOut = std::min(samplesFor(e.fadeOutMs, rate), x.size());
        for (size_t i = 0; i < fadeOut; ++i)
            x[x.size() - 1 - i] *=
                static_cast<float>(0.5 - 0.5 * std::cos(kPi * static_cast<double>(i) / static_cast<double>(fadeOut)));
        // Early / late balance.
        if (std::abs(e.balanceDb) > 1.0e-3)
        {
            const double from = 0.060 * rate, to = 0.100 * rate;
            for (size_t i = 0; i < x.size(); ++i)
            {
                const double t = smoothstep((static_cast<double>(i) - from) / (to - from));
                x[i] *= static_cast<float>(std::pow(10.0, e.balanceDb * (t - 0.5) / 20.0));
            }
        }
        // Cuts.
        if (e.lowCutHz > 20.5)
            cut(x, true, std::min(e.lowCutHz, 0.45 * rate), rate);
        if (e.highCutHz < 19999.5 && e.highCutHz < 0.49 * rate)
            cut(x, false, std::max(e.highCutHz, 10.0), rate);
        // Predelay.
        const size_t pre = std::min(samplesFor(e.predelayMs, rate), cap - std::min(cap, x.size()));
        if (pre > 0)
            x.insert(x.begin(), pre, 0.0f);
        out.channels.push_back(std::move(x));
    }
    // Width: the side level of every left / right pair (Stereo: 0/1; TrueStereo: 0/1 and 2/3).
    if (std::abs(e.width - 1.0) > 1.0e-6 && out.layout != IrLayout::Mono)
    {
        const float w = static_cast<float>(e.width);
        for (size_t p = 0; p + 1 < out.channels.size(); p += 2)
        {
            auto& a = out.channels[p];
            auto& b = out.channels[p + 1];
            for (size_t i = 0; i < a.size() && i < b.size(); ++i)
            {
                const float mid = 0.5f * (a[i] + b[i]), side = 0.5f * (a[i] - b[i]) * w;
                a[i] = mid + side;
                b[i] = mid - side;
            }
        }
    }
    return out;
}

} // namespace crucible
