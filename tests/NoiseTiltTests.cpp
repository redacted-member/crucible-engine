// SPDX-License-Identifier: MIT
// The noise oscillator's Noise Tilt (oscN_noise_color): brightness rises monotonically from -1 to +1 for white, pink and
// brown noise, and colours at or below 0 render bit-identically to the original low-pass law.

#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Dsp.h>
#include <crucible/module/Oscillator.h>

#include <algorithm>
#include <array>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr double kRate = 48000.0;
constexpr size_t kSamples = 65536;

struct Result
{
    std::vector<float> left;
    std::uint64_t hash = 0;
};

// The original noise law (white, pink, brown with the original tilt low-pass, default width 1, seed 4242), rendered in the
// same 32-sample chunks and hashed exactly like renderNoise.
std::uint64_t referenceOriginalHash(int kind, float tilt)
{
    Rng rng;
    rng.state = (4242U * 3266489917U) | 1U;
    std::array<float, 2> pink0{}, pink1{}, pink2{}, brown{}, lp{};
    const auto sample = [&](size_t c)
    {
        const float white = rng.bipolar();
        switch (kind)
        {
        case 1:
            pink0[c] = 0.99765f * pink0[c] + white * 0.0990460f;
            pink1[c] = 0.96300f * pink1[c] + white * 0.2965164f;
            pink2[c] = 0.57000f * pink2[c] + white * 1.0526913f;
            return (pink0[c] + pink1[c] + pink2[c] + white * 0.1848f) * 0.25f;
        case 2:
            brown[c] = std::clamp(brown[c] * 0.998f + white * 0.04f, -1.0f, 1.0f);
            return brown[c] * 3.0f;
        default:
            return white;
        }
    };
    const float rate = static_cast<float>(kRate);
    const float c = 1.0f - 1.0f;
    const float cross = std::sqrt(std::max(0.0f, 1.0f - c * c));
    const float a = std::abs(tilt);
    const float fc = 20000.0f * std::exp2(-7.0f * a);
    const float coef = 1.0f - std::exp(-6.2831853f * std::min(fc, 0.45f * rate) / rate);
    std::uint64_t hash = 1469598103934665603ULL;
    for (size_t i = 0; i < kSamples; ++i)
    {
        const float nl = sample(0);
        const float ni = sample(1);
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
        l *= 0.5f;
        rr *= 0.5f;
        for (const float v : {l, rr})
        {
            std::uint32_t bits;
            std::memcpy(&bits, &v, 4);
            hash = (hash ^ bits) * 1099511628211ULL;
        }
    }
    return hash;
}

// Renders the noise module (slot 0) directly in chunks of 32 samples at a fixed seed.
Result renderNoise(int type, float color)
{
    const auto& registry = ModuleRegistry::instance();
    const ModuleDescriptor* desc = nullptr;
    for (const auto* d : registry.ofKind(ModuleKind::Oscillator))
        if (d->id == "osc.noise")
            desc = d;
    auto inst = desc->createOscillator(0, registry.parameters());
    inst->prepare(kRate, 1);
    auto values = ParameterMap::defaultValues();
    setValue(values, "osc1_noise_type", static_cast<float>(type));
    setValue(values, "osc1_noise_color", color);
    OscNoteInfo info;
    info.seed = 4242;
    inst->noteOn(info);
    Result res;
    res.hash = 1469598103934665603ULL;
    std::vector<float> l(32), r(32);
    for (size_t done = 0; done < kSamples; done += 32)
    {
        std::fill(l.begin(), l.end(), 0.0f);
        std::fill(r.begin(), r.end(), 0.0f);
        OscRender rr;
        rr.values = values.data();
        rr.pitch = 60.0f;
        rr.sampleRate = kRate;
        rr.numSamples = 32;
        rr.left = l.data();
        rr.right = r.data();
        inst->render(rr);
        for (int i = 0; i < 32; ++i)
        {
            for (const float s : {l[static_cast<size_t>(i)], r[static_cast<size_t>(i)]})
            {
                std::uint32_t bits;
                std::memcpy(&bits, &s, 4);
                res.hash = (res.hash ^ bits) * 1099511628211ULL;
            }
            res.left.push_back(l[static_cast<size_t>(i)]);
        }
    }
    return res;
}

// Energy above 8 kHz minus energy below 250 Hz, dB.
double tiltDb(const std::vector<float>& x)
{
    const auto d = m::toDouble(x);
    return m::bandRmsDbfs(d.data(), d.size(), kRate, 8000.0, 20000.0) - m::bandRmsDbfs(d.data(), d.size(), kRate, 20.0, 250.0);
}

class NoiseTiltTests : public UnitTest
{
public:
    NoiseTiltTests() : UnitTest("Noise tilt", "Oscillator") {}

    void runTest() override
    {
        beginTest("tilt (energy above 8 kHz minus below 250 Hz) rises strictly from colour -1 to +1 for white, pink, brown");
        {
            const char* names[] = {"white", "pink", "brown"};
            for (int type = 0; type < 3; ++type)
            {
                double prev = -1.0e9;
                std::string row, level;
                for (int k = -10; k <= 10; ++k)
                {
                    const double t = tiltDb(renderNoise(type, static_cast<float>(k) / 10.0f).left);
                    row += " " + std::to_string(static_cast<int>(std::lround(t * 10.0)) / 10.0).substr(0, 5);
                    expect(t > prev, std::string(names[type]) + " not rising at colour " + std::to_string(k / 10.0) + ": " +
                                         std::to_string(t) + " after " + std::to_string(prev));
                    prev = t;
                    if (k % 5 == 0)
                        level += " " + std::to_string(m::rmsDbfs(m::toDouble(renderNoise(type, static_cast<float>(k) / 10.0f).left)));
                }
                logMessage(std::string("noise RMS dBFS ") + names[type] + " (-1, -0.5, 0, 0.5, 1):" + level);
                logMessage(std::string("noise tilt dB ") + names[type] + " (-1 .. 1 step 0.1):" + row);
            }
        }

        beginTest("colour 0 and negative colours render bit-identically to the original law");
        {
            // Compares against an in-test copy of the original (pre-fix) white/pink/brown + tilt law, built with the same
            // compiler and flags as the module, so the check holds on every platform (no pinned hashes).
            struct Ref
            {
                int type;
                float color;
            };
            const Ref refs[] = {{0, 0.0f}, {0, -0.3f}, {0, -1.0f}, {1, 0.0f}, {1, -0.5f}, {2, -1.0f}, {2, 0.0f}, {1, -0.005f}};
            for (const auto& ref : refs)
            {
                const auto h = renderNoise(ref.type, ref.color).hash;
                const auto want = referenceOriginalHash(ref.type, ref.color);
                expect(h == want, "bit-identity type " + std::to_string(ref.type) + " colour " + std::to_string(ref.color));
            }
        }
    }
};

static NoiseTiltTests noiseTiltTests;
} // namespace
