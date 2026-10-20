// SPDX-License-Identifier: MIT
// The attack curve in the analog RC envelope mode: it bends the RC attack (negative starts faster, positive slower), the
// peak is still reached at the attack time, and curve 0 follows the plain RC recursion bit for bit.

#include "TestFramework.h"

#include <crucible/AnalogEnvelope.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <vector>

using namespace crucible;
using namespace crucible::test;

namespace
{
constexpr double kFs = 48000.0;
constexpr float kAttack = 0.05f;
constexpr std::uint64_t kCurveZeroHash = 15940283339599501923ULL;

AnalogEnvelope makeRc(float curve)
{
    AdsrParams p;
    p.attack = kAttack;
    p.decay = 0.3f;
    p.sustain = 0.8f;
    p.release = 0.25f;
    p.attackCurve = curve;
    EnvShape s;
    s.mode = EnvMode::AnalogRc;
    AnalogEnvelope e;
    e.prepare(kFs);
    e.setParams(p, s, 1.0f, 0.0f);
    e.noteOn(1.0f, 60);
    return e;
}

std::vector<float> attackRun(float curve, int n)
{
    auto e = makeRc(curve);
    std::vector<float> v(static_cast<size_t>(n));
    for (auto& x : v)
        x = e.tick();
    return v;
}

size_t samplesAt(double ms) { return static_cast<size_t>(ms * 0.001 * kFs) - 1; }

size_t peakIndex(const std::vector<float>& v)
{
    for (size_t i = 0; i < v.size(); ++i)
        if (v[i] >= 1.0f)
            return i;
    return v.size();
}
} // namespace

class RcAttackCurveTests : public UnitTest
{
public:
    RcAttackCurveTests() : UnitTest("RC attack curve", "Envelope") {}

    void runTest() override
    {
        const int n = static_cast<int>(0.2 * kFs);
        const float curves[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

        beginTest("curve 0 follows the plain RC recursion and matches the hash recorded before the curve applied in RC mode");
        {
            const double overshoot = 0.35;
            const double target = 1.0 + overshoot;
            const double coef = std::exp(-std::log((1.0 + overshoot) / overshoot) / (static_cast<double>(kAttack) * kFs));
            const auto v = attackRun(0.0f, n);
            double level = 0.0;
            double worst = 0.0;
            for (size_t i = 0; i < v.size() && level < 1.0; ++i)
            {
                level = std::min(1.0, target - (target - level) * coef);
                worst = std::max(worst, std::abs(static_cast<double>(v[i]) - level));
            }
            expect(worst < 1.0e-6, "recursion, worst difference " + std::to_string(worst));
            std::uint64_t h = 1469598103934665603ULL;
            for (float x : v)
            {
                std::uint32_t bits;
                std::memcpy(&bits, &x, 4);
                h = (h ^ bits) * 1099511628211ULL;
            }
            logMessage("curve 0 hash " + std::to_string(h));
            expect(h == kCurveZeroHash, "bit-identical to the pre-change output");
        }

        beginTest("the curve bends the RC attack: more negative is faster at 5, 10 and 15 ms");
        {
            std::vector<std::vector<float>> runs;
            for (float c : curves)
                runs.push_back(attackRun(c, n));
            for (double ms : {5.0, 10.0, 15.0})
            {
                std::string row;
                bool mono = true;
                for (size_t k = 0; k < runs.size(); ++k)
                {
                    const float x = runs[k][samplesAt(ms)];
                    row += " " + std::to_string(x).substr(0, 6);
                    if (k > 0)
                        mono = mono && x < runs[k - 1][samplesAt(ms)];
                }
                logMessage("levels at " + std::to_string(static_cast<int>(ms)) + " ms (curve -1 -0.5 0 0.5 1):" + row);
                expect(mono, "strictly falling with the curve at " + std::to_string(ms) + " ms");
            }
        }

        beginTest("the peak is still reached at the attack time, and the attack is monotonic");
        {
            const size_t ref = peakIndex(attackRun(0.0f, n));
            for (float c : curves)
            {
                const auto v = attackRun(c, n);
                const size_t pk = peakIndex(v);
                expect(pk + 2 >= ref && pk <= ref + 2, "peak sample for curve " + std::to_string(c));
                bool mono = true;
                for (size_t i = 1; i <= pk && i < v.size(); ++i)
                    mono = mono && v[i] >= v[i - 1];
                expect(mono, "monotonic for curve " + std::to_string(c));
            }
        }

        beginTest("a retrigger from a level continues from it without a jump");
        {
            for (float c : {-0.7f, 0.7f})
            {
                auto e = makeRc(c);
                for (int i = 0; i < 600; ++i)
                    e.tick();
                const float before = e.value();
                e.noteOn(1.0f, 60);
                const float after = e.tick();
                expect(after >= before && after - before < 0.02f, "continuous for curve " + std::to_string(c));
            }
        }

        beginTest("changing the curve mid-attack makes no step in the rendered output");
        {
            const float pairs[][2] = {{0.0f, 0.7f}, {0.0f, -0.7f}, {-0.7f, 0.7f}, {0.7f, -0.7f}, {0.7f, 0.0f}, {-0.3f, -1.0f}};
            for (const auto& pr : pairs)
            {
                auto e = makeRc(pr[0]);
                AdsrParams p;
                p.attack = kAttack;
                p.decay = 0.3f;
                p.sustain = 0.8f;
                p.release = 0.25f;
                EnvShape s;
                s.mode = EnvMode::AnalogRc;
                float prev = 0.0f;
                float worst = 0.0f;
                for (int i = 0; i < 1200; ++i)
                {
                    if (i == 600)
                    {
                        p.attackCurve = pr[1];
                        e.setParams(p, s, 1.0f, 0.0f);
                    }
                    const float x = e.tick();
                    worst = std::max(worst, std::abs(x - prev));
                    prev = x;
                }
                expect(worst < 0.01f, "curve " + std::to_string(pr[0]) + " -> " + std::to_string(pr[1]) +
                                          ", largest step " + std::to_string(worst));
            }
            // A sweeping curve while the note attacks.
            auto e = makeRc(0.0f);
            AdsrParams p;
            p.attack = 0.3f;
            p.decay = 0.3f;
            p.sustain = 0.8f;
            p.release = 0.25f;
            EnvShape s;
            s.mode = EnvMode::AnalogRc;
            float prev = 0.0f;
            float worst = 0.0f;
            for (int i = 0; i < 6000; ++i)
            {
                p.attackCurve = std::sin(static_cast<float>(i) * 0.01f);
                e.setParams(p, s, 1.0f, 0.0f);
                const float x = e.tick();
                worst = std::max(worst, std::abs(x - prev));
                prev = x;
            }
            expect(worst < 0.01f, "swept curve, largest step " + std::to_string(worst));
        }
    }
};

static RcAttackCurveTests rcAttackCurveTests;
