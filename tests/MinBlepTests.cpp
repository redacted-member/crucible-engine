// SPDX-License-Identifier: MIT
#include <crucible/MinBlep.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

class MinBlepTests : public UnitTest
{
public:
    MinBlepTests() : UnitTest("MinBLEP", "Engine") {}

    void runTest() override
    {
        beginTest("the correction starts at -1 (a step of one unit becomes zero at the discontinuity) and settles to 0");
        {
            const auto& m = MinBlep::instance();
            expectWithinAbsoluteError(m.at(0.0f), -1.0f, 1.0e-4f);
            expect(std::abs(m.at(40.0f)) < 0.002f);
            expectWithinAbsoluteError(m.at(static_cast<float>(MinBlep::length)), 0.0f, 1.0e-6f);
            // Causal: the correction never acts before the discontinuity (the table starts at the event).
            float worst = 0.0f;
            for (float x = 0.0f; x < 16.0f; x += 0.05f)
                worst = std::max(worst, std::abs(m.at(x)));
            expect(worst <= 1.05f);
        }

        beginTest("a naive sawtooth with minBLEP steps is far cleaner than the naive one");
        {
            const double f0 = 1234.5;
            const int n = (1 << 14) + 200;
            std::vector<float> naive(static_cast<size_t>(n)), blep(static_cast<size_t>(n));
            MinBlepBuffer buffer;
            const double inc = f0 / kRate;
            double phase = 0.0;
            for (int i = 0; i < n; ++i)
            {
                float v = static_cast<float>(2.0 * phase - 1.0);
                naive[static_cast<size_t>(i)] = v;
                blep[static_cast<size_t>(i)] = v + buffer.tick();
                phase += inc;
                if (phase >= 1.0)
                {
                    phase -= 1.0;
                    // The wrap happened `phase / inc` samples before the next sample's time.
                    buffer.addStep(-2.0f, static_cast<float>(phase / inc));
                }
            }
            const double a = aliasingBelowDb(naive, f0, kRate, 20000.0, 100);
            const double b = aliasingBelowDb(blep, f0, kRate, 20000.0, 100);
            std::printf("    aliasing below 20 kHz: naive saw %.1f dB, minBLEP saw %.1f dB\n", a, b);
            expect(b < a - 40.0);
            expect(b < -85.0);
        }

        beginTest("minBLAMP: a naive triangle with corrected slope breaks aliases less");
        {
            const double f0 = 2345.6;
            const int n = (1 << 14) + 200;
            std::vector<float> naive(static_cast<size_t>(n)), fixedUp(static_cast<size_t>(n));
            MinBlepBuffer buffer;
            const double inc = f0 / kRate;
            double phase = 0.0;
            const auto tri = [](double p) { return static_cast<float>(p < 0.5 ? 4.0 * p - 1.0 : 3.0 - 4.0 * p); };
            for (int i = 0; i < n; ++i)
            {
                const float v = tri(phase);
                naive[static_cast<size_t>(i)] = v;
                fixedUp[static_cast<size_t>(i)] = v + buffer.tick();
                const double before = phase;
                phase += inc;
                // Slope breaks at phase 0.5 (down) and at the wrap (up), in value per sample.
                if (before < 0.5 && phase >= 0.5)
                    buffer.addKink(static_cast<float>(-8.0 * inc), static_cast<float>((phase - 0.5) / inc));
                if (phase >= 1.0)
                {
                    phase -= 1.0;
                    buffer.addKink(static_cast<float>(8.0 * inc), static_cast<float>(phase / inc));
                }
            }
            const double a = aliasingBelowDb(naive, f0, kRate, 20000.0, 100);
            const double b = aliasingBelowDb(fixedUp, f0, kRate, 20000.0, 100);
            std::printf("    triangle aliasing below 20 kHz: naive %.1f dB, minBLAMP %.1f dB\n", a, b);
            expect(b < a - 15.0);
        }
    }
};

static MinBlepTests minBlepTests;
