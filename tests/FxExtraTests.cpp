// SPDX-License-Identifier: MIT
#include <crucible/Fx.h>
#include <crucible/FxExtra.h>
#include "TestSupport.h"

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kPi = 3.14159265358979323846;

std::vector<float> sine(double hz, float amp, int n)
{
    std::vector<float> v(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
        v[static_cast<size_t>(i)] = amp * static_cast<float>(std::sin(2.0 * kPi * hz * i / kRate));
    return v;
}

// Amplitude of the component at hz (Goertzel over the range), in dB re 1.0 peak.
double toneDb(const std::vector<float>& x, double hz, size_t from = 0, size_t to = 0)
{
    if (to == 0)
        to = x.size();
    const double w = 2.0 * kPi * hz / kRate;
    double re = 0.0, im = 0.0;
    for (size_t i = from; i < to; ++i)
    {
        // Hann window
        const double win =
            0.5 - 0.5 * std::cos(2.0 * kPi * static_cast<double>(i - from) / static_cast<double>(to - from - 1));
        re += x[i] * win * std::cos(w * static_cast<double>(i));
        im -= x[i] * win * std::sin(w * static_cast<double>(i));
    }
    const double amp = 2.0 * std::sqrt(re * re + im * im) / (0.5 * static_cast<double>(to - from));
    return 20.0 * std::log10(std::max(amp, 1.0e-12));
}

float rms(const std::vector<float>& v, size_t from = 0, size_t to = 0)
{
    if (to == 0)
        to = v.size();
    double s = 0.0;
    for (size_t i = from; i < to; ++i)
        s += static_cast<double>(v[i]) * v[i];
    return static_cast<float>(std::sqrt(s / static_cast<double>(std::max<size_t>(to - from, 1))));
}

template <typename Fx, typename Params>
void runFx(Fx& fx, std::vector<float>& l, std::vector<float>& r, const Params& p, int block = 64)
{
    const int n = static_cast<int>(l.size());
    for (int pos = 0; pos < n; pos += block)
        fx.process(l.data() + pos, r.data() + pos, std::min(block, n - pos), p);
}

bool finite(const std::vector<float>& v)
{
    for (const float x : v)
        if (!std::isfinite(x))
            return false;
    return true;
}
} // namespace

class FxExtraTests : public UnitTest
{
public:
    FxExtraTests() : UnitTest("FX second generation", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("filter FX: lowpass attenuates, the LFO sweeps, every model is finite");
        {
            FxFilterParams p;
            p.on = true;
            p.model = static_cast<int>(FilterModel::Lowpass24);
            p.cutoffHz = 500.0f;
            p.resonance = 0.1f;
            FxFilter f;
            f.prepare(kRate);
            auto l = sine(5000.0, 0.5f, 24000), r = l;
            runFx(f, l, r, p);
            expect(rms(l, 12000) < 0.5f * 0.7071f * 0.05f, "5 kHz through a 500 Hz LP24 " + String(rms(l, 12000)));

            p.lfoDepth = 3.0f;
            p.lfoRateHz = 2.0f;
            FxFilter g;
            g.prepare(kRate);
            auto a = sine(2000.0, 0.5f, 48000), b = a;
            runFx(g, a, b, p);
            float lo = 1.0f, hi = 0.0f;
            for (size_t w = 0; w < 48000; w += 2400)
            {
                const float level = rms(a, w, w + 2400);
                lo = std::min(lo, level);
                hi = std::max(hi, level);
            }
            expect(hi > lo * 3.0f, "LFO sweep changes the level " + String(lo) + " .. " + String(hi));

            for (int m = 0; m < static_cast<int>(FilterModel::Count); ++m)
            {
                FxFilterParams q;
                q.on = true;
                q.model = m;
                q.resonance = 1.0f;
                q.drive = 1.0f;
                q.lfoDepth = 4.0f;
                q.lfoRateHz = 10.0f;
                FxFilter h;
                h.prepare(kRate);
                std::vector<float> x(9600), y(9600);
                Rng rng;
                for (size_t i = 0; i < x.size(); ++i)
                    x[i] = y[i] = 0.5f * rng.bipolar();
                runFx(h, x, y, q, 7);
                float peak = 0.0f;
                for (const float v : x)
                    peak = std::max(peak, std::abs(v));
                expect(finite(x) && peak < 20.0f, "model " + String(m) + " peak " + String(peak));
            }
        }

        beginTest("multiband compressor: compresses the loud band only and reports gain reduction");
        {
            FxMultibandParams p;
            p.on = true;
            p.ratio = 10.0f;
            p.thresholdDb = {-30.0f, -30.0f, -30.0f};
            FxMultiband mb;
            mb.prepare(kRate);
            auto lowTone = sine(80.0, 0.5f, 48000);
            auto highTone = sine(8000.0, 0.01f, 48000); // -40 dB: below threshold
            std::vector<float> l(48000), r(48000);
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = r[i] = lowTone[i] + highTone[i];
            const auto dry = l;
            runFx(mb, l, r, p);
            const double lowChange = toneDb(l, 80.0, 24000) - toneDb(dry, 80.0, 24000);
            const double highChange = toneDb(l, 8000.0, 24000) - toneDb(dry, 8000.0, 24000);
            expect(lowChange < -10.0, "low band reduced " + String(lowChange));
            expect(std::abs(highChange) < 1.0, "high band untouched " + String(highChange));
            expect(mb.gainReductionDb(0) > 10.0f && mb.gainReductionDb(2) < 0.5f);

            // At ratio 1 the crossover sums back to the input level.
            FxMultibandParams flat;
            flat.on = true;
            flat.ratio = 1.0f;
            FxMultiband m2;
            m2.prepare(kRate);
            for (const double hz : {60.0, 200.0, 1000.0, 3000.0, 10000.0})
            {
                auto a = sine(hz, 0.3f, 24000), b = a;
                m2.reset();
                runFx(m2, a, b, flat);
                expectWithinAbsoluteError(toneDb(a, hz, 12000), toneDb(sine(hz, 0.3f, 24000), hz, 12000), 0.5,
                                          "LR4 sum at " + String(hz));
            }
        }

        beginTest("trance gate: pattern steps follow the tempo; depth 0 and mix 0 are neutral");
        {
            FxTranceGateParams p;
            p.on = true;
            p.pattern = 0x5555U; // steps 0, 2, 4, ... on
            p.steps = 16;
            p.stepBeats = 0.25f;
            p.bpm = 120.0; // 16th = 6000 samples at 48 kHz
            p.releaseMs = 2.0f;
            FxTranceGate g;
            g.prepare(kRate);
            std::vector<float> l(48000, 0.5f), r(48000, 0.5f);
            runFx(g, l, r, p);
            for (int s = 0; s < 8; ++s)
            {
                const float level = rms(l, static_cast<size_t>(s * 6000 + 1000), static_cast<size_t>(s * 6000 + 5000));
                if (s % 2 == 0)
                    expect(level > 0.45f, "step " + String(s) + " open " + String(level));
                else
                    expect(level < 0.01f, "step " + String(s) + " closed " + String(level));
            }
            p.depth = 0.0f;
            FxTranceGate h;
            h.prepare(kRate);
            std::vector<float> a(9600, 0.5f), b(9600, 0.5f);
            runFx(h, a, b, p);
            expectWithinAbsoluteError(rms(a), 0.5f, 1.0e-4f);
        }

        beginTest("tremolo: depth sets the modulation, 180 degrees pans, depth 0 is neutral");
        {
            FxTremoloParams p;
            p.on = true;
            p.rateHz = 5.0f;
            p.depth = 1.0f;
            p.shape = 0;
            FxTremolo t;
            t.prepare(kRate);
            std::vector<float> l(48000, 0.5f), r(48000, 0.5f);
            runFx(t, l, r, p);
            float lo = 1.0f, hi = 0.0f;
            for (size_t i = 4800; i < l.size(); ++i)
            {
                lo = std::min(lo, l[i]);
                hi = std::max(hi, l[i]);
            }
            expect(lo < 0.02f && hi > 0.48f, "full-depth sine tremolo " + String(lo) + " .. " + String(hi));
            p.stereoPhaseDeg = 180.0f;
            FxTremolo pan;
            pan.prepare(kRate);
            std::vector<float> a(48000, 0.5f), b(48000, 0.5f);
            runFx(pan, a, b, p);
            float sumDev = 0.0f;
            for (size_t i = 4800; i < a.size(); ++i)
                sumDev = std::max(sumDev, std::abs(a[i] + b[i] - 0.5f));
            expect(sumDev < 0.02f, "auto-pan keeps L + R constant " + String(sumDev));
            p.depth = 0.0f;
            FxTremolo none;
            none.prepare(kRate);
            std::vector<float> c(4800, 0.5f), d(4800, 0.5f);
            runFx(none, c, d, p);
            expectWithinAbsoluteError(rms(c), 0.5f, 1.0e-5f);
        }

        beginTest("ring modulator makes sum and difference; frequency shifter moves one sideband");
        {
            FxRingShiftParams p;
            p.on = true;
            p.mix = 1.0f;
            p.mode = 0;
            p.freqHz = 200.0f;
            FxRingShift ring;
            ring.prepare(kRate);
            auto l = sine(1000.0, 0.5f, 48000), r = l;
            runFx(ring, l, r, p);
            expect(toneDb(l, 800.0, 4800) > -15.0 && toneDb(l, 1200.0, 4800) > -15.0);
            expect(toneDb(l, 1000.0, 4800) < -50.0, "carrier suppressed");

            p.mode = 1;
            p.freqHz = 150.0f;
            FxRingShift shift;
            shift.prepare(kRate);
            auto a = sine(1000.0, 0.5f, 48000), b = a;
            runFx(shift, a, b, p);
            const double up = toneDb(a, 1150.0, 4800);
            const double image = toneDb(a, 850.0, 4800);
            expect(up > -10.0 && up - image > 30.0, "shift up: wanted " + String(up) + " image " + String(image));
            p.mode = 2;
            FxRingShift down;
            down.prepare(kRate);
            auto c = sine(1000.0, 0.5f, 48000), d = c;
            runFx(down, c, d, p);
            expect(toneDb(c, 850.0, 4800) - toneDb(c, 1150.0, 4800) > 30.0, "shift down");
        }

    }
};

static FxExtraTests fxExtraTests;
