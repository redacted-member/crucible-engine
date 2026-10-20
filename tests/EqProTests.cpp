// SPDX-License-Identifier: MIT
// Analog-matched filters, the linear-phase engine and the pro EQ module (FX5, docs/plans/ENGINE-FX-PLAN.md sections 3.3 and
// 4.2): magnitude against the analog prototype up to 20 kHz at 44.1 / 48 / 96 / 192 kHz, phase modes, channel modes, dynamic
// bands, automation without zipper, latency, allocations and CPU. Every claim is a number; the printed lines are what
// docs/MEASUREMENTS.md quotes.

#include "AllocCounter.h"
#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/LinearPhase.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ModuleRack.h>

#include <chrono>
#include <random>
#include <thread>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG) ||                                    \
    (defined(__x86_64__) && defined(__APPLE__))
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#include <crucible/Simd.h>
#if !CRUCIBLE_SIMD_NATIVE
#define CRUCIBLE_TEST_UNTIMED 1
#endif

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;
using crucible::testing::kRate;
namespace cfx = crucible::fx;

namespace
{
constexpr double kTau = 6.28318530717958647692;

// Error (dB, largest absolute) of a designed band against its analog prototype from 20 Hz to 20 kHz.
double bandError(const cfx::BandSpec& spec, double rate, cfx::DesignQuality quality = cfx::DesignQuality::Minimum,
                 double* phaseDegrees = nullptr)
{
    const auto proto = cfx::bandPrototype(spec);
    const auto filter = cfx::designBand(spec, proto, rate, quality);
    double worst = 0.0, worstPhase = 0.0;
    const double top = std::min(20000.0, 0.499 * rate);
    for (int i = 0; i <= 600; ++i)
    {
        const double hz = 20.0 * std::pow(top / 20.0, i / 600.0);
        const double a = cfx::bandAnalogMagnitudeDb(proto, spec, hz);
        if (a < -60.0)
            continue;
        const double omega = kTau * hz / rate;
        worst = std::max(worst, std::abs(a - filter.magnitudeDb(omega)));
        if (phaseDegrees != nullptr && hz <= 16000.0)
        {
            const auto ratio = filter.response(omega) / cfx::bandAnalogResponse(proto, spec, hz);
            worstPhase = std::max(worstPhase, std::abs(std::arg(ratio)) * 180.0 / 3.14159265358979);
        }
    }
    if (phaseDegrees != nullptr)
        *phaseDegrees = worstPhase;
    return worst;
}

// The legacy design: pre-warped bilinear transform (what the RBJ cookbook biquad is).
double bilinearError(const cfx::BandSpec& spec, double rate)
{
    const auto proto = cfx::bandPrototype(spec);
    const cfx::Biquad s = cfx::bilinearSection(proto.part[0].section, spec.freqHz, rate);
    double worst = 0.0;
    for (int i = 0; i <= 600; ++i)
    {
        const double hz = 20.0 * std::pow(1000.0, i / 600.0);
        worst = std::max(
            worst, std::abs(cfx::bandAnalogMagnitudeDb(proto, spec, hz) - cfx::biquadMagnitudeDb(s, kTau * hz / rate)));
    }
    return worst;
}

// A module through the rack with every other effect off.
struct EqHarness
{
    ModuleRack rack;
    std::vector<float> values;
    std::vector<float> legacy;
    explicit EqHarness(double rate = kRate, Quality quality = Quality::Standard) : rack(ModuleRegistry::instance())
    {
        ModuleContext ctx;
        ctx.sampleRate = rate;
        ctx.quality = quality;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set("fx_ep_on", 1.0f);
    }
    void set(const std::string& id, float v)
    {
        const int i = ParameterMap::indexOf(id);
        values[static_cast<size_t>(i)] = v;
    }
    void band(int b, const char* type, double hz, double gain, double q, int channel = 0)
    {
        const std::string n = "fx_ep_b" + std::to_string(b) + "_";
        set(n + "on", 1.0f);
        set(n + "type", static_cast<float>(typeIndex(type)));
        set(n + "freq", static_cast<float>(hz));
        set(n + "gain", static_cast<float>(gain));
        set(n + "q", static_cast<float>(q));
        set(n + "ch", static_cast<float>(channel));
    }
    static int typeIndex(const std::string& t)
    {
        const char* names[] = {"Bell", "LowShelf", "HighShelf", "Tilt", "BandPass", "Notch", "AllPass", "LowCut", "HighCut"};
        for (int i = 0; i < 9; ++i)
            if (t == names[i])
                return i;
        return 0;
    }
    void run(float* l, float* r, int n, const float* auxL = nullptr, const float* auxR = nullptr)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, auxL, auxR);
    }
    // Whole signal in blocks of 32 (what the engine does); returns the left or right output.
    std::pair<Signal, Signal> runStereo(const Signal& inL, const Signal& inR, const Signal* aux = nullptr)
    {
        std::vector<float> l(inL.begin(), inL.end()), r(inR.begin(), inR.end()), a;
        if (aux != nullptr)
            a.assign(aux->begin(), aux->end());
        for (size_t pos = 0; pos < l.size(); pos += 32)
        {
            const int n = static_cast<int>(std::min<size_t>(32, l.size() - pos));
            run(l.data() + pos, r.data() + pos, n, aux != nullptr ? a.data() + pos : nullptr,
                aux != nullptr ? a.data() + pos : nullptr);
        }
        return {Signal(l.begin(), l.end()), Signal(r.begin(), r.end())};
    }
    Signal runMono(const Signal& in, const Signal* aux = nullptr) { return runStereo(in, in, aux).first; }
    LatencyReporter* latency() { return rack.findCapability<LatencyReporter>("fx.eq_pro"); }
};

Signal impulse(size_t n, size_t at = 0)
{
    Signal x(n, 0.0);
    x[at] = 1.0;
    return x;
}

// Impulse response of the left output once the module has settled (a mode switch needs a moment): `length` samples from the
// impulse, which arrives after `settle` samples of silence.
Signal settledImpulseResponse(EqHarness& h, size_t length, size_t settle = 24000)
{
    const Signal y = h.runMono(impulse(settle + length, settle));
    return Signal(y.begin() + static_cast<long>(settle), y.end());
}

// Magnitude (dB) of an impulse response at hz; the response must have decayed inside the window.
double magnitudeAt(const Signal& ir, double hz, double rate)
{
    const Signal rect(ir.size(), 1.0);
    return amplitudeToDb(std::abs(windowedDtft(ir.data(), rect.data(), ir.size(), hz, rate)));
}

double bandMagnitude(const cfx::BandFilter& f, double hz, double rate)
{
    return f.magnitudeDb(kTau * hz / rate);
}
} // namespace

class EqProTests : public UnitTest
{
public:
    EqProTests() : UnitTest("EqPro", "Effects") {}

    void runTest() override
    {
        matchedDesigns();
        otherShapes();
        phaseOfDesigns();
        moduleMatchesDesign();
        channels();
        phaseModes();
        linearKernel();
        automation();
        dynamicBands();
        latency();
        realTimeLinear();
        resolutionChange();
        requestSlot();
        views();
        realTime();
        extremeBands();
    }

private:
    // -----------------------------------------------------------------------------------------------------------
    void matchedDesigns()
    {
        beginTest("bells and shelves: magnitude against the analog prototype, 20 Hz to 20 kHz, centres 20 Hz to 18 kHz");
        {
            struct Row
            {
                double rate, tolerance;
            };
            // Tolerances sit just above what the biquad can do (a second-order section has five coefficients to follow a
            // curve that the digital response has to leave flat at Nyquist); the plan's 0.1 dB holds from 96 kHz up.
            const Row rows[] = {{44100.0, 0.40}, {48000.0, 0.30}, {96000.0, 0.10}, {192000.0, 0.05}};
            for (const Row& row : rows)
            {
                double worst12 = 0.0, worst24 = 0.0, worstRbj = 0.0;
                for (const auto type : {cfx::BandType::Bell, cfx::BandType::LowShelf, cfx::BandType::HighShelf})
                    for (const double fc : {20.0, 100.0, 1000.0, 5000.0, 10000.0, 14000.0, 16000.0, 18000.0})
                        for (const double gain : {-24.0, -12.0, -6.0, -3.0, 3.0, 6.0, 12.0, 24.0})
                            for (const double q : {0.5, 0.7071, 1.0, 2.0, 5.0})
                            {
                                if (type != cfx::BandType::Bell && q > 4.0)
                                    continue;
                                cfx::BandSpec s;
                                s.type = type;
                                s.freqHz = fc;
                                s.gainDb = gain;
                                s.q = q;
                                s.slope = 1;
                                const double e = bandError(s, row.rate);
                                (std::abs(gain) <= 12.0 ? worst12 : worst24) =
                                    std::max(std::abs(gain) <= 12.0 ? worst12 : worst24, e);
                                if (std::abs(gain) <= 12.0)
                                    worstRbj = std::max(worstRbj, bilinearError(s, row.rate));
                            }
                logMessage(String("fs ") + String(row.rate / 1000.0, 1) + " kHz: worst error |gain| <= 12 dB " +
                           String(worst12, 3) + " dB, |gain| = 24 dB " + String(worst24, 3) +
                           " dB (pre-warped bilinear / RBJ: " + String(worstRbj, 2) + " dB)");
                expect(worst12 <= row.tolerance, String("fs ") + String(row.rate, 0) + " error " + String(worst12, 3));
                expect(worst24 <= 2.0 * row.tolerance + 0.1,
                       String("fs ") + String(row.rate, 0) + " 24 dB error " + String(worst24, 3));
            }
        }

        beginTest("the cramping case of the plan: +12 dB, Q 1 bell at 16 kHz and 12 kHz at 44.1 kHz");
        {
            cfx::BandSpec s;
            s.type = cfx::BandType::Bell;
            s.gainDb = 12.0;
            s.q = 1.0;
            s.slope = 1;
            for (const double fc : {16000.0, 12000.0})
            {
                s.freqHz = fc;
                const double rbj = bilinearError(s, 44100.0);
                const double matched = bandError(s, 44100.0);
                logMessage(String("bell ") + String(fc, 0) + " Hz +12 dB Q 1 at 44.1 kHz: bilinear error " + String(rbj, 2) +
                           " dB, matched " + String(matched, 3) + " dB");
                expect(rbj > 3.0, "the bilinear design cramps");
                expect(matched <= 0.3, "matched design");
            }
        }

        beginTest("20 Hz bells at 192 kHz keep their accuracy (double-precision coefficients and states)");
        {
            double worst = 0.0;
            for (const double gain : {-12.0, 12.0})
                for (const double q : {0.7071, 2.0, 5.0})
                {
                    cfx::BandSpec s;
                    s.type = cfx::BandType::Bell;
                    s.freqHz = 20.0;
                    s.gainDb = gain;
                    s.q = q;
                    worst = std::max(worst, bandError(s, 192000.0));
                }
            // And through the filter: a 20 Hz tone at the centre reads the set gain after the transient.
            cfx::BandSpec s;
            s.type = cfx::BandType::Bell;
            s.freqHz = 20.0;
            s.gainDb = 12.0;
            s.q = 2.0;
            const auto f = cfx::designBand(s, 192000.0);
            cfx::BiquadChain chain;
            chain.setNow(f);
            const size_t n = static_cast<size_t>(192000.0 * 2.0);
            Signal tone = sineWave(n, 20.0, 192000.0, 0.5);
            std::vector<double> a(tone.begin(), tone.end()), b = a;
            for (size_t pos = 0; pos < n; pos += 64)
                chain.process(a.data() + pos, b.data() + pos, static_cast<int>(std::min<size_t>(64, n - pos)));
            const double measured = amplitudeToDb(toneAmplitude(a.data() + n / 2, n / 2, 20.0, 192000.0) / 0.5);
            logMessage(String("20 Hz bell at 192 kHz: worst magnitude error ") + String(worst, 4) + " dB, tone gain " +
                       String(measured, 3) + " dB (set 12)");
            expect(worst <= 0.05, "magnitude");
            expect(std::abs(measured - 12.0) <= 0.1, "tone through the filter");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void otherShapes()
    {
        beginTest("tilt, band-pass, notch, all-pass and the cuts: magnitude against the analog prototype");
        {
            for (const double rate : {44100.0, 48000.0, 96000.0})
            {
                double tilt = 0.0, bp = 0.0, notch = 0.0, allPass = 0.0, lowCut = 0.0, highCut = 0.0;
                for (const double fc : {30.0, 100.0, 500.0, 2000.0, 6000.0, 12000.0, 16000.0, 18000.0})
                {
                    for (const double q : {0.5, 0.7071, 2.0, 8.0})
                    {
                        cfx::BandSpec s;
                        s.freqHz = fc;
                        s.q = q;
                        s.type = cfx::BandType::BandPass;
                        bp = std::max(bp, bandError(s, rate));
                        s.type = cfx::BandType::Notch;
                        notch = std::max(notch, bandError(s, rate));
                        s.type = cfx::BandType::AllPass;
                        allPass = std::max(allPass, bandError(s, rate));
                    }
                    for (const double gain : {-12.0, 12.0})
                    {
                        cfx::BandSpec s;
                        s.type = cfx::BandType::Tilt;
                        s.freqHz = fc;
                        s.gainDb = gain;
                        tilt = std::max(tilt, bandError(s, rate));
                    }
                    for (int slope = 0; slope < cfx::kNumCutSlopes; ++slope)
                        for (int shape = 0; shape < 4; ++shape)
                        {
                            cfx::BandSpec s;
                            s.freqHz = fc;
                            s.q = 2.0;
                            s.slope = slope;
                            s.shape = static_cast<cfx::CutShape>(shape);
                            s.type = cfx::BandType::LowCut;
                            lowCut = std::max(lowCut, bandError(s, rate));
                            s.type = cfx::BandType::HighCut;
                            highCut = std::max(highCut, bandError(s, rate));
                        }
                }
                logMessage(String("fs ") + String(rate / 1000.0, 1) + " kHz worst error: tilt " + String(tilt, 3) +
                           ", band-pass " + String(bp, 3) + ", notch " + String(notch, 3) + ", all-pass " +
                           String(allPass, 5) + ", low cut " + String(lowCut, 3) + ", high cut " + String(highCut, 3) +
                           " dB (cuts compared down to -60 dB)");
                const double scale = rate < 50000.0 ? 1.0 : (rate < 80000.0 ? 0.4 : 0.15);
                expect(tilt <= 0.5 * scale + 0.05, "tilt");
                expect(bp <= 1.0 * scale + 0.1, "band-pass");
                expect(notch <= 1.2 * scale + 0.1, "notch");
                expect(allPass <= 1.0e-6, "all-pass has unit magnitude");
                expect(lowCut <= 2.5 * scale + 0.1, "low cut");
                expect(highCut <= 2.5 * scale + 0.1, "high cut");
            }
        }

        beginTest("cut slopes: 6, 12, 18, 24, 36, 48 and 96 dB per octave asymptotes and -3 / -6 dB points");
        {
            const double rate = 96000.0;
            const double expectedSlope[] = {6.0, 12.0, 18.0, 24.0, 36.0, 48.0, 96.0};
            double worst = 0.0;
            for (int slope = 0; slope < cfx::kNumCutSlopes; ++slope)
            {
                cfx::BandSpec s;
                s.type = cfx::BandType::LowCut;
                s.freqHz = 6000.0;
                s.slope = slope;
                const auto f = cfx::designBand(s, rate);
                const double m1 = bandMagnitude(f, 375.0, rate), m2 = bandMagnitude(f, 750.0, rate);
                const double measured = m2 - m1; // between 4 and 8 octaves below... two points an octave apart
                // Far below the corner the analog filter falls at slope dB per octave; compare octave 375 -> 750 Hz when the
                // attenuation is still above -120 dB.
                if (m1 > -110.0)
                {
                    worst = std::max(worst, std::abs(measured - expectedSlope[slope]));
                }
                const double atCorner = bandMagnitude(f, 6000.0, rate);
                expect(std::abs(atCorner + 3.0103) <= 0.15, String("Butterworth -3 dB at the corner, slope ") +
                                                                String(expectedSlope[slope], 0) + ": " +
                                                                String(atCorner, 3));
            }
            logMessage(String("cut slope error over the orders where the stop band is above -110 dB: ") + String(worst, 3) +
                       " dB per octave");
            expect(worst <= 0.5, "slope");
            cfx::BandSpec s;
            s.type = cfx::BandType::HighCut;
            s.freqHz = 3000.0;
            s.slope = 3;
            s.shape = cfx::CutShape::LinkwitzRiley;
            const double lr = bandMagnitude(cfx::designBand(s, rate), 3000.0, rate);
            logMessage(String("Linkwitz-Riley 24 dB/oct low-pass at its corner: ") + String(lr, 3) + " dB");
            expect(std::abs(lr + 6.0206) <= 0.15, "LR corner -6 dB");
            s.shape = cfx::CutShape::Bessel;
            s.slope = 2;
            const double bessel = bandMagnitude(cfx::designBand(s, rate), 3000.0, rate);
            logMessage(String("Bessel 18 dB/oct low-pass at its corner: ") + String(bessel, 3) + " dB");
            expect(std::abs(bessel + 3.0103) <= 0.2, "Bessel corner -3 dB");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void phaseOfDesigns()
    {
        beginTest("natural phase: the phase follows the analog prototype more closely than the magnitude-only fit");
        {
            for (const double rate : {44100.0, 48000.0})
            {
                double minMag = 0.0, minPhase = 0.0, natMag = 0.0, natPhase = 0.0;
                for (const double fc : {2000.0, 6000.0, 10000.0, 14000.0, 18000.0})
                    for (const double gain : {-12.0, 12.0})
                        for (const double q : {0.7071, 2.0})
                        {
                            cfx::BandSpec s;
                            s.type = cfx::BandType::Bell;
                            s.freqHz = fc;
                            s.gainDb = gain;
                            s.q = q;
                            double p = 0.0;
                            minMag = std::max(minMag, bandError(s, rate, cfx::DesignQuality::Minimum, &p));
                            minPhase = std::max(minPhase, p);
                            natMag = std::max(natMag, bandError(s, rate, cfx::DesignQuality::Natural, &p));
                            natPhase = std::max(natPhase, p);
                        }
                logMessage(String("fs ") + String(rate / 1000.0, 1) +
                           " kHz bell, phase against the analog prototype up to 16 kHz: minimum " + String(minPhase, 1) +
                           " deg (magnitude " + String(minMag, 3) + " dB), natural " + String(natPhase, 1) +
                           " deg (magnitude " + String(natMag, 3) + " dB)");
                expect(natPhase <= minPhase + 0.5, "natural phase is not worse");
                expect(natMag <= 0.8, "natural phase magnitude");
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void moduleMatchesDesign()
    {
        beginTest("the module's impulse response matches the design and the analog prototype (rack, blocks of 32)");
        {
            for (const double rate : {44100.0, 48000.0, 96000.0})
            {
                EqHarness h(rate);
                h.band(1, "Bell", 4000.0, 12.0, 2.0);
                h.band(2, "HighShelf", 10000.0, -6.0, 0.7071);
                h.band(3, "LowShelf", 120.0, 4.0, 0.7071);
                h.band(4, "Bell", 14500.0, 9.0, 1.0);
                const Signal ir = h.runMono(impulse(static_cast<size_t>(rate * 1.0)));
                cfx::BandSpec s1, s2, s3, s4;
                s1.type = cfx::BandType::Bell;
                s1.freqHz = 4000.0;
                s1.gainDb = 12.0;
                s1.q = 2.0;
                s2.type = cfx::BandType::HighShelf;
                s2.freqHz = 10000.0;
                s2.gainDb = -6.0;
                s2.q = 0.7071;
                s2.slope = 1;
                s3.type = cfx::BandType::LowShelf;
                s3.freqHz = 120.0;
                s3.gainDb = 4.0;
                s3.q = 0.7071;
                s3.slope = 1;
                s4.type = cfx::BandType::Bell;
                s4.freqHz = 14500.0;
                s4.gainDb = 9.0;
                s4.q = 1.0;
                const cfx::BandSpec specs[] = {s1, s2, s3, s4};
                double against = 0.0, vsDesign = 0.0;
                for (const double hz : logSpacedFrequencies(30.0, std::min(20000.0, 0.45 * rate), 12))
                {
                    double analog = 0.0, digital = 0.0;
                    for (const auto& s : specs)
                    {
                        const auto proto = cfx::bandPrototype(s);
                        analog += cfx::bandAnalogMagnitudeDb(proto, s, hz);
                        digital += cfx::designBand(s, proto, rate).magnitudeDb(kTau * hz / rate);
                    }
                    const double measured = magnitudeAt(ir, hz, rate);
                    against = std::max(against, std::abs(measured - analog));
                    vsDesign = std::max(vsDesign, std::abs(measured - digital));
                }
                logMessage(String("fs ") + String(rate / 1000.0, 1) + " kHz, four bands: measured against the design " +
                           String(vsDesign, 4) + " dB, against the analog prototype " + String(against, 3) + " dB");
                expect(vsDesign <= 0.02, "the module runs the design");
                expect(against <= (rate < 50000.0 ? 0.8 : 0.3), "analog prototype");
            }
        }

        beginTest("a flat module (no band on) is bit-transparent, and gain 0 bands are identities");
        {
            EqHarness h;
            Random rng(11);
            Signal x(48000);
            for (auto& v : x)
                v = 0.3 * rng.gaussian();
            const Signal y = h.runMono(x);
            double err = 0.0;
            for (size_t i = 0; i < x.size(); ++i)
                err = std::max(err, std::abs(y[i] - static_cast<double>(static_cast<float>(x[i]))));
            h.band(1, "Bell", 1000.0, 0.0, 1.0);
            h.band(2, "HighShelf", 8000.0, 0.0, 0.7);
            const Signal z = EqHarness(kRate).runMono(x);
            double err2 = 0.0;
            for (size_t i = 0; i < x.size(); ++i)
                err2 = std::max(err2, std::abs(z[i] - y[i]));
            logMessage(String("flat module: largest sample difference ") + String(err, 9) + ", gain-0 bands " +
                       String(err2, 9));
            expect(err <= 1.0e-6, "transparent");
            expect(err2 <= 1.0e-6, "identity bands");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void channels()
    {
        beginTest("channel modes: stereo, left, right, mid and side bands act on the right lanes");
        {
            const size_t n = static_cast<size_t>(0.6 * kRate);
            const Signal tone = sineWave(n, 1000.0, kRate, 0.25);
            Signal neg(n);
            for (size_t i = 0; i < n; ++i)
                neg[i] = -tone[i];
            const auto gainOf = [&](const Signal& out, const Signal& in)
            {
                return amplitudeToDb(toneAmplitude(out.data() + n / 2, n / 2, 1000.0, kRate)) -
                       amplitudeToDb(toneAmplitude(in.data() + n / 2, n / 2, 1000.0, kRate));
            };
            struct Case
            {
                int channel;
                const char* name;
                double left, right, midOfMono, sideOfSide;
            };
            // Gains read on: left channel with the mono input (l = r), right channel with the mono input, left channel with
            // an input that is pure side (l = -r).
            const double set = 9.0;
            for (const Case& c :
                 {Case{0, "stereo", set, set, 0, 0}, Case{1, "left", set, 0.0, 0, 0}, Case{2, "right", 0.0, set, 0, 0},
                  Case{3, "mid", set / 2, set / 2, 0, 0}, Case{4, "side", 0.0, 0.0, 0, 0}})
            {
                EqHarness h;
                h.band(1, "Bell", 1000.0, set, 1.0, c.channel);
                const auto mono = h.runStereo(tone, tone);
                EqHarness h2;
                h2.band(1, "Bell", 1000.0, set, 1.0, c.channel);
                const auto side = h2.runStereo(tone, neg);
                const double gl = gainOf(mono.first, tone), gr = gainOf(mono.second, tone), gs = gainOf(side.first, tone);
                logMessage(String("channel ") + c.name + ": mono input left " + String(gl, 2) + " right " + String(gr, 2) +
                           " dB, side input left " + String(gs, 2) + " dB");
                if (c.channel == 3)
                {
                    // Mid only: a mono signal gets the full gain, a pure side signal none.
                    expect(std::abs(gl - set) <= 0.05 && std::abs(gr - set) <= 0.05, "mid on mono");
                    expect(std::abs(gs) <= 0.05, "mid leaves the side alone");
                }
                else if (c.channel == 4)
                {
                    expect(std::abs(gl) <= 0.05 && std::abs(gr) <= 0.05, "side leaves a mono signal alone");
                    expect(std::abs(gs - set) <= 0.05, "side on pure side");
                }
                else
                {
                    expect(std::abs(gl - c.left) <= 0.05, "left lane");
                    expect(std::abs(gr - c.right) <= 0.05, "right lane");
                }
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    // Magnitude the four-band test curve should have, from the designs (the single source of truth for the phase modes).
    static double targetDb(double hz, double rate)
    {
        cfx::BandSpec a, b, c;
        a.type = cfx::BandType::Bell;
        a.freqHz = 1200.0;
        a.gainDb = 8.0;
        a.q = 1.5;
        b.type = cfx::BandType::Bell;
        b.freqHz = 5000.0;
        b.gainDb = -6.0;
        b.q = 2.0;
        c.type = cfx::BandType::HighShelf;
        c.freqHz = 9000.0;
        c.gainDb = 4.0;
        c.q = 0.7071;
        c.slope = 1;
        const cfx::BandSpec specs[] = {a, b, c};
        double db = 0.0;
        for (const auto& s : specs)
            db += cfx::designBand(s, rate).magnitudeDb(kTau * hz / rate);
        return db;
    }

    static void threeBands(EqHarness& h)
    {
        h.band(1, "Bell", 1200.0, 8.0, 1.5);
        h.band(2, "Bell", 5000.0, -6.0, 2.0);
        h.band(3, "HighShelf", 9000.0, 4.0, 0.7071);
        h.set("fx_ep_b3_slope", 1.0f);
    }

    void phaseModes()
    {
        beginTest(
            "linear phase: magnitude within 0.05 dB of the target, symmetric impulse response, latency equals the report");
        {
            for (const int resolution : {1, 2, 3})
            {
                EqHarness h(kRate, Quality::High);
                threeBands(h);
                h.set("fx_ep_phase", 2.0f);
                h.set("fx_ep_res", static_cast<float>(resolution));
                const int log2 = cfx::kMinKernelLog2 + resolution;
                const int latency = cfx::linearPhaseLatency(log2);
                const size_t n = static_cast<size_t>(latency * 2 + 24000);
                const Signal ir = settledImpulseResponse(h, n);
                // Peak (centre of the symmetric kernel) at the reported latency.
                size_t peak = 0;
                for (size_t i = 0; i < ir.size(); ++i)
                    if (std::abs(ir[i]) > std::abs(ir[peak]))
                        peak = i;
                double worst = 0.0;
                for (const double hz : logSpacedFrequencies(150.0, 16000.0, 8))
                    worst = std::max(worst, std::abs(magnitudeAt(ir, hz, kRate) - targetDb(hz, kRate)));
                // Symmetry about the centre tap, relative to the centre value.
                const size_t centre = static_cast<size_t>(latency);
                double asym = 0.0;
                const size_t span = static_cast<size_t>(cfx::kernelCentre(log2)) - 1;
                for (size_t k = 1; k < span; ++k)
                    asym = std::max(asym, std::abs(ir[centre - k] - ir[centre + k]));
                const double asymDb = amplitudeToDb(asym / std::abs(ir[peak]));
                logMessage(String("linear phase, ") + String((1 << log2) - 1) + " taps: magnitude error " +
                           String(worst, 4) + " dB, asymmetry " + String(asymDb, 1) + " dB, peak at " +
                           String(static_cast<double>(peak), 0) + " (reported latency " + String(latency) + ")");
                expect(worst <= 0.05, String("magnitude ") + String(worst, 4));
                expect(asymDb <= -120.0, "symmetry");
                expect(std::abs(static_cast<double>(peak) - static_cast<double>(latency)) <= 1.0, "latency");
                expectEquals(h.latency()->latencySamples(), latency, "reported latency");
            }
        }

        beginTest("minimum and natural phase: no latency, magnitude as designed");
        {
            for (const int mode : {0, 1})
            {
                EqHarness h;
                threeBands(h);
                h.set("fx_ep_phase", static_cast<float>(mode));
                const Signal ir = settledImpulseResponse(h, static_cast<size_t>(kRate), 9600);
                size_t peak = 0;
                for (size_t i = 0; i < 64; ++i)
                    if (std::abs(ir[i]) > std::abs(ir[peak]))
                        peak = i;
                double worst = 0.0;
                for (const double hz : logSpacedFrequencies(100.0, 16000.0, 8))
                    worst = std::max(worst, std::abs(magnitudeAt(ir, hz, kRate) - targetDb(hz, kRate)));
                logMessage(String(mode == 0 ? "minimum" : "natural") +
                           " phase: magnitude against the minimum-phase design " + String(worst, 3) +
                           " dB, reported latency " + String(h.latency()->latencySamples()));
                expectEquals(h.latency()->latencySamples(), 0, "latency");
                expect(worst <= (mode == 0 ? 0.02 : 0.6), "magnitude");
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void linearKernel()
    {
        beginTest("linear-phase engine: mix 0 returns the input delayed by the reported latency");
        {
            EqHarness h(kRate, Quality::High);
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 2.0f);
            h.set("fx_ep_mix", 0.0f);
            threeBands(h);
            Random rng(5);
            Signal x(static_cast<size_t>(1.0 * kRate));
            for (auto& v : x)
                v = 0.3 * rng.gaussian();
            const Signal y = h.runMono(x);
            const double nullDb = nullTest(x, y, h.latency()->latencySamples(), static_cast<size_t>(3000));
            logMessage(String("mix 0 in linear phase: output against the input delayed by the reported latency: ") +
                       String(nullDb, 1) + " dB");
            expect(nullDb <= -100.0, "dry path aligned");
        }

        beginTest("linear-phase kernel design: a 1.5 kHz, Q 4 bell within 0.1 dB at 8k taps");
        {
            EqHarness h(kRate, Quality::High);
            h.band(1, "Bell", 1500.0, 10.0, 4.0);
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 3.0f);
            const int log2 = cfx::kMinKernelLog2 + 3;
            const Signal ir = settledImpulseResponse(h, static_cast<size_t>(2 * cfx::linearPhaseLatency(log2) + 24000));
            cfx::BandSpec s;
            s.type = cfx::BandType::Bell;
            s.freqHz = 1500.0;
            s.gainDb = 10.0;
            s.q = 4.0;
            const auto f = cfx::designBand(s, kRate);
            double worst = 0.0;
            for (const double hz : logSpacedFrequencies(100.0, 16000.0, 12))
                worst = std::max(worst, std::abs(magnitudeAt(ir, hz, kRate) - bandMagnitude(f, hz, kRate)));
            logMessage(String("1.5 kHz Q 4 bell, 8191 taps: error against the matched magnitude ") + String(worst, 4) +
                       " dB");
            expect(worst <= 0.1, "kernel accuracy");
        }

        beginTest("linear-phase resolution: a low narrow bell (100 Hz, Q 4, +10 dB) needs a long kernel");
        {
            cfx::BandSpec s;
            s.type = cfx::BandType::Bell;
            s.freqHz = 100.0;
            s.gainDb = 10.0;
            s.q = 4.0;
            const auto f = cfx::designBand(s, kRate);
            double last = 0.0;
            String line("error of the magnitude against the matched design by kernel length:");
            for (int resolution = 0; resolution <= 5; ++resolution)
            {
                EqHarness h(kRate, Quality::High);
                h.band(1, "Bell", 100.0, 10.0, 4.0);
                h.set("fx_ep_phase", 2.0f);
                h.set("fx_ep_res", static_cast<float>(resolution));
                const int log2 = cfx::kMinKernelLog2 + resolution;
                const Signal ir = settledImpulseResponse(h, static_cast<size_t>(2 * cfx::linearPhaseLatency(log2) + 30000));
                double worst = 0.0;
                for (const double hz : logSpacedFrequencies(30.0, 400.0, 12))
                    worst = std::max(worst, std::abs(magnitudeAt(ir, hz, kRate) - bandMagnitude(f, hz, kRate)));
                line = line + " " + String(static_cast<double>((1 << log2) - 1), 0) + " taps " + String(worst, 3) + " dB;";
                last = worst;
            }
            logMessage(line);
            expect(last <= 0.1, "32k taps resolve a 100 Hz, Q 4 bell");
        }

        beginTest("linear-phase kernel swaps while a band's gain moves: the output stays continuous");
        {
            EqHarness h(kRate, Quality::High);
            h.band(1, "Bell", 1000.0, 0.0, 1.0);
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 2.0f);
            const size_t n = static_cast<size_t>(2.0 * kRate);
            const Signal tone = sineWave(n, 1000.0, kRate, 0.5);
            std::vector<float> l(tone.begin(), tone.end()), r = l;
            for (size_t pos = 0; pos < n; pos += 32)
            {
                const double t = static_cast<double>(pos) / kRate;
                const double a = t < 0.5 ? 0.0 : (t > 1.2 ? 1.0 : (t - 0.5) / 0.7);
                h.set("fx_ep_b1_gain", static_cast<float>(-12.0 + 24.0 * a));
                h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(32, n - pos)));
            }
            const Signal out(l.begin(), l.end());
            const ClickResult c = clickTest(out, 1000.0, kRate, static_cast<size_t>(0.4 * kRate));
            const double gainEnd = amplitudeToDb(toneAmplitude(out.data() + n - 9600, 9600, 1000.0, kRate) / 0.5);
            logMessage(String("gain swept -12 .. +12 dB over 0.7 s in linear phase (4k taps, a new kernel every 256 "
                              "samples): largest step ") +
                       String(c.stepDb, 1) + " dB, final gain " + String(gainEnd, 2) + " dB");
            expect(c.stepDb <= -70.0, "no clicks from the kernel swaps");
            expect(std::abs(gainEnd - 12.0) <= 0.1, "the sweep arrives");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void automation()
    {
        beginTest("automation: a band's frequency swept 20 Hz to 20 kHz (50 ms ramp and 2 Hz square) leaves no zipper above "
                  "-90 dB");
        {
            for (const int mode : {0, 1})
            {
                const auto render = [&](const Signal& in, const Signal& automation)
                {
                    EqHarness h;
                    h.band(1, "Bell", 1000.0, 12.0, 1.0);
                    h.set("fx_ep_phase", static_cast<float>(mode));
                    std::vector<float> l(in.begin(), in.end()), r = l;
                    for (size_t pos = 0; pos < l.size(); pos += 32)
                    {
                        const size_t end = std::min(pos + 32, l.size());
                        const double a = automation[end - 1];
                        h.set("fx_ep_b1_freq", static_cast<float>(20.0 * std::pow(1000.0, a)));
                        h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                    }
                    return Signal(l.begin(), l.end());
                };
                const ZipperResult z = zipperTest(render, kRate, 1000.0, -6.0);
                logMessage(String(mode == 0 ? "minimum" : "natural") +
                           " phase, bell +12 dB Q 1 swept in frequency: ramp step " + String(z.rampDb, 1) + " dB, square " +
                           String(z.squareDb, 1) + " dB");
                expect(z.rampDb <= -90.0, "ramp zipper");
                expect(z.squareDb <= -90.0, "square zipper");
            }
            const auto renderGain = [&](const Signal& in, const Signal& automation)
            {
                EqHarness h;
                h.band(1, "Bell", 1000.0, 0.0, 1.0);
                std::vector<float> l(in.begin(), in.end()), r = l;
                for (size_t pos = 0; pos < l.size(); pos += 32)
                {
                    const size_t end = std::min(pos + 32, l.size());
                    h.set("fx_ep_b1_gain", static_cast<float>(-24.0 + 48.0 * automation[end - 1]));
                    h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                }
                return Signal(l.begin(), l.end());
            };
            const ZipperResult g = zipperTest(renderGain, kRate, 1000.0, -6.0);
            logMessage(String("bell gain -24 .. +24 dB: ramp step ") + String(g.rampDb, 1) + " dB, square " +
                       String(g.squareDb, 1) + " dB");
            expect(g.rampDb <= -90.0 && g.squareDb <= -90.0, "gain zipper");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void dynamicBands()
    {
        beginTest("dynamic band: static curve within 0.2 dB of the gain computer, attack and release within 10 %");
        {
            const double centre = 4000.0;
            auto setup = [&](EqHarness& h, double thr, double ratio, double range, double attack, double release, int mode)
            {
                h.band(1, "Bell", centre, 0.0, 1.0);
                h.set("fx_ep_b1_dyn", static_cast<float>(mode));
                h.set("fx_ep_b1_range", static_cast<float>(range));
                h.set("fx_ep_b1_thr", static_cast<float>(thr));
                h.set("fx_ep_b1_ratio", static_cast<float>(ratio));
                h.set("fx_ep_b1_att", static_cast<float>(attack));
                h.set("fx_ep_b1_rel", static_cast<float>(release));
            };
            double worst = 0.0;
            struct Case
            {
                double thr, ratio, range;
                int mode;
            };
            for (const Case c : {Case{-30.0, 4.0, -12.0, 1}, Case{-24.0, 100.0, -15.0, 1}, Case{-20.0, 3.0, 10.0, 2},
                                 Case{-40.0, 2.0, 8.0, 1}})
            {
                for (double level = -60.0; level <= 0.0; level += 6.0)
                {
                    EqHarness h;
                    setup(h, c.thr, c.ratio, c.range, 5.0, 50.0, c.mode);
                    const Signal x = sineWave(static_cast<size_t>(1.0 * kRate), centre, kRate, dbToAmplitude(level));
                    const Signal y = h.runMono(x);
                    const double gain = amplitudeToDb(toneAmplitude(y.data() + 36000, 9600, centre, kRate)) - level;
                    const float e = c.mode == 1 ? dyn_amount(true, level, c.thr, c.ratio, std::abs(c.range))
                                                : dyn_amount(false, level, c.thr, c.ratio, std::abs(c.range));
                    const double ideal = (c.range < 0 ? -1.0 : 1.0) * e;
                    if (std::abs(gain - ideal) > 0.2)
                        logMessage(String("  case thr ") + String(c.thr, 0) + " ratio " + String(c.ratio, 0) + " range " +
                                   String(c.range, 0) + " level " + String(level, 0) + ": gain " + String(gain, 3) +
                                   " ideal " + String(ideal, 3));
                    worst = std::max(worst, std::abs(gain - ideal));
                }
            }
            logMessage(String("dynamic band static curve, four settings, -60 to 0 dBFS: worst error ") + String(worst, 3) +
                       " dB (plan 0.2)");
            expect(worst <= 0.2, "static curve");

            double worstTime = 0.0;
            for (const double attack : {20.0, 50.0, 150.0})
            {
                const auto render = [&](const Signal& in)
                {
                    EqHarness h;
                    setup(h, -40.0, 4.0, -18.0, attack, 100.0, 1);
                    return h.runMono(in);
                };
                const StepMetrics m = stepTimes(render, kRate, -50.0, -20.0, centre, 0.6, 1.5);
                worstTime = std::max(worstTime, std::abs(m.t63Ms / attack - 1.0));
                expect(m.valid() && std::abs(m.t63Ms / attack - 1.0) <= 0.10,
                       String("attack ") + String(attack, 0) + " ms measured " + String(m.t63Ms, 2));
            }
            for (const double release : {100.0, 300.0, 800.0})
            {
                const auto render = [&](const Signal& in)
                {
                    EqHarness h;
                    setup(h, -40.0, 4.0, -18.0, 10.0, release, 1);
                    return h.runMono(in);
                };
                std::vector<size_t> starts;
                const Signal in = programSine({{-20.0, 0.6}, {-50.0, 0.5 + 6.0 * release / 1000.0}}, centre, kRate, &starts);
                const Signal g = gainTrace(in, render(in), centre, kRate);
                const double measured = t63(g, kRate, starts[1], centre, 2.0);
                worstTime = std::max(worstTime, std::abs(measured / release - 1.0));
                expect(measured > 0.0 && std::abs(measured / release - 1.0) <= 0.10,
                       String("release ") + String(release, 0) + " ms measured " + String(measured, 2));
            }
            logMessage(String("dynamic band times: worst error ") + String(100.0 * worstTime, 1) + " % (plan 10 %)");
        }

        beginTest("dynamic band on the mid channel with an external key");
        {
            EqHarness h;
            h.band(1, "Bell", 2000.0, 0.0, 1.0, 3);
            h.set("fx_ep_b1_dyn", 1.0f);
            h.set("fx_ep_b1_range", -12.0f);
            h.set("fx_ep_b1_thr", -30.0f);
            h.set("fx_ep_b1_ratio", 100.0f);
            h.set("fx_ep_b1_att", 5.0f);
            h.set("fx_ep_b1_rel", 50.0f);
            h.set("fx_ep_b1_key", 1.0f);
            const size_t n = static_cast<size_t>(1.0 * kRate);
            const Signal tone = sineWave(n, 2000.0, kRate, 0.2), key = sineWave(n, 2000.0, kRate, 0.5);
            const auto pure = h.runStereo(tone, tone, &key);
            EqHarness quiet;
            quiet.values = h.values;
            Signal silent(n, 0.0);
            const auto none = quiet.runStereo(tone, tone, &silent);
            const double with =
                amplitudeToDb(toneAmplitude(pure.first.data() + n - 9600, 9600, 2000.0, kRate)) - amplitudeToDb(0.2);
            const double without =
                amplitudeToDb(toneAmplitude(none.first.data() + n - 9600, 9600, 2000.0, kRate)) - amplitudeToDb(0.2);
            logMessage(String("mid band, loud key: gain ") + String(with, 2) + " dB, silent key: " + String(without, 2) +
                       " dB");
            expect(with <= -10.0, "the key closes the band");
            expect(std::abs(without) <= 0.1, "a silent key leaves it open");
        }
    }

    // Time to 63 % of the settled change of a gain trace, measured after the first `skipMs` (the filter's own transient
    // when a narrow band sees a level step is shorter than that and is not part of the dynamics).
    static double t63(const Signal& g, double rate, size_t event, double hz, double skipMs)
    {
        const size_t w = static_cast<size_t>(std::llround(rate / hz));
        const double start = g[event - w];
        const size_t tail = std::max<size_t>(w, (g.size() - event) / 20);
        double end = 0.0;
        for (size_t i = g.size() - tail; i < g.size(); ++i)
            end += g[i];
        end /= static_cast<double>(tail);
        const double target = start + (end - start) * (1.0 - std::exp(-1.0));
        const double sign = end > start ? 1.0 : -1.0;
        for (size_t i = event + static_cast<size_t>(skipMs * 1.0e-3 * rate); i + 1 < g.size(); ++i)
            if (sign * (g[i] - target) >= 0.0)
                return static_cast<double>(i - event) / rate * 1000.0;
        return -1.0;
    }

    static float dyn_amount(bool above, double level, double thr, double ratio, double range)
    {
        const float lvl = static_cast<float>(level), t = static_cast<float>(thr), r = static_cast<float>(ratio);
        const float e = above ? cfx::dyn::compressorReduction(lvl, t, r, 6.0f) : cfx::dyn::upwardLift(lvl, t, r, 6.0f);
        return std::min(e, static_cast<float>(range));
    }

    // -----------------------------------------------------------------------------------------------------------
    // Sixteen bands at the extremes of every parameter (the settings the feature audit uses): the output stays finite and
    // bounded, and a chord that has ended leaves no tail, whatever the coefficient glides do on the way.
    void extremeBands()
    {
        beginTest("sixteen bands at extreme settings (10 Hz, Q 40, +-30 dB, every type): finite, and the tail ends (auto gain on or off)");
        int bad = 0;
        std::string first;
        for (const Quality quality : {Quality::Standard, Quality::High})
            for (std::uint32_t seed = 0; seed < 24; ++seed)
            {
                EngineConfig ec;
                ec.maxVoices = 12;
                ec.seed = 7;
                ec.quality = quality;
                Engine e(ec);
                e.prepare(44100.0, 256);
                e.resetParamsToDefaults();
                e.setParam("osc1_table", 4.0f);
                e.setParam("osc1_level", 0.8f);
                e.setParam("filter_cutoff", 5000.0f);
                e.setParam("amp_release", 0.15f);
                e.setParam("amp_sustain", 1.0f);
                std::mt19937 rng(seed * 7919u + 3u);
                const auto& desc = *ModuleRegistry::instance().find("fx.eq_pro");
                for (const auto& param : desc.params)
                {
                    if (param.id == desc.onParam || param.id == desc.positionParam)
                        continue;
                    const float n = seed % 4 == 0 ? 1.0f : (rng() & 1u) ? 1.0f : static_cast<float>(rng() % 3) * 0.5f;
                    e.setParamNormalised(ParameterMap::indexOf(param.id), n);
                }
                e.setParam(desc.onParam, 1.0f);
                std::vector<float> l(256), r(256);
                double tail = 0.0;
                bool finite = true;
                float peak = 0.0f;
                const int total = 3 * 44100 + 4410;
                for (int done = 0; done < total; done += 256)
                {
                    std::vector<MidiInput> midi;
                    if (done == 0)
                        for (const int note : {57, 61, 64})
                            midi.push_back({0, 0x90, static_cast<std::uint8_t>(note), 90});
                    if (done <= 30000 && 30000 < done + 256)
                        for (const int note : {57, 61, 64})
                            midi.push_back({30000 - done, 0x80, static_cast<std::uint8_t>(note), 0});
                    e.process(l.data(), r.data(), 256, midi.empty() ? nullptr : midi.data(), static_cast<int>(midi.size()));
                    for (int i = 0; i < 256; ++i)
                    {
                        finite = finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
                        peak = std::max({peak, std::abs(l[static_cast<size_t>(i)]), std::abs(r[static_cast<size_t>(i)])});
                        if (done + i >= total - 4800)
                            tail += static_cast<double>(l[static_cast<size_t>(i)]) * l[static_cast<size_t>(i)];
                    }
                }
                const double rms = std::sqrt(tail / 4800.0);
                if (!finite || peak > 5000.0f || rms >= 1.0)
                {
                    ++bad;
                    if (first.empty())
                        first = "seed " + std::to_string(seed) + (quality == Quality::High ? " high" : " standard") + " peak " +
                                std::to_string(peak) + " tail rms " + std::to_string(rms);
                }
            }
        expect(bad == 0, std::to_string(bad) + " of 48 renders failed; first: " + first);
    }

    void latency()
    {
        beginTest(
            "latency reporting: minimum phase 0, linear phase the kernel centre plus the block; change flag on a switch");
        {
            EqHarness h;
            threeBands(h);
            Signal x(48000, 0.0);
            h.runMono(x);
            expectEquals(h.latency()->latencySamples(), 0, "minimum");
            h.latency()->takeLatencyChanged();
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 4.0f);
            // The report follows the wet path, which switches once the worker has designed the kernel.
            bool changed = false;
            for (int i = 0; i < 2500 && !changed; ++i)
            {
                h.runMono(Signal(256, 0.0));
                changed = h.latency()->takeLatencyChanged();
                if (!changed)
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            expect(changed, "changed flag after the switch");
            expectEquals(h.latency()->latencySamples(), cfx::linearPhaseLatency(14), "16k taps");
            logMessage(String("16383-tap kernel: latency ") + String(h.latency()->latencySamples()) +
                       " samples = " + String(1000.0 * h.latency()->latencySamples() / kRate, 1) + " ms at 48 kHz");
        }

        beginTest("measured delay of noise through the module equals the reported latency to one sample");
        {
            for (const int resolution : {0, 2, 4})
            {
                EqHarness h(kRate, Quality::High);
                h.band(1, "Bell", 3000.0, 3.0, 0.7071);
                h.set("fx_ep_phase", 2.0f);
                h.set("fx_ep_res", static_cast<float>(resolution));
                Random rng(9);
                Signal x(static_cast<size_t>(2.5 * kRate));
                for (auto& v : x)
                    v = 0.2 * rng.gaussian();
                const Signal y = h.runMono(x);
                const size_t skip = static_cast<size_t>(0.8 * kRate);
                const Signal a(x.begin() + static_cast<long>(skip), x.end()),
                    b(y.begin() + static_cast<long>(skip), y.end());
                const double measured = measureLatency(a, b);
                const int reported = h.latency()->latencySamples();
                logMessage(String("resolution ") + String(resolution) + ": measured " + String(measured, 2) + ", reported " +
                           String(reported));
                expect(std::abs(measured - reported) <= 1.0, "latency");
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    // Feeds noise in blocks of 256 until the output carries signal again after the duck (the wet path has been switched in
    // and the convolver has primed); false after `seconds` of wall clock. The worker designs kernels on its own thread.
    static bool runUntilWet(EqHarness& h, Random& rng, double seconds)
    {
        const auto start = std::chrono::steady_clock::now();
        std::vector<float> l(256), r(256);
        int blocks = 0;
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < seconds)
        {
            double sum = 0.0;
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = r[i] = static_cast<float>(0.1 * rng.gaussian());
            h.run(l.data(), r.data(), 256);
            for (const float v : l)
                sum += static_cast<double>(v) * v;
            if (++blocks > 40 && std::sqrt(sum / 256.0) > 0.02) // past the duck (240 samples) and the priming
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return false;
    }

    void realTimeLinear()
    {
        beginTest(
            "linear phase in real time (Standard quality): the wet path comes back after the switch and carries the kernel");
        {
            EqHarness h(kRate, Quality::Standard);
            h.band(1, "Bell", 3000.0, 3.0, 0.7071);
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 0.0f);
            Random rng(11);
            expect(runUntilWet(h, rng, 5.0), "wet path active within 5 s");
            const int latency = cfx::linearPhaseLatency(cfx::kMinKernelLog2);
            const Signal ir = settledImpulseResponse(h, static_cast<size_t>(2 * latency + 8000), 8000);
            size_t peak = 0;
            for (size_t i = 0; i < ir.size(); ++i)
                if (std::abs(ir[i]) > std::abs(ir[peak]))
                    peak = i;
            logMessage(String("standard quality linear phase: peak ") + String(std::abs(ir[peak]), 3) + " at " +
                       String(static_cast<double>(peak), 0) + ", latency " + String(latency));
            expect(std::abs(ir[peak]) > 0.5, "impulse response is not muted");
            expect(std::abs(static_cast<double>(peak) - static_cast<double>(latency)) <= 1.0,
                   "peak at the reported latency");
        }
    }

    // Changing the Linear Resolution while it plays: the wet path ducks through the switch (no step in the output), and the
    // reported latency changes only when the new kernel carries the signal.
    void resolutionChange()
    {
        beginTest(
            "linear phase in real time: a Resolution change ducks through the switch, no step, latency reported in step");
        {
            EqHarness h(kRate, Quality::Standard);
            h.band(1, "Bell", 3000.0, 3.0, 0.7071);
            h.set("fx_ep_phase", 2.0f);
            h.set("fx_ep_res", 0.0f);
            Random rng(12);
            expect(runUntilWet(h, rng, 5.0), "wet path active");
            const int oldLatency = cfx::linearPhaseLatency(cfx::kMinKernelLog2);
            const int newLatency = cfx::linearPhaseLatency(cfx::kMinKernelLog2 + 2);
            expectEquals(h.latency()->latencySamples(), oldLatency, "latency before the change");

            const double step = 2.0 * 3.14159265358979 * 1000.0 / kRate;
            double phase = 0.0, worstJump = 0.0, lastOut = 0.0, startOfReport = 1.0;
            std::vector<float> l(256), r(256);
            for (int block = 0; block < 12; ++block) // a steady sine through the old kernel
            {
                for (size_t i = 0; i < l.size(); ++i)
                {
                    l[i] = r[i] = static_cast<float>(0.3 * std::sin(phase));
                    phase += step;
                }
                h.run(l.data(), r.data(), 256);
                lastOut = l.back();
            }
            h.set("fx_ep_res", 2.0f);
            int previousLatency = oldLatency, reports = 0;
            long worstAt = 0, totalSamples = 0, reportAt = 0;
            bool settled = false;
            const auto start = std::chrono::steady_clock::now();
            int afterSwitch = 0;
            while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() < 5.0 && afterSwitch < 40)
            {
                for (size_t i = 0; i < l.size(); ++i)
                {
                    l[i] = r[i] = static_cast<float>(0.3 * std::sin(phase));
                    phase += step;
                }
                h.run(l.data(), r.data(), 256);
                const int latencyNow = h.latency()->latencySamples();
                if (latencyNow != previousLatency)
                {
                    ++reports;
                    reportAt = totalSamples;
                    startOfReport = 0.0;
                    for (int i = 0; i < 4; ++i)
                        startOfReport = std::max(startOfReport, static_cast<double>(std::abs(l[static_cast<size_t>(i)])));
                    previousLatency = latencyNow;
                }
                for (const float v : l)
                {
                    if (std::abs(static_cast<double>(v) - lastOut) > worstJump)
                        worstAt = totalSamples;
                    worstJump = std::max(worstJump, std::abs(static_cast<double>(v) - lastOut));
                    lastOut = v;
                    ++totalSamples;
                }
                if (latencyNow == newLatency)
                {
                    settled = true;
                    ++afterSwitch;
                }
                if (!settled)
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            logMessage(String("resolution change: largest sample step ") + String(worstJump, 4) +
                       " (sine slope 0.038), latency reports " + String(reports) + ", output at the report " +
                       String(startOfReport, 4) + " worst at " + String(static_cast<double>(worstAt), 0) + " report at " +
                       String(static_cast<double>(reportAt), 0));
            expect(settled, "new latency reported");
            expectEquals(reports, 1, "latency changes once");
            expect(worstJump <= 0.06, "no step in the output through the switch");
            expect(startOfReport <= 0.01, "latency changes while the wet path is ducked");
        }
    }

    // Offline determinism: the request slot that hands designs to the worker.
    void requestSlot()
    {
        beginTest("request slot: a worker polling the slot makes tryPost fail, but an offline post never loses a request");
        {
            cfx::LatestRequestSlot<int> slot;
            std::atomic<bool> stop{false};
            std::thread poller(
                [&]
                {
                    int out = 0;
                    std::uint64_t seen = 0;
                    while (!stop.load())
                        slot.fetch(out, seen);
                });
            int refused = 0;
            for (int i = 0; i < 200000; ++i)
                if (!slot.tryPost(i))
                    ++refused;
            const std::uint64_t before = slot.posted();
            const int posts = 20000;
            for (int i = 0; i < posts; ++i)
                slot.post(i);
            stop.store(true);
            poller.join();
            logMessage(String("tryPost refused ") + String(refused) + " of 200000 posts while polled");
            expectEquals(static_cast<int>(slot.posted() - before), posts, "every offline post took");
        }

        beginTest(
            "convolver: dropKernel forgets the installed and the waiting set (a prepare must not keep an old-rate kernel)");
        {
            cfx::PartitionedConvolver convolver;
            convolver.prepare(2);
            cfx::KernelSetBuilder builder;
            std::vector<float> taps(static_cast<size_t>(cfx::kernelTaps(cfx::kMinKernelLog2)), 0.0f);
            taps[static_cast<size_t>(cfx::kernelCentre(cfx::kMinKernelLog2))] = 1.0f;
            const float* pointers[4] = {taps.data(), nullptr, nullptr, taps.data()};
            convolver.mailbox().post(builder.build(cfx::kMinKernelLog2, 2, pointers, 1));
            expect(convolver.adopt(cfx::kMinKernelLog2), "adopted before the drop");
            convolver.mailbox().post(builder.build(cfx::kMinKernelLog2, 2, pointers, 2));
            convolver.dropKernel();
            expect(!convolver.hasKernel(), "no installed kernel after the drop");
            expect(!convolver.adopt(cfx::kMinKernelLog2), "no waiting kernel after the drop");
        }

        beginTest("convolver: with the mailbox's retired slots full, a kernel install never leaks or blocks a set");
        {
            cfx::KernelSetBuilder builder;
            const auto buildAt = [&](int log2, std::uint64_t gen)
            {
                std::vector<float> taps(static_cast<size_t>(cfx::kernelTaps(log2)), 0.0f);
                taps[static_cast<size_t>(cfx::kernelCentre(log2))] = 1.0f;
                const float* pointers[4] = {taps.data(), nullptr, nullptr, taps.data()};
                return builder.build(log2, 2, pointers, gen);
            };
            const int base = cfx::KernelSet::liveCount().load();
            {
                cfx::PartitionedConvolver convolver;
                convolver.prepare(2);
                const int k0 = cfx::kMinKernelLog2;
                convolver.mailbox().post(buildAt(k0, 1));
                expect(convolver.adopt(k0), "first kernel adopted");
                // Occupy every retired slot, as a worker that is slow to collect would leave them.
                for (int i = 0; i < cfx::KernelMailbox::kRetired; ++i)
                    expect(convolver.mailbox().retire(buildAt(k0, 100 + static_cast<std::uint64_t>(i)).release()));
                // Three length-class changes in a row, each one retiring the one before into full slots.
                for (int k = 1; k <= 3; ++k)
                {
                    convolver.mailbox().post(buildAt(k0 + k, 10 + static_cast<std::uint64_t>(k)));
                    convolver.adopt(k0 + k);
                }
                // The worker collects; the kernels still held reach the mailbox over the next blocks and the last one is adopted.
                convolver.mailbox().collect();
                std::vector<float> in(256, 0.0f), o0(256), o1(256);
                float* outs[2] = {o0.data(), o1.data()};
                for (int b = 0; b < 400; ++b)
                {
                    convolver.process(in.data(), in.data(), outs, 2, 256);
                    convolver.mailbox().collect();
                }
                expect(convolver.adopt(k0 + 3), "the newest kernel ends up installed");
            }
            expectEquals(cfx::KernelSet::liveCount().load(), base, "every kernel set was freed");
        }

        beginTest("sample-rate change: the kernel of the old rate is not used again (same output as a fresh instance at the "
                  "new rate)");
        {
            const auto configure = [](EqHarness& h)
            {
                h.band(1, "Bell", 1000.0, 12.0, 4.0);
                h.set("fx_ep_phase", 2.0f);
                h.set("fx_ep_res", 1.0f);
            };
            EqHarness h(44100.0, Quality::High);
            configure(h);
            h.runMono(Signal(24000, 0.0));
            ModuleContext ctx;
            ctx.sampleRate = 96000.0;
            ctx.quality = Quality::High;
            h.rack.prepare(ctx);
            EqHarness fresh(96000.0, Quality::High);
            configure(fresh);
            Random rng(21);
            Signal x(12000);
            for (auto& v : x)
                v = 0.2 * rng.gaussian();
            const Signal y = h.runMono(x), expected = fresh.runMono(x);
            double worst = 0.0;
            for (size_t i = 0; i < x.size(); ++i)
                worst = std::max(worst, std::abs(y[i] - expected[i]));
            logMessage(String("96 kHz re-prepare against a fresh instance: largest difference ") + String(worst, 6));
            expect(worst <= 1.0e-5, "no trace of the old-rate kernel");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    void views()
    {
        beginTest("response view: the curve the module reports equals the designed magnitude plus the output gain");
        {
            EqHarness h;
            threeBands(h);
            h.set("fx_ep_out", 2.0f);
            h.runMono(Signal(4800, 0.0));
            auto* view = h.rack.findCapability<ResponseView>("fx.eq_pro");
            expect(view != nullptr, "capability");
            std::vector<float> curve(static_cast<size_t>(view->responsePoints()));
            view->fillResponse(curve.data());
            double worst = 0.0;
            for (int i = 0; i < view->responsePoints(); ++i)
            {
                const double hz = 20.0 * std::pow(20000.0 / 20.0, static_cast<double>(i) / (view->responsePoints() - 1));
                worst = std::max(worst, std::abs(curve[static_cast<size_t>(i)] - (targetDb(hz, kRate) + 2.0)));
            }
            logMessage(String("response view, ") + String(view->responsePoints()) + " points: largest difference " +
                       String(worst, 4) + " dB");
            expect(worst <= 0.01, "response view");
            expect(view->spectrumBins() > 0, "spectrum view exists");
        }

        beginTest("the engine publishes the response and spectrum views of the module (P6)");
        {
            Engine e;
            e.prepare(kRate, 256);
            e.setParam("fx_ep_on", 1.0f);
            e.setParam("fx_ep_b1_on", 1.0f);
            e.setParam("fx_ep_b1_type", 0.0f);
            e.setParam("fx_ep_b1_freq", 1000.0f);
            e.setParam("fx_ep_b1_gain", 12.0f);
            e.setParam("fx_ep_b1_q", 1.0f);
            const int response = e.views().find("fx.eq_pro", "response"), spectrum = e.views().find("fx.eq_pro", "spectrum");
            expect(response >= 0 && spectrum >= 0, "both views are registered");
            const int s1 = e.views().subscribe(response, 0.0f), s2 = e.views().subscribe(spectrum, 0.0f);
            std::vector<float> l(256), r(256);
            for (int b = 0; b < 100; ++b)
                e.process(l.data(), r.data(), 256);
            std::vector<float> curve(static_cast<size_t>(e.views().info(response).maxElements)),
                bins(static_cast<size_t>(e.views().info(spectrum).maxElements));
            const int nc = e.views().read(response, curve.data(), static_cast<int>(curve.size()), nullptr);
            const int nb = e.views().read(spectrum, bins.data(), static_cast<int>(bins.size()), nullptr);
            // The point nearest 1 kHz reads the set gain.
            double best = 1.0e9, atPeak = 0.0;
            for (int i = 0; i < nc; ++i)
            {
                const double hz = 20.0 * std::pow(1000.0, static_cast<double>(i) / (nc - 1));
                if (std::abs(hz - 1000.0) < best)
                {
                    best = std::abs(hz - 1000.0);
                    atPeak = curve[static_cast<size_t>(i)];
                }
            }
            logMessage(String("engine views: response ") + String(nc) + " points, 12 dB bell reads " + String(atPeak, 2) +
                       " dB near 1 kHz; spectrum " + String(nb) + " bins");
            expect(nc == 256 && std::abs(atPeak - 12.0) <= 0.2, "response view content");
            expect(nb == 1025, "spectrum view size");
            e.views().unsubscribe(s1);
            e.views().unsubscribe(s2);
        }

        beginTest("TPT state-variable filter: matches the analog bell below a quarter of the sampling rate and survives "
                  "audio-rate modulation");
        {
            cfx::TptSvf svf;
            svf.set(cfx::TptSvf::Mode::Bell, kRate, 1000.0, 1.0, 12.0);
            double worst = 0.0;
            for (const double hz : logSpacedFrequencies(30.0, 6000.0, 6))
            {
                cfx::BandSpec spec;
                spec.type = cfx::BandType::Bell;
                spec.freqHz = 1000.0;
                spec.gainDb = 12.0;
                spec.q = 1.0;
                const double analog = cfx::bandAnalogMagnitudeDb(cfx::bandPrototype(spec), spec, hz);
                worst = std::max(worst, std::abs(amplitudeToDb(std::abs(svf.response(kTau * hz / kRate))) - analog));
            }
            // Audio-rate modulation of the cutoff: bounded output for bounded input.
            cfx::TptSvf mod;
            double peak = 0.0;
            Random rng(8);
            for (int i = 0; i < 480000; ++i)
            {
                if (i % 8 == 0)
                    mod.set(cfx::TptSvf::Mode::Bell, kRate,
                            40.0 + 15000.0 * (0.5 + 0.5 * std::sin(kTau * 900.0 * i / kRate)), 6.0, 18.0);
                peak = std::max(peak, std::abs(mod.process(0.3 * rng.gaussian())));
            }
            logMessage(String("TPT bell, 30 Hz to 6 kHz: error against the analog prototype ") + String(worst, 3) +
                       " dB; cutoff swept at 900 Hz with Q 6 and +18 dB: peak " + String(peak, 2));
            expect(worst <= 0.35, "TPT magnitude");
            expect(peak < 40.0, "TPT modulation bounded");
        }

        beginTest("auto gain removes the mean of the curve");
        {
            EqHarness h;
            h.band(1, "Bell", 1000.0, 12.0, 0.7071);
            h.set("fx_ep_auto", 1.0f);
            h.runMono(Signal(9600, 0.0));
            auto* view = h.rack.findCapability<ResponseView>("fx.eq_pro");
            std::vector<float> curve(static_cast<size_t>(view->responsePoints()));
            view->fillResponse(curve.data());
            double mean = 0.0;
            int count = 0;
            for (int i = 0; i < view->responsePoints(); ++i)
            {
                const double hz = 20.0 * std::pow(1000.0, static_cast<double>(i) / (view->responsePoints() - 1));
                if (hz < 40.0)
                    continue;
                mean += curve[static_cast<size_t>(i)];
                ++count;
            }
            mean /= count;
            logMessage(String("auto gain: mean of the displayed curve ") + String(mean, 3) +
                       " dB (curve peak +12 dB before)");
            expect(std::abs(mean) <= 0.4, "mean near 0 dB");
        }
    }

    // -----------------------------------------------------------------------------------------------------------
    static double cpuPercent(const std::vector<std::pair<std::string, float>>& setup)
    {
        const size_t n = static_cast<size_t>(8.0 * kRate);
        Random rng(5);
        std::vector<float> src(n);
        for (auto& v : src)
            v = static_cast<float>(0.25 * rng.gaussian());
        const auto run = [&](bool enabled)
        {
            EqHarness h;
            h.set("fx_ep_on", 0.0f);
            if (enabled)
                for (const auto& kv : setup)
                    h.set(kv.first, kv.second);
            std::vector<float> l = src, r = src;
            for (size_t pos = 0; pos < 96000; pos += 32)
                h.run(l.data() + pos, r.data() + pos, 32);
            const auto t0 = std::chrono::steady_clock::now();
            for (size_t pos = 96000; pos + 32 <= n; pos += 32)
                h.run(l.data() + pos, r.data() + pos, 32);
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
        double best = 1.0e9;
        for (int i = 0; i < 3; ++i)
            best = std::min(best, run(true) - run(false));
        return 100.0 * best / (static_cast<double>(n - 96000) / kRate);
    }

    static std::vector<std::pair<std::string, float>> eightBands(float phase)
    {
        std::vector<std::pair<std::string, float>> s{{"fx_ep_on", 1.0f}, {"fx_ep_phase", phase}};
        const double freqs[] = {60.0, 160.0, 400.0, 1000.0, 2500.0, 6000.0, 10000.0, 15000.0};
        for (int b = 0; b < 8; ++b)
        {
            const std::string n = "fx_ep_b" + std::to_string(b + 1) + "_";
            s.push_back({n + "on", 1.0f});
            s.push_back({n + "freq", static_cast<float>(freqs[b])});
            s.push_back({n + "gain", b % 2 == 0 ? 3.0f : -3.0f});
            s.push_back({n + "q", 1.2f});
        }
        return s;
    }

    void realTime()
    {
        beginTest("auto gain is bounded: random extreme settings (seed 49 of the audit) leave no constant output after a note");
        {
            // Found by the feature audit (random extremes): with every band at an extreme the mean of the curve is far below
            // -30 dB, the auto makeup went past +100 dB and the noise floor of the ended note came out as a constant at the
            // output clamp (29.9998). The makeup is limited to +-30 dB like a band gain.
            const ModuleDescriptor* d = ModuleRegistry::instance().find("fx.eq_pro");
            expect(d != nullptr);
            if (d == nullptr)
                return;
            std::mt19937 rng(49);
            Engine e;
            e.prepare(44100.0, 256);
            for (const auto& p : d->params)
            {
                if (p.id == d->onParam || p.id == d->positionParam)
                    continue;
                const float n = (rng() & 1u) ? 1.0f : static_cast<float>(rng() % 3) * 0.5f;
                e.setParamNormalised(ParameterMap::indexOf(p.id), n);
            }
            e.setParam(d->onParam, 1.0f);
            std::vector<float> l(256), r(256);
            MidiInput on[3] = {{0, 0x90, 48, 100}, {0, 0x90, 55, 100}, {0, 0x90, 60, 100}};
            MidiInput off[3] = {{0, 0x80, 48, 0}, {0, 0x80, 55, 0}, {0, 0x80, 60, 0}};
            double tail = 0.0;
            for (int block = 0; block < 560; ++block)
            {
                e.process(l.data(), r.data(), 256, block == 0 ? on : (block == 86 ? off : nullptr),
                          block == 0 || block == 86 ? 3 : 0);
                if (block >= 540)
                    for (const float v : l)
                        tail = std::max(tail, static_cast<double>(std::abs(v)));
            }
            expect(tail < 1.0, "tail peak " + String(tail));
        }

        beginTest("no allocation on the audio thread: eight bands, every phase mode, dynamic and mid / side bands, "
                  "parameters changing every block");
        {
            EqHarness h;
            for (const auto& kv : eightBands(0.0f))
                h.set(kv.first, kv.second);
            h.set("fx_ep_b2_dyn", 1.0f);
            h.set("fx_ep_b2_ch", 3.0f);
            h.set("fx_ep_b3_dyn", 2.0f);
            h.set("fx_ep_b3_key", 1.0f);
            h.set("fx_ep_b4_ch", 4.0f);
            Random rng(3);
            std::vector<float> l(32), r(32), aux(32);
            for (int warm = 0; warm < 20; ++warm)
                h.run(l.data(), r.data(), 32, aux.data(), aux.data());
            crucible::testing::gAllocations = 0;
            crucible::testing::gTrackAllocations = true;
            for (int block = 0; block < 1500; ++block)
            {
                for (size_t i = 0; i < 32; ++i)
                {
                    l[i] = static_cast<float>(0.3 * rng.gaussian());
                    r[i] = static_cast<float>(0.3 * rng.gaussian());
                    aux[i] = static_cast<float>(0.3 * rng.gaussian());
                }
                h.set("fx_ep_phase", static_cast<float>((block / 150) % 3));
                h.set("fx_ep_res", static_cast<float>((block / 400) % 6));
                h.set("fx_ep_b1_freq", 40.0f + static_cast<float>((block * 37) % 900));
                h.set("fx_ep_b5_gain", static_cast<float>(block % 25) - 12.0f);
                h.set("fx_ep_b6_type", static_cast<float>(block % 9));
                h.set("fx_ep_b6_slope", static_cast<float>(block % 7));
                h.set("fx_ep_b6_shape", static_cast<float>((block / 3) % 4));
                h.set("fx_ep_b7_ch", static_cast<float>(block % 5));
                h.set("fx_ep_b2_range", static_cast<float>((block % 13) - 6));
                h.run(l.data(), r.data(), 32, (block % 4 != 0) ? aux.data() : nullptr,
                      (block % 4 != 0) ? aux.data() : nullptr);
            }
            crucible::testing::gTrackAllocations = false;
            expectEquals(static_cast<int>(crucible::testing::gAllocations), 0, "allocations");
            bool finite = true;
            for (size_t i = 0; i < 32; ++i)
                finite = finite && std::isfinite(l[i]) && std::isfinite(r[i]);
            expect(finite, "finite output");
        }

#if !defined(CRUCIBLE_TEST_UNTIMED)
        beginTest("CPU: percent of one core per stereo instance at 48 kHz, eight active bands (plan: 0.25 % minimum, 0.8 % "
                  "linear)");
        {
            const double minimum = cpuPercent(eightBands(0.0f));
            const double natural = cpuPercent(eightBands(1.0f));
            auto linearSetup = eightBands(2.0f);
            linearSetup.push_back({"fx_ep_res", 2.0f});
            const double linear = cpuPercent(linearSetup);
            auto dynamicSetup = eightBands(0.0f);
            dynamicSetup.push_back({"fx_ep_b2_dyn", 1.0f});
            dynamicSetup.push_back({"fx_ep_b4_dyn", 1.0f});
            const double dynamic = cpuPercent(dynamicSetup);
            logMessage(String("eight bands: minimum ") + String(minimum, 3) + " %, natural " + String(natural, 3) +
                       " %, linear (4k taps) " + String(linear, 3) + " %, minimum with two dynamic bands " +
                       String(dynamic, 3) + " %");
            expect(minimum <= 0.25 * 1.2, "minimum phase CPU");
            expect(linear <= 0.8 * 1.2, "linear phase CPU");
        }
#endif
    }
};

static EqProTests eqProTests;
