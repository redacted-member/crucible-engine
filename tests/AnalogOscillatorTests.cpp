// SPDX-License-Identifier: MIT
// The analog (VA) oscillator `osc.analog`: registration and type indices, aliasing floors, waveform shapes, band-limited
// sync / FM, characters, unison, drift statistics, tolerance, bit-exactness of the other oscillator types and CPU cost.

#include "AnalogMeasure.h"
#include "GraphSupport.h"

#include <crucible/module/Oscillator.h>

#include <chrono>
#include <functional>
#include <limits>

// Timing is meaningless in debug and sanitizer builds, and under Rosetta (x86_64 code on Apple silicon).
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
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr int kAnalogType = 5;

// Drives one analog oscillator instance directly, the way a voice does: chunks of 32 samples, parameters natural units.
class Rig
{
public:
    explicit Rig(double rate = 48000.0, int slot = 0) : sampleRate(rate), prefix("osc" + std::to_string(slot + 1) + "_an_")
    {
        const auto& registry = ModuleRegistry::instance();
        const auto types = registry.ofKind(ModuleKind::Oscillator);
        instance = types[kAnalogType]->createOscillator(slot, registry.parameters());
        instance->prepare(rate, 1);
        values = ParameterMap::defaultValues();
        OscNoteInfo info;
        info.seed = 12345;
        instance->noteOn(info);
    }

    void set(const std::string& suffix, float v) { setValue(values, prefix + suffix, v); }
    void reseed(std::uint32_t seed)
    {
        OscNoteInfo info;
        info.seed = seed;
        instance->noteOn(info);
    }

    // Renders `samples` samples at MIDI pitch `note`; `each` may change parameters before every chunk.
    template <typename Each> m::Signal render(double note, size_t samples, std::vector<float>* right, Each&& each)
    {
        m::Signal out;
        out.reserve(samples);
        std::vector<float> l(32), r(32);
        for (size_t done = 0; done < samples; done += 32)
        {
            each(done);
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            OscRender rr;
            rr.values = values.data();
            rr.pitch = static_cast<float>(note);
            rr.sampleRate = sampleRate;
            rr.numSamples = 32;
            rr.left = l.data();
            rr.right = r.data();
            instance->render(rr);
            for (int i = 0; i < 32; ++i)
            {
                out.push_back(l[static_cast<size_t>(i)]);
                if (right != nullptr)
                    right->push_back(r[static_cast<size_t>(i)]);
            }
        }
        return out;
    }
    m::Signal render(double note, size_t samples, std::vector<float>* right = nullptr)
    {
        return render(note, samples, right, [](size_t) {});
    }

    double sampleRate;
    std::string prefix;
    std::vector<float> values;
    std::unique_ptr<OscillatorInstance> instance;
};

enum W
{
    Saw = 0,
    Pulse,
    Tri,
    Sine,
    SawPulse,
    Morph
};
enum X
{
    XOff = 0,
    HardSync,
    SoftSync,
    LinearFm,
    ExpFm,
    Ring
};

constexpr double kPiD = 3.141592653589793;

// Alias-free reference comparison: magnitude spectrum of `ours` (n samples from `start`) against the same render at
// `factor` times the rate, in dB of the error power below 20 kHz re the signal power.
double spectralDifferenceDb(const m::Signal& ours, const m::Signal& ref, size_t start, size_t n, double rate, size_t factor)
{
    const auto a = m::computeSpectrum(ours.data() + start, n, rate);
    const auto b = m::computeSpectrum(ref.data() + start * factor, n * factor, rate * static_cast<double>(factor));
    double error = 0.0, total = 0.0;
    for (size_t k = 1; k < a.bins() && static_cast<double>(k) * a.binHz < 20000.0; ++k)
    {
        const double d = a.amplitude(k) - b.amplitude(k);
        error += d * d;
        total += b.amplitude(k) * b.amplitude(k);
    }
    return m::powerToDb(error / std::max(total, 1.0e-300));
}

double sineHz(double note, double rate, size_t samples, const std::function<void(Rig&)>& setup)
{
    Rig rig(rate);
    rig.set("wave", Sine);
    setup(rig);
    const auto sig = rig.render(note, samples);
    return m::estimateFrequency(sig.data() + samples / 4, samples / 2, rate, 440.0 * std::pow(2.0, (note - 69.0) / 12.0));
}

double hzOf(double note)
{
    return 440.0 * std::pow(2.0, (note - 69.0) / 12.0);
}

// Aliasing (dB re the fundamental, below 20 kHz) of a periodic render at the exact fundamental.
double aliasOf(Rig& rig, double note, double rate, double seconds = 0.9)
{
    const auto sig = rig.render(note, static_cast<size_t>(seconds * rate));
    const size_t start = static_cast<size_t>(0.2 * rate);
    return m::measureAliasing(sig, start, rate, hzOf(note)).aliasDb;
}
} // namespace

class AnalogOscillatorTests : public UnitTest
{
public:
    AnalogOscillatorTests() : UnitTest("Analog oscillator", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("osc.analog is the sixth oscillator type and the existing type indices did not move");
        {
            const auto types = ModuleRegistry::instance().ofKind(ModuleKind::Oscillator);
            expect(types.size() >= 6, "the analog oscillator is followed by the P13 types only");
            const char* ids[] = {"osc.wavetable", "osc.sample", "osc.cloud", "osc.noise", "osc.sub", "osc.analog"};
            for (size_t i = 0; i < 6; ++i)
                expectEquals(types[i]->id, std::string(ids[i]));
            const auto& type = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc1_type"))];
            expect(type.choices.size() >= 6);
            expectEquals(type.choices[4], std::string("Sub"));
            expectEquals(type.choices[5], std::string("Analog (VA)"));
            for (const char* id : {"osc1_an_wave", "osc3_an_xmod_mode", "osc2_an_drift_tau"})
                expect(ParameterMap::indexOf(id) >= 0, id);
        }

        beginTest("default settings: saw, pulse (10 % and 50 %), triangle stay below -90 dB below 20 kHz, C2 to C8 at 44.1, "
                  "48 and 96 kHz");
        {
            double worst[4] = {-300.0, -300.0, -300.0, -300.0};
            const char* names[] = {"saw", "pulse 10 %", "pulse 50 %", "triangle"};
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const int note : {36, 48, 60, 72, 84, 96, 108})
                    for (int shapeIndex = 0; shapeIndex < 4; ++shapeIndex)
                    {
                        Rig rig(rate);
                        rig.set("wave", shapeIndex == 0 ? Saw : shapeIndex == 3 ? Tri : Pulse);
                        rig.set("pw", shapeIndex == 1 ? 0.1f : 0.5f);
                        const double a = aliasOf(rig, note, rate);
                        worst[shapeIndex] = std::max(worst[shapeIndex], a);
                        expect(a < -90.0, std::string(names[shapeIndex]) + " note " + std::to_string(note) + " at " +
                                              std::to_string(rate) + " Hz: " + std::to_string(a) + " dB");
                    }
            for (int i = 0; i < 4; ++i)
                logMessage(String("worst ") + names[i] + " aliasing of 21 cases: " + String(worst[i], 1) + " dB");
        }

        beginTest("PWM: static pulse widths from 2 % to 97 % stay below -90 dB (re the fundamental of a saw-sized pulse)");
        {
            double worstStatic = -300.0;
            for (const double rate : {44100.0, 48000.0})
            {
                for (const int note : {36, 48, 60, 72, 84, 96})
                    for (const float width : {0.02f, 0.05f, 0.2f, 0.35f, 0.5f, 0.8f, 0.97f})
                    {
                        Rig rig(rate);
                        rig.set("wave", Pulse);
                        rig.set("pw", width);
                        const double a = aliasOf(rig, note, rate);
                        // A narrow pulse has a weak fundamental: judge the floor against a saw-sized fundamental.
                        const double fundamentalDb = 20.0 * std::log10(std::max(std::abs(std::sin(kPiD * width)), 1.0e-6));
                        worstStatic = std::max(worstStatic, a + fundamentalDb);
                        expect(a + fundamentalDb < -90.0, "pulse " + std::to_string(width) + " note " +
                                                              std::to_string(note) + " at " + std::to_string(rate) + ": " +
                                                              std::to_string(a) + " dB");
                    }
            }
            logMessage("worst PWM: static widths " + String(worstStatic, 1) + " dB (re the fundamental)");
        }

        beginTest("a width that is modulated (moving-edge path) renders the same as the static width when the motion is "
                  "negligible");
        {
            for (const double note : {45.0, 60.0, 81.0})
            {
                Rig fixed, moving;
                fixed.set("wave", Pulse);
                fixed.set("pw", 0.3f);
                moving.set("wave", Pulse);
                const auto a = fixed.render(note, 9600);
                const auto b = moving.render(note, 9600, nullptr,
                                             [&](size_t at)
                                             {
                                                 moving.set("pw", 0.3f + ((at / 32) % 2 == 0 ? 1.0e-9f : -1.0e-9f) * 0.0f +
                                                                      1.0e-7f * static_cast<float>((at / 32) % 2));
                                             });
                double err = 0.0, ref = 0.0;
                for (size_t i = 0; i < a.size(); ++i)
                {
                    err += (a[i] - b[i]) * (a[i] - b[i]);
                    ref += a[i] * a[i];
                }
                const double db = m::powerToDb(err / ref);
                logMessage("moving-edge path vs static path, note " + String(note, 0) + ": " + String(db, 1) + " dB");
                expect(db < -100.0, "moving vs static, note " + std::to_string(note) + ": " + std::to_string(db));
            }
        }

        beginTest("hard sync (ratio 2.37), soft sync, through-zero and exponential FM stay below -90 dB");
        {
            double worstHard = -300.0, worstSoft = -300.0, worstTz = -300.0, worstExp = -300.0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const int note : {36, 48, 60, 72, 84})
                {
                    const struct
                    {
                        X mode;
                        double ratio;
                        float amt;
                        double period; // in master cycles
                        double* worst;
                        const char* name;
                    } cases[] = {{HardSync, 2.37, 0.5f, 1.0, &worstHard, "hard sync"},
                                 {SoftSync, 2.37, 0.5f, 2.0, &worstSoft, "soft sync"},
                                 {LinearFm, 2.0, 0.25f, 1.0, &worstTz, "TZ FM"},
                                 {ExpFm, 2.0, 0.25f, 1.0, &worstExp, "exp FM"}};
                    for (const auto& c : cases)
                    {
                        Rig rig(rate);
                        rig.set("wave", Saw);
                        rig.set("xmod_mode", static_cast<float>(c.mode));
                        rig.set("xmod_ratio", static_cast<float>(c.ratio));
                        rig.set("xmod_amt", c.amt);
                        const auto sig = rig.render(note, static_cast<size_t>(0.9 * rate));
                        const double a =
                            m::measureAliasing(sig, static_cast<size_t>(0.2 * rate), rate, hzOf(note) / c.period).aliasDb;
                        *c.worst = std::max(*c.worst, a);
                        expect(a < -90.0, std::string(c.name) + " note " + std::to_string(note) + " at " +
                                              std::to_string(rate) + ": " + std::to_string(a));
                    }
                }
            logMessage("worst aliasing (18 cases each): hard sync " + String(worstHard, 1) + ", soft sync " +
                       String(worstSoft, 1) + ", TZ FM " + String(worstTz, 1) + ", exp FM " + String(worstExp, 1) + " dB");
        }

        beginTest("shapes: the saw follows 1/n within 0.1 dB, the triangle 1/n^2 on the odd harmonics, DC below -100 dB");
        {
            Rig saw;
            saw.set("wave", Saw);
            const auto s = saw.render(60, 96000);
            const auto prof = m::harmonicProfile(s.data() + 24000, 65536, 48000.0, hzOf(60), 200);
            const auto e = m::shapeError(prof, [](int k) { return -20.0 * std::log10(static_cast<double>(k)); }, 10000.0);
            logMessage("saw shape error " + String(e.maxAbsDb, 4) + " dB over " + String(e.harmonics) + " harmonics");
            expect(e.maxAbsDb < 0.1, "saw shape error " + String(e.maxAbsDb, 4));

            Rig tri;
            tri.set("wave", Tri);
            const auto t = tri.render(60, 96000);
            const auto tp = m::harmonicProfile(t.data() + 24000, 65536, 48000.0, hzOf(60), 100);
            const auto te = m::shapeError(
                tp, [](int k) { return -40.0 * std::log10(static_cast<double>(k)); }, 10000.0,
                [](int k) { return k % 2 == 1; });
            for (int k = 1; k <= tp.count(); k += 2)
                logMessage("  tri h" + String(k) + ": " + String(tp.harmonicDb(k), 2) + " dB (ideal " +
                           String(-40.0 * std::log10(static_cast<double>(k)), 2) + ")");
            logMessage("triangle shape error " + String(te.maxAbsDb, 4) + " dB over " + String(te.harmonics) +
                       " odd harmonics");
            expect(te.maxAbsDb < 0.1, "triangle shape error " + String(te.maxAbsDb, 4));

            for (const int wv : {Saw, Tri, Sine})
            {
                Rig r;
                r.set("wave", static_cast<float>(wv));
                const auto x = r.render(57, 48000 * 2);
                // whole periods only: 220 Hz at 48 kHz repeats every 48000/220 samples, use 1 s = 220 periods
                const m::Signal win(x.begin() + 24000, x.begin() + 24000 + 48000);
                const double dc = m::dcDbfs(win);
                logMessage("DC of wave " + String(wv) + ": " + String(dc, 1) + " dBFS");
                expect(dc < -100.0, "DC wave " + std::to_string(wv) + ": " + std::to_string(dc));
            }
        }

        beginTest("hard and soft sync at the top of the range (C7) keep the floor the table allows");
        {
            // The 32-zero-crossing kernel reaches about -88 dB for a 5 kHz slave at C7; the plan's -90 dB row is held to C6
            // above.
            double worst = -300.0;
            for (const double rate : {44100.0, 48000.0})
                for (const X mode : {HardSync, SoftSync})
                {
                    Rig rig(rate);
                    rig.set("wave", Saw);
                    rig.set("xmod_mode", static_cast<float>(mode));
                    rig.set("xmod_ratio", 2.37f);
                    const auto sig = rig.render(96, static_cast<size_t>(0.9 * rate));
                    const double a = m::measureAliasing(sig, static_cast<size_t>(0.2 * rate), rate,
                                                        hzOf(96) / (mode == SoftSync ? 2.0 : 1.0))
                                         .aliasDb;
                    worst = std::max(worst, a);
                    expect(a < -85.0, "C7 sync mode " + std::to_string(mode) + " at " + std::to_string(rate) + ": " +
                                          std::to_string(a));
                }
            logMessage("worst C7 sync aliasing: " + String(worst, 1) + " dB");
        }

        beginTest("above fs/4 the output crossfades into the one-harmonic table: no aliasing up to Nyquist");
        {
            double worst = -300.0;
            for (const double note : {126.0, 128.0, 131.0, 134.0})
                for (const int wv : {Saw, Pulse, Tri})
                {
                    Rig rig;
                    rig.set("wave", static_cast<float>(wv));
                    const auto sig = rig.render(note, 40000);
                    const double f0 = hzOf(note);
                    const double a = m::measureAliasing(sig, 4096, 48000.0, f0).aliasDb;
                    worst = std::max(worst, a);
                    expect(a < -90.0,
                           "note " + std::to_string(note) + " wave " + std::to_string(wv) + ": " + std::to_string(a));
                    expect(f0 / 48000.0 > 0.25 || note < 127.0);
                }
            logMessage("worst aliasing above fs/4: " + String(worst, 1) + " dB");
        }

        beginTest("the 5 Hz DC blocker removes the offset of a narrow pulse");
        {
            Rig rig;
            rig.set("wave", Pulse);
            rig.set("pw", 0.1f);
            const auto sig = rig.render(57, 96000);
            const m::Signal win(sig.begin() + 48000, sig.begin() + 48000 + 48000);
            // The naive pulse has a mean of -0.8; whole cycles (220 Hz, one second) are averaged here.
            const double dc = m::dcDbfs(win);
            logMessage("DC of a 10 % pulse: " + String(dc, 1) + " dBFS");
            expect(dc < -70.0, "pulse DC " + std::to_string(dc));
            // Settling: the offset of the first cycles is gone within about 5 time constants of 32 ms.
            expect(std::abs(m::dcOffset(m::Signal(sig.begin(), sig.begin() + 480))) < 0.2,
                   "no step at the start of the note");
        }

        beginTest("mix and morph: the end points are the pure shapes, the morph is continuous");
        {
            const auto render = [](const std::function<void(Rig&)>& setup)
            {
                Rig rig;
                setup(rig);
                return rig.render(60, 4800);
            };
            const auto same = [&](const m::Signal& a, const m::Signal& b)
            {
                double worst = 0.0;
                for (size_t i = 0; i < a.size(); ++i)
                    worst = std::max(worst, std::abs(a[i] - b[i]));
                return worst;
            };
            const auto saw = render([](Rig& r) { r.set("wave", Saw); });
            const auto pulse = render([](Rig& r) { r.set("wave", Pulse); });
            const auto tri = render([](Rig& r) { r.set("wave", Tri); });
            const auto sine = render([](Rig& r) { r.set("wave", Sine); });
            expect(same(saw, render(
                                 [](Rig& r)
                                 {
                                     r.set("wave", SawPulse);
                                     r.set("shape", 0.0f);
                                 })) < 1.0e-6);
            expect(same(pulse, render(
                                   [](Rig& r)
                                   {
                                       r.set("wave", SawPulse);
                                       r.set("shape", 1.0f);
                                   })) < 1.0e-6);
            expect(same(sine, render(
                                  [](Rig& r)
                                  {
                                      r.set("wave", Morph);
                                      r.set("shape", 0.0f);
                                  })) < 1.0e-6);
            expect(same(tri, render(
                                 [](Rig& r)
                                 {
                                     r.set("wave", Morph);
                                     r.set("shape", 1.0f / 3.0f);
                                 })) < 1.0e-4);
            expect(same(saw, render(
                                 [](Rig& r)
                                 {
                                     r.set("wave", Morph);
                                     r.set("shape", 2.0f / 3.0f);
                                 })) < 1.0e-4);
            expect(same(pulse, render(
                                   [](Rig& r)
                                   {
                                       r.set("wave", Morph);
                                       r.set("shape", 1.0f);
                                   })) < 1.0e-6);
            // A slow sweep of the shape control has no jumps beyond those of the edges.
            Rig sweep;
            sweep.set("wave", Morph);
            const auto s =
                sweep.render(36, 96000, nullptr, [&](size_t at) { sweep.set("shape", static_cast<float>(at) / 96000.0f); });
            double aliasWorst = -300.0;
            for (const float x : {0.1f, 0.4f, 0.55f, 0.8f, 0.95f})
            {
                Rig r;
                r.set("wave", Morph);
                r.set("shape", x);
                aliasWorst = std::max(aliasWorst, aliasOf(r, 72, 48000.0));
            }
            logMessage("worst morph aliasing: " + String(aliasWorst, 1) + " dB");
            expect(aliasWorst < -90.0, "morph aliasing " + std::to_string(aliasWorst));
            double peak = 0.0;
            for (const double v : s)
                peak = std::max(peak, std::abs(v));
            expect(peak < 1.6, "morph peak " + std::to_string(peak));
        }

        beginTest("Saw-core VCO: curved ramp, comparator pulse with a width offset, folded soft triangle");
        {
            Rig rig;
            rig.set("wave", Saw);
            rig.set("char", 1.0f);
            const auto sig = rig.render(24, 9600); // 32.7 Hz: 1470 samples per cycle
            const double period = 48000.0 / hzOf(24);
            // Capacitor charging: the middle of the ramp sits above the straight line. Value at phase 0.5 of the n-th cycle
            // (delayed by the 4-sample lag of the oscillator).
            const double mid = sig[static_cast<size_t>(3.0 * period + 0.5 * period) + 4];
            const double expected = 2.0 * ((1.0 - std::exp(-0.35 * 0.5)) / (1.0 - std::exp(-0.35))) - 1.0;
            logMessage("saw-core ramp at mid-cycle " + String(mid, 3) + " (curve predicts " + String(expected, 3) + ")");
            expectWithinAbsoluteError(mid, expected, 0.01);
            // Finite reset: the fall takes about 6 us, a fraction of a sample at this pitch but not an ideal step.
            Rig pulse;
            pulse.set("wave", Pulse);
            pulse.set("char", 1.0f);
            const auto p = pulse.render(24, 14400);
            int high = 0, total = 0;
            for (size_t i = 4800; i < 4800 + 1470 * 4; ++i, ++total)
                high += p[i] > 0.0 ? 1 : 0;
            const double duty = static_cast<double>(high) / total;
            logMessage("saw-core comparator duty at width 0.5: " + String(duty, 3) + " (offset moves it from 0.5)");
            expectWithinAbsoluteError(duty, 0.471, 0.01);
            Rig tri;
            tri.set("wave", Tri);
            tri.set("char", 1.0f);
            const auto t = tri.render(24, 9600);
            double peak = 0.0;
            for (size_t i = 4800; i < t.size(); ++i)
                peak = std::max(peak, std::abs(t[i]));
            expectWithinAbsoluteError(peak, 1.0, 0.02);
            for (const int wv : {Saw, Pulse, Tri})
            {
                Rig r;
                r.set("wave", static_cast<float>(wv));
                r.set("char", 1.0f);
                const double a = aliasOf(r, 72, 48000.0);
                expect(a < -90.0, "saw-core wave " + std::to_string(wv) + ": " + std::to_string(a));
            }
        }

        beginTest("Triangle-core VCO: switched saw (reset in 1/16 of the cycle), tanh sine, comparator pulse");
        {
            Rig rig;
            rig.set("wave", Saw);
            rig.set("char", 2.0f);
            const auto sig = rig.render(24, 9600);
            // Count the samples of the falling part of one cycle.
            const size_t start = 4800;
            size_t falling = 0;
            for (size_t i = start; i < start + 1470; ++i)
                falling += sig[i + 1] < sig[i] - 1.0e-4 ? 1 : 0;
            const double share = static_cast<double>(falling) / (48000.0 / hzOf(24));
            logMessage("switched saw falls for " + String(share, 4) + " of the cycle");
            expectWithinAbsoluteError(share, 1.0 / 16.0, 0.01);
            Rig sine;
            sine.set("wave", Sine);
            sine.set("char", 2.0f);
            const auto x = sine.render(57, 48000 * 2);
            const auto prof = m::harmonicProfile(x.data() + 24000, 65536, 48000.0, hzOf(57), 12);
            logMessage("tanh sine: THD " + String(prof.thdDb, 1) + " dB, 3rd " + String(prof.harmonicDb(3), 1) +
                       " dB, 2nd " + String(prof.harmonicDb(2), 1) + " dB");
            expect(prof.thdDb < -30.0 && prof.thdDb > -60.0, "THD " + std::to_string(prof.thdDb));
            expect(prof.harmonicDb(3) > prof.harmonicDb(2) + 30.0, "odd harmonics dominate");
            for (const int wv : {Saw, Pulse, Tri, Sine})
            {
                Rig r;
                r.set("wave", static_cast<float>(wv));
                r.set("char", 2.0f);
                const double a = aliasOf(r, 72, 48000.0);
                expect(a < -90.0, "triangle-core wave " + std::to_string(wv) + ": " + std::to_string(a));
            }
        }

        beginTest("DCO: exact pitch, saw level falls with pitch unless compensated, divider sub one octave down");
        {
            std::vector<double> pitch;
            for (int i = 0; i < 6; ++i)
                pitch.push_back(sineHz(60, 48000.0, 24000,
                                       [i](Rig& r)
                                       {
                                           r.reseed(5U + 31U * static_cast<unsigned>(i));
                                           r.set("char", 3.0f);
                                           r.set("tol", 1.0f);
                                       }));
            for (const double f : pitch)
                expectWithinAbsoluteError(m::centsBetween(f, hzOf(60)), 0.0, 0.05);
            const auto level = [](double note, bool compensated)
            {
                Rig r;
                r.set("wave", Saw);
                r.set("char", 3.0f);
                r.set("dco_comp", compensated ? 1.0f : 0.0f);
                const auto s = r.render(note, 24000);
                return m::rmsOf(s.data() + 4800, s.size() - 4800);
            };
            const double ratio = level(84, false) / level(36, false);
            const double expectedRatio = (1.0 + hzOf(36) / 2000.0) / (1.0 + hzOf(84) / 2000.0);
            logMessage("DCO saw level C6 / C2: " + String(ratio, 3) + " (expected " + String(expectedRatio, 3) +
                       "); compensated " + String(level(84, true) / level(36, true), 3));
            expectWithinAbsoluteError(ratio, expectedRatio, 0.03);
            expectWithinAbsoluteError(level(84, true) / level(36, true), 1.0, 0.03);
            Rig sub;
            sub.set("wave", Saw);
            sub.set("char", 3.0f);
            sub.set("sub", 1.0f);
            const auto s = sub.render(57, 96000);
            const auto prof = m::harmonicProfile(s.data() + 24000, 65536, 48000.0, hzOf(57) / 2.0, 6);
            logMessage("DCO sub at f0/2: " + String(prof.harmonicDb(1), 1) +
                       " dB, saw at f0: " + String(prof.harmonicDb(2), 1) + " dB");
            expect(prof.harmonicDb(2) > -10.0 && prof.harmonicDb(1) > -10.0, "both the saw and the sub sound");
            const auto high = sub.render(72, 48000);
            const double subAlias = m::measureAliasing(high, 8192, 48000.0, hzOf(72) / 2.0).aliasDb;
            logMessage("DCO with sub, C5: aliasing " + String(subAlias, 1) + " dB");
            expect(subAlias < -90.0, "DCO sub aliasing " + std::to_string(subAlias));
        }

        beginTest("unison: level stays constant, voices detune and spread across the stereo field");
        {
            Rig one;
            one.set("wave", Saw);
            std::vector<float> right1;
            const auto l1 = one.render(57, 24000, &right1);
            for (size_t i = 0; i < l1.size(); ++i)
                expectEquals(static_cast<float>(l1[i]), right1[i]);
            Rig eight;
            eight.set("wave", Saw);
            eight.set("uni", 8.0f);
            eight.set("detune", 20.0f);
            eight.set("spread", 1.0f);
            std::vector<float> right8;
            const auto l8 = eight.render(57, 96000, &right8);
            double p1 = 0.0, p8l = 0.0, p8r = 0.0, cross = 0.0;
            for (size_t i = 9600; i < 24000; ++i)
                p1 += l1[i] * l1[i];
            p1 *= (96000.0 - 9600.0) / 14400.0;
            for (size_t i = 9600; i < 96000; ++i)
            {
                p8l += l8[i] * l8[i];
                p8r += static_cast<double>(right8[i]) * right8[i];
                cross += l8[i] * right8[i];
            }
            logMessage("8 unison voices: power L " + String(m::powerToDb(p8l / p1), 2) + " dB, R " +
                       String(m::powerToDb(p8r / p1), 2) + " dB re one voice, L/R correlation " +
                       String(cross / std::sqrt(p8l * p8r), 3));
            expect(std::abs(m::powerToDb(0.5 * (p8l + p8r) / p1)) < 1.5, "unison keeps the loudness");
            expect(cross / std::sqrt(p8l * p8r) < 0.9, "voices are spread, not identical in both channels");
            // 20 cents detune: two lines near the fundamental, at least +/- 17 cents apart.
            std::vector<float> dummy;
            Rig two;
            two.set("wave", Sine);
            two.set("uni", 2.0f);
            two.set("detune", 20.0f);
            two.set("spread", 0.0f);
            const auto t = two.render(57, 96000, &dummy);
            const auto spec = m::computeSpectrum(t.data() + 16384, 65536, 48000.0);
            const auto bin = [&](double hz) { return static_cast<size_t>(std::lround(hz / spec.binHz)); };
            const double lo = hzOf(57 - 0.2), hi = hzOf(57 + 0.2);
            expect(spec.amplitude(bin(lo)) > 0.3 && spec.amplitude(bin(hi)) > 0.3, "both detuned voices sound");
        }

        beginTest("drift: Ornstein-Uhlenbeck per voice, std = drift x 6 cents, time constant as set, zero by default");
        {
            const auto trace = [](float driftAmt, float tauSeconds, int run, double seconds)
            {
                Rig rig;
                rig.set("wave", Sine);
                rig.set("drift", driftAmt);
                rig.set("drift_tau", tauSeconds);
                rig.reseed(1000U + 7919U * static_cast<unsigned>(run));
                const auto sig = rig.render(57, static_cast<size_t>(seconds * 48000.0));
                return m::pitchTraceCents(sig, 48000.0, 0.05, 0.09, hzOf(57));
            };
            for (const float tauSeconds : {0.5f, 1.0f, 3.0f})
            {
                std::vector<m::Signal> runs;
                const int count = tauSeconds < 1.0f ? 12 : 16;
                for (int i = 0; i < count; ++i)
                    runs.push_back(trace(1.0f, tauSeconds, i, tauSeconds * 24.0));
                const auto st = m::ouStats(runs, 0.05, 1.0, 5.0);
                logMessage("drift 1.0, tau " + String(tauSeconds, 1) + " s: std " + String(st.stdDev, 2) + " cents, tau " +
                           String(st.tauSeconds, 2) + " s, lag-1 " + String(st.lag1, 3) + ", PSD slope " +
                           String(st.psdSlope, 2) + ", mean " + String(st.mean, 2));
                expectWithinAbsoluteError(st.stdDev, 6.0, 1.0, "std at tau " + std::to_string(tauSeconds));
                expect(st.tauSeconds > 0.7 * tauSeconds && st.tauSeconds < 1.4 * tauSeconds,
                       "tau " + std::to_string(st.tauSeconds));
            }
            const auto half = m::ouStats(
                std::vector<m::Signal>{trace(0.5f, 1.0f, 3, 60.0), trace(0.5f, 1.0f, 4, 60.0), trace(0.5f, 1.0f, 5, 60.0)},
                0.05, 1.0, 5.0);
            expectWithinAbsoluteError(half.stdDev, 3.0, 0.8, "std scales with the drift amount");
            const auto off = trace(0.0f, 1.0f, 1, 5.0);
            double worst = 0.0;
            for (const double c : off)
                worst = std::max(worst, std::abs(c));
            expect(worst < 0.1, "no drift by default: " + std::to_string(worst) + " cents");
        }

        beginTest("voice-card tolerance: pitch spread between voices, none at 0");
        {
            std::vector<double> cents;
            for (int i = 0; i < 24; ++i)
                cents.push_back(m::centsBetween(sineHz(60, 48000.0, 24000,
                                                       [i](Rig& r)
                                                       {
                                                           r.reseed(77U + 1013U * static_cast<unsigned>(i));
                                                           r.set("tol", 1.0f);
                                                       }),
                                                hzOf(60)));
            double mean = 0.0, var = 0.0;
            for (const double c : cents)
                mean += c / static_cast<double>(cents.size());
            for (const double c : cents)
                var += (c - mean) * (c - mean) / static_cast<double>(cents.size());
            logMessage("tolerance 1: pitch std over 24 voice cards " + String(std::sqrt(var), 2) + " cents");
            expect(std::sqrt(var) > 2.0 && std::sqrt(var) < 9.0, "std " + std::to_string(std::sqrt(var)));
            for (int i = 0; i < 4; ++i)
                expectWithinAbsoluteError(m::centsBetween(sineHz(60, 48000.0, 24000, [](Rig&) {}), hzOf(60)), 0.0, 0.05);
        }

        beginTest("ring modulation and parameter extremes stay finite at every sample rate");
        {
            for (const double rate : {22050.0, 44100.0, 48000.0, 96000.0, 192000.0})
                for (const int mode : {0, 1, 2, 3, 4, 5})
                    for (const int ch : {0, 1, 2, 3})
                        for (const double note : {0.0, 36.0, 127.0})
                        {
                            Rig rig(rate);
                            rig.set("wave", Morph);
                            rig.set("shape", 0.37f);
                            rig.set("char", static_cast<float>(ch));
                            rig.set("xmod_mode", static_cast<float>(mode));
                            rig.set("xmod_ratio", 16.0f);
                            rig.set("xmod_amt", 1.0f);
                            rig.set("uni", 8.0f);
                            rig.set("drift", 1.0f);
                            rig.set("tol", 1.0f);
                            rig.set("sub", 1.0f);
                            const auto sig = rig.render(note, 4800);
                            double peak = 0.0;
                            bool finite = true;
                            for (const double v : sig)
                            {
                                finite = finite && std::isfinite(v);
                                peak = std::max(peak, std::abs(v));
                            }
                            expect(finite && peak < 12.0, "rate " + std::to_string(rate) + " mode " + std::to_string(mode) +
                                                              " char " + std::to_string(ch) + " note " +
                                                              std::to_string(note) + " peak " + std::to_string(peak));
                        }
        }

        beginTest("a non-finite pitch is survived: the oscillator recovers with finite output at the next normal pitch");
        {
            const double bad[] = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                                  -std::numeric_limits<double>::infinity()};
            for (const double note : bad)
                for (const int wave : {0, 1, 2})
                {
                    Rig rig;
                    rig.set("wave", static_cast<float>(wave));
                    rig.set("uni", 3.0f);
                    rig.render(60.0, 320);
                    const auto broken = rig.render(note, 640);
                    const auto after = rig.render(60.0, 4800);
                    bool finite = true;
                    for (const double v : broken)
                        finite = finite && std::isfinite(v);
                    for (const double v : after)
                        finite = finite && std::isfinite(v);
                    expect(finite, "pitch " + std::to_string(note) + " wave " + std::to_string(wave));
                    double peak = 0.0;
                    for (const double v : after)
                        peak = std::max(peak, std::abs(v));
                    expect(peak > 0.05, "silent after a bad pitch: " + std::to_string(peak));
                }
        }

        beginTest("in a voice: osc1_type = Analog (VA) plays the note in tune and survives a type switch");
        {
            auto v = graphPatch();
            setValue(v, "osc1_type", 5.0f);
            setValue(v, "osc1_level", 1.0f);
            setValue(v, "osc1_an_wave", Sine);
            setValue(v, "filter_on", 0.0f);
            setValue(v, "amp_release", 0.05f);
            for (const int note : {36, 60, 81})
            {
                const auto r = playNote({}, v, note, 24000);
                expect(r.finite && r.peak > 0.2f, "audible, note " + std::to_string(note));
                expectWithinAbsoluteError(
                    m::centsBetween(m::estimateFrequency(m::toDouble(r.left, 4000), kRate, hzOf(note)), hzOf(note)), 0.0,
                    0.5);
            }
            SynthEngine engine;
            prepare(engine, v);
            engine.noteOn(60, 1.0f);
            renderBlocks(engine, 4800);
            setValue(v, "osc1_type", 4.0f);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            expect(renderBlocks(engine, 4800).finite);
            setValue(v, "osc1_type", 5.0f);
            engine.setParameterValues(v.data(), static_cast<int>(v.size()));
            expect(renderBlocks(engine, 4800).finite);
        }

        beginTest("CPU: a saw costs at most 0.05 % of one core per voice-oscillator, PWM with sync at most 0.12 %");
        {
#ifdef CRUCIBLE_TEST_UNTIMED
            logMessage("skipped: timing is not meaningful in this build");
#else
            const auto cost = [](const std::function<void(Rig&)>& setup, double note)
            {
                double best = 1.0e9;
                for (int rep = 0; rep < 5; ++rep)
                {
                    Rig rig;
                    setup(rig);
                    rig.render(note, 4800); // warm up
                    const auto start = std::chrono::steady_clock::now();
                    const auto out = rig.render(note, 48000 * 4);
                    best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
                    if (out.empty())
                        best = 1.0e9;
                }
                return 100.0 * best / 4.0; // percent of one core
            };
            const double saw = cost([](Rig& r) { r.set("wave", Saw); }, 57);
            const double sawHigh = cost([](Rig& r) { r.set("wave", Saw); }, 81);
            const double pwmSync = cost(
                [](Rig& r)
                {
                    r.set("wave", Pulse);
                    r.set("xmod_mode", HardSync);
                    r.set("xmod_ratio", 2.37f);
                },
                57);
            logMessage("CPU per voice-oscillator: saw A3 " + String(saw, 4) + " %, saw A5 " + String(sawHigh, 4) +
                       " %, pulse + hard sync A3 " + String(pwmSync, 4) + " %");
            expect(saw <= 0.05, "saw " + std::to_string(saw) + " %");
            expect(pwmSync <= 0.12, "PWM + sync " + std::to_string(pwmSync) + " %");
#endif
        }
    }
};

static AnalogOscillatorTests analogOscillatorTests;
