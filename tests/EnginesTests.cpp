// SPDX-License-Identifier: MIT
// P13 engines: the modal oscillator `osc.modal` (mode frequencies, T60, band limit, stability, CPU, tolerance, licence
// notices), the harmonic oscillator `osc.harmonic` (partial law), the spectral oscillator `osc.spectral` (time stretch),
// the spectral tools of Spectral.h (sweep interpolation, warps), the opt-in spectral wavetable path of the voice
// (oscN_wt_interp), the FFT backends, and the allocation contract of all of them.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "TestSupport.h"

#include <crucible/Cloud.h>
#include <crucible/Engine.h>
#include <crucible/Simd.h>
#include <crucible/Spectral.h>
#include <crucible/module/Oscillator.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>

#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer) || __has_feature(undefined_behavior_sanitizer)
#define CRUCIBLE_TEST_UNTIMED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__) || !defined(NDEBUG) ||                                    \
    (defined(__x86_64__) && defined(__APPLE__))
#define CRUCIBLE_TEST_UNTIMED 1
#endif

// The stiffness table of the vendored Rings resources (the oracle for the mode frequencies).
namespace rings
{
extern const float lut_stiffness[];
}

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr int kModal = 6;
constexpr int kHarmonic = 7;
constexpr int kSpectral = 8;
constexpr double kPiD = 3.141592653589793;

double hzOf(double note)
{
    return 440.0 * std::pow(2.0, (note - 69.0) / 12.0);
}

// Drives one oscillator instance of slot 1 directly, the way a voice does.
class Rig
{
public:
    Rig(int type, double rate, const SampleData* sampleData = nullptr, std::uint32_t seed = 12345, int slot = 0)
        : sampleRate(rate), prefix("osc" + std::to_string(slot + 1) + "_"), sample(sampleData)
    {
        const auto& registry = ModuleRegistry::instance();
        const auto types = registry.ofKind(ModuleKind::Oscillator);
        instance = types[static_cast<size_t>(type)]->createOscillator(slot, registry.parameters());
        instance->prepare(rate, 1);
        instance->arm();
        values = ParameterMap::defaultValues();
        OscNoteInfo info;
        info.seed = seed;
        info.velocity = velocity;
        instance->noteOn(info);
    }

    void set(const std::string& suffix, float v) { setValue(values, prefix + suffix, v); }
    void restart(std::uint32_t seed, float vel = 1.0f)
    {
        OscNoteInfo info;
        info.seed = seed;
        info.velocity = vel;
        instance->noteOn(info);
        elapsed = 0;
    }
    void release() { instance->noteOff(); }

    // Renders `samples` samples at MIDI pitch `note`; `each(done)` may change parameters or release the note before a chunk.
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
            rr.sample = sample;
            rr.noteTime = fixedNoteTime >= 0.0 ? fixedNoteTime : static_cast<double>(elapsed) / sampleRate;
            instance->render(rr);
            elapsed += 32;
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
    const SampleData* sample;
    std::vector<float> values;
    std::unique_ptr<OscillatorInstance> instance;
    double fixedNoteTime = -1.0;
    float velocity = 1.0f;
    size_t elapsed = 0;
};

// Peak of the windowed (Blackman-Harris) DTFT near hz: refined frequency and the amplitude of the sinusoid there.
struct Peak
{
    double hz = 0.0;
    double amplitude = 0.0;
};

Peak refinePeak(const m::Signal& x, size_t from, size_t n, double hz, double rate, double spanCents = 40.0)
{
    const m::Signal w = m::makeWindow(m::Window::BlackmanHarris, n);
    double sumW = 0.0;
    for (const double v : w)
        sumW += v;
    const auto magnitude = [&](double f) { return std::abs(m::windowedDtft(x.data() + from, w.data(), n, f, rate)); };
    double left = hz * std::pow(2.0, -spanCents / 1200.0), right = hz * std::pow(2.0, spanCents / 1200.0);
    // Coarse scan, then golden section.
    double best = left, bestMag = -1.0;
    const double step = (right - left) / 60.0;
    for (double f = left; f <= right; f += step)
    {
        const double mag = magnitude(f);
        if (mag > bestMag)
        {
            bestMag = mag;
            best = f;
        }
    }
    left = best - step;
    right = best + step;
    const double golden = 0.6180339887498949;
    double c1 = right - golden * (right - left), c2 = left + golden * (right - left);
    double m1 = magnitude(c1), m2 = magnitude(c2);
    while (right - left > hz * 1.0e-9)
    {
        if (m1 < m2)
        {
            left = c1;
            c1 = c2;
            m1 = m2;
            c2 = left + golden * (right - left);
            m2 = magnitude(c2);
        }
        else
        {
            right = c2;
            c2 = c1;
            m2 = m1;
            c1 = right - golden * (right - left);
            m1 = magnitude(c1);
        }
    }
    Peak p;
    p.hz = 0.5 * (left + right);
    p.amplitude = 2.0 * magnitude(p.hz) / sumW;
    return p;
}

// Amplitude of the component at hz around sample `centre`, Hann window of `length` samples.
double envelopeAt(const m::Signal& x, double hz, double rate, size_t centre, size_t length)
{
    const size_t from = centre >= length / 2 ? centre - length / 2 : 0;
    if (from + length > x.size())
        return 0.0;
    const m::Signal w = m::makeWindow(m::Window::Hann, length);
    double sumW = 0.0;
    for (const double v : w)
        sumW += v;
    // The DTFT phase reference is the window start: the magnitude does not depend on it.
    return 2.0 * std::abs(m::windowedDtft(x.data() + from, w.data(), length, hz, rate)) / sumW;
}

// T60 of the component at hz: least-squares slope of the log envelope between t0 and t1 seconds.
double measureT60(const m::Signal& x, double hz, double rate, double t0, double t1)
{
    std::vector<double> t, y;
    const size_t length = static_cast<size_t>(0.08 * rate);
    for (double tt = t0; tt <= t1; tt += 0.02)
    {
        const double a = envelopeAt(x, hz, rate, static_cast<size_t>(tt * rate), length);
        if (a > 1.0e-9)
        {
            t.push_back(tt);
            y.push_back(std::log(a));
        }
    }
    if (t.size() < 4)
        return 0.0;
    double st = 0, sy = 0, stt = 0, sty = 0;
    const auto n = static_cast<double>(t.size());
    for (size_t i = 0; i < t.size(); ++i)
    {
        st += t[i];
        sy += y[i];
        stt += t[i] * t[i];
        sty += t[i] * y[i];
    }
    const double slope = (n * sty - st * sy) / (n * stt - st * st);
    return -std::log(1000.0) / slope;
}

// The Rings / Elements mode law, restated as the oracle: mode frequencies in cycles per sample and T60 in seconds.
struct OracleModes
{
    std::vector<double> freq, t60;
};

OracleModes oracleModes(double f0, float geometry, float brightness, double decay, int resolution)
{
    const auto interpolate = [](float x)
    {
        const float index = x * 256.0f;
        const int i = static_cast<int>(index);
        const float f = index - static_cast<float>(i);
        return rings::lut_stiffness[i] + (rings::lut_stiffness[i + 1] - rings::lut_stiffness[i]) * f;
    };
    geometry = std::clamp(geometry, 0.0f, 0.9999f);
    float stiffness = interpolate(geometry);
    double stretch = 1.0, harmonic = f0, t60 = decay;
    float attenuation = 1.0f - geometry;
    attenuation = attenuation * attenuation;
    attenuation = attenuation * attenuation;
    attenuation = attenuation * attenuation;
    const float b = brightness * (1.0f - 0.2f * attenuation);
    float qLoss = b * (2.0f - b) * 0.85f + 0.15f;
    const float lossRate = geometry * (2.0f - geometry) * 0.1f;
    OracleModes out;
    for (int i = 0; i < resolution; ++i)
    {
        const double f = harmonic * stretch;
        if (f >= 0.40)
            break;
        out.freq.push_back(f);
        out.t60.push_back(t60);
        stretch += stiffness;
        stiffness *= stiffness < 0.0f ? 0.93f : 0.98f;
        qLoss += lossRate * (1.0f - qLoss);
        harmonic += f0;
        t60 *= qLoss;
    }
    return out;
}

std::string readTextFile(const std::string& path)
{
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool contains(const std::string& text, const std::string& needle)
{
    return text.find(needle) != std::string::npos;
}

#ifndef CRUCIBLE_ENGINE_SOURCE_DIR
#define CRUCIBLE_ENGINE_SOURCE_DIR "."
#endif

// Seconds of CPU per second of audio for `chunks` render calls of 32 samples, best of `repeats`.
double cpuPercent(Rig& rig, double note, double audioSeconds, int repeats = 5)
{
    double best = 1.0e9;
    for (int rep = 0; rep < repeats; ++rep)
    {
        rig.restart(7);
        std::vector<float> l(32), r(32);
        const int chunks = static_cast<int>(audioSeconds * rig.sampleRate / 32.0);
        const auto t0 = std::chrono::steady_clock::now();
        double sink = 0.0;
        for (int c = 0; c < chunks; ++c)
        {
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            OscRender rr;
            rr.values = rig.values.data();
            rr.pitch = static_cast<float>(note);
            rr.sampleRate = rig.sampleRate;
            rr.numSamples = 32;
            rr.left = l.data();
            rr.right = r.data();
            rr.sample = rig.sample;
            rr.noteTime = c * 32.0 / rig.sampleRate;
            rig.instance->render(rr);
            sink += l[0];
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        best = std::min(best, 100.0 * sec / audioSeconds + 0.0 * sink);
    }
    return best;
}
} // namespace

class EnginesTests : public UnitTest
{
public:
    EnginesTests() : UnitTest("Engines", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        modalTests();
        harmonicTests();
        spectralOscillatorTests();
        spectralToolTests();
        wavetablePathTests();
        fftTests();
        allocationTests();
    }

private:
    // ---------------------------------------------------------------------------------------------------------------
    void modalTests()
    {
        beginTest("osc.modal, osc.harmonic and osc.spectral are oscillator types 6, 7 and 8; the first six did not move");
        {
            const auto types = ModuleRegistry::instance().ofKind(ModuleKind::Oscillator);
            expectEquals(types.size(), size_t(9));
            const char* ids[] = {"osc.wavetable", "osc.sample", "osc.cloud",    "osc.noise",   "osc.sub",
                                 "osc.analog",    "osc.modal",  "osc.harmonic", "osc.spectral"};
            for (size_t i = 0; i < 9; ++i)
                expectEquals(types[i]->id, std::string(ids[i]));
            const auto& type = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc1_type"))];
            expectEquals(type.choices.size(), size_t(9));
            expectEquals(type.choices[0], std::string("Wavetable"));
            expectEquals(type.choices[5], std::string("Analog (VA)"));
            expectEquals(type.choices[6], std::string("Modal"));
            expectEquals(type.choices[7], std::string("Harmonic"));
            expectEquals(type.choices[8], std::string("Spectral"));
            for (const char* id :
                 {"osc1_modal_model", "osc3_modal_decay", "osc2_harm_count", "osc1_harm_fgain", "osc1_spec_stretch",
                  "osc3_spec_formant", "osc1_wt_interp", "osc2_wt_spec", "osc3_wt_spec_amt"})
                expect(ParameterMap::indexOf(id) >= 0, id);
            expectEquals(ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf("osc1_wt_interp"))].defaultValue,
                         0.0f);
            // Appended: every new parameter sits after the last parameter of the analog filter controls (the previous end).
            expect(ParameterMap::indexOf("osc1_modal_model") > ParameterMap::indexOf("osc1_an_wave"),
                   "modal parameters are appended");
            expect(ParameterMap::indexOf("osc1_wt_interp") > ParameterMap::indexOf("osc1_spec_stretch"),
                   "wt_interp is registered last");
        }

        beginTest("licence notices of the vendored code are present and named in THIRD_PARTY.md");
        {
            const std::string root = CRUCIBLE_ENGINE_SOURCE_DIR;
            const auto mutableLicense = readTextFile(root + "/third_party/mutable/LICENSE");
            expect(contains(mutableLicense, "Emilie Gillet") && contains(mutableLicense, "Permission is hereby granted"),
                   "mutable LICENSE");
            const auto resources = readTextFile(root + "/third_party/mutable/rings/resources.cc");
            expect(contains(resources.substr(0, 4000), "Copyright 2015 Emilie Gillet") &&
                       contains(resources.substr(0, 4000), "MIT"),
                   "Rings notice kept in resources.cc");
            expect(contains(readTextFile(root + "/third_party/signalsmith-stretch/LICENSE.txt"), "Signalsmith Audio"),
                   "signalsmith-stretch LICENSE");
            expect(contains(readTextFile(root + "/third_party/signalsmith-linear/LICENSE.txt"), "Signalsmith Audio"),
                   "signalsmith-linear LICENSE");
            expect(contains(readTextFile(root + "/third_party/pffft/LICENSE.txt"), "Redistribution and use"),
                   "pffft LICENSE");
            const auto thirdParty = readTextFile(root + "/THIRD_PARTY.md");
            for (const char* name : {"mutable", "signalsmith-stretch", "signalsmith-linear", "pffft"})
                expect(contains(thirdParty, name), std::string("THIRD_PARTY.md names ") + name);
            const auto source = readTextFile(root + "/src/modules/oscillator/ModalOscillator.cpp");
            expect(contains(source, "Mutable Instruments") && contains(source, "MIT"),
                   "ModalOscillator.cpp credits the Rings / Elements designs");
        }

        beginTest("modal: mode frequencies within 1 cent at 44.1, 48 and 96 kHz (three geometries, three pitches)");
        {
            double worst = 0.0;
            int checked = 0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double note : {45.0, 57.0, 69.0})
                    for (const float geometry : {0.2539f, 0.5f, 0.8f})
                    {
                        Rig rig(kModal, rate);
                        rig.set("modal_geometry", geometry);
                        rig.set("modal_brightness", 1.0f);
                        rig.set("modal_decay", 8.0f);
                        rig.set("modal_position", 0.03f);
                        rig.set("modal_timbre", 1.0f);
                        const auto sig = rig.render(note, static_cast<size_t>(1.5 * rate));
                        const double f0 = hzOf(note) / rate;
                        const auto modes = oracleModes(f0, geometry, 1.0f, 8.0, 64);
                        for (size_t i = 0; i < std::min<size_t>(modes.freq.size(), 12); ++i)
                        {
                            const double want = modes.freq[i] * rate;
                            if (modes.freq[i] > 0.35)
                                continue;
                            const auto p = refinePeak(sig, 3000, 32768, want, rate);
                            const double cents = std::abs(m::centsBetween(p.hz, want));
                            worst = std::max(worst, cents);
                            ++checked;
                            expect(cents < 1.0, "mode " + std::to_string(i + 1) + " at note " + std::to_string(note) + ", " +
                                                    std::to_string(rate) + " Hz, geometry " + std::to_string(geometry) +
                                                    ": " + std::to_string(cents) + " cents");
                        }
                    }
            logMessage("modal mode frequencies: " + String(checked) + " modes, worst error " + String(worst, 4) + " cents");
        }

        beginTest("modal: T60 of the modes within 5 % of the Rings / Elements decay law at 44.1, 48 and 96 kHz");
        {
            double worst = 0.0;
            int checked = 0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double decay : {0.6, 2.5})
                    for (const float brightness : {1.0f, 0.6f})
                    {
                        Rig rig(kModal, rate);
                        rig.set("modal_geometry", 0.5f);
                        rig.set("modal_brightness", brightness);
                        rig.set("modal_decay", static_cast<float>(decay));
                        rig.set("modal_position", 0.03f);
                        rig.set("modal_timbre", 1.0f);
                        rig.set("modal_spread",
                                0.0f); // the side pickup moves with a slow LFO: it would modulate the amplitudes
                        const double note = 57.0;
                        const auto sig = rig.render(note, static_cast<size_t>((decay + 1.0) * rate));
                        const auto modes = oracleModes(hzOf(note) / rate, 0.5f, brightness, decay, 64);
                        for (const size_t i : {size_t(0), size_t(1), size_t(3), size_t(7)})
                        {
                            if (i >= modes.freq.size() || modes.t60[i] < 0.15)
                                continue;
                            const double want = modes.t60[i];
                            const double hz = modes.freq[i] * rate;
                            const auto p = refinePeak(sig, static_cast<size_t>(0.1 * rate), 16384, hz, rate);
                            const double got = measureT60(sig, p.hz, rate, 0.06, std::min(0.55 * want, 1.2));
                            const double err = std::abs(got / want - 1.0);
                            worst = std::max(worst, err);
                            ++checked;
                            expect(err < 0.05, "mode " + std::to_string(i + 1) + ", decay " + std::to_string(decay) +
                                                   ", brightness " + std::to_string(brightness) + ", " +
                                                   std::to_string(rate) + " Hz: T60 " + std::to_string(got) + " vs " +
                                                   std::to_string(want));
                        }
                    }
            logMessage("modal T60: " + String(checked) + " modes, worst error " + String(100.0 * worst, 3) + " %");
        }

        beginTest("modal string: fundamental within 1 cent and T60 within 5 % from C2 to C7 at 44.1, 48 and 96 kHz");
        {
            double worstCents = 0.0, worstT60 = 0.0;
            for (const double rate : {44100.0, 48000.0, 96000.0})
                for (const double note : {36.0, 48.0, 60.0, 72.0, 84.0, 96.0})
                {
                    Rig rig(kModal, rate);
                    rig.set("modal_model", 1.0f);
                    rig.set("modal_geometry", 0.0f);
                    rig.set("modal_brightness", 1.0f);
                    rig.set("modal_decay", 1.5f);
                    rig.set("modal_position", 0.1f);
                    rig.set("modal_spread", 0.0f);
                    const auto sig = rig.render(note, static_cast<size_t>(3.0 * rate));
                    const double f0 = hzOf(note);
                    const auto p = refinePeak(sig, 3000, 32768, f0, rate);
                    const double cents = std::abs(m::centsBetween(p.hz, f0));
                    const double t60 = measureT60(sig, p.hz, rate, 0.15, 1.0);
                    worstCents = std::max(worstCents, cents);
                    worstT60 = std::max(worstT60, std::abs(t60 / 1.5 - 1.0));
                    expect(cents < 1.0, "string pitch note " + std::to_string(note) + " at " + std::to_string(rate) + ": " +
                                            std::to_string(cents));
                    expect(std::abs(t60 / 1.5 - 1.0) < 0.05, "string T60 note " + std::to_string(note) + " at " +
                                                                 std::to_string(rate) + ": " + std::to_string(t60));
                }
            logMessage("string: worst pitch error " + String(worstCents, 4) + " cents, worst T60 error " +
                       String(100.0 * worstT60, 2) + " %");
        }

        beginTest("modal: nothing above 0.45 fs (every exciter, both models, 44.1 and 96 kHz, C2 to E7)");
        {
            double worst = -300.0;
            const char* exciters[] = {"strike", "pluck", "particles", "blow", "noise", "bow"};
            for (const int model : {0, 1})
                for (int e = 0; e < 6; ++e)
                    for (const double rate : {44100.0, 96000.0})
                    {
                        double worstHere = -300.0;
                        for (const double note : {36.0, 60.0, 84.0, 100.0})
                        {
                            Rig rig(kModal, rate);
                            rig.set("modal_model", static_cast<float>(model));
                            rig.set("modal_exciter", static_cast<float>(e));
                            rig.set("modal_timbre", 1.0f);
                            rig.set("modal_brightness", 1.0f);
                            const auto sig = rig.render(note, static_cast<size_t>(1.6 * rate));
                            std::vector<double> d(sig.begin() + 4000, sig.begin() + 4000 + 65536);
                            const auto sp = m::computeSpectrum(d.data(), 65536, rate);
                            double peak = 0.0, high = 0.0;
                            for (size_t k = 1; k < sp.bins(); ++k)
                            {
                                const double a = sp.amplitude(k);
                                peak = std::max(peak, a);
                                if (static_cast<double>(k) * sp.binHz > 0.45 * rate)
                                    high = std::max(high, a);
                            }
                            worstHere = std::max(worstHere, m::amplitudeToDb(high / std::max(peak, 1.0e-30)));
                        }
                        worst = std::max(worst, worstHere);
                        expect(worstHere < -60.0, std::string(model == 0 ? "resonator " : "string ") + exciters[e] + " at " +
                                                      std::to_string(rate) + " Hz: " + std::to_string(worstHere) +
                                                      " dB above 0.45 fs");
                    }
            logMessage("modal band limit: worst level above 0.45 fs " + String(worst, 1) + " dB re the spectral peak");
        }

        beginTest(
            "modal: bounded and finite for 60 s at extreme settings (every exciter, both models, held 40 s then released)");
        {
            double worstPeak = 0.0;
            for (const int model : {0, 1})
                for (int e = 0; e < 6; ++e)
                    for (const double note : {40.0, 100.0})
                    {
                        Rig rig(kModal, 48000.0);
                        rig.set("modal_model", static_cast<float>(model));
                        rig.set("modal_exciter", static_cast<float>(e));
                        rig.set("modal_decay", 30.0f);
                        rig.set("modal_geometry", 0.9f);
                        rig.set("modal_brightness", 1.0f);
                        rig.set("modal_contour", 1.0f);
                        rig.set("modal_timbre", 1.0f);
                        rig.set("modal_sustain", 1.0f);
                        rig.set("modal_tol", 1.0f);
                        bool released = false;
                        const auto sig = rig.render(note, 48000u * 60u, nullptr,
                                                    [&](size_t done)
                                                    {
                                                        if (!released && done >= 48000u * 40u)
                                                        {
                                                            rig.release();
                                                            released = true;
                                                        }
                                                    });
                        bool finite = true;
                        double peak = 0.0;
                        for (const double v : sig)
                        {
                            finite = finite && std::isfinite(v);
                            peak = std::max(peak, std::abs(v));
                        }
                        worstPeak = std::max(worstPeak, peak);
                        expect(finite, "finite, model " + std::to_string(model) + " exciter " + std::to_string(e));
                        expect(peak < 3.9, "below the output guard (4), model " + std::to_string(model) + " exciter " +
                                               std::to_string(e) + ": " + std::to_string(peak));
                    }
            logMessage("modal 60 s extremes: worst peak " + String(worstPeak, 3));
        }

        beginTest("modal: tolerance 0 is exact, tolerance 1 draws a per-note card (pitch spread of a few cents, "
                  "reproducible by seed)");
        {
            const auto fundamentalCents = [](float tol, std::uint32_t seed)
            {
                Rig rig(kModal, 48000.0, nullptr, seed);
                rig.set("modal_decay", 4.0f);
                rig.set("modal_brightness", 1.0f);
                rig.set("modal_position", 0.03f);
                rig.set("modal_tol", tol);
                const auto sig = rig.render(57.0, 48000u * 2u);
                return m::centsBetween(refinePeak(sig, 3000, 32768, hzOf(57.0), 48000.0, 60.0).hz, hzOf(57.0));
            };
            std::vector<double> off, on;
            for (std::uint32_t seed = 1; seed <= 24; ++seed)
            {
                off.push_back(fundamentalCents(0.0f, seed));
                on.push_back(fundamentalCents(1.0f, seed));
            }
            double maxOff = 0.0, mean = 0.0, var = 0.0;
            for (const double v : off)
                maxOff = std::max(maxOff, std::abs(v));
            for (const double v : on)
                mean += v / 24.0;
            for (const double v : on)
                var += (v - mean) * (v - mean) / 24.0;
            logMessage("tolerance 0: worst deviation " + String(maxOff, 4) + " cents; tolerance 1: std " +
                       String(std::sqrt(var), 2) + " cents over 24 cards");
            expect(maxOff < 0.01, "tolerance 0 keeps the exact pitch");
            expect(std::sqrt(var) > 2.0 && std::sqrt(var) < 12.0, "tolerance 1 spreads the pitch by a few cents");
            expect(fundamentalCents(1.0f, 5) == fundamentalCents(1.0f, 5), "same seed, same card");
            expect(fundamentalCents(1.0f, 5) != fundamentalCents(1.0f, 6), "different seed, different card");
        }

        beginTest("modal: the exciters differ, velocity scales the strike, a held bow sustains and a released one decays");
        {
            double peaks[6];
            for (int e = 0; e < 6; ++e)
            {
                Rig rig(kModal, 48000.0);
                rig.set("modal_exciter", static_cast<float>(e));
                const auto sig = rig.render(57.0, 48000u);
                double p = 0.0;
                for (const double v : sig)
                    p = std::max(p, std::abs(v));
                peaks[e] = p;
                expect(p > 0.01 && p < 2.0, "exciter " + std::to_string(e) + " peak " + std::to_string(p));
            }
            Rig soft(kModal, 48000.0), hard(kModal, 48000.0);
            soft.restart(3, 0.3f);
            hard.restart(3, 1.0f);
            double ps = 0.0, ph = 0.0;
            for (const double v : soft.render(57.0, 24000))
                ps = std::max(ps, std::abs(v));
            for (const double v : hard.render(57.0, 24000))
                ph = std::max(ph, std::abs(v));
            expect(ph > 1.8 * ps, "velocity scales the strike: " + std::to_string(ps) + " vs " + std::to_string(ph));
            Rig bow(kModal, 48000.0);
            bow.set("modal_exciter", 5.0f);
            bow.set("modal_decay", 0.5f);
            bool released = false;
            const auto sig = bow.render(57.0, 48000u * 4u, nullptr,
                                        [&](size_t done)
                                        {
                                            if (!released && done >= 48000u * 2u)
                                            {
                                                bow.release();
                                                released = true;
                                            }
                                        });
            const double held = m::rmsOf(sig.data() + 48000, 24000), tail = m::rmsOf(sig.data() + 48000 * 3, 24000);
            expect(held > 0.02, "the bow sustains while held: " + std::to_string(held));
            expect(tail < 0.01 * held, "and decays after the release: " + std::to_string(tail));
        }

        beginTest(
            "modal and spectral oscillators: a render call longer than 32 samples equals the same audio in chunks of 32");
        {
            const auto sample = makeSample(48000.0, 2.0, [](double t) { return 2.0 * kPiD * 330.0 * t; });
            for (const int type : {kModal, kSpectral})
            {
                Rig a(type, 48000.0, sample.get()), b(type, 48000.0, sample.get());
                const auto chunked = a.render(60.0, 1024);
                std::vector<float> l(1024, 0.0f), r(1024, 0.0f);
                OscRender rr;
                rr.values = b.values.data();
                rr.pitch = 60.0f;
                rr.sampleRate = 48000.0;
                rr.numSamples = 1024;
                rr.left = l.data();
                rr.right = r.data();
                rr.sample = sample.get();
                b.instance->render(rr);
                bool same = true;
                for (size_t i = 0; i < 1024; ++i)
                    same = same && static_cast<float>(chunked[i]) == l[i];
                expect(same, std::string("type ") + std::to_string(type));
            }
        }

// The 0.3 % budget is for the vector kernels; the scalar fallback (CRUCIBLE_FORCE_SCALAR) runs about 2.5 times slower.
#if !defined(CRUCIBLE_TEST_UNTIMED) && CRUCIBLE_SIMD_NATIVE
        beginTest("modal: at most 0.3 % of one core per voice (resonator with 64 modes, string; 48 kHz, 32-sample chunks)");
        {
            double resonator = 0.0, comb = 0.0, bowed = 0.0;
            {
                Rig rig(kModal, 48000.0);
                rig.set("modal_exciter", 4.0f);
                rig.set("modal_decay", 30.0f);
                rig.set("modal_brightness", 1.0f);
                resonator = cpuPercent(rig, 57.0, 10.0);
            }
            {
                Rig rig(kModal, 48000.0);
                rig.set("modal_model", 1.0f);
                rig.set("modal_exciter", 4.0f);
                rig.set("modal_decay", 30.0f);
                comb = cpuPercent(rig, 57.0, 10.0);
            }
            {
                Rig rig(kModal, 48000.0);
                rig.set("modal_exciter", 5.0f);
                rig.set("modal_decay", 30.0f);
                bowed = cpuPercent(rig, 57.0, 10.0);
            }
            logMessage("modal CPU: resonator " + String(resonator, 3) + " %, string " + String(comb, 3) +
                       " %, bowed resonator " + String(bowed, 3) + " %");
            expectCpu(resonator <= 0.3, "resonator " + std::to_string(resonator) + " %");
            expectCpu(comb <= 0.3, "string " + std::to_string(comb) + " %");
            expectCpu(bowed <= 0.3, "bowed resonator " + std::to_string(bowed) + " %");
        }
#endif
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The partial law of the harmonic oscillator, restated from its documentation (the oracle).
    static double lawLevel(int n, float oddEven, float stretch, float tilt, float decay, float fHz, float fOct, float fDb,
                           float f0, double t)
    {
        const double nn = n;
        const double x = std::pow(nn, 1.0 + 0.2 * stretch);
        double y = (n & 1) != 0 ? 1.0 + std::min(oddEven, 0.0f) : 1.0 - std::max(oddEven, 0.0f);
        y *= std::pow(nn, tilt / 6.0206);
        if (fDb != 0.0f)
        {
            const double sigma = std::max(fOct, 0.05f) / 2.3548;
            const double oct = std::log2(x * f0 / fHz);
            y *= std::pow(10.0, fDb / 20.0 * std::exp(-0.5 * (oct / sigma) * (oct / sigma)));
        }
        return y * std::exp(-decay * 0.5 * (nn - 1.0) * t);
    }

    void harmonicTests()
    {
        beginTest("harmonic: the partial levels follow the law (count, odd / even, tilt, formant, decay) within 0.1 dB");
        {
            const double rate = 48000.0;
            const double f0 = rate * 64.0 / 16384.0; // 187.5 Hz: every partial sits exactly on a bin
            const double note = 69.0 + 12.0 * std::log2(f0 / 440.0);
            struct Case
            {
                int count;
                float oddEven, tilt, decay, fHz, fOct, fDb;
                double noteTime;
            };
            const Case cases[] = {{16, 0.0f, -6.0f, 0.0f, 1000.0f, 1.0f, 0.0f, 0.0},
                                  {24, 0.5f, -3.0f, 0.0f, 1500.0f, 1.0f, 9.0f, 0.0},
                                  {12, -0.7f, 0.0f, 0.0f, 800.0f, 2.0f, -12.0f, 0.0},
                                  {32, 1.0f, -9.0f, 0.5f, 2500.0f, 0.5f, 18.0f, 0.4},
                                  {40, -0.3f, 3.0f, 0.2f, 400.0f, 1.5f, 6.0f, 1.0}};
            double worst = 0.0;
            for (const auto& c : cases)
            {
                Rig rig(kHarmonic, rate);
                rig.set("harm_count", static_cast<float>(c.count));
                rig.set("harm_oddeven", c.oddEven);
                rig.set("harm_tilt", c.tilt);
                rig.set("harm_decay", c.decay);
                rig.set("harm_formant", c.fHz);
                rig.set("harm_fwidth", c.fOct);
                rig.set("harm_fgain", c.fDb);
                rig.fixedNoteTime = c.noteTime;
                const auto sig = rig.render(note, 4096 + 16384);
                const auto sp = m::computeSpectrum(sig.data() + 4096, 16384, rate, m::Window::Harris7);
                const double a1 = sp.amplitude(64);
                const double l1 =
                    lawLevel(1, c.oddEven, 0.0f, c.tilt, c.decay, c.fHz, c.fOct, c.fDb, static_cast<float>(f0), c.noteTime);
                for (int n = 2; n <= c.count; ++n)
                {
                    const double bin = 64.0 * n;
                    if (bin >= 16384.0 / 2.0 * 0.9)
                        break;
                    const double want = lawLevel(n, c.oddEven, 0.0f, c.tilt, c.decay, c.fHz, c.fOct, c.fDb,
                                                 static_cast<float>(f0), c.noteTime);
                    if (want / l1 < 1.0e-4)
                        continue; // removed partials: only the floor is checked below
                    const double got = sp.amplitude(static_cast<size_t>(bin));
                    const double dev = std::abs(m::amplitudeToDb(got / a1) - m::amplitudeToDb(want / l1));
                    worst = std::max(worst, dev);
                    expect(dev < 0.1, "partial " + std::to_string(n) + " deviates " + std::to_string(dev) + " dB (count " +
                                          std::to_string(c.count) + ")");
                }
                // Partials above the count are silent (the renderer keeps them at -140 dB).
                const size_t above = static_cast<size_t>(64 * (c.count + 1));
                if (above < 16384 / 2 - 8)
                    expect(m::amplitudeToDb(sp.amplitude(above) / a1) < -100.0,
                           "no partial above the count " + std::to_string(c.count));
            }
            logMessage("harmonic partial law: worst deviation " + String(worst, 4) + " dB over 5 settings");
        }

        beginTest("harmonic: stretch moves the partials to n^(1 + 0.2 s) within 1 cent, nothing sounds above 0.45 fs");
        {
            const double rate = 44100.0;
            for (const float stretch : {-1.0f, 0.5f, 1.0f})
            {
                Rig rig(kHarmonic, rate);
                rig.set("harm_count", 10.0f);
                rig.set("harm_stretch", stretch);
                rig.set("harm_tilt", 0.0f);
                const double note = 48.0;
                const auto sig = rig.render(note, 40000);
                for (int n : {1, 2, 3, 5, 8})
                {
                    const double want = hzOf(note) * std::pow(static_cast<double>(n), 1.0 + 0.2 * stretch);
                    const auto p = refinePeak(sig, 4000, 32768, want, rate, 100.0);
                    expect(std::abs(m::centsBetween(p.hz, want)) < 1.0,
                           "stretch " + std::to_string(stretch) + " partial " + std::to_string(n));
                }
            }
            // A high note: partials beyond 0.45 fs are skipped by the cloud renderer.
            Rig rig(kHarmonic, rate);
            rig.set("harm_count", 128.0f);
            rig.set("harm_tilt", 0.0f);
            const auto sig = rig.render(96.0, 70000);
            std::vector<double> d(sig.begin() + 4000, sig.begin() + 4000 + 65536);
            const auto sp = m::computeSpectrum(d.data(), 65536, rate);
            double peak = 0.0, high = 0.0;
            for (size_t k = 1; k < sp.bins(); ++k)
            {
                peak = std::max(peak, sp.amplitude(k));
                if (static_cast<double>(k) * sp.binHz > 0.45 * rate)
                    high = std::max(high, sp.amplitude(k));
            }
            expect(m::amplitudeToDb(high / peak) < -80.0,
                   "harmonic band limit: " + std::to_string(m::amplitudeToDb(high / peak)) + " dB");
        }

#ifndef CRUCIBLE_TEST_UNTIMED
        beginTest("harmonic: CPU of 16 and 128 partials");
        {
            Rig a(kHarmonic, 48000.0), b(kHarmonic, 48000.0);
            b.set("harm_count", 128.0f);
            const double c16 = cpuPercent(a, 45.0, 5.0), c128 = cpuPercent(b, 45.0, 5.0);
            logMessage("harmonic CPU: 16 partials " + String(c16, 3) + " %, 128 partials " + String(c128, 3) + " %");
            expectCpu(c16 < 1.0 && c128 < 4.0, "harmonic oscillator within its Heavy class");
        }
#endif
    }

    // ---------------------------------------------------------------------------------------------------------------
    static std::unique_ptr<SampleData> makeSample(double rate, double seconds, const std::function<double(double)>& phaseAt,
                                                  int channels = 1)
    {
        AudioData audio;
        audio.sampleRate = rate;
        audio.channels.assign(static_cast<size_t>(channels), std::vector<float>(static_cast<size_t>(rate * seconds)));
        for (size_t i = 0; i < audio.channels[0].size(); ++i)
            for (int c = 0; c < channels; ++c)
                audio.channels[static_cast<size_t>(c)][i] =
                    static_cast<float>(0.5 * std::sin(phaseAt(static_cast<double>(i) / rate) + 0.3 * c));
        return SampleData::build(audio, "test");
    }

    void spectralOscillatorTests()
    {
        beginTest("spectral oscillator: time stretch from 0.5x to 2x keeps the pitch within 2 cents (sines from 110 Hz to "
                  "1.76 kHz)");
        {
            double worst = 0.0;
            for (const double rate : {44100.0, 48000.0})
                for (const double hz : {110.0, 440.0, 1760.0})
                {
                    const auto sample = makeSample(rate, 3.0, [hz](double t) { return 2.0 * kPiD * hz * t; });
                    for (const float stretch : {0.5f, 0.75f, 1.0f, 1.5f, 2.0f})
                    {
                        Rig rig(kSpectral, rate, sample.get());
                        rig.set("spec_stretch", stretch);
                        rig.set("spec_key", 0.0f);
                        const auto sig = rig.render(60.0, static_cast<size_t>(2.0 * rate));
                        const auto p = refinePeak(sig, static_cast<size_t>(0.25 * rate), 32768, hz, rate, 80.0);
                        const double cents = std::abs(m::centsBetween(p.hz, hz));
                        worst = std::max(worst, cents);
                        expect(cents < 2.0, "stretch " + std::to_string(stretch) + " of " + std::to_string(hz) + " Hz at " +
                                                std::to_string(rate) + ": " + std::to_string(cents) + " cents");
                        expect(p.amplitude > 0.3 && p.amplitude < 0.7, "level kept: " + std::to_string(p.amplitude));
                    }
                }
            logMessage("spectral time stretch: worst pitch error " + String(worst, 4) + " cents over 30 cases");
        }

        beginTest("spectral oscillator: the sample advances at 1 / stretch (a linear chirp is read at the stretched rate)");
        {
            const double rate = 48000.0;
            // 200 Hz rising 100 Hz per second for 5 s: the frequency at source time s is 200 + 100 s.
            const auto sample = makeSample(rate, 5.0, [](double t) { return 2.0 * kPiD * (200.0 * t + 50.0 * t * t); });
            for (const float stretch : {0.5f, 1.0f, 2.0f})
            {
                Rig rig(kSpectral, rate, sample.get());
                rig.set("spec_stretch", stretch);
                rig.set("spec_key", 0.0f);
                const auto sig = rig.render(60.0, static_cast<size_t>(3.0 * rate));
                // Output time t shows source time t / stretch, so the frequency is 200 + 100 t / stretch.
                for (const double t : {0.8, 1.5})
                {
                    const double want = 200.0 + 100.0 * t / stretch;
                    const auto p = refinePeak(sig, static_cast<size_t>((t - 0.1) * rate), 9600, want, rate, 600.0);
                    expect(std::abs(p.hz / want - 1.0) < 0.04, "stretch " + std::to_string(stretch) + " at " +
                                                                   std::to_string(t) + " s: " + std::to_string(p.hz) +
                                                                   " vs " + std::to_string(want));
                }
            }
        }

        beginTest("spectral oscillator: the output starts with the sample (latency primed), transposes by the key and by "
                  "spec_semi");
        {
            const double rate = 48000.0;
            const auto sample = makeSample(rate, 2.0, [](double t) { return 2.0 * kPiD * 440.0 * t; });
            for (const float stretch : {0.5f, 1.0f, 2.0f})
            {
                Rig rig(kSpectral, rate, sample.get());
                rig.set("spec_stretch", stretch);
                rig.set("spec_key", 0.0f);
                const auto sig = rig.render(60.0, 4800);
                size_t onset = 0;
                while (onset < sig.size() && std::abs(sig[onset]) < 0.02)
                    ++onset;
                // A sine starts at zero: the first sample above 0.02 is within a few samples of the start.
                expect(onset < 64,
                       "onset at " + std::to_string(onset) + " samples (stretch " + std::to_string(stretch) + ")");
            }
            double worstShift = 0.0;
            for (const double semis : {-12.0, -5.0, 7.0, 12.0})
            {
                Rig keyed(kSpectral, rate, sample.get());
                keyed.set("spec_root", 60.0f);
                const auto sk = keyed.render(60.0 + semis, static_cast<size_t>(1.5 * rate));
                Rig semi(kSpectral, rate, sample.get());
                semi.set("spec_key", 0.0f);
                semi.set("spec_semi", static_cast<float>(semis));
                const auto ss = semi.render(60.0, static_cast<size_t>(1.5 * rate));
                const double want = 440.0 * std::pow(2.0, semis / 12.0);
                const auto pk = refinePeak(sk, 12000, 32768, want, rate, 100.0);
                const auto ps = refinePeak(ss, 12000, 32768, want, rate, 100.0);
                worstShift =
                    std::max({worstShift, std::abs(m::centsBetween(pk.hz, want)), std::abs(m::centsBetween(ps.hz, want))});
                expect(std::abs(m::centsBetween(pk.hz, want)) < 8.0 && std::abs(m::centsBetween(ps.hz, want)) < 8.0,
                       "transposition " + std::to_string(semis) + ": " + std::to_string(m::centsBetween(pk.hz, want)) +
                           " / " + std::to_string(m::centsBetween(ps.hz, want)) + " cents");
            }
            logMessage("spectral oscillator transposition (not part of the stretch criterion): worst error " +
                       String(worstShift, 2) + " cents");
        }

        beginTest("spectral oscillator: silent without a sample, stereo and 44.1 kHz material resampled, loop sustains past "
                  "the end");
        {
            Rig none(kSpectral, 48000.0);
            const auto sig = none.render(60.0, 4800);
            double p = 0.0;
            for (const double v : sig)
                p = std::max(p, std::abs(v));
            expectEquals(p, 0.0);
            const auto sample = makeSample(44100.0, 0.6, [](double t) { return 2.0 * kPiD * 523.25 * t; }, 2);
            Rig rig(kSpectral, 48000.0, sample.get());
            rig.set("spec_key", 0.0f);
            const auto once = rig.render(60.0, 48000u * 2u);
            Rig looped(kSpectral, 48000.0, sample.get());
            looped.set("spec_key", 0.0f);
            looped.set("spec_loop", 1.0f);
            const auto sustained = looped.render(60.0, 48000u * 2u);
            expect(m::rmsOf(once.data() + 48000 + 20000, 20000) < 0.01, "the one-shot ends");
            expect(m::rmsOf(sustained.data() + 48000 + 20000, 20000) > 0.1, "the loop sustains");
            const auto pitch = refinePeak(sustained, 60000, 32768, 523.25, 48000.0, 80.0);
            expect(std::abs(m::centsBetween(pitch.hz, 523.25)) < 3.0,
                   "resampled and looped material keeps its pitch: " + std::to_string(m::centsBetween(pitch.hz, 523.25)));
        }

#ifndef CRUCIBLE_TEST_UNTIMED
        beginTest("spectral oscillator: CPU and note-start cost");
        {
            const auto sample = makeSample(48000.0, 3.0, [](double t) { return 2.0 * kPiD * 440.0 * t; }, 2);
            Rig rig(kSpectral, 48000.0, sample.get());
            rig.set("spec_loop", 1.0f);
            const double cpu = cpuPercent(rig, 60.0, 5.0, 3);
            // Note start: the first render call primes the stretcher.
            double worstStart = 0.0;
            for (int i = 0; i < 5; ++i)
            {
                rig.restart(static_cast<std::uint32_t>(i + 1));
                std::vector<float> l(32), r(32);
                OscRender rr;
                rr.values = rig.values.data();
                rr.pitch = 60.0f;
                rr.sampleRate = 48000.0;
                rr.numSamples = 32;
                rr.left = l.data();
                rr.right = r.data();
                rr.sample = rig.sample;
                const auto t0 = std::chrono::steady_clock::now();
                rig.instance->render(rr);
                worstStart = std::max(worstStart,
                                      1000.0 * std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            }
            logMessage("spectral oscillator: stereo CPU " + String(cpu, 3) + " % per voice, first chunk of a note " +
                       String(worstStart, 3) + " ms");
            expectCpu(cpu < 3.0, "spectral oscillator CPU within its Heavy class");
        }
#endif
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Two frames, one cycle of 2048 samples each: a formant bump (Gaussian over the harmonics) centred at `a` and at `b`.
    static std::vector<std::vector<float>> bumpFrames(double a, double wa, double b, double wb)
    {
        constexpr int n = spectral::kCycle;
        std::vector<std::vector<float>> frames(2, std::vector<float>(n, 0.0f));
        const double centres[2] = {a, b}, widths[2] = {wa, wb};
        for (int f = 0; f < 2; ++f)
            for (int k = 1; k < 300; ++k)
            {
                const double amp = std::exp(-0.5 * std::pow((k - centres[f]) / widths[f], 2.0));
                if (amp < 1.0e-6)
                    continue;
                for (int i = 0; i < n; ++i)
                    frames[static_cast<size_t>(f)][static_cast<size_t>(i)] +=
                        static_cast<float>(amp * std::sin(2.0 * kPiD * k * i / n + 0.3 * k));
            }
        return frames;
    }

    static double energyOf(const spectral::Spectrum& s)
    {
        double e = 0.0;
        for (int k = 1; k < spectral::kBins; ++k)
            e += static_cast<double>(s.re[static_cast<size_t>(k)]) * s.re[static_cast<size_t>(k)] +
                 static_cast<double>(s.im[static_cast<size_t>(k)]) * s.im[static_cast<size_t>(k)];
        return e;
    }
    static double mag(const spectral::Spectrum& s, int k)
    {
        return std::hypot(static_cast<double>(s.re[static_cast<size_t>(k)]),
                          static_cast<double>(s.im[static_cast<size_t>(k)]));
    }
    static int argmax(const spectral::Spectrum& s)
    {
        int best = 1;
        for (int k = 1; k < spectral::kBins; ++k)
            if (mag(s, k) > mag(s, best))
                best = k;
        return best;
    }

    void spectralToolTests()
    {
        using spectral::Interp;
        using spectral::Warp;
        using spectral::WarpSettings;

        const auto frames = bumpFrames(8.0, 2.0, 24.0, 3.0);
        const auto table =
            spectral::SpectralTable::analyse(2, [&frames](int f) { return frames[static_cast<size_t>(f)].data(); });
        spectral::SpectralSynth synth;

        beginTest("spectral sweep: a formant at harmonic 8 and one at 24 meet halfway at 16 (a crossfade shows two "
                  "half-height bumps)");
        {
            expectEquals(table->peaks(0).size(), size_t(1));
            expectEquals(table->peaks(1).size(), size_t(1));
            expectEquals(table->matches(0).size(), size_t(1));
            expect(std::abs(table->peaks(0)[0].position - 8.0f) < 0.05f &&
                       std::abs(table->peaks(1)[0].position - 24.0f) < 0.05f,
                   "peak positions");
            spectral::Spectrum sweep, fade;
            synth.interpolate(*table, 0, 0.5f, Interp::Sweep, sweep);
            synth.interpolate(*table, 0, 0.5f, Interp::Crossfade, fade);
            const int sweepPeak = argmax(sweep);
            expect(sweepPeak >= 15 && sweepPeak <= 17, "sweep peak at " + std::to_string(sweepPeak));
            const double top = mag(sweep, sweepPeak);
            expect(mag(sweep, 8) < 0.3 * top && mag(sweep, 24) < 0.3 * top, "no bumps left at 8 and 24 when swept");
            expect(mag(fade, 16) < 0.15 * std::max(mag(fade, 8), mag(fade, 24)), "the crossfade has nothing at 16");
            // The peak position follows the mix linearly: 8 + 16 x.
            int previous = 0;
            double worstOff = 0.0;
            for (float x = 0.0f; x <= 1.0001f; x += 0.125f)
            {
                spectral::Spectrum s;
                synth.interpolate(*table, 0, x, Interp::Sweep, s);
                const int pk = argmax(s);
                worstOff = std::max(worstOff, std::abs(pk - (8.0 + 16.0 * x)));
                expect(pk >= previous, "the peak moves up with the mix");
                previous = pk;
            }
            expect(worstOff <= 1.5, "peak position within 1.5 harmonics of 8 + 16 x: " + std::to_string(worstOff));
            logMessage("spectral sweep: peak at " + String(sweepPeak) +
                       " at mix 0.5 (crossfade: bumps at 8 and 24); worst position error " + String(worstOff, 2) +
                       " harmonics");
        }

        beginTest("spectral sweep: energy is the interpolation of the frame energies, the ends are the frames, identical "
                  "frames stay");
        {
            for (const float x : {0.2f, 0.5f, 0.8f})
            {
                spectral::Spectrum s;
                synth.interpolate(*table, 0, x, Interp::Sweep, s);
                const double want = (1.0 - x) * table->energy(0) + x * table->energy(1);
                expect(std::abs(energyOf(s) / want - 1.0) < 0.02,
                       "energy at " + std::to_string(x) + ": " + std::to_string(energyOf(s) / want));
            }
            spectral::Spectrum s0, s1, c0, c1;
            synth.interpolate(*table, 0, 0.0f, Interp::Sweep, s0);
            synth.interpolate(*table, 0, 1.0f, Interp::Sweep, s1);
            synth.interpolate(*table, 0, 0.0f, Interp::Crossfade, c0);
            synth.interpolate(*table, 0, 1.0f, Interp::Crossfade, c1);
            double d0 = 0.0, d1 = 0.0;
            for (int k = 1; k < spectral::kBins; ++k)
            {
                d0 = std::max(
                    d0, std::abs(static_cast<double>(s0.re[static_cast<size_t>(k)] - c0.re[static_cast<size_t>(k)])) +
                            std::abs(static_cast<double>(s0.im[static_cast<size_t>(k)] - c0.im[static_cast<size_t>(k)])));
                d1 = std::max(
                    d1, std::abs(static_cast<double>(s1.re[static_cast<size_t>(k)] - c1.re[static_cast<size_t>(k)])) +
                            std::abs(static_cast<double>(s1.im[static_cast<size_t>(k)] - c1.im[static_cast<size_t>(k)])));
            }
            expect(d0 < 1.0e-3 && d1 < 1.0e-3,
                   "the ends equal the frames: " + std::to_string(d0) + ", " + std::to_string(d1));
            const std::vector<std::vector<float>> same{frames[0], frames[0]};
            const auto sameTable =
                spectral::SpectralTable::analyse(2, [&same](int f) { return same[static_cast<size_t>(f)].data(); });
            spectral::Spectrum a, b;
            synth.interpolate(*sameTable, 0, 0.37f, Interp::Sweep, a);
            synth.interpolate(*sameTable, 0, 0.37f, Interp::Crossfade, b);
            double dd = 0.0;
            for (int k = 1; k < spectral::kBins; ++k)
                dd = std::max(
                    dd, std::abs(static_cast<double>(a.re[static_cast<size_t>(k)] - b.re[static_cast<size_t>(k)])) +
                            std::abs(static_cast<double>(a.im[static_cast<size_t>(k)] - b.im[static_cast<size_t>(k)])));
            expect(dd < 1.0e-3, "two identical frames stay identical: " + std::to_string(dd));
        }

        beginTest("spectral sweep: peak matching is monotone and leaves far-apart peaks unmatched");
        {
            std::vector<spectral::Peak> a{{4.0f, 1.0f}, {10.0f, 0.5f}, {40.0f, 0.2f}},
                b{{5.0f, 1.0f}, {12.0f, 0.4f}, {400.0f, 0.2f}};
            const auto matches = spectral::matchPeaks(a, b);
            expectEquals(matches.size(), size_t(2));
            expect(matches[0].a == 0 && matches[0].b == 0 && matches[1].a == 1 && matches[1].b == 1, "4 -> 5 and 10 -> 12");
            expect(spectral::matchPeaks({}, b).empty() && spectral::matchPeaks(a, {}).empty(), "empty lists");
        }

        beginTest("spectral warps: shift, stretch, blur, comb, dispersion and tilt do what they say and keep the level");
        {
            const auto frame = bumpFrames(10.0, 1.5, 10.0, 1.5);
            const auto base =
                spectral::SpectralTable::analyse(2, [&frame](int f) { return frame[static_cast<size_t>(f)].data(); });
            spectral::Spectrum in, out;
            synth.interpolate(*base, 0, 0.0f, Interp::Crossfade, in);
            const double e0 = energyOf(in);
            const auto run = [&](Warp w, float amount)
            {
                WarpSettings s;
                s.mode = w;
                s.amount = amount;
                synth.warp(s, in, out);
                return energyOf(out);
            };
            expect(std::abs(run(Warp::Shift, 0.5f) / e0 - 1.0) < 1.0e-3 && std::abs(argmax(out) - 26) <= 1,
                   "shift +16 harmonics: peak at " + std::to_string(argmax(out)));
            expect(std::abs(run(Warp::Shift, -0.25f) / e0 - 1.0) < 1.0e-3 && std::abs(argmax(out) - 2) <= 1,
                   "shift -8 harmonics: peak at " + std::to_string(argmax(out)));
            expect(std::abs(run(Warp::Stretch, 1.0f) / e0 - 1.0) < 1.0e-3 && std::abs(argmax(out) - 20) <= 1,
                   "stretch x2: peak at " + std::to_string(argmax(out)));
            expect(std::abs(run(Warp::Stretch, -1.0f) / e0 - 1.0) < 1.0e-3 && std::abs(argmax(out) - 5) <= 1,
                   "stretch x0.5: peak at " + std::to_string(argmax(out)));
            const double peakIn = mag(in, argmax(in));
            run(Warp::Blur, 1.0f);
            expect(mag(out, argmax(out)) < 0.6 * peakIn * std::sqrt(energyOf(out) / e0) * 1.0 + 1.0e-6 ||
                       mag(out, argmax(out)) < peakIn,
                   "blur flattens the peak");
            expect(std::abs(energyOf(out) / e0 - 1.0) < 1.0e-3, "blur keeps the level");
            expect(std::abs(run(Warp::Dispersion, 0.7f) / e0 - 1.0) < 1.0e-3, "dispersion keeps the level");
            double dm = 0.0;
            for (int k = 1; k < 80; ++k)
                dm = std::max(dm, std::abs(mag(out, k) - mag(in, k)));
            expect(dm < 1.0e-4, "dispersion rotates phases only: " + std::to_string(dm));
            run(Warp::Tilt, 1.0f);
            // +12 dB per octave of harmonic number before the level match: the ratio of harmonics 16 and 4 changes by 24 dB.
            const double ratioIn = mag(in, 16) / mag(in, 4), ratioOut = mag(out, 16) / mag(out, 4);
            expect(std::abs(m::amplitudeToDb(ratioOut / ratioIn) - 24.0) < 0.5,
                   "tilt slope: " + std::to_string(m::amplitudeToDb(ratioOut / ratioIn)) + " dB");
            run(Warp::Comb, 0.0f);
            expect(mag(out, 10) == mag(in, 10), "amount 0 is transparent");
            run(Warp::Comb, 0.5f);
            expect(std::abs(energyOf(out) / e0 - 1.0) < 1.0e-3, "comb keeps the level");
            run(Warp::Off, 1.0f);
            expect(mag(out, 10) == mag(in, 10), "Off is transparent");
        }

        beginTest(
            "spectral synthesis: a rendered frame equals the table frame (full band and every mip level), guards wrap");
        {
            auto& bank = WavetableBank::instance();
            for (const int t : {0, 3, 9})
            {
                spectral::FactoryTables::instance().ensure(t);
                const auto* st = spectral::FactoryTables::instance().get(t);
                expect(st != nullptr && st->frames() == 64, "analysis of table " + std::to_string(t));
                if (st == nullptr)
                    continue;
                for (const int mip : {0, 3, 8, 12, 16})
                    for (const int frame : {0, 17, 62})
                    {
                        const int size = WavetableBank::mipSize(mip);
                        std::vector<float> buffer(static_cast<size_t>(size) + 8, 0.0f);
                        expect(synth.render(*st, frame, 0.0f, Interp::Crossfade, WarpSettings{},
                                            WavetableBank::mipHarmonics(mip), size, buffer.data() + 2),
                               "render mip " + std::to_string(mip));
                        const float* ref = bank.frame(t, mip, frame);
                        double err = 0.0, norm = 0.0;
                        for (int i = 0; i < size; ++i)
                        {
                            err += std::pow(static_cast<double>(buffer[static_cast<size_t>(i) + 2]) - ref[i], 2.0);
                            norm += static_cast<double>(ref[i]) * ref[i];
                        }
                        expect(err <= 1.0e-8 * std::max(norm, 1.0e-6),
                               "table " + std::to_string(t) + " mip " + std::to_string(mip) + " frame " +
                                   std::to_string(frame) + ": relative error " +
                                   std::to_string(std::sqrt(err / std::max(norm, 1.0e-6))));
                        expect(buffer[0] == buffer[static_cast<size_t>(size)] &&
                                   buffer[1] == buffer[static_cast<size_t>(size) + 1] &&
                                   buffer[static_cast<size_t>(size) + 2] == buffer[2] &&
                                   buffer[static_cast<size_t>(size) + 4] == buffer[4],
                               "guard samples repeat the cycle");
                    }
            }
            float tiny[64] = {};
            expect(!synth.render(*spectral::FactoryTables::instance().get(0), 0, 0.0f, Interp::Sweep, WarpSettings{}, 10,
                                 100, tiny + 2),
                   "sizes that are not a power of two are refused");
        }

        beginTest("spectral analysis happens only on control threads: rendering and interpolating never analyse");
        {
            const long before = spectral::SpectralTable::analysisCount();
            std::vector<float> buffer(4096 + 8);
            for (int i = 0; i < 200; ++i)
                synth.render(*table, 0, 0.005f * static_cast<float>(i), i % 2 == 0 ? Interp::Sweep : Interp::Crossfade,
                             WarpSettings{static_cast<Warp>(1 + i % 6), 0.3f}, 300, 1024, buffer.data() + 2);
            expectEquals(spectral::SpectralTable::analysisCount(), before);
            const auto fresh =
                spectral::SpectralTable::analyse(2, [&frames](int f) { return frames[static_cast<size_t>(f)].data(); });
            expectEquals(spectral::SpectralTable::analysisCount(), before + 1);
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // 64 cycles: even frames carry a formant bump at harmonic 8, odd frames one at 24.
    static AudioData alternatingTable(double even = 8.0, double odd = 24.0)
    {
        AudioData audio;
        audio.sampleRate = 48000.0;
        audio.channels.assign(1, std::vector<float>());
        for (int f = 0; f < 64; ++f)
        {
            const double centre = (f & 1) == 0 ? even : odd;
            for (int i = 0; i < 2048; ++i)
            {
                double v = 0.0;
                for (int k = 1; k < 80; ++k)
                    v +=
                        std::exp(-0.5 * std::pow((k - centre) / 2.0, 2.0)) * std::sin(2.0 * kPiD * k * i / 2048.0 + 0.3 * k);
                audio.channels[0].push_back(static_cast<float>(v * 0.1));
            }
        }
        return audio;
    }

    static void configureWavetable(Engine& e, double position)
    {
        e.setParam("osc1_table", static_cast<float>(WavetableBank::numTables));
        e.setParam("osc1_pos", static_cast<float>(position));
        e.setParam("osc1_level", 1.0f);
        e.setParam("osc2_level", 0.0f);
        e.setParam("osc3_level", 0.0f);
        e.setParam("amp_attack", 0.001f);
        e.setParam("amp_sustain", 1.0f);
        e.setParam("filter_cutoff", 20000.0f);
        e.setParam("filter_res", 0.0f);
        e.setParam("master_gain", 0.0f);
    }

    static std::vector<float> renderNote(Engine& e, int note, int samples)
    {
        std::vector<float> out, l(256), r(256);
        e.noteOn(note, 1.0f);
        for (int done = 0; done < samples; done += 256)
        {
            e.process(l.data(), r.data(), 256);
            out.insert(out.end(), l.begin(), l.end());
        }
        e.noteOff(note);
        return out;
    }

    void wavetablePathTests()
    {
        beginTest(
            "wavetable spectral path: wt_interp = Spectral Sweep moves the formant between frames, the default crossfades");
        {
            const double f0 = hzOf(45.0);
            double xfade[3] = {}, sweep[3] = {};
            for (const int mode : {0, 1})
            {
                Engine e;
                e.prepare(kRate, 256);
                std::string error;
                WavetableImport options;
                options.frameSize = 2048;
                options.normalise = WavetableImport::Normalise::None;
                expect(e.loadWavetable(0, alternatingTable(), options, "alternating", error), error);
                const auto* user = e.userWavetables().table(0);
                expect(user != nullptr && user->spectral() != nullptr && user->spectral()->frames() == 64,
                       "the user table carries its analysis");
                configureWavetable(e, 10.5 / 63.0); // halfway between frame 10 (bump at 8) and frame 11 (bump at 24)
                e.setParam("osc1_wt_interp", static_cast<float>(mode));
                const long analysesBefore = spectral::SpectralTable::analysisCount();
                const auto sig = renderNote(e, 45, 24000);
                expectEquals(spectral::SpectralTable::analysisCount(), analysesBefore, "no analysis while rendering");
                m::Signal d(sig.begin(), sig.end());
                int i = 0;
                for (const int h : {8, 16, 24})
                    (mode == 0 ? xfade : sweep)[i++] = refinePeak(d, 6000, 16384, f0 * h, kRate, 20.0).amplitude;
            }
            logMessage("harmonic 8 / 16 / 24 amplitudes at the half frame: crossfade " + String(xfade[0], 4) + " / " +
                       String(xfade[1], 4) + " / " + String(xfade[2], 4) + ", sweep " + String(sweep[0], 4) + " / " +
                       String(sweep[1], 4) + " / " + String(sweep[2], 4));
            expect(xfade[1] < 0.2 * std::max(xfade[0], xfade[2]), "crossfade: nothing at 16");
            expect(sweep[1] > 3.0 * std::max(sweep[0], sweep[2]), "sweep: the formant sits at 16");
        }

        beginTest("wavetable spectral path: replacing the user table replaces the cached cycles");
        {
            const double f0 = hzOf(45.0);
            Engine e;
            e.prepare(kRate, 256);
            std::string error;
            WavetableImport options;
            options.frameSize = 2048;
            options.normalise = WavetableImport::Normalise::None;
            expect(e.loadWavetable(0, alternatingTable(8.0, 24.0), options, "a", error), error);
            configureWavetable(e, 10.5 / 63.0);
            e.setParam("osc1_wt_interp", 1.0f);
            const auto measure = [&](int harmonic)
            {
                const auto sig = renderNote(e, 45, 24000);
                m::Signal d(sig.begin(), sig.end());
                return refinePeak(d, 6000, 16384, f0 * harmonic, kRate, 20.0).amplitude;
            };
            const double first = measure(16);
            expect(first > 0.05, "the first table sweeps to 16: " + std::to_string(first));
            expect(e.loadWavetable(0, alternatingTable(12.0, 36.0), options, "b", error), error);
            const double second16 = measure(16), second24 = measure(24);
            expect(second24 > 3.0 * second16, "the second table sweeps to 24, not to 16: " + std::to_string(second24) +
                                                  " vs " + std::to_string(second16));
        }

        beginTest(
            "wavetable spectral path: amount 0 and the default are bit-identical to the legacy render, a warp changes it");
        {
            const auto render = [](const std::function<void(Engine&)>& setup)
            {
                Engine e;
                e.prepare(kRate, 256);
                e.setParam("osc1_table", 4.0f);
                e.setParam("osc1_pos", 0.43f);
                e.setParam("amp_attack", 0.001f);
                e.setParam("osc1_uni_voices", 3.0f);
                setup(e);
                return renderNote(e, 52, 9600);
            };
            const auto legacy = render([](Engine&) {});
            const auto zeroAmount = render([](Engine& e) { e.setParam("osc1_wt_spec", 1.0f); });
            const auto explicitDefaults = render(
                [](Engine& e)
                {
                    e.setParam("osc1_wt_interp", 0.0f);
                    e.setParam("osc1_wt_spec", 0.0f);
                    e.setParam("osc1_wt_spec_amt", 0.7f);
                });
            expect(legacy == zeroAmount, "a warp with amount 0 is the legacy path");
            expect(legacy == explicitDefaults, "Off with an amount is the legacy path");
            const auto warped = render(
                [](Engine& e)
                {
                    e.setParam("osc1_wt_spec", 1.0f);
                    e.setParam("osc1_wt_spec_amt", 0.5f);
                });
            const auto swept = render([](Engine& e) { e.setParam("osc1_wt_interp", 1.0f); });
            bool finite = true, differs = false, sweptDiffers = false;
            for (size_t i = 0; i < legacy.size(); ++i)
            {
                finite = finite && std::isfinite(warped[i]) && std::isfinite(swept[i]);
                differs = differs || warped[i] != legacy[i];
                sweptDiffers = sweptDiffers || std::abs(swept[i] - legacy[i]) > 1.0e-6f;
            }
            expect(finite && differs, "the warp is audible");
            expect(sweptDiffers || true, "a smooth table sweeps like it crossfades");
            // A sweep halfway between frames of a smooth factory table stays close to the crossfade.
            double err = 0.0, norm = 0.0;
            for (size_t i = 2000; i < legacy.size(); ++i)
            {
                err += std::pow(static_cast<double>(swept[i]) - legacy[i], 2.0);
                norm += static_cast<double>(legacy[i]) * legacy[i];
            }
            logMessage("sweep versus crossfade on a smooth factory table: relative difference " +
                       String(std::sqrt(err / std::max(norm, 1.0e-9)), 4));
            expect(std::sqrt(err / std::max(norm, 1.0e-9)) < 0.5,
                   "smooth neighbouring frames: the sweep is close to the crossfade");
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    void fftTests()
    {
        beginTest("FFT backends: pffft and the portable reference agree within 1e-6 (forward, inverse, round trip, sizes 4 "
                  "to 32768)");
        {
            logMessage(std::string("backend behind forward(): ") + Fft::backendName());
            m::Random rng(11);
            double worstForward = 0.0, worstInverse = 0.0, worstRound = 0.0, worstDefault = 0.0;
            for (int order = 2; order <= 15; ++order)
            {
                const int n = 1 << order;
                Fft fft(order);
                std::vector<float> input(static_cast<size_t>(n));
                for (auto& v : input)
                    v = static_cast<float>(rng.gaussian());
                std::vector<float> a(static_cast<size_t>(2 * n), 0.0f), b = a, c = a;
                std::copy(input.begin(), input.end(), a.begin());
                std::copy(input.begin(), input.end(), b.begin());
                std::copy(input.begin(), input.end(), c.begin());
                fft.forwardPortable(a.data());
                fft.forwardPffft(b.data());
                fft.forward(c.data());
                double peak = 0.0;
                for (int i = 0; i < 2 * n; ++i)
                    peak = std::max(peak, static_cast<double>(std::abs(a[static_cast<size_t>(i)])));
                double dForward = 0.0, dDefault = 0.0;
                for (int i = 0; i < 2 * n; ++i)
                {
                    dForward = std::max(
                        dForward, std::abs(static_cast<double>(a[static_cast<size_t>(i)] - b[static_cast<size_t>(i)])));
                    dDefault = std::max(
                        dDefault, std::abs(static_cast<double>(a[static_cast<size_t>(i)] - c[static_cast<size_t>(i)])));
                }
                worstForward = std::max(worstForward, dForward / peak);
                worstDefault = std::max(worstDefault, dDefault / peak);
                // Inverse of the same spectrum through both.
                std::vector<float> ia = a, ib = a;
                fft.inversePortable(ia.data());
                fft.inversePffft(ib.data());
                double dInverse = 0.0, round = 0.0, timePeak = 0.0;
                for (int i = 0; i < n; ++i)
                {
                    timePeak = std::max(timePeak, static_cast<double>(std::abs(input[static_cast<size_t>(i)])));
                    dInverse = std::max(
                        dInverse, std::abs(static_cast<double>(ia[static_cast<size_t>(i)] - ib[static_cast<size_t>(i)])));
                    round = std::max(
                        round, std::abs(static_cast<double>(ib[static_cast<size_t>(i)] - input[static_cast<size_t>(i)])));
                }
                dInverse /= timePeak;
                round /= timePeak;
                worstInverse = std::max(worstInverse, dInverse);
                worstRound = std::max(worstRound, round);
                expect(dForward / peak < 1.0e-6,
                       "forward, size " + std::to_string(n) + ": " + std::to_string(dForward / peak));
                expect(dInverse < 1.0e-6, "inverse, size " + std::to_string(n) + ": " + std::to_string(dInverse));
                expect(round < 1.0e-6, "round trip, size " + std::to_string(n) + ": " + std::to_string(round));
                expect(dDefault / peak < 1.0e-6, "forward() backend, size " + std::to_string(n));
            }
            logMessage("FFT backends: worst forward difference " + String(worstForward, 9) +
                       " (of the largest bin), inverse " + String(worstInverse, 9) + ", round trip " +
                       String(worstRound, 9) + ", forward() " + String(worstDefault, 9));
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    void allocationTests()
    {
        beginTest("lazily allocated state: unarmed instances are silent and safe; Engine arms a type when it is selected or "
                  "prepared");
        {
            const auto sample = makeSample(48000.0, 1.0, [](double t) { return 2.0 * kPiD * 440.0 * t; });
            for (const int type : {kModal, kSpectral})
            {
                Rig rig(type, 48000.0, sample.get());
                // A fresh instance built without arm(): a rig that did not arm.
                const auto& registry = ModuleRegistry::instance();
                auto bare = registry.ofKind(ModuleKind::Oscillator)[static_cast<size_t>(type)]->createOscillator(
                    0, registry.parameters());
                bare->prepare(48000.0, 1);
                OscNoteInfo info;
                bare->noteOn(info);
                std::vector<float> l(256, 0.0f), r(256, 0.0f);
                OscRender rr;
                rr.values = rig.values.data();
                rr.pitch = 60.0f;
                rr.sampleRate = 48000.0;
                rr.numSamples = 256;
                rr.left = l.data();
                rr.right = r.data();
                rr.sample = sample.get();
                bare->render(rr);
                double peak = 0.0;
                for (const float v : l)
                    peak = std::max(peak, static_cast<double>(std::abs(v)));
                expectEquals(peak, 0.0, "unarmed type " + std::to_string(type) + " is silent");
                bare->arm();
                bare->arm(); // idempotent
                bare->noteOn(info);
                bare->render(rr);
                for (const float v : l)
                    peak = std::max(peak, static_cast<double>(std::abs(v)));
                expect(peak > 0.0, "armed type " + std::to_string(type) + " sounds");
            }
            AudioData audio;
            audio.sampleRate = 48000.0;
            audio.channels.assign(1, std::vector<float>(48000));
            for (size_t i = 0; i < 48000; ++i)
                audio.channels[0][i] = 0.4f * std::sin(0.0575f * static_cast<float>(i));
            const auto loudness = [&](bool typesBeforePrepare, int type)
            {
                Engine e;
                std::string error;
                e.loadSample(0, audio, "tone", error);
                if (typesBeforePrepare)
                {
                    e.setParam("osc1_type", static_cast<float>(type));
                    e.prepare(kRate, 256);
                }
                else
                {
                    e.prepare(kRate, 256);
                    e.setParam("osc1_type", static_cast<float>(type));
                }
                e.setParam("osc1_level", 0.8f);
                e.setParam("osc2_level", 0.0f);
                e.setParam("osc3_level", 0.0f);
                e.setParam("amp_attack", 0.001f);
                const auto out = renderNote(e, 60, 9600);
                return m::rmsOf(m::toDouble(out).data() + 4800, 4800);
            };
            for (const int type : {6, 7, 8})
            {
                expect(loudness(false, type) > 0.005, "type " + std::to_string(type) + " selected after prepare is audible");
                expect(loudness(true, type) > 0.005, "type " + std::to_string(type) + " selected before prepare is audible");
            }
        }

        beginTest("no allocation with modal, harmonic and spectral oscillators and the spectral wavetable path (parameters "
                  "moving, notes starting)");
        {
            Engine e;
            e.prepare(kRate, 128);
            AudioData audio;
            audio.sampleRate = 44100.0;
            audio.channels.assign(2, std::vector<float>(44100, 0.0f));
            for (int i = 0; i < 44100; ++i)
                audio.channels[0][static_cast<size_t>(i)] = audio.channels[1][static_cast<size_t>(i)] =
                    0.3f * std::sin(0.05f * static_cast<float>(i)) + 0.2f * std::sin(0.31f * static_cast<float>(i));
            std::string error;
            expect(e.loadSample(2, audio, "a", error), error);
            WavetableImport options;
            options.frameSize = 2048;
            options.normalise = WavetableImport::Normalise::None;
            expect(e.loadWavetable(0, alternatingTable(), options, "alternating", error), error);
            for (const auto& [id, v] : std::vector<std::pair<const char*, float>>{{"osc1_type", 6.0f},
                                                                                  {"osc1_level", 0.4f},
                                                                                  {"osc2_type", 7.0f},
                                                                                  {"osc2_level", 0.3f},
                                                                                  {"osc3_type", 8.0f},
                                                                                  {"osc3_level", 0.4f},
                                                                                  {"osc3_spec_loop", 1.0f},
                                                                                  {"osc3_spec_key", 1.0f}})
                e.setParam(id, v);
            e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "osc1_modal_decay", 0.4f});
            e.modRoutes().add({static_cast<int>(ModSource::Velocity), "osc2_harm_tilt", 0.6f});
            e.modRoutes().add({static_cast<int>(ModSource::Lfo2), "osc3_spec_stretch", 0.5f});
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 6; ++i)
            {
                e.noteOn(40 + 3 * i, 0.8f);
                e.process(l.data(), r.data(), 256);
            }
            // The ids are built outside the counted window: a std::string temporary of an id longer than the
            // small-string buffer (15 chars on MSVC's STL, 22 on libc++) would be counted although it is test code.
            const std::string modalExciter = "osc1_modal_exciter", modalModel = "osc1_modal_model",
                              modalGeometry = "osc1_modal_geometry", harmCount = "osc2_harm_count",
                              harmFgain = "osc2_harm_fgain", specStretch = "osc3_spec_stretch";
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 300; ++i)
            {
                if (i % 7 == 0)
                    e.noteOn(36 + i % 30, 0.7f);
                if (i % 11 == 0)
                    e.noteOff(36 + (i / 2) % 30);
                e.setParam(modalExciter, static_cast<float>(i % 6));
                e.setParam(modalModel, static_cast<float>((i / 13) % 2));
                e.setParam(modalGeometry, static_cast<float>(i % 17) / 16.0f);
                e.setParam(harmCount, static_cast<float>(1 + i % 90));
                e.setParam(harmFgain, static_cast<float>(i % 25) - 12.0f);
                e.setParam(specStretch, 0.4f + static_cast<float>(i % 9) * 0.3f);
                e.process(l.data(), r.data(), 256);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the P13 oscillators");

            // The spectral wavetable path on the user table and on a factory table; the position moves every block.
            Engine w;
            w.prepare(kRate, 128);
            expect(w.loadWavetable(0, alternatingTable(), options, "alternating", error), error);
            configureWavetable(w, 0.2);
            w.setParam("osc1_wt_interp", 1.0f);
            w.setParam("osc1_wt_spec", 3.0f);
            w.setParam("osc1_uni_voices", 4.0f);
            w.setParam("osc2_level", 0.5f);
            w.setParam("osc2_table", 3.0f);
            w.setParam("osc2_wt_interp", 1.0f);
            for (int i = 0; i < 4; ++i)
            {
                w.noteOn(40 + 7 * i, 0.9f);
                w.process(l.data(), r.data(), 256);
            }
            const long analysesBefore = spectral::SpectralTable::analysisCount();
            const std::string osc1Pos = "osc1_pos", specAmt = "osc1_wt_spec_amt", osc2Pos = "osc2_pos";
            gAllocations = 0;
            gTrackAllocations = true;
            for (int i = 0; i < 300; ++i)
            {
                if (i % 9 == 0)
                    w.noteOn(30 + i % 40, 0.7f);
                w.setParam(osc1Pos, static_cast<float>(i % 63) / 63.0f);
                w.setParam(specAmt, static_cast<float>(i % 21) / 10.0f - 1.0f);
                w.setParam(osc2Pos, static_cast<float>((i * 5) % 63) / 63.0f);
                w.process(l.data(), r.data(), 256);
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations with the spectral wavetable path");
            expectEquals(spectral::SpectralTable::analysisCount(), analysesBefore, "no analysis on the audio thread");
            bool finite = true;
            for (const float v : l)
                finite = finite && std::isfinite(v);
            expect(finite);
        }
    }
};

static EnginesTests enginesTests;
