// SPDX-License-Identifier: MIT
// P12: the audio bus and audio-rate modulation. Band-limited audio-rate LFOs, per-sample filter cutoff (filter FM) against
// an offline double-precision model, audio-rate pulse width from another oscillator slot, pitch FM, amplitude modulation,
// the chunk-mean fallback, the eight-buffer limit, and real-time behaviour.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "GraphSupport.h"

#include <crucible/AudioLfo.h>
#include <crucible/Modulation.h>
#include <crucible/Modulators.h>
#include <crucible/filters/FilterModel.h>
#include <crucible/graph/Compiler.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>
#if defined(__APPLE__)
#include <malloc/malloc.h>
#include <thread>
#endif

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr double kPiD = 3.14159265358979323846;

std::string num(double v, int decimals = 2)
{
    return String(v, decimals).str();
}

double hzOf(int note)
{
    return 440.0 * std::pow(2.0, (note - 69) / 12.0);
}

// Magnitude of the component at `hz` of x[from .. from + n) with a Blackman-Harris window, as an amplitude.
double toneAmplitude(const std::vector<double>& x, size_t from, size_t n, double hz, double rate = kRate)
{
    const m::Signal window = m::makeWindow(m::Window::BlackmanHarris, n);
    double sumW = 0.0;
    for (const double w : window)
        sumW += w;
    const auto c = m::windowedDtft(x.data() + from, window.data(), n, hz, rate);
    return 2.0 * std::abs(c) / sumW;
}

// The ladder of the transistor model at resonance 0, drive 0: four trapezoidal one-poles with the same integrator gain,
// run in double precision at twice the rate, with a cutoff that follows fc(t) sample by sample. Returns the output at the
// host rate (every second sample).
std::vector<double> ladderOracle(const std::vector<double>& inputAtHost, double fc0, const std::vector<double>& octaves,
                                 double rate)
{
    const double fsOs = 2.0 * rate;
    double s[4] = {0, 0, 0, 0};
    std::vector<double> out(inputAtHost.size());
    const size_t n = inputAtHost.size();
    for (size_t i = 0; i < 2 * n; ++i)
    {
        // Linear interpolation of the host-rate samples for the oversampled instants; the tones are far below fs / 4.
        const size_t j = i / 2;
        const double frac = (i % 2) * 0.5;
        const double x0 = inputAtHost[j], x1 = j + 1 < n ? inputAtHost[j + 1] : inputAtHost[j];
        double x = x0 + (x1 - x0) * frac;
        const double o0 = octaves[j], o1 = j + 1 < n ? octaves[j + 1] : octaves[j];
        const double oct = o0 + (o1 - o0) * frac;
        const double fc = std::min(fc0 * std::exp2(oct), 0.45 * rate);
        const double g = std::tan(kPiD * fc / fsOs);
        const double G = g / (1.0 + g);
        for (auto& st : s)
        {
            const double v = G * (x - st);
            const double y = v + st;
            st = y + v;
            x = y;
        }
        if (i % 2 == 0)
            out[j] = x;
    }
    return out;
}

// ---- Helpers for the engine tests.

enum Wave
{
    Saw = 0,
    Pulse,
    Tri,
    Sine
};

std::vector<float> analogPatch()
{
    auto v = ParameterMap::defaultValues();
    setValue(v, "amp_attack", 0.001f);
    setValue(v, "amp_sustain", 1.0f);
    setValue(v, "master_gain", 0.0f);
    for (const char* id : {"osc1_level", "osc2_level", "osc3_level", "sub_level", "noise_level"})
        setValue(v, id, 0.0f);
    setValue(v, "filter_on", 0.0f);
    setValue(v, "filter2_on", 0.0f);
    for (int o = 1; o <= 3; ++o)
    {
        setValue(v, "osc" + std::to_string(o) + "_type", 5.0f);
        setValue(v, "osc" + std::to_string(o) + "_an_drift", 0.0f);
        setValue(v, "osc" + std::to_string(o) + "_an_tol", 0.0f);
        setValue(v, "osc" + std::to_string(o) + "_pan", 0.0f);
    }
    return v;
}

constexpr int kOsc1Audio = kSourceAudio + kAudioOscFirst;
constexpr int kLfo1Audio = kSourceAudio + kAudioLfoFirst;

// Renders `samples` of one note and returns the left channel as doubles.
std::vector<double> renderNote(const std::vector<float>& values, int note, int samples,
                               const std::vector<ModRoute>& routes = {}, EngineConfig cfg = {},
                               std::uint64_t* bankBlocks = nullptr)
{
    SynthEngine engine(cfg);
    prepare(engine, values);
    for (const auto& r : routes)
        if (!engine.modRoutes().add(r))
            return {};
    engine.noteOn(note, 1.0f);
    const auto r = renderBlocks(engine, samples, 512, true);
    if (bankBlocks != nullptr)
        *bankBlocks = engine.batchedFilterBlocks();
    return m::toDouble(r.left);
}

// Windowed amplitude of a signal given at `rate` for a line at hz.
double lineAmplitude(const std::vector<double>& x, size_t from, size_t n, double hz, double rate)
{
    return toneAmplitude(x, from, n, hz, rate);
}

// Amplitude of the line at hz of an ideal pulse train of rate f0 whose width is base + dev sin(2 pi fm t): the exact
// Fourier integral over [0, T) with a Blackman-Harris window, no sampling involved.
double pwmLineAmplitude(double f0, double base, double dev, double fm, double hz, double seconds)
{
    constexpr double a0 = 0.35875, a1 = 0.48829, a2 = 0.14128, a3 = 0.01168;
    const auto win = [&](double t)
    {
        const double x = 2.0 * kPiD * t / seconds;
        return a0 - a1 * std::cos(x) + a2 * std::cos(2.0 * x) - a3 * std::cos(3.0 * x);
    };
    // 6-point Gauss-Legendre.
    static const double gx[6] = {-0.9324695142031521, -0.6612093864662645, -0.2386191860831969,
                                 0.2386191860831969,  0.6612093864662645,  0.9324695142031521};
    static const double gw[6] = {0.1713244923791704, 0.3607615730481386, 0.4679139345726910,
                                 0.4679139345726910, 0.3607615730481386, 0.1713244923791704};
    const double w = 2.0 * kPiD * hz;
    std::complex<double> sum = 0.0;
    const auto integrate = [&](double lo, double hi, double sign)
    {
        // Split long intervals so that e^{-jwt} stays well sampled.
        const int pieces = std::max(1, static_cast<int>(std::ceil((hi - lo) * hz * 2.0)));
        for (int p = 0; p < pieces; ++p)
        {
            const double l = lo + (hi - lo) * p / pieces, h = lo + (hi - lo) * (p + 1) / pieces;
            for (int q = 0; q < 6; ++q)
            {
                const double t = 0.5 * (h - l) * gx[q] + 0.5 * (h + l);
                sum += sign * gw[q] * 0.5 * (h - l) * win(t) * std::polar(1.0, -w * t);
            }
        }
    };
    const int cycles = static_cast<int>(seconds * f0);
    double rise = 0.0;
    for (int n = 0; n <= cycles; ++n)
    {
        // Falling edge: f0 t - n = width(t).
        double t = (n + base) / f0;
        for (int it = 0; it < 30; ++it)
        {
            const double width = base + dev * std::sin(2.0 * kPiD * fm * t);
            const double g = f0 * t - n - width;
            const double dg = f0 - dev * 2.0 * kPiD * fm * std::cos(2.0 * kPiD * fm * t);
            t -= g / dg;
        }
        const double fall = t;
        const double nextRise = (n + 1) / f0;
        const double lo = std::max(rise, 0.0), hi = std::min(fall, seconds);
        if (hi > lo)
            integrate(lo, hi, 1.0);
        const double lo2 = std::max(fall, 0.0), hi2 = std::min(nextRise, seconds);
        if (hi2 > lo2)
            integrate(lo2, hi2, -1.0);
        rise = nextRise;
        if (rise >= seconds)
            break;
    }
    return 2.0 * std::abs(sum) / (a0 * seconds);
}
} // namespace

class AudioRateModTests : public UnitTest
{
public:
    AudioRateModTests() : UnitTest("Audio-rate modulation", "Modulation") {}

    void runTest() override
    {
        audioLfo();
        filterFm();
        filterFmEngine();
        pwmFromAnotherSlot();
        pitchFm();
        fmAmount();
        amplitudeModulation();
        wavetableTargets();
        limitsAndFallback();
        graphEdges();
        realTime();
        freshThreadScratch();
        pitchRouteCrossMod();
    }

private:
    void audioLfo()
    {
        beginTest("audio-rate LFO: saw, square, triangle, saw down and pulse alias below -90 dB up to 12 kHz; the "
                  "fundamental is exact");
        constexpr size_t n = 1 << 15;
        double worst = -400.0;
        std::string where;
        for (const int shape : {2, 3, 1, 5, 7})
            for (const double hz : {440.0, 1760.0, 3333.3, 5333.7, 8200.0, 12345.6})
            {
                AudioLfo lfo;
                lfo.reset(0.0);
                std::vector<float> out(n + 4096);
                lfo.render(shape, hz / kRate, 1.0f, static_cast<int>(out.size()), out.data());
                const auto sig = m::toDouble(out, 4096, n);
                const auto a = m::measureAliasing(sig, 0, kRate, hz);
                if (a.aliasDb > worst)
                {
                    worst = a.aliasDb;
                    where = "shape " + std::to_string(shape) + " at " + num(hz, 1) + " Hz";
                }
            }
        logMessage("worst aliasing " + String(worst, 1) + " dB (" + where + ")");
        expect(worst < -90.0, "aliasing " + num(worst, 1) + " dB, " + where);
        // Fundamental amplitudes: saw 2 / pi, square 4 / pi, triangle 8 / pi^2.
        const struct
        {
            int shape;
            double amplitude;
        } expected[] = {{2, 2.0 / kPiD}, {3, 4.0 / kPiD}, {1, 8.0 / (kPiD * kPiD)}, {0, 1.0}};
        for (const auto& e : expected)
        {
            AudioLfo lfo;
            lfo.reset(0.0);
            std::vector<float> out(n + 4096);
            lfo.render(e.shape, 440.0 / kRate, 1.0f, static_cast<int>(out.size()), out.data());
            const auto sig = m::toDouble(out, 4096, n);
            const double got = toneAmplitude(sig, 0, n, 440.0);
            expectWithinAbsoluteError(20.0 * std::log10(got / e.amplitude), 0.0, 0.05,
                                      "fundamental of shape " + String(e.shape));
        }
    }

    void filterFm()
    {
        beginTest("filter FM: ladder sidebands within 0.5 dB of an offline double-precision model (depth up to 1 octave; "
                  "0.75 dB at 2 octaves)");
        constexpr size_t n = 1 << 14;
        constexpr size_t settle = 8192;
        const double carrier = 1500.0, modHz = 375.0, fc0 = 2000.0;
        double worstSmall = 0.0, worstDeep = 0.0;
        for (const double depth : {0.5, 1.0, 2.0})
        {
            std::vector<double> input(n + settle), oct(n + settle);
            for (size_t i = 0; i < input.size(); ++i)
            {
                input[i] = 0.01 * std::sin(2.0 * kPiD * carrier * static_cast<double>(i) / kRate);
                oct[i] = depth * std::sin(2.0 * kPiD * modHz * static_cast<double>(i) / kRate);
            }
            auto filter = makeTransistorLadder();
            filter->prepare(kRate, Quality::Standard, 1, 512);
            filter->setControls(static_cast<float>(fc0), 0.0f, 0.0f, 0.0f, 0.0f);
            std::vector<float> io(input.size()), mod(input.size());
            for (size_t i = 0; i < input.size(); ++i)
            {
                io[i] = static_cast<float>(input[i]);
                mod[i] = static_cast<float>(oct[i]);
            }
            for (size_t pos = 0; pos < io.size(); pos += 32)
                filter->process(io.data() + pos, 32, mod.data() + pos);
            const auto engine = m::toDouble(io);
            const auto oracle = ladderOracle(input, fc0, oct, kRate);
            const double carrierE = toneAmplitude(engine, settle, n, carrier),
                         carrierO = toneAmplitude(oracle, settle, n, carrier);
            for (int k = -4; k <= 4; ++k)
            {
                const double hz = carrier + k * modHz;
                if (hz < 100.0)
                    continue;
                const double e = 20.0 * std::log10(toneAmplitude(engine, settle, n, hz) / carrierE);
                const double o = 20.0 * std::log10(toneAmplitude(oracle, settle, n, hz) / carrierO);
                if (o < -60.0)
                    continue;
                (depth <= 1.0 ? worstSmall : worstDeep) = std::max(depth <= 1.0 ? worstSmall : worstDeep, std::abs(e - o));
            }
        }
        logMessage("largest sideband error: " + String(worstSmall, 2) + " dB up to 1 octave, " + String(worstDeep, 2) +
                   " dB at 2 octaves");
        expect(worstSmall <= 0.5, "up to one octave of depth: " + num(worstSmall));
        expect(worstDeep <= 0.75, "two octaves: " + num(worstDeep));

        beginTest(
            "filter FM: every analog model stays finite and bounded under deep fast modulation, and four lanes match one");
        {
            using Factory = std::unique_ptr<AnalogFilter> (*)();
            static const Factory factories[8] = {makeTransistorLadder, makeOtaCascade,        makeDiodeLadder,
                                                 makeSallenKeyLowpass, makeSallenKeyHighpass, makeSvfOta,
                                                 makeOta4Pole,         makePassiveHp};
            for (size_t f = 0; f < 8; ++f)
            {
                auto filter = factories[f]();
                filter->prepare(kRate, Quality::Standard, 1, 512);
                filter->setControls(1500.0f, 0.9f, 0.3f, 0.5f, 0.0f);
                bool finite = true;
                double peak = 0.0;
                std::vector<float> io(32), mod(32);
                for (int block = 0; block < 3000; ++block)
                {
                    for (int i = 0; i < 32; ++i)
                    {
                        const double t = static_cast<double>(block * 32 + i) / kRate;
                        io[static_cast<size_t>(i)] = static_cast<float>(0.3 * std::sin(2.0 * kPiD * 220.0 * t) +
                                                                        0.2 * std::sin(2.0 * kPiD * 3100.0 * t));
                        mod[static_cast<size_t>(i)] = static_cast<float>(3.0 * std::sin(2.0 * kPiD * 2300.0 * t));
                    }
                    filter->process(io.data(), 32, mod.data());
                    for (const float v : io)
                    {
                        finite = finite && std::isfinite(v);
                        peak = std::max(peak, static_cast<double>(std::abs(v)));
                    }
                }
                expect(finite && peak < 8.0, "model " + std::to_string(f) + ": peak " + num(peak));
            }
            // Four lanes: lane pair 0/1 modulated, pair 2/3 not.
            auto quad = makeTransistorLadder();
            quad->prepare(kRate, Quality::Standard, 4, 32);
            auto single = makeTransistorLadder();
            single->prepare(kRate, Quality::Standard, 1, 32);
            auto still = makeTransistorLadder();
            still->prepare(kRate, Quality::Standard, 1, 32);
            for (int l = 0; l < 4; ++l)
                quad->setLaneControls(l, 1800.0f, 0.3f, 0.1f, 0.5f, 0.0f);
            single->setControls(1800.0f, 0.3f, 0.1f, 0.5f, 0.0f);
            still->setControls(1800.0f, 0.3f, 0.1f, 0.5f, 0.0f);
            double worstMod = 0.0, worstStill = 0.0;
            for (int block = 0; block < 400; ++block)
            {
                std::array<std::array<float, 32>, 4> lanes{};
                std::array<float, 32> mod{}, ref{}, ref2{};
                for (int i = 0; i < 32; ++i)
                {
                    const double t = static_cast<double>(block * 32 + i) / kRate;
                    const float x = static_cast<float>(0.2 * std::sin(2.0 * kPiD * 440.0 * t));
                    for (auto& lane : lanes)
                        lane[static_cast<size_t>(i)] = x;
                    ref[static_cast<size_t>(i)] = ref2[static_cast<size_t>(i)] = x;
                    mod[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPiD * 700.0 * t));
                }
                float* io[4] = {lanes[0].data(), lanes[1].data(), lanes[2].data(), lanes[3].data()};
                const float* mods[4] = {mod.data(), mod.data(), nullptr, nullptr};
                quad->processQuad(io, 32, mods);
                single->process(ref.data(), 32, mod.data());
                still->process(ref2.data(), 32, nullptr);
                for (int i = 0; i < 32; ++i)
                {
                    worstMod = std::max(worstMod, static_cast<double>(std::abs(lanes[0][static_cast<size_t>(i)] -
                                                                               ref[static_cast<size_t>(i)])));
                    worstStill = std::max(worstStill, static_cast<double>(std::abs(lanes[2][static_cast<size_t>(i)] -
                                                                                   ref2[static_cast<size_t>(i)])));
                }
            }
            logMessage("four lanes against one: modulated lane " + String(worstMod, 8) + ", unmodulated lane " +
                       String(worstStill, 8));
            expect(worstMod < 1.0e-4, "modulated lanes " + num(worstMod, 7));
            expect(worstStill < 1.0e-4, "unmodulated lanes " + num(worstStill, 7));
        }

        beginTest("a quad lane without cutoff modulation keeps its own control rate next to a modulated lane");
        {
            auto quad = makeTransistorLadder();
            quad->prepare(kRate, Quality::Standard, 4, 32);
            auto single = makeTransistorLadder();
            single->prepare(kRate, Quality::Standard, 1, 32);
            double worst = 0.0;
            for (int block = 0; block < 300; ++block)
            {
                // Fast cutoff moves on the unmodulated lanes, which the single path ramps over 16 samples.
                const float cutoff = (block % 2 == 0) ? 600.0f : 5000.0f;
                for (int l = 0; l < 4; ++l)
                    quad->setLaneControls(l, cutoff, 0.4f, 0.1f, 0.5f, 0.0f);
                single->setControls(cutoff, 0.4f, 0.1f, 0.5f, 0.0f);
                std::array<std::array<float, 32>, 4> lanes{};
                std::array<float, 32> mod{}, ref{};
                for (int i = 0; i < 32; ++i)
                {
                    const double t = static_cast<double>(block * 32 + i) / kRate;
                    const float x = static_cast<float>(0.2 * std::sin(2.0 * kPiD * 440.0 * t));
                    for (auto& lane : lanes)
                        lane[static_cast<size_t>(i)] = x;
                    ref[static_cast<size_t>(i)] = x;
                    mod[static_cast<size_t>(i)] = static_cast<float>(0.8 * std::sin(2.0 * kPiD * 700.0 * t));
                }
                float* io[4] = {lanes[0].data(), lanes[1].data(), lanes[2].data(), lanes[3].data()};
                const float* mods[4] = {mod.data(), nullptr, nullptr, nullptr};
                quad->processQuad(io, 32, mods);
                single->process(ref.data(), 32, nullptr);
                for (size_t i = 0; i < 32; ++i)
                    for (size_t l = 1; l < 4; ++l)
                        worst = std::max(worst, static_cast<double>(std::abs(lanes[l][i] - ref[i])));
            }
            logMessage("unmodulated lanes beside a modulated one: largest difference " + String(worst, 8));
            expect(worst < 1.0e-5, "unmodulated lanes follow the single path " + num(worst, 7));
        }
    }

    void filterFmEngine()
    {
        beginTest(
            "filter FM in the voice: an audio-rate LFO routed to the cutoff gives the model's sidebands, batched or not");
        const int note = 79;                                                          // 1568 Hz
        const double carrier = hzOf(note), modHz = 375.0, fc0 = 2000.0, amount = 0.2; // 5 x 0.2 = 1 octave
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_level", 0.02f);
        setValue(v, "filter_on", 1.0f);
        setValue(v, "filter_type", 17.0f);
        setValue(v, "filter_cutoff", static_cast<float>(fc0));
        setValue(v, "filter_res", 0.0f);
        setValue(v, "filter_env", 0.0f);
        setValue(v, "filter_key", 0.0f);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo1_audio_hz", static_cast<float>(modHz));
        setValue(v, "lfo1_shape", 0.0f);
        const std::vector<ModRoute> routes = {{kLfo1Audio, "filter_cutoff", static_cast<float>(amount), 0, 0.0f, 0.0f}};
        constexpr size_t n = 1 << 14;
        constexpr size_t settle = 8192;
        std::vector<double> input(n + settle), oct(n + settle);
        for (size_t i = 0; i < input.size(); ++i)
        {
            input[i] = 0.01 * std::sin(2.0 * kPiD * carrier * static_cast<double>(i) / kRate);
            oct[i] = 5.0 * amount * std::sin(2.0 * kPiD * modHz * static_cast<double>(i) / kRate);
        }
        const auto oracle = ladderOracle(input, fc0, oct, kRate);
        const double carrierO = toneAmplitude(oracle, settle, n, carrier);
        std::vector<std::vector<double>> renders;
        for (const bool batched : {true, false})
        {
            EngineConfig cfg;
            cfg.batchAnalogFilters = batched;
            std::uint64_t bankBlocks = 0;
            const auto out = renderNote(v, note, static_cast<int>(n + settle) + 4096, routes, cfg, &bankBlocks);
            // The bank filters the voice when batching is on and never when it is off (the outputs may be bit-identical).
            expect(batched ? bankBlocks > 0 : bankBlocks == 0,
                   std::string(batched ? "batched" : "single") + ": bank blocks " + num(static_cast<double>(bankBlocks)));
            expect(!out.empty());
            renders.push_back(out);
            const double carrierE = toneAmplitude(out, settle + 2048, n, carrier);
            double worst = 0.0;
            for (int k = -3; k <= 4; ++k)
            {
                const double hz = carrier + k * modHz;
                const double e = 20.0 * std::log10(toneAmplitude(out, settle + 2048, n, hz) / carrierE);
                const double o = 20.0 * std::log10(toneAmplitude(oracle, settle, n, hz) / carrierO);
                if (o > -60.0)
                    worst = std::max(worst, std::abs(e - o));
            }
            logMessage(std::string(batched ? "batched" : "single") + ": largest sideband error " + String(worst, 2) + " dB");
            expect(worst <= 0.5, std::string(batched ? "batched" : "single") + " " + num(worst));
        }
        // Batched and single-voice paths agree to within rounding, and are different code (bit for bit).
        double peak = 0.0, diff = 0.0;
        for (size_t i = 0; i < renders[0].size() && i < renders[1].size(); ++i)
        {
            peak = std::max(peak, std::abs(renders[0][i]));
            diff = std::max(diff, std::abs(renders[0][i] - renders[1][i]));
        }
        logMessage("batched against voice by voice: largest difference " + String(diff, 9) + " at peak " + String(peak, 5));
        expect(diff <= 1.0e-5 * std::max(peak, 1.0e-3),
               "batched against single: " + num(diff, 9) + " at peak " + num(peak, 5));

        beginTest("filter FM in the voice: filterN_fm_src / filterN_fm_amt equal the route (bit for bit)");
        {
            auto w = v;
            setValue(w, "filter_fm_src", 1.0f); // LFO 1 audio
            setValue(w, "filter_fm_amt", static_cast<float>(amount));
            const auto viaParams = renderNote(w, note, 20000);
            const auto viaRoute = renderNote(v, note, 20000, routes);
            expect(viaParams == viaRoute && !viaRoute.empty());
        }

        beginTest(
            "filter FM in the voice: a filter FM source keeps its bus buffer when eight other audio sources are routed");
        {
            auto w = v;
            setValue(w, "filter_fm_src", static_cast<float>(kAudioEnvFirst + 1)); // per-sample mod envelope: bus source 12
            setValue(w, "filter_fm_amt", 0.5f);
            std::vector<ModRoute> busy;
            for (int a = 0; a < 8; ++a)
                busy.push_back({kSourceAudio + a, kTargetVoiceAmp, 1.0e-6f, 0, 0.0f, 0.0f});
            const auto alone = renderNote(w, note, 20000);
            const auto crowded = renderNote(w, note, 20000, busy);
            auto plain = w;
            setValue(plain, "filter_fm_amt", 0.0f);
            const auto unmodulated = renderNote(plain, note, 20000);
            expect(!alone.empty() && alone.size() == crowded.size() && alone.size() == unmodulated.size());
            double peak = 0.0, lost = 0.0, fmEffect = 0.0;
            for (size_t i = 0; i < alone.size() && i < crowded.size() && i < unmodulated.size(); ++i)
            {
                peak = std::max(peak, static_cast<double>(std::abs(alone[i])));
                lost = std::max(lost, static_cast<double>(std::abs(alone[i] - crowded[i])));
                fmEffect = std::max(fmEffect, static_cast<double>(std::abs(alone[i] - unmodulated[i])));
            }
            logMessage("filter FM with eight other bus sources: difference " + String(lost, 9) + ", FM effect " +
                       String(fmEffect, 9) + ", peak " + String(peak, 6));
            expect(fmEffect > 0.05 * peak, "the FM is audible " + num(fmEffect, 9));
            expect(lost <= 1.0e-3 * peak, "FM source lost its buffer " + num(lost, 9) + " at peak " + num(peak, 6));
        }
    }

    void pwmFromAnotherSlot()
    {
        beginTest("PWM from another slot: osc 1 modulates the pulse width of osc 2 sample by sample (P9's gap), lines match "
                  "the exact Fourier integral");
        const int note = 69;
        const double f0 = 440.0;
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_semi", -15.0f); // 185 Hz: an edge that moves slower than the phase (2 pi fm dev < f0)
        setValue(v, "osc1_level", 0.0f);
        setValue(v, "osc2_an_wave", Pulse);
        setValue(v, "osc2_level", 0.5f);
        setValue(v, "osc2_an_pw", 0.5f);
        const double fm = f0 * std::pow(2.0, -15.0 / 12.0);
        const float amount = 0.3125f;
        const double dev = amount * 0.96;
        constexpr size_t n = 1 << 15;
        const auto out =
            renderNote(v, note, static_cast<int>(n) + 12000, {{kOsc1Audio, "osc2_an_pw", amount, 0, 0.0f, 0.0f}});
        expect(!out.empty());
        const double seconds = static_cast<double>(n) / kRate;
        const double fundamentalE = lineAmplitude(out, 8000, n, f0, kRate);
        const double fundamentalR = pwmLineAmplitude(f0, 0.5, dev, fm, f0, seconds);
        double worst = 0.0;
        int compared = 0;
        for (int k = 1; k <= 3; ++k)
            for (int l = -2; l <= 2; ++l)
            {
                const double hz = k * f0 + l * fm;
                if (hz < 50.0 || hz > 15000.0)
                    continue;
                const double e = 20.0 * std::log10(lineAmplitude(out, 8000, n, hz, kRate) / fundamentalE);
                const double r = 20.0 * std::log10(pwmLineAmplitude(f0, 0.5, dev, fm, hz, seconds) / fundamentalR);
                if (r < -50.0)
                    continue;
                ++compared;
                worst = std::max(worst, std::abs(e - r));
            }
        logMessage("PWM lines compared " + String(compared) + ", largest difference " + String(worst, 2) + " dB");
        expect(compared >= 6 && worst <= 0.5,
               "difference " + num(worst, 2) + " dB over " + std::to_string(compared) + " lines");
        // Without the route the pulse is static: the sidebands are not there.
        const auto still = renderNote(v, note, static_cast<int>(n) + 12000);
        const double sideband =
            20.0 * std::log10(lineAmplitude(still, 8000, n, f0 + fm, kRate) / lineAmplitude(still, 8000, n, f0, kRate));
        const double withRoute = 20.0 * std::log10(lineAmplitude(out, 8000, n, f0 + fm, kRate) / fundamentalE);
        expect(sideband < -90.0 && withRoute > -30.0,
               "sideband " + num(withRoute, 1) + " dB with the route, " + num(sideband, 1) + " dB without");

        beginTest("PWM from another slot aliases below -90 dB (modulator at 3x the carrier, deep modulation)");
        {
            auto w = v;
            setValue(w, "osc1_semi", 19.0f);
            setValue(w, "osc1_fine", 1.955001f);
            const auto sig = renderNote(w, note, 12000 + (1 << 15), {{kOsc1Audio, "osc2_an_pw", 0.03f, 0, 0.0f, 0.0f}});
            const auto tail = m::Signal(sig.begin() + 12000, sig.end());
            const auto a = m::measureAliasing(tail, 0, kRate, f0);
            logMessage("PWM aliasing " + String(a.aliasDb, 1) + " dB, worst spur " + String(a.worstSpurDb, 1) + " dB at " +
                       String(a.worstSpurHz, 0) + " Hz");
            expect(a.aliasDb < -90.0, "aliasing " + num(a.aliasDb, 1) + " dB");
        }
    }

    void pitchFm()
    {
        beginTest("pitch FM: an oscillator's output moves another slot's pitch per sample; lines match numerical FM, "
                  "aliasing below -90 dB");
        const int note = 69;
        const double f0 = 440.0;
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_semi", 14.0f);
        setValue(v, "osc1_level", 0.0f);
        setValue(v, "osc2_an_wave", Sine);
        setValue(v, "osc2_level", 0.5f);
        const double fm = f0 * std::pow(2.0, 14.0 / 12.0);
        const float amount = 0.16f; // 24 x 0.16 = 3.84 semitones peak: index about 0.3 x f0 / fm x 4
        constexpr size_t n = 1 << 15;
        const auto out =
            renderNote(v, note, static_cast<int>(n) + 12000, {{kOsc1Audio, "osc2_semi", amount, 0, 0.0f, 0.0f}});
        expect(!out.empty());
        // Reference: phase = integral of f0 2^(2 amount sin(2 pi fm t)) on a 16x grid.
        constexpr size_t over = 16;
        std::vector<double> ref(n * over);
        double phase = 0.0;
        const double dt = 1.0 / (kRate * over);
        for (size_t i = 0; i < ref.size(); ++i)
        {
            const double t = static_cast<double>(i) * dt;
            phase += f0 * std::pow(2.0, 2.0 * amount * std::sin(2.0 * kPiD * fm * t)) * dt;
            ref[i] = std::sin(2.0 * kPiD * phase);
        }
        const double carrierE = lineAmplitude(out, 8000, n, f0, kRate),
                     carrierR = lineAmplitude(ref, 0, ref.size(), f0, kRate * over);
        double worst = 0.0;
        int compared = 0;
        for (int l = -3; l <= 3; ++l)
        {
            const double hz = f0 + l * fm;
            if (hz < 50.0)
                continue;
            const double e = 20.0 * std::log10(lineAmplitude(out, 8000, n, hz, kRate) / carrierE);
            const double r = 20.0 * std::log10(lineAmplitude(ref, 0, ref.size(), hz, kRate * over) / carrierR);
            if (r < -60.0)
                continue;
            ++compared;
            worst = std::max(worst, std::abs(e - r));
        }
        logMessage("FM lines compared " + String(compared) + ", largest difference " + String(worst, 2) + " dB");
        expect(compared >= 3 && worst <= 0.3,
               "difference " + num(worst, 2) + " dB over " + std::to_string(compared) + " lines");

        beginTest("pitch FM of a saw aliases below -90 dB");
        {
            auto w = v;
            setValue(w, "osc2_an_wave", Saw);
            setValue(w, "osc1_semi", 19.0f);
            setValue(w, "osc1_fine", 1.955001f);
            const auto sig = renderNote(w, note, 12000 + (1 << 15), {{kOsc1Audio, "osc2_semi", 0.02f, 0, 0.0f, 0.0f}});
            const auto tail = m::Signal(sig.begin() + 12000, sig.end());
            // 3 f0 modulator: every line is a multiple of f0.
            const auto a = m::measureAliasing(tail, 0, kRate, f0);
            logMessage("saw FM aliasing " + String(a.aliasDb, 1) + " dB, worst spur " + String(a.worstSpurDb, 1) + " dB");
            expect(a.aliasDb < -90.0, "aliasing " + num(a.aliasDb, 1) + " dB");
        }
    }

    void fmAmount()
    {
        beginTest("FM amount: an audio-rate LFO moves oscN_an_xmod_amt per sample (the analog oscillator's own FM)");
        const int note = 69;
        auto v = analogPatch();
        setValue(v, "osc2_an_wave", Sine);
        setValue(v, "osc2_level", 0.5f);
        setValue(v, "osc2_an_xmod_mode", 3.0f); // linear FM
        setValue(v, "osc2_an_xmod_ratio", 2.0f);
        setValue(v, "osc2_an_xmod_amt", 0.3f);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo1_audio_hz", 1000.0f);
        constexpr size_t n = 1 << 15;
        const auto still = renderNote(v, note, static_cast<int>(n) + 12000);
        const auto moved =
            renderNote(v, note, static_cast<int>(n) + 12000, {{kLfo1Audio, "osc2_an_xmod_amt", 0.25f, 0, 0.0f, 0.0f}});
        expect(!still.empty() && !moved.empty());
        const double fundamentalStill = lineAmplitude(still, 8000, n, 440.0, kRate);
        const double fundamentalMoved = lineAmplitude(moved, 8000, n, 440.0, kRate);
        const double sideStill = 20.0 * std::log10(lineAmplitude(still, 8000, n, 1440.0, kRate) / fundamentalStill);
        const double sideMoved = 20.0 * std::log10(lineAmplitude(moved, 8000, n, 1440.0, kRate) / fundamentalMoved);
        logMessage("440 + 1000 Hz line: " + String(sideMoved, 1) + " dB with the LFO on the amount, " +
                   String(sideStill, 1) + " dB without");
        expect(sideStill < -90.0 && sideMoved > -40.0, "sidebands appear only with the modulated amount");
        bool finite = true;
        for (const double x : moved)
            finite = finite && std::isfinite(x);
        expect(finite);
        // The wavetable oscillator's cross-modulation amount (oscN_xmod_amt) takes the same route in groups of four samples.
        auto w = ParameterMap::defaultValues();
        setValue(w, "amp_attack", 0.001f);
        setValue(w, "amp_sustain", 1.0f);
        setValue(w, "master_gain", 0.0f);
        setValue(w, "osc1_level", 0.0f);
        setValue(w, "osc2_level", 0.4f);
        setValue(w, "osc3_level", 0.0f);
        setValue(w, "osc2_xmod_mode", 2.0f); // ring modulation by slot 1
        setValue(w, "osc2_xmod_src", 0.0f);
        setValue(w, "filter_on", 0.0f);
        setValue(w, "lfo1_audio", 1.0f);
        const auto ringStill = renderNote(w, note, 20000);
        const auto ringMoved = renderNote(w, note, 20000, {{kLfo1Audio, "osc2_xmod_amt", 0.4f, 0, 0.0f, 0.0f}});
        expect(!ringMoved.empty() && ringStill != ringMoved, "the ring amount follows the LFO");
    }

    void amplitudeModulation()
    {
        beginTest("amplitude modulation: voice:amp from an audio source gives sidebands of exactly half the depth");
        const int note = 69;
        const double f0 = 440.0, fm = 1000.0;
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_level", 0.5f);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo1_audio_hz", static_cast<float>(fm));
        constexpr size_t n = 1 << 15;
        const auto out =
            renderNote(v, note, static_cast<int>(n) + 12000, {{kLfo1Audio, kTargetVoiceAmp, 0.5f, 0, 0.0f, 0.0f}});
        expect(!out.empty());
        const double carrier = lineAmplitude(out, 8000, n, f0, kRate);
        for (const int l : {-1, 1})
        {
            const double side = lineAmplitude(out, 8000, n, f0 + l * fm, kRate);
            expectWithinAbsoluteError(20.0 * std::log10(side / carrier), 20.0 * std::log10(0.25), 0.1,
                                      "sideband " + String(l));
        }
        const auto flat = renderNote(v, note, static_cast<int>(n) + 12000);
        expectWithinAbsoluteError(20.0 * std::log10(lineAmplitude(flat, 8000, n, f0, kRate) / carrier), 0.0, 0.1,
                                  "the carrier keeps its level");
    }

    void wavetableTargets()
    {
        beginTest("wavetable oscillator: pitch and position take audio-rate modulation in groups of four samples");
        const int note = 69;
        const double f0 = 440.0, fm = 1000.0;
        auto v = ParameterMap::defaultValues();
        setValue(v, "amp_attack", 0.001f);
        setValue(v, "amp_sustain", 1.0f);
        setValue(v, "master_gain", 0.0f);
        setValue(v, "osc1_table", 0.0f);
        setValue(v, "osc1_pos", 0.0f);
        setValue(v, "osc1_level", 0.5f);
        setValue(v, "osc2_level", 0.0f);
        setValue(v, "osc3_level", 0.0f);
        setValue(v, "filter_on", 0.0f);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo1_audio_hz", static_cast<float>(fm));
        constexpr size_t n = 1 << 15;
        const float amount = 0.04f;
        const auto out =
            renderNote(v, note, static_cast<int>(n) + 12000, {{kLfo1Audio, "osc1_semi", amount, 0, 0.0f, 0.0f}});
        expect(!out.empty());
        constexpr size_t over = 16;
        std::vector<double> ref(n * over);
        double phase = 0.0;
        const double dt = 1.0 / (kRate * over);
        for (size_t i = 0; i < ref.size(); ++i)
        {
            const double t = static_cast<double>(i) * dt;
            phase += f0 * std::pow(2.0, 2.0 * amount * std::sin(2.0 * kPiD * fm * t)) * dt;
            ref[i] = std::sin(2.0 * kPiD * phase);
        }
        // The wavetable's first table at position 0 is a sine: compare the lines of the carrier and first sidebands.
        const double carrierE = lineAmplitude(out, 8000, n, f0, kRate),
                     carrierR = lineAmplitude(ref, 0, ref.size(), f0, kRate * over);
        double worst = 0.0;
        for (const int l : {-1, 1, 2})
        {
            const double hz = f0 + l * fm;
            if (hz < 50.0)
                continue;
            const double e = 20.0 * std::log10(lineAmplitude(out, 8000, n, hz, kRate) / carrierE);
            const double r = 20.0 * std::log10(lineAmplitude(ref, 0, ref.size(), hz, kRate * over) / carrierR);
            worst = std::max(worst, std::abs(e - r));
        }
        logMessage("wavetable pitch FM (groups of four samples): sideband error " + String(worst, 2) + " dB");
        expect(worst <= 1.0, "difference " + num(worst, 2) + " dB");
        // Position: the scan moves the spectrum; with the route the output differs from the static one.
        auto w = v;
        setValue(w, "osc1_pos", 0.5f);
        const auto still = renderNote(w, note, 20000);
        const auto scanned = renderNote(w, note, 20000, {{kLfo1Audio, "osc1_pos", 0.3f, 0, 0.0f, 0.0f}});
        expect(!scanned.empty() && still != scanned && m::rmsDbfs(m::Signal(scanned.begin() + 4000, scanned.end())) > -40.0,
               "position scan");
    }

    void limitsAndFallback()
    {
        beginTest("the bus has eight buffers: a ninth distinct audio source is refused; other targets get the chunk mean");
        {
            ModRouteStore store;
            for (int a = 0; a < 8; ++a)
                expect(store.add({kSourceAudio + a, kTargetVoiceAmp, 0.1f, 0, 0.0f, 0.0f}), "source " + std::to_string(a));
            expect(!store.add({kSourceAudio + 8, kTargetVoiceAmp, 0.1f, 0, 0.0f, 0.0f}), "the ninth source");
            expect(store.add({kSourceAudio + 3, "filter_cutoff", 0.1f, 0, 0.0f, 0.0f}),
                   "another route from a source already on the bus");
        }
        {
            // A per-sample envelope on a target without per-sample input gets the chunk mean: during the sustain the
            // mean is the value, so it equals the ordinary envelope route.
            auto v = analogPatch();
            setValue(v, "osc1_an_wave", Sine);
            setValue(v, "osc1_level", 0.3f);
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_type", 8.0f); // a legacy ladder
            setValue(v, "filter_cutoff", 600.0f);
            setValue(v, "filter_res", 0.0f);
            setValue(v, "menv_attack", 0.001f);
            setValue(v, "menv_decay", 0.01f);
            setValue(v, "menv_sustain", 0.6f);
            const auto audio =
                renderNote(v, 60, 16000, {{kSourceAudio + kAudioEnvFirst, "filter_res", 0.5f, 0, 0.0f, 0.0f}});
            const auto plain =
                renderNote(v, 60, 16000, {{static_cast<int>(ModSource::ModEnv), "filter_res", 0.5f, 0, 0.0f, 0.0f}});
            double diff = 0.0;
            for (size_t i = 8000; i < 16000 && i < audio.size() && i < plain.size(); ++i)
                diff = std::max(diff, std::abs(audio[i] - plain[i]));
            expect(!audio.empty() && !plain.empty() && diff < 1.0e-3,
                   "chunk mean equals the envelope in its sustain: " + num(diff, 6));
        }
    }

    // A graph product from text; null with the error on failure.
    static std::shared_ptr<const graph::CompiledProduct> compileText(const std::string& text, std::string& error)
    {
        graph::ProductDef def;
        if (!graph::productFromText(text, def, error))
            return nullptr;
        return graph::compile(def, error);
    }

    static std::vector<double> renderProduct(const std::shared_ptr<const graph::CompiledProduct>& compiled, int note,
                                             int samples)
    {
        EngineConfig cfg;
        cfg.compiled = compiled;
        SynthEngine engine(cfg);
        const auto values = compiled->defaultValues();
        engine.prepare(kRate);
        engine.setParameterValues(values.data(), static_cast<int>(values.size()));
        engine.noteOn(note, 1.0f);
        return m::toDouble(renderBlocks(engine, samples, 512, true).left);
    }

    void graphEdges()
    {
        beginTest("graph: audio-rate edges are explicit (audio output into an audio-rate input), other wirings are refused");
        {
            const auto check = [&](const std::string& edges)
            {
                const std::string text = R"({"format": "crucible-product", "version": 1, "id": "t",
                    "voice": { "nodes": [ { "id": "osc", "type": "osc.analog", "slot": 1 }, { "id": "lfo", "type": "lfo.audio" },
                        { "id": "env", "type": "env.adsr" }, { "id": "filt", "type": "filter.analog" }, { "id": "out", "type": "mix.out" } ],
                    "edges": [ { "from": "osc.out", "to": "filt.in" }, { "from": "filt.out", "to": "out.in" })" +
                                         edges + "] } }";
                std::string error;
                compileText(text, error);
                return error;
            };
            expect(check(R"(, { "from": "lfo.out", "to": "filt.cutoff_fm", "amount": 0.2 })").empty(),
                   "audio into audio-rate");
            expect(check(R"(, { "from": "osc.out", "to": "filt.cutoff_fm" })").empty(), "an oscillator into audio-rate");
            const auto e1 = check(R"(, { "from": "env.out", "to": "filt.cutoff_fm" })");
            expect(e1.find("audio-rate input takes an audio output") != std::string::npos,
                   "modulation into audio-rate: " + e1);
            const auto e2 = check(R"(, { "from": "lfo.out", "to": "filt.cutoff" })");
            expect(e2.find("audio and modulation ports cannot be connected") != std::string::npos,
                   "audio into modulation: " + e2);
            expect(portTypeFromName("audio.rate", *std::make_unique<PortType>()), "port type name");
        }

        beginTest("graph: lfo.audio into filter.analog.cutoff_fm gives the ladder's sidebands (0.5 dB), per sample");
        {
            const int note = 79;
            const double carrier = hzOf(note), modHz = 375.0, fc0 = 2000.0, amount = 0.2;
            const std::string text = R"({"format": "crucible-product", "version": 1, "id": "test.fm",
                "voice": { "nodes": [
                    { "id": "osc", "type": "osc.analog", "slot": 1 },
                    { "id": "lfo", "type": "lfo.audio", "params": { "rate": 375.0 } },
                    { "id": "filt", "type": "filter.analog", "params": { "model": 0, "cutoff": 2000.0, "res": 0.0, "drive": 0.0, "character": 0.0, "tolerance": 0.0 } },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "osc.out", "to": "filt.in" },
                    { "from": "lfo.out", "to": "filt.cutoff_fm", "amount": 0.2 },
                    { "from": "filt.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 0.02, "osc1_an_wave": 3.0, "osc1_an_drift": 0.0 } })";
            std::string error;
            const auto compiled = compileText(text, error);
            expect(compiled != nullptr, error);
            if (compiled)
            {
                constexpr size_t n = 1 << 14;
                constexpr size_t settle = 8192;
                const auto out = renderProduct(compiled, note, static_cast<int>(n + settle) + 4096);
                std::vector<double> input(n + settle), oct(n + settle);
                for (size_t i = 0; i < input.size(); ++i)
                {
                    input[i] = 0.01 * std::sin(2.0 * kPiD * carrier * static_cast<double>(i) / kRate);
                    oct[i] = 5.0 * amount * std::sin(2.0 * kPiD * modHz * static_cast<double>(i) / kRate);
                }
                const auto oracle = ladderOracle(input, fc0, oct, kRate);
                const double carrierE = toneAmplitude(out, settle + 2048, n, carrier),
                             carrierO = toneAmplitude(oracle, settle, n, carrier);
                double worst = 0.0;
                for (int k = -3; k <= 4; ++k)
                {
                    const double hz = carrier + k * modHz;
                    const double e = 20.0 * std::log10(toneAmplitude(out, settle + 2048, n, hz) / carrierE);
                    const double o = 20.0 * std::log10(toneAmplitude(oracle, settle, n, hz) / carrierO);
                    if (o > -60.0)
                        worst = std::max(worst, std::abs(e - o));
                }
                logMessage("graph filter FM: largest sideband error " + String(worst, 2) + " dB");
                expect(worst <= 0.5, "sideband error " + num(worst, 2));
            }
        }

        beginTest(
            "graph: an oscillator node into another's width_mod is PWM from another slot; vca.am is amplitude modulation");
        {
            const std::string text = R"({"format": "crucible-product", "version": 1, "id": "test.pwm",
                "voice": { "nodes": [
                    { "id": "mod", "type": "osc.analog", "slot": 1 },
                    { "id": "car", "type": "osc.analog", "slot": 2 },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "mod.out", "to": "car.width_mod", "amount": 0.3 },
                    { "from": "car.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 1.0, "osc1_an_wave": 3.0, "osc1_semi": -15.0, "osc1_an_drift": 0.0,
                              "osc2_level": 0.5, "osc2_an_wave": 1.0, "osc2_an_pw": 0.5, "osc2_an_drift": 0.0 } })";
            std::string error;
            const auto compiled = compileText(text, error);
            expect(compiled != nullptr, error);
            if (compiled)
            {
                constexpr size_t n = 1 << 15;
                const double f0 = 440.0, fm = f0 * std::pow(2.0, -15.0 / 12.0);
                const auto out = renderProduct(compiled, 69, static_cast<int>(n) + 12000);
                const double seconds = static_cast<double>(n) / kRate;
                const double fundamentalE = lineAmplitude(out, 8000, n, f0, kRate);
                const double fundamentalR = pwmLineAmplitude(f0, 0.5, 0.3, fm, f0, seconds);
                double worst = 0.0;
                int compared = 0;
                for (int k = 1; k <= 3; ++k)
                    for (int l = -2; l <= 2; ++l)
                    {
                        const double hz = k * f0 + l * fm;
                        if (hz < 50.0 || hz > 15000.0)
                            continue;
                        const double e = 20.0 * std::log10(lineAmplitude(out, 8000, n, hz, kRate) / fundamentalE);
                        const double r = 20.0 * std::log10(pwmLineAmplitude(f0, 0.5, 0.3, fm, hz, seconds) / fundamentalR);
                        if (r < -50.0)
                            continue;
                        ++compared;
                        worst = std::max(worst, std::abs(e - r));
                    }
                logMessage("graph PWM lines compared " + String(compared) + ", largest difference " + String(worst, 2) +
                           " dB");
                expect(compared >= 6 && worst <= 0.5, "difference " + num(worst, 2) + " dB");
            }
            const std::string am = R"({"format": "crucible-product", "version": 1, "id": "test.am",
                "voice": { "nodes": [
                    { "id": "car", "type": "osc.analog", "slot": 1 },
                    { "id": "lfo", "type": "lfo.audio", "params": { "rate": 1000.0 } },
                    { "id": "vca", "type": "amp.vca" },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "car.out", "to": "vca.in" },
                    { "from": "lfo.out", "to": "vca.am", "amount": 0.5 },
                    { "from": "vca.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 0.5, "osc1_an_wave": 3.0, "osc1_an_drift": 0.0 } })";
            const auto amCompiled = compileText(am, error);
            expect(amCompiled != nullptr, error);
            if (amCompiled)
            {
                constexpr size_t n = 1 << 15;
                const auto out = renderProduct(amCompiled, 69, static_cast<int>(n) + 12000);
                const double carrier = lineAmplitude(out, 8000, n, 440.0, kRate);
                for (const int l : {-1, 1})
                    expectWithinAbsoluteError(
                        20.0 * std::log10(lineAmplitude(out, 8000, n, 440.0 + l * 1000.0, kRate) / carrier),
                        20.0 * std::log10(0.25), 0.1, "vca.am sideband " + String(l));
            }
        }
    }

    // A thread that renders an audio-rate pulse width for the first time must not allocate (a thread_local work space is
    // allocated by the runtime on first use of the thread). Counted through the malloc zone, since the TLS block does not go
    // through operator new.
    void freshThreadScratch()
    {
        beginTest("real time: the first audio-rate pulse-width render on a new thread does not allocate");
#if defined(__APPLE__)
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_level", 0.0f);
        setValue(v, "osc2_an_wave", Pulse);
        setValue(v, "osc2_level", 0.4f);
        SynthEngine engine;
        prepare(engine, v);
        expect(engine.modRoutes().add({kOsc1Audio, "osc2_an_pw", 0.3f, 0, 0.0f, 0.0f}));
        engine.noteOn(60, 1.0f);
        std::vector<float> left(32), right(32);
        for (int block = 0; block < 8; ++block)
            engine.render(left.data(), right.data(), 32);
        size_t blocks = 0;
        std::thread worker(
            [&]
            {
                malloc_statistics_t before{}, after{};
                malloc_zone_statistics(nullptr, &before);
                for (int block = 0; block < 4; ++block)
                    engine.render(left.data(), right.data(), 32);
                malloc_zone_statistics(nullptr, &after);
                blocks = after.blocks_in_use > before.blocks_in_use ? after.blocks_in_use - before.blocks_in_use : 0;
            });
        worker.join();
        expectEquals(static_cast<int>(blocks), 0, "heap blocks allocated by the first render on a new thread");
#else
        expect(true);
#endif
    }

    // Relative RMS difference between a note at a static offset and the same offset delivered as a (constant) audio-rate
    // source on the pitch: the audio-rate path has to move the master phase of sync, FM and ring exactly like the static
    // one.
    double pitchRouteDifference(int mode, double ratio, int note)
    {
        constexpr float semis = 6.0f;
        auto v = analogPatch();
        setValue(v, "osc2_an_wave", Saw);
        setValue(v, "osc2_level", 0.5f);
        setValue(v, "osc2_an_xmod_mode", static_cast<float>(mode));
        setValue(v, "osc2_an_xmod_ratio", static_cast<float>(ratio));
        setValue(v, "osc2_an_xmod_amt", 0.3f);
        auto stat = v;
        setValue(stat, "osc2_semi", semis);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo1_audio_hz", 1.0f);
        setValue(v, "lfo1_shape", 3.0f); // square: constant over the first half second
        constexpr int n = 12000;
        const auto a = renderNote(stat, note, n);
        const auto b = renderNote(v, note, n, {{kLfo1Audio, "osc2_semi", semis / 24.0f, 0, 0.0f, 0.0f}});
        expect(a.size() == b.size() && !a.empty());
        double num2 = 0.0, den = 0.0;
        for (size_t i = 4000; i < a.size() && i < b.size(); ++i)
        {
            num2 += (a[i] - b[i]) * (a[i] - b[i]);
            den += a[i] * a[i];
        }
        return std::sqrt(num2 / std::max(den, 1.0e-30));
    }

    void pitchRouteCrossMod()
    {
        beginTest("audio-rate pitch moves the master phase of hard sync, soft sync, linear FM, exp FM and ring like the "
                  "static pitch");
        const struct
        {
            int mode;
            double ratio;
            int note;
            const char* name;
        } cases[] = {{0, 1.0, 60, "plain"},     {1, 1.5, 60, "hard sync"}, {1, 1.5, 96, "hard sync high"},
                     {2, 1.5, 72, "soft sync"}, {3, 1.0, 60, "linear FM"}, {4, 2.0, 60, "exp FM"},
                     {5, 3.0, 60, "ring"}};
        for (const auto& c : cases)
        {
            const double d = pitchRouteDifference(c.mode, c.ratio, c.note);
            logMessage(std::string(c.name) + ": relative difference " + String(d, 4).str());
            expect(d < 0.02, std::string(c.name) + ": relative difference " + num(d, 4));
        }
    }

    void realTime()
    {
        beginTest("real time: zero allocations with the bank, audio-rate routes, filter FM and audio-rate LFOs running");
        auto v = analogPatch();
        setValue(v, "osc1_an_wave", Sine);
        setValue(v, "osc1_level", 0.0f);
        setValue(v, "osc2_an_wave", Pulse);
        setValue(v, "osc2_level", 0.4f);
        setValue(v, "filter_on", 1.0f);
        setValue(v, "filter_type", 17.0f);
        setValue(v, "lfo1_audio", 1.0f);
        setValue(v, "lfo2_audio", 2.0f);
        setValue(v, "lfo2_shape", 2.0f);
        setValue(v, "filter_fm_src", 2.0f);
        setValue(v, "filter_fm_amt", 0.1f);
        setValue(v, "bank1_type", static_cast<float>(ModulatorType::Chaos));
        setValue(v, "bank1_mode", 1.0f);
        setValue(v, "bank2_type", static_cast<float>(ModulatorType::Turing));
        setValue(v, "bank2_mode", 2.0f);
        setValue(v, "bank3_type", static_cast<float>(ModulatorType::Follower));
        setValue(v, "bank3_scope", 2.0f);
        setValue(v, "bank4_type", static_cast<float>(ModulatorType::PitchTracker));
        SynthEngine engine;
        prepare(engine, v);
        const auto& routes = engine.modRoutes();
        (void)routes;
        expect(engine.modRoutes().add({kOsc1Audio, "osc2_an_pw", 0.3f, 0, 0.0f, 0.0f}));
        expect(engine.modRoutes().add({kLfo1Audio, "osc2_semi", 0.02f, 0, 0.0f, 0.0f}));
        expect(engine.modRoutes().add({kLfo1Audio, kTargetVoiceAmp, 0.2f, 0, 0.0f, 0.0f}));
        expect(engine.modRoutes().add({kSourceModulator, "filter_cutoff", 0.2f, 2, 0.0f, 0.0f}));
        expect(engine.modRoutes().add({kSourceModulator + 1, "filter_res", 0.3f, 2, 0.0f, 0.0f}));
        expect(engine.modRoutes().add({kSourceModulator + 2, routeAmountTargetId(3), 0.2f, 0, 0.0f, 0.0f}));
        for (int note : {48, 55, 60})
            engine.noteOn(note, 1.0f);
        std::vector<float> left(128), right(128);
        for (int block = 0; block < 32; ++block)
            engine.render(left.data(), right.data(), 128);
        float peak = 0.0f;
        bool finite = true;
        gAllocations = 0;
        gTrackAllocations = true;
        for (int block = 0; block < 375; ++block)
        {
            engine.render(left.data(), right.data(), 128);
            for (int i = 0; i < 128; ++i)
            {
                finite =
                    finite && std::isfinite(left[static_cast<size_t>(i)]) && std::isfinite(right[static_cast<size_t>(i)]);
                peak = std::max(peak, std::abs(left[static_cast<size_t>(i)]));
            }
        }
        gTrackAllocations = false;
        expect(finite && peak > 0.01f && peak < 4.0f, "audible and finite: peak " + num(peak, 3));
        expectEquals(static_cast<int>(gAllocations), 0, "allocations while rendering");
    }
};

static AudioRateModTests audioRateModTests;
