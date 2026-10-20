// SPDX-License-Identifier: MIT
// fx.saturator: the mix-grade saturation module on the shared models of include/crucible/Saturation.h (tubes, tape
// hysteresis, transformer, diode, rail and the memoryless curves with antiderivative anti-aliasing). Behind the
// half-band chains of include/crucible/fx/HalfBandChain.h: emphasis, drive and model, de-emphasis, loudness-matched gain,
// then a wet / dry mix whose dry path is delayed by exactly the latency of the wet path.
#include "FxModuleSupport.h"
#include "SaturatorParts.h"

#include <crucible/Saturation.h>
#include <crucible/fx/Loudness.h>
#include <crucible/fx/HalfBandChain.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <cstdint>
#include <cstdlib>

namespace crucible::modules::fx
{
namespace
{
using crucible::fx::HalfBandChain;
using saturation::Model;

constexpr float kMaxDriveDb = 36.0f;
using detail::kPiD;

// Model order of the "Model" choice. The numbers are part of the patch format: append only.
struct ModelEntry
{
    const char* name;
    Model model;
};
constexpr ModelEntry kModels[] = {
    {"Soft Clip", Model::Soft},
    {"Hard Clip", Model::Hard},
    {"Op-Amp Rail", Model::Rail},
    {"Diode", Model::Diode},
    {"Wavefolder", Model::Fold},
    {"Tube Triode", Model::TriodeSE},
    {"Tube Push-Pull", Model::TriodePP},
    {"Tube Pentode", Model::Pentode},
    {"Tape", Model::Tape},
    {"Transformer", Model::Transformer},
    {"Rectify Half", Model::RectHalf},
    {"Rectify Full", Model::RectFull},
    {"Sine Fold", Model::SineFold},
    {"Linear Fold", Model::LinFold},
    {"Zero-Square", Model::ZeroSquare},
    {"Asymmetric", Model::Asym},
};
constexpr int kNumModels = static_cast<int>(sizeof(kModels) / sizeof(kModels[0]));

// One setting of the saturation library from the module's controls. character: 0 to 100 %; for the tubes it is the
// sag (level-dependent bias shift), for tape the loop width and for the transformer the flux corner (2.4 Hz at 0 %,
// 12 Hz at 50 %, 60 Hz at 100 %). The other models ignore it.
saturation::Settings libraryFor(int modelIndex, float driveDb, float bias, float character, int integrator)
{
    saturation::Settings s;
    s.model = kModels[std::clamp(modelIndex, 0, kNumModels - 1)].model;
    // The wavefolder and the hysteresis have a shorter drive range (half and two thirds): beyond it a 5 kHz tone moves
    // the folder through dozens of folds, or the field of the tape model by many knee widths, in one oversampled step.
    // The appended shapers with a steep or folding transfer have shorter ranges for the same reason (the full rectifier
    // and the sine fold produce harmonics up to many times the fundamental, the linear fold has twice the slope and a
    // kink at each fold, the zero-square approaches a square wave).
    float driveScale = 1.0f;
    switch (s.model)
    {
    case Model::Fold:
        driveScale = 0.5f;
        break;
    case Model::Tape:
    case Model::ZeroSquare:
        driveScale = 2.0f / 3.0f;
        break;
    case Model::RectFull:
        driveScale = 0.6f;
        break;
    case Model::SineFold:
        driveScale = 0.6f;
        break;
    case Model::LinFold:
        driveScale = 0.2f;
        break;
    default:
        break;
    }
    s.driveDb = driveDb * driveScale;
    s.bias = bias;
    s.integrator = integrator;
    s.tapeFast = true;
    s.adaaOrder = 2;
    const float c = std::clamp(character * 0.01f, 0.0f, 1.0f);
    // The push-pull pair cancels the even-order products that a one-sided sag would bring back.
    s.sag = s.model == Model::TriodePP ? 0.0f : 0.5f * c;
    s.hysteresis = c;
    s.cornerHz = 12.0f * std::pow(5.0f, 2.0f * (c - 0.5f));
    return s;
}

// Pink noise (Paul Kellet's economy filter) on a fixed seed, for the loudness calibration.
class PinkNoise
{
public:
    float next()
    {
        state = state * 1664525u + 1013904223u;
        const double w = static_cast<double>(static_cast<std::int32_t>(state)) / 2147483648.0;
        b0 = 0.99886 * b0 + w * 0.0555179;
        b1 = 0.99332 * b1 + w * 0.0750759;
        b2 = 0.96900 * b2 + w * 0.1538520;
        b3 = 0.86650 * b3 + w * 0.3104856;
        b4 = 0.55000 * b4 + w * 0.5329522;
        b5 = -0.7616 * b5 - w * 0.0168980;
        const double out = b0 + b1 + b2 + b3 + b4 + b5 + b6 + w * 0.5362;
        b6 = w * 0.115926;
        return static_cast<float>(out * 0.11);
    }

private:
    std::uint32_t state = 12345u;
    double b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0;
};

// Gain that keeps the K-weighted loudness of pink noise at -18 dB RMS unchanged, per model and drive, measured once
// through the module's own Standard-quality chain (a settle stretch, then 0.75 s). The signal is a fixed pseudo-random
// sequence, so the table is the same on every machine.
class AutoGainTable
{
public:
    static constexpr int kStepDb = 2;
    static constexpr int kSteps = static_cast<int>(kMaxDriveDb) / kStepDb + 1;

    AutoGainTable()
    {
        constexpr double rate = 48000.0;
        constexpr size_t settle = 8192, count = 36000;
        std::vector<float> in(settle + count);
        PinkNoise pink;
        double sq = 0.0;
        for (auto& v : in)
        {
            v = pink.next();
            sq += static_cast<double>(v) * v;
        }
        const float scale = static_cast<float>(0.125 / std::sqrt(sq / static_cast<double>(in.size())));
        for (auto& v : in)
            v *= scale;
        const double inEnergy = weightedEnergy(in, settle, rate);
        std::vector<float> x(in.size());
        for (int m = 0; m < kNumModels; ++m)
            for (int k = 0; k < kSteps; ++k)
            {
                saturation::OversampledSaturator os;
                os.prepare(rate, 4, OversamplingFilter::HalfBand, Quality::Standard, 512);
                os.set(libraryFor(m, static_cast<float>(k * kStepDb), 0.0f, 50.0f, 2));
                x = in;
                for (size_t i = 0; i < x.size(); i += 512)
                    os.process(x.data() + i, static_cast<int>(std::min<size_t>(512, x.size() - i)));
                const double outEnergy = weightedEnergy(x, settle, rate);
                db[static_cast<size_t>(m)][static_cast<size_t>(k)] =
                    outEnergy > 0.0 ? static_cast<float>(10.0 * std::log10(inEnergy / outEnergy)) : 0.0f;
            }
    }

    // Gain in dB for the model at a drive in dB (linear interpolation between the 2 dB steps).
    float gainDb(int model, float driveDb) const
    {
        const float p = std::clamp(driveDb, 0.0f, kMaxDriveDb) / static_cast<float>(kStepDb);
        const int k = std::min(static_cast<int>(p), kSteps - 2);
        const float f = p - static_cast<float>(k);
        const auto& row = db[static_cast<size_t>(std::clamp(model, 0, kNumModels - 1))];
        return row[static_cast<size_t>(k)] + (row[static_cast<size_t>(k + 1)] - row[static_cast<size_t>(k)]) * f;
    }

private:
    static double weightedEnergy(const std::vector<float>& x, size_t settle, double rate)
    {
        crucible::fx::KWeighting k;
        k.prepare(rate);
        k.energy(x.data(), static_cast<int>(settle), 0.0);
        return k.energy(x.data() + settle, static_cast<int>(x.size() - settle), 0.0);
    }

    std::array<std::array<float, kSteps>, kNumModels> db{};
};

const AutoGainTable& autoGainTable()
{
    static const AutoGainTable table;
    return table;
}

// Tilt pair around the shaper: a first-order high shelf, balanced so that the gain at the corner frequency is about
// unity (low frequencies are reduced by half the tilt, high frequencies raised by half of it), and its exact inverse
// afterwards, so with a linear shaper the pair is the identity. k + 1 is the high-frequency gain over the low-frequency
// gain, 10^(tilt/20); a is the low-frequency gain.
struct Emphasis
{
    double s = 0.0;
    double run(double x, double k, double a, double g)
    {
        const double v = (x - s) * g;
        const double lp = v + s;
        s = lp + v;
        return a * (x + k * (x - lp));
    }
};
struct DeEmphasis
{
    double s = 0.0;
    double run(double x, double k, double a, double g)
    {
        const double u = x / a;
        const double y = (u + k * (1.0 - g) * s) / (1.0 + k - k * g);
        const double lp = g * y + (1.0 - g) * s;
        s = 2.0 * lp - s;
        return y;
    }
};

using detail::Flatten;
using detail::Pad;

// The models that run on antiderivative anti-aliasing (the hysteresis and the transformer do not).
bool usesAdaa(int modelIndex)
{
    const Model m = kModels[std::clamp(modelIndex, 0, kNumModels - 1)].model;
    return m != Model::Tape && m != Model::Transformer;
}

// The resonant filter that sits before or after the shaper: a zero-delay-feedback state-variable filter (trapezoidal
// integration, the standard topological form) run at the base rate, so a pre filter feeds the oversampler and cannot
// alias, and a post filter works on the already decimated signal. The band-pass output is scaled by the damping, so its
// peak gain is unity at every resonance. Cutoff glides geometrically and resonance linearly, both per sample across the
// grid chunk and landing exactly on the target, so a constant setting runs on fixed coefficients.
class ResonantFilter
{
public:
    enum Type
    {
        LowPass = 0,
        BandPass = 1,
        HighPass = 2
    };

    void prepare(double rate)
    {
        sr = rate;
        reset(1000.0f, 0.0f);
    }
    // Damping k = 1 / Q, with Q from 0.707 (resonance 0) up to about 21 (resonance 1), exponentially.
    static double dampingFor(double resonance)
    {
        return 1.0 / (0.70710678 * std::pow(30.0, std::clamp(resonance, 0.0, 1.0)));
    }
    void clear() { s1[0] = s1[1] = s2[0] = s2[1] = 0.0; }
    void reset(float hz, float res)
    {
        cutoff.reset(clampHz(hz));
        resonance.reset(res);
        settle();
        clear();
    }
    void setTarget(float hz, float res, int samples)
    {
        cutoff.setTarget(clampHz(hz), samples);
        resonance.setTarget(res, samples);
        if (!cutoff.ramping() && !resonance.ramping())
            settle();
    }
    void setType(int t) { type = std::clamp(t, 0, 2); }

    void run(float* l, float* r, int n)
    {
        if (!cutoff.ramping() && !resonance.ramping())
        {
            for (int i = 0; i < n; ++i)
            {
                l[i] = static_cast<float>(tick(0, static_cast<double>(l[i])));
                r[i] = static_cast<float>(tick(1, static_cast<double>(r[i])));
            }
            return;
        }
        for (int i = 0; i < n; ++i)
        {
            coefficients(cutoff.next(), resonance.next());
            l[i] = static_cast<float>(tick(0, static_cast<double>(l[i])));
            r[i] = static_cast<float>(tick(1, static_cast<double>(r[i])));
        }
    }
    bool ramping() const { return cutoff.ramping() || resonance.ramping(); }

private:
    float clampHz(float hz) const { return std::clamp(hz, 20.0f, std::min(20000.0f, 0.45f * static_cast<float>(sr))); }
    void settle() { coefficients(cutoff.value(), resonance.value()); }
    void coefficients(float hz, float res)
    {
        const double g = std::tan(kPiD * static_cast<double>(hz) / sr);
        k = dampingFor(static_cast<double>(res));
        a1 = 1.0 / (1.0 + g * (g + k));
        a2 = g * a1;
        a3 = g * a2;
    }
    double tick(size_t c, double x)
    {
        const double v3 = x - s2[c];
        const double v1 = a1 * s1[c] + a2 * v3;
        const double v2 = s2[c] + a2 * s1[c] + a3 * v3;
        s1[c] = 2.0 * v1 - s1[c];
        s2[c] = 2.0 * v2 - s2[c];
        switch (type)
        {
        case BandPass:
            return k * v1;
        case HighPass:
            return x - k * v1 - v2;
        default:
            return v2;
        }
    }

    double sr = 48000.0, a1 = 0.0, a2 = 0.0, a3 = 0.0, k = 1.0;
    double s1[2] = {0.0, 0.0}, s2[2] = {0.0, 0.0};
    int type = LowPass;
    crucible::fx::LogRamp cutoff;
    crucible::fx::LinearRamp resonance;
};

// Short names for LCD displays: the ids stay spelled out, the short names are at most six characters.
void shortNames(std::vector<ParamDef>& p)
{
    static const std::pair<const char*, const char*> names[] = {
        {"fx_saturator_on", "On"},         {"fx_saturator_mix", "Mix"},          {"fx_saturator_model", "Model"},
        {"fx_saturator_drive", "Drive"},   {"fx_saturator_bias", "Bias"},        {"fx_saturator_output", "Out"},
        {"fx_saturator_tilt", "Tilt"},     {"fx_saturator_tilt_freq", "TiltHz"}, {"fx_saturator_character", "Char"},
        {"fx_saturator_autogain", "Auto"}, {"fx_pos_saturator", "Slot"},
        {"fx_saturator_filter_mode", "Filter"}, {"fx_saturator_filter_type", "FType"},
        {"fx_saturator_filter_cutoff", "FCut"}, {"fx_saturator_filter_res", "FRes"},
    };
    for (auto& d : p)
        for (const auto& n : names)
            if (d.id == n.first)
                d.shortName = n.second;
}

// One oversampling configuration: the filter chains, the shapers and everything else that runs at the oversampled rate
// or depends on it. The module keeps one or two of them (a lower and a higher factor) and moves between them with the
// drive, see Instance.
class Path
{
public:
    static constexpr int kChunk = 512;

    // stopBandDb: rejection of the half-band stages, which sets their length and so the latency.
    void prepare(double rate, int stages, double stopBandDb)
    {
        sr = rate;
        HalfBandChain::Design designs[HalfBandChain::kMaxStages];
        for (int i = 0; i < stages; ++i)
            designs[i] = HalfBandChain::designFor(sr, i, stopBandDb);
        for (size_t k = 0; k < 2; ++k)
        {
            chain[k].prepare(stages, designs, kChunk);
            wet[k].assign(static_cast<size_t>(kChunk) << stages, 0.0f);
        }
        factor = 1 << stages;
        osRate = sr * static_cast<double>(factor);
        // The wet path is the oversampling filters plus, for the antiderivative models, one oversampled sample for the
        // second-order ADAA and three base-rate samples for the flattening filter. Every model is brought to the same
        // latency, a whole number of base-rate samples, by padding with oversampled samples.
        const int firOs = chain[0].latencyFine();
        const int adaaOs = firOs + 1 + Flatten::kTaps * factor;
        const int total = (adaaOs + factor - 1) / factor * factor;
        padAdaa = total - adaaOs;
        padPlain = total - firOs;
        latency = total / factor;
        for (auto& f : flatten)
            f.design(sr, osRate);
        reset();
    }

    int latencySamples() const { return latency; }
    int oversampling() const { return factor; }
    // Extra base-rate delay that brings this path to the latency of the slowest one.
    void alignTo(int latencyTarget) { basePadding = latencyTarget - latency; }

    // A fresh library state, glides included, so that a reset path renders what a new one does.
    void reset()
    {
        for (auto& c : chain)
            c.reset();
        for (auto& s : sat)
        {
            s = saturation::Saturator{};
            s.prepare(osRate);
        }
        for (auto& s : satNext)
        {
            s = saturation::Saturator{};
            s.prepare(osRate);
        }
        clearMemory();
    }

    // The memory outside the shapers: emphasis pair, flattening filter, paddings.
    void clearMemory()
    {
        pre = {};
        post = {};
        for (auto& f : flatten)
            f.clear();
        for (auto& d : pad)
            d.clear();
        for (auto& d : basePad)
            d.clear();
    }

    void set(const saturation::Settings& s)
    {
        for (auto& x : sat)
            x.set(s);
    }
    void setNext(const saturation::Settings& s)
    {
        for (auto& x : satNext)
            x.set(s);
    }
    void swapModels()
    {
        std::swap(sat, satNext);
        for (auto& f : flatten)
            f.clear();
        for (auto& d : pad)
            d.clear();
    }

    // Takes over the slow state of another path (drive glide, envelopes, loop position, emphasis states) so that this
    // one can be warmed up on a stretch of recent input and continue where the other one is.
    void adopt(const Path& o)
    {
        for (size_t c = 0; c < 2; ++c)
        {
            sat[c].adoptState(o.sat[c]);
            satNext[c].adoptState(o.satNext[c]);
            pre[c].s = o.pre[c].s;
            post[c].s = o.post[c].s;
        }
    }

    struct Tone
    {
        bool shelf = false;
        double hz = 1000.0, k = 0.0, a = 1.0;
    };

    // in and out may be the same arrays. adaa: the active model is one of the antiderivative ones; warming: the next model
    // runs alongside.
    void run(const float* const* in, float* const* out, int n, bool adaa, bool warming, const Tone& tone)
    {
        const double g = std::tan(kPiD * std::min(tone.hz, 0.45 * osRate) / osRate);
        shelfG = g / (1.0 + g);
        shelfK = tone.k;
        shelfA = tone.a;
        const bool shelf = tone.shelf;
        const int count = n * factor;
        for (size_t c = 0; c < 2; ++c)
        {
            chain[c].up(in[c], n, wet[c].data());
            float* p = wet[c].data();
            if (shelf)
                for (int i = 0; i < count; ++i)
                    p[i] = static_cast<float>(pre[c].run(p[i], shelfK, shelfA, shelfG));
        }
        if (warming)
        {
            for (size_t c = 0; c < 2; ++c)
                std::copy(wet[c].begin(), wet[c].begin() + count, scratch[c].begin());
            saturation::Saturator::processPair(satNext[0], satNext[1], scratch[0].data(), scratch[1].data(), count);
        }
        saturation::Saturator::processPair(sat[0], sat[1], wet[0].data(), wet[1].data(), count);
        for (size_t c = 0; c < 2; ++c)
        {
            float* p = wet[c].data();
            const int padding = adaa ? padAdaa : padPlain;
            if (padding > 0)
                pad[c].run(p, count, padding);
            if (shelf)
                for (int i = 0; i < count; ++i)
                    p[i] = static_cast<float>(post[c].run(p[i], shelfK, shelfA, shelfG));
            chain[c].down(p, n, out[c]);
            if (adaa)
                flatten[c].run(out[c], n);
            if (basePadding > 0)
                basePad[c].run(out[c], n, basePadding);
        }
    }

private:
    double sr = 48000.0, osRate = 48000.0, shelfG = 0.0, shelfK = 0.0, shelfA = 1.0;
    int factor = 1, padAdaa = 0, padPlain = 0, latency = 0, basePadding = 0;
    std::array<HalfBandChain, 2> chain;
    std::array<std::vector<float>, 2> wet;
    std::array<saturation::Saturator, 2> sat, satNext;
    std::array<std::array<float, kChunk * 16>, 2> scratch{};
    std::array<Flatten, 2> flatten;
    std::array<Pad, 2> pad, basePad;
    std::array<Emphasis, 2> pre;
    std::array<DeEmphasis, 2> post;
};

// Highest drive in dB at which the lower factor of a quality still meets the plan's aliasing row (-90 dB Standard,
// -100 dB High, 5 and 7 kHz at -6 dBFS, measured with tests/ColourDynamicsTests.cpp "aliasing" over the whole drive
// range), with 3 dB of margin; the higher factor covers the rest. A negative limit sends the model to the higher factor
// always, a large one keeps it on the lower factor always.
constexpr float kLowFactorMaxDrive[2][kNumModels] = {
    // Soft Hard Rail Diode Fold Triode PushPull Pentode Tape Transformer RectHalf RectFull SineFold LinFold ZeroSq Asym
    {21, 15, 21, 18, 12, 24, 24, 12, -1, 99, 12, 10, 21, -1, 10, 20}, // Standard: 4x up to the limit, 8x above
    {30, 30, 30, 33, 24, 33, 36, 33, -1, 99, 24, 28, 21, 18, 25, 30}, // High: 8x up to the limit, 16x above
};

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_saturator_mix")), model(localIndex(d, "fx_saturator_model")),
          drive(localIndex(d, "fx_saturator_drive")), bias(localIndex(d, "fx_saturator_bias")),
          output(localIndex(d, "fx_saturator_output")), tilt(localIndex(d, "fx_saturator_tilt")),
          tiltFreq(localIndex(d, "fx_saturator_tilt_freq")), character(localIndex(d, "fx_saturator_character")),
          autoGain(localIndex(d, "fx_saturator_autogain")), filterMode(localIndex(d, "fx_saturator_filter_mode")),
          filterType(localIndex(d, "fx_saturator_filter_type")), filterCutoff(localIndex(d, "fx_saturator_filter_cutoff")),
          filterRes(localIndex(d, "fx_saturator_filter_res"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        sr = c.sampleRate;
        quality = c.quality;
        // Draft: one path. Standard and High: a lower and a higher oversampling factor (4x / 8x, 8x / 16x).
        const int first = quality == Quality::Draft ? 1 : (quality == Quality::Standard ? 2 : 3);
        // The rejection of the half-band filters stays 10 dB under the quality's aliasing row (Draft has none).
        const double stopBand = quality == Quality::Draft ? 60.0 : (quality == Quality::Standard ? 100.0 : 110.0);
        pathCount = quality == Quality::Draft ? 1 : 2;
        latency = 0;
        for (int i = 0; i < pathCount; ++i)
        {
            path[static_cast<size_t>(i)].prepare(sr, first + i, stopBand);
            latency = std::max(latency, path[static_cast<size_t>(i)].latencySamples());
        }
        for (int i = 0; i < pathCount; ++i)
            path[static_cast<size_t>(i)].alignTo(latency);
        for (auto& k : kIn)
            k.prepare(sr);
        for (auto& k : kOut)
            k.prepare(sr);
        filter.prepare(sr);
        dryFilter.prepare(sr);
        table = &autoGainTable();
        size_t size = 64;
        while (size < static_cast<size_t>(latency + kChunk + kWarmup + 8))
            size *= 2;
        for (auto& r : dryRing)
            r.assign(size, 0.0f);
        ringMask = size - 1;
        reset();
    }

    void reset() override
    {
        for (int i = 0; i < pathCount; ++i)
            path[static_cast<size_t>(i)].reset();
        warming = false;
        for (auto& r : dryRing)
            std::fill(r.begin(), r.end(), 0.0f);
        ringPos = 0;
        gridPos = 0;
        current = 0;
        primed = false;
        gainSeen = false;
        fade = 1.0f;
        appliedMix = 0.0f;
        gainPrev = 1.0f;
        outPrev = 1.0f;
        restartTrim();
        filter.reset(1000.0f, 0.0f);
        dryFilter.reset(1000.0f, 0.0f);
        filterShare = filterShareTarget = 0.0f;
        filterActive = 0;
    }

    // Switched off: forget the memory (hysteresis, filters) so that switching on never replays old audio.
    void deactivate() override
    {
        if (primed)
            reset();
    }

    // The work is cut at multiples of kChunk samples of the instance's own time, whatever the host block size: the
    // parameters are read, the path and the model change are decided and the gain targets are set on that grid, and every
    // ramp runs over the grid chunk, so the output does not depend on how the host splits its blocks.
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        for (int pos = 0; pos < n;)
        {
            const int m = std::min(kChunk - gridPos, n - pos);
            if (gridPos == 0)
                beginGrid(in);
            fragment(l + pos, r + pos, m);
            gridPos += m;
            pos += m;
            if (gridPos == kChunk)
            {
                endGrid();
                gridPos = 0;
            }
        }
    }

    int latencySamples() const override { return latency; }

private:
    static constexpr int kChunk = 128;     // the grid of parameter reads, ramps and path decisions
    static constexpr int kTrimBlock = 512; // the grid of the loudness trim
    static constexpr int kWarmup = 256;
    static constexpr double kTrimSeconds = 1.0, kTrustSeconds = 0.5;

    // Which path the model wants at this drive: the lower factor up to the limit, the higher above it; going back takes
    // 1.5 dB less so that a drive sitting at the limit does not toggle.
    int wantedPath(int modelIndex, float driveDb) const
    {
        if (pathCount < 2)
            return 0;
        const float limit = kLowFactorMaxDrive[quality == Quality::High ? 1 : 0][modelIndex];
        return driveDb > limit ? 1 : (driveDb < limit - 1.5f ? 0 : current);
    }

    // Start of a grid chunk: parameters, model change, path change, targets of the ramps.
    void beginGrid(const EffectInputs& in)
    {
        const int wanted = std::clamp(in.choice(model), 0, kNumModels - 1);
        const float driveDb = std::clamp(in.value(drive), 0.0f, kMaxDriveDb);
        if (!primed)
        {
            active = wanted;
            current = wantedPath(wanted, driveDb) == 1 ? 1 : 0;
            primed = true;
        }
        // Drive at its minimum is a bypass: the wet share rises over the first half decibel, so the output nulls exactly.
        mixTarget = in.value(mix) * std::min(1.0f, driveDb * 2.0f);
        // A model change fades the wet path out over 15 ms while the new model runs alongside on the same input, so that
        // its memory (hysteresis, flux, filters) is warm; at silence the two swap places and the new one fades in.
        warming = false;
        if (wanted != active)
        {
            if (fade <= 0.0f)
            {
                for (int i = 0; i < pathCount; ++i)
                    path[static_cast<size_t>(i)].swapModels();
                active = wanted;
                restartTrim();
            }
            else
                warming = true;
        }
        fadeTarget = wanted != active ? 0.0f : 1.0f;
        fadeStep = static_cast<float>(1.0 / (0.015 * sr));
        const double tiltDb = in.value(tilt);
        tone.hz = std::clamp(in.value(tiltFreq), 20.0f, 0.45f * static_cast<float>(sr));
        tone.k = std::pow(10.0, tiltDb / 20.0) - 1.0;
        tone.a = std::pow(10.0, -tiltDb / 40.0);
        tone.shelf = std::abs(tone.k) > 1.0e-9;
        const int order =
            1; // the hysteresis runs at 8x or 16x, where the explicit Euler step is as accurate as RK4 is at 4x
        settings = libraryFor(active, driveDb, in.value(bias) * 0.01f, in.value(character), order);
        settingsNext = libraryFor(wanted, driveDb, in.value(bias) * 0.01f, in.value(character), order);
        target = wantedPath(active, driveDb);
        automatic = in.on(autoGain);
        tableTarget = automatic
                          ? static_cast<float>(std::pow(10.0, static_cast<double>(table->gainDb(active, driveDb)) / 20.0))
                          : 1.0f;
        outTarget = std::pow(10.0f, in.value(output) / 20.0f);

        // The resonant filter: off keeps it parked on the target with clear memory; a new mode starts from clear memory.
        const int mode = std::clamp(in.choice(filterMode), 0, 2);
        const float fHz = in.value(filterCutoff), fRes = std::clamp(in.value(filterRes), 0.0f, 1.0f);
        if (mode == 0 || mode != filterActive)
        {
            filter.reset(fHz, fRes);
            dryFilter.reset(fHz, fRes);
        }
        else
        {
            filter.setTarget(fHz, fRes, kChunk);
            dryFilter.setTarget(fHz, fRes, kChunk);
        }
        filter.setType(in.choice(filterType));
        dryFilter.setType(in.choice(filterType));
        // Below the first half decibel of drive the shaper is a bypass and the wet share is zero: the same filter then
        // runs on the aligned dry path (the shaper is the identity there, so pre and post are the same signal), taking
        // over what the wet share gives up, so the filter is heard at 0 dB drive.
        filterShareTarget = mode == 0 ? 0.0f : in.value(mix) * (1.0f - std::min(1.0f, driveDb * 2.0f));
        filterActive = mode;

        if (!gainSeen)
        {
            gainPrev = tableTarget;
            appliedMix = mixTarget;
            filterShare = filterShareTarget;
            gainSeen = true;
        }

        // A move to the other path: it first runs over the last stretch of input, starting from the state of the current
        // one, so that its filters, shaper and loop are warm, and then the two are cross-faded over this grid chunk.
        crossing = target != current;
        adaa = usesAdaa(active);
        Path& cur = path[static_cast<size_t>(current)];
        if (crossing)
        {
            Path& from = cur;
            Path& to = path[static_cast<size_t>(target)];
            to.clearMemory();
            to.set(settings); // before adopt(): a changed model resets the shaper's state
            if (warming)
                to.setNext(settingsNext);
            to.adopt(from);
            float* warm[2] = {warmIn[0].data(), warmIn[1].data()};
            for (size_t c = 0; c < 2; ++c)
                for (size_t i = 0; i < static_cast<size_t>(kWarmup); ++i)
                    warm[c][i] = dryRing[c][(ringPos - static_cast<size_t>(kWarmup) + i) & ringMask];
            const float* warmConst[2] = {warm[0], warm[1]};
            to.run(warmConst, warm, kWarmup, usesAdaa(active), warming, tone);
        }
        cur.set(settings);
        if (warming)
            cur.setNext(settingsNext);
        if (crossing)
        {
            Path& to = path[static_cast<size_t>(target)];
            to.set(settings);
            if (warming)
                to.setNext(settingsNext);
        }
    }

    // m samples of the grid chunk, starting at gridPos.
    void fragment(float* l, float* r, int m)
    {
        float* io[2] = {l, r};
        // Dry path: the input delayed by the latency of the wet path, whole samples.
        const size_t lat = static_cast<size_t>(latency);
        for (size_t c = 0; c < 2; ++c)
        {
            auto& ring = dryRing[c];
            for (int i = 0; i < m; ++i)
            {
                ring[(ringPos + static_cast<size_t>(i)) & ringMask] = io[c][i];
                dryBlock[c][static_cast<size_t>(i)] = ring[(ringPos + static_cast<size_t>(i) - lat) & ringMask];
            }
        }
        ringPos += static_cast<size_t>(m);
        // A pre filter shapes what the shaper sees (the dry path stays unfiltered); the loudness measure follows it, so
        // the auto gain does not undo the filter. Post, the filter comes after the measure for the same reason.
        if (filterActive == 1)
            filter.run(l, r, m);
        if (automatic)
        {
            crucible::fx::KWeighting::energy(kIn[0], l, kIn[1], r, m, energyIn[0], energyIn[1]);
        }

        Path& cur = path[static_cast<size_t>(current)];
        const float inv = 1.0f / static_cast<float>(kChunk);
        if (crossing)
        {
            Path& to = path[static_cast<size_t>(target)];
            float* keep[2] = {pathIn[0].data(), pathIn[1].data()};
            const float* keepConst[2] = {keep[0], keep[1]};
            for (size_t c = 0; c < 2; ++c)
                std::copy(io[c], io[c] + m, keep[c]);
            float* alt[2] = {pathOut[0].data(), pathOut[1].data()};
            cur.run(keepConst, io, m, adaa, warming, tone);
            to.run(keepConst, alt, m, adaa, warming, tone);
            for (size_t c = 0; c < 2; ++c)
                for (int i = 0; i < m; ++i)
                {
                    const float w = static_cast<float>(gridPos + i + 1) * inv;
                    io[c][i] += (alt[c][i] - io[c][i]) * w;
                }
        }
        else
        {
            const float* inConst[2] = {l, r};
            cur.run(inConst, io, m, adaa, warming, tone);
        }

        // Loudness-matched gain: the table gain (instant, calibrated on pink noise at -18 dB RMS) times a slow trim that
        // measures what the wet path really did to the K-weighted level of the input.
        for (size_t c = 0; c < 2; ++c)
        {
            int i = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
            // MSVC left these per-sample ramps scalar; four samples at a time, the same float operations per lane.
            const __m128 g0 = _mm_set1_ps(gainPrev), gd = _mm_set1_ps(tableTarget - gainPrev), vinv = _mm_set1_ps(inv);
            for (; i + 4 <= m; i += 4)
            {
                const __m128 k = _mm_cvtepi32_ps(_mm_add_epi32(_mm_set1_epi32(gridPos + i + 1), _mm_set_epi32(3, 2, 1, 0)));
                const __m128 g = _mm_add_ps(g0, _mm_mul_ps(_mm_mul_ps(gd, k), vinv));
                _mm_storeu_ps(io[c] + i, _mm_mul_ps(_mm_loadu_ps(io[c] + i), g));
            }
#endif
            for (; i < m; ++i)
                io[c][i] *= gainPrev + (tableTarget - gainPrev) * static_cast<float>(gridPos + i + 1) * inv;
        }
        if (automatic)
        {
            crucible::fx::KWeighting::energy(kOut[0], l, kOut[1], r, m, energyOut[0], energyOut[1]);
        }

        if (filterActive == 2)
            filter.run(l, r, m);

        // At its target the fade stays where it is (the clamp of a zero difference is zero), so the chain of the fade
        // only runs while it moves.
        if (fade == fadeTarget && !std::signbit(fade) && fadeStep >= 0.0f)
            std::fill(fades.begin(), fades.begin() + m, fade);
        else
            for (int i = 0; i < m; ++i)
            {
                fade += std::clamp(fadeTarget - fade, -fadeStep, fadeStep);
                fades[static_cast<size_t>(i)] = fade;
            }
        if (filterActive != 0)
        {
            float* df[2] = {dryFiltered[0].data(), dryFiltered[1].data()};
            for (size_t c = 0; c < 2; ++c)
                std::copy(dryBlock[c].begin(), dryBlock[c].begin() + m, df[c]);
            dryFilter.run(df[0], df[1], m);
            for (size_t c = 0; c < 2; ++c)
                for (int i = 0; i < m; ++i)
                {
                    const float f = filterShare + (filterShareTarget - filterShare) * static_cast<float>(gridPos + i + 1) * inv;
                    float& d = dryBlock[c][static_cast<size_t>(i)];
                    d += (df[c][i] - d) * f;
                }
        }
        for (size_t c = 0; c < 2; ++c)
        {
            int i = 0;
#if CRUCIBLE_SIMD_SSE_EXACT
            const __m128 vinv = _mm_set1_ps(inv), t0 = _mm_set1_ps(trimPrev), td = _mm_set1_ps(trimTarget - trimPrev),
                         m0 = _mm_set1_ps(appliedMix), md = _mm_set1_ps(mixTarget - appliedMix), o0 = _mm_set1_ps(outPrev),
                         od = _mm_set1_ps(outTarget - outPrev);
            for (; i + 4 <= m; i += 4)
            {
                const __m128 f = _mm_mul_ps(
                    _mm_cvtepi32_ps(_mm_add_epi32(_mm_set1_epi32(gridPos + i + 1), _mm_set_epi32(3, 2, 1, 0))), vinv);
                const __m128 wet = _mm_mul_ps(_mm_loadu_ps(io[c] + i), _mm_add_ps(t0, _mm_mul_ps(td, f)));
                const __m128 mx = _mm_mul_ps(_mm_add_ps(m0, _mm_mul_ps(md, f)), _mm_loadu_ps(fades.data() + i));
                const __m128 dry = _mm_loadu_ps(dryBlock[c].data() + i);
                _mm_storeu_ps(io[c] + i, _mm_mul_ps(_mm_add_ps(dry, _mm_mul_ps(_mm_sub_ps(wet, dry), mx)),
                                                    _mm_add_ps(o0, _mm_mul_ps(od, f))));
            }
#endif
            for (; i < m; ++i)
            {
                const float f = static_cast<float>(gridPos + i + 1) * inv;
                const float wet = io[c][i] * (trimPrev + (trimTarget - trimPrev) * f);
                const float mx = (appliedMix + (mixTarget - appliedMix) * f) * fades[static_cast<size_t>(i)];
                const float dry = dryBlock[c][static_cast<size_t>(i)];
                io[c][i] = (dry + (wet - dry) * mx) * (outPrev + (outTarget - outPrev) * f);
            }
        }
    }

    // End of a grid chunk: the ramps arrive, the path change completes and the trim learns from this chunk's levels.
    void endGrid()
    {
        if (crossing)
            current = target;
        gainPrev = tableTarget;
        trimPrev = trimTarget;
        outPrev = outTarget;
        appliedMix = mixTarget;
        filterShare = filterShareTarget;
        if (automatic)
        {
            trimCount += kChunk;
            if (trimCount >= kTrimBlock)
            {
                updateTrim(trimCount);
                trimCount = 0;
                energyIn[0] = energyIn[1] = energyOut[0] = energyOut[1] = 0.0;
            }
        }
        else
        {
            trimTarget = 1.0f;
            restartTrim();
        }
    }

    // The trim learns the K-weighted levels over the last n samples.
    void updateTrim(int n)
    {
        const double ein = (energyIn[0] + energyIn[1]) / n, eout = (energyOut[0] + energyOut[1]) / n;
        if (ein > 1.0e-9 && eout > 1.0e-12)
        {
            const double a = 1.0 - std::exp(-static_cast<double>(n) / (kTrimSeconds * sr));
            if (!trimSeen)
            {
                einAvg = ein;
                eoutAvg = eout;
                trimSeen = true;
            }
            else
            {
                einAvg += (ein - einAvg) * a;
                eoutAvg += (eout - eoutAvg) * a;
            }
            trust = std::min(1.0, trust + static_cast<double>(n) / (kTrustSeconds * sr));
            const double ratio = std::clamp(std::sqrt(einAvg / eoutAvg), 0.25, 4.0);
            trimTarget = static_cast<float>(1.0 + (ratio - 1.0) * trust);
        }
    }

    void restartTrim()
    {
        trimSeen = false;
        trust = 0.0;
        trimPrev = trimTarget = 1.0f;
        einAvg = eoutAvg = 0.0;
        trimCount = 0;
        energyIn[0] = energyIn[1] = energyOut[0] = energyOut[1] = 0.0;
        for (auto& k : kIn)
            k.reset();
        for (auto& k : kOut)
            k.reset();
    }

    const int mix, model, drive, bias, output, tilt, tiltFreq, character, autoGain, filterMode, filterType, filterCutoff,
        filterRes;
    ResonantFilter filter;
    ResonantFilter dryFilter; // the same filter on the dry path while the shaper is a bypass
    float filterShare = 0.0f, filterShareTarget = 0.0f;
    std::array<std::array<float, kChunk>, 2> dryFiltered{};
    int filterActive = 0; // 0 off, 1 pre, 2 post
    std::array<Path, 2> path;
    int pathCount = 1, current = 0;
    std::array<std::array<float, kWarmup>, 2> warmIn{};
    std::array<std::array<float, kChunk>, 2> pathIn{}, pathOut{};
    std::array<crucible::fx::KWeighting, 2> kIn, kOut;
    std::array<std::vector<float>, 2> dryRing;
    std::array<std::array<float, kChunk>, 2> dryBlock{};
    std::array<float, kChunk> fades{};
    size_t ringMask = 63, ringPos = 0;
    const AutoGainTable* table = nullptr;
    double sr = 48000.0;
    double einAvg = 0.0, eoutAvg = 0.0, trust = 0.0;
    bool trimSeen = false;
    Quality quality = Quality::Standard;
    int active = 0, latency = 0;
    bool primed = false, gainSeen = false, warming = false;
    float fade = 1.0f, appliedMix = 0.0f, gainPrev = 1.0f, outPrev = 1.0f, trimPrev = 1.0f, trimTarget = 1.0f;
    // Held for the grid chunk in progress.
    int gridPos = 0, target = 0;
    bool crossing = false, automatic = false, adaa = false;
    float mixTarget = 0.0f, fadeTarget = 1.0f, fadeStep = 0.0f, tableTarget = 1.0f, outTarget = 1.0f;
    Path::Tone tone;
    saturation::Settings settings, settingsNext;
    double energyIn[2] = {0.0, 0.0}, energyOut[2] = {0.0, 0.0};
    int trimCount = 0;
};
} // namespace

ModuleDescriptor saturatorEffect()
{
    auto d = effectDescriptor(
        "fx.saturator", "Saturator", "Distortion", CpuClass::Heavy,
        "Mix-grade saturation with ten generic device models: soft clip, hard clip, op-amp rail clipper, "
        "diode, wavefolder, single-ended triode, push-pull triode, pentode, magnetic tape hysteresis and an "
        "iron-core transformer. The memoryless curves and the tubes use second-order antiderivative "
        "anti-aliasing inside a linear-phase half-band oversampler: 2x in Draft, 4x or 8x in Standard and "
        "8x or 16x in High; the higher factor takes over as the drive rises past the point where a model "
        "would alias above -90 dB (Standard) or -100 dB (High), and the tape model always uses it (the move "
        "is warmed up and cross-faded). Drive, bias (asymmetry), "
        "a tilt (a high-shelf emphasis before the shaper and its exact inverse after it, so distortion "
        "can be pushed toward the highs or the lows), a character control (tube sag, tape loop width or "
        "transformer flux corner), loudness-matched auto gain calibrated on pink noise, output and a mix "
        "against a dry path aligned to the wet latency. An optional resonant filter (low-pass, band-pass or "
        "high-pass, 20 Hz to 20 kHz) runs before the shaper, ahead of the oversampler, or after it on the wet path. "
        "Drive at 0 dB bypasses the shaper exactly. The "
        "latency is the oversampling filters, reported.");
    auto& p = d.params;
    addOn(p, "saturator", "Saturator", "fx_saturator", 1.0f);
    std::vector<std::string> names;
    for (const auto& m : kModels)
        names.emplace_back(m.name);
    p.push_back(params::choice("fx_saturator_model", "Saturator Model", "fx_saturator", names, 0));
    addF(p, "saturator_drive", "Saturator Drive", "fx_saturator", 0.0f, kMaxDriveDb, 12.0f, "dB");
    addF(p, "saturator_bias", "Saturator Bias", "fx_saturator", -100.0f, 100.0f, 0.0f, "%");
    addF(p, "saturator_output", "Saturator Output", "fx_saturator", -24.0f, 12.0f, 0.0f, "dB");
    addF(p, "saturator_tilt", "Saturator Tilt", "fx_saturator_2", -12.0f, 12.0f, 0.0f, "dB");
    addHz(p, "saturator_tilt_freq", "Saturator Tilt Frequency", "fx_saturator_2", 100.0f, 8000.0f, 1000.0f, 0.4f);
    addF(p, "saturator_character", "Saturator Character", "fx_saturator_2", 0.0f, 100.0f, 50.0f, "%");
    p.push_back(params::toggle("fx_saturator_autogain", "Saturator Auto Gain", "fx_saturator_2", true));
    p.push_back(params::integer("fx_pos_saturator", "Saturator Slot", "fx_saturator_2", 1, kNumModuleFx, kNumModuleFx));
    // Resonant filter before or after the shaper (version 2): flagged appended so the frozen parameter order does not move.
    const size_t firstNew = p.size();
    p.push_back(params::choice("fx_saturator_filter_mode", "Saturator Filter", "fx_saturator_filter",
                               {"Off", "Pre", "Post"}, 0));
    p.push_back(params::choice("fx_saturator_filter_type", "Saturator Filter Type", "fx_saturator_filter",
                               {"Low-pass", "Band-pass", "High-pass"}, 0));
    addHz(p, "saturator_filter_cutoff", "Saturator Filter Cutoff", "fx_saturator_filter", 20.0f, 20000.0f, 1000.0f, 0.3f);
    addF(p, "saturator_filter_res", "Saturator Filter Resonance", "fx_saturator_filter", 0.0f, 1.0f, 0.0f);
    for (size_t i = firstNew; i < p.size(); ++i)
    {
        p[i].since = 2;
        p[i].appended = 2;
    }
    shortNames(p);
    d.version = 2;
    d.onParam = "fx_saturator_on";
    d.positionParam = "fx_pos_saturator";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_saturator", "Saturator", "fx", "Saturator", "Model, drive, bias, output and mix."});
    d.pages.push_back({"fx_saturator_2", "Saturator 2", "fx", "Saturator Tone",
                       "Tilt and its frequency, character (tube sag, tape loop width, transformer flux corner), auto gain "
                       "and the chain slot."});
    d.pages.push_back({"fx_saturator_filter", "Saturator Filter", "fx", "Saturator Filter",
                       "Resonant filter (low-pass, band-pass, high-pass) before the shaper to shape what it distorts, or "
                       "after it to tame the result. Off by default."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.saturator")); };
    return d;
}

} // namespace crucible::modules::fx
