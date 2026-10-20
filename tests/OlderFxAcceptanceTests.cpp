// SPDX-License-Identifier: MIT
// Common acceptance (docs/plans/ENGINE-FX-PLAN.md section 3, "Common acceptance for every new module" and section 4.7)
// for the effects that predate it: fx.comp_clean, fx.comp_vca, fx.gate2, fx.eq_pro, fx.reverb_algo, fx.convolution,
// fx.maximizer, fx.clipper and util.meter. Every check goes through the module rack, like the newer modules' tests.
//
//   rates      the main accept rows (silence, unity gain, one module specific row, 8 s of random parameter extremes)
//              at 44.1 / 48 / 96 / 192 kHz
//   latency    measured delay (cross-spectrum phase slope) equals the reported latency to one sample at every rate
//   zipper     every continuous parameter, 0-to-1 ramp over 50 ms and 2 Hz square, at host blocks of 1, 7 and 512,
//              below -90 dB (or, for a nonlinear module whose held setting already leaves a residue, the held floor + 3 dB)
//   reset      reset() and an off / on cycle leave no stale state (silence in gives silence out below -150 dBFS)
//
// Known failures. The limits are the plan's and are never loosened. A check that a module does not meet yet is listed in
// kKnownFail (by key prefix). Two test classes run the same code: "OlderFxPass" skips the listed checks, "OlderFxKnownFail"
// runs only the listed checks and is registered in CMake with WILL_FAIL, so it turns red the moment a listed check starts
// passing (delete the entry then). "OlderFxAll" runs everything strictly; use it to regenerate the list. The findings
// (measured against bound, suspected source) were recorded when the list was made.

#include "FxMeasure.h"
#include "TestSupport.h"

#include <crucible/Engine.h>
#include <crucible/Fx.h>
#include <crucible/fx/Dynamics.h>
#include <crucible/fx/Loudness.h>
#include <crucible/module/ConvolutionIr.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/MeterReadout.h>
#include <crucible/module/ModuleRack.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::measure;
using crucible::test::String;
using crucible::test::UnitTest;

namespace
{
// ---------------------------------------------------------------------------------------------------------------------
// Known failures (key prefixes). Keys read "<module>|<check>|<detail...>".
// The list lives in tests/OlderFxKnownFail.txt (one key prefix per line, '#' comments).
const std::vector<std::string>& knownFail()
{
    static const std::vector<std::string> list = []
    {
        std::vector<std::string> v;
        std::FILE* f = std::fopen(CRUCIBLE_OLDERFX_KNOWN_FAIL, "r");
        if (f == nullptr)
            return v;
        char line[512];
        while (std::fgets(line, sizeof(line), f) != nullptr)
        {
            std::string s(line);
            while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' '))
                s.pop_back();
            if (!s.empty() && s[0] != '#')
                v.push_back(s);
        }
        std::fclose(f);
        return v;
    }();
    return list;
}

enum class Mode
{
    All,   // every check, strict
    Pass,  // every check that is not a known failure
    Known, // only the known failures
};

double gRate = 48000.0;
constexpr int kMaxBlock = 512;      // ModuleContext::maxBlockSize default: the largest host block
constexpr float kSilence = 3.2e-8f; // -150 dBFS

std::string rateTag()
{
    char tag[32];
    std::snprintf(tag, sizeof(tag), "%g", gRate);
    return tag;
}

std::string key(std::initializer_list<std::string> parts)
{
    std::string k;
    for (const auto& p : parts)
    {
        if (!k.empty())
            k += "|";
        k += p;
    }
    return k;
}

bool isKnown(const std::string& k)
{
    for (const auto& p : knownFail())
        if (k.rfind(p, 0) == 0)
            return true;
    return false;
}

size_t secs(double s)
{
    return static_cast<size_t>(std::llround(s * gRate));
}

using Samples = std::vector<float>;

// ---------------------------------------------------------------------------------------------------------------------
// One module through the rack, every other effect off.
struct Rack;
struct Mod
{
    const char* id;
    const char* on;
    Quality quality;
    void (*neutral)(Rack&); // transparent settings (unity small-signal gain), or null
    void (*active)(Rack&);  // settings that make the module do something
};

struct Rack
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values, legacy;
    ConvolutionIr* conv = nullptr;
    double rate;
    explicit Rack(const Mod& m, double r, Quality q = Quality::Standard) : rate(r)
    {
        ModuleContext ctx;
        ctx.sampleRate = r;
        ctx.maxBlockSize = kMaxBlock;
        ctx.quality = q;
        rack.prepare(ctx);
        values = ParameterMap::defaultValues();
        legacy.assign(static_cast<size_t>(kNumModDests), 0.0f);
        for (int i = 0; i < rack.size(); ++i)
            set(rack.descriptor(i).onParam, 0.0f);
        set(m.on, 1.0f);
        conv = rack.findCapability<ConvolutionIr>("fx.convolution");
    }
    void set(const std::string& id, float v)
    {
        const int i = ParameterMap::indexOf(id);
        if (i < 0)
        {
            std::fprintf(stderr, "unknown parameter %s\n", id.c_str());
            std::abort();
        }
        values[static_cast<size_t>(i)] = v;
    }
    float get(const std::string& id) const { return values[static_cast<size_t>(ParameterMap::indexOf(id))]; }
    void run(float* l, float* r, int n, const float* aL = nullptr, const float* aR = nullptr)
    {
        rack.process(l, r, n, values.data(), legacy.data(), nullptr, 120.0, aL, aR);
    }
    // Left output of a mono input fed to both channels, in host blocks of `block`.
    Samples render(const Samples& in, size_t block = 128, const Samples* aux = nullptr)
    {
        Samples l = in, r = in;
        for (size_t pos = 0; pos < l.size(); pos += block)
        {
            const int n = static_cast<int>(std::min(block, l.size() - pos));
            run(l.data() + pos, r.data() + pos, n, aux != nullptr ? aux->data() + pos : nullptr,
                aux != nullptr ? aux->data() + pos : nullptr);
        }
        return l;
    }
    void silence(size_t n, size_t block = 128) { render(Samples(n, 0.0f), block); }
    void setIr(std::vector<Samples> channels, IrLayout layout)
    {
        ImpulseResponse ir;
        ir.sampleRate = rate;
        ir.layout = layout;
        ir.channels = std::move(channels);
        conv->setImpulseResponse(ir, "test");
        silence(static_cast<size_t>(0.35 * rate)); // the kernel is built off the audio thread, then crossfaded in
    }
};

Samples toF(const Signal& s)
{
    return Samples(s.begin(), s.end());
}

Samples sineF(size_t n, double hz, double amp, double rate)
{
    Samples s(n);
    for (size_t i = 0; i < n; ++i)
        s[i] = static_cast<float>(amp * std::sin(2.0 * kPi * hz * static_cast<double>(i) / rate));
    return s;
}

Samples noiseF(size_t n, double rms, std::uint64_t seed)
{
    Random rng(seed);
    Samples s(n);
    for (auto& v : s)
        v = static_cast<float>(rms * rng.gaussian());
    return s;
}

float peakOf(const Samples& s, size_t from = 0)
{
    float p = 0.0f;
    for (size_t i = from; i < s.size(); ++i)
        p = std::max(p, std::abs(s[i]));
    return p;
}

// Amplitude (dB re 1) of the hz component over the last `seconds` of y.
double toneDb(const Samples& y, double hz, double seconds)
{
    const size_t n = std::min(y.size(), secs(seconds));
    const Signal d = toDouble(y, y.size() - n);
    return amplitudeToDb(toneAmplitude(d.data(), d.size(), hz, gRate));
}

Samples decayingNoise(size_t length, std::uint64_t seed, double gain)
{
    Random rng(seed);
    Samples k(length);
    for (size_t i = 0; i < length; ++i)
        k[i] = static_cast<float>(gain * rng.gaussian() *
                                  std::exp(-5.0 * static_cast<double>(i) / static_cast<double>(length)));
    return k;
}

// ---------------------------------------------------------------------------------------------------------------------
// Module settings
void bandOn(Rack& h, int b, int type, double hz, double gain, double q)
{
    const std::string n = "fx_ep_b" + std::to_string(b) + "_";
    h.set(n + "on", 1.0f);
    h.set(n + "type", static_cast<float>(type));
    h.set(n + "freq", static_cast<float>(hz));
    h.set(n + "gain", static_cast<float>(gain));
    h.set(n + "q", static_cast<float>(q));
}

void ccNeutral(Rack& h)
{
    h.set("fx_cc_thresh", 0.0f);
    h.set("fx_cc_ratio", 1.0f);
    h.set("fx_cc_knee", 0.0f);
    h.set("fx_cc_detector", 0.0f);
}
void ccActive(Rack& h)
{
    h.set("fx_cc_thresh", -30.0f);
    h.set("fx_cc_ratio", 4.0f);
    h.set("fx_cc_makeup", 6.0f);
}
void cvNeutral(Rack& h)
{
    h.set("fx_cv_thresh", 10.0f);
    h.set("fx_cv_knee", 0.0f);
}
void cvActive(Rack& h)
{
    h.set("fx_cv_thresh", -30.0f);
    h.set("fx_cv_ratio", 2.0f);
    h.set("fx_cv_makeup", 6.0f);
}
void g2Neutral(Rack& h)
{
    h.set("fx_g2_mode", 0.0f);
    h.set("fx_g2_thresh", -90.0f);
    h.set("fx_g2_ratio", 1.0f);
    h.set("fx_g2_range", 0.0f);
}
void g2Active(Rack& h)
{
    h.set("fx_g2_mode", 1.0f);
    h.set("fx_g2_thresh", -30.0f);
    h.set("fx_g2_ratio", 3.0f);
}
void epNeutral(Rack& h)
{
    bandOn(h, 1, 0, 1000.0, 0.0, 1.0);
}
void epActive(Rack& h)
{
    bandOn(h, 1, 0, 1000.0, 9.0, 2.0);
    bandOn(h, 2, 2, 8000.0, -6.0, 0.7071);
    bandOn(h, 3, 0, 300.0, -6.0, 1.0);
    h.set("fx_ep_b3_dyn", 1.0f);
}
void raActive(Rack& h)
{
    h.set("fx_ra_mix", 0.6f);
}
void cxNeutral(Rack& h)
{
    Samples delta(64, 0.0f);
    delta[0] = 1.0f;
    h.setIr({delta}, IrLayout::Mono);
}
void cxActive(Rack& h)
{
    const size_t len = secs(0.6);
    // Normalised so that the sum of the magnitudes is 1 (a bound for the gain on any input).
    auto unit = [](Samples k)
    {
        double s = 0.0;
        for (const float v : k)
            s += std::abs(v);
        for (auto& v : k)
            v = static_cast<float>(v / s);
        return k;
    };
    h.setIr({unit(decayingNoise(len, 1, 0.3)), unit(decayingNoise(len, 2, 0.3))}, IrLayout::Stereo);
    h.set("fx_cx_mix", 0.8f);
}
void mxNeutral(Rack& h)
{
    h.set("fx_mx_gain", 0.0f);
    h.set("fx_mx_ceiling", 0.0f);
}
void mxActive(Rack& h)
{
    h.set("fx_mx_gain", 12.0f);
    h.set("fx_mx_ceiling", -1.0f);
}
void clpNeutral(Rack& h)
{
    h.set("fx_clp_drive", 0.0f);
    h.set("fx_clp_ceiling", 0.0f);
}
void clpActive(Rack& h)
{
    h.set("fx_clp_drive", 12.0f);
}

const Mod kMods[] = {
    {"fx.comp_clean", "fx_cc_on", Quality::Standard, ccNeutral, ccActive},
    {"fx.comp_vca", "fx_cv_on", Quality::Standard, cvNeutral, cvActive},
    {"fx.gate2", "fx_g2_on", Quality::Standard, g2Neutral, g2Active},
    {"fx.eq_pro", "fx_ep_on", Quality::Standard, epNeutral, epActive},
    {"fx.reverb_algo", "fx_ra_on", Quality::Standard, nullptr, raActive},
    {"fx.convolution", "fx_cx_on", Quality::High, cxNeutral, cxActive},
    {"fx.maximizer", "fx_mx_on", Quality::Standard, mxNeutral, mxActive},
    {"fx.clipper", "fx_clp_on", Quality::Standard, clpNeutral, clpActive},
    {"util.meter", "fx_mtr_on", Quality::Standard, [](Rack&) {}, [](Rack&) {}},
};

const Mod& modOf(const char* id)
{
    for (const Mod& m : kMods)
        if (std::strcmp(m.id, id) == 0)
            return m;
    std::abort();
}

// ---------------------------------------------------------------------------------------------------------------------
// Inter-sample-peak torture signals (a subset of tests/MaximizerTests.cpp, at the rack rate).
struct Item
{
    std::string name;
    Signal x;
};

std::vector<Item> tortureSet()
{
    std::vector<Item> items;
    const size_t n = secs(0.5);
    for (const double deg : {0.0, 22.5, 45.0})
        items.push_back({"fs/4 sine " + std::to_string(static_cast<int>(deg)) + " deg",
                         sineWave(n, gRate / 4.0, gRate, 1.0, deg * kPi / 180.0)});
    {
        Signal x(n);
        for (size_t i = 0; i < n; ++i)
            x[i] = (i / 2) % 2 == 0 ? 1.0 : -1.0;
        items.push_back({"++-- square", x});
    }
    {
        Signal x(n);
        double phase = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const double hz = 100.0 * std::pow(190.0, static_cast<double>(i) / static_cast<double>(n));
            phase += 2.0 * kPi * hz / gRate;
            const double fade = std::min({1.0, static_cast<double>(i) / 480.0, static_cast<double>(n - i) / 480.0});
            x[i] = 1.6 * std::sin(phase) * fade;
        }
        items.push_back({"sweep 100 Hz to 19 kHz at +4 dBFS", x});
    }
    return items;
}

// ---------------------------------------------------------------------------------------------------------------------
class OlderFxBase : public UnitTest
{
public:
    OlderFxBase(const char* name, Mode m) : UnitTest(name, "Effects"), mode(m) {}

    void runTest() override
    {
        // Rates: the 48 kHz run first, then the others.
        for (const double rate : {48000.0, 44100.0, 96000.0, 192000.0})
        {
            gRate = rate;
            rateRows();
            latencyRows();
        }
        gRate = 48000.0;
        zipperRows();
        resetBypassRows(48000.0);
        resetBypassRows(192000.0);
        gRate = 48000.0;
    }

private:
    Mode mode;

    // CRUCIBLE_OLDERFX_ONLY="fx.comp_clean,fx.gate2" restricts a run to keys that start with one of the prefixes (sanitizer runs).
    static bool onlyFilter(const std::string& k)
    {
        const char* env = std::getenv("CRUCIBLE_OLDERFX_ONLY");
        if (env == nullptr || *env == '\0')
            return true;
        const std::string list = env;
        size_t pos = 0;
        while (pos <= list.size())
        {
            const size_t end = std::min(list.find(',', pos), list.size());
            if (end > pos && k.compare(0, end - pos, list, pos, end - pos) == 0)
                return true;
            pos = end + 1;
        }
        return false;
    }
    bool want(const std::string& k) const
    {
        if (!onlyFilter(k))
            return false;
        switch (mode)
        {
        case Mode::Pass:
            return !isKnown(k);
        case Mode::Known:
            return isKnown(k);
        default:
            return true;
        }
    }

    // One verdict. The section name carries the module; the key goes first in the message so that the findings can be
    // cut from the log.
    void verdict(const std::string& k, bool ok, const String& detail) { expect(ok, String(k + ": ") + detail); }

    // ---- rate rows ------------------------------------------------------------------------------------------------
    void rateRows()
    {
        for (const Mod& m : kMods)
        {
            beginTest(std::string(m.id) + " [" + rateTag() + " Hz]: silence, unity gain, module rows, random extremes");
            rowSilence(m);
            rowUnity(m);
            rowExtremes(m);
        }
        beginTest(std::string("fx.comp_clean [") + rateTag() + " Hz]: static curve");
        compCleanCurve();
        beginTest(std::string("fx.comp_vca [") + rateTag() + " Hz]: static curve");
        compVcaCurve();
        beginTest(std::string("fx.gate2 [") + rateTag() + " Hz]: onset residual and ducker depth");
        gate2Rows();
        beginTest(std::string("fx.eq_pro [") + rateTag() + " Hz]: bell gain at the centre, minimum and linear phase");
        eqRows();
        beginTest(std::string("fx.reverb_algo [") + rateTag() + " Hz]: RT60 of the five algorithms");
        reverbRows();
        beginTest(std::string("fx.convolution [") + rateTag() + " Hz]: equals direct convolution");
        convolutionRows();
        beginTest(std::string("fx.maximizer [") + rateTag() + " Hz]: true peak on the torture set");
        maximizerRows();
        beginTest(std::string("fx.clipper [") + rateTag() + " Hz]: alias power and ceiling");
        clipperRows();
        beginTest(std::string("util.meter [") + rateTag() + " Hz]: pass-through, loudness, true peak");
        meterRows();
    }

    void rowSilence(const Mod& m)
    {
        const std::string k = key({m.id, "silence", rateTag()});
        if (!want(k))
            return;
        Rack h(m, gRate, m.quality);
        m.active(h);
        const Samples y = h.render(Samples(secs(2.0), 0.0f));
        const float p = peakOf(y);
        verdict(k, p < kSilence, "peak " + String(amplitudeToDb(p), 1) + " dBFS (bound -150)");
    }

    void rowUnity(const Mod& m)
    {
        if (m.neutral == nullptr)
            return;
        const std::string k = key({m.id, "unity", rateTag()});
        if (!want(k))
            return;
        Rack h(m, gRate, m.quality);
        m.neutral(h);
        const double amp = 0.01; // -40 dBFS
        const Samples y = h.render(sineF(secs(1.5), 1000.0, amp, gRate));
        const double gain = toneDb(y, 1000.0, 0.5) - amplitudeToDb(amp);
        verdict(k, std::abs(gain) <= 0.2, "gain " + String(gain, 3) + " dB (bound +-0.2)");
    }

    // Random parameter extremes: every 100 ms each parameter takes its minimum, maximum, a random value or keeps its value.
    // The plan asks for 60 s; 8 s per rate keeps the suite short (the 60 s runs are in the per-module test files).
    void rowExtremes(const Mod& m)
    {
        const std::string k = key({m.id, "extremes", rateTag()});
        if (!want(k))
            return;
        const ModuleDescriptor* d = ModuleRegistry::instance().find(m.id);
        Rack h(m, gRate, m.quality);
        m.active(h);
        Random rng(static_cast<std::uint64_t>(gRate) + 17);
        const Samples noise = noiseF(secs(8.0), 0.3, 5);
        Samples l = noise, r = noise;
        bool finite = true;
        float peak = 0.0f;
        const size_t step = secs(0.1);
        for (size_t pos = 0; pos < l.size(); pos += step)
        {
            for (const ParamDef& p : d->params)
            {
                if (p.id == m.on || p.id.rfind("fx_pos_", 0) == 0)
                    continue;
                const double u = rng.uniform();
                if (u < 0.35)
                    continue;
                float v = p.min;
                if (u < 0.55)
                    v = p.min;
                else if (u < 0.75)
                    v = p.max;
                else
                    v = static_cast<float>(p.min + (p.max - p.min) * rng.uniform());
                if (p.type != ParamType::Float)
                    v = std::round(v);
                h.set(p.id, v);
            }
            for (size_t b = pos; b < std::min(pos + step, l.size()); b += 128)
            {
                const int n = static_cast<int>(std::min<size_t>(128, std::min(pos + step, l.size()) - b));
                h.run(l.data() + b, r.data() + b, n);
            }
        }
        for (size_t i = 0; i < l.size(); ++i)
        {
            if (!std::isfinite(l[i]) || !std::isfinite(r[i]))
                finite = false;
            peak = std::max({peak, std::abs(l[i]), std::abs(r[i])});
        }
        // Bound: +60 dBFS. fx.comp_clean may add upward compression range 60 dB + makeup 18 dB + automatic makeup: +120
        // dBFS.
        const float bound = std::strcmp(m.id, "fx.comp_clean") == 0 ? 1.0e6f : 1000.0f;
        verdict(k, finite && peak < bound,
                String(finite ? "finite" : "NOT FINITE") + ", peak " + String(amplitudeToDb(peak), 1) + " dBFS (bound " +
                    String(amplitudeToDb(bound), 0) + ")");
    }

    // ---- fx.comp_clean --------------------------------------------------------------------------------------------
    void compCleanCurve()
    {
        const std::string k = key({"fx.comp_clean", "static_curve", rateTag()});
        if (!want(k))
            return;
        double worst = 0.0;
        for (double level = -48.0; level <= 0.0; level += 6.0)
        {
            Rack h(modOf("fx.comp_clean"), gRate);
            h.set("fx_cc_thresh", -24.0f);
            h.set("fx_cc_ratio", 3.0f);
            h.set("fx_cc_knee", 8.0f);
            h.set("fx_cc_detector", 1.0f);
            h.set("fx_cc_rms", 10.0f);
            h.set("fx_cc_attack", 5.0f);
            h.set("fx_cc_release", 50.0f);
            const Samples y = h.render(sineF(secs(1.0), 1000.0, dbToAmplitude(level), gRate));
            const double gain = toneDb(y, 1000.0, 0.2) - level;
            const double ideal = -crucible::fx::dyn::compressorReduction(static_cast<float>(level), -24.0f, 3.0f, 8.0f);
            worst = std::max(worst, std::abs(gain - ideal));
        }
        verdict(k, worst <= 0.1, "worst static-curve error " + String(worst, 3) + " dB (bound 0.1)");
    }

    // ---- fx.comp_vca ----------------------------------------------------------------------------------------------
    void compVcaCurve()
    {
        const std::string k = key({"fx.comp_vca", "static_curve", rateTag()});
        if (!want(k))
            return;
        double worst = 0.0;
        const float ratios[] = {1.5f, 2.0f, 4.0f, 10.0f};
        for (int ratioIx : {1, 2, 3})
            for (double level : {-30.0, -14.0, -6.0, 0.0})
            {
                Rack h(modOf("fx.comp_vca"), gRate);
                h.set("fx_cv_thresh", -20.0f);
                h.set("fx_cv_knee", 0.0f);
                h.set("fx_cv_ratio", static_cast<float>(ratioIx));
                h.set("fx_cv_attack", 3.0f);
                h.set("fx_cv_release", 1.0f);
                const Samples y = h.render(sineF(secs(2.0), 1000.0, dbToAmplitude(level), gRate));
                const double gain = toneDb(y, 1000.0, 0.2) - level;
                const double ideal = level > -20.0 ? -(level + 20.0) * (1.0 - 1.0 / ratios[ratioIx]) : 0.0;
                worst = std::max(worst, std::abs(gain - ideal));
            }
        verdict(k, worst <= 0.5, "worst static-curve error " + String(worst, 3) + " dB (bound 0.5, ratios 2, 4, 10:1)");
    }

    // ---- fx.gate2 -------------------------------------------------------------------------------------------------
    void gate2Rows()
    {
        {
            const std::string k = key({"fx.gate2", "onset_residual", rateTag()});
            if (want(k))
            {
                Rack h(modOf("fx.gate2"), gRate);
                h.set("fx_g2_mode", 0.0f);
                h.set("fx_g2_thresh", -40.0f);
                h.set("fx_g2_hyst", 6.0f);
                h.set("fx_g2_ratio", 100.0f);
                h.set("fx_g2_range", -80.0f);
                h.set("fx_g2_attack", 0.1f);
                h.set("fx_g2_release", 20.0f);
                h.set("fx_g2_hold", 50.0f);
                h.set("fx_g2_detector", 0.0f);
                h.set("fx_g2_lookahead", 2.0f);
                Signal in;
                in.insert(in.end(), secs(0.2), 0.0);
                const Signal burst = sineWave(secs(0.2), 1000.0, gRate, dbToAmplitude(-10.0));
                in.insert(in.end(), burst.begin(), burst.end());
                in.insert(in.end(), secs(0.4), 0.0);
                const Samples y = h.render(toF(in));
                const int latency = h.rack.findCapability<LatencyReporter>("fx.gate2")->latencySamples();
                const double db = nullTest(in, toDouble(y), latency, 0);
                verdict(k, db <= -60.0, "onset residual " + String(db, 1) + " dB (bound -60), latency " + String(latency));
            }
        }
        {
            const std::string k = key({"fx.gate2", "ducker_depth", rateTag()});
            if (want(k))
            {
                double worstDucked = 0.0, free = 0.0;
                const double range = 20.0;
                for (const bool keyed : {true, false})
                {
                    Rack h(modOf("fx.gate2"), gRate);
                    h.set("fx_g2_mode", 2.0f);
                    h.set("fx_g2_thresh", -30.0f);
                    h.set("fx_g2_hyst", 3.0f);
                    h.set("fx_g2_ratio", 100.0f);
                    h.set("fx_g2_range", static_cast<float>(-range));
                    h.set("fx_g2_attack", 1.0f);
                    h.set("fx_g2_release", 50.0f);
                    h.set("fx_g2_detector", 1.0f);
                    h.set("fx_g2_key", 1.0f);
                    const size_t n = secs(1.0);
                    const Samples x = sineF(n, 440.0, dbToAmplitude(-20.0), gRate);
                    const Samples keyTone = keyed ? sineF(n, 100.0, dbToAmplitude(-10.0), gRate) : Samples(n, 0.0f);
                    const Samples y = h.render(x, 128, &keyTone);
                    const double gain = toneDb(y, 440.0, 0.2) - (-20.0);
                    (keyed ? worstDucked : free) = gain;
                }
                verdict(k, std::abs(worstDucked + range) <= 0.2 && std::abs(free) <= 0.01,
                        "ducked gain " + String(worstDucked, 3) + " dB (bound -20 +-0.2), free " + String(free, 4) +
                            " dB (bound +-0.01)");
            }
        }
    }

    // ---- fx.eq_pro ------------------------------------------------------------------------------------------------
    void eqRows()
    {
        struct Bell
        {
            double hz, gain, q;
        };
        const double top = std::min(18000.0, 0.4 * gRate);
        const Bell bells[] = {{20.0, 12.0, 1.0}, {100.0, 9.0, 2.0}, {1000.0, 9.0, 2.0}, {top, 9.0, 2.0}};
        for (const int phase : {0, 2})
            for (const Bell& b : bells)
            {
                if (phase == 2 && b.hz < 500.0)
                    continue; // a 4k-tap kernel has bins of 12 Hz at 48 kHz
                const std::string k = key({"fx.eq_pro", phase == 0 ? "bell_min" : "bell_linear", rateTag(),
                                           std::to_string(static_cast<int>(b.hz)) + "Hz"});
                if (!want(k))
                    continue;
                Rack h(modOf("fx.eq_pro"), gRate, phase == 2 ? Quality::High : Quality::Standard);
                bandOn(h, 1, 0, b.hz, b.gain, b.q);
                h.set("fx_ep_phase", static_cast<float>(phase));
                h.set("fx_ep_res", 2.0f);
                const double amp = 0.05;
                const Samples y = h.render(sineF(secs(b.hz < 50.0 ? 3.0 : 2.0), b.hz, amp, gRate), 128);
                const double gain = toneDb(y, b.hz, b.hz < 50.0 ? 1.0 : 0.5) - amplitudeToDb(amp);
                const double bound = phase == 0 ? 0.1 : 0.05;
                verdict(k, std::abs(gain - b.gain) <= bound,
                        "gain at the centre " + String(gain, 3) + " dB, set " + String(b.gain, 1) + " (bound +-" +
                            String(bound, 2) + ")");
            }
    }

    // ---- fx.reverb_algo -------------------------------------------------------------------------------------------
    void reverbRows()
    {
        const char* const names[] = {"Room", "Hall", "Chamber", "Plate", "Ambience"};
        for (int a = 0; a < 5; ++a)
        {
            const std::string k = key({"fx.reverb_algo", "rt60", rateTag(), names[a]});
            if (!want(k))
                continue;
            const double decay = a == 4 ? 0.8 : 1.0;
            Rack h(modOf("fx.reverb_algo"), gRate);
            h.set("fx_ra_algo", static_cast<float>(a));
            h.set("fx_ra_mix", 1.0f);
            h.set("fx_ra_decay", static_cast<float>(decay));
            h.set("fx_ra_predelay", 0.0f);
            h.set("fx_ra_lomult", 1.0f);
            h.set("fx_ra_himult", 1.0f);
            h.set("fx_ra_lowcut", 20.0f);
            h.set("fx_ra_highcut", 20000.0f);
            Samples in(secs(4.5 * decay + 1.0), 0.0f);
            in[0] = 1.0f;
            const Samples ir = h.render(in, 512);
            const auto bands = energyDecay(toDouble(ir), gRate, {250, 1000, 4000});
            double worst = 0.0;
            String line;
            for (const auto& b : bands)
            {
                worst = std::max(worst, std::abs(b.t30S / decay - 1.0));
                line = line + " " + String(b.t30S, 3);
            }
            verdict(k, worst < 0.10,
                    "set " + String(decay, 1) + " s, T30 at 250 / 1000 / 4000 Hz:" + line + " s, worst error " +
                        String(100.0 * worst, 1) + " % (bound 10)");
        }
    }

    // ---- fx.convolution -------------------------------------------------------------------------------------------
    void convolutionRows()
    {
        for (const int mode : {0, 1})
        {
            const std::string k = key({"fx.convolution", "direct", rateTag(), mode == 0 ? "zero_latency" : "low_cpu"});
            if (!want(k))
                continue;
            Rack h(modOf("fx.convolution"), gRate, Quality::High);
            h.set("fx_cx_mode", static_cast<float>(mode));
            h.set("fx_cx_mix", 1.0f);
            const Samples kern = decayingNoise(secs(0.12), 11, 0.2);
            h.setIr({kern}, IrLayout::Mono);
            const Samples in = noiseF(secs(0.25), 0.4, 5);
            const Samples out = h.render(in, 200);
            const int latency = h.rack.findCapability<LatencyReporter>("fx.convolution")->latencySamples();
            double err = 0.0, sig = 0.0;
            for (size_t i = static_cast<size_t>(latency); i < in.size(); ++i)
            {
                const size_t n = i - static_cast<size_t>(latency);
                double ref = 0.0;
                for (size_t j = 0; j <= std::min(n, kern.size() - 1); ++j)
                    ref += static_cast<double>(kern[j]) * in[n - j];
                const double d = out[i] - ref;
                err += d * d;
                sig += ref * ref;
            }
            const double db = powerToDb(err / std::max(sig, 1.0e-300));
            verdict(k, db < -100.0, "error against direct convolution " + String(db, 1) + " dB (bound -100; plan -120)");
        }
    }

    // ---- fx.maximizer ---------------------------------------------------------------------------------------------
    void maximizerRows()
    {
        const auto items = tortureSet();
        for (const bool os : {false, true})
        {
            double worstTp = -100.0, worstSample = -100.0;
            std::string worstName;
            const std::string k = key({"fx.maximizer", os ? "true_peak_4x" : "true_peak_sidechain", rateTag()});
            if (!want(k))
                continue;
            for (const Item& item : items)
            {
                Rack h(modOf("fx.maximizer"), gRate);
                h.set("fx_mx_ceiling", -1.0f);
                h.set("fx_mx_os", os ? 1.0f : 0.0f);
                const Samples y = h.render(toF(item.x), 64);
                const Signal d = toDouble(y);
                const double tp = amplitudeToDb(truePeak(d.data(), d.size(), 16, 32)) + 1.0;
                const double sp = amplitudeToDb(peakOf(y)) + 1.0;
                if (tp > worstTp)
                {
                    worstTp = tp;
                    worstName = item.name;
                }
                worstSample = std::max(worstSample, sp);
            }
            const double bound = os ? 0.02 : 0.1;
            verdict(k, worstTp <= bound && worstSample <= 1.0e-6,
                    "true peak over the ceiling by " + String(worstTp, 3) + " dB on '" + worstName + "' (bound " +
                        String(bound, 2) + "), sample peak over by " + String(worstSample, 6) + " dB (bound 0)");
        }
    }

    // ---- fx.clipper -----------------------------------------------------------------------------------------------
    void clipperRows()
    {
        {
            const std::string k = key({"fx.clipper", "alias_5k", rateTag()});
            if (want(k))
            {
                Rack h(modOf("fx.clipper"), gRate);
                h.set("fx_clp_drive", 6.0206f);
                h.set("fx_clp_ceiling", 0.0f);
                h.set("fx_clp_guard", 0.0f);
                const size_t settle = secs(0.2), window = 1 << 15;
                const Samples y = h.render(sineF(settle + window, 5000.0, 1.0, gRate), 64);
                const Signal d = toDouble(y, settle);
                const AliasResult a = measureAliasing(d.data(), window, gRate, 5000.0);
                verdict(k, a.aliasDb <= -90.0,
                        "alias power " + String(a.aliasDb, 1) + " dB, worst spur " + String(a.worstSpurDb, 1) +
                            " dB (bound -90 at Standard)");
            }
        }
        {
            const std::string k = key({"fx.clipper", "ceiling", rateTag()});
            if (want(k))
            {
                double worstTp = -100.0, worstSample = -100.0;
                for (const auto& item : tortureSet())
                {
                    Rack h(modOf("fx.clipper"), gRate);
                    h.set("fx_clp_drive", 0.0f);
                    h.set("fx_clp_ceiling", -1.0f);
                    h.set("fx_clp_guard", 1.0f);
                    const Samples y = h.render(toF(item.x), 64);
                    const Signal d = toDouble(y);
                    worstTp = std::max(worstTp, amplitudeToDb(truePeak(d.data(), d.size(), 16, 32)) + 1.0);
                    worstSample = std::max(worstSample, amplitudeToDb(peakOf(y)) + 1.0);
                }
                verdict(k, worstTp <= 0.1 && worstSample <= 1.0e-3,
                        "true peak over the ceiling by " + String(worstTp, 3) + " dB (bound 0.1), sample peak over by " +
                            String(worstSample, 4) + " dB (bound 0.001)");
            }
        }
    }

    // ---- util.meter -----------------------------------------------------------------------------------------------
    void meterRows()
    {
        crucible::fx::KWeightingCoefficients shelf, hp;
        crucible::fx::designKWeighting(gRate, shelf, hp);
        const auto mag = [&](const crucible::fx::KWeightingCoefficients& c)
        {
            const std::complex<double> z = std::polar(1.0, -2.0 * kPi * 1000.0 / gRate), z2 = z * z;
            return std::abs((c.b0 + c.b1 * z + c.b2 * z2) / (1.0 + c.a1 * z + c.a2 * z2));
        };
        const double a = std::pow(10.0, (-23.0 + 0.691) / 20.0) / (mag(shelf) * mag(hp));
        Rack h(modOf("util.meter"), gRate);
        const Samples tone = sineF(secs(10.0), 1000.0, a, gRate);
        const Samples y = h.render(tone, 128);
        auto* readout = h.rack.findCapability<MeterReadout>("util.meter");
        {
            const std::string k = key({"util.meter", "passthrough", rateTag()});
            if (want(k))
                verdict(k, std::memcmp(y.data(), tone.data(), tone.size() * sizeof(float)) == 0,
                        "output differs from input");
        }
        {
            const std::string k = key({"util.meter", "loudness_23", rateTag()});
            if (want(k))
            {
                float v[MeterReadout::kLoudnessValues] = {};
                readout->loudness(v);
                const double worst = std::max({std::abs(v[0] + 23.0), std::abs(v[1] + 23.0), std::abs(v[2] + 23.0)});
                verdict(k, worst <= 0.1,
                        "momentary " + String(v[0], 3) + ", short-term " + String(v[1], 3) + ", integrated " +
                            String(v[2], 3) + " LUFS (bound -23 +-0.1)");
            }
        }
        {
            const std::string k = key({"util.meter", "true_peak", rateTag()});
            if (want(k))
            {
                Rack t(modOf("util.meter"), gRate);
                const Signal x = sineWave(secs(1.0), gRate / 4.0, gRate, 0.9, 45.0 * kPi / 180.0);
                t.render(toF(x), 128);
                float v[MeterReadout::kLoudnessValues] = {};
                t.rack.findCapability<MeterReadout>("util.meter")->loudness(v);
                const double ref = amplitudeToDb(truePeak(x.data(), x.size(), 16, 32));
                verdict(k, std::abs(v[4] - ref) <= 0.2,
                        "true peak " + String(v[4], 3) + " dB, 16x reference " + String(ref, 3) + " (bound +-0.2)");
            }
        }
    }

    // ---- latency --------------------------------------------------------------------------------------------------
    struct LatencyCase
    {
        std::string name;
        Quality quality = Quality::Standard;
        std::function<void(Rack&)> setup;
        double noiseRms = 0.2;
    };

    void latencyCheck(const char* id, const LatencyCase& c)
    {
        const std::string k = key({id, "latency", rateTag(), c.name});
        if (!want(k))
            return;
        const Mod& m = modOf(id);
        Rack h(m, gRate, c.quality);
        c.setup(h);
        const Samples in = noiseF(secs(2.5), c.noiseRms, 7);
        const Samples out = h.render(in, 128);
        auto* lat = h.rack.findCapability<LatencyReporter>(id);
        const int reported = lat != nullptr ? lat->latencySamples() : 0;
        const size_t skip = secs(0.8);
        const double measured = measureLatency(toDouble(in, skip), toDouble(out, skip));
        const int rackReported = h.rack.latencySamples(h.values.data());
        logMessage(k + ": reported " + String(reported) + ", measured " + String(measured, 3));
        verdict(k, std::abs(measured - reported) <= 1.0 && rackReported == reported,
                "reported " + String(reported) + " samples, measured " + String(measured, 3) + " (bound +-1), rack total " +
                    String(rackReported));
    }

    void latencyRows()
    {
        beginTest(std::string("latency, reported against measured [") + rateTag() + " Hz]");
        for (const double look : {0.0, 1.0, 2.5, 5.0, 10.0})
            for (const bool tp : {false, true})
                latencyCheck("fx.comp_clean",
                             {"look" + String(look, 1).str() + (tp ? "_tp" : ""), Quality::Standard, [look, tp](Rack& h)
                              {
                                  ccNeutral(h);
                                  h.set("fx_cc_lookahead", static_cast<float>(look));
                                  h.set("fx_cc_truepeak", tp ? 1.0f : 0.0f);
                              }});
        latencyCheck("fx.comp_clean", {"feedback_look5", Quality::Standard, [](Rack& h)
                                       {
                                           ccNeutral(h);
                                           h.set("fx_cc_topology", 1.0f);
                                           h.set("fx_cc_lookahead", 5.0f);
                                       }});
        latencyCheck("fx.comp_vca", {"default", Quality::Standard, cvNeutral});
        for (const double look : {0.0, 1.0, 2.0, 5.0, 10.0})
            latencyCheck("fx.gate2", {"look" + String(look, 1).str(), Quality::Standard, [look](Rack& h)
                                      {
                                          g2Neutral(h);
                                          h.set("fx_g2_lookahead", static_cast<float>(look));
                                      }});
        for (const int phase : {0, 1})
            latencyCheck("fx.eq_pro", {phase == 0 ? "minimum" : "natural", Quality::Standard, [phase](Rack& h)
                                       {
                                           bandOn(h, 1, 0, 3000.0, 0.0, 0.7071);
                                           h.set("fx_ep_phase", static_cast<float>(phase));
                                       }});
        for (const int res : {0, 1, 2})
            latencyCheck("fx.eq_pro", {"linear_res" + std::to_string(res), Quality::High, [res](Rack& h)
                                       {
                                           bandOn(h, 1, 0, 3000.0, 3.0, 0.7071);
                                           h.set("fx_ep_phase", 2.0f);
                                           h.set("fx_ep_res", static_cast<float>(res));
                                       }});
        latencyCheck("fx.reverb_algo", {"mix0", Quality::Standard, [](Rack& h) { h.set("fx_ra_mix", 0.0f); }});
        for (const int mode : {0, 1})
            latencyCheck("fx.convolution", {mode == 0 ? "zero_latency" : "low_cpu", Quality::High, [mode](Rack& h)
                                            {
                                                h.set("fx_cx_mode", static_cast<float>(mode));
                                                h.set("fx_cx_mix", 1.0f);
                                                cxNeutral(h);
                                            }});
        for (const bool os : {false, true})
            for (const double look : {0.5, 1.0, 3.0, 5.0, 10.0})
                latencyCheck("fx.maximizer",
                             {std::string(os ? "4x_" : "side_") + "look" + String(look, 1).str(), Quality::Standard,
                              [os, look](Rack& h)
                              {
                                  mxNeutral(h);
                                  h.set("fx_mx_os", os ? 1.0f : 0.0f);
                                  h.set("fx_mx_lookahead", static_cast<float>(look));
                              },
                              0.05});
        for (const int os : {0, 1, 2, 3, 4})
            for (const bool guard : {false, true})
                latencyCheck("fx.clipper", {"os" + std::to_string(os) + (guard ? "_guard" : ""), Quality::Standard,
                                            [os, guard](Rack& h)
                                            {
                                                clpNeutral(h);
                                                h.set("fx_clp_shape", 0.0f);
                                                h.set("fx_clp_os", static_cast<float>(os));
                                                h.set("fx_clp_guard", guard ? 1.0f : 0.0f);
                                            },
                                            0.05});
        latencyCheck("util.meter", {"default", Quality::Standard, [](Rack&) {}});
    }

    // ---- zipper ---------------------------------------------------------------------------------------------------
    // Settings under which the parameter `p` of module `m` changes the output of a -6 dBFS (or the stated level) 1 kHz sine.
    struct ZipperSetup
    {
        double carrierDb = -6.0;
        double warmSeconds = 0.5;
        std::function<void(Rack&)> setup;
        Quality quality = Quality::Standard;
    };

    static ZipperSetup zipperSetupFor(const Mod& m, const ParamDef& p, int eqPhase)
    {
        ZipperSetup z;
        const std::string id = m.id;
        if (id == "fx.comp_clean")
            z.setup = [](Rack& h) { h.set("fx_cc_thresh", -30.0f); };
        else if (id == "fx.comp_vca")
            z.setup = [](Rack& h)
            {
                h.set("fx_cv_thresh", -30.0f);
                h.set("fx_cv_console", 1.0f);
            };
        else if (id == "fx.gate2")
            z.setup = [](Rack& h)
            {
                h.set("fx_g2_mode", 1.0f);
                h.set("fx_g2_thresh", -3.0f);
                h.set("fx_g2_ratio", 3.0f);
                h.set("fx_g2_range", -20.0f);
                // Slow ballistics: the gain then follows a changed threshold / filter smoothly, so what remains is zipper.
                h.set("fx_g2_attack", 10.0f);
                h.set("fx_g2_release", 200.0f);
            };
        else if (id == "fx.eq_pro")
        {
            // "fx_ep_b<n>_<name>"
            int band = 0;
            if (p.id.rfind("fx_ep_b", 0) == 0)
                band = std::atoi(p.id.c_str() + 7);
            const bool dynamicParam =
                p.id.size() > 4 && (p.id.find("_range") != std::string::npos || p.id.find("_thr") != std::string::npos ||
                                    p.id.find("_ratio") != std::string::npos || p.id.find("_att") != std::string::npos ||
                                    p.id.find("_rel") != std::string::npos);
            z.setup = [band, dynamicParam, eqPhase](Rack& h)
            {
                const int b = band == 0 ? 1 : band;
                bandOn(h, b, 0, 1000.0, 6.0, 1.0);
                if (dynamicParam)
                    h.set("fx_ep_b" + std::to_string(b) + "_dyn", 1.0f);
                h.set("fx_ep_phase", static_cast<float>(eqPhase));
                h.set("fx_ep_res", 1.0f);
            };
            if (eqPhase == 2)
                z.quality = Quality::High;
        }
        else if (id == "fx.reverb_algo")
        {
            z.warmSeconds = 2.0;
            z.setup = [](Rack& h)
            {
                h.set("fx_ra_mix", 1.0f);
                h.set("fx_ra_moddepth", 0.0f);
                h.set("fx_ra_decay", 1.0f);
            };
        }
        else if (id == "fx.convolution")
        {
            z.warmSeconds = 1.0;
            z.quality = Quality::High;
            z.setup = [](Rack& h)
            {
                const size_t len = secs(0.5);
                Samples k(len), kr(len);
                for (size_t i = 0; i < len; ++i)
                {
                    const double t = static_cast<double>(i) / gRate;
                    k[i] = static_cast<float>(2.8e-4 * std::exp(-t / 0.15) * std::cos(2.0 * kPi * 1000.0 * t + 0.3));
                    kr[i] = static_cast<float>(2.8e-4 * std::exp(-t / 0.15) * std::cos(2.0 * kPi * 1000.0 * t + 1.1));
                }
                h.setIr({k, kr}, IrLayout::Stereo);
            };
        }
        else if (id == "fx.maximizer")
            z.setup = [](Rack& h)
            {
                h.set("fx_mx_gain", 12.0f);
                h.set("fx_mx_ceiling", -6.0f);
                h.set("fx_mx_release", 500.0f);
            };
        else if (id == "fx.clipper")
        {
            z.carrierDb = -30.0;
            z.setup = [](Rack& h) { h.set("fx_clp_drive", 12.0f); };
        }
        else
            z.setup = [](Rack&) {};
        return z;
    }

    static double automate(const ParamDef& p, double a)
    {
        if (p.skew != 1.0f && p.min > 0.0f)
            return p.min * std::pow(static_cast<double>(p.max) / p.min, a);
        return p.min + (static_cast<double>(p.max) - p.min) * a;
    }

    // fx.maximizer fx_mx_gain: raising the gain drives the limiter deeper, so the gain-reduction motion that follows
    // is intended, not zipper. Maintainer decision: the reference is the same maximizer fed with the same gain change
    // applied smoothly, per sample, to its INPUT (the module gain held at its starting value); the residue
    // (module output minus reference) holds only real block steps. Control: the same gain change applied to the input
    // once per block, unsmoothed, must still fail the check.
    enum class MxRef
    {
        None,    // ordinary check
        Smooth,  // module automated per block, judged against the smooth-input reference
        Control, // gain stepped once per block on the input, judged against the smooth-input reference
    };

    void zipperCheck(const Mod& m, const ParamDef& p, size_t block, int eqPhase, MxRef mx = MxRef::None)
    {
        const bool mxGain = std::strcmp(m.id, "fx.maximizer") == 0 && p.id == "fx_mx_gain";
        const std::string k = key({m.id, eqPhase == 2 ? "zipper_linear" : "zipper", p.id, "block" + std::to_string(block)});
        const std::string ctrlKey =
            key({m.id, "zipper_control", p.id, "block" + std::to_string(block)});
        if (mx == MxRef::Control ? !want(ctrlKey) : !want(k))
            return;
        if (mxGain && mx == MxRef::None)
            mx = MxRef::Smooth;
        const ZipperSetup z = zipperSetupFor(m, p, eqPhase);
        const double amp = dbToAmplitude(z.carrierDb);
        // mode: 0 module automated per block; 1 module gain held, input scaled smoothly per sample; 2 module gain held,
        // input scaled once per block.
        const auto renderMode = [&](const Signal& in, const Signal& automation, int mode)
        {
            Rack h(m, gRate, z.quality);
            z.setup(h);
            const double g0 = automate(p, automation[0]);
            h.set(p.id, static_cast<float>(g0));
            const auto scale = [&](size_t src) { return std::pow(10.0, (automate(p, automation[src]) - g0) / 20.0); };
            // The module's own gain smoothing (three one-pole stages of 20 ms on the linear gain, src/fx/Maximizer.cpp),
            // run per sample on the smooth automation: the reference for what the module does with a smooth gain change.
            const double sc = 1.0 - std::exp(-1.0 / (0.020 * gRate));
            double s1 = 1.0, s2 = 1.0, s3 = 1.0;
            const auto smoothScale = [&](size_t src)
            {
                const double t = scale(src);
                s1 += (t - s1) * sc;
                s2 += (s1 - s2) * sc;
                s3 += (s2 - s3) * sc;
                return s3;
            };
            if (mode != 0)
            {
                const size_t warm = secs(z.warmSeconds);
                Samples l(warm + in.size());
                for (size_t i = 0; i < l.size(); ++i)
                    l[i] = static_cast<float>(
                        amp * std::sin(2.0 * kPi * 1000.0 * (static_cast<double>(i) - static_cast<double>(warm)) / gRate));
                for (size_t pos = 0; pos < l.size(); pos += block)
                {
                    const size_t end = std::min(pos + block, l.size());
                    const size_t held = end - 1 >= warm ? end - 1 - warm : 0;
                    for (size_t i = pos; i < end; ++i)
                    {
                        const size_t src = i >= warm ? i - warm : 0;
                        l[i] = static_cast<float>(l[i] * (mode == 1 ? smoothScale(src) : scale(held)));
                    }
                }
                Samples r = l;
                for (size_t pos = 0; pos < l.size(); pos += block)
                {
                    const size_t end = std::min(pos + block, l.size());
                    h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
                }
                return toDouble(l, warm);
            }
            const size_t warm = secs(z.warmSeconds);
            Samples l(warm + in.size());
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = static_cast<float>(
                    amp * std::sin(2.0 * kPi * 1000.0 * (static_cast<double>(i) - static_cast<double>(warm)) / gRate));
            Samples r = l;
            for (size_t pos = 0; pos < l.size(); pos += block)
            {
                const size_t end = std::min(pos + block, l.size());
                const size_t src = end - 1 >= warm ? end - 1 - warm : 0;
                h.set(p.id, static_cast<float>(automate(p, automation[src])));
                h.run(l.data() + pos, r.data() + pos, static_cast<int>(end - pos));
            }
            return toDouble(l, warm);
        };
        const auto render = [&](const Signal& in, const Signal& automation)
        {
            if (mx == MxRef::None)
                return renderMode(in, automation, 0);
            Signal a = renderMode(in, automation, mx == MxRef::Smooth ? 0 : 2);
            const Signal ref = renderMode(in, automation, 1);
            for (size_t i = 0; i < a.size(); ++i)
                a[i] -= ref[i];
            return a;
        };
        const ZipperResult zr = zipperTest(render, gRate, 1000.0, z.carrierDb);
        // Floor of the held setting at both ends: a nonlinear or time-varying module leaves a residue that the step
        // estimator cannot tell from a step. The limit is the plan's -90 dB, or that floor + 3 dB where the floor is above
        // it.
        const size_t held = secs(0.6);
        double floorDb = -300.0;
        for (const double a : {0.0, 1.0})
        {
            const Signal hold = render(sineWave(held, 1000.0, gRate, amp), Signal(held, a));
            floorDb = std::max(floorDb, clickTest(hold, 1000.0, gRate, secs(0.1)).stepDb);
        }
        const double limit = std::max(-90.0, floorDb + 3.0);
        if (mx == MxRef::Control)
        {
            verdict(ctrlKey, zr.rampDb > limit || zr.squareDb > limit,
                    "control (gain stepped once per block, unsmoothed) must fail: ramp " + String(zr.rampDb, 1) +
                        " dB, square " + String(zr.squareDb, 1) + " dB, limit " + String(limit, 1) + " dB");
            return;
        }
        verdict(k, zr.rampDb <= limit && zr.squareDb <= limit,
                "ramp " + String(zr.rampDb, 1) + " dB, square " + String(zr.squareDb, 1) + " dB, held floor " +
                    String(floorDb, 1) + " dB, limit " + String(limit, 1) + " dB");
    }

    void zipperRows()
    {
        gRate = 48000.0;
        for (const Mod& m : kMods)
        {
            const ModuleDescriptor* d = ModuleRegistry::instance().find(m.id);
            beginTest(std::string(m.id) +
                      ": zipper below -90 dB on every continuous parameter at host blocks of 1, 7 and 512");
            int cases = 0;
            for (const ParamDef& p : d->params)
            {
                if (p.type != ParamType::Float || p.id == m.on || p.interval != 0.0f)
                    continue;
                for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
                {
                    zipperCheck(m, p, block, 0), ++cases;
                    if (std::strcmp(m.id, "fx.maximizer") == 0 && p.id == "fx_mx_gain" && block > 1)
                        zipperCheck(m, p, block, 0, MxRef::Control), ++cases;
                }
            }
            if (std::strcmp(m.id, "fx.eq_pro") == 0)
            {
                // Linear phase: the band parameters that change the kernel.
                for (const char* name : {"fx_ep_b1_freq", "fx_ep_b1_gain", "fx_ep_b1_q", "fx_ep_out"})
                    for (const ParamDef& p : d->params)
                        if (p.id == name)
                            for (const size_t block : {size_t{1}, size_t{7}, size_t{512}})
                                zipperCheck(m, p, block, 2), ++cases;
            }
            if (std::strcmp(m.id, "util.meter") == 0)
                meterDiscrete();
            logMessage(String(m.id) + ": " + String(cases) + " zipper cases");
        }
    }

    // util.meter has no continuous parameter: toggling its discrete ones must leave the audio bit-exact.
    void meterDiscrete()
    {
        const std::string k = key({"util.meter", "discrete_changes", "48000"});
        if (!want(k))
            return;
        Rack h(modOf("util.meter"), 48000.0);
        const Samples tone = sineF(secs(2.0), 1000.0, 0.5, 48000.0);
        Samples l = tone, r = tone;
        for (size_t pos = 0; pos < l.size(); pos += 100)
        {
            h.set("fx_mtr_reset", (pos / 1000) % 2 == 0 ? 1.0f : 0.0f);
            h.set("fx_mtr_ppm", (pos / 3000) % 2 == 0 ? 1.0f : 0.0f);
            h.run(l.data() + pos, r.data() + pos, static_cast<int>(std::min<size_t>(100, l.size() - pos)));
        }
        verdict(k, std::memcmp(l.data(), tone.data(), tone.size() * sizeof(float)) == 0, "output differs from input");
    }

    // ---- reset and bypass -----------------------------------------------------------------------------------------
    void resetBypassRows(double rate)
    {
        gRate = rate;
        for (const Mod& m : kMods)
        {
            beginTest(std::string(m.id) + " [" + rateTag() + " Hz]: reset() and an off / on cycle leave no stale state");
            if (std::strcmp(m.id, "fx.reverb_algo") == 0)
            {
                // Control for the checks below: without reset() the tail must still be there.
                const std::string k = key({m.id, "reset_control", rateTag()});
                if (want(k))
                {
                    Rack h(m, gRate, m.quality);
                    m.active(h);
                    h.render(noiseF(secs(1.0), 0.5, 3), 128);
                    const Samples y = h.render(Samples(secs(0.2), 0.0f), 128);
                    verdict(k, peakOf(y) > 1.0e-4f, "the test can see a tail");
                }
            }
            {
                const std::string k = key({m.id, "reset", rateTag()});
                if (want(k))
                {
                    Rack h(m, gRate, m.quality);
                    m.active(h);
                    h.render(noiseF(secs(1.0), 0.5, 3), 128);
                    h.rack.reset();
                    const Samples y = h.render(Samples(secs(1.0), 0.0f), 128);
                    const float p = peakOf(y);
                    verdict(k, p < kSilence, "peak after reset " + String(amplitudeToDb(p), 1) + " dBFS (bound -150)");
                }
            }
            {
                const std::string k = key({m.id, "bypass_cycle", rateTag()});
                if (want(k))
                {
                    Rack h(m, gRate, m.quality);
                    m.active(h);
                    h.render(noiseF(secs(1.0), 0.5, 3), 128);
                    h.set(m.on, 0.0f);
                    const Samples loud = noiseF(secs(0.5), 0.5, 4);
                    const Samples bypassed = h.render(loud, 128);
                    // Bypassed: the input, bit for bit, once the 10 ms fade is over.
                    size_t differs = 0;
                    for (size_t i = secs(0.1); i < loud.size(); ++i)
                        differs += bypassed[i] != loud[i] ? 1 : 0;
                    h.set(m.on, 1.0f);
                    const Samples y = h.render(Samples(secs(1.0), 0.0f), 128);
                    const float p = peakOf(y);
                    verdict(k, differs == 0 && p < kSilence,
                            "bypassed output differs from input in " + String(static_cast<double>(differs), 0) +
                                " samples (bound 0), peak after switching on " + String(amplitudeToDb(p), 1) +
                                " dBFS (bound -150)");
                }
            }
        }
        beginTest(std::string("loudness readouts after reset [") + rateTag() + " Hz]");
        for (const char* id : {"util.meter", "fx.maximizer"})
        {
            const std::string k = key({id, "readout_reset", rateTag()});
            if (!want(k))
                continue;
            const Mod& m = modOf(id);
            Rack h(m, gRate);
            m.active(h);
            h.render(noiseF(secs(3.0), 0.2, 3), 128);
            h.rack.reset();
            h.render(Samples(secs(0.5), 0.0f), 128);
            float v[MeterReadout::kLoudnessValues] = {};
            h.rack.findCapability<MeterReadout>(id)->loudness(v);
            verdict(k, v[0] <= -100.0f && v[1] <= -100.0f && v[2] <= -100.0f && v[4] <= -100.0f,
                    "momentary " + String(v[0], 1) + ", short-term " + String(v[1], 1) + ", integrated " + String(v[2], 1) +
                        ", true peak " + String(v[4], 1) + " after reset (bound <= -100)");
        }
    }
};

class OlderFxPass : public OlderFxBase
{
public:
    OlderFxPass() : OlderFxBase("OlderFxPass", Mode::Pass) {}
};
class OlderFxKnownFail : public OlderFxBase
{
public:
    OlderFxKnownFail() : OlderFxBase("OlderFxKnownFail", Mode::Known) {}
};
class OlderFxAll : public OlderFxBase
{
public:
    OlderFxAll() : OlderFxBase("OlderFxAll", Mode::All) {}
};

OlderFxPass gOlderFxPass;
OlderFxKnownFail gOlderFxKnownFail;
OlderFxAll gOlderFxAll;
} // namespace
