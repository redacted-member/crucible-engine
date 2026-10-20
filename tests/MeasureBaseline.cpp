// SPDX-License-Identifier: MIT
// Baseline measurement of the current filters and oscillators with the primitives of AnalogMeasure.h. Prints
// Markdown tables (docs/MEASUREMENTS.md embeds the output) and is not part of CTest:
//
//   crucible_engine_measure [filters|oscillators|envelope|drift|all]
//
// Everything is rendered offline and analysed as numbers; nothing is played.

#include "AnalogMeasure.h"
#include "GraphSupport.h"

#include <crucible/Filters.h>
#include <crucible/Voice.h>
#include <crucible/Wavetables.h>

#include <cstdio>
#include <cstring>
#include <string>

using namespace crucible;
using namespace crucible::testing;
namespace m = crucible::measure;

namespace
{
constexpr double kFilterRate = 48000.0;

auto makeFilter(int model, float cutoff, float res)
{
    return [=]
    {
        FilterChannel c;
        c.configure({model, cutoff, res}, kFilterRate);
        return [c](double x) mutable { return static_cast<double>(c.process(static_cast<float>(x))); };
    };
}

bool isLowpass(int model)
{
    const auto f = static_cast<FilterModel>(model);
    return f == FilterModel::Lowpass12 || f == FilterModel::Lowpass24 || f == FilterModel::LadderLowpass ||
           f == FilterModel::DiodeLowpass || f == FilterModel::DirtyLowpass;
}

bool isHighpass(int model)
{
    const auto f = static_cast<FilterModel>(model);
    return f == FilterModel::Highpass12 || f == FilterModel::Highpass24 || f == FilterModel::LadderHighpass;
}

std::string fmt(double v, int decimals = 2)
{
    if (v <= -290.0)
        return "exact zero";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

void filterTables()
{
    const auto& names = filterModelNames();
    const int models = static_cast<int>(names.size());
    const auto grid = m::logSpacedFrequencies(20.0, 20000.0, 12);

    std::printf("### Filters at 1 kHz, resonance 0 (stepped sines at -40 dBFS, 48 kHz)\n\n");
    std::printf("Slope: least-squares dB per octave between 2 and 4 octaves above (low-pass) or below (high-pass) the "
                "cutoff; for the "
                "low-passes also at a 250 Hz cutoff, where the bilinear transform's frequency warping does not yet bend the "
                "response.\n\n");
    std::printf("| model | gain 100 Hz (dB) | gain 10 kHz (dB) | -3 dB point (Hz) | slope, 1 kHz cutoff (dB/oct) | slope, "
                "250 Hz cutoff (dB/oct) |\n|---|---|---|---|---|---|\n");
    for (int model = 0; model < models; ++model)
    {
        if (!isLowpass(model) && !isHighpass(model))
            continue;
        const auto c = m::steppedResponse(makeFilter(model, 1000.0f, 0.0f), grid, kFilterRate, 0.01, 0.4, 0.2);
        const auto at = [&](double f) { return m::steppedGainDb(makeFilter(model, 1000.0f, 0.0f), f, kFilterRate, 0.01); };
        const double pass = isLowpass(model) ? at(100.0) : at(10000.0);
        // -3 dB against the passband level.
        m::ResponseCurve rel = c;
        for (auto& g : rel.gainDb)
            g -= pass;
        double three = 0.0;
        if (isLowpass(model))
            three = m::crossingBelow(rel, -3.0103, 200.0);
        else
            for (size_t i = 1; i < rel.freqHz.size(); ++i)
                if (rel.gainDb[i - 1] < -3.0103 && rel.gainDb[i] >= -3.0103)
                {
                    three = rel.freqHz[i];
                    break;
                }
        const double slope =
            isLowpass(model) ? m::slopeDbPerOctave(c, 4000.0, 16000.0) : m::slopeDbPerOctave(c, 62.5, 250.0);
        std::string slope250 = "-";
        if (isLowpass(model))
        {
            const auto lowGrid = m::logSpacedFrequencies(500.0, 8000.0, 12);
            slope250 = fmt(m::slopeDbPerOctave(
                               m::steppedResponse(makeFilter(model, 250.0f, 0.0f), lowGrid, kFilterRate, 0.01, 0.4, 0.2),
                               1000.0, 4000.0),
                           1);
        }
        std::printf("| %s | %s | %s | %s | %s | %s |\n", names[static_cast<size_t>(model)].c_str(), fmt(at(100.0)).c_str(),
                    fmt(at(10000.0)).c_str(), fmt(three, 0).c_str(), fmt(slope, 1).c_str(), slope250.c_str());
    }

    std::printf("\n### Resonance peak and bass loss, cutoff 1 kHz\n\n");
    std::printf(
        "Peak: highest point of the response between 100 Hz and 8 kHz (stepped sines at -40 dBFS, 24 points per octave). "
        "Bass loss: gain at 50 Hz against the same filter at resonance 0.\n\n");
    std::printf("| model | resonance | peak (dB) | peak at (Hz) | bass loss at 50 Hz (dB) |\n|---|---|---|---|---|\n");
    const auto fine = m::logSpacedFrequencies(100.0, 8000.0, 24);
    for (int model = 0; model < models; ++model)
    {
        if (!isLowpass(model))
            continue;
        const double base = m::steppedGainDb(makeFilter(model, 1000.0f, 0.0f), 50.0, kFilterRate, 0.01, 0.6, 0.3);
        for (const float res : {0.25f, 0.5f, 0.75f, 0.9f, 1.0f})
        {
            const auto c = m::steppedResponse(makeFilter(model, 1000.0f, res), fine, kFilterRate, 0.01, 0.6, 0.2);
            const auto pk = m::resonancePeak(c);
            const double bass = m::steppedGainDb(makeFilter(model, 1000.0f, res), 50.0, kFilterRate, 0.01, 0.6, 0.3);
            std::printf("| %s | %.2f | %s | %s | %s |\n", names[static_cast<size_t>(model)].c_str(), res,
                        fmt(pk.gainDb).c_str(), fmt(pk.freqHz, 0).c_str(), fmt(bass - base).c_str());
        }
    }

    std::printf("\n### Self-oscillation (resonance 1, cutoff 1 kHz, impulse start, 2 s)\n\n");
    std::printf("| model | oscillates | frequency (Hz) | error against cutoff (cent) | level (dBFS rms) | level variation "
                "(dB) |\n|---|---|---|---|---|---|\n");
    std::vector<int> oscillating;
    for (int model = 0; model < models; ++model)
    {
        const auto r = m::measureSelfOscillation(makeFilter(model, 1000.0f, 1.0f), kFilterRate, 2.0, 0.05, 1000.0);
        if (r.oscillating)
            oscillating.push_back(model);
        std::printf("| %s | %s | %s | %s | %s | %s |\n", names[static_cast<size_t>(model)].c_str(),
                    r.oscillating ? "yes" : "no", r.oscillating ? fmt(r.freqHz, 1).c_str() : "-",
                    r.oscillating ? fmt(m::centsBetween(r.freqHz, 1000.0), 1).c_str() : "-",
                    r.levelDbfs > -120.0 ? fmt(r.levelDbfs, 1).c_str() : "decays to nothing",
                    r.levelDbfs > -120.0 ? fmt(r.amplitudeVariationDb, 2).c_str() : "-");
    }

    std::printf("\n### Self-oscillation pitch tracking (error in cents against the cutoff setting)\n\n");
    const float cutoffs[] = {40.0f, 100.0f, 250.0f, 1000.0f, 3000.0f, 8000.0f};
    std::printf("| model |");
    for (const float fc : cutoffs)
        std::printf(" %g Hz |", fc);
    std::printf(" amplitude spread (dB) |\n|---|");
    for (size_t i = 0; i < std::size(cutoffs); ++i)
        std::printf("---|");
    std::printf("---|\n");
    for (const int model : oscillating)
    {
        std::printf("| %s |", names[static_cast<size_t>(model)].c_str());
        double lo = 1.0e9, hi = -1.0e9;
        for (const float fc : cutoffs)
        {
            const auto r = m::measureSelfOscillation(makeFilter(model, fc, 1.0f), kFilterRate, 3.0, 0.05, fc);
            if (!r.oscillating)
            {
                std::printf(" - |");
                continue;
            }
            lo = std::min(lo, r.levelDbfs);
            hi = std::max(hi, r.levelDbfs);
            std::printf(" %s |", fmt(m::centsBetween(r.freqHz, fc), 1).c_str());
        }
        std::printf(" %s |\n", fmt(hi - lo, 1).c_str());
    }

    std::printf("\n### Level dependence (gain at the resonance peak, resonance 0.9, cutoff 1 kHz; input level in dBFS)\n\n");
    std::printf("| model | -40 | -12 | 0 | +6 |\n|---|---|---|---|---|\n");
    for (int model = 0; model < models; ++model)
    {
        if (!isLowpass(model))
            continue;
        const auto base = m::steppedResponse(makeFilter(model, 1000.0f, 0.9f), fine, kFilterRate, 0.01, 0.6, 0.2);
        const double peakHz = m::resonancePeak(base).freqHz;
        std::printf("| %s |", names[static_cast<size_t>(model)].c_str());
        for (const double level : {-40.0, -12.0, 0.0, 6.0})
            std::printf(" %s |", fmt(m::steppedGainDb(makeFilter(model, 1000.0f, 0.9f), peakHz, kFilterRate,
                                                      m::dbToAmplitude(level), 0.6, 0.2))
                                     .c_str());
        std::printf("\n");
    }

    std::printf("\n### Filter drive: THD of a 1 kHz sine at 0.5 amplitude (cutoff 8 kHz, resonance 0)\n\n");
    std::printf("| model | drive 0 | drive 0.5 | drive 1 |\n|---|---|---|---|\n");
    for (const int model : {static_cast<int>(FilterModel::Lowpass12), static_cast<int>(FilterModel::Lowpass24),
                            static_cast<int>(FilterModel::LadderLowpass), static_cast<int>(FilterModel::DiodeLowpass),
                            static_cast<int>(FilterModel::DirtyLowpass)})
    {
        std::printf("| %s |", names[static_cast<size_t>(model)].c_str());
        for (const float drive : {0.0f, 0.5f, 1.0f})
        {
            FilterUnit unit;
            unit.configure({model, 8000.0f, 0.0f}, drive, kFilterRate);
            std::vector<double> out(1 << 16);
            for (size_t i = 0; i < out.size() + 8192; ++i)
            {
                float l = static_cast<float>(0.5 * std::sin(2.0 * m::kPi * 1000.0 * static_cast<double>(i) / kFilterRate)),
                      r = l;
                unit.process(l, r);
                if (i >= 8192)
                    out[i - 8192] = l;
            }
            std::printf(" %s dB |", fmt(m::harmonicProfile(out, kFilterRate, 1000.0, 20).thdDb, 1).c_str());
        }
        std::printf("\n");
    }
    std::printf("\n");
}

// A one-oscillator patch: filter off, a single voice, phase fixed.
std::vector<float> oscPatch(const char* table, float pos)
{
    auto v = graphPatch();
    setValue(v, "filter_on", 0.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc1_uni_rand", 0.0f);
    int index = 0;
    for (size_t i = 0; i < WavetableBank::tableNames().size(); ++i)
        if (WavetableBank::tableNames()[i] == table)
            index = static_cast<int>(i);
    setValue(v, "osc1_table", static_cast<float>(index));
    setValue(v, "osc1_pos", pos);
    return v;
}

m::Signal renderNote(const std::vector<float>& values, double rate, int note, double seconds, const EngineConfig& cfg = {})
{
    SynthEngine engine(cfg);
    engine.prepare(rate);
    engine.setParameterValues(values.data(), static_cast<int>(values.size()));
    engine.noteOn(note, 1.0f);
    const auto r = renderBlocks(engine, static_cast<int>(seconds * rate), 512, true);
    return m::toDouble(r.left);
}

void oscillatorTables()
{
    struct Wave
    {
        const char* name;
        const char* table;
        float pos;
    };
    const Wave waves[] = {{"sine", "Sine-Saw", 0.0f}, {"saw", "Sine-Saw", 1.0f}, {"square", "Analog Square", 0.5f}};
    const int notes[] = {36, 60, 84, 96, 108};

    std::printf("### Wavetable oscillator: tuning, aliasing and shape (one voice, filter off)\n\n");
    std::printf("Aliasing: power outside the harmonics below 20 kHz against the fundamental. Spur: strongest single "
                "non-harmonic line. Shape: worst deviation of the harmonic levels from the ideal law (1/n saw, odd 1/n "
                "square) up to 10 kHz. "
                "THD+N for the sine.\n\n");
    std::printf("| wave | rate | note | f0 (Hz) | tuning (cent) | alias (dB) | worst spur (dB) | shape (dB) | THD+N (dB) | "
                "DC (dBFS) |\n|---|---|---|---|---|---|---|---|---|---|\n");
    for (const Wave& w : waves)
        for (const double rate : {44100.0, 48000.0, 96000.0})
            for (const int note : notes)
            {
                const auto sig = renderNote(oscPatch(w.table, w.pos), rate, note, 2.2);
                const size_t start = static_cast<size_t>(0.6 * rate);
                const size_t n = m::floorPow2(sig.size() - start);
                const double expected = midiToHz(note);
                const double f0 = m::estimateFrequency(sig.data() + start, n, rate, expected);
                const auto alias = m::measureAliasing(sig.data() + start, n, rate, f0);
                const auto prof = m::harmonicProfile(sig.data() + start, n, rate, f0, 200);
                std::string shape = "-";
                if (std::strcmp(w.name, "saw") == 0)
                    shape = fmt(m::shapeError(
                                    prof, [](int k) { return -20.0 * std::log10(static_cast<double>(k)); }, 10000.0)
                                    .maxAbsDb,
                                2);
                else if (std::strcmp(w.name, "square") == 0)
                    shape = fmt(m::shapeError(
                                    prof, [](int k) { return -20.0 * std::log10(static_cast<double>(k)); }, 10000.0,
                                    [](int k) { return (k & 1) != 0; })
                                    .maxAbsDb,
                                2);
                std::printf("| %s | %g | %d | %s | %s | %s | %s | %s | %s | %s |\n", w.name, rate, note, fmt(f0, 3).c_str(),
                            fmt(m::centsBetween(f0, expected), 3).c_str(), fmt(alias.aliasDb, 1).c_str(),
                            fmt(alias.worstSpurDb, 1).c_str(), shape.c_str(),
                            std::strcmp(w.name, "sine") == 0 ? fmt(prof.thdPlusNoiseDb, 1).c_str() : "-",
                            fmt(m::amplitudeToDb(std::abs(m::dcOffsetWindowed(sig.data() + start, n))), 1).c_str());
            }

    std::printf("\n### Sine harmonic profile (note 60, 48 kHz): level of harmonics 2..6 against the fundamental\n\n");
    {
        const auto sig = renderNote(oscPatch("Sine-Saw", 0.0f), 48000.0, 60, 2.2);
        const size_t start = 28800, n = m::floorPow2(sig.size() - start);
        const auto prof = m::harmonicProfile(sig.data() + start, n, 48000.0, 0.0, 6);
        std::printf("| THD (dB) | H2 | H3 | H4 | H5 | H6 |\n|---|---|---|---|---|---|\n| %s |", fmt(prof.thdDb, 1).c_str());
        for (int k = 2; k <= 6; ++k)
            std::printf(" %s |", fmt(prof.harmonicDb(k), 1).c_str());
        std::printf("\n");
    }

    std::printf("\n### Warp aliasing (saw table, note 84, 48 kHz): alias power below 20 kHz against the fundamental\n\n");
    std::printf("| warp | amount 0.2 | amount 0.5 | amount 0.8 |\n|---|---|---|---|\n");
    const struct
    {
        const char* name;
        WarpMode mode;
    } warps[] = {{"PWM", WarpMode::Pwm},       {"Hard sync", WarpMode::SyncHard}, {"Soft sync", WarpMode::SyncSoft},
                 {"Mirror", WarpMode::Mirror}, {"Asym", WarpMode::Asym},          {"Fold", WarpMode::Fold}};
    for (const auto& w : warps)
    {
        std::printf("| %s |", w.name);
        for (const float amount : {0.2f, 0.5f, 0.8f})
        {
            auto v = oscPatch("Sine-Saw", 1.0f);
            setValue(v, "osc1_warp", static_cast<float>(w.mode));
            setValue(v, "osc1_warp_a", amount);
            const auto sig = renderNote(v, 48000.0, 84, 2.2);
            const size_t start = 28800, n = m::floorPow2(sig.size() - start);
            const double f0 = m::estimateFrequency(sig.data() + start, n, 48000.0, midiToHz(84));
            std::printf(" %s dB |",
                        fmt(m::measureAliasing(sig.data() + start, n, 48000.0, f0, 20000.0, 2).aliasDb, 1).c_str());
        }
        std::printf("\n");
    }

    std::printf("\n### Noise floor and idle output\n\n");
    {
        auto v = oscPatch("Sine-Saw", 0.0f);
        setValue(v, "osc1_level", 0.0f);
        const auto held = renderNote(v, 48000.0, 60, 1.5);
        const m::Signal tail(held.begin() + 24000, held.end());
        SynthEngine idle;
        prepare(idle, ParameterMap::defaultValues());
        const auto silent = m::toDouble(renderBlocks(idle, 48000, 512, true).left);
        std::printf("| case | RMS (dBFS) | DC (dBFS) |\n|---|---|---|\n| voice held, oscillator level 0 | %s | %s |\n| no "
                    "notes | %s | %s |\n\n",
                    fmt(m::rmsDbfs(tail), 1).c_str(), fmt(m::dcDbfs(tail), 1).c_str(), fmt(m::rmsDbfs(silent), 1).c_str(),
                    fmt(m::dcDbfs(silent), 1).c_str());
    }
}

void envelopeTables(bool analog = false)
{
    std::printf("### Amplitude envelope%s (sine A4, attack 100 ms, decay 300 ms, sustain 50 %%, release 400 ms; RMS "
                "envelope, %s window)\n\n",
                analog ? ", analog RC" : "", analog ? "3-period" : "9-period");
    auto v = oscPatch("Sine-Saw", 0.0f);
    if (analog)
        setValue(v, "amp_mode", 1.0f);
    setValue(v, "amp_attack", 0.1f);
    setValue(v, "amp_decay", 0.3f);
    setValue(v, "amp_sustain", 0.5f);
    setValue(v, "amp_release", 0.4f);
    SynthEngine engine;
    engine.prepare(48000.0);
    engine.setParameterValues(v.data(), static_cast<int>(v.size()));
    engine.noteOn(69, 1.0f);
    m::Signal audio;
    const auto append = [&](int samples)
    {
        const auto r = renderBlocks(engine, samples, 512, true);
        const auto d = m::toDouble(r.left);
        audio.insert(audio.end(), d.begin(), d.end());
    };
    append(static_cast<int>(1.1 * 48000.0));
    engine.noteOff(69);
    append(static_cast<int>(1.5 * 48000.0));
    // The analog RC table reads the audio with a 3-period window (half the smoothing of the digital rows' 9) and a 0.5 ms
    // hop.
    const auto env =
        analog ? m::rmsEnvelope(audio, 48000.0, 3.0 / 440.0, 0.0005) : m::rmsEnvelope(audio, 48000.0, 9.0 / 440.0, 0.002);
    double peak = 0.0, peakT = 0.0;
    for (size_t i = 0; i < env.level.size(); ++i)
        if (env.level[i] > peak)
        {
            peak = env.level[i];
            peakT = env.timeSeconds[i];
        }
    const auto slice = [&](double t0, double t1)
    {
        m::Signal t, y;
        for (size_t i = 0; i < env.level.size(); ++i)
            if (env.timeSeconds[i] >= t0 && env.timeSeconds[i] <= t1)
            {
                t.push_back(env.timeSeconds[i]);
                y.push_back(env.level[i] / peak);
            }
        return std::make_pair(t, y);
    };
    const auto [ta, ya] = slice(0.02, peakT);
    const auto [td, yd] = slice(peakT, 1.05);
    const auto [tr, yr] = slice(1.1, 2.3);
    const auto a = m::fitExponential(ta, ya);
    const auto d = m::fitExponential(td, yd);
    const auto r = m::fitExponential(tr, yr);
    const m::Signal allT = env.timeSeconds;
    m::Signal norm;
    for (const double y : env.level)
        norm.push_back(y / peak);
    std::printf("| segment | time to peak / constant | single-exponential fit residual (rms, %% of range) | note "
                "|\n|---|---|---|---|\n");
    const double t10 = m::timeToLevel(allT, norm, 0.1), t90 = m::timeToLevel(allT, norm, 0.9);
    std::printf("| attack | peak at %s s (10-90 %%: %s) | %s | RC fit tau %s s, target level %s |\n", fmt(peakT, 3).c_str(),
                t10 >= 0.0 && t90 >= 0.0 ? (fmt(t90 - t10, 3) + " s").c_str() : "starts above 10 %",
                fmt(100.0 * a.residualRms, 2).c_str(), fmt(a.tau, 3).c_str(), fmt(a.offset, 2).c_str());
    std::printf("| decay | tau %s s | %s | settles at %s of peak |\n", fmt(d.tau, 3).c_str(),
                fmt(100.0 * d.residualRms, 2).c_str(), fmt(d.offset, 3).c_str());
    std::printf("| release | tau %s s | %s | falls to %s |\n\n", fmt(r.tau, 3).c_str(),
                fmt(100.0 * r.residualRms, 2).c_str(), fmt(r.offset, 4).c_str());
}

void driftTables()
{
    std::printf("### Voice drift (voice_drift = 1, sine A4, pitch measured every 20 ms from 42.7 ms windows)\n\n");
    auto v = oscPatch("Sine-Saw", 0.0f);
    setValue(v, "voice_drift", 1.0f);
    std::vector<m::Signal> runs;
    for (std::uint32_t seed = 1; seed <= 24; ++seed)
    {
        EngineConfig cfg;
        cfg.seed = seed;
        const auto sig = renderNote(v, 48000.0, 69, 10.5, cfg);
        const m::Signal body(sig.begin() + 24000, sig.end());
        runs.push_back(m::pitchTraceCents(body, 48000.0, 0.02, 0.0427, 440.0));
    }
    const auto s = m::ouStats(runs, 0.02, 1.0, 10.0);
    // A long single run for the spectrum.
    EngineConfig cfg;
    cfg.seed = 99;
    const auto longSig = renderNote(v, 48000.0, 69, 180.5, cfg);
    const m::Signal body(longSig.begin() + 24000, longSig.end());
    const auto trace = m::pitchTraceCents(body, 48000.0, 0.02, 0.0427, 440.0);
    const auto longStats = m::ouStats(trace, 0.02, 0.5, 10.0);
    std::printf("| quantity | value |\n|---|---|\n");
    std::printf("| runs | 24 notes x 10 s |\n| mean (cent) | %s |\n| standard deviation (cent) | %s |\n| autocorrelation "
                "time (s) | %s |\n| lag-1 autocorrelation (20 ms) | %s |\n",
                fmt(s.mean, 3).c_str(), fmt(s.stdDev, 3).c_str(), fmt(s.tauSeconds, 3).c_str(), fmt(s.lag1, 3).c_str());
    std::printf("| PSD slope 0.5-10 Hz (one 180 s run) | %s |\n\n", fmt(longStats.psdSlope, 2).c_str());
}

} // namespace

int main(int argc, char** argv)
{
    const ScopedFlushDenormals noDenormals;
    const std::string which = argc > 1 ? argv[1] : "all";
    if (which == "all" || which == "filters")
        filterTables();
    if (which == "all" || which == "oscillators")
        oscillatorTables();
    if (which == "all" || which == "envelope")
    {
        envelopeTables();
        envelopeTables(true);
    }
    if (which == "all" || which == "drift")
        driftTables();
    return 0;
}
