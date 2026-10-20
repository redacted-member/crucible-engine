// SPDX-License-Identifier: MIT
// Analog voice behaviour (P11): the RC envelope (attack that reaches the peak at the attack time, exponential decay and
// release, sustain through the RC), the retrigger modes and the de-click, velocity and key time scaling, the voice card's
// tolerances (statistics, determinism, and their effect on oscillators, envelopes, filters and the amplifier), the
// seeded noise floor and hum, the digital envelope staying bit-identical, the graph envelope nodes, voice-quad batching
// with the new voice, and the absence of allocations.

#include "AllocCounter.h"
#include "AnalogMeasure.h"
#include "GraphSupport.h"

#include <crucible/AnalogEnvelope.h>
#include <crucible/VoiceCard.h>
#include <crucible/graph/Compiler.h>
#include <crucible/graph/Product.h>

#include <cstring>
#include <numeric>

using namespace crucible;
using namespace crucible::testing;
using crucible::graph::ProductDef;
namespace m = crucible::measure;

namespace
{
constexpr double kFs = 48000.0;

AdsrParams adsrOf(float attack, float decay, float sustain, float release, float hold = 0.0f)
{
    AdsrParams a;
    a.attack = attack;
    a.decay = decay;
    a.sustain = sustain;
    a.release = release;
    a.hold = hold;
    return a;
}

AnalogEnvelope makeEnv(const AdsrParams& p, const EnvShape& s, float timeScale = 1.0f)
{
    AnalogEnvelope e;
    e.prepare(kFs);
    e.setParams(p, s, timeScale, 0.0f);
    return e;
}

EnvShape rcShape(Retrigger r = Retrigger::FromLevel, float overshoot = 0.35f)
{
    EnvShape s;
    s.mode = EnvMode::AnalogRc;
    s.retrigger = r;
    s.overshoot = overshoot;
    return s;
}

// One envelope value per sample.
std::vector<double> ticks(AnalogEnvelope& e, int n)
{
    std::vector<double> out(static_cast<size_t>(n));
    for (auto& v : out)
        v = static_cast<double>(e.tick());
    return out;
}

m::Signal times(size_t from, size_t to)
{
    m::Signal t;
    for (size_t i = from; i < to; ++i)
        t.push_back(static_cast<double>(i + 1) / kFs);
    return t;
}

m::Signal slice(const std::vector<double>& x, size_t from, size_t to)
{
    return m::Signal(x.begin() + static_cast<long>(from), x.begin() + static_cast<long>(to));
}

// Time (seconds) of the first sample at or above level.
double firstAtOrAbove(const std::vector<double>& x, double level)
{
    for (size_t i = 0; i < x.size(); ++i)
        if (x[i] >= level)
            return static_cast<double>(i + 1) / kFs;
    return -1.0;
}

// High-frequency content of a 1 kHz tone under the envelope: RMS of the 6 to 22 kHz band of a window around `centre`,
// in dBFS. The tone's own spectrum and the smooth envelope stay far below the band; a step or a kink in the envelope
// puts its energy there.
double clickDbfs(const std::vector<double>& env, size_t centre)
{
    const size_t n = 8192;
    m::Signal x(n);
    const size_t start = centre - n / 2;
    for (size_t i = 0; i < n; ++i)
        x[i] = env[start + i] * std::sin(2.0 * 3.14159265358979 * 1000.0 * static_cast<double>(start + i) / kFs);
    return m::bandRmsDbfs(x.data(), n, kFs, 6000.0, 22000.0);
}

std::vector<float> sinePatch()
{
    auto v = graphPatch();
    int index = 0;
    for (size_t i = 0; i < WavetableBank::tableNames().size(); ++i)
        if (WavetableBank::tableNames()[i] == "Sine-Saw")
            index = static_cast<int>(i);
    setValue(v, "osc1_table", static_cast<float>(index));
    setValue(v, "osc1_pos", 0.0f);
    setValue(v, "osc1_level", 1.0f);
    setValue(v, "osc2_level", 0.0f);
    setValue(v, "osc3_level", 0.0f);
    setValue(v, "osc1_uni_rand", 0.0f);
    setValue(v, "osc1_uni_voices", 1.0f);
    setValue(v, "filter_on", 0.0f);
    setValue(v, "filter2_on", 0.0f);
    return v;
}

// RMS envelope of the left channel of a note played on a fresh engine (hop 0.5 ms, window of nine periods of A4).
m::EnvelopeTrace noteEnvelope(const std::vector<float>& v, double seconds, const EngineConfig& cfg = {}, int note = 69)
{
    CRUCIBLE_HEAP(SynthEngine, engine, cfg);
    prepare(engine, v);
    engine.noteOn(note, 1.0f);
    const auto r = renderBlocks(engine, static_cast<int>(seconds * kFs), 512, true);
    return m::rmsEnvelope(m::toDouble(r.left), kFs, 9.0 / 440.0, 0.0005);
}

// Time at which the RMS envelope first reaches `fraction` of its plateau (the mean of the last tenth).
double riseTime(const m::EnvelopeTrace& e, double fraction)
{
    const size_t tail = e.level.size() / 10;
    double plateau = 0.0;
    for (size_t i = e.level.size() - tail; i < e.level.size(); ++i)
        plateau += e.level[i] / static_cast<double>(tail);
    return m::timeToLevel(e.timeSeconds, e.level, fraction * plateau);
}

double stddev(const std::vector<double>& x, double* meanOut = nullptr)
{
    double mean = 0.0, var = 0.0;
    for (const double v : x)
        mean += v / static_cast<double>(x.size());
    for (const double v : x)
        var += (v - mean) * (v - mean) / static_cast<double>(x.size());
    if (meanOut != nullptr)
        *meanOut = mean;
    return std::sqrt(var);
}

double maxAbsDifference(const std::vector<float>& a, const std::vector<float>& b)
{
    double d = 0.0;
    for (size_t i = 0; i < std::min(a.size(), b.size()); ++i)
        d = std::max(d, static_cast<double>(std::abs(a[i] - b[i])));
    return d;
}

std::shared_ptr<const graph::CompiledProduct> compileText(const char* text, std::string& error)
{
    ProductDef def;
    if (!graph::productFromText(text, def, error))
        return nullptr;
    return graph::compile(def, error);
}
} // namespace

class AnalogVoiceTests : public UnitTest
{
public:
    AnalogVoiceTests() : UnitTest("AnalogVoice", "Engine") {}

    void runTest() override
    {
        envelopeCircuit();
        retriggerModes();
        scaling();
        cards();
        cardsInTheEngine();
        noiseAndHum();
        batching();
        graphNodes();
        realTime();
    }

private:
    void envelopeCircuit()
    {
        beginTest("Digital is the default and the wrapped envelope equals a bare Adsr bit for bit");
        {
            expectEquals(static_cast<int>(EnvShape{}.mode), static_cast<int>(EnvMode::Digital));
            expectEquals(static_cast<int>(EnvShape{}.retrigger), static_cast<int>(Retrigger::FromLevel));
            const auto v = ParameterMap::defaultValues();
            for (const char* p : {"amp", "menv", "env3", "env4"})
            {
                expectEquals(v[static_cast<size_t>(ParameterMap::indexOf(std::string(p) + "_mode"))], 0.0f);
                expectEquals(v[static_cast<size_t>(ParameterMap::indexOf(std::string(p) + "_retrig"))], 0.0f);
            }
            for (const int sets : {0, 1, 2})
            {
                Adsr bare;
                bare.prepare(kFs);
                AnalogEnvelope wrapped;
                wrapped.prepare(kFs);
                AdsrParams p =
                    sets == 0 ? adsrOf(0.01f, 0.2f, 0.6f, 0.3f)
                              : (sets == 1 ? adsrOf(0.3f, 0.05f, 0.0f, 1.0f, 0.1f) : adsrOf(0.001f, 2.0f, 1.0f, 0.001f));
                p.curve = sets == 2 ? 0.5f : -0.8f;
                p.attackCurve = sets == 1 ? -0.4f : 0.0f;
                bare.setParams(p);
                wrapped.setParams(p, EnvShape{});
                bare.noteOn();
                wrapped.noteOn(0.7f, 50);
                bool same = true;
                for (int i = 0; i < 90000; ++i)
                {
                    if (i == 20000)
                    {
                        bare.noteOff();
                        wrapped.noteOff();
                    }
                    if (i == 30000)
                    {
                        bare.noteOn();
                        wrapped.noteOn(0.2f, 90);
                    }
                    if (i == 33000)
                    {
                        p.sustain = 0.3f;
                        bare.setParams(p);
                        wrapped.setParams(p, EnvShape{});
                    }
                    if (i == 50000)
                    {
                        bare.noteOff();
                        wrapped.noteOff();
                    }
                    const float a = bare.tick(), b = wrapped.tick();
                    same = same && std::memcmp(&a, &b, sizeof(float)) == 0 && bare.isIdle() == wrapped.isIdle();
                }
                expect(same, "set " + std::to_string(sets));
            }
        }

        beginTest("RC attack: reaches the peak at the attack time, fits one exponential toward 1 + overshoot");
        {
            double worstPeak = 0.0, worstTau = 0.0, worstResidual = 0.0;
            for (const float attack : {0.005f, 0.02f, 0.1f, 0.5f, 2.0f})
                for (const float overshoot : {0.1f, 0.35f, 1.0f})
                {
                    auto e = makeEnv(adsrOf(attack, 0.3f, 0.5f, 0.4f), rcShape(Retrigger::FromLevel, overshoot));
                    e.noteOn(1.0f, 60);
                    const int n = static_cast<int>(attack * kFs * 1.5);
                    const auto x = ticks(e, n);
                    const double peak = firstAtOrAbove(x, 1.0);
                    worstPeak = std::max(worstPeak, std::abs(peak / attack - 1.0));
                    const size_t end = static_cast<size_t>(std::lround(peak * kFs));
                    const auto fit = m::fitExponential(times(0, end), slice(x, 0, end));
                    const double tau = attack / std::log((1.0 + overshoot) / overshoot);
                    worstTau = std::max(worstTau, std::abs(fit.tau / tau - 1.0));
                    worstResidual = std::max(worstResidual, fit.residualRms);
                    expect(std::abs(fit.offset - (1.0 + overshoot)) < 0.01, "target " + std::to_string(fit.offset));
                }
            logMessage("RC attack: peak time error " + String(100.0 * worstPeak, 3) + " %, tau error " +
                       String(100.0 * worstTau, 3) + " %, fit residual " + String(100.0 * worstResidual, 4) + " % of range");
            expect(worstPeak < 0.02, "peak time within 2 %");
            expect(worstTau < 0.01, "time constant within 1 %");
            expect(worstResidual < 0.005, "residual below 0.5 %");
        }

        beginTest("RC decay, sustain change and release are exponential with tau = time / ln(100)");
        {
            const double decay = 0.3, release = 0.4;
            auto e = makeEnv(adsrOf(0.1f, static_cast<float>(decay), 0.5f, static_cast<float>(release)), rcShape());
            e.noteOn(1.0f, 60);
            std::vector<double> x = ticks(e, static_cast<int>(1.2 * kFs));
            const size_t peak = static_cast<size_t>(firstAtOrAbove(x, 1.0) * kFs);
            const auto d = m::fitExponential(times(peak, static_cast<size_t>(0.8 * kFs)),
                                             slice(x, peak, static_cast<size_t>(0.8 * kFs)));
            const double tauD = decay / std::log(100.0);
            logMessage("RC decay: tau " + String(d.tau, 5) + " s (nominal " + String(tauD, 5) + "), residual " +
                       String(100.0 * d.residualRms, 4) + " %, settles at " + String(d.offset, 4));
            expect(std::abs(d.tau / tauD - 1.0) < 0.01, "decay tau");
            expect(d.residualRms < 0.005, "decay residual");
            expect(std::abs(d.offset - 0.5) < 0.002, "decay target is the sustain level");
            expect(e.isSustaining() && std::abs(e.value() - 0.5f) < 1.0e-4f, "settled on the sustain level");
            // Raise the sustain while the key is held: the level moves through the same RC.
            e.setParams(adsrOf(0.1f, static_cast<float>(decay), 0.9f, static_cast<float>(release)), rcShape());
            const size_t riseStart = x.size();
            const auto y = ticks(e, static_cast<int>(1.0 * kFs));
            const auto s = m::fitExponential(times(riseStart, riseStart + 24000), slice(y, 0, 24000));
            logMessage("RC sustain change: tau " + String(s.tau, 5) + " s, target " + String(s.offset, 4));
            expect(std::abs(s.tau / tauD - 1.0) < 0.01, "sustain change tau");
            expect(std::abs(s.offset - 0.9) < 0.002, "moves to the new sustain level");
            expect(y[1] - y[0] < (0.9 - 0.5) / (tauD * kFs) * 1.01, "no step: the level moves, it does not jump");
            e.noteOff();
            const auto r = ticks(e, static_cast<int>(2.5 * kFs));
            const size_t span = static_cast<size_t>(0.8 * kFs);
            const auto f = m::fitExponential(times(0, span), slice(r, 0, span));
            const double tauR = release / std::log(100.0);
            logMessage("RC release: tau " + String(f.tau, 5) + " s (nominal " + String(tauR, 5) + "), residual " +
                       String(100.0 * f.residualRms, 4) + " %");
            expect(std::abs(f.tau / tauR - 1.0) < 0.01, "release tau");
            expect(f.residualRms < 0.005, "release residual");
            expect(std::abs(f.offset) < 0.002, "release goes to zero");
            expect(e.isIdle(), "idle after the release");
            const double toSilence = firstAtOrAbove(std::vector<double>(r.rbegin(), r.rend()), 0.0);
            (void)toSilence;
        }

        beginTest("RC envelope through the engine (audio read back): peak at the attack time, one-exponential fits");
        {
            auto v = sinePatch();
            setValue(v, "amp_mode", 1.0f);
            setValue(v, "amp_attack", 0.1f);
            setValue(v, "amp_decay", 0.3f);
            setValue(v, "amp_sustain", 0.5f);
            setValue(v, "amp_release", 0.4f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            engine.noteOn(69, 1.0f);
            auto audio = m::toDouble(renderBlocks(engine, static_cast<int>(1.1 * kFs), 512, true).left);
            engine.noteOff(69);
            const auto tail = m::toDouble(renderBlocks(engine, static_cast<int>(1.5 * kFs), 512, true).left);
            audio.insert(audio.end(), tail.begin(), tail.end());
            const auto env = m::rmsEnvelope(audio, kFs, 3.0 / 440.0, 0.0005);
            double peak = 0.0, peakT = 0.0;
            for (size_t i = 0; i < env.level.size(); ++i)
                if (env.level[i] > peak)
                {
                    peak = env.level[i];
                    peakT = env.timeSeconds[i];
                }
            const auto part = [&](double t0, double t1)
            {
                m::Signal t, y;
                for (size_t i = 0; i < env.level.size(); ++i)
                    if (env.timeSeconds[i] >= t0 && env.timeSeconds[i] <= t1)
                    {
                        t.push_back(env.timeSeconds[i]);
                        y.push_back(env.level[i] / peak);
                    }
                return m::fitExponential(t, y);
            };
            const auto a = part(0.02, peakT), d = part(peakT, 1.0), r = part(1.15, 2.4);
            logMessage("through the engine: peak at " + String(peakT, 4) + " s; attack tau " + String(a.tau, 4) +
                       " s (target " + String(a.offset, 3) + ", residual " + String(100.0 * a.residualRms, 2) +
                       " %); decay tau " + String(d.tau, 4) + " s (residual " + String(100.0 * d.residualRms, 2) +
                       " %); release tau " + String(r.tau, 4) + " s (residual " + String(100.0 * r.residualRms, 2) + " %)");
            expect(std::abs(peakT / 0.1 - 1.0) < 0.02, "peak time within 2 %");
            expect(a.residualRms < 0.005 && d.residualRms < 0.005 && r.residualRms < 0.005, "fit residuals below 0.5 %");
            expect(std::abs(a.tau / (0.1 / std::log(1.35 / 0.35)) - 1.0) < 0.03, "attack time constant within 3 %");
            expect(std::abs(d.tau / (0.3 / std::log(100.0)) - 1.0) < 0.03, "decay time constant within 3 %");
            expect(std::abs(r.tau / (0.4 / std::log(100.0)) - 1.0) < 0.03, "release time constant within 3 %");
        }

        beginTest("RC times hold across sample rates and tolerance scales the times");
        {
            for (const double rate : {44100.0, 96000.0})
            {
                AnalogEnvelope e;
                e.prepare(rate);
                e.setParams(adsrOf(0.1f, 0.3f, 0.5f, 0.4f), rcShape());
                e.noteOn(1.0f, 60);
                int i = 0;
                while (e.tick() < 1.0f && i < 1000000)
                    ++i;
                expect(std::abs(static_cast<double>(i + 1) / rate / 0.1 - 1.0) < 0.002,
                       "peak at the attack time at " + std::to_string(rate));
            }
            auto base = makeEnv(adsrOf(0.1f, 0.3f, 0.5f, 0.4f), rcShape(), 1.0f);
            auto slow = makeEnv(adsrOf(0.1f, 0.3f, 0.5f, 0.4f), rcShape(), 1.08f);
            base.noteOn(1.0f, 60);
            slow.noteOn(1.0f, 60);
            const double tb = firstAtOrAbove(ticks(base, 12000), 1.0), ts = firstAtOrAbove(ticks(slow, 12000), 1.0);
            expect(std::abs(ts / tb - 1.08) < 0.002, "tolerance time scale " + std::to_string(ts / tb));
        }
    }

    void retriggerModes()
    {
        beginTest("retrigger From Level: continuous, the attack starts where the level is (RC reaches the peak sooner)");
        {
            auto e = makeEnv(adsrOf(0.1f, 0.3f, 0.5f, 0.4f), rcShape());
            e.noteOn(1.0f, 60);
            auto x = ticks(e, static_cast<int>(0.7 * kFs));
            e.noteOff();
            const auto rel = ticks(e, 2400);
            e.noteOn(1.0f, 60);
            const auto again = ticks(e, static_cast<int>(0.2 * kFs));
            const double jump = std::abs(again[0] - rel.back());
            const double peak = firstAtOrAbove(again, 1.0);
            const double level = rel.back();
            logMessage("From Level retrigger at " + String(level, 3) + ": first step " + String(jump, 5) + ", peak after " +
                       String(peak, 4) + " s");
            expect(jump < 0.01, "no jump at the retrigger");
            expect(peak > 0.0 && peak < 0.1, "peaks sooner than the full attack time");
            // The expected time from the RC: tau * ln((target - level) / overshoot).
            const double tau = 0.1 / std::log(1.35 / 0.35);
            expect(std::abs(peak - tau * std::log((1.35 - level) / 0.35)) < 0.001, "time matches the circuit");
        }

        beginTest("retrigger Reset: fades the old level out over 0.5 ms, restarts from zero; click below -83 dB");
        {
            double worstReset = -300.0, worstLevel = -300.0, worstDigitalReset = -300.0;
            for (const EnvMode mode : {EnvMode::AnalogRc, EnvMode::Digital})
                for (const float held : {1.0f, 0.5f})
                {
                    for (const bool reset : {false, true})
                    {
                        EnvShape s;
                        s.mode = mode;
                        s.retrigger = reset ? Retrigger::Reset : Retrigger::FromLevel;
                        auto e = makeEnv(adsrOf(0.01f, 0.1f, held, 0.4f), s);
                        e.noteOn(1.0f, 60);
                        auto x = ticks(e, 24000);
                        const size_t at = x.size();
                        e.noteOn(1.0f, 60);
                        const auto y = ticks(e, 24000);
                        x.insert(x.end(), y.begin(), y.end());
                        const double click = clickDbfs(x, at);
                        if (reset)
                        {
                            expect(std::abs(y[0] - x[at - 1]) < 0.02, "the reset starts from the old level");
                            expect(y[30] < 0.1 * held + 0.1, "the old level is gone after 0.5 ms");
                            worstReset = std::max(worstReset, mode == EnvMode::AnalogRc ? click : worstReset);
                            worstDigitalReset =
                                std::max(worstDigitalReset, mode == EnvMode::Digital ? click : worstDigitalReset);
                        }
                        else
                        {
                            worstLevel = std::max(worstLevel, click);
                        }
                        logMessage(String(mode == EnvMode::AnalogRc ? "RC" : "Digital") +
                                   (reset ? " Reset" : " From Level") + " retrigger at " + String(held, 1) + ": click " +
                                   String(click, 1) + " dBFS");
                    }
                }
            expect(worstReset < -83.0, "RC Reset click " + std::to_string(worstReset));
            expect(worstDigitalReset < -83.0, "Digital Reset click " + std::to_string(worstDigitalReset));
            expect(worstLevel < -83.0, "From Level click " + std::to_string(worstLevel));
        }

        beginTest("retrigger Legato: a note-on while the key is held changes nothing; after a note-off it restarts");
        {
            EnvShape s = rcShape(Retrigger::Legato);
            auto a = makeEnv(adsrOf(0.05f, 0.2f, 0.6f, 0.3f), s);
            auto b = makeEnv(adsrOf(0.05f, 0.2f, 0.6f, 0.3f), s);
            a.noteOn(1.0f, 60);
            b.noteOn(1.0f, 60);
            std::vector<double> xa, xb;
            for (int i = 0; i < 20000; ++i)
            {
                if (i == 7000)
                    b.noteOn(0.5f, 64);
                xa.push_back(static_cast<double>(a.tick()));
                xb.push_back(static_cast<double>(b.tick()));
            }
            expect(xa == xb, "legato note-on is ignored while held");
            b.noteOff();
            ticks(b, 800);
            const double before = b.value();
            b.noteOn(1.0f, 60);
            const auto again = ticks(b, 200);
            expect(again[199] > before, "the attack restarts after a release");
            // Digital legato behaves the same way.
            EnvShape d;
            d.retrigger = Retrigger::Legato;
            auto da = makeEnv(adsrOf(0.05f, 0.2f, 0.6f, 0.3f), d), db = makeEnv(adsrOf(0.05f, 0.2f, 0.6f, 0.3f), d);
            da.noteOn(1.0f, 60);
            db.noteOn(1.0f, 60);
            bool same = true;
            for (int i = 0; i < 20000; ++i)
            {
                if (i == 7000)
                    db.noteOn(1.0f, 60);
                same = same && da.tick() == db.tick();
            }
            expect(same, "digital legato");
        }

        beginTest("retrigger ADR: no sustain, the decay runs while the key is held, a note-off releases from the level");
        {
            for (const EnvMode mode : {EnvMode::AnalogRc, EnvMode::Digital})
            {
                EnvShape s;
                s.mode = mode;
                s.retrigger = Retrigger::Adr;
                auto e = makeEnv(adsrOf(0.01f, 0.2f, 0.9f, 0.3f), s);
                e.noteOn(1.0f, 60);
                const auto x = ticks(e, static_cast<int>(0.6 * kFs));
                double maxv = 0.0;
                for (const double v : x)
                    maxv = std::max(maxv, v);
                expect(maxv >= 0.99, "reaches the peak");
                expect(x[static_cast<size_t>(0.3 * kFs)] < 0.5 * maxv, "the sustain setting is ignored: it decays on");
                ticks(e, static_cast<int>(1.5 * kFs));
                expect(e.isIdle(), "ends by itself with the key still held");
                // Released mid-decay: the release starts at the current level.
                auto r = makeEnv(adsrOf(0.01f, 0.5f, 0.9f, 0.3f), s);
                r.noteOn(1.0f, 60);
                ticks(r, 4800);
                const float at = r.value();
                r.noteOff();
                const auto after = ticks(r, 50);
                expect(after[0] <= at && after[0] > 0.9 * at, "release from the level, no jump");
            }
        }
    }

    void scaling()
    {
        beginTest("velocity and key scaling of the times: an octave of key or full velocity halves them at +1");
        {
            const auto peakAt = [](float velocityTime, float keyTime, float velocity, int note)
            {
                EnvShape s = rcShape();
                s.velocityTime = velocityTime;
                s.keyTime = keyTime;
                auto e = makeEnv(adsrOf(0.2f, 0.3f, 0.5f, 0.4f), s);
                e.noteOn(velocity, note);
                return firstAtOrAbove(ticks(e, 48000), 1.0);
            };
            const double base = peakAt(0.0f, 0.0f, 1.0f, 60);
            expect(std::abs(base / 0.2 - 1.0) < 0.002, "unscaled");
            expect(std::abs(peakAt(1.0f, 0.0f, 1.0f, 60) / base - 0.5) < 0.005, "full velocity at +1 halves");
            expect(std::abs(peakAt(1.0f, 0.0f, 0.0f, 60) / base - 2.0) < 0.01, "zero velocity at +1 doubles");
            expect(std::abs(peakAt(-1.0f, 0.0f, 1.0f, 60) / base - 2.0) < 0.01, "full velocity at -1 doubles");
            expect(std::abs(peakAt(0.0f, 1.0f, 1.0f, 72) / base - 0.5) < 0.005, "an octave up at +1 halves");
            expect(std::abs(peakAt(0.0f, 1.0f, 1.0f, 48) / base - 2.0) < 0.01, "an octave down at +1 doubles");
            expect(std::abs(peakAt(0.5f, 0.5f, 0.5f, 60) / base - 1.0) < 0.002, "mid velocity at middle C is neutral");
            // The digital envelope scales the same way.
            EnvShape d;
            d.keyTime = 1.0f;
            auto e = makeEnv(adsrOf(0.2f, 0.3f, 0.5f, 0.4f), d);
            e.noteOn(1.0f, 72);
            const double t = firstAtOrAbove(ticks(e, 48000), 0.999);
            expect(t > 0.09 && t < 0.105, "digital attack " + std::to_string(t));
        }
    }

    void cards()
    {
        beginTest("voice cards: 64 cards are bounded, centred, unit spread, and identical per seed and voice");
        {
            struct Series
            {
                const char* name;
                std::vector<double> v;
            };
            std::vector<Series> series = {{"pitch", {}},     {"level", {}},     {"osc pitch", {}},
                                          {"osc width", {}}, {"osc level", {}}, {"osc curve", {}},
                                          {"filter", {}},    {"env time", {}},  {"env sustain", {}}};
            double bound = 0.0;
            for (int voice = 0; voice < 64; ++voice)
            {
                const VoiceCard c = VoiceCard::make(1234U, voice);
                series[0].v.push_back(c.pitch);
                series[1].v.push_back(c.level);
                series[2].v.push_back(c.osc[0][0].pitch);
                series[3].v.push_back(c.osc[1][2].width);
                series[4].v.push_back(c.osc[2][1].level);
                series[5].v.push_back(c.osc[0][3].curve);
                series[6].v.push_back(c.filterSpread[0]);
                series[7].v.push_back(c.envTime[0]);
                series[8].v.push_back(c.envSustain[1]);
                for (const auto& s : series)
                    bound = std::max(bound, std::abs(s.v.back()));
            }
            for (const auto& s : series)
            {
                double mean = 0.0;
                const double sd = stddev(s.v, &mean);
                logMessage(String("card ") + s.name + ": mean " + String(mean, 3) + ", std " + String(sd, 3));
                expect(std::abs(mean) < 0.4, String(s.name) + " mean " + std::to_string(mean));
                expect(sd > 0.75 && sd < 1.3, String(s.name) + " std " + std::to_string(sd));
            }
            logMessage("largest draw over 64 cards: " + String(bound, 2));
            expect(bound <= 6.0, "bounded");
            // Pooled over every draw of every card the moments are tight.
            std::vector<double> pool;
            for (int voice = 0; voice < 64; ++voice)
            {
                const VoiceCard c = VoiceCard::make(99U, voice);
                for (const auto& slot : c.osc)
                    for (const auto& u : slot)
                        pool.insert(pool.end(), {u.pitch, u.width, u.level, u.curve});
            }
            double mean = 0.0;
            const double sd = stddev(pool, &mean);
            logMessage("pooled oscillator draws: mean " + String(mean, 4) + ", std " + String(sd, 4));
            expect(std::abs(mean) < 0.03 && std::abs(sd - 1.0) < 0.03);

            for (int voice = 0; voice < 8; ++voice)
            {
                const VoiceCard a = VoiceCard::make(7U, voice), b = VoiceCard::make(7U, voice);
                expect(std::memcmp(&a, &b, sizeof(VoiceCard)) == 0, "same seed and voice, same card");
                expect(a.pitch != VoiceCard::make(8U, voice).pitch, "another seed, another card");
                expect(a.pitch != VoiceCard::make(7U, voice + 1).pitch, "another voice, another card");
            }
            // The legacy source of osc.analog is unchanged: the same draws as before, from the note seed.
            std::array<OscCard, kCardUnison> legacy;
            VoiceCard::oscillatorFromSeed(12345U, legacy.data());
            Rng g;
            g.state = (12345U * 3266489917U + 0x9e3779b9U) | 1U;
            bool same = true;
            for (const auto& o : legacy)
                same = same && o.pitch == VoiceCard::draw(g) && o.width == VoiceCard::draw(g) &&
                       o.level == VoiceCard::draw(g) && o.curve == VoiceCard::draw(g);
            expect(same, "legacy oscillator draws");
        }

        beginTest("voice card tolerances: envelope times, amplifier and pitch spread at tolerance 1");
        {
            std::vector<double> timeFactor, gain, pitch;
            for (int voice = 0; voice < 64; ++voice)
            {
                const VoiceCard c = VoiceCard::make(5U, voice);
                timeFactor.push_back(std::log(c.envTimeScale(0, 1.0f)));
                gain.push_back(c.gain(1.0f) - 1.0);
                pitch.push_back(c.pitchSemitones(1.0f) * 100.0);
                expect(c.envTimeScale(2, 0.0f) == 1.0f && c.gain(0.0f) == 1.0f && c.pitchSemitones(0.0f) == 0.0f,
                       "amount 0 is neutral");
            }
            logMessage("tolerance 1 over 64 cards: envelope time std " + String(100.0 * stddev(timeFactor), 2) +
                       " %, amp gain std " + String(100.0 * stddev(gain), 2) + " %, pitch std " + String(stddev(pitch), 2) +
                       " cents");
            expect(std::abs(stddev(timeFactor) - 0.05) < 0.012);
            expect(std::abs(stddev(gain) - 0.02) < 0.005);
            expect(std::abs(stddev(pitch) - 5.0) < 1.2);
        }
    }

    // Pitch deviation (cents) of every note of a chord of sines, one voice each.
    static std::vector<double> chordCents(const std::vector<float>& v, const EngineConfig& cfg)
    {
        CRUCIBLE_HEAP(SynthEngine, engine, cfg);
        prepare(engine, v);
        std::vector<int> notes;
        for (int k = 0; k < 16; ++k)
            notes.push_back(40 + 3 * k);
        for (const int n : notes)
            engine.noteOn(n, 1.0f);
        renderBlocks(engine, 24000);
        const auto r = renderBlocks(engine, 1 << 17, 512, true);
        const m::Signal x = m::toDouble(r.left);
        // Several sines in one render: look for each peak in a window of +-60 cents around its note.
        const m::Signal w = m::makeWindow(m::Window::Harris7, x.size());
        std::vector<double> cents;
        for (const int n : notes)
        {
            const double hz = 440.0 * std::pow(2.0, (n - 69) / 12.0);
            const auto magnitude = [&](double c)
            { return std::abs(m::windowedDtft(x.data(), w.data(), x.size(), hz * std::pow(2.0, c / 1200.0), kFs)); };
            double best = -60.0, bestMag = 0.0;
            for (double c = -60.0; c <= 60.0; c += 2.0)
                if (const double a = magnitude(c); a > bestMag)
                {
                    bestMag = a;
                    best = c;
                }
            double lo = best - 2.0, hi = best + 2.0;
            for (int it = 0; it < 40; ++it)
            {
                const double c1 = lo + (hi - lo) / 3.0, c2 = hi - (hi - lo) / 3.0;
                if (magnitude(c1) < magnitude(c2))
                    lo = c1;
                else
                    hi = c2;
            }
            cents.push_back(0.5 * (lo + hi));
        }
        return cents;
    }

    void cardsInTheEngine()
    {
        beginTest(
            "voice card in the engine: default is the legacy voice; pitch spread, amp gain, envelope time follow the card");
        {
            auto v = sinePatch();
            setValue(v, "voice_poly", 16.0f);
            const auto legacy = chordCents(v, EngineConfig{});
            double worst = 0.0;
            for (const double c : legacy)
                worst = std::max(worst, std::abs(c));
            expect(worst < 0.1, "no tolerance by default: " + std::to_string(worst) + " cents");
            setValue(v, "voice_card_osc", 1.0f);
            const auto cents = chordCents(v, EngineConfig{});
            double mean = 0.0;
            const double sd = stddev(cents, &mean);
            logMessage("voice card, oscillator tolerance 1: pitch std over 16 voices " + String(sd, 2) + " cents (mean " +
                       String(mean, 2) + ")");
            expect(sd > 2.5 && sd < 8.0, "pitch std " + std::to_string(sd));
            EngineConfig other;
            other.seed = 4242U;
            const auto cents2 = chordCents(v, other);
            const auto cents3 = chordCents(v, EngineConfig{});
            expect(cents == cents3, "identical per seed");
            expect(cents != cents2, "another seed, other cards");
            setValue(v, "voice_card_osc", 0.5f);
            double half = 0.0;
            half = stddev(chordCents(v, EngineConfig{}));
            expect(std::abs(half / sd - 0.5) < 0.05, "tolerance scales the spread: " + std::to_string(half / sd));
        }

        beginTest("voice card: amplifier gain and envelope times of voice 0 follow the card; fixed from note to note");
        {
            auto v = sinePatch();
            setValue(v, "amp_attack", 0.3f);
            setValue(v, "amp_sustain", 1.0f);
            setValue(v, "amp_mode", 1.0f);
            const auto base = noteEnvelope(v, 1.2);
            setValue(v, "voice_card_env", 1.0f);
            setValue(v, "voice_card_level", 1.0f);
            const auto carded = noteEnvelope(v, 1.2);
            const VoiceCard c = VoiceCard::make(0U, 0);
            const double expectedTime = c.envTimeScale(0, 1.0f);
            const double ratio = riseTime(carded, 0.999) / riseTime(base, 0.999);
            logMessage("voice 0 card: envelope time factor measured " + String(ratio, 4) + ", card " +
                       String(expectedTime, 4));
            expect(std::abs(ratio / expectedTime - 1.0) < 0.004, "envelope time");
            double basePeak = 0.0, cardPeak = 0.0;
            for (size_t i = 0; i < base.level.size(); ++i)
            {
                basePeak = std::max(basePeak, base.level[i]);
                cardPeak = std::max(cardPeak, carded.level[i]);
            }
            expect(std::abs(cardPeak / basePeak / c.gain(1.0f) - 1.0) < 0.002,
                   "amplifier gain " + std::to_string(cardPeak / basePeak) + " vs " + std::to_string(c.gain(1.0f)));
            // The same voice plays two notes: the card, unlike a per-note seed, does not change.
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            const auto level = [&]
            {
                engine.noteOn(69, 1.0f);
                const auto r = renderBlocks(engine, 24000, 512, true);
                engine.noteOff(69);
                renderBlocks(engine, 48000);
                return m::toDouble(r.left);
            };
            const auto a = level(), b = level();
            expect(m::rmsOf(a.data(), a.size()) == m::rmsOf(b.data(), b.size()) ||
                       std::abs(m::rmsOf(a.data(), a.size()) / m::rmsOf(b.data(), b.size()) - 1.0) < 1.0e-6,
                   "the same voice sounds the same on the next note");
        }

        beginTest("osc.analog takes its tolerances from the voice card (unified source): the same offset note after note");
        {
            auto v = sinePatch();
            setValue(v, "osc1_type", 5.0f);
            setValue(v, "osc1_an_wave", 3.0f);
            const auto trace = [&](float cardOsc, float anTol, std::uint32_t seed)
            {
                auto values = v;
                setValue(values, "voice_card_osc", cardOsc);
                setValue(values, "osc1_an_tol", anTol);
                EngineConfig cfg;
                cfg.seed = seed;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                prepare(engine, values);
                std::vector<double> cents;
                for (int note = 0; note < 4; ++note)
                {
                    engine.noteOn(57, 1.0f);
                    renderBlocks(engine, 12000);
                    const auto r = renderBlocks(engine, 1 << 15, 512, true);
                    const m::Signal x = m::toDouble(r.left);
                    cents.push_back(m::centsBetween(m::estimateFrequency(x.data(), x.size(), kFs, 220.0), 220.0));
                    engine.noteOff(57);
                    renderBlocks(engine, 48000);
                }
                return cents;
            };
            const auto perNote = trace(0.0f, 1.0f, 0U);
            const auto perVoice = trace(1.0f, 0.0f, 0U);
            logMessage("osc.analog tolerance 1 per note seed: " + String(perNote[0], 2) + " " + String(perNote[1], 2) + " " +
                       String(perNote[2], 2) +
                       " cents; "
                       "voice card: " +
                       String(perVoice[0], 2) + " " + String(perVoice[1], 2) + " " + String(perVoice[2], 2) + " cents");
            expect(std::abs(perNote[0] - perNote[1]) > 0.2 || std::abs(perNote[1] - perNote[2]) > 0.2,
                   "per note: each note another offset");
            expect(std::abs(perVoice[0] - perVoice[1]) < 0.05 && std::abs(perVoice[1] - perVoice[2]) < 0.05 &&
                       std::abs(perVoice[2] - perVoice[3]) < 0.05,
                   "voice card: the same offset every note");
            const VoiceCard c = VoiceCard::make(0U, 0);
            expect(std::abs(perVoice[0] - c.osc[0][0].pitch * 5.0) < 0.15,
                   "equals the card's draw: " + std::to_string(perVoice[0]) + " vs " +
                       std::to_string(c.osc[0][0].pitch * 5.0));
            expect(trace(1.0f, 0.0f, 99U)[0] != perVoice[0], "another engine seed, another card");
        }

        beginTest("voice card filter tolerance reaches the analog models and changes the sound within bounds");
        {
            auto v = sinePatch();
            setValue(v, "osc1_table", 1.0f);
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_type", 17.0f);
            setValue(v, "filter_cutoff", 1500.0f);
            setValue(v, "filter_res", 0.6f);
            const auto render = [&](float amount, bool batched)
            {
                auto values = v;
                setValue(values, "voice_card_filter", amount);
                EngineConfig cfg;
                cfg.batchAnalogFilters = batched;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                prepare(engine, values);
                engine.noteOn(45, 1.0f);
                return renderBlocks(engine, 12000, 512, true);
            };
            for (const bool batched : {true, false})
            {
                const auto off = render(0.0f, batched), on = render(1.0f, batched);
                const double e0 = m::rmsOf(m::toDouble(off.left).data(), off.left.size()),
                             e1 = m::rmsOf(m::toDouble(on.left).data(), on.left.size());
                logMessage(String(batched ? "banked" : "single") + ": filter RMS " + String(e0, 4) + " -> " + String(e1, 4) +
                           " with card tolerance 1");
                expect(on.finite && maxAbsDifference(off.left, on.left) > 1.0e-4, "the tolerance changes the filter");
                expect(std::abs(e1 / e0 - 1.0) < 0.5, "bounded");
            }
        }
    }

    void noiseAndHum()
    {
        beginTest("noise floor and hum are off by default: an idle engine is exactly silent");
        {
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, ParameterMap::defaultValues());
            const auto r = renderBlocks(engine, 48000, 512, true);
            expect(r.peak == 0.0f);
        }

        beginTest("noise floor level rows: -120 to -60 dBFS RMS, white, independent per channel, seeded");
        {
            for (const float db : {-120.0f, -108.0f, -96.0f, -84.0f, -72.0f, -60.0f})
            {
                auto v = ParameterMap::defaultValues();
                setValue(v, "voice_noise_on", 1.0f);
                setValue(v, "voice_noise_db", db);
                CRUCIBLE_HEAP(SynthEngine, engine);
                prepare(engine, v);
                const auto r = renderBlocks(engine, 96000, 480, true);
                const m::Signal l = m::toDouble(r.left, 2000), rr = m::toDouble(r.right, 2000);
                const double level = m::rmsDbfs(l), levelR = m::rmsDbfs(rr);
                double cross = 0.0;
                for (size_t i = 0; i < l.size(); ++i)
                    cross += l[i] * rr[i];
                cross /= static_cast<double>(l.size()) * m::rmsOf(l.data(), l.size()) * m::rmsOf(rr.data(), rr.size());
                const double low = m::bandRmsDbfs(l.data(), 1 << 16, kFs, 100.0, 4000.0),
                             high = m::bandRmsDbfs(l.data(), 1 << 16, kFs, 12000.0, 20000.0);
                logMessage("noise " + String(db, 0) + " dBFS: measured L " + String(level, 2) + ", R " + String(levelR, 2) +
                           ", L/R correlation " + String(cross, 3));
                expect(std::abs(level - db) < 0.1 && std::abs(levelR - db) < 0.1, "level " + std::to_string(level));
                expect(std::abs(cross) < 0.02, "independent channels");
                expect(std::abs((low - 10.0 * std::log10(3900.0)) - (high - 10.0 * std::log10(8000.0))) < 0.6,
                       "white spectrum");
            }
            auto v = ParameterMap::defaultValues();
            setValue(v, "voice_noise_on", 1.0f);
            const auto render = [&](std::uint32_t seed)
            {
                EngineConfig cfg;
                cfg.seed = seed;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                prepare(engine, v);
                return renderBlocks(engine, 4800, 512, true);
            };
            expect(render(1U).left == render(1U).left, "identical per seed");
            expect(render(1U).left != render(2U).left, "another seed, other noise");
        }

        beginTest("hum level rows: 50 and 60 Hz with harmonics, the same on both channels");
        {
            for (const float db : {-120.0f, -96.0f, -84.0f, -60.0f})
                for (const float hz : {0.0f, 1.0f})
                {
                    auto v = ParameterMap::defaultValues();
                    setValue(v, "voice_hum_on", 1.0f);
                    setValue(v, "voice_hum_db", db);
                    setValue(v, "voice_hum_hz", hz);
                    CRUCIBLE_HEAP(SynthEngine, engine);
                    prepare(engine, v);
                    const auto r = renderBlocks(engine, 96000, 480, true);
                    const m::Signal l = m::toDouble(r.left, 4800);
                    const double level = m::rmsDbfs(l);
                    const double f0 = hz > 0.5f ? 60.0 : 50.0;
                    const double est = m::estimateFrequency(l.data(), 1 << 16, kFs, f0);
                    const double fundamental = m::bandRmsDbfs(l.data(), 1 << 16, kFs, f0 - 3.0, f0 + 3.0);
                    const double second = m::bandRmsDbfs(l.data(), 1 << 16, kFs, 2.0 * f0 - 3.0, 2.0 * f0 + 3.0);
                    logMessage("hum " + String(db, 0) + " dBFS at " + String(f0, 0) + " Hz: RMS " + String(level, 2) +
                               " dBFS, frequency " + String(est, 2) + " Hz, 2nd harmonic " +
                               String(second - fundamental, 1) + " dB");
                    expect(std::abs(level - db) < 0.1, "level " + std::to_string(level));
                    expect(std::abs(est - f0) < 0.1, "frequency");
                    expect(std::abs((second - fundamental) - m::amplitudeToDb(0.5)) < 0.5,
                           "second harmonic at half amplitude");
                    expect(r.left == r.right, "the same on both channels");
                }
        }

        beginTest("noise and hum add under playing voices and a level change ramps (no step)");
        {
            auto v = sinePatch();
            CRUCIBLE_HEAP(SynthEngine, plain);
            prepare(plain, v);
            plain.noteOn(60, 1.0f);
            const auto a = renderBlocks(plain, 9600, 480, true);
            setValue(v, "voice_noise_on", 1.0f);
            setValue(v, "voice_noise_db", -60.0f);
            CRUCIBLE_HEAP(SynthEngine, noisy);
            prepare(noisy, v);
            noisy.noteOn(60, 1.0f);
            const auto b = renderBlocks(noisy, 9600, 480, true);
            double diff = 0.0;
            for (size_t i = 0; i < a.left.size(); ++i)
                diff += static_cast<double>(b.left[i] - a.left[i]) * (b.left[i] - a.left[i]) /
                        static_cast<double>(a.left.size());
            expect(std::abs(10.0 * std::log10(diff) + 60.0) < 0.5,
                   "added noise level " + std::to_string(10.0 * std::log10(diff)));
            // Switching on mid-stream: the first samples ramp from zero.
            CRUCIBLE_HEAP(SynthEngine, engine);
            auto idle = ParameterMap::defaultValues();
            prepare(engine, idle);
            renderBlocks(engine, 4800);
            setValue(idle, "voice_noise_on", 1.0f);
            setValue(idle, "voice_noise_db", -60.0f);
            engine.setParameterValues(idle.data(), static_cast<int>(idle.size()));
            const auto r = renderBlocks(engine, 64, 64, true);
            expect(std::abs(r.left[0]) < 0.0004, "first sample " + std::to_string(r.left[0]));
        }
    }

    void batching()
    {
        beginTest("voice-quad batching stays engaged and equivalent with analog envelopes, cards and retrigger modes");
        {
            auto v = sinePatch();
            setValue(v, "osc1_table", 1.0f);
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_type", 17.0f);
            setValue(v, "filter_cutoff", 1500.0f);
            setValue(v, "filter_res", 0.5f);
            setValue(v, "filter_env", 0.4f);
            setValue(v, "amp_mode", 1.0f);
            setValue(v, "amp_retrig", 1.0f);
            setValue(v, "amp_vel_time", 0.5f);
            setValue(v, "menv_mode", 1.0f);
            setValue(v, "menv_decay", 0.2f);
            setValue(v, "voice_card_osc", 1.0f);
            setValue(v, "voice_card_env", 1.0f);
            setValue(v, "voice_card_level", 1.0f);
            const auto play = [&](bool batched, std::uint64_t* quadBlocks)
            {
                EngineConfig cfg;
                cfg.batchAnalogFilters = batched;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                prepare(engine, v);
                Rendered all;
                std::vector<float> l(512), r(512);
                static const int sizes[] = {37, 100, 64, 5, 211, 29, 32, 160};
                for (int call = 0; call < 60; ++call)
                {
                    if (call == 0 || call == 2 || call == 3 || call == 5)
                        engine.noteOn(48 + 5 * call, 0.8f);
                    if (call == 14)
                        engine.noteOff(58);
                    if (call == 20)
                        engine.noteOn(48, 1.0f);
                    const int n = sizes[call % 8];
                    engine.render(l.data(), r.data(), n);
                    all.left.insert(all.left.end(), l.begin(), l.begin() + n);
                    all.right.insert(all.right.end(), r.begin(), r.begin() + n);
                }
                *quadBlocks = engine.batchedFilterBlocks();
                return all;
            };
            std::uint64_t quadOn = 0, quadOff = 0;
            const auto a = play(true, &quadOn), b = play(false, &quadOff);
            const double peak = *std::max_element(a.left.begin(), a.left.end());
            const double diff = std::max(maxAbsDifference(a.left, b.left), maxAbsDifference(a.right, b.right));
            logMessage("batched vs voice by voice: largest difference " + String(diff) + " at peak " + String(peak));
            expect(peak > 1.0e-2);
            expect(diff <= 1.0e-5 * std::max(peak, 1.0), "equivalent " + std::to_string(diff));
            // The batched and single paths can agree bit for bit (x86 SSE, MSVC), so engagement is counted, not inferred.
            expect(quadOn > 0 && quadOff == 0, "the bank is engaged only when batching is on");
        }
    }

    void graphNodes()
    {
        beginTest("graph envelope nodes: env.analog reaches the peak at the attack time, env.adr ends by itself; the voice "
                  "card scales them");
        {
            const char* text = R"({
                "format": "crucible-product", "version": 1, "id": "test.analog",
                "voice": { "nodes": [
                    { "id": "osc", "type": "osc.sub", "slot": 1 },
                    { "id": "env", "type": "env.analog", "params": { "attack": 0.2, "sustain": 1.0, "mode": 1 } },
                    { "id": "vca", "type": "amp.vca" },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "osc.out", "to": "vca.in" },
                    { "from": "env.out", "to": "vca.gain" },
                    { "from": "vca.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 1.0 } })";
            std::string error;
            const auto compiled = compileText(text, error);
            expect(compiled != nullptr, error);
            if (!compiled)
                return;
            const auto envelopeOf = [&](float cardEnv)
            {
                EngineConfig cfg;
                cfg.compiled = compiled;
                CRUCIBLE_HEAP(SynthEngine, engine, cfg);
                auto values = compiled->defaultValues();
                const int ci = engine.model().indexOf("voice_card_env");
                if (ci >= 0)
                    values[static_cast<size_t>(ci)] = cardEnv;
                engine.prepare(kRate);
                engine.setParameterValues(values.data(), static_cast<int>(values.size()));
                engine.noteOn(72, 1.0f); // the sub oscillator sounds an octave down: 261.6 Hz
                const auto r = renderBlocks(engine, 48000, 480, true);
                return m::rmsEnvelope(m::toDouble(r.left), kFs, 6.0 / 261.6256, 0.0005);
            };
            const auto plain = envelopeOf(0.0f);
            const double t99 = riseTime(plain, 0.999);
            logMessage("env.analog node: reaches 99.9 % of the plateau at " + String(t99, 4) + " s for an attack of 0.2 s");
            // The RMS window lags a concave rise by a few milliseconds; the exact timing is measured on the envelope itself
            // above.
            expect(t99 > 0.19 && t99 < 0.21, "peak at the attack time");
            const int ci = ModuleRegistry::instance().parameters().indexOf("voice_card_env");
            expect(ci >= 0, "voice.analog is registered");
            // The compiled product carries the module too (same registration path); with the card on the node's times scale.
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, probe, cfg);
            if (probe.model().indexOf("voice_card_env") >= 0)
            {
                const auto carded = envelopeOf(1.0f);
                std::size_t nodeIndex = 0;
                for (std::size_t i = 0; i < compiled->layers[0].nodes.size(); ++i)
                    if (compiled->layers[0].nodes[i].id == "env")
                        nodeIndex = i;
                const double expected = VoiceCard::make(0U, 0).envTimeScale(static_cast<int>(nodeIndex % kCardEnvs), 1.0f);
                const double ratio = riseTime(carded, 0.999) / t99;
                logMessage("graph voice card: envelope time factor measured " + String(ratio, 4) + ", card " +
                           String(expected, 4));
                expect(std::abs(ratio / expected - 1.0) < 0.004, "card scales the node's times");
            }
            else
            {
                logMessage("compiled product has no voice_card_env parameter");
            }
        }

        beginTest("env.adr node: attack, decay and release without a sustain; the voice ends with the key still held");
        {
            const char* text = R"({
                "format": "crucible-product", "version": 1, "id": "test.adr",
                "voice": { "nodes": [
                    { "id": "osc", "type": "osc.noise", "slot": 1 },
                    { "id": "env", "type": "env.adr", "params": { "attack": 0.01, "decay": 0.2, "release": 0.2 } },
                    { "id": "vca", "type": "amp.vca" },
                    { "id": "out", "type": "mix.out" } ],
                  "edges": [
                    { "from": "osc.out", "to": "vca.in" },
                    { "from": "env.out", "to": "vca.gain" },
                    { "from": "vca.out", "to": "out.in" } ] },
                "defaults": { "osc1_level": 1.0 } })";
            std::string error;
            const auto compiled = compileText(text, error);
            expect(compiled != nullptr, error);
            if (!compiled)
                return;
            EngineConfig cfg;
            cfg.compiled = compiled;
            CRUCIBLE_HEAP(SynthEngine, engine, cfg);
            engine.prepare(kRate);
            const auto values = compiled->defaultValues();
            engine.setParameterValues(values.data(), static_cast<int>(values.size()));
            engine.noteOn(60, 1.0f);
            const auto early = renderBlocks(engine, 4800, 480, true);
            expect(early.peak > 0.01f, "sounds");
            renderBlocks(engine, 48000);
            expectEquals(engine.activeVoiceCount(), 0, "the voice ended by itself");
        }
    }

    void realTime()
    {
        beginTest(
            "no allocation with 16 voices, analog envelopes, retrigger modes, cards, noise and hum, and parameter changes");
        {
            auto v = sinePatch();
            for (const char* p : {"amp", "menv", "env3", "env4"})
            {
                setValue(v, std::string(p) + "_mode", 1.0f);
                setValue(v, std::string(p) + "_vel_time", 0.4f);
                setValue(v, std::string(p) + "_key_time", -0.3f);
            }
            setValue(v, "voice_card_osc", 1.0f);
            setValue(v, "voice_card_env", 1.0f);
            setValue(v, "voice_card_level", 1.0f);
            setValue(v, "voice_card_filter", 1.0f);
            setValue(v, "voice_noise_on", 1.0f);
            setValue(v, "voice_hum_on", 1.0f);
            setValue(v, "filter_on", 1.0f);
            setValue(v, "filter_type", 18.0f);
            CRUCIBLE_HEAP(SynthEngine, engine);
            prepare(engine, v);
            for (int n = 0; n < 16; ++n)
                engine.noteOn(36 + n * 3, 0.9f);
            std::vector<float> l(256), r(256);
            for (int i = 0; i < 10; ++i)
                engine.render(l.data(), r.data(), 256);
            gAllocations = 0;
            gTrackAllocations = true;
            for (int round = 0; round < 12; ++round)
            {
                setValue(v, "amp_retrig", static_cast<float>(round % 4));
                setValue(v, "menv_retrig", static_cast<float>((round + 1) % 4));
                setValue(v, "amp_mode", static_cast<float>(round % 2));
                setValue(v, "voice_card_env", round % 3 == 0 ? 0.0f : 1.0f);
                setValue(v, "voice_noise_on", static_cast<float>(round % 2));
                engine.setParameterValues(v.data(), static_cast<int>(v.size()));
                for (int i = 0; i < 8; ++i)
                {
                    if (i % 3 == 0)
                    {
                        engine.noteOff(36 + ((round + i) % 16) * 3);
                        engine.noteOn(40 + i + round, 0.8f);
                    }
                    engine.render(l.data(), r.data(), i % 2 == 0 ? 256 : 77);
                }
            }
            gTrackAllocations = false;
            expectEquals(static_cast<int>(gAllocations), 0, "allocations");
        }
    }
};

static AnalogVoiceTests analogVoiceTests;
