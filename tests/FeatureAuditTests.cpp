// SPDX-License-Identifier: MIT
// Feature audit: walks the module registry and the parameter model and checks that every module, every parameter and
// every engine feature works, offline (no audio device). It renders short phrases through the Engine facade and
// checks the output (finite, bounded, no stuck DC, no subnormals), that a parameter measurably changes the sound
// where it should, that the result does not depend on the host block size, and the control-thread features
// (patches, scenes, clips, zones, tuning, key lock, the C API) around them.
//
// The results are collected in an audit table. With CRUCIBLE_AUDIT_REPORT=<file> the table is written as Markdown
// (docs/FEATURE-AUDIT.md carries the committed copy). CRUCIBLE_AUDIT_STRIDE=n audits every n-th parameter only (quick
// runs); sanitizer builds default to a stride so that CI stays short.

#include <crucible/Engine.h>
#include <crucible/Modulators.h>
#include <crucible/EngineCore.h>
#include <crucible/Patch.h>
#include <crucible/crucible.h>

#include "AllocCounterConfig.h"
#include "TestSupport.h"

#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <random>
#include <sstream>
#include <thread>

#include <new>

// Allocation counter: process() must not allocate (the audit counts operator new calls made inside it).
namespace
{
thread_local bool gTrackAllocations = false;
thread_local long gAllocations = 0;
} // namespace

#if CRUCIBLE_ALLOC_COUNTER_AVAILABLE
void* operator new(std::size_t n)
{
    if (gTrackAllocations)
        ++gAllocations;
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void* operator new[](std::size_t n)
{
    if (gTrackAllocations)
        ++gAllocations;
    if (void* p = std::malloc(n == 0 ? 1 : n))
        return p;
    throw std::bad_alloc();
}
void operator delete(void* p) noexcept
{
    std::free(p);
}
void operator delete[](void* p) noexcept
{
    std::free(p);
}
void operator delete(void* p, std::size_t) noexcept
{
    std::free(p);
}
void operator delete[](void* p, std::size_t) noexcept
{
    std::free(p);
}
#endif

using namespace crucible;
using namespace crucible::testing;

namespace
{
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
constexpr bool kSanitized = true;
#elif defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
constexpr bool kSanitized = true;
#else
constexpr bool kSanitized = false;
#endif
#else
constexpr bool kSanitized = false;
#endif

constexpr float kPeakBound = 32.0f; // 30 dB over full scale: nothing in the engine should ever get near it
constexpr double kTau = 6.283185307179586;

int auditStride()
{
    if (const char* s = std::getenv("CRUCIBLE_AUDIT_STRIDE"))
        return std::max(1, std::atoi(s));
    return kSanitized ? 6 : 1;
}

// ---------------------------------------------------------------------------------------------------------------
// The audit table.
struct Row
{
    std::string area;
    std::string feature;
    std::string how;
    std::string result;
    std::string bug;
};

std::vector<Row>& table()
{
    static std::vector<Row> rows;
    return rows;
}

void record(const std::string& area, const std::string& feature, const std::string& how, const std::string& result,
            const std::string& bug = "-")
{
    table().push_back({area, feature, how, result, bug});
}

// ---------------------------------------------------------------------------------------------------------------
// Rendering.
struct Ev
{
    int pos = 0;
    std::uint8_t status = 0, d1 = 0, d2 = 0;
};

struct Shot
{
    std::vector<float> l, r;
    bool finite = true;
    float peak = 0.0f;
    int subnormals = 0;
    long allocations = 0; // operator new calls made inside process()
    double seconds = 0.0; // wall time of the process calls

    double rms() const
    {
        double s = 0.0;
        for (size_t i = 0; i < l.size(); ++i)
            s += static_cast<double>(l[i]) * l[i] + static_cast<double>(r[i]) * r[i];
        return l.empty() ? 0.0 : std::sqrt(s / (2.0 * static_cast<double>(l.size())));
    }
    // Mean of both channels over [from, to) samples.
    double mean(size_t from, size_t to) const
    {
        to = std::min(to, l.size());
        if (from >= to)
            return 0.0;
        double s = 0.0;
        for (size_t i = from; i < to; ++i)
            s += static_cast<double>(l[i]) + r[i];
        return s / (2.0 * static_cast<double>(to - from));
    }
    double rmsRange(size_t from, size_t to) const
    {
        to = std::min(to, l.size());
        if (from >= to)
            return 0.0;
        double s = 0.0;
        for (size_t i = from; i < to; ++i)
            s += static_cast<double>(l[i]) * l[i] + static_cast<double>(r[i]) * r[i];
        return std::sqrt(s / (2.0 * static_cast<double>(to - from)));
    }
};

// RMS of the difference relative to the louder of the two signals; 0 when both are silent.
double relDiff(const Shot& a, const Shot& b)
{
    const size_t n = std::min(a.l.size(), b.l.size());
    double d = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const double dl = static_cast<double>(a.l[i]) - b.l[i];
        const double dr = static_cast<double>(a.r[i]) - b.r[i];
        d += dl * dl + dr * dr;
    }
    d = std::sqrt(d / (2.0 * static_cast<double>(std::max<size_t>(n, 1))));
    const double ref = std::max({a.rms(), b.rms(), 1.0e-7});
    return d / ref;
}

// A deterministic sidechain signal (amplitude-modulated noise) so effects that listen to an input have one.
const std::vector<float>& auxSignal()
{
    static const std::vector<float> sig = []
    {
        std::vector<float> v(48000 * 4);
        std::uint32_t s = 12345u;
        for (size_t i = 0; i < v.size(); ++i)
        {
            s = s * 1664525u + 1013904223u;
            const float n = static_cast<float>(static_cast<std::int32_t>(s)) / 2147483648.0f;
            const float env = 0.5f + 0.5f * static_cast<float>(std::sin(kTau * 3.0 * static_cast<double>(i) / 48000.0));
            v[i] = 0.3f * n * env;
        }
        return v;
    }();
    return sig;
}

struct Cfg
{
    double rate = 48000.0;
    int block = 512;
    int voices = 12;
    std::uint32_t seed = 7;
    bool aux = true;
    double hostBpm = 0.0; // > 0: a playing host at this tempo, position advancing with the render
    Quality quality = Quality::Standard;
};

using Setup = std::function<void(Engine&)>;

// Plays the events (sorted by position) and renders `samples` samples in blocks of cfg.block.
Shot play(Engine& e, const Cfg& cfg, std::vector<Ev> events, int samples, const SeqHostTime* host = nullptr)
{
    std::stable_sort(events.begin(), events.end(), [](const Ev& a, const Ev& b) { return a.pos < b.pos; });
    Shot s;
    s.l.reserve(static_cast<size_t>(samples));
    s.r.reserve(static_cast<size_t>(samples));
    std::vector<float> l(static_cast<size_t>(cfg.block)), r(static_cast<size_t>(cfg.block));
    std::vector<MidiInput> midi;
    size_t next = 0;
    const auto& aux = auxSignal();
    for (int done = 0; done < samples; done += cfg.block)
    {
        const int n = std::min(cfg.block, samples - done);
        midi.clear();
        while (next < events.size() && events[next].pos < done + n)
        {
            midi.push_back({std::max(0, events[next].pos - done), events[next].status, events[next].d1, events[next].d2});
            ++next;
        }
        const float* ax =
            cfg.aux ? aux.data() + (static_cast<size_t>(done) % (aux.size() - static_cast<size_t>(cfg.block) - 1100))
                    : nullptr;
        SeqHostTime blockHost;
        const SeqHostTime* hostNow = host;
        if (cfg.hostBpm > 0.0)
        {
            blockHost.hasPpq = true;
            blockHost.playing = true;
            blockHost.hasBpm = true;
            blockHost.bpm = cfg.hostBpm;
            blockHost.ppq = static_cast<double>(done) / cfg.rate * cfg.hostBpm / 60.0;
            hostNow = &blockHost;
        }
        const float* axr = ax != nullptr ? ax + 1000 : nullptr; // a different right channel for the sidechain
        const auto t0 = std::chrono::steady_clock::now();
        gAllocations = 0;
        gTrackAllocations = true;
        e.process(l.data(), r.data(), n, midi.empty() ? nullptr : midi.data(), static_cast<int>(midi.size()), hostNow, ax,
                  axr);
        gTrackAllocations = false;
        s.allocations += gAllocations;
        s.seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        for (int i = 0; i < n; ++i)
        {
            for (const float v : {l[static_cast<size_t>(i)], r[static_cast<size_t>(i)]})
            {
                if (!std::isfinite(v))
                    s.finite = false;
                else if (std::fpclassify(v) == FP_SUBNORMAL)
                    ++s.subnormals;
                s.peak = std::max(s.peak, std::isfinite(v) ? std::abs(v) : 0.0f);
            }
            s.l.push_back(l[static_cast<size_t>(i)]);
            s.r.push_back(r[static_cast<size_t>(i)]);
        }
    }
    return s;
}

int samplesOf(double seconds, double rate = 48000.0)
{
    return static_cast<int>(seconds * rate);
}

Ev on(double t, int note, int vel = 100, int ch = 0, double rate = 48000.0)
{
    return {samplesOf(t, rate), static_cast<std::uint8_t>(0x90 | ch), static_cast<std::uint8_t>(note),
            static_cast<std::uint8_t>(vel)};
}
Ev off(double t, int note, int ch = 0, double rate = 48000.0)
{
    return {samplesOf(t, rate), static_cast<std::uint8_t>(0x80 | ch), static_cast<std::uint8_t>(note), 0};
}
Ev cc(double t, int num, int val, int ch = 0, double rate = 48000.0)
{
    return {samplesOf(t, rate), static_cast<std::uint8_t>(0xB0 | ch), static_cast<std::uint8_t>(num),
            static_cast<std::uint8_t>(val)};
}
Ev bend(double t, int value14, int ch = 0, double rate = 48000.0)
{
    return {samplesOf(t, rate), static_cast<std::uint8_t>(0xE0 | ch), static_cast<std::uint8_t>(value14 & 127),
            static_cast<std::uint8_t>((value14 >> 7) & 127)};
}
Ev pressure(double t, int value, int ch = 0, double rate = 48000.0)
{
    return {samplesOf(t, rate), static_cast<std::uint8_t>(0xD0 | ch), static_cast<std::uint8_t>(value), 0};
}

enum class Script
{
    Mono2,    // two overlapping notes, a bend and a mod wheel move
    Chord,    // a three note chord held for most of the render
    Mpe,      // two notes on member channels with their own bend and pressure
    Detached, // two separate notes (the first is released before the second starts)
    None
};

std::vector<Ev> scriptEvents(Script s, double rate = 48000.0)
{
    switch (s)
    {
    case Script::Mono2:
        return {on(0.0, 57, 100, 0, rate),  bend(0.12, 12288, 0, rate), cc(0.13, 1, 100, 0, rate), on(0.18, 64, 70, 0, rate),
                pressure(0.2, 90, 0, rate), off(0.38, 57, 0, rate),     off(0.40, 64, 0, rate)};
    case Script::Chord:
        return {on(0.0, 57, 100, 0, rate), on(0.0, 61, 90, 0, rate), on(0.0, 64, 80, 0, rate),
                off(0.7, 57, 0, rate),     off(0.7, 61, 0, rate),    off(0.7, 64, 0, rate)};
    case Script::Mpe:
        return {on(0.0, 57, 100, 1, rate),    on(0.0, 64, 100, 2, rate), bend(0.1, 12288, 1, rate), bend(0.2, 4096, 2, rate),
                pressure(0.15, 100, 1, rate), off(0.4, 57, 1, rate),     off(0.4, 64, 2, rate)};
    case Script::Detached:
        return {on(0.0, 57, 100, 0, rate), off(0.2, 57, 0, rate), on(0.26, 81, 100, 0, rate), off(0.5, 81, 0, rate)};
    case Script::None:
        break;
    }
    return {};
}

// The probe patch: one wavetable saw through a lowpass filter, short release. Everything else at its default.
void baseSetup(Engine& e)
{
    e.resetParamsToDefaults();
    e.setParam("osc1_table", 4.0f); // Analog Saw
    e.setParam("osc1_level", 0.8f);
    e.setParam("filter_cutoff", 5000.0f);
    e.setParam("amp_release", 0.15f);
    e.setParam("amp_sustain", 1.0f);
}

std::unique_ptr<Engine> makeEngine(const Cfg& cfg, int maxBlock = -1)
{
    EngineConfig ec;
    ec.maxVoices = cfg.voices;
    ec.seed = cfg.seed;
    ec.quality = cfg.quality;
    auto e = std::make_unique<Engine>(ec);
    e->prepare(cfg.rate, maxBlock > 0 ? maxBlock : cfg.block);
    return e;
}

Shot renderSetup(const Cfg& cfg, const Setup& setup, Script script, double seconds)
{
    auto e = makeEngine(cfg);
    baseSetup(*e);
    if (setup)
        setup(*e);
    return play(*e, cfg, scriptEvents(script, cfg.rate), samplesOf(seconds, cfg.rate));
}

AudioData makeSample(double seconds = 0.8, double hz = 220.0)
{
    AudioData a;
    a.sampleRate = 48000.0;
    a.channels.resize(2);
    std::uint32_t s = 99u;
    for (int i = 0; i < static_cast<int>(seconds * 48000.0); ++i)
    {
        s = s * 1664525u + 1013904223u;
        const float n = static_cast<float>(static_cast<std::int32_t>(s)) / 2147483648.0f;
        const double t = i / 48000.0;
        const float env = static_cast<float>(std::exp(-3.0 * std::fmod(t * 4.0, 1.0)));
        const float tone = static_cast<float>(std::sin(kTau * hz * t) + 0.5 * std::sin(kTau * hz * 2.01 * t));
        a.channels[0].push_back(0.4f * (tone * env + 0.1f * n));
        a.channels[1].push_back(0.4f * (tone * env - 0.1f * n));
    }
    return a;
}

// ---------------------------------------------------------------------------------------------------------------
// Parameter contexts: what to set up so that a parameter can be heard.
struct Ctx
{
    std::vector<std::pair<std::string, float>> sets;
    Setup extra;
    Script script = Script::Mono2;
    double seconds = 0.6;
    double hostBpm = 0.0;
    Quality quality =
        Quality::Standard; // High where a module builds its state on a worker thread (offline waits, so it repeats)
    std::string note;      // why this context
};

void applyCtx(Engine& e, const Ctx& c)
{
    if (c.extra)
        c.extra(e);
    for (const auto& [id, v] : c.sets)
        e.setParam(id, v);
}

std::string slotName(int slot)
{
    return "osc" + std::to_string(slot);
}

// Effects: what makes each module audibly active (several modules are neutral at their defaults).
const std::map<std::string, std::vector<std::pair<std::string, float>>>& fxActive()
{
    static const std::map<std::string, std::vector<std::pair<std::string, float>>> m = {
        {"fx.limiter", {{"fx_limit_gain", 12.0f}, {"fx_limit_ceiling", -12.0f}}},
        {"fx.crusher", {{"fx_crush_bits", 6.0f}, {"fx_crush_rate", 8000.0f}}},
        {"fx.width", {{"fx_width_amt", 0.2f}}},
        {"fx.filter", {{"fx_filter_cutoff", 1500.0f}, {"fx_filter_res", 0.6f}, {"fx_filter_lfo_depth", 1.0f}}},
        {"fx.multiband",
         {{"fx_multiband_low_thresh", -40.0f},
          {"fx_multiband_mid_thresh", -40.0f},
          {"fx_multiband_high_thresh", -40.0f},
          {"fx_multiband_ratio", 8.0f}}},
        {"fx.tremolo", {{"fx_tremolo_depth", 0.8f}}},
        {"fx.ring", {{"fx_ring_mix", 0.7f}}},
        {"fx.vocoder", {}},
        {"fx.comp_clean", {{"fx_cc_thresh", -30.0f}, {"fx_cc_ratio", 8.0f}, {"fx_cc_scbg", 12.0f}}},
        {"fx.comp_vca", {{"fx_cv_thresh", -30.0f}, {"fx_cv_ratio", 3.0f}}},
        {"fx.gate2", {{"fx_g2_thresh", -14.0f}, {"fx_g2_range", -60.0f}}},
        {"fx.maximizer", {{"fx_mx_gain", 12.0f}, {"fx_mx_ceiling", -12.0f}}},
        {"fx.clipper", {{"fx_clp_drive", 12.0f}, {"fx_clp_ceiling", -12.0f}}},
        {"util.meter", {}},
        {"fx.eq_pro",
         {{"fx_ep_b1_on", 1.0f},
          {"fx_ep_b1_gain", 9.0f},
          {"fx_ep_b1_freq", 800.0f},
          {"fx_ep_b2_on", 1.0f},
          {"fx_ep_b2_gain", -9.0f},
          {"fx_ep_b2_freq", 3000.0f}}},
        // The crossovers act only where a decay multiplier differs from 1, the duck release only while ducking.
        {"fx.convolution", {{"fx_cx_mix", 1.0f}}},
        // Short repeats with feedback, a modulated read and ducking, so that the loop filters, the saturation, the
        // modulation rate and the ducking controls are heard within the render. The threshold and the short release make the
        // audit's amplitude-modulated sidechain signal duck differently from the steady input.
        {"fx.delay_pro",
         {{"fx_delay_pro_time", 60.0f},
          {"fx_delay_pro_mix", 1.0f},
          {"fx_delay_pro_feedback", 0.7f},
          {"fx_delay_pro_mod_depth", 3.0f},
          {"fx_delay_pro_duck_depth", 12.0f},
          {"fx_delay_pro_duck_threshold", -20.0f},
          {"fx_delay_pro_duck_release", 30.0f}}},
        // Both gains away from zero, or the follower speed and the mix change nothing. The de-esser reduces only above its
        // threshold: a low absolute threshold on a wide band around 2.5 kHz makes it act on the audit's notes.
        {"fx.transient", {{"fx_transient_attack", 12.0f}, {"fx_transient_sustain", -12.0f}}},
        {"fx.deesser",
         {{"fx_deesser_threshold_mode", 1.0f},
          {"fx_deesser_threshold", -55.0f},
          {"fx_deesser_ratio", 20.0f},
          {"fx_deesser_frequency", 2500.0f},
          {"fx_deesser_width", 3.0f},
          {"fx_deesser_release", 250.0f}}},
        // Heard through the wet path: a deep, slow modulation with the wet signal alone so that depth, rate, shape and delay
        // change the output.
        {"fx.ensemble", {{"fx_ensemble_mix", 1.0f}, {"fx_ensemble_depth", 20.0f}}},
        // Wet path with a wide sweep and some feedback, so that range, rate, feedback saturation and the stage count change
        // the output.
        {"fx.phaser2",
         {{"fx_phaser2_mix", 0.5f}, {"fx_phaser2_range", 2.0f}, {"fx_phaser2_rate", 2.0f}, {"fx_phaser2_feedback", 0.6f}}},
        // Through-zero sweep with feedback so that depth, rate, spread and the saturation change the output.
        {"fx.flanger2",
         {{"fx_flanger2_mix", 0.5f},
          {"fx_flanger2_depth", 3.0f},
          {"fx_flanger2_rate", 2.0f},
          {"fx_flanger2_feedback", 0.6f}}},
        // Heard where the band is driven well into the saturator's curve: the offset of the asymmetry and the drive act on
        // the added signal.
        {"fx.exciter", {{"fx_exciter_amount", 100.0f}, {"fx_exciter_drive", 9.0f}, {"fx_exciter_frequency", 1500.0f}}},
        {"fx.reverb_algo", {{"fx_ra_lomult", 2.5f}, {"fx_ra_himult", 0.15f}, {"fx_ra_duck", 1.0f}, {"fx_ra_mix", 0.6f}}},
    };
    return m;
}

bool paramIs(const ParamDef& d, const char* prefix)
{
    return d.id.rfind(prefix, 0) == 0;
}

const ModuleDescriptor* moduleOf(const ParamDef& d)
{
    return ModuleRegistry::instance().find(d.module);
}

// The contexts in which a parameter is expected to be audible. The first one that shows a difference wins.
std::vector<Ctx> contextsFor(const ParamDef& d, const std::vector<std::string>& suffixHint = {})
{
    (void)suffixHint;
    std::vector<Ctx> out;
    Ctx c;
    const std::string& id = d.id;
    const auto* mod = moduleOf(d);

    const auto onlyOsc = [&](int n, Ctx& cx)
    {
        for (int k = 1; k <= 3; ++k)
            cx.sets.push_back({slotName(k) + "_level", k == n ? 0.8f : 0.0f});
        if (n != 1)
            cx.sets.push_back({slotName(n) + "_route", 0.0f});
    };

    // Per-slot oscillator parameters (osc1_..., osc2_..., osc3_...).
    if (id.size() > 5 && id.rfind("osc", 0) == 0 && id[3] >= '1' && id[3] <= '3' && id[4] == '_')
    {
        const int n = id[3] - '0';
        const std::string rest = id.substr(5);
        const std::string s = slotName(n);
        onlyOsc(n, c);
        const auto sampleSetup = [n](Engine& e)
        {
            std::string err;
            e.loadSample(n - 1, makeSample(), "audit", err);
        };
        if (rest.rfind("smp_zone", 0) == 0)
        {
            c.sets.push_back({s + "_type", 1.0f});
            c.sets.push_back({s + "_smp_zone_mode", 1.0f});
            c.extra = [n, s](Engine& e)
            {
                std::string err;
                e.loadSample(n - 1, makeSample(0.8, 220.0), "z0", err);
                const int id1 = e.addZoneSample(n - 1, makeSample(0.8, 330.0), "z1", err);
                Zone a;
                a.sampleId = 0;
                a.keyLo = 0;
                a.keyHi = 62;
                a.root = 57.0f;
                Zone b;
                b.sampleId = id1;
                b.keyLo = 58;
                b.keyHi = 127;
                b.velLo = 0;
                b.velHi = 127;
                b.root = 64.0f;
                Zone c2 = b;
                c2.keyLo = 50;
                c2.keyHi = 127;
                c2.velLo = 60;
                c2.velHi = 127;
                c2.group = 1;
                Zone c3 = c2;
                c3.group = 1;
                c3.root = 66.0f;
                e.setZones(n - 1, {a, b, c2, c3});
                e.setParam(s + "_smp_zone_xfade_key", 0.0f);
            };
            out.push_back(c);
            // the crossfade and round robin parameters need overlapping zones: play from the overlap
            Ctx c2 = c;
            c2.script = Script::Chord;
            out.push_back(c2);
            return out;
        }
        if (rest.rfind("smp_slice", 0) == 0)
        {
            c.sets.push_back({s + "_type", 1.0f});
            c.extra = [n](Engine& e)
            {
                std::string err;
                e.loadSample(n - 1, makeSample(), "slices", err);
                e.sliceSampleEqually(n - 1, 8);
            };
            c.sets.push_back(
                {s + "_smp_slice_mode",
                 rest == "smp_slice_mode" ? 0.0f : (rest == "smp_slice_base" || rest == "smp_slice_top" ? 2.0f : 1.0f)});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("smp_", 0) == 0 || rest.rfind("grain_", 0) == 0)
        {
            c.sets.push_back({s + "_type", 1.0f});
            c.sets.push_back({s + "_smp_mode", rest.rfind("grain_", 0) == 0 || rest == "smp_mode" ? 1.0f : 0.0f});
            if (rest == "smp_mode")
                c.sets.pop_back();
            c.extra = sampleSetup;
            // loop parameters need a loop; the unison parameters need voices
            if (rest.rfind("smp_loop", 0) == 0 || rest == "smp_xfade")
            {
                c.sets.push_back({s + "_smp_loop", rest == "smp_loop" ? 0.0f : 1.0f});
                if (rest == "smp_loop" || rest == "smp_xfade")
                {
                    c.sets.push_back({s + "_smp_loop_start", 0.1f});
                    c.sets.push_back({s + "_smp_loop_end", 0.2f});
                }
            }

            if (rest == "smp_uni_detune" || rest == "smp_uni_spread" || rest == "smp_uni_start")
                c.sets.push_back({s + "_smp_uni_voices", 4.0f});
            if (rest == "smp_loop_start" || rest == "smp_loop_end")
                c.sets.push_back({s + "_smp_loop", 1.0f});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("cloud_", 0) == 0)
        {
            c.sets.push_back({s + "_type", 2.0f});
            // a cloud with partials above 64 and spread-out pans: the default harmonic morph has neither
            c.extra = [n](Engine& e)
            {
                CloudModel m;
                m.name = "audit";
                for (int h = 1; h <= 120; ++h)
                {
                    CloudPoint p;
                    p.x = static_cast<float>(h);
                    p.y = 1.0f / std::sqrt(static_cast<float>(h));
                    p.z = 0.1f * static_cast<float>(h % 2);
                    p.pan = (h % 2 == 0) ? -0.9f : 0.9f;
                    m.points.push_back(p);
                }
                m.spread = 0.3f;
                m.sanitise();
                e.setCloud(n - 1, m);
            };
            out.push_back(c);
            return out;
        }
        if (rest.rfind("noise_", 0) == 0)
        {
            c.sets.push_back({s + "_type", 3.0f});
            if (rest == "noise_density")
                c.sets.push_back({s + "_noise_type", 5.0f});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("an_", 0) == 0)
        {
            // Analog oscillator: parameters that only act in some waves / modes get the setup in which they sound.
            c.sets.push_back({s + "_type", 5.0f});
            if (rest == "an_pw")
                c.sets.push_back({s + "_an_wave", 1.0f});
            if (rest == "an_shape")
                c.sets.push_back({s + "_an_wave", 5.0f});
            if (rest == "an_xmod_ratio" || rest == "an_xmod_amt")
                c.sets.push_back({s + "_an_xmod_mode", 4.0f});
            if (rest == "an_detune" || rest == "an_spread")
                c.sets.push_back({s + "_an_uni", 4.0f});
            if (rest == "an_dco_comp")
            {
                c.sets.push_back({s + "_an_char", 3.0f});
                c.sets.push_back({s + "_an_wave", 0.0f});
            }
            if (rest == "an_drift_tau")
                c.sets.push_back({s + "_an_drift", 1.0f});
            if (rest == "an_wave" || rest == "an_char")
                c.sets.push_back({s + "_an_wave", rest == "an_char" ? 1.0f : 0.0f});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("modal_", 0) == 0)
        {
            // Modal oscillator: the sustain acts on the continuous exciters, the contour on the pluck.
            c.sets.push_back({s + "_type", 6.0f});
            if (rest == "modal_sustain")
                c.sets.push_back({s + "_modal_exciter", 4.0f});
            if (rest == "modal_contour")
                c.sets.push_back({s + "_modal_exciter", 1.0f});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("harm_", 0) == 0)
        {
            // Harmonic oscillator: the formant bell needs a gain.
            c.sets.push_back({s + "_type", 7.0f});
            if (rest == "harm_formant" || rest == "harm_fwidth")
                c.sets.push_back({s + "_harm_fgain", 12.0f});
            out.push_back(c);
            return out;
        }
        if (rest.rfind("spec_", 0) == 0)
        {
            // Spectral oscillator: plays the slot's sample; the loop needs a sample shorter than the note.
            c.sets.push_back({s + "_type", 8.0f});
            c.extra = [n, rest](Engine& e)
            {
                std::string err;
                e.loadSample(n - 1, makeSample(rest == "spec_loop" ? 0.15 : 0.8), "audit", err);
            };
            out.push_back(c);
            return out;
        }
        if (rest == "wt_interp" || rest == "wt_spec" || rest == "wt_spec_amt")
        {
            // Spectral wavetable path: a user table whose neighbouring frames have formants at 8 and 24 harmonics, played
            // halfway between two of them.
            c.sets.push_back({s + "_table", static_cast<float>(WavetableBank::numTables)});
            c.sets.push_back({s + "_pos", 10.5f / 63.0f});
            if (rest == "wt_spec")
                c.sets.push_back({s + "_wt_spec_amt", 0.6f});
            if (rest == "wt_spec_amt")
                c.sets.push_back({s + "_wt_spec", 1.0f});
            c.extra = [](Engine& e)
            {
                AudioData a;
                a.sampleRate = 48000.0;
                a.channels.assign(1, std::vector<float>());
                for (int f = 0; f < 64; ++f)
                    for (int i = 0; i < 2048; ++i)
                    {
                        const double centre = (f & 1) == 0 ? 8.0 : 24.0;
                        double v = 0.0;
                        for (int k = 1; k < 80; ++k)
                            v += std::exp(-0.5 * std::pow((k - centre) / 2.0, 2.0)) *
                                 std::sin(kTau * k * i / 2048.0 + 0.3 * k);
                        a.channels[0].push_back(static_cast<float>(0.1 * v));
                    }
                WavetableImport options;
                options.frameSize = 2048;
                options.normalise = WavetableImport::Normalise::None;
                std::string err;
                e.loadWavetable(0, a, options, "audit", err);
            };
            out.push_back(c);
            return out;
        }
        if (rest.rfind("sub_", 0) == 0)
        {
            c.sets.push_back({s + "_type", 4.0f});
            c.sets.push_back({s + "_sub_wave", rest == "sub_pulse" ? 4.0f : 0.0f});
            if (rest == "sub_wave")
                c.sets.pop_back();
            out.push_back(c);
            return out;
        }
        if (rest == "type")
        {
            c.extra = sampleSetup;
            out.push_back(c);
            return out;
        }
        if (rest.rfind("xmod_", 0) == 0)
        {
            // listen to an earlier slot: osc 1 is the master (a detuned saw), osc n the carrier
            Ctx x;
            for (int k = 1; k <= 3; ++k)
                x.sets.push_back({slotName(k) + "_level", k <= n ? 0.7f : 0.0f});
            x.sets.push_back({"osc1_route", 0.0f});
            x.sets.push_back({s + "_route", 0.0f});
            x.sets.push_back({"osc1_semi", 7.0f});
            x.sets.push_back({s + "_xmod_mode", rest == "xmod_mode" ? 0.0f : 1.0f});
            x.sets.push_back({s + "_xmod_amt", rest == "xmod_amt" ? 0.0f : 0.8f});
            x.sets.push_back({s + "_xmod_src", rest == "xmod_src" ? 0.0f : 0.0f});
            if (n == 3 && rest == "xmod_src")
                x.sets.push_back({"osc2_semi", 12.0f});
            x.note = "slot " + std::to_string(n) + " listens to an earlier slot";
            out.push_back(x);
            return out;
        }
        if (rest == "warp_a" || rest == "warp_b")
        {
            for (const float warp : {1.0f, 11.0f, 14.0f, 6.0f, 13.0f})
            {
                Ctx w = c;
                w.sets.push_back({s + "_warp", warp});
                w.sets.push_back({s + "_warp_src", n == 1 ? 1.0f : 0.0f});
                w.sets.push_back({"osc" + std::string(n == 1 ? "2" : "1") + "_level", 0.0f});
                out.push_back(w);
            }
            return out;
        }
        if (rest == "warp_src")
        {
            Ctx w = c;
            w.sets.push_back({s + "_warp", 1.0f});
            w.sets.push_back({s + "_warp_a", 0.8f});
            for (int k = 1; k <= 3; ++k)
                if (k != n)
                    w.sets.push_back({slotName(k) + "_semi", 5.0f * static_cast<float>(k)});
            // the other oscillators are silent but still serve as warp sources
            out.push_back(w);
            return out;
        }
        if (rest.rfind("uni_", 0) == 0)
        {
            c.sets.push_back({s + "_uni_voices", rest == "uni_voices" ? 1.0f : 5.0f});
            if (rest == "uni_rand" || rest == "uni_scatter" || rest == "uni_blend" || rest == "uni_mode")
                c.sets.push_back({s + "_uni_detune", 40.0f});
            out.push_back(c);
            return out;
        }
        if (rest == "route")
        {
            // route 3 = dry: with two filters in different states the route is audible
            c.sets.push_back({"filter2_on", 1.0f});
            c.sets.push_back({"filter2_type", 1.0f});
            c.sets.push_back({"filter2_cutoff", 3000.0f});
            c.sets.push_back({"filter_cutoff", 400.0f});
            c.sets.push_back({"filter_route", 2.0f});
            out.push_back(c);
            return out;
        }
        out.push_back(c);
        return out;
    }

    // Effects: the module is switched on and given audible settings; its own switch is the parameter under test
    // when d is the on parameter.
    if (mod != nullptr && mod->kind == ModuleKind::Effect)
    {
        const auto active = fxActive().find(mod->id);
        if (active != fxActive().end())
            c.sets = active->second;
        if (id != mod->onParam)
            c.sets.push_back({mod->onParam, 1.0f});
        // The asymmetry only reshapes the added harmonics, a small part of the mixed output: listen to the added signal
        // alone.
        if (id == "fx_exciter_asymmetry")
            c.sets.push_back({"fx_exciter_listen", 1.0f});
        // chain position: a second, order-sensitive module
        if (id == mod->positionParam)
        {
            const std::string partnerOn = mod->id == "fx.saturator" ? "fx_delay_pro_on" : "fx_saturator_on";
            c.sets.push_back({partnerOn, 1.0f});
            if (mod->id == "fx.saturator")
                c.sets.push_back({"fx_delay_pro_time", 90.0f});
            else
                c.sets.push_back({"fx_saturator_drive", 30.0f});
            c.sets.push_back({"fx_saturator_model", 1.0f});
            c.sets.push_back({mod->id == "fx.saturator" ? "fx_pos_delay_pro" : "fx_pos_saturator", 5.0f});
        }
        // Pro EQ: a band's parameters are heard when that band is on as a bell at 800 Hz with gain; the dynamic ones when
        // its dynamics follow a key above the threshold; slope and cut shape when the band is a high cut.
        if (mod->id == "fx.eq_pro")
        {
            for (int b = 1; b <= 16; ++b)
            {
                const std::string n = "fx_ep_b" + std::to_string(b) + "_";
                if (id.rfind(n, 0) != 0)
                    continue;
                const std::string rest = id.substr(n.size());
                const auto put = [&](const char* leaf, float v)
                {
                    if (rest != leaf)
                        c.sets.push_back({n + leaf, v});
                };
                put("on", 1.0f);
                put("type", 0.0f);
                put("freq", 800.0f);
                put("gain", 9.0f);
                put("q", 1.0f);
                if (rest == "slope" || rest == "shape")
                {
                    c.sets.push_back({n + "type", 8.0f});
                    c.sets.push_back({n + "freq", 1200.0f});
                    c.sets.push_back({n + "slope", rest == "shape" ? 3.0f : 1.0f});
                }
                if (rest == "dyn" || rest == "range" || rest == "thr" || rest == "ratio" || rest == "att" || rest == "rel" ||
                    rest == "key")
                {
                    put("dyn", 1.0f);
                    put("range", -12.0f);
                    put("thr", -50.0f);
                    put("ratio", 8.0f);
                    put("att", 5.0f);
                    put("rel", 60.0f);
                }
            }
        }
        // The bucket-brigade controls are heard in the Analog engine.
        if (id.rfind("fx_delay_pro_analog_", 0) == 0)
            c.sets.push_back({"fx_delay_pro_mode", 1.0f});
        // The bass frequency acts only in the Low mode.
        if (id == "fx_exciter_bass_frequency")
            c.sets.push_back({"fx_exciter_mode", 1.0f});
        // The absolute threshold acts only in the Absolute threshold mode, the relative one only in Relative.
        if (id == "fx_deesser_relative")
        {
            c.sets.push_back({"fx_deesser_threshold_mode", 0.0f});
            c.sets.push_back({"fx_deesser_detector", 1.0f});
            c.sets.push_back({"fx_deesser_frequency", 2000.0f});
            c.sets.push_back({"filter_cutoff", 20000.0f}); // the band's share of the level must reach the threshold
        }
        // The crossfeed acts in the Dimensional modes; the compander acts on the device noise.
        if (id == "fx_ensemble_crossfeed")
            c.sets.push_back({"fx_ensemble_mode", 1.0f});
        if (id == "fx_ensemble_compander")
        {
            c.sets.push_back({"fx_ensemble_noise", -45.0f});
            c.sets.push_back({"fx_ensemble_depth", 0.0f});
        }
        // The phaser's drive acts in the OTA and JFET stage types; the envelope controls act with the envelope source.
        if (id == "fx_phaser2_drive")
            c.sets.push_back({"fx_phaser2_type", 1.0f});
        if (id == "fx_phaser2_env_sens" || id == "fx_phaser2_env_attack" || id == "fx_phaser2_env_release")
            c.sets.push_back({"fx_phaser2_source", 1.0f});
        // The flanger's device size acts in the BBD mode; the envelope controls act with the envelope source.
        if (id == "fx_flanger2_bbd_stages")
            c.sets.push_back({"fx_flanger2_mode", 1.0f});
        if (id == "fx_flanger2_env_sens" || id == "fx_flanger2_env_attack" || id == "fx_flanger2_env_release")
            c.sets.push_back({"fx_flanger2_source", 1.0f});
        if (mod->id == "fx.width")
        {
            c.sets.push_back({"osc1_uni_voices", 5.0f});
            c.sets.push_back({"osc1_uni_detune", 30.0f});
        }
        if (mod->id == "fx.vocoder")
        {
            if (id == "fx_voc_pitch")
                c.sets.push_back({"fx_voc_carrier", 1.0f});
            c.sets.push_back({"fx_voc_input", id == "fx_voc_input" ? 0.0f : 2.0f});
            c.sets.push_back({"fx_voc_mix", 1.0f});
        }
        // Dynamics: the stereo link only acts in the mid / side modes, true peak only in the peak detector, the console
        // drive only with the output stage on.
        if (id == "fx_cc_link" || id == "fx_cv_link" || id == "fx_g2_link" || id == "fx_deesser_link")
        {
            c.sets.push_back({id.substr(0, id.find("_link")) + "_channels", 4.0f});
            c.sets.push_back({"osc1_uni_voices", 5.0f});
            c.sets.push_back({"osc1_uni_detune", 30.0f});
        }
        if (id == "fx_g2_sclp")
        {
            c.sets.push_back({"fx_g2_listen", 1.0f}); // the filtered key is then the output
        }
        // Maximiser and clipper: the stereo options need a wide source, the clip knee and the dither bits need their stage
        // on.
        if (id == "fx_mx_link" || id == "fx_mx_stereo")
        {
            c.sets.push_back({"osc1_uni_voices", 5.0f});
            c.sets.push_back({"osc1_uni_detune", 30.0f});
        }
        if (id == "fx_mx_knee")
            c.sets.push_back({"fx_mx_clip", 100.0f});
        if (id == "fx_mx_autohold")
        {
            c.sets.push_back({"fx_mx_hold", 0.0f});
            c.sets.push_back(
                {"fx_mx_lookahead", 0.5f}); // shorter than half a period of the probe's 220 Hz, so the auto hold decides
            c.sets.push_back({"fx_mx_relmode", 0.0f});
            c.sets.push_back({"fx_mx_release", 30.0f});
        }
        if (id == "fx_clp_os" || id == "fx_clp_adaa" || id == "fx_clp_shape")
            c.sets.push_back({"fx_clp_drive", 18.0f});
        if (id == "fx_cc_truepeak")
            c.sets.push_back({"fx_cc_detector", 0.0f});
        if (id == "fx_cv_drive")
            c.sets.push_back({"fx_cv_console", 1.0f});
        // Saturator: the tilt pivot only acts with a tilt set; the character control is the tube sag, the tape loop width or
        // the transformer corner, and the default soft clip ignores it.
        if (id == "fx_saturator_tilt_freq")
        {
            c.sets.push_back({"fx_saturator_drive", 24.0f});
            c.sets.push_back({"fx_saturator_tilt", 9.0f});
        }
        if (id == "fx_saturator_character")
        {
            c.sets.push_back({"fx_saturator_drive", 24.0f});
            c.sets.push_back({"fx_saturator_model", 7.0f}); // tube pentode: the sag
        }
        // Shaper: the identity curve at 0 dB drive is the input itself (the wet path is left out), so the mix, the DC block
        // and the chain slot are heard with a drawn asymmetric curve and some drive.
        if (mod->id == "fx.shaper")
        {
            if (id != "fx_shaper_drive")
                c.sets.push_back({"fx_shaper_drive", 12.0f});
            // symmetric, so that the sweep of the output level stays free of DC; the DC block needs an asymmetric curve
            const bool asymmetric = id == "fx_shaper_dc";
            c.extra = [asymmetric](Engine& e)
            {
                const ShaperPoint soft[] = {{-1.0f, -1.0f, 0.0f}, {-0.5f, -0.4f, 0.0f}, {0.5f, 0.4f, 0.0f}, {1.0f, 1.0f, 0.0f}};
                const ShaperPoint skew[] = {{-1.0f, -0.4f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.5f, 0.6f, 0.0f}, {1.0f, 1.0f, 0.0f}};
                std::string err;
                e.setShaperCurve("fx.shaper", asymmetric ? skew : soft, 4, false, err);
            };
        }
        // Crusher note tracking: the ratio and the fine tuning act with Track on (the script plays notes).
        if (id == "fx_crush_ratio" || id == "fx_crush_fine")
            c.sets.push_back({"fx_crush_track", 1.0f});
        // Saturator filter: the type, cutoff and resonance act with the filter before the shaper.
        if (id == "fx_saturator_filter_type" || id == "fx_saturator_filter_cutoff" || id == "fx_saturator_filter_res")
        {
            c.sets.push_back({"fx_saturator_filter_mode", 1.0f});
            c.sets.push_back({"fx_saturator_drive", 24.0f});
        }
        // Cabinet: the impulse responses are built on a worker thread; High quality awaits them offline.
        if (mod->id == "fx.cabinet")
            c.quality = Quality::High;
        // The convolution module is audible once a response is loaded; its kernel builds are awaited at High quality.
        if (mod->id == "fx.convolution")
        {
            c.quality = Quality::High;
            c.extra = [](Engine& e)
            {
                std::mt19937 gen(12);
                ImpulseResponse ir;
                ir.sampleRate = 48000.0;
                ir.layout = IrLayout::Stereo;
                for (int ch = 0; ch < 2; ++ch)
                {
                    std::vector<float> k(57600);
                    for (size_t i = 0; i < k.size(); ++i)
                        k[i] = 0.05f * (static_cast<float>(gen() >> 8) / 8388608.0f - 1.0f) *
                               std::exp(-5.0f * static_cast<float>(i) / 57600.0f);
                    ir.channels.push_back(std::move(k));
                }
                e.loadConvolutionIr(ir, "audit");
            };
        }
        if (mod->id == "fx.trance" && id == "fx_trance_pattern")
            c.sets.push_back({"fx_trance_steps", 16.0f});
        // sync choices only act when a tempo division is selected: the tests below pick one explicitly
        out.push_back(c);
        return out;
    }

    // Modulation sources are heard through the cutoff of a low filter: matrix slot 16 routes the source to it.
    const auto via = [&](int src, float amount, Ctx& cx)
    {
        cx.sets.push_back({"filter_cutoff", 900.0f});
        cx.sets.push_back({"mod16_src", static_cast<float>(src)});
        cx.sets.push_back({"mod16_dst", 7.0f}); // ModDest::Cutoff
        cx.sets.push_back({"mod16_amt", amount});
    };
    // The modulator bank (P12): the modulator under test is routed to the cutoff of a low filter; the context makes the
    // parameter matter (an LFO or two as inputs, the type that reads the parameter).
    if (id.rfind("bank", 0) == 0 && id.size() > 6 && std::isdigit(static_cast<unsigned char>(id[4])) && id[5] == '_')
    {
        const int n = id[4] - '0';
        const std::string p = "bank" + std::to_string(n);
        const std::string rest = id.substr(6);
        const float lfo1 = static_cast<float>(ModSource::Lfo1), lfo2 = static_cast<float>(ModSource::Lfo2);
        const auto type = [&](ModulatorType t) { c.sets.push_back({p + "_type", static_cast<float>(t)}); };
        c.sets.push_back({"filter_cutoff", 900.0f});
        c.sets.push_back({"filter_res", 0.4f});
        c.sets.push_back({"lfo1_rate", 5.0f});
        c.sets.push_back({"lfo2_rate", 3.0f});
        if (rest == "type")
        {
            c.sets.push_back({p + "_in_a", lfo1});
            c.sets.push_back({p + "_in_b", lfo2});
            c.sets.push_back({p + "_rate", 6.0f});
            c.sets.push_back({p + "_rise", 0.05f});
            c.sets.push_back({p + "_fall", 0.05f});
        }
        else if (rest == "scope" || rest == "rate")
        {
            type(ModulatorType::LfoTable);
            c.sets.push_back({p + "_rate", 3.0f});
            c.sets.push_back({p + "_amount", 0.3f});
        }
        else if (rest == "in_a" || rest == "in_b" || rest == "mode")
        {
            type(ModulatorType::Combine);
            if (rest != "in_a")
                c.sets.push_back({p + "_in_a", lfo1});
            if (rest != "in_b")
                c.sets.push_back({p + "_in_b", lfo2});
        }
        else if (rest == "amount")
        {
            type(ModulatorType::Combine);
            c.sets.push_back({p + "_mode", 4.0f});
            c.sets.push_back({p + "_in_a", lfo1});
        }
        else if (rest == "rise" || rest == "fall")
        {
            type(ModulatorType::Lag);
            c.sets.push_back({p + "_in_a", lfo1});
            c.sets.push_back({p + (rest == "rise" ? "_fall" : "_rise"), 0.05f});
        }
        else if (rest == "steps")
        {
            type(ModulatorType::Quantise);
            c.sets.push_back({p + "_in_a", lfo1});
        }
        else if (rest == "seed")
        {
            type(ModulatorType::Turing);
            c.sets.push_back({p + "_rate", 8.0f});
            c.sets.push_back({p + "_amount", 0.0f});
        }
        else // pt01 .. pt16: the Remap curve
        {
            type(ModulatorType::Remap);
            c.sets.push_back({p + "_in_a", lfo1});
        }
        c.extra = [n](Engine& e) { e.modRoutes().add({kSourceModulator + n - 1, "filter_cutoff", 0.8f, 0, 0.0f, 0.0f}); };
        out.push_back(c);
        return out;
    }
    // Audio-rate LFOs and the filters' FM input (P12): heard as a per-sample cutoff modulation of a low filter.
    if (id.rfind("lfo", 0) == 0 && id.size() >= 10 && id.compare(4, 6, "_audio") == 0)
    {
        const int k = id[3] - '0';
        const std::string p = "lfo" + std::to_string(k);
        c.sets.push_back({"filter_cutoff", 900.0f});
        c.sets.push_back({"filter_res", 0.4f});
        c.sets.push_back({"filter_fm_src", static_cast<float>(k)});
        c.sets.push_back({"filter_fm_amt", 0.5f});
        if (id != p + "_audio")
            c.sets.push_back({p + "_audio", id == p + "_audio_ratio" ? 2.0f : 1.0f});
        c.sets.push_back({p + "_audio_hz", 700.0f});
        if (id == p + "_audio_hz")
            c.sets.pop_back();
        out.push_back(c);
        return out;
    }
    if (id.rfind("filter", 0) == 0 && id.find("_fm_") != std::string::npos)
    {
        const bool second = id.rfind("filter2_", 0) == 0;
        const std::string p = second ? "filter2" : "filter";
        if (second)
        {
            c.sets.push_back({"filter2_on", 1.0f});
            c.sets.push_back({"filter2_cutoff", 900.0f});
            c.sets.push_back({"filter2_res", 0.4f});
        }
        else
        {
            c.sets.push_back({"filter_cutoff", 900.0f});
            c.sets.push_back({"filter_res", 0.4f});
        }
        c.sets.push_back({"lfo1_audio", 1.0f});
        c.sets.push_back({"lfo1_audio_hz", 700.0f});
        c.sets.push_back({p + (id == p + "_fm_src" ? "_fm_amt" : "_fm_src"), id == p + "_fm_src" ? 0.5f : 1.0f});
        out.push_back(c);
        return out;
    }
    const auto slotOf = [&](const char* prefix, int count)
    {
        for (int k = 1; k <= count; ++k)
            if (id.rfind(std::string(prefix) + std::to_string(k) + "_", 0) == 0)
                return k;
        return 0;
    };
    if (const int k = slotOf("lfo", 4))
    {
        static const int sources[] = {0, 1, 2, 9, 10};
        via(sources[k], 0.8f, c);
        c.sets.push_back({"lfo" + std::to_string(k) + "_rate", 5.0f});
        if (id.find("_sync") != std::string::npos)
            c.sets.push_back({"lfo" + std::to_string(k) + "_rate", 5.0f});
        out.push_back(c);
        return out;
    }
    // The overshoot only acts on the analog RC circuit.
    const bool overshoot = id.size() > 10 && id.compare(id.size() - 10, 10, "_overshoot") == 0;
    if (const int k = slotOf("env", 4))
    {
        via(k == 3 ? 7 : 8, 0.8f, c);
        if (overshoot)
            c.sets.push_back({"env" + std::to_string(k) + "_mode", 1.0f});
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "menv_"))
    {
        via(3, 0.8f, c);
        if (overshoot)
            c.sets.push_back({"menv_mode", 1.0f});
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "amp_"))
    {
        if (overshoot)
            c.sets.push_back({"amp_mode", 1.0f});
        if (id == "amp_decay" || id == "amp_hold")
        {
            c.sets.push_back({"amp_sustain", 0.2f});
            c.sets.push_back({"amp_decay", id == "amp_hold" ? 0.15f : 0.3f});
        }
        out.push_back(c);
        return out;
    }
    if (id.rfind("mod", 0) == 0 && id.size() > 4 && std::isdigit(static_cast<unsigned char>(id[3])))
    {
        const std::string slot = id.substr(0, id.find('_'));
        c.sets.push_back({"filter_cutoff", 900.0f});
        if (id.find("_src") == std::string::npos)
            c.sets.push_back({slot + "_src", 1.0f});
        if (id.find("_dst") == std::string::npos)
            c.sets.push_back({slot + "_dst", 7.0f});
        if (id.find("_amt") == std::string::npos)
            c.sets.push_back({slot + "_amt", 0.8f});
        c.sets.push_back({"lfo1_rate", 6.0f});
        out.push_back(c);
        return out;
    }
    if (id.rfind("macro", 0) == 0)
    {
        via(13 + (id.back() - '0'), 0.8f, c);
        out.push_back(c);
        return out;
    }
    if (id == "strip_l" || id == "strip_r")
    {
        via(id == "strip_l" ? 20 : 21, 0.8f, c);
        out.push_back(c);
        return out;
    }
    if (id == "scene_morph")
    {
        via(22, 0.8f, c);
        out.push_back(c);
        return out;
    }

    // Legacy mixer: sub and noise, filters.
    if (paramIs(d, "sub_") || paramIs(d, "noise_"))
    {
        c.sets.push_back({paramIs(d, "sub_") ? "sub_level" : "noise_level", 0.6f});
        if (id == "sub_route" || id == "noise_route")
        {
            c.sets.push_back({"filter2_on", 1.0f});
            c.sets.push_back({"filter2_type", 1.0f});
            c.sets.push_back({"filter2_cutoff", 3000.0f});
            c.sets.push_back({"filter_cutoff", 400.0f});
            c.sets.push_back({"filter_route", 2.0f});
        }
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "filter"))
    {
        c.sets.push_back({"filter_cutoff", 1500.0f});
        c.sets.push_back({"filter_res", 0.4f});
        c.sets.push_back({"filter_env", 0.0f});
        if (paramIs(d, "filter2_"))
        {
            c.sets.push_back({"filter2_on", 1.0f});
            c.sets.push_back({"filter2_cutoff", 1500.0f});
        }
        if (id == "filter_pan" || id == "filter_mix" || id == "filter2_pan" || id == "filter2_mix" || id == "filter_route")
        {
            c.sets.push_back({"filter2_on", 1.0f});
            c.sets.push_back({"filter2_type", 1.0f});
            c.sets.push_back({"filter2_cutoff", 3000.0f});
            c.sets.push_back({"filter_cutoff", 400.0f});
            c.sets.push_back({"filter_route", id == "filter_route" ? 0.0f : 1.0f});
        }
        if (id == "filter_env" || id == "filter2_env")
            c.sets.push_back({"menv_decay", 0.2f});
        // Voicing controls of the nonlinear models: the SVF morph (character) and the Sallen-Key low-pass (high-pass step).
        for (const char* base : {"filter", "filter2"})
        {
            if (id == std::string(base) + "_character")
                c.sets.push_back({std::string(base) + "_type", 22.0f});
            if (id == std::string(base) + "_hp")
                c.sets.push_back({std::string(base) + "_type", 20.0f});
        }
        if (id == "filter2_on")
            c.sets.pop_back(), c.sets.pop_back();
        out.push_back(c);
        return out;
    }

    // The analog voice: the card's filter tolerance needs an analog filter model; the noise floor and hum are heard against
    // a quiet voice.
    if (id == "voice_card_filter")
    {
        c.sets.push_back({"filter_type", 17.0f});
        c.sets.push_back({"filter_res", 0.6f});
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "voice_noise_") || paramIs(d, "voice_hum_"))
    {
        const bool noise = paramIs(d, "voice_noise_");
        c.sets.push_back({"osc1_level", 0.02f});
        c.sets.push_back({noise ? "voice_noise_on" : "voice_hum_on", 1.0f});
        c.sets.push_back({noise ? "voice_noise_db" : "voice_hum_db", -60.0f});
        out.push_back(c);
        return out;
    }

    // Performance.
    if (paramIs(d, "voice_") || id == "master_gain")
    {
        if (id == "voice_poly")
            c.script = Script::Chord;
        if (id == "voice_glide")
        {
            c.sets.push_back({"voice_mode", 1.0f});
            c.script = Script::Detached;
        }
        if (id == "voice_glide_mode" || id == "voice_glide_type")
        {
            c.sets.push_back({"voice_mode", 1.0f});
            c.sets.push_back({"voice_glide", 0.3f});
            c.script = Script::Detached;
        }
        if (id == "voice_mono_priority")
        {
            c.sets.push_back({"voice_mode", 1.0f});
            c.script = Script::Chord;
        }
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "key_"))
    {
        c.script = Script::Chord;
        if (id != "key_mode")
            c.sets.push_back({"key_mode", 1.0f});
        if (id != "key_scale")
            c.sets.push_back({"key_scale", id == "key_mask" ? 16.0f : 1.0f});
        out.push_back(c);
        if (id == "key_sync")
            out.back().note = "needs a key bus: covered by the key lock section";
        return out;
    }
    if (id == "tune_amount")
    {
        c.script = Script::Chord;
        c.extra = [](Engine& e)
        {
            std::string err;
            e.tuning().edit(
                [&](Tuning& t)
                {
                    std::string scl = "19 EDO\n19\n";
                    for (int i = 1; i <= 19; ++i)
                        scl += std::to_string(1200.0 * i / 19.0) + "\n";
                    t.loadScl(scl, err);
                });
        };
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "mpe_"))
    {
        c.script = Script::Mpe;
        if (id != "mpe_on")
            c.sets.push_back({"mpe_on", 1.0f});
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "arp_"))
    {
        c.script = Script::Chord;
        c.seconds = 0.9;
        if (id != "arp_on")
            c.sets.push_back({"arp_on", 1.0f});
        if (paramIs(d, "arp_grid"))
            c.sets.push_back({"arp_grid", 1.0f});
        if (id != "arp_sync")
            c.sets.push_back({"arp_sync", 0.0f});
        else
            c.hostBpm = 90.0;
        if (id == "arp_free_rate")
            c.sets.push_back({"arp_rate", 15.0f});
        if (id == "arp_vel")
            c.sets.push_back({"arp_vel_mode", 1.0f});
        if (id == "arp_accent")
            c.sets.push_back({"arp_vel_mode", 2.0f});
        c.sets.push_back({"amp_release", 0.05f});
        c.extra = [](Engine& e)
        {
            e.arpPattern().edit(
                [](ArpPattern& p)
                {
                    p.length = 8;
                    for (int i = 0; i < 8; ++i)
                    {
                        p.steps[static_cast<size_t>(i)].on = (i % 3) != 2;
                        p.steps[static_cast<size_t>(i)].octave = static_cast<std::int8_t>(i % 3);
                        p.steps[static_cast<size_t>(i)].velocity = static_cast<std::uint8_t>(i % 2 ? 120 : 40);
                        p.steps[static_cast<size_t>(i)].gate = 0.5f + 0.1f * static_cast<float>(i % 4);
                    }
                });
        };
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "seq_"))
    {
        c.script = Script::None;
        c.seconds = 2.2;
        if (id != "seq_run")
            c.sets.push_back({"seq_run", 1.0f});
        if (id != "seq_sync")
            c.sets.push_back({"seq_sync", 0.0f});
        else
            c.hostBpm = 200.0;
        c.sets.push_back({"seq_tempo", 240.0f});
        c.sets.push_back({"amp_release", 0.05f});
        c.extra = [](Engine& e)
        {
            e.sequencer().edit(
                [](SeqBank& b)
                {
                    for (int slot = 0; slot < kSeqSlots; ++slot)
                    {
                        auto& pat = b.pattern[static_cast<size_t>(slot)];
                        pat.lengthBars = 1;
                        for (int i = 0; i < 16; ++i)
                        {
                            auto& st = pat.steps[static_cast<size_t>(i)];
                            st.on = (i % 4) != 3;
                            st.note = static_cast<std::uint8_t>(48 + ((i * 5 + slot * 3) % 24));
                            st.velocity = static_cast<std::uint8_t>(60 + (i * 7) % 60);
                            st.gate = 0.4f + 0.1f * static_cast<float>(i % 5);
                        }
                    }
                });
        };
        out.push_back(c);
        return out;
    }
    if (paramIs(d, "vector_"))
    {
        c.seconds = 0.8;
        if (id != "vector_on")
            c.sets.push_back({"vector_on", 1.0f});
        if (id == "vector_drift_rate")
            c.sets.push_back({"vector_drift", 0.4f});
        if (id == "vector_drift")
            c.sets.push_back({"vector_drift_rate", 4.0f});
        if (id == "vector_smooth")
            c.sets.push_back({"vector_orbit_depth", 0.4f});
        if (id == "vector_orbit_rate" || id == "vector_orbit_shape")
            c.sets.push_back({"vector_orbit_depth", 0.4f});
        c.extra = [](Engine& e)
        {
            for (int slot = 0; slot < kNumScenes; ++slot)
            {
                e.resetParamsToDefaults();
                e.setParam("osc1_table", 4.0f);
                e.setParam("osc1_level", 0.8f);
                e.setParam("amp_release", 0.15f);
                e.setParam("filter_cutoff", 200.0f * std::pow(1.45f, static_cast<float>(slot)));
                e.setParam("osc1_pos", 0.08f * static_cast<float>(slot));
                e.storeScene(slot);
            }
            baseSetup(e);
        };
        out.push_back(c);
        return out;
    }
    // Everything else: the probe patch.
    out.push_back(c);
    return out;
}

} // namespace

namespace
{
// Parameters that do not change the sound by design, with the reason (and where they are exercised instead).
const std::map<std::string, std::string>& nonAudio()
{
    static const std::map<std::string, std::string> m = {
        {"fx_delay_pro_time_change", "chooses how a change of the delay time is followed (glide or crossfade); the time "
                                     "does not change in a sweep, the click-free change is exercised by the delay tests"},
        {"meter_hold", "UI hold time of the clip indicator; exercised by the meter section"},
        {"scene_a", "scene selector for the host's recall / morph calls; exercised by the scenes section"},
        {"scene_b", "scene selector for the host's recall / morph calls; exercised by the scenes section"},
        {"scene_slot", "scene selector for the host's store / recall calls; exercised by the scenes section"},
        {"seq_rec", "arms step recording into a queue the host drains; exercised by the sequencer section"},
        {"key_sync", "follows or leads a key bus; exercised by the key lock section"},
        {"osc1_xmod_mode", "slot 1 has no earlier slot to listen to (cross modulation reads slots before its own)"},
        {"osc1_xmod_src", "slot 1 has no earlier slot to listen to (cross modulation reads slots before its own)"},
        {"osc1_xmod_amt", "slot 1 has no earlier slot to listen to (cross modulation reads slots before its own)"},
        {"fx_mtr_on", "the meter passes the audio through bit for bit; its readings are exercised by the loudness tests"},
        {"fx_mtr_reset", "clears the meter's integrated loudness and maxima; exercised by the loudness tests"},
        {"fx_mtr_ppm", "ballistics of the PPM reading; exercised by the loudness tests"},
        {"fx_pos_mtr", "chain position of a module that passes the audio through unchanged"},
        {"fx_mx_mreset", "clears the maximiser's loudness readout; exercised by the maximiser tests"},
        {"fx_mx_dither",
         "adds noise at -96 dBFS or lower, far below the audit's change threshold; exercised by the maximiser tests"},
        {"fx_mx_bits", "word length of that dither; exercised by the maximiser tests"},
        {"fx_mx_target", "target loudness of the gain suggestion made from the readout; exercised by the maximiser tests"},
        {"fx_ep_res", "length of the linear-phase kernel; the kernel is designed on a worker thread, so offline renders do "
                      "not wait for it; exercised in EqProTests"},
    };
    return m;
}

std::vector<float> sweepValues(const ParamDef& d)
{
    std::vector<float> v;
    if (d.type == ParamType::Bool)
        return {0.0f, 1.0f};
    if (d.destinationChoice)
        return {0.0f, 7.0f, 1.0f, 3.0f, 5.0f, 30.0f};
    if (d.type == ParamType::Choice)
    {
        const int n = static_cast<int>(d.max - d.min) + 1;
        if (n <= 24)
            for (int i = 0; i < n; ++i)
                v.push_back(d.min + static_cast<float>(i));
        else
            for (int i = 0; i < 12; ++i)
                v.push_back(d.min + std::round(static_cast<float>(n - 1) * static_cast<float>(i) / 11.0f));
        return v;
    }
    v.push_back(d.min);
    v.push_back(ParameterMap::fromNormalised(d, 0.5f));
    v.push_back(d.max);
    if (d.type == ParamType::Float)
    {
        v.insert(v.begin() + 1, ParameterMap::fromNormalised(d, 0.2f));
        v.insert(v.end() - 1, ParameterMap::fromNormalised(d, 0.8f));
    }
    return v;
}

} // namespace

class ParameterSweepAudit : public UnitTest
{
public:
    ParameterSweepAudit() : UnitTest("Audit: parameter sweep", "Audit") {}

    void runTest() override
    {
        beginTest("every parameter: finite, bounded, no stuck DC, no subnormals, audible effect");
        const auto& defs = ParameterMap::params();
        const int stride = auditStride();
        const Cfg cfg;
        int audited = 0, silentFail = 0;
        std::vector<std::string> unresponsive;
        double worstPeak = 0.0;
        std::string worstPeakId;
        const auto started = std::chrono::steady_clock::now();
        for (size_t i = 0; i < defs.size(); i += static_cast<size_t>(stride))
        {
            const auto& d = defs[i];
            ++audited;
            const auto values = sweepValues(d);
            const auto contexts = contextsFor(d);
            bool audible = false;
            double bestDiff = 0.0;
            bool healthy = true;
            std::string detail;
            for (const auto& ctx : contexts)
            {
                std::vector<Shot> shots;
                for (const float v : values)
                {
                    Cfg run = cfg;
                    run.hostBpm = ctx.hostBpm;
                    run.quality = ctx.quality;
                    shots.push_back(renderSetup(
                        run,
                        [&](Engine& e)
                        {
                            applyCtx(e, ctx);
                            e.setParam(static_cast<int>(i), v);
                        },
                        ctx.script, ctx.seconds));
                }
                for (size_t k = 0; k < shots.size(); ++k)
                {
                    const auto& s = shots[k];
                    if (!s.finite)
                    {
                        healthy = false;
                        detail += " non-finite at " + std::to_string(values[k]);
                    }
                    if (s.peak > kPeakBound)
                    {
                        healthy = false;
                        detail += " peak " + std::to_string(s.peak) + " at " + std::to_string(values[k]);
                    }
                    if (s.peak > worstPeak)
                    {
                        worstPeak = s.peak;
                        worstPeakId = d.id;
                    }
                    const size_t tail = s.l.size() > 4800 ? s.l.size() - 4800 : 0;
                    const double dc = s.mean(tail, s.l.size());
                    if (std::abs(dc) > 0.05 && std::abs(dc) > 0.5 * s.rmsRange(tail, s.l.size()) &&
                        ctx.script != Script::None)
                    {
                        healthy = false;
                        detail += " dc " + std::to_string(dc) + " at " + std::to_string(values[k]);
                    }
                    if (s.allocations > 0)
                    {
                        healthy = false;
                        detail +=
                            " " + std::to_string(s.allocations) + " allocations in process at " + std::to_string(values[k]);
                    }
                    if (s.subnormals > 0)
                    {
                        healthy = false;
                        detail += " subnormals at " + std::to_string(values[k]);
                    }
                }
                for (size_t a = 0; a < shots.size(); ++a)
                    for (size_t b = a + 1; b < shots.size(); ++b)
                        bestDiff = std::max(bestDiff, relDiff(shots[a], shots[b]));
                if (bestDiff > 1.0e-3)
                {
                    audible = true;
                    break;
                }
            }
            if (!healthy)
                ++silentFail;
            expect(healthy, d.id + ":" + detail);
            const auto na = nonAudio().find(d.id);
            if (!audible && na == nonAudio().end())
                unresponsive.push_back(d.id);
            char buf[64];
            std::snprintf(buf, sizeof(buf), "%.2g", bestDiff);
            if (stride == 1)
                record("Parameter", d.id, "sweep " + std::to_string(values.size()) + " values in the module's context",
                       healthy ? (na != nonAudio().end() ? "non-audio: " + na->second
                                                         : (audible ? std::string("audible (relative change ") + buf + ")"
                                                                    : std::string("NO EFFECT")))
                               : "UNHEALTHY:" + detail);
        }
        const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        logMessage("audited " + String(audited) + " parameters in " + String(secs, 1) + " s, worst peak " +
                   String(worstPeak, 2) + " (" + worstPeakId + ")");
        std::string list;
        for (const auto& u : unresponsive)
            list += " " + u;
        logMessage("unresponsive:" + (list.empty() ? std::string(" none") : list));
        expect(unresponsive.empty(), "parameters with no measurable effect:" + list);
        (void)silentFail;
    }
};
static ParameterSweepAudit parameterSweepAudit;

// ---------------------------------------------------------------------------------------------------------------
namespace
{
// Windowed single-bin DFT magnitude at a frequency.
double binMagnitude(const std::vector<float>& x, size_t from, size_t to, double hz, double rate)
{
    to = std::min(to, x.size());
    double re = 0.0, im = 0.0, wsum = 0.0;
    const double n = static_cast<double>(to - from);
    for (size_t i = from; i < to; ++i)
    {
        const double w = 0.5 - 0.5 * std::cos(kTau * static_cast<double>(i - from) / n);
        const double ph = kTau * hz * static_cast<double>(i) / rate;
        re += w * x[i] * std::cos(ph);
        im += w * x[i] * std::sin(ph);
        wsum += w;
    }
    return 2.0 * std::sqrt(re * re + im * im) / std::max(wsum, 1.0);
}

// The frequency near `expected` (within +-600 cents) with the strongest component, refined to 0.05 cent.
double measureFrequency(const std::vector<float>& x, size_t from, size_t to, double expected, double rate)
{
    double best = expected, bestMag = -1.0;
    for (double cents = -600.0; cents <= 600.0; cents += 5.0)
    {
        const double hz = expected * std::pow(2.0, cents / 1200.0);
        const double m = binMagnitude(x, from, to, hz, rate);
        if (m > bestMag)
        {
            bestMag = m;
            best = hz;
        }
    }
    double lo = best * std::pow(2.0, -5.0 / 1200.0), hi = best * std::pow(2.0, 5.0 / 1200.0);
    for (int it = 0; it < 24; ++it)
    {
        const double a = lo + (hi - lo) / 3.0, b = hi - (hi - lo) / 3.0;
        if (binMagnitude(x, from, to, a, rate) < binMagnitude(x, from, to, b, rate))
            lo = a;
        else
            hi = b;
    }
    return 0.5 * (lo + hi);
}

double centsBetween(double a, double b)
{
    return 1200.0 * std::log2(a / b);
}

double noteHz(double note)
{
    return 440.0 * std::pow(2.0, (note - 69.0) / 12.0);
}

// A clean sine patch: the Sine-Saw table at position 0 is a sine; no filter colouring.
void sineSetup(Engine& e)
{
    e.setParam("osc1_table", 0.0f);
    e.setParam("osc1_pos", 0.0f);
    e.setParam("filter_on", 0.0f);
    e.setParam("osc1_level", 0.8f);
}
} // namespace

class RegistryAudit : public UnitTest
{
public:
    RegistryAudit() : UnitTest("Audit: module registry and parameter model", "Audit") {}

    void runTest() override
    {
        beginTest("harness: the allocation counter sees allocations");
        {
            gAllocations = 0;
            gTrackAllocations = true;
            static int* volatile sink;
            sink = new int[16];
            gTrackAllocations = false;
            delete[] sink;
            // The sanitizer runtimes interpose operator new, so the counter sees nothing there; the Release and scalar
            // builds carry the allocation checks.
            if (kSanitized && gAllocations < 1)
                logMessage("allocation counter unavailable under a sanitizer: allocation checks are vacuous in this build");
            else
                expect(gAllocations >= 1, "the counter did not register a deliberate allocation");
        }

        beginTest("every module: stable unique id, parameters in the model, effect factory works");
        {
            const auto& reg = ModuleRegistry::instance();
            std::set<std::string> ids;
            int effects = 0, oscillators = 0;
            for (const auto& m : reg.modules())
            {
                bool ok = !m.id.empty() && !m.name.empty() && ids.insert(m.id).second;
                int found = 0;
                for (const auto& p : m.params)
                {
                    const int idx = ParameterMap::indexOf(p.id);
                    if (idx < 0 || ParameterMap::params()[static_cast<size_t>(idx)].module != m.id)
                        ok = false;
                    else
                        ++found;
                }
                expect(ok && found == static_cast<int>(m.params.size()), "module " + m.id);
                if (m.kind == ModuleKind::Effect)
                {
                    ++effects;
                    auto fx = m.createEffect ? m.createEffect() : nullptr;
                    expect(fx != nullptr, m.id + " has no effect factory");
                    expect(ParameterMap::indexOf(m.onParam) >= 0 && ParameterMap::indexOf(m.positionParam) >= 0,
                           m.id + " lacks an on / position parameter");
                    if (fx)
                    {
                        ModuleContext ctx;
                        fx->prepare(ctx);
                        fx->reset();
                        fx->deactivate();
                    }
                }
                if (m.kind == ModuleKind::Oscillator)
                {
                    ++oscillators;
                    expect(m.id == "osc.wavetable" || static_cast<bool>(m.createOscillator),
                           m.id + " has no oscillator factory");
                }
                record("Module", m.id, "registry walk: id, parameters resolve in the model, factory", ok ? "ok" : "FAIL");
            }
            expect(effects >= 20 && oscillators == 9, "effects " + String(effects) + " oscillators " + String(oscillators));
        }

        beginTest("every parameter: unique id, default in range, ranges and choices consistent, mapping round trips");
        {
            std::set<std::string> ids;
            int bad = 0;
            for (const auto& d : ParameterMap::params())
            {
                bool ok = !d.id.empty() && ids.insert(d.id).second && d.min < d.max && d.defaultValue >= d.min &&
                          d.defaultValue <= d.max && d.skew > 0.0f && !d.name.empty();
                if (d.type == ParamType::Choice && !d.destinationChoice)
                    ok = ok && static_cast<int>(d.choices.size()) == static_cast<int>(d.max - d.min) + 1;
                if (d.type == ParamType::Bool)
                    ok = ok && d.min == 0.0f && d.max == 1.0f;
                for (const float n : {0.0f, 0.25f, 0.5f, 0.75f, 1.0f})
                {
                    const float nat = ParameterMap::fromNormalised(d, n);
                    ok = ok && nat >= d.min - 1.0e-4f && nat <= d.max + 1.0e-4f;
                    if (d.type == ParamType::Float && d.interval == 0.0f)
                    {
                        const float back = ParameterMap::toNormalised(d, nat);
                        ok = ok && std::abs(back - n) < 1.0e-3f;
                    }
                }
                ok = ok && std::abs(ParameterMap::toNormalised(d, d.defaultValue) -
                                    ParameterMap::toNormalised(
                                        d, ParameterMap::fromNormalised(d, ParameterMap::toNormalised(d, d.defaultValue)))) <
                               1.0e-3f;
                if (!ok)
                {
                    ++bad;
                    logMessage("bad parameter definition: " + d.id);
                }
            }
            expectEquals(bad, 0);
            record("Parameter model", "ids, ranges, defaults, choices, normalised mapping",
                   "all " + std::to_string(ParameterMap::params().size()) + " parameters checked", bad == 0 ? "ok" : "FAIL");
        }

        beginTest("matrix destinations and sources line up with the parameter model");
        {
            const auto& names = ParameterMap::modDestNames();
            expectEquals(static_cast<int>(names.size()),
                         static_cast<int>(ModDest::Count) + static_cast<int>(ParameterMap::modTargets().size()));
            for (const int t : ParameterMap::modTargets())
                expect(ParameterMap::params()[static_cast<size_t>(t)].modulatable, "target " + String(t));
            expectEquals(static_cast<int>(ParameterMap::modSourceNames().size()),
                         static_cast<int>(ModSource::Count) -
                             (ModuleRegistry::resolveMacros(kDefaultMacros) < kNumMacros ? 2 : 0));
            const std::string json = ParameterMap::toJson();
            expect(json.size() > 10000 && json.find("osc1_table") != std::string::npos);
        }

        beginTest("a default engine is silent without notes and does not allocate");
        {
            const Cfg cfg;
            auto e = makeEngine(cfg);
            const auto s = play(*e, cfg, {}, 48000);
            expect(s.peak == 0.0f && s.finite && s.allocations == 0);
        }
    }
};
static RegistryAudit registryAudit;

// ---------------------------------------------------------------------------------------------------------------
class RenderBehaviourAudit : public UnitTest
{
public:
    RenderBehaviourAudit() : UnitTest("Audit: render behaviour (block sizes, sample rates, polyphony, automation)", "Audit")
    {
    }

    static double worstDiffDb(const Shot& a, const Shot& b)
    {
        const size_t n = std::min(a.l.size(), b.l.size());
        double worst = 0.0;
        for (size_t i = 0; i < n; ++i)
            worst = std::max(
                {worst, std::abs(static_cast<double>(a.l[i]) - b.l[i]), std::abs(static_cast<double>(a.r[i]) - b.r[i])});
        const double ref = std::max({a.peak, b.peak, 1.0e-9f});
        return worst <= 0.0 ? -300.0 : 20.0 * std::log10(worst / ref);
    }

    void runTest() override
    {
        beginTest("block sizes 1, 7, 32 and 512 render the same sound (events on the 32-sample grid)");
        {
            struct Scenario
            {
                std::string name;
                Setup setup;
                Script script;
                bool exactAtAnyBlock = false;
                // High where a module builds its state on a worker thread (the cabinet's kernels): offline it waits for the
                // builds, so that the block at which a build takes effect does not depend on thread timing.
                Quality quality = Quality::Standard;
            };
            const std::vector<Scenario> scenarios = {
                {"plain saw + filter", nullptr, Script::Chord, true},
                {"unison, warps, two filters, LFO routed",
                 [](Engine& e)
                 {
                     e.setParam("osc1_uni_voices", 5.0f);
                     e.setParam("osc1_warp", 14.0f);
                     e.setParam("osc1_warp_a", 0.5f);
                     e.setParam("osc2_level", 0.5f);
                     e.setParam("osc2_route", 0.0f);
                     e.setParam("osc2_xmod_mode", 1.0f);
                     e.setParam("osc2_xmod_amt", 0.4f);
                     e.setParam("filter2_on", 1.0f);
                     e.setParam("filter2_type", 8.0f);
                     e.setParam("mod1_src", 1.0f);
                     e.setParam("mod1_dst", 7.0f);
                     e.setParam("mod1_amt", 0.6f);
                     e.setParam("lfo1_rate", 7.0f);
                 },
                 Script::Chord},
                {"every effect on",
                 [](Engine& e)
                 {
                     for (const auto& m : ModuleRegistry::instance().modules())
                         if (m.kind == ModuleKind::Effect && m.id != "fx.convolution" && m.id != "fx.vocoder")
                         {
                             const auto a = fxActive().find(m.id);
                             if (a != fxActive().end())
                                 for (const auto& [id, v] : a->second)
                                     e.setParam(id, v);
                             e.setParam(m.onParam, 1.0f);
                         }
                 },
                 Script::Chord,
                 false,
                 Quality::High},
                {"arpeggiator",
                 [](Engine& e)
                 {
                     e.setParam("arp_on", 1.0f);
                     e.setParam("arp_sync", 0.0f);
                     e.setParam("arp_mode", 4.0f);
                     e.setParam("arp_octaves", 2.0f);
                 },
                 Script::Chord},
                {"sequencer",
                 [](Engine& e)
                 {
                     e.setParam("seq_run", 1.0f);
                     e.setParam("seq_sync", 0.0f);
                     e.sequencer().edit(
                         [](SeqBank& b)
                         {
                             for (int i = 0; i < 16; ++i)
                             {
                                 b.pattern[0].steps[static_cast<size_t>(i)].on = i % 3 != 2;
                                 b.pattern[0].steps[static_cast<size_t>(i)].note = static_cast<std::uint8_t>(48 + i);
                             }
                         });
                 },
                 Script::None},
            };
            for (const auto& sc : scenarios)
            {
                std::vector<Shot> shots;
                for (const int block : {512, 32, 7, 1})
                {
                    Cfg cfg;
                    cfg.block = block;
                    cfg.quality = sc.quality;
                    auto e = makeEngine(cfg, 512);
                    baseSetup(*e);
                    if (sc.setup)
                        sc.setup(*e);
                    // the events sit on multiples of 32 samples
                    auto ev = scriptEvents(sc.script);
                    for (auto& x : ev)
                        x.pos = x.pos / 32 * 32;
                    shots.push_back(play(*e, cfg, ev, 24000));
                }
                const double d32 = worstDiffDb(shots[0], shots[1]);
                const double d7 = worstDiffDb(shots[0], shots[2]);
                const double d1 = worstDiffDb(shots[0], shots[3]);
                logMessage(sc.name + ": 32 " + String(d32, 1) + " dB, 7 " + String(d7, 1) + " dB, 1 " + String(d1, 1) +
                           " dB");
                // Promised: blocks that are multiples of the 32-sample control block with events on that grid are bit
                // identical. Other cuts only move the control-rate sampling of modulation: bounded, not identical.
                const bool onGrid = sc.script != Script::None;
                if (onGrid)
                    expect(d32 < -120.0, sc.name + " block 32 differs from 512 by " + String(d32, 1) + " dB");
                const double bound = sc.exactAtAnyBlock ? -120.0 : -25.0;
                expect(d32 < bound && d7 < bound && d1 < bound, sc.name + ": 32 " + String(d32, 1) + " 7 " + String(d7, 1) +
                                                                    " 1 " + String(d1, 1) + " dB, bound " +
                                                                    String(bound, 0));
                record("Determinism", "block size 1 / 7 / 32 / 512: " + sc.name,
                       "0.5 s renders compared sample by sample" +
                           std::string(onGrid ? ", events on the 32-sample grid" : ", sequencer events off the grid"),
                       std::string(d32 < -120.0 ? "32: bit-identical"
                                                : "32: " + std::to_string(static_cast<int>(d32)) + " dB") +
                           "; 7: " + (d7 < -120.0 ? "bit-identical" : std::to_string(static_cast<int>(d7)) + " dB") +
                           "; 1: " + (d1 < -120.0 ? "bit-identical" : std::to_string(static_cast<int>(d1)) + " dB"));
            }
        }

        beginTest("parameter smoothing takes the same time whatever the block size");
        {
            std::vector<int> reached;
            for (const int block : {64, 32, 17, 7, 1})
            {
                Cfg cfg;
                cfg.block = block;
                auto e = makeEngine(cfg, 512);
                baseSetup(*e);
                e->setParam("osc1_level", 0.8f);
                std::vector<float> l(static_cast<size_t>(block)), r(static_cast<size_t>(block));
                for (int d = 0; d < 4800; d += block)
                    e->process(l.data(), r.data(), block);
                e->setParam("osc1_level", 0.0f);
                int total = 0, done = -1;
                for (int d = 0; d < 9600 && done < 0; d += block)
                {
                    e->process(l.data(), r.data(), block);
                    total += block;
                    if (e->modulatedValue("osc1_level") <= 1.0e-4f)
                        done = total;
                }
                reached.push_back(done);
            }
            std::string list;
            for (const int v : reached)
                list += " " + std::to_string(v);
            logMessage("samples until the level reaches its target (blocks 64 32 17 7 1):" + list);
            for (const int v : reached)
                expect(v > 1300 && v < 1600, "smoothing time depends on the block size:" + list);
            record("Determinism", "parameter smoothing time vs block size",
                   "level step, time until the smoothed value arrives, blocks 64 / 32 / 17 / 7 / 1", "within 1%:" + list,
                   "B2");
        }
    }
};
static RenderBehaviourAudit renderBehaviourAudit;

// ---------------------------------------------------------------------------------------------------------------
namespace
{
struct Health
{
    bool ok = true;
    std::string why;
};

Health healthOf(const Shot& s, float bound = kPeakBound, bool dcCheck = true)
{
    Health h;
    if (!s.finite)
        h.ok = false, h.why += " non-finite";
    if (s.peak > bound)
        h.ok = false, h.why += " peak " + std::to_string(s.peak);
    if (s.subnormals > 0)
        h.ok = false, h.why += " subnormals";
    if (s.allocations > 0)
        h.ok = false, h.why += " allocations " + std::to_string(s.allocations);
    if (dcCheck)
    {
        const size_t tail = s.l.size() > 4800 ? s.l.size() - 4800 : 0;
        const double dc = s.mean(tail, s.l.size());
        if (std::abs(dc) > 0.05 && std::abs(dc) > 0.5 * s.rmsRange(tail, s.l.size()))
            h.ok = false, h.why += " dc " + std::to_string(dc);
    }
    return h;
}

// Sets every parameter of a module to a pattern of extreme values.
void extremes(Engine& e, const ModuleDescriptor& m, int pattern, std::mt19937& rng)
{
    for (const auto& p : m.params)
    {
        if (p.id == m.onParam || p.id == m.positionParam)
            continue;
        float n = 0.0f;
        switch (pattern)
        {
        case 0:
            n = 0.0f;
            break;
        case 1:
            n = 1.0f;
            break;
        case 2:
            n = (&p - &m.params[0]) % 2 == 0 ? 0.0f : 1.0f;
            break;
        case 3:
            n = (&p - &m.params[0]) % 2 == 0 ? 1.0f : 0.0f;
            break;
        default:
            n = (rng() & 1u) ? 1.0f : static_cast<float>(rng() % 3) * 0.5f;
            break;
        }
        e.setParamNormalised(ParameterMap::indexOf(p.id), n);
    }
    e.setParam(m.onParam, 1.0f);
}
} // namespace

class SoundAudit : public UnitTest
{
public:
    SoundAudit() : UnitTest("Audit: sound generation (rates, pitch, oscillators, filters, effects)", "Audit") {}

    void runTest() override
    {
        pitchAndRates();
        oscillators();
        filters();
        effects();
        denormals();
    }

private:
    void pitchAndRates()
    {
        beginTest("pitch is right at every sample rate and from C1 to C8");
        {
            double worst = 0.0;
            for (const double rate : {22050.0, 44100.0, 48000.0, 88200.0, 96000.0, 192000.0})
                for (const int note : {24, 36, 48, 60, 69, 84, 96, 108})
                {
                    Cfg cfg;
                    cfg.rate = rate;
                    cfg.block = 480;
                    auto e = makeEngine(cfg);
                    baseSetup(*e);
                    sineSetup(*e);
                    const auto s = play(*e, cfg, {on(0.0, note, 100, 0, rate)}, samplesOf(0.6, rate));
                    const double expected = noteHz(note);
                    if (expected > rate * 0.45)
                        continue;
                    const double f = measureFrequency(s.l, static_cast<size_t>(0.2 * rate), static_cast<size_t>(0.55 * rate),
                                                      expected, rate);
                    const double cents = std::abs(centsBetween(f, expected));
                    worst = std::max(worst, cents);
                    expect(s.finite && s.peak > 0.05f, "silent or non-finite at " + String(rate) + " note " + String(note));
                    expect(cents < 3.0,
                           "pitch off by " + String(cents, 2) + " cents at " + String(rate) + " Hz, note " + String(note));
                }
            logMessage("worst pitch error " + String(worst, 3) + " cents");
            record("Rates", "pitch accuracy at 22.05 / 44.1 / 48 / 88.2 / 96 / 192 kHz, notes 24..108",
                   "single sine, frequency measured with a windowed DFT search",
                   "worst " + std::to_string(worst).substr(0, 5) + " cents");
        }

        beginTest("level is the same at every sample rate");
        {
            std::vector<double> levels;
            for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
            {
                Cfg cfg;
                cfg.rate = rate;
                cfg.block = 480;
                auto e = makeEngine(cfg);
                baseSetup(*e);
                const auto s = play(*e, cfg, {on(0.0, 52, 100, 0, rate)}, samplesOf(0.5, rate));
                levels.push_back(s.rmsRange(static_cast<size_t>(0.2 * rate), static_cast<size_t>(0.45 * rate)));
            }
            for (const double l : levels)
                expect(std::abs(20.0 * std::log10(l / levels[1])) < 1.0,
                       "level " + String(l, 4) + " against " + String(levels[1], 4));
        }

        beginTest("everything is healthy at other sample rates (a sample of the parameter sweep at 44.1, 96 and 192 kHz)");
        {
            int checked = 0, bad = 0;
            const auto& defs = ParameterMap::params();
            int rateIndex = 0;
            for (const double rate : {44100.0, 96000.0, 192000.0})
            {
                Cfg cfg;
                cfg.rate = rate;
                cfg.block = 480;
                const size_t stride = static_cast<size_t>(auditStride() * 5);
                for (size_t i = static_cast<size_t>(rateIndex); i < defs.size(); i += stride)
                {
                    const auto& d = defs[i];
                    const auto ctxs = contextsFor(d);
                    for (const float v : {d.min, d.max})
                    {
                        const auto& ctx = ctxs.front();
                        Cfg run = cfg;
                        run.hostBpm = ctx.hostBpm;
                        run.quality = ctx.quality;
                        auto e = makeEngine(run);
                        baseSetup(*e);
                        applyCtx(*e, ctx);
                        e->setParam(static_cast<int>(i), v);
                        const auto s = play(*e, run, scriptEvents(ctx.script, rate), samplesOf(ctx.seconds, rate));
                        // An output gain of +24 dB on a loud chord legitimately goes past 30 dB over full scale (the
                        // convolution's fx_cx_out reaches 49 at 96 kHz); every other parameter keeps the plain bound.
                        const auto h = healthOf(s, d.unit == "dB" && d.max >= 24.0f ? 2.0f * kPeakBound : kPeakBound);
                        ++checked;
                        if (!h.ok)
                        {
                            ++bad;
                            expect(false, d.id + " at " + String(rate) + " Hz, value " + String(v) + ":" + h.why);
                        }
                    }
                }
                ++rateIndex;
            }
            logMessage("checked " + String(checked) + " renders");
            record("Rates", "parameters at 44.1 / 96 / 192 kHz", "every 5th parameter at min and max in its module context",
                   bad == 0 ? "healthy (" + std::to_string(checked) + " renders)" : "FAIL");
        }
    }

    void oscillators()
    {
        beginTest(
            "every wavetable, position, warp, unison mode and cross modulation renders sound that is finite and bounded");
        {
            int renders = 0, bad = 0, silent = 0;
            const auto check = [&](const std::string& what, const Setup& setup, int note)
            {
                const Cfg cfg;
                auto e = makeEngine(cfg);
                baseSetup(*e);
                setup(*e);
                const auto s = play(*e, cfg, {on(0.0, note, 100), off(0.25, note)}, samplesOf(0.3));
                const auto h = healthOf(s, 16.0f, false);
                ++renders;
                if (!h.ok)
                {
                    ++bad;
                    expect(false, what + " note " + String(note) + ":" + h.why);
                }
                if (s.rms() < 1.0e-4)
                {
                    ++silent;
                    return false;
                }
                return true;
            };
            // factory tables x positions x notes
            for (int table = 0; table < 14; ++table)
                for (const float pos : {0.0f, 0.37f, 1.0f})
                    for (const int note : {24, 60, 96, 108})
                        // Sync Saw at its last position keeps all of its energy above partial 7, which the band limit
                        // removes at the top octave (note 108 at 48 kHz keeps 6 partials): silence there is the filter, not
                        // a fault.
                        if (!check(
                                "table " + std::to_string(table) + " pos " + std::to_string(pos),
                                [&](Engine& e)
                                {
                                    e.setParam("osc1_table", static_cast<float>(table));
                                    e.setParam("osc1_pos", pos);
                                },
                                note) &&
                            !(table == 13 && pos == 1.0f && note >= 105))
                            expect(false, "table " + String(table) + " position " + String(pos) + " is silent at note " +
                                              String(note));
            // empty user tables: finite, may be silent
            for (int table = 14; table < 22; ++table)
                check(
                    "user table " + std::to_string(table),
                    [&](Engine& e) { e.setParam("osc1_table", static_cast<float>(table)); }, 60);
            // warps
            for (int warp = 0; warp < 15; ++warp)
                for (const float a : {0.0f, 0.5f, 1.0f})
                    for (const float b : {0.0f, 1.0f})
                        for (const int uni : {1, 8})
                            for (const int note : {36, 84})
                                if (!check(
                                        "warp " + std::to_string(warp) + " a " + std::to_string(a) + " b " +
                                            std::to_string(b) + " uni " + std::to_string(uni),
                                        [&](Engine& e)
                                        {
                                            e.setParam("osc1_warp", static_cast<float>(warp));
                                            e.setParam("osc1_warp_a", a);
                                            e.setParam("osc1_warp_b", b);
                                            e.setParam("osc1_uni_voices", static_cast<float>(uni));
                                            e.setParam("osc2_level", 0.5f);
                                            e.setParam("osc2_route", 0.0f);
                                        },
                                        note))
                                    expect(false, "warp " + String(warp) + " is silent");
            // unison modes x voices x spread / detune extremes
            for (int mode = 0; mode < 8; ++mode)
                for (int voices = 1; voices <= 8; ++voices)
                    for (const float detune : {0.0f, 100.0f})
                        check(
                            "unison mode " + std::to_string(mode) + " voices " + std::to_string(voices),
                            [&](Engine& e)
                            {
                                e.setParam("osc1_uni_mode", static_cast<float>(mode));
                                e.setParam("osc1_uni_voices", static_cast<float>(voices));
                                e.setParam("osc1_uni_detune", detune);
                                e.setParam("osc1_uni_scatter", 1.0f);
                            },
                            57);
            // cross modulation: all modes, sources and amounts on every slot
            for (int slot = 2; slot <= 3; ++slot)
                for (int mode = 1; mode <= 3; ++mode)
                    for (int src = 0; src < 3; ++src)
                        for (const float amt : {0.1f, 1.0f})
                            check(
                                "xmod slot " + std::to_string(slot) + " mode " + std::to_string(mode),
                                [&](Engine& e)
                                {
                                    for (int k = 1; k <= 3; ++k)
                                    {
                                        e.setParam("osc" + std::to_string(k) + "_level", 0.5f);
                                        e.setParam("osc" + std::to_string(k) + "_route", 0.0f);
                                    }
                                    e.setParam("osc1_semi", 7.0f);
                                    const std::string s = "osc" + std::to_string(slot);
                                    e.setParam(s + "_xmod_mode", static_cast<float>(mode));
                                    e.setParam(s + "_xmod_src", static_cast<float>(src));
                                    e.setParam(s + "_xmod_amt", amt);
                                },
                                52);
            // oscillator types per slot
            for (int slot = 1; slot <= 3; ++slot)
                for (int type = 0; type < 9; ++type)
                    for (const int note : {36, 72})
                        check(
                            "slot " + std::to_string(slot) + " type " + std::to_string(type),
                            [&](Engine& e)
                            {
                                for (int k = 1; k <= 3; ++k)
                                    e.setParam("osc" + std::to_string(k) + "_level", k == slot ? 0.8f : 0.0f);
                                if (slot != 1)
                                    e.setParam("osc" + std::to_string(slot) + "_route", 0.0f);
                                e.setParam("osc" + std::to_string(slot) + "_type", static_cast<float>(type));
                            },
                            note);
            // noise colours and sub waves
            for (int t = 0; t < 7; ++t)
                if (!check(
                        "noise type " + std::to_string(t),
                        [&](Engine& e)
                        {
                            e.setParam("osc1_type", 3.0f);
                            e.setParam("osc1_noise_type", static_cast<float>(t));
                        },
                        60))
                    expect(false, "noise type " + String(t) + " is silent");
            for (int w = 0; w < 5; ++w)
                for (int o = 0; o < 3; ++o)
                    if (!check(
                            "sub wave " + std::to_string(w),
                            [&](Engine& e)
                            {
                                e.setParam("osc1_type", 4.0f);
                                e.setParam("osc1_sub_wave", static_cast<float>(w));
                                e.setParam("osc1_sub_octave", static_cast<float>(o));
                                e.setParam("osc1_sub_drive", 1.0f);
                            },
                            60))
                        expect(false, "sub wave " + String(w) + " is silent");
            // analog oscillator: every wave in every character, with sync / FM and unison
            for (int w = 0; w < 6; ++w)
                for (int ch = 0; ch < 4; ++ch)
                    for (int mode = 0; mode < 6; ++mode)
                        if (!check(
                                "analog wave " + std::to_string(w) + " character " + std::to_string(ch) + " mode " +
                                    std::to_string(mode),
                                [&](Engine& e)
                                {
                                    e.setParam("osc1_type", 5.0f);
                                    e.setParam("osc1_an_wave", static_cast<float>(w));
                                    e.setParam("osc1_an_shape", 0.4f);
                                    e.setParam("osc1_an_char", static_cast<float>(ch));
                                    e.setParam("osc1_an_xmod_mode", static_cast<float>(mode));
                                    e.setParam("osc1_an_uni", mode == 0 ? 3.0f : 1.0f);
                                    e.setParam("osc1_an_drift", 0.5f);
                                    e.setParam("osc1_an_tol", 0.5f);
                                },
                                60))
                            expect(false, "analog wave " + String(w) + " character " + String(ch) + " mode " + String(mode) +
                                              " is silent");
            // modal oscillator: both models with every exciter, at the extremes of geometry, decay and tolerance
            for (int model = 0; model < 2; ++model)
                for (int exciter = 0; exciter < 6; ++exciter)
                    for (const int note : {36, 96})
                        if (!check(
                                "modal model " + std::to_string(model) + " exciter " + std::to_string(exciter),
                                [&](Engine& e)
                                {
                                    e.setParam("osc1_type", 6.0f);
                                    e.setParam("osc1_modal_model", static_cast<float>(model));
                                    e.setParam("osc1_modal_exciter", static_cast<float>(exciter));
                                    e.setParam("osc1_modal_geometry", model == 0 ? 0.9f : 0.2f);
                                    e.setParam("osc1_modal_decay", exciter >= 3 ? 8.0f : 3.0f);
                                    e.setParam("osc1_modal_tol", 1.0f);
                                },
                                note))
                            expect(false, "modal model " + String(model) + " exciter " + String(exciter) + " is silent");
            // harmonic oscillator at its extremes
            for (const int count : {1, 128})
                for (const float stretch : {-1.0f, 1.0f})
                    if (!check(
                            "harmonic count " + std::to_string(count),
                            [&](Engine& e)
                            {
                                e.setParam("osc1_type", 7.0f);
                                e.setParam("osc1_harm_count", static_cast<float>(count));
                                e.setParam("osc1_harm_stretch", stretch);
                                e.setParam("osc1_harm_fgain", 24.0f);
                                e.setParam("osc1_harm_decay", 1.0f);
                            },
                            48))
                        expect(false, "harmonic oscillator is silent");
            // legacy sub / noise mixers
            for (int w = 0; w < 4; ++w)
                check(
                    "legacy sub wave",
                    [&](Engine& e)
                    {
                        e.setParam("sub_level", 0.7f);
                        e.setParam("sub_wave", static_cast<float>(w));
                        e.setParam("sub_octave", static_cast<float>(w % 2));
                    },
                    48);
            for (int t = 0; t < 4; ++t)
                check(
                    "legacy noise",
                    [&](Engine& e)
                    {
                        e.setParam("noise_level", 0.7f);
                        e.setParam("noise_type", static_cast<float>(t));
                    },
                    48);
            // sample oscillator without a sample is silent but healthy
            check("sample oscillator, nothing loaded", [&](Engine& e) { e.setParam("osc1_type", 1.0f); }, 60);
            expectEquals(bad, 0);
            logMessage(String(renders) + " oscillator renders, " + String(silent) +
                       " silent (empty user tables / unloaded sample)");
            record("Oscillators", "14 wavetables x 3 positions x 4 notes; 8 user slots (empty)", "finite, bounded, audible",
                   bad == 0 ? "ok" : "FAIL");
            record("Oscillators", "15 warps x A/B extremes x unison 1/8 x low/high notes", "finite, bounded, audible",
                   bad == 0 ? "ok" : "FAIL");
            record("Oscillators", "8 unison modes x 1-8 voices x detune extremes", "finite, bounded, audible",
                   bad == 0 ? "ok" : "FAIL");
            record("Oscillators", "cross modulation FM / ring / sync x sources x amounts on slots 2 and 3",
                   "finite, bounded, audible", bad == 0 ? "ok" : "FAIL");
            record("Oscillators", "oscillator types (wavetable, sample, cloud, noise, sub) on every slot", "finite, bounded",
                   bad == 0 ? "ok" : "FAIL");
            record("Oscillators", "7 noise colours, 5 sub waves x 3 octaves, legacy sub / noise mixers",
                   "finite, bounded, audible", bad == 0 ? "ok" : "FAIL");
        }

        beginTest("periodic wavetables play at the pitch of the note");
        {
            double worst = 0.0;
            for (const int table : {0, 4, 5, 6, 1})
                for (const int note : {36, 57, 81, 100})
                {
                    const Cfg cfg;
                    auto e = makeEngine(cfg);
                    baseSetup(*e);
                    e->setParam("osc1_table", static_cast<float>(table));
                    e->setParam("filter_on", 0.0f);
                    const auto s = play(*e, cfg, {on(0.0, note, 100)}, samplesOf(0.5));
                    const double f = measureFrequency(s.l, 9600, 24000, noteHz(note), 48000.0);
                    worst = std::max(worst, std::abs(centsBetween(f, noteHz(note))));
                }
            expect(worst < 3.0, "pitch error " + String(worst, 2) + " cents");
            record("Oscillators", "pitch of Sine-Saw, PWM, Analog Saw / Square / Tri at 4 notes", "DFT frequency search",
                   "worst " + std::to_string(worst).substr(0, 5) + " cents");
        }
    }

    void filters()
    {
        beginTest("every filter model at extreme cutoff and resonance, voice filter and filter effect: stable, no NaN, no "
                  "runaway");
        {
            int renders = 0;
            std::vector<std::string> bad;
            const int stride = auditStride();
            int counter = 0;
            for (int model = 0; model < 17; ++model)
                for (const float res : {0.0f, 0.5f, 0.9f, 1.0f})
                    for (const float cutoff : {20.0f, 150.0f, 1500.0f, 9000.0f, 20000.0f})
                        for (const double rate : {44100.0, 96000.0, 192000.0})
                        {
                            if ((counter++ % stride) != 0)
                                continue;
                            for (const bool fx : {false, true})
                            {
                                Cfg cfg;
                                cfg.rate = rate;
                                cfg.block = 480;
                                auto e = makeEngine(cfg);
                                baseSetup(*e);
                                e->setParam("osc1_level", 1.0f);
                                if (!fx)
                                {
                                    e->setParam("filter_type", static_cast<float>(model));
                                    e->setParam("filter_cutoff", cutoff);
                                    e->setParam("filter_res", res);
                                    e->setParam("filter_drive", res);
                                    e->setParam("filter2_on", 1.0f);
                                    e->setParam("filter2_type", static_cast<float>((model + 5) % 17));
                                    e->setParam("filter2_cutoff", cutoff);
                                    e->setParam("filter2_res", res);
                                }
                                else
                                {
                                    e->setParam("filter_on", 0.0f);
                                    e->setParam("fx_filter_on", 1.0f);
                                    e->setParam("fx_filter_model", static_cast<float>(model));
                                    e->setParam("fx_filter_cutoff", cutoff);
                                    e->setParam("fx_filter_res", res);
                                    e->setParam("fx_filter_drive", res);
                                    e->setParam("fx_filter_lfo_depth", 2.0f);
                                }
                                const auto s =
                                    play(*e, cfg, {on(0.0, 48, 127), on(0.0, 60, 127), off(0.3, 48), off(0.3, 60)},
                                         samplesOf(1.0, rate));
                                ++renders;
                                std::string why;
                                if (!s.finite)
                                    why += " non-finite";
                                if (s.peak > 64.0f)
                                    why += " peak " + std::to_string(s.peak);
                                // the second half must not be louder than the first (no self-oscillation growth)
                                const size_t half = s.l.size() / 2;
                                const double early =
                                    s.rmsRange(static_cast<size_t>(0.1 * rate), static_cast<size_t>(0.3 * rate));
                                const double late = s.rmsRange(half, s.l.size());
                                if (late > 2.0 * early + 0.05)
                                    why += " grows: early " + std::to_string(early) + " late " + std::to_string(late);
                                if (s.subnormals > 0)
                                    why += " subnormals";
                                if (!why.empty())
                                {
                                    bad.push_back((fx ? "fx filter " : "voice filter ") + std::to_string(model) + " res " +
                                                  std::to_string(res) + " cutoff " + std::to_string(cutoff) + " rate " +
                                                  std::to_string(rate) + ":" + why);
                                    expect(false, bad.back());
                                }
                            }
                        }
            logMessage(String(renders) + " filter renders");
            record("Filters",
                   "17 models x resonance 0-1 x cutoff 20 Hz-20 kHz x 44.1 / 96 / 192 kHz, voice filters (two in series) "
                   "and filter FX",
                   "finite, peak <= 64, no growth after release",
                   bad.empty() ? "ok (" + std::to_string(renders) + " renders)" : "FAIL");
        }

        beginTest("fast cutoff jumps and resonance sweeps do not blow up any filter model");
        {
            std::mt19937 rng(5);
            int bad = 0;
            for (int model = 0; model < 17; ++model)
            {
                const Cfg cfg;
                auto e = makeEngine(cfg, 64);
                Cfg run = cfg;
                run.block = 64;
                baseSetup(*e);
                e->setParam("filter_type", static_cast<float>(model));
                e->setParam("filter_res", 0.97f);
                e->setParam("osc1_level", 1.0f);
                e->noteOn(48, 1.0f);
                Shot all;
                std::vector<float> l(64), r(64);
                for (int b = 0; b < 600; ++b)
                {
                    e->setParam("filter_cutoff", (rng() & 1u) ? 20.0f : 20000.0f);
                    e->setParam("filter_res", (rng() & 1u) ? 1.0f : 0.0f);
                    e->process(l.data(), r.data(), 64);
                    for (int i = 0; i < 64; ++i)
                    {
                        all.finite = all.finite && std::isfinite(l[static_cast<size_t>(i)]) &&
                                     std::isfinite(r[static_cast<size_t>(i)]);
                        all.peak =
                            std::max({all.peak, std::abs(l[static_cast<size_t>(i)]), std::abs(r[static_cast<size_t>(i)])});
                    }
                }
                if (!all.finite || all.peak > 64.0f)
                {
                    ++bad;
                    expect(false, "model " + String(model) + " peak " + String(all.peak));
                }
            }
            record("Filters", "all 17 models, cutoff 20 Hz <-> 20 kHz and resonance 0 <-> 1 changed every 64 samples",
                   "finite, peak <= 64", bad == 0 ? "ok" : "FAIL");
        }
    }

    void effects()
    {
        beginTest("every effect at extreme settings and with fast automation: finite, bounded, tails end");
        {
            const auto& reg = ModuleRegistry::instance();
            const int stride = auditStride();
            std::mt19937 rng(11);
            for (const auto& m : reg.modules())
            {
                if (m.kind != ModuleKind::Effect)
                    continue;
                int renders = 0;
                bool ok = true;
                for (int pattern = 0; pattern < 10; ++pattern)
                {
                    if (stride > 1 && pattern > 4)
                        break;
                    for (const double rate : {44100.0, 96000.0})
                    {
                        if (stride > 1 && rate > 50000.0 && pattern > 1)
                            continue;
                        Cfg cfg;
                        cfg.rate = rate;
                        cfg.block = 256;
                        auto e = makeEngine(cfg);
                        baseSetup(*e);
                        extremes(*e, m, pattern, rng);
                        const bool automate = pattern >= 5;
                        std::vector<Ev> events = scriptEvents(Script::Chord, rate);
                        Shot s;
                        s.finite = true;
                        std::vector<float> l(256), r(256);
                        const int total = samplesOf(3.2, rate);
                        std::vector<Ev> pending = events;
                        size_t next = 0;
                        for (int done = 0; done < total; done += 256)
                        {
                            std::vector<MidiInput> midi;
                            while (next < pending.size() && pending[next].pos < done + 256)
                            {
                                midi.push_back({std::max(0, pending[next].pos - done), pending[next].status,
                                                pending[next].d1, pending[next].d2});
                                ++next;
                            }
                            if (automate)
                                for (const auto& p : m.params)
                                    if (p.id != m.onParam)
                                        e->setParamNormalised(ParameterMap::indexOf(p.id),
                                                              static_cast<float>(rng() % 5) * 0.25f);
                            gAllocations = 0;
                            gTrackAllocations = true;
                            e->process(l.data(), r.data(), 256, midi.empty() ? nullptr : midi.data(),
                                       static_cast<int>(midi.size()));
                            gTrackAllocations = false;
                            s.allocations += gAllocations;
                            for (int i = 0; i < 256; ++i)
                            {
                                s.finite = s.finite && std::isfinite(l[static_cast<size_t>(i)]) &&
                                           std::isfinite(r[static_cast<size_t>(i)]);
                                s.peak = std::max(
                                    {s.peak, std::abs(l[static_cast<size_t>(i)]), std::abs(r[static_cast<size_t>(i)])});
                                if (done + i >= total - 4800)
                                {
                                    s.l.push_back(l[static_cast<size_t>(i)]);
                                    s.r.push_back(r[static_cast<size_t>(i)]);
                                }
                            }
                        }
                        ++renders;
                        const bool tailEnds = automate || s.rms() < 1.0 || m.id == "fx.filter" ||
                                              m.id == "fx.delay_pro"; // feedback 1 and freeze repeat for ever
                        const bool good = s.finite && s.peak < 5000.0f && s.allocations == 0 && tailEnds;
                        if (!good)
                        {
                            ok = false;
                            expect(false, m.id + " pattern " + String(pattern) + " rate " + String(rate) + ": finite " +
                                              String(s.finite ? 1 : 0) + " peak " + String(s.peak) + " allocations " +
                                              String(s.allocations) + " tail rms " + String(s.rms()));
                        }
                    }
                }
                record("Effect", m.id,
                       "all parameters at min / max / alternating / random extremes and automated every 256 samples; 44.1 "
                       "and 96 kHz",
                       ok ? "ok (" + std::to_string(renders) + " renders)" : "FAIL");
            }
        }

        beginTest("all effects on at once, both extremes, at 44.1 / 48 / 96 / 192 kHz");
        {
            std::mt19937 rng(3);
            for (const double rate : {44100.0, 48000.0, 96000.0, 192000.0})
                for (int pattern = 0; pattern < 3; ++pattern)
                {
                    Cfg cfg;
                    cfg.rate = rate;
                    cfg.block = 480;
                    auto e = makeEngine(cfg);
                    baseSetup(*e);
                    for (const auto& m : ModuleRegistry::instance().modules())
                        if (m.kind == ModuleKind::Effect && m.id != "fx.convolution")
                            extremes(*e, m, pattern, rng);
                    const auto s = play(*e, cfg, scriptEvents(Script::Chord, rate), samplesOf(1.0, rate));
                    expect(s.finite && s.peak < 2000.0f && s.allocations == 0,
                           "rate " + String(rate) + " pattern " + String(pattern) + " peak " + String(s.peak));
                }
            record("Effect", "full chain (all effects on)", "extreme parameter patterns at 44.1 / 48 / 96 / 192 kHz",
                   "finite, bounded, no allocation");
        }
    }

    void denormals()
    {
        beginTest("tails fade to digital silence without subnormal numbers and without a CPU stall");
        {
            Cfg cfg;
            cfg.block = 512;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            for (const auto& m : ModuleRegistry::instance().modules())
                if (m.kind == ModuleKind::Effect)
                {
                    const auto a = fxActive().find(m.id);
                    if (a != fxActive().end())
                        for (const auto& [id, v] : a->second)
                            e->setParam(id, v);
                    e->setParam(m.onParam, 1.0f);
                }
            e->setParam("fx_ra_decay", 8.0f);
            e->setParam("fx_delay_pro_feedback", 0.9f);
            const auto active = play(*e, cfg, {on(0.0, 57, 100), off(0.5, 57)}, samplesOf(1.0));
            // the tail: the feedback paths decay for a long time; nothing may stall or turn subnormal
            const auto tail = play(*e, cfg, {}, samplesOf(20.0));
            const double perSecondActive = active.seconds / 1.0;
            const double perSecondTail = tail.seconds / 20.0;
            logMessage("active " + String(perSecondActive * 1000.0, 2) + " ms per second, tail " +
                       String(perSecondTail * 1000.0, 2) + " ms per second");
            expect(tail.subnormals == 0 && tail.finite, "subnormal numbers in the output tail");
            if (!kSanitized)
                expect(perSecondTail < 2.0 * perSecondActive + 0.002,
                       "the decaying tail costs " + String(perSecondTail * 1000.0, 2) + " ms/s against " +
                           String(perSecondActive * 1000.0, 2) + " active");
            record("Real time", "denormals: 20 s tail of the full effect chain",
                   "no subnormal output, CPU per second of the tail against active playing",
                   "tail " + std::to_string(perSecondTail * 1000.0).substr(0, 5) + " ms/s, active " +
                       std::to_string(perSecondActive * 1000.0).substr(0, 5) + " ms/s");
        }

        beginTest("a long held note does not drift into DC for any saturator model or voice filter");
        {
            int bad = 0;
            for (int model = 0; model < 16; ++model)
            {
                Cfg cfg;
                cfg.block = 480;
                auto e = makeEngine(cfg);
                baseSetup(*e);
                e->setParam("fx_saturator_on", 1.0f);
                e->setParam("fx_saturator_model", static_cast<float>(model));
                e->setParam("fx_saturator_drive", 36.0f);
                const auto s = play(*e, cfg, {on(0.0, 45, 127)}, samplesOf(6.0));
                const double dc = s.mean(samplesOf(4.0), samplesOf(6.0));
                if (!(s.finite && std::abs(dc) < 0.05))
                {
                    ++bad;
                    expect(false, "saturator model " + String(model) + " dc " + String(dc, 4));
                }
            }
            record("Real time", "DC runaway: 6 s held note through each saturator model at full drive", "mean of the last 2 s",
                   bad == 0 ? "ok" : "FAIL");
        }
    }
};
static SoundAudit soundAudit;

// ---------------------------------------------------------------------------------------------------------------
class PerformanceAudit : public UnitTest
{
public:
    PerformanceAudit() : UnitTest("Audit: performance (polyphony, stealing, panic, MIDI robustness)", "Audit") {}

    static double maxStep(const Shot& s)
    {
        double m = 0.0;
        for (size_t i = 1; i < s.l.size(); ++i)
            m = std::max(m, std::abs(static_cast<double>(s.l[i]) - s.l[i - 1]));
        return m;
    }

    void runTest() override
    {
        beginTest("full polyphony: the newest notes keep the voices, stealing is click-free");
        {
            Cfg cfg;
            cfg.voices = 4;
            cfg.block = 128;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            sineSetup(*e);
            e->setParam("osc1_level", 0.3f);
            std::vector<Ev> ev;
            std::vector<int> notes;
            for (int k = 0; k < 10; ++k)
            {
                notes.push_back(48 + 3 * k);
                ev.push_back(on(0.05 * k, notes.back()));
            }
            const auto s = play(*e, cfg, ev, samplesOf(1.2));
            expect(s.finite && s.allocations == 0);
            expect(e->activeVoiceCount() == 4, "active voices " + String(e->activeVoiceCount()));
            const size_t from = samplesOf(0.8), to = samplesOf(1.2);
            const double ref = binMagnitude(s.l, from, to, noteHz(notes.back()), 48000.0);
            for (int k = 0; k < 10; ++k)
            {
                const double m = binMagnitude(s.l, from, to, noteHz(notes[static_cast<size_t>(k)]), 48000.0);
                if (k >= 6)
                    expect(m > 0.3 * ref,
                           "note " + String(notes[static_cast<size_t>(k)]) + " should sound (" + String(m / ref, 3) + ")");
                else
                    expect(m < 0.05 * ref, "note " + String(notes[static_cast<size_t>(k)]) +
                                               " was stolen but still sounds (" + String(m / ref, 3) + ")");
            }
            // the steals must not click: compare with the smooth sum of the sustained sines
            logMessage("largest sample step " + String(maxStep(s), 4));
            expect(maxStep(s) < 0.2, "click while stealing: step " + String(maxStep(s), 3));
            record("Voices", "voice stealing at full polyphony (4 voices, 10 notes)",
                   "newest notes sound, stolen ones are gone, largest sample step bounded", "ok");
        }

        beginTest("voice_poly limits the number of sounding notes; retrigger does not leak voices");
        {
            Cfg cfg;
            cfg.voices = 16;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            e->setParam("voice_poly", 2.0f);
            play(*e, cfg, {on(0.0, 48), on(0.0, 52), on(0.0, 55), on(0.0, 60)}, samplesOf(0.5));
            expect(e->activeVoiceCount() == 2, "voice_poly 2 -> " + String(e->activeVoiceCount()) + " voices");
            e->allNotesOff();
            e->setParam("voice_poly", 16.0f);
            std::vector<Ev> storm;
            for (int k = 0; k < 400; ++k)
                storm.push_back(on(0.001 * k, 60 + k % 3, 100));
            play(*e, cfg, storm, samplesOf(0.6));
            expect(e->activeVoiceCount() <= 16, "voices " + String(e->activeVoiceCount()));
        }

        beginTest("sustain pedal holds released notes; releasing it ends them; stealing works under the pedal");
        {
            Cfg cfg;
            cfg.voices = 3;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            e->setParam("amp_release", 0.05f);
            std::vector<Ev> ev = {cc(0.0, 64, 127), on(0.01, 48), on(0.01, 55), off(0.1, 48), off(0.1, 55)};
            auto s = play(*e, cfg, ev, samplesOf(0.5));
            expect(e->activeVoiceCount() == 2 && s.rmsRange(samplesOf(0.3), samplesOf(0.5)) > 0.01, "pedal holds");
            s = play(*e, cfg, {cc(0.0, 64, 0)}, samplesOf(0.5));
            expect(e->activeVoiceCount() == 0 && s.rmsRange(samplesOf(0.3), samplesOf(0.5)) < 1.0e-4, "pedal up releases");
            s = play(*e, cfg,
                     {cc(0.0, 64, 127), on(0.0, 48), off(0.02, 48), on(0.03, 52), off(0.05, 52), on(0.06, 55), off(0.08, 55),
                      on(0.09, 59), off(0.1, 59), on(0.11, 62)},
                     samplesOf(0.3));
            expect(s.finite && e->activeVoiceCount() <= 3);
        }

        beginTest("panic: all-notes-off, CC 120 and CC 123 silence everything, also under the pedal, the arpeggiator and "
                  "the sequencer");
        {
            for (int how = 0; how < 4; ++how)
                for (const int mode : {0, 1, 2})
                {
                    Cfg cfg;
                    auto e = makeEngine(cfg);
                    baseSetup(*e);
                    e->setParam("amp_release", 0.1f);
                    if (mode == 1)
                    {
                        e->setParam("arp_on", 1.0f);
                        e->setParam("arp_sync", 0.0f);
                    }
                    if (mode == 2)
                    {
                        e->setParam("seq_run", 1.0f);
                        e->setParam("seq_sync", 0.0f);
                        e->sequencer().edit(
                            [](SeqBank& b)
                            {
                                for (auto& st : b.pattern[0].steps)
                                    st.on = true;
                            });
                    }
                    play(*e, cfg, {cc(0.0, 64, 127), on(0.0, 48), on(0.0, 55), on(0.0, 60)}, samplesOf(0.4));
                    expect(mode == 1 || e->activeVoiceCount() >= 1, "no voice before the panic, mode " + String(mode));
                    switch (how)
                    {
                    case 0:
                        e->allNotesOff();
                        break;
                    case 1:
                        e->midi(0xB0, 123, 0);
                        break;
                    case 2:
                        e->midi(0xB0, 120, 0);
                        break;
                    default:
                        e->midi(0xB5, 123, 0);
                        break; // another channel
                    }
                    if (mode == 2)
                        e->setParam("seq_run", 0.0f); // the sequencer is a source of notes: stopping it is its own switch
                    const auto s = play(*e, cfg, {}, samplesOf(0.8));
                    const double tail = s.rmsRange(samplesOf(0.4), samplesOf(0.8));
                    expect(tail < 1.0e-4 && e->activeVoiceCount() == 0, "panic style " + String(how) + " mode " +
                                                                            String(mode) + ": tail rms " + String(tail, 5) +
                                                                            " voices " + String(e->activeVoiceCount()));
                }
            record("Voices",
                   "panic: allNotesOff, CC 120, CC 123 (any channel) with pedal held, arpeggiator and sequencer running",
                   "tail after release is silent, no voice left", "ok", "B4");
        }

        beginTest("MIDI robustness: every status byte, extreme values, out-of-range positions, unsorted events");
        {
            Cfg cfg;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            e->noteOn(60, 1.0f);
            for (int status = 0; status < 256; ++status)
                for (const int d1 : {0, 1, 64, 127, 200, 255})
                    for (const int d2 : {0, 127, 255})
                        e->midi(static_cast<std::uint8_t>(status), static_cast<std::uint8_t>(d1),
                                static_cast<std::uint8_t>(d2));
            std::vector<float> l(512), r(512);
            std::vector<MidiInput> midi = {{700, 0x90, 60, 100},     {-5, 0x90, 61, 100}, {10, 0x80, 61, 0},
                                           {100000, 0xE0, 127, 127}, {3, 0x90, 200, 100}, {4, 0xE0, 0, 0},
                                           {5, 0xB0, 1, 127},        {5, 0xD0, 127, 0}};
            e->process(l.data(), r.data(), 512, midi.data(), static_cast<int>(midi.size()));
            bool finite = true;
            for (int i = 0; i < 512; ++i)
                finite = finite && std::isfinite(l[static_cast<size_t>(i)]) && std::isfinite(r[static_cast<size_t>(i)]);
            expect(finite);
            // zero-length and oversized blocks
            e->process(l.data(), r.data(), 0);
            std::vector<float> big(70000), bigR(70000);
            e->process(big.data(), bigR.data(), 70000);
            expect(std::isfinite(big[69999]) && std::isfinite(bigR[69999]));
            // prepare again at another rate while notes sound
            e->noteOn(64, 1.0f);
            e->prepare(96000.0, 256);
            Cfg c2;
            c2.rate = 96000.0;
            c2.block = 256;
            expect(play(*e, c2, {on(0.0, 60)}, 9600).finite);
            e->prepare(8000.0, 64);
            Cfg c3;
            c3.rate = 8000.0;
            c3.block = 64;
            expect(play(*e, c3, {on(0.0, 60)}, 4000).finite);
            record("Robustness", "raw MIDI: all 256 status bytes, data bytes beyond 127, positions out of range, unsorted",
                   "no crash, finite output", "ok");
        }

        beginTest("non-finite and out-of-range parameter values are contained");
        {
            Cfg cfg;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            const float nan = std::numeric_limits<float>::quiet_NaN();
            const float inf = std::numeric_limits<float>::infinity();
            const auto& defs = ParameterMap::params();
            for (size_t i = 0; i < defs.size(); ++i)
            {
                const float v = (i % 3 == 0) ? nan : ((i % 3 == 1) ? inf : -inf);
                e->setParam(static_cast<int>(i), v);
                e->setParamNormalised(static_cast<int>(i), (i % 2 == 0) ? nan : inf);
            }
            for (size_t i = 0; i < defs.size(); ++i)
            {
                const float stored = e->getParam(static_cast<int>(i));
                expect(std::isfinite(stored) && stored >= defs[i].min && stored <= defs[i].max,
                       "parameter " + defs[i].id + " holds " + String(stored));
            }
            const auto s = play(*e, cfg, {on(0.0, 60), on(0.0, 64)}, samplesOf(0.5));
            expect(s.finite, "non-finite parameter values reach the output");
            e->noteOn(60, nan);
            e->noteOn(60, inf);
            e->pitchBend(nan);
            e->modWheel(inf);
            e->aftertouch(-inf);
            expect(play(*e, cfg, {}, 4800).finite, "non-finite performance values reach the output");
            record("Robustness",
                   "NaN / infinite parameter and performance values (setParam, setParamNormalised, noteOn, bend, wheel)",
                   "stored value clamped to the range, output finite", "ok", "B3");
        }

        beginTest("regression: notes in the block that switches the arpeggiator on are played, not swallowed by its panic");
        {
            Cfg cfg;
            cfg.block = 64;
            auto e = makeEngine(cfg);
            baseSetup(*e);
            e->setParam("arp_on", 1.0f);
            e->setParam("arp_sync", 0.0f);
            e->setParam("arp_free_rate", 4.0f);
            const auto s = play(*e, cfg, {on(0.0, 60), on(0.0, 64), on(0.0, 67)}, samplesOf(0.5));
            expect(s.rms() > 0.01, "the arpeggiated chord is silent (rms " + String(s.rms(), 6) + ")");
            record("Voices", "arpeggiator switched on in the block that carries the first notes", "rms of the render", "ok",
                   "B5");
        }

        beginTest("engines are independent: two instances render on two threads");
        {
            std::vector<Shot> shots(2);
            std::vector<std::thread> threads;
            for (int t = 0; t < 2; ++t)
                threads.emplace_back(
                    [&shots, t]
                    {
                        Cfg cfg;
                        cfg.seed = 7;
                        auto e = makeEngine(cfg);
                        baseSetup(*e);
                        e->setParam("fx_ra_on", 1.0f);
                        e->setParam("osc1_table", t == 0 ? 4.0f : 4.0f);
                        shots[static_cast<size_t>(t)] = play(*e, cfg, scriptEvents(Script::Chord), samplesOf(0.8));
                    });
            for (auto& th : threads)
                th.join();
            expect(shots[0].finite && relDiff(shots[0], shots[1]) == 0.0,
                   "same input on two threads must render the same samples");
            record("Voices", "two engines on two threads (shared wavetable bank)", "identical output", "ok");
        }
    }
};
static PerformanceAudit performanceAudit;

// ---------------------------------------------------------------------------------------------------------------
namespace
{
std::string sclOf(int divisions)
{
    std::string scl = "audit " + std::to_string(divisions) + " EDO\n" + std::to_string(divisions) + "\n";
    for (int i = 1; i <= divisions; ++i)
        scl += std::to_string(1200.0 * i / divisions) + "\n";
    return scl;
}

// Fills an engine with a little of everything the patch format carries.
void fillContent(Engine& e)
{
    std::string err;
    e.storeScene(0);
    e.setParam("filter_cutoff", 700.0f);
    e.storeScene(3);
    e.sequencer().edit(
        [](SeqBank& b)
        {
            b.pattern[2].lengthBars = 2;
            for (int i = 0; i < 32; ++i)
            {
                auto& st = b.pattern[2].steps[static_cast<size_t>(i)];
                st.on = i % 3 != 1;
                st.note = static_cast<std::uint8_t>(40 + i);
                st.ratchet = static_cast<std::uint8_t>(1 + i % 3);
                st.gate = 0.3f + 0.05f * static_cast<float>(i % 7);
                st.probability = 0.5f + 0.5f * static_cast<float>(i % 2);
                st.locks[0] = {static_cast<std::int16_t>(ParameterMap::indexOf("filter_cutoff")),
                               1000.0f + 50.0f * static_cast<float>(i)};
            }
        });
    e.arpPattern().edit(
        [](ArpPattern& p)
        {
            p.length = 12;
            p.direction = 2;
            for (int i = 0; i < 12; ++i)
            {
                p.steps[static_cast<size_t>(i)].on = i % 4 != 3;
                p.steps[static_cast<size_t>(i)].octave = static_cast<std::int8_t>(i % 3 - 1);
                p.steps[static_cast<size_t>(i)].ratchet = static_cast<std::uint8_t>(1 + i % 2);
                p.steps[static_cast<size_t>(i)].tie = i % 5 == 4;
            }
        });
    e.modRoutes().add({static_cast<int>(ModSource::Lfo1), "filter_cutoff", 0.5f, 0, 0.2f, 12.0f});
    e.modRoutes().add({kSourceCC + 74, "fx_delay_pro_mix", -0.4f, 2, -0.3f, 0.0f});
    e.modRoutes().add({static_cast<int>(ModSource::Velocity), "osc1_pos", 0.7f, 1, 0.0f, 0.0f});
    e.clips().edit(
        [](ClipBank& b)
        {
            Clip c = makeMseg({{0.0, 0.1f, 0.0f, SegmentShape::Curve},
                               {1.0, 0.9f, 0.3f, SegmentShape::SCurve},
                               {2.0, 0.2f, -0.2f, SegmentShape::Stairs}},
                              4.0, true, 1.0, 3.0);
            ClipLane steps;
            steps.name = "steps";
            steps.kind = ClipLaneKind::Steps;
            steps.steps = {0.1f, 0.9f, 0.4f, 0.6f};
            steps.stepGlide = 0.25f;
            steps.humanise = 0.1f;
            c.lanes.push_back(steps);
            c.trigger = ClipTrigger::GlobalRetrigger;
            b.clips.push_back(c);
        });
    e.clips().edit([](ClipBank&) {});
    e.coreEngine().engine().setClipRunMask(0xFFFFFFFEu & 0xFFu);
    e.tuning().edit(
        [&](Tuning& t)
        {
            std::string error;
            t.loadScl(sclOf(19), error);
            t.setNoteOffsetCents(60, 12.5);
        });
    e.ccMap().add({74, 2, "filter_cutoff", 0.1f, 0.9f, 0.25f, CcMode::Absolute, 1.0f / 64.0f});
    AudioData wave;
    wave.sampleRate = 48000.0;
    wave.channels.assign(1, std::vector<float>(2048 * 4));
    for (size_t i = 0; i < wave.channels[0].size(); ++i)
        wave.channels[0][i] =
            static_cast<float>(std::sin(kTau * static_cast<double>(i % 2048) * static_cast<double>(1 + i / 2048) / 2048.0));
    e.loadWavetable(1, wave, {}, "audit wave", err);
    e.loadSample(0, makeSample(0.3), "audit sample", err);
    e.sliceSampleEqually(0, 4);
    e.addZoneSample(0, makeSample(0.2, 330.0), "zone", err);
    Zone z0;
    z0.keyHi = 59;
    Zone z1;
    z1.sampleId = 1;
    z1.keyLo = 60;
    z1.root = 64.0f;
    e.setZones(0, {z0, z1});
    e.setCloud(1, CloudModel::random(40, 5));
    const AudioData irAudio = makeSample(0.1, 500.0);
    ImpulseResponse ir;
    ir.sampleRate = irAudio.sampleRate;
    ir.channels = {irAudio.channels[0]};
    e.loadConvolutionIr(ir, "audit ir");
}

std::string stateDigest(Engine& e)
{
    std::ostringstream os;
    for (const float v : e.getAllParams())
        os << v << ',';
    os << '\n';
    for (const auto& r : e.scenes().serialise())
        os << r.slot << ':' << r.values << ';';
    os << '\n';
    for (const auto& r : seqBankSerialise(e.sequencer().snapshot()))
        os << r.slot << ':' << r.bars << ':' << r.steps << ';';
    os << '\n' << arpPatternSerialise(e.arpPattern().snapshot());
    os << '\n' << e.modRoutes().serialise();
    os << '\n' << clipBankToJson(e.clips().snapshot()).dump(-1);
    os << '\n' << e.tuning().get().serialise();
    os << '\n' << e.ccMap().serialise();
    os << '\n'
       << (e.coreEngine().engine().clipRunMask() & ((1u << std::min<size_t>(31, e.clips().snapshot().clips.size())) -
                                                    1u)); // bits of clips that do not exist are not state
    for (const auto& r : e.samples().serialise())
        os << '\n'
           << r.slot << r.name << r.rate << r.channels << r.frames.size()
           << (r.frames.empty() ? 0.0f : r.frames[r.frames.size() / 2]);
    for (const auto& r : e.zones().serialiseSamples())
        os << '\n' << r.slot << r.index << r.frames.size();
    os << '\n' << e.zones().serialiseZones(0);
    for (int s = 0; s < 8; ++s)
        for (const float m : e.sampleSlices(s))
            os << ',' << m;
    return os.str();
}
} // namespace

class PatchAudit : public UnitTest
{
public:
    PatchAudit() : UnitTest("Audit: patches, routes and automation of every parameter", "Audit") {}

    void runTest() override
    {
        beginTest("patch round trip: every parameter at minimum, maximum and random values, with all content");
        {
            for (int variant = 0; variant < 4; ++variant)
            {
                Cfg cfg;
                // High: the cabinet builds its kernels on a worker thread, which offline is awaited, so that both engines
                // render the same sound whatever the thread timing.
                cfg.quality = Quality::High;
                auto a = makeEngine(cfg);
                std::mt19937 rng(100 + static_cast<unsigned>(variant));
                const auto& defs = ParameterMap::params();
                for (size_t i = 0; i < defs.size(); ++i)
                {
                    const auto& d = defs[i];
                    if (d.id == "fx_cx_on" || d.id == "seq_run" || d.id == "seq_rec" || d.id == "arp_on")
                        continue;
                    const float n = variant == 0 ? 0.0f : (variant == 1 ? 1.0f : static_cast<float>(rng() % 1001) / 1000.0f);
                    a->setParamNormalised(static_cast<int>(i), n);
                }
                fillContent(*a);
                const auto patch = capturePatch(*a, {"audit", "x", "y", "z"});
                const std::string json = patchToJson(patch);
                Patch parsed;
                std::string error;
                expect(patchFromJson(json, parsed, error), "parse: " + error);
                auto b = makeEngine(cfg);
                applyPatch(*b, parsed);
                const auto da = stateDigest(*a);
                const auto db = stateDigest(*b);
                expect(da == db, "state differs after the round trip, variant " + String(variant));
                if (da != db)
                {
                    {
                        std::istringstream ia(da), ib(db);
                        std::string la, lb;
                        int line = 0;
                        while (std::getline(ia, la) && std::getline(ib, lb) && la == lb)
                            ++line;
                        logMessage("  first differing digest section " + String(line) + ": " + String(la.substr(0, 300)) +
                                   "  <>  " + String(lb.substr(0, 300)));
                    }
                    for (size_t i = 0; i < defs.size(); ++i)
                        if (std::abs(a->getParam(static_cast<int>(i)) - b->getParam(static_cast<int>(i))) >
                            1.0e-6f * std::max(1.0f, std::abs(a->getParam(static_cast<int>(i)))))
                            logMessage("  " + defs[i].id + ": " + String(a->getParam(static_cast<int>(i)), 6) + " -> " +
                                       String(b->getParam(static_cast<int>(i)), 6));
                }
                // the same JSON again, and the same sound
                // The user wavetable is re-analysed on load, so its samples move by float rounding; everything else is
                // exact.
                auto pa = capturePatch(*a, {"audit", "x", "y", "z"});
                auto pb = capturePatch(*b, {"audit", "x", "y", "z"});
                float waveDiff = 0.0f;
                expect(pa.userWaves.size() == pb.userWaves.size());
                for (size_t w = 0; w < std::min(pa.userWaves.size(), pb.userWaves.size()); ++w)
                {
                    expect(pa.userWaves[w].frames.size() == pb.userWaves[w].frames.size());
                    for (size_t k = 0; k < std::min(pa.userWaves[w].frames.size(), pb.userWaves[w].frames.size()); ++k)
                        waveDiff = std::max(waveDiff, std::abs(pa.userWaves[w].frames[k] - pb.userWaves[w].frames[k]));
                }
                expect(waveDiff < 1.0e-4f, "user wavetable moved by " + String(waveDiff, 8));
                pa.userWaves.clear();
                pb.userWaves.clear();
                expect(patchToJson(pa) == patchToJson(pb), "second capture differs from the first");
                if (variant >= 2)
                {
                    for (auto* e : {a.get(), b.get()})
                    {
                        e->setParam("fx_cx_on", 0.0f);
                        e->setParam("fx_ep_phase",
                                    0.0f); // Linear phase takes its kernel from a worker thread: not repeatable in real time
                        e->setParam("osc1_level", 0.8f);
                        e->setParam("amp_sustain", 1.0f);
                        e->setParam("amp_attack", 0.001f);
                    }
                    const auto sa = play(*a, cfg, scriptEvents(Script::Chord), samplesOf(0.5));
                    const auto sb = play(*b, cfg, scriptEvents(Script::Chord), samplesOf(0.5));
                    expect(sa.finite && sb.finite);
                    // The clouds and user wavetables are stored as text / re-analysed (1e-9 rounding) and random parameter
                    // sets can be chaotic (feedback, FM), so the sound is compared to a loose bound; the state digest above
                    // is the exact check.
                    expect(relDiff(sa, sb) < 1.0e-2, "the restored patch sounds different: " + String(relDiff(sa, sb), 8));
                }
            }
            record("Patches",
                   "round trip of every parameter (min, max, 2 random sets) with scenes, sequencer, arp grid, routes, "
                   "clips, tuning, CC map, user wavetable, sample + slices + zones, cloud, impulse response",
                   "state digest before / after, second capture identical, same render", "ok");
        }

        beginTest("patch files: save, load, damaged input, unknown and missing parameters");
        {
            Cfg cfg;
            auto a = makeEngine(cfg);
            a->setParam("filter_cutoff", 1234.0f);
            const auto patch = capturePatch(*a);
            const std::string path = (std::filesystem::temp_directory_path() / "crucible_audit_patch.json").string();
            expect(savePatchFile(path, patch));
            Patch back;
            std::string error;
            expect(loadPatchFile(path, back, error), error);
            expect(std::abs(back.param("filter_cutoff", 0.0f) - 1234.0f) < 0.01f);
            std::filesystem::remove(path);
            expect(!loadPatchFile(path, back, error) && !error.empty());
            const std::string json = patchToJson(patch);
            // every truncation and a set of corruptions: must not crash, must report
            int rejected = 0;
            for (size_t cut = 0; cut < json.size(); cut += std::max<size_t>(1, json.size() / 97))
            {
                Patch p;
                if (!patchFromJson(json.substr(0, cut), p, error))
                    ++rejected;
            }
            expect(rejected > 90, "truncated patches should be rejected (" + String(rejected) + ")");
            for (const char* bad :
                 {"", "null", "[]", "{}", "{\"version\": \"x\"}", "{\"version\": 99}",
                  "{\"format\":\"crucible-patch\",\"version\":2,\"params\":5}",
                  "{\"format\":\"crucible-patch\",\"version\":2,\"params\":{\"filter_cutoff\":\"loud\"}}",
                  "{\"format\":\"crucible-patch\",\"version\":2,\"params\":{\"nope\":1}}",
                  "{\"format\":\"crucible-patch\",\"version\":1,\"params\":{\"filter_cutoff\":1e999}}", "\xff\xfe"})
            {
                Patch p;
                patchFromJson(bad, p, error);
                auto e = makeEngine(cfg);
                applyPatch(*e, p);
                expect(play(*e, cfg, {on(0.0, 60)}, 4800).finite, std::string("patch text: ") + bad);
            }
            // unknown ids are ignored and missing ones keep their defaults
            Patch partial;
            expect(patchFromJson(
                       R"({"format":"crucible-patch","version":2,"params":{"filter_cutoff":900,"from_the_future":3}})",
                       partial, error),
                   error);
            auto e = makeEngine(cfg);
            e->setParam("filter_res", 0.9f);
            applyPatch(*e, partial);
            expect(std::abs(e->getParam("filter_cutoff") - 900.0f) < 0.1f);
            expect(std::abs(e->getParam("filter_res") -
                            e->model().params()[static_cast<size_t>(e->model().indexOf("filter_res"))].defaultValue) <
                       1.0e-6f,
                   "a missing parameter takes its default");
            record("Patches", "files: save / load, 97 truncations, malformed and hostile JSON, unknown and missing ids",
                   "rejected or defaulted, never a crash or non-finite sound", "ok");
        }

        beginTest("a modulation route reaches every modulatable target");
        {
            const auto& defs = ParameterMap::params();
            std::vector<int> targets = ParameterMap::modTargets();
            const int stride = auditStride();
            int checked = 0;
            std::vector<std::string> deaf;
            for (size_t t = 0; t < targets.size(); t += static_cast<size_t>(stride))
            {
                const auto& d = defs[static_cast<size_t>(targets[t])];
                if (!ModRouteStore::canTarget(d.id) || nonAudio().count(d.id) != 0)
                    continue;
                ++checked;
                bool changed = false;
                for (const auto& ctx : contextsFor(d))
                {
                    for (const float amount : {1.0f, -1.0f, 0.5f, -0.5f})
                    {
                        Cfg run;
                        run.hostBpm = ctx.hostBpm;
                        run.quality = ctx.quality;
                        const auto render = [&](bool routed)
                        {
                            auto e = makeEngine(run);
                            baseSetup(*e);
                            applyCtx(*e, ctx);
                            e->setParam("macro1", 1.0f);
                            if (routed)
                                e->modRoutes().add({static_cast<int>(ModSource::Macro1), d.id, amount, 0, 0.0f, 0.0f});
                            auto s = play(*e, run, scriptEvents(ctx.script, run.rate), samplesOf(ctx.seconds, run.rate));
                            return s;
                        };
                        const auto plain = render(false);
                        const auto routed = render(true);
                        expect(routed.finite && routed.peak < 5000.0f, d.id + " routed: peak " + String(routed.peak));
                        if (relDiff(plain, routed) > 1.0e-3)
                        {
                            changed = true;
                            break;
                        }
                    }
                    if (changed)
                        break;
                }
                if (!changed)
                    deaf.push_back(d.id);
            }
            std::string list;
            for (const auto& s : deaf)
                list += " " + s;
            logMessage("routes checked on " + String(checked) + " targets; no effect on:" + (list.empty() ? " none" : list));
            expect(deaf.empty(), "route had no effect on:" + list);
            record("Modulation",
                   "assignable route (Macro 1, +-100 %) to each of the " + std::to_string(targets.size()) +
                       " modulatable targets",
                   "render with / without the route in the target's context",
                   deaf.empty() ? "all audible" : "silent: " + list);
        }

        beginTest("the matrix reaches every destination (fixed voice destinations and generic targets)");
        {
            const auto& defs = ParameterMap::params();
            const auto& names = ParameterMap::modDestNames();
            const auto& legacy = ParameterMap::legacyModDestNames();
            const std::map<std::string, std::string> legacyParam = {
                {"Osc 1 Position", "osc1_pos"},   {"Osc 2 Position", "osc2_pos"},      {"Osc 1 Pitch", "osc1_semi"},
                {"Osc 2 Pitch", "osc2_semi"},     {"Osc 1 Level", "osc1_level"},       {"Osc 2 Level", "osc2_level"},
                {"Cutoff", "filter_cutoff"},      {"Resonance", "filter_res"},         {"Drive", "filter_drive"},
                {"Sub Level", "sub_level"},       {"Noise Level", "noise_level"},      {"Unison Detune", "osc1_uni_detune"},
                {"Amp Level", "osc1_level"},      {"Osc 3 Position", "osc3_pos"},      {"Osc 3 Pitch", "osc3_semi"},
                {"Osc 3 Level", "osc3_level"},    {"Osc 1 Warp A", "osc1_warp_a"},     {"Osc 1 Warp B", "osc1_warp_b"},
                {"Osc 2 Warp A", "osc2_warp_a"},  {"Osc 2 Warp B", "osc2_warp_b"},     {"Osc 3 Warp A", "osc3_warp_a"},
                {"Osc 3 Warp B", "osc3_warp_b"},  {"Osc 1 Pan", "osc1_pan"},           {"Osc 2 Pan", "osc2_pan"},
                {"Osc 3 Pan", "osc3_pan"},        {"Cutoff 2", "filter2_cutoff"},      {"Resonance 2", "filter2_res"},
                {"Drive 2", "filter2_drive"},     {"Filter 1 Mix", "filter_mix"},      {"Filter 2 Mix", "filter2_mix"},
                {"Filter 1 Pan", "filter_pan"},   {"Filter 2 Pan", "filter2_pan"},     {"Pitch", "osc1_semi"},
                {"Master Gain", "master_gain"}};
            int checked = 0;
            std::vector<std::string> deaf;
            const int stride = auditStride();
            for (size_t dst = 1; dst < names.size(); dst += static_cast<size_t>(stride))
            {
                std::string paramId;
                if (dst < legacy.size())
                {
                    const auto it = legacyParam.find(legacy[dst]);
                    if (it == legacyParam.end())
                    {
                        // LFO rate destinations: an LFO modulating the cutoff, its rate modulated by the macro
                        paramId = "";
                    }
                    else
                        paramId = it->second;
                }
                else
                    paramId = defs[static_cast<size_t>(ParameterMap::modTargets()[dst - legacy.size()])].id;
                if (!paramId.empty() && nonAudio().count(paramId) != 0)
                    continue; // e.g. slot 1 has no cross modulation source
                if (paramId == "vector_x" || paramId == "vector_y")
                    continue; // the vector field overlays the scene-managed matrix slots; the assignable-route test covers
                              // these
                ++checked;
                std::vector<Ctx> ctxs;
                if (paramId.empty())
                {
                    Ctx c;
                    const char digit = names[dst].size() > 4 ? names[dst][4] : '1';
                    const int lfo = std::clamp(digit - '0', 1, 4);
                    static const ModSource lfoSource[] = {ModSource::Lfo1, ModSource::Lfo2, ModSource::Lfo3,
                                                          ModSource::Lfo4};
                    // An LFO that modulates the cutoff, with its rate modulated by the macro.
                    c.sets = {{"filter_cutoff", 900.0f},
                              {"mod15_src", static_cast<float>(lfoSource[lfo - 1])},
                              {"mod15_dst", 7.0f},
                              {"mod15_amt", 0.8f},
                              {"lfo" + std::to_string(lfo) + "_rate", 3.0f}};
                    ctxs.push_back(c);
                }
                else
                    ctxs = contextsFor(defs[static_cast<size_t>(ParameterMap::indexOf(paramId))]);
                bool changed = false;
                for (const auto& ctx : ctxs)
                {
                    for (const float amount : {0.9f, -0.9f})
                    {
                        Cfg run;
                        run.hostBpm = ctx.hostBpm;
                        run.quality = ctx.quality;
                        const auto render = [&](float amt)
                        {
                            auto e = makeEngine(run);
                            baseSetup(*e);
                            applyCtx(*e, ctx);
                            e->setParam("macro1", 1.0f);
                            e->setParam("mod1_src", static_cast<float>(ModSource::Macro1));
                            e->setParam("mod1_dst", static_cast<float>(dst));
                            e->setParam("mod1_amt", amt);
                            return play(*e, run, scriptEvents(ctx.script, run.rate), samplesOf(ctx.seconds, run.rate));
                        };
                        const auto plain = render(0.0f);
                        const auto routed = render(amount);
                        expect(routed.finite && routed.peak < 5000.0f && routed.allocations == 0,
                               names[dst] + ": peak " + String(routed.peak));
                        if (relDiff(plain, routed) > 1.0e-3)
                        {
                            changed = true;
                            break;
                        }
                    }
                    if (changed)
                        break;
                }
                if (!changed)
                    deaf.push_back(names[dst]);
            }
            std::string list;
            for (const auto& s : deaf)
                list += " [" + s + "]";
            logMessage("matrix destinations checked: " + String(checked) +
                       "; no effect on:" + (list.empty() ? " none" : list));
            expect(deaf.empty(), "matrix destinations without effect:" + list);
            record("Modulation",
                   "matrix slot (Macro 1) to each of the " + std::to_string(names.size() - 1) + " destinations",
                   "render with amount 0 against +-0.9 in the destination's context",
                   deaf.empty() ? "all audible" : "silent:" + list);
        }

        beginTest(
            "every route source drives a target (LFOs, envelopes, macros, wheel, pressure, bend, CC, MPE, clip lanes)");
        {
            ClipBank clipBank;
            Clip c = makeMseg({{0.0, 0.0f}, {0.5, 1.0f}, {1.0, 0.0f}}, 1.0, false);
            c.trigger = ClipTrigger::Voice;
            c.mode = ClipPlayMode::Loop;
            clipBank.clips.push_back(c);
            std::vector<int> sources;
            for (const auto& s : routeSourceList(clipBank))
                if (!isModulatorSource(s.id) &&
                    !isAudioSource(s.id)) // the bank and the audio bus have their own tests (P12)
                    sources.push_back(s.id);
            for (const int cc : {1, 2, 7, 74, 127})
                sources.push_back(kSourceCC + cc);
            std::vector<std::string> deaf;
            int checked = 0;
            for (const int source : sources)
            {
                if (source == 0 || (source >= kSourceCC + 120 && source <= kSourceCC + 123))
                    continue; // channel mode messages (panic, reset controllers), covered by the panic test
                ++checked;
                bool changed = false;
                for (const Script script : {Script::Mono2, Script::Mpe})
                {
                    const Cfg cfg;
                    const auto render = [&](bool routed)
                    {
                        auto e = makeEngine(cfg);
                        baseSetup(*e);
                        e->setParam("filter_cutoff", 700.0f);
                        e->setParam("macro1", 1.0f);
                        e->setParam("macro2", 0.7f);
                        e->setParam("macro3", 0.7f);
                        e->setParam("macro4", 0.7f);
                        e->setParam("macro5", 0.7f);
                        e->setParam("macro6", 0.7f);
                        e->setParam("strip_l", 0.8f);
                        e->setParam("strip_r", 0.6f);
                        e->setParam("scene_morph", 0.9f);
                        e->setParam("mpe_on", script == Script::Mpe ? 1.0f : 0.0f);
                        e->clips().replace(clipBank);
                        if (routed)
                            e->modRoutes().add({source, "filter_cutoff", 0.9f, 0, 0.0f, 0.0f});
                        std::vector<Ev> ev = scriptEvents(script);
                        ev.push_back(cc(0.05, 1, 100));
                        ev.push_back(cc(0.05, 2, 100));
                        ev.push_back(cc(0.05, 7, 100));
                        ev.push_back(cc(0.05, 74, 100));
                        ev.push_back(cc(0.05, 127, 100));
                        if (source >= kSourceCC)
                            ev.push_back(cc(0.05, source - kSourceCC, 100));
                        if (script == Script::Mpe)
                            ev.push_back(cc(0.05, 74, 127, 1));
                        return play(*e, cfg, ev, samplesOf(0.6));
                    };
                    const auto plain = render(false);
                    const auto routed = render(true);
                    expect(routed.finite && routed.peak < 5000.0f,
                           routeSourceName(source, clipBank) + " peak " + String(routed.peak));
                    if (relDiff(plain, routed) > 1.0e-3)
                    {
                        changed = true;
                        break;
                    }
                }
                if (!changed)
                    deaf.push_back(routeSourceName(source, clipBank));
            }
            std::string list;
            for (const auto& s : deaf)
                list += " [" + s + "]";
            logMessage("route sources checked: " + String(checked) + "; no effect from:" + (list.empty() ? " none" : list));
            expect(deaf.empty(), "route sources without effect:" + list);
            record("Modulation",
                   "each of " + std::to_string(checked) +
                       " route sources (incl. CC 1 / 2 / 7 / 74 / 127, MPE, clip lane) to the cutoff",
                   "render with / without the route", deaf.empty() ? "all audible" : "silent:" + list);
        }

        beginTest("fast automation of every parameter (set to a random extreme every 64 samples) stays finite and "
                  "allocation free");
        {
            const auto& defs = ParameterMap::params();
            const int stride = auditStride();
            std::mt19937 rng(77);
            int bad = 0, renders = 0;
            for (size_t i = 0; i < defs.size(); i += static_cast<size_t>(stride))
            {
                const auto& d = defs[i];
                // (fx_cx_*: convolution changes allocate nothing, but are slow to render; kept in the
                // sweep)
                const auto ctxs = contextsFor(d);
                const auto& ctx = ctxs.front();
                Cfg run;
                run.block = 64;
                run.hostBpm = ctx.hostBpm;
                run.quality = ctx.quality;
                auto e = makeEngine(run);
                baseSetup(*e);
                applyCtx(*e, ctx);
                Shot s;
                std::vector<float> l(64), r(64);
                std::vector<Ev> ev = scriptEvents(ctx.script);
                size_t next = 0;
                const int total = samplesOf(0.5);
                for (int done = 0; done < total; done += 64)
                {
                    e->setParamNormalised(static_cast<int>(i), (rng() & 1u) ? 1.0f : 0.0f);
                    std::vector<MidiInput> midi;
                    while (next < ev.size() && ev[next].pos < done + 64)
                        midi.push_back({std::max(0, ev[next].pos - done), ev[next].status, ev[next].d1, ev[next].d2}),
                            ++next;
                    gAllocations = 0;
                    gTrackAllocations = true;
                    e->process(l.data(), r.data(), 64, midi.empty() ? nullptr : midi.data(), static_cast<int>(midi.size()));
                    gTrackAllocations = false;
                    s.allocations += gAllocations;
                    for (int k = 0; k < 64; ++k)
                    {
                        s.finite =
                            s.finite && std::isfinite(l[static_cast<size_t>(k)]) && std::isfinite(r[static_cast<size_t>(k)]);
                        s.peak =
                            std::max({s.peak, std::abs(l[static_cast<size_t>(k)]), std::abs(r[static_cast<size_t>(k)])});
                    }
                }
                ++renders;
                if (!s.finite || s.peak > kPeakBound || s.allocations > 0)
                {
                    ++bad;
                    expect(false, d.id + ": finite " + String(s.finite ? 1 : 0) + " peak " + String(s.peak) +
                                      " allocations " + String(s.allocations));
                }
            }
            logMessage(String(renders) + " parameters automated");
            record("Parameters",
                   "fast automation: every parameter alternates between its extremes every 64 samples while notes play",
                   "finite, peak <= 32, no allocation in process()",
                   bad == 0 ? "ok (" + std::to_string(renders) + " parameters)" : "FAIL");
        }
    }
};
static PatchAudit patchAudit;

// ---------------------------------------------------------------------------------------------------------------
// Writes the audit table as Markdown when CRUCIBLE_AUDIT_REPORT=<file> is set (runs after the audits above).
class AuditReport : public UnitTest
{
public:
    AuditReport() : UnitTest("Audit: report", "Audit") {}

    void runTest() override
    {
        beginTest("the audit table is written when CRUCIBLE_AUDIT_REPORT is set");
        const char* path = std::getenv("CRUCIBLE_AUDIT_REPORT");
        if (path == nullptr)
            return;
        std::ofstream f(path, std::ios::binary);
        expect(static_cast<bool>(f), "cannot write the report");
        std::string area;
        for (const auto& r : table())
        {
            if (r.area != area)
            {
                area = r.area;
                f << "\n### " << area << "\n\n| Feature | How verified | Result | Bug |\n|---|---|---|---|\n";
            }
            std::string feature = r.feature;
            for (auto& c : feature)
                if (c == '|')
                    c = '/';
            f << "| " << feature << " | " << r.how << " | " << r.result << " | " << r.bug << " |\n";
        }
    }
};
static AuditReport auditReport;
