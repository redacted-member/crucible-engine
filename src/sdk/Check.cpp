// SPDX-License-Identifier: MIT
#include <crucible/sdk/Check.h>

#include <crucible/Denormals.h>
#include <crucible/ModMatrix.h>
#include <crucible/Voice.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <set>

namespace crucible::sdk::check
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

const ModuleRegistry& neutralRegistry()
{
    static const std::shared_ptr<const ModuleRegistry> registry = ModuleRegistry::build(ModuleRegistry::neutralLayout());
    return *registry;
}

// Xorshift generator: the harness never uses rand() so every run with a seed is the same run.
struct Rng
{
    std::uint32_t s;
    explicit Rng(std::uint32_t seed) : s(seed * 2654435761U + 0x9e3779b9U)
    {
        if (s == 0)
            s = 1;
    }
    std::uint32_t next()
    {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        return s;
    }
    float bipolar() { return static_cast<float>(next() >> 8) * (2.0f / 16777216.0f) - 1.0f; }
    bool bit() { return (next() & 0x100U) != 0U; }
};

// Allows denormals for the lifetime of the object (the engine's callers normally flush them).
class ScopedDenormalsAllowed
{
public:
    ScopedDenormalsAllowed() noexcept
    {
#if defined(__aarch64__)
        asm volatile("mrs %0, fpcr" : "=r"(saved));
        const std::uint64_t allowed = saved & ~(1ULL << 24);
        asm volatile("msr fpcr, %0" : : "r"(allowed));
#elif defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
        saved = _mm_getcsr();
        _mm_setcsr(static_cast<unsigned>(saved) & ~0x8040U);
#endif
    }
    ~ScopedDenormalsAllowed()
    {
#if defined(__aarch64__)
        asm volatile("msr fpcr, %0" : : "r"(saved));
#elif defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
        _mm_setcsr(static_cast<unsigned>(saved));
#endif
    }
    ScopedDenormalsAllowed(const ScopedDenormalsAllowed&) = delete;
    ScopedDenormalsAllowed& operator=(const ScopedDenormalsAllowed&) = delete;

private:
    std::uint64_t saved = 0;
};

// ---- Runners ---------------------------------------------------------------------------------------------------
class EffectRunner final : public Runner
{
public:
    explicit EffectRunner(const ModuleDescriptor& descriptor)
        : desc(descriptor), index(descriptor.params.size()), legacy(static_cast<size_t>(kNumModDests), 0.0f)
    {
        std::iota(index.begin(), index.end(), 0);
        module = desc.createEffect();
    }
    bool prepare(double sr, int maxBlock, std::uint32_t seed) override
    {
        ModuleContext c;
        c.sampleRate = sr;
        c.maxBlockSize = maxBlock;
        c.seed = seed;
        module->prepare(c);
        Rng rng(seed ^ 0x51ed270bU);
        aux.resize(static_cast<size_t>(maxBlock));
        for (auto& a : aux)
            a = 0.25f * rng.bipolar();
        return true;
    }
    void reset() override { module->reset(); }
    void process(const float* inL, const float* inR, float* outL, float* outR, int n, const float* values) override
    {
        std::memcpy(outL, inL, static_cast<size_t>(n) * sizeof(float));
        std::memcpy(outR, inR, static_cast<size_t>(n) * sizeof(float));
        EffectInputs in;
        in.current = values;
        in.globalIndex = index.data();
        in.defs = desc.params.data();
        in.legacyDests = legacy.data();
        in.tempo = 120.0;
        in.auxL = aux.data();
        in.auxR = aux.data();
        module->process(outL, outR, n, in);
    }

private:
    const ModuleDescriptor& desc;
    std::unique_ptr<EffectModule> module;
    std::vector<int> index;
    std::vector<float> legacy, aux;
};

class OscillatorRunner final : public Runner
{
public:
    // global: index into the neutral product's parameters per subject parameter.
    OscillatorRunner(const ModuleDescriptor& descriptor, std::vector<int> global)
        : desc(descriptor), mapping(std::move(global)), all(neutralRegistry().parameters().defaultValues())
    {
        recreate();
    }
    bool prepare(double sr, int, std::uint32_t seedValue) override
    {
        rate = sr;
        seed = seedValue;
        instance->prepare(sr, 1);
        instance->arm();
        return true;
    }
    void reset() override
    {
        recreate();
        instance->prepare(rate, 1);
        instance->arm();
        time = 0.0;
    }
    void noteOn(float note, float velocity, std::uint32_t noteSeed) override
    {
        pitch = note;
        time = 0.0;
        OscNoteInfo info;
        info.note = note;
        info.velocity = velocity;
        info.seed = noteSeed + seed;
        instance->noteOn(info);
    }
    void noteOff() override { instance->noteOff(); }
    void process(const float*, const float*, float* outL, float* outR, int n, const float* values) override
    {
        for (size_t i = 0; i < mapping.size(); ++i)
            all[static_cast<size_t>(mapping[i])] = values[i];
        for (int done = 0; done < n; done += kChunkSize)
        {
            const int c = std::min(kChunkSize, n - done);
            std::fill(outL + done, outL + done + c, 0.0f);
            std::fill(outR + done, outR + done + c, 0.0f);
            OscRender r;
            r.values = all.data();
            r.pitch = pitch;
            r.sampleRate = rate;
            r.numSamples = c;
            r.left = outL + done;
            r.right = outR + done;
            r.noteTime = time;
            instance->render(r);
            time += static_cast<double>(c) / rate;
        }
    }

private:
    void recreate() { instance = desc.createOscillator(0, neutralRegistry().parameters()); }
    const ModuleDescriptor& desc;
    std::vector<int> mapping;
    std::vector<float> all;
    std::unique_ptr<OscillatorInstance> instance;
    double rate = 48000.0;
    double time = 0.0;
    float pitch = 60.0f;
    std::uint32_t seed = 1;
};

class NodeRunner final : public Runner
{
public:
    explicit NodeRunner(const graph::NodeType& nodeType) : type(nodeType), index(nodeType.params.size())
    {
        std::iota(index.begin(), index.end(), 0);
        for (const auto& p : type.inputs)
            (p.type == PortType::AudioMono || p.type == PortType::AudioStereo ? numAudioIn : numModIn) += 1;
        for (const auto& p : type.outputs)
            (p.type == PortType::AudioMono || p.type == PortType::AudioStereo ? numAudioOut : numModOut) += 1;
        recreate();
        arena.assign(static_cast<size_t>(std::max(numAudioOut, 1)) * 2 * kChunkSize, 0.0f);
        in.resize(static_cast<size_t>(numAudioIn));
        out.resize(static_cast<size_t>(numAudioOut));
        modIn.assign(static_cast<size_t>(numModIn), 0.0f);
        modOut.assign(static_cast<size_t>(numModOut), 0.0f);
    }
    bool prepare(double sr, int, std::uint32_t seedValue) override
    {
        rate = sr;
        seed = seedValue;
        graph::NodePrepare p;
        p.sampleRate = sr;
        p.maxBlock = kChunkSize;
        p.seed = seed;
        node->prepare(p);
        return true;
    }
    void reset() override
    {
        recreate();
        graph::NodePrepare p;
        p.sampleRate = rate;
        p.maxBlock = kChunkSize;
        p.seed = seed;
        node->prepare(p);
        node->reset();
        std::fill(modOut.begin(), modOut.end(), 0.0f);
        phase = 0.0;
    }
    void noteOn(float noteValue, float velocityValue, std::uint32_t noteSeed) override
    {
        note = noteValue;
        velocity = velocityValue;
        graph::NodeNote n;
        n.note = noteValue;
        n.velocity = velocityValue;
        n.seed = noteSeed + seed;
        node->reset();
        node->noteOn(n);
    }
    void noteOff() override { node->noteOff(); }
    void process(const float* inL, const float* inR, float* outL, float* outR, int n, const float* values) override
    {
        for (int done = 0; done < n; done += kChunkSize)
        {
            const int c = std::min(kChunkSize, n - done);
            graph::NodeIO io;
            for (int p = 0; p < numAudioIn; ++p)
                in[static_cast<size_t>(p)] = {const_cast<float*>(inL + done), const_cast<float*>(inR + done)};
            for (int p = 0; p < numAudioOut; ++p)
                out[static_cast<size_t>(p)] = {arena.data() + static_cast<size_t>(p) * 2 * kChunkSize,
                                               arena.data() + (static_cast<size_t>(p) * 2 + 1) * kChunkSize};
            // A slow sine on every modulation input, so that connected modulation is exercised.
            for (int p = 0; p < numModIn; ++p)
                modIn[static_cast<size_t>(p)] = static_cast<float>(std::sin(phase + 1.7 * p));
            phase += 2.0 * kPi * 0.5 * c / rate;
            io.in = in.data();
            io.numIn = numAudioIn;
            io.out = out.data();
            io.numOut = numAudioOut;
            io.modIn = modIn.data();
            io.numModIn = numModIn;
            io.modOut = modOut.data();
            io.numModOut = numModOut;
            io.values = values;
            io.paramIndex = index.data();
            io.note = note;
            io.velocity = velocity;
            io.modUsed = numModIn >= 32 ? ~0U : (1U << numModIn) - 1U;
            node->process(io, c);
            if (numAudioOut > 0)
            {
                std::memcpy(outL + done, out[0].left, static_cast<size_t>(c) * sizeof(float));
                std::memcpy(outR + done, out[0].right, static_cast<size_t>(c) * sizeof(float));
            }
            else
            {
                const float v = numModOut > 0 ? modOut[0] : 0.0f;
                std::fill(outL + done, outL + done + c, v);
                std::fill(outR + done, outR + done + c, v);
            }
        }
    }

private:
    void recreate()
    {
        graph::NodeCreate c;
        c.model = &neutralRegistry().parameters();
        c.instanceId = "check";
        c.paramIndex = index;
        node = type.create(c);
    }
    graph::NodeType type;
    std::vector<int> index;
    std::unique_ptr<graph::Node> node;
    int numAudioIn = 0, numModIn = 0, numAudioOut = 0, numModOut = 0;
    std::vector<float> arena;
    std::vector<graph::AudioBuffer> in, out;
    std::vector<float> modIn, modOut;
    double rate = 48000.0;
    double phase = 0.0;
    float note = 60.0f, velocity = 1.0f;
    std::uint32_t seed = 1;
};

// ---- Subjects --------------------------------------------------------------------------------------------------
Subject baseSubject(const std::string& id, ModuleKind kind, CpuClass cpu, const std::vector<ParamDef>& params,
                    const std::vector<PortDef>& inputs, const std::vector<PortDef>& outputs)
{
    Subject s;
    s.id = id;
    s.kind = moduleKindName(kind);
    s.cpu = cpu;
    s.params = params;
    s.inputs = inputs;
    s.outputs = outputs;
    return s;
}

Subject effectSubject(const ModuleDescriptor& d)
{
    Subject s = baseSubject(d.id, d.kind, d.cpu, d.params, d.inputs, d.outputs);
    s.frozen.assign(d.params.size(), 0);
    for (size_t i = 0; i < d.params.size(); ++i)
    {
        if (d.params[i].id == d.onParam)
            s.onParam = static_cast<int>(i);
        if (d.params[i].id == d.positionParam)
            s.frozen[i] = 1;
    }
    auto copy = std::make_shared<ModuleDescriptor>(d);
    s.make = [copy]() -> std::unique_ptr<Runner> { return std::make_unique<EffectRunner>(*copy); };
    return s;
}

Subject oscillatorSubject(const ModuleDescriptor& d)
{
    // The slot 1 parameters of the module in the neutral product.
    std::vector<ParamDef> params;
    std::vector<int> global;
    const auto& model = neutralRegistry().parameters();
    for (const auto& p : d.params)
    {
        if (p.id.compare(0, 5, "osc1_") != 0)
            continue;
        const int g = model.indexOf(p.id);
        if (g < 0)
            continue;
        params.push_back(model.params()[static_cast<size_t>(g)]);
        global.push_back(g);
    }
    Subject s = baseSubject(d.id, d.kind, d.cpu, params, d.inputs, d.outputs);
    s.voiced = true;
    const ModuleDescriptor* ptr = &d;
    s.make = [ptr, global]() -> std::unique_ptr<Runner> { return std::make_unique<OscillatorRunner>(*ptr, global); };
    return s;
}

Subject nodeSubject(const graph::NodeType& t)
{
    Subject s = baseSubject(t.type, t.kind, CpuClass::Light, t.params, t.inputs, t.outputs);
    s.voiced = true;
    auto copy = std::make_shared<graph::NodeType>(t);
    s.make = [copy]() -> std::unique_ptr<Runner> { return std::make_unique<NodeRunner>(*copy); };
    return s;
}

// ---- Signals and renders ----------------------------------------------------------------------------------------
struct Signal
{
    std::vector<float> left, right;
};

// Noise and a sine for the first `active` of the samples, then silence (the tail).
Signal makeSignal(std::uint32_t seed, size_t total, size_t active, float amp, double rate)
{
    Signal s;
    s.left.assign(total, 0.0f);
    s.right.assign(total, 0.0f);
    Rng rng(seed ^ 0xa5a5a5a5U);
    for (size_t i = 0; i < std::min(total, active); ++i)
    {
        const float tone = static_cast<float>(std::sin(2.0 * kPi * 220.0 * static_cast<double>(i) / rate));
        s.left[i] = amp * (0.5f * rng.bipolar() + 0.5f * tone);
        s.right[i] = amp * (0.5f * rng.bipolar() + 0.5f * tone);
    }
    return s;
}

struct Rendered
{
    std::vector<float> left, right;
    double peak = 0.0;
    long nonFinite = 0;
    long subnormal = 0;
};

struct Scenario
{
    const Signal* input = nullptr;
    int blockSize = 128;
    bool voiced = false;
    std::uint32_t noteSeed = 7;
    // Where the note is released (fraction of the render); an effect ignores it.
    double releaseAt = 0.4;
    // Parameter values change every block between these two sets when set (the allocation check).
    const std::vector<float>* altValues = nullptr;
};

void renderInto(Runner& runner, const Scenario& sc, const float* values, Rendered& out)
{
    const size_t total = sc.input->left.size();
    const auto block = static_cast<size_t>(sc.blockSize);
    const size_t release = static_cast<size_t>(sc.releaseAt * static_cast<double>(total));
    bool released = false;
    if (sc.voiced)
        runner.noteOn(60.0f, 0.8f, sc.noteSeed);
    size_t blockNo = 0;
    for (size_t off = 0; off < total; off += block, ++blockNo)
    {
        const int n = static_cast<int>(std::min(block, total - off));
        if (sc.voiced && !released && off >= release)
        {
            runner.noteOff();
            released = true;
        }
        const float* v = (sc.altValues != nullptr && (blockNo & 1U) != 0U) ? sc.altValues->data() : values;
        runner.process(sc.input->left.data() + off, sc.input->right.data() + off, out.left.data() + off,
                       out.right.data() + off, n, v);
    }
}

void analyse(Rendered& r)
{
    r.peak = 0.0;
    r.nonFinite = 0;
    r.subnormal = 0;
    for (const auto* ch : {&r.left, &r.right})
        for (const float x : *ch)
        {
            if (!std::isfinite(x))
            {
                ++r.nonFinite;
                continue;
            }
            if (std::fpclassify(x) == FP_SUBNORMAL)
                ++r.subnormal;
            r.peak = std::max(r.peak, static_cast<double>(std::fabs(x)));
        }
}

Rendered makeBuffers(size_t total)
{
    Rendered r;
    r.left.assign(total, 0.0f);
    r.right.assign(total, 0.0f);
    return r;
}

// ---- Parameter sets ---------------------------------------------------------------------------------------------
std::vector<float> defaultsOf(const Subject& s)
{
    std::vector<float> v(s.params.size());
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = s.params[i].defaultValue;
    if (s.onParam >= 0)
        v[static_cast<size_t>(s.onParam)] = 1.0f;
    return v;
}

bool swept(const Subject& s, size_t i)
{
    return static_cast<int>(i) != s.onParam && (s.frozen.empty() || s.frozen[i] == 0);
}

struct Case
{
    std::string label;
    std::vector<float> values;
};

std::vector<Case> extremeCases(const Subject& s, int corners, std::uint32_t seed)
{
    std::vector<Case> cases;
    const auto base = defaultsOf(s);
    for (size_t i = 0; i < s.params.size(); ++i)
    {
        if (!swept(s, i))
            continue;
        for (const bool high : {false, true})
        {
            Case c{s.params[i].id + (high ? " = max" : " = min"), base};
            c.values[i] = high ? s.params[i].max : s.params[i].min;
            cases.push_back(std::move(c));
        }
    }
    for (const bool high : {false, true})
    {
        Case c{high ? "all at max" : "all at min", base};
        for (size_t i = 0; i < s.params.size(); ++i)
            if (swept(s, i))
                c.values[i] = high ? s.params[i].max : s.params[i].min;
        cases.push_back(std::move(c));
    }
    Rng rng(seed);
    for (int k = 0; k < corners; ++k)
    {
        Case c{"corner " + std::to_string(k), base};
        for (size_t i = 0; i < s.params.size(); ++i)
            if (swept(s, i))
                c.values[i] = rng.bit() ? s.params[i].max : s.params[i].min;
        cases.push_back(std::move(c));
    }
    return cases;
}

// The values of the determinism, denormal and CPU runs: a seeded mix of mid-range values.
std::vector<float> midValues(const Subject& s, std::uint32_t seed)
{
    auto v = defaultsOf(s);
    Rng rng(seed ^ 0x1234567U);
    for (size_t i = 0; i < v.size(); ++i)
        if (swept(s, i) && s.params[i].type == ParamType::Float)
            v[i] = s.params[i].min + (s.params[i].max - s.params[i].min) * (0.25f + 0.5f * (0.5f + 0.5f * rng.bipolar()));
    return v;
}

std::string fmt(double v, int digits = 3)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*g", digits, v);
    return buf;
}

CheckResult pass(std::string name, std::string detail = {})
{
    return {std::move(name), Status::Pass, std::move(detail)};
}
CheckResult failed(std::string name, std::string detail)
{
    return {std::move(name), Status::Fail, std::move(detail)};
}
CheckResult skipped(std::string name, std::string detail)
{
    return {std::move(name), Status::Skip, std::move(detail)};
}

// ---- The checks -------------------------------------------------------------------------------------------------
CheckResult checkMetadata(const Subject& s)
{
    std::set<std::string> ids;
    for (const auto& p : s.params)
    {
        if (p.id.empty() || !ids.insert(p.id).second)
            return failed("metadata", "parameter id \"" + p.id + "\" is empty or used twice");
        if (p.name.empty())
            return failed("metadata", "parameter \"" + p.id + "\" has no name");
        if (!std::isfinite(p.min) || !std::isfinite(p.max) || !std::isfinite(p.defaultValue))
            return failed("metadata", "parameter \"" + p.id + "\" has a non-finite range or default");
        if (p.type == ParamType::Choice)
        {
            if (p.choices.size() < 2)
                return failed("metadata", "choice \"" + p.id + "\" has fewer than two choices");
            if (p.defaultValue < 0.0f || p.defaultValue > static_cast<float>(p.choices.size() - 1))
                return failed("metadata", "choice \"" + p.id + "\" has a default outside its choices");
        }
        else
        {
            if (!(p.min < p.max))
                return failed("metadata", "parameter \"" + p.id + "\" needs min < max");
            if (p.defaultValue < p.min || p.defaultValue > p.max)
                return failed("metadata", "parameter \"" + p.id + "\" has a default outside its range");
        }
    }
    std::set<std::string> in, out;
    for (const auto& p : s.inputs)
        if (!in.insert(p.name).second)
            return failed("metadata", "input port \"" + p.name + "\" is declared twice");
    for (const auto& p : s.outputs)
        if (!out.insert(p.name).second)
            return failed("metadata", "output port \"" + p.name + "\" is declared twice");
    return pass("metadata", std::to_string(s.params.size()) + " parameters");
}

CheckResult checkAllocation(const Subject& s, const Options& o)
{
    if (!o.probe.available())
        return skipped("allocation", "no allocation counter in this program");
    auto runner = s.make();
    if (!runner->prepare(o.sampleRate, o.blockSize, o.seed))
        return failed("allocation", "prepare() refused the configuration");
    const auto total = static_cast<size_t>(o.sampleRate * 0.5);
    const Signal input = makeSignal(o.seed, total, total * 3 / 4, 0.25f, o.sampleRate);
    Rendered out = makeBuffers(total);
    const auto lo = defaultsOf(s);
    auto a = lo;
    auto b = lo;
    for (size_t i = 0; i < lo.size(); ++i)
        if (swept(s, i))
        {
            a[i] = s.params[i].min;
            b[i] = s.params[i].max;
        }
    Scenario sc;
    sc.input = &input;
    sc.blockSize = o.blockSize;
    sc.voiced = s.voiced;
    sc.altValues = &b;
    // One warm-up render outside the counter: lazily built tables and the like belong to prepare().
    renderInto(*runner, sc, a.data(), out);
    o.probe.track(true);
    renderInto(*runner, sc, a.data(), out);
    renderInto(*runner, sc, lo.data(), out);
    const long n = o.probe.count();
    o.probe.track(false);
    if (n != 0)
        return failed("allocation", std::to_string(n) + " allocation(s) on the audio thread");
    return pass("allocation", "0 allocations");
}

CheckResult checkExtremes(const Subject& s, const Options& o, CheckResult& finiteResult)
{
    auto runner = s.make();
    if (!runner->prepare(o.sampleRate, o.blockSize, o.seed))
        return failed("extremes", "prepare() refused the configuration");
    const auto total = static_cast<size_t>(o.sampleRate * o.seconds);
    const Signal loud = makeSignal(o.seed, total, total / 2, 0.5f, o.sampleRate);
    const Signal hot = makeSignal(o.seed + 1, total, total / 2, 8.0f, o.sampleRate);
    const Signal tiny = makeSignal(o.seed + 2, total, total / 2, 1.0e-39f, o.sampleRate);
    const Signal silent = makeSignal(o.seed, total, 0, 0.0f, o.sampleRate);
    Rendered out = makeBuffers(total);
    const auto cases = extremeCases(s, o.corners, o.seed);
    double worst = 0.0;
    std::string worstCase;
    long nonFinite = 0;
    std::string nonFiniteCase;
    int runs = 0;
    const auto once = [&](const std::string& label, const Signal& in, const std::vector<float>& values)
    {
        Scenario sc;
        sc.input = &in;
        sc.blockSize = o.blockSize;
        sc.voiced = s.voiced;
        runner->reset();
        renderInto(*runner, sc, values.data(), out);
        analyse(out);
        ++runs;
        if (out.nonFinite > 0 && nonFinite == 0)
        {
            nonFinite = out.nonFinite;
            nonFiniteCase = label;
        }
        if (out.peak > worst)
        {
            worst = out.peak;
            worstCase = label;
        }
    };
    for (const auto& c : cases)
    {
        once(c.label, loud, c.values);
        // Silence through the same settings catches modules that generate output from their own state alone (self
        // oscillation, noise floors) at extreme settings; sources ignore the input anyway.
        if (!s.voiced)
            once(c.label + " (silent input)", silent, c.values);
    }
    const auto mid = midValues(s, o.seed);
    once("default settings, +18 dBFS input", hot, defaultsOf(s));
    once("mid settings, denormal input", tiny, mid);
    once("mid settings, +18 dBFS input", hot, mid);

    finiteResult = nonFinite == 0
                       ? pass("finite", std::to_string(runs) + " renders, no NaN or infinity")
                       : failed("finite", std::to_string(nonFinite) + " non-finite samples at \"" + nonFiniteCase + "\"");
    if (worst > o.maxPeak)
        return failed("extremes", "peak " + fmt(worst) + " at \"" + worstCase + "\" exceeds " + fmt(o.maxPeak));
    return pass("extremes", std::to_string(runs) + " renders, worst peak " + fmt(worst) +
                                (worst > 0.0 ? " at \"" + worstCase + "\"" : ""));
}

CheckResult checkDenormals(const Subject& s, const Options& o)
{
    auto runner = s.make();
    if (!runner->prepare(o.sampleRate, o.blockSize, o.seed))
        return failed("denormals", "prepare() refused the configuration");
    // A short burst and a long decaying tail, at mid settings.
    const auto total = static_cast<size_t>(o.sampleRate * std::max(2.0, o.seconds));
    const Signal input = makeSignal(o.seed, total, total / 20, 0.5f, o.sampleRate);
    Scenario sc;
    sc.input = &input;
    sc.blockSize = o.blockSize;
    sc.voiced = s.voiced;
    sc.releaseAt = 0.05;
    const auto values = midValues(s, o.seed);
    Rendered open = makeBuffers(total), flushed = makeBuffers(total);
    double tOpen = 1e30, tFlushed = 1e30;
    for (int rep = 0; rep < 2; ++rep)
    {
        {
            ScopedDenormalsAllowed allowed;
            runner->reset();
            const auto t0 = std::chrono::steady_clock::now();
            renderInto(*runner, sc, values.data(), open);
            tOpen = std::min(tOpen, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        }
        {
            ScopedFlushDenormals flush;
            runner->reset();
            const auto t0 = std::chrono::steady_clock::now();
            renderInto(*runner, sc, values.data(), flushed);
            tFlushed = std::min(tFlushed, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        }
    }
    analyse(open);
    analyse(flushed);
    if (open.nonFinite > 0)
        return failed("denormals", "non-finite output while denormals are enabled");
    double diff = 0.0;
    for (size_t i = 0; i < total; ++i)
    {
        diff = std::max(diff, static_cast<double>(std::fabs(open.left[i] - flushed.left[i])));
        diff = std::max(diff, static_cast<double>(std::fabs(open.right[i] - flushed.right[i])));
    }
    const double slowdown = tFlushed > 0.0 ? tOpen / tFlushed : 1.0;
    const std::string info = "differs " + fmt(diff) + " from the flushed render, tail " + fmt(slowdown, 2) + "x";
    if (diff > 1.0e-9)
        return failed("denormals", "the output depends on flush-to-zero: " + info);
    if (slowdown > 50.0 && tOpen - tFlushed > 0.05)
    {
        if (o.enforceDenormalTime)
            return failed("denormals", "a decaying tail runs " + fmt(slowdown, 2) + "x slower with denormals enabled");
        return pass("denormals", info + " (slow tail not enforced)");
    }
    return pass("denormals", info);
}

CheckResult checkDeterminism(const Subject& s, const Options& o)
{
    const auto total = static_cast<size_t>(o.sampleRate * 0.5);
    const Signal input = makeSignal(o.seed, total, total * 3 / 4, 0.4f, o.sampleRate);
    Scenario sc;
    sc.input = &input;
    sc.blockSize = o.blockSize;
    sc.voiced = s.voiced;
    const auto values = midValues(s, o.seed);
    const auto same = [](const Rendered& a, const Rendered& b)
    {
        return std::memcmp(a.left.data(), b.left.data(), a.left.size() * sizeof(float)) == 0 &&
               std::memcmp(a.right.data(), b.right.data(), a.right.size() * sizeof(float)) == 0;
    };
    Rendered first = makeBuffers(total), second = makeBuffers(total), third = makeBuffers(total);
    auto a = s.make();
    auto b = s.make();
    if (!a->prepare(o.sampleRate, o.blockSize, o.seed) || !b->prepare(o.sampleRate, o.blockSize, o.seed))
        return failed("determinism", "prepare() refused the configuration");
    renderInto(*a, sc, values.data(), first);
    renderInto(*b, sc, values.data(), second);
    if (!same(first, second))
        return failed("determinism", "two instances with seed " + std::to_string(o.seed) + " render different audio");
    a->reset();
    renderInto(*a, sc, values.data(), third);
    if (!same(first, third))
        return failed("determinism", "an instance after reset() renders different audio than a new one");
    return pass("determinism", "bit-identical for seed " + std::to_string(o.seed));
}

CheckResult checkCpu(const Subject& s, const Options& o, double& loadOut)
{
    auto runner = s.make();
    if (!runner->prepare(o.sampleRate, o.blockSize, o.seed))
        return failed("cpu", "prepare() refused the configuration");
    const auto total = static_cast<size_t>(o.sampleRate * 2.0);
    const Signal input = makeSignal(o.seed, total, total * 3 / 4, 0.3f, o.sampleRate);
    Rendered out = makeBuffers(total);
    Scenario sc;
    sc.input = &input;
    sc.blockSize = o.blockSize;
    sc.voiced = s.voiced;
    sc.releaseAt = 0.9;
    const auto values = midValues(s, o.seed);
    double best = 1e30;
    for (int rep = 0; rep < 3; ++rep)
    {
        runner->reset();
        const auto t0 = std::chrono::steady_clock::now();
        renderInto(*runner, sc, values.data(), out);
        best = std::min(best, std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
    }
    loadOut = best / (static_cast<double>(total) / o.sampleRate);
    const double budget = s.cpu == CpuClass::Light ? o.cpuLight : (s.cpu == CpuClass::Medium ? o.cpuMedium : o.cpuHeavy);
    const std::string info = fmt(loadOut * 100.0, 3) + " % of one core (budget " + fmt(budget * 100.0, 3) + " %)";
    if (loadOut > budget && o.enforceCpu)
        return failed("cpu", info);
    return pass("cpu", info);
}
} // namespace

bool Report::passed() const
{
    return failures() == 0;
}

int Report::failures() const
{
    int n = 0;
    for (const auto& c : checks)
        n += c.status == Status::Fail ? 1 : 0;
    return n;
}

std::vector<Subject> builtInSubjects()
{
    std::vector<Subject> out;
    const ModuleRegistry& registry = neutralRegistry();
    for (const auto& d : registry.modules())
    {
        if (d.kind == ModuleKind::Effect && d.createEffect)
            out.push_back(effectSubject(d));
        else if (d.kind == ModuleKind::Oscillator && d.createOscillator)
            out.push_back(oscillatorSubject(d));
        else
            out.push_back(baseSubject(d.id, d.kind, d.cpu, d.params, d.inputs, d.outputs));
    }
    for (const auto& t : graph::NodeRegistry::builtIn().types())
    {
        // The oscillator wrappers are the oscillator modules above; the classic voice is the whole voice.
        if (t.usesSlot || t.type == "classic.voice")
            continue;
        out.push_back(nodeSubject(t));
    }
    return out;
}

std::vector<Subject> sdkSubjects(const graph::NodeRegistry& nodes)
{
    std::vector<Subject> out;
    for (const auto& e : nodes.effects())
        out.push_back(effectSubject(e));
    const auto& builtIn = graph::NodeRegistry::builtIn();
    for (const auto& t : nodes.types())
        if (builtIn.find(t.type) == nullptr)
            out.push_back(nodeSubject(t));
    return out;
}

Report run(const Subject& subject, const Options& options)
{
    Report r;
    r.id = subject.id;
    r.kind = subject.kind;
    r.checks.push_back(checkMetadata(subject));
    if (!subject.make)
    {
        for (const char* name : {"allocation", "extremes", "finite", "denormals", "determinism", "cpu"})
            r.checks.push_back(skipped(name, "this module has no processor of its own"));
        return r;
    }
    try
    {
        r.checks.push_back(checkAllocation(subject, options));
        CheckResult finiteResult;
        r.checks.push_back(checkExtremes(subject, options, finiteResult));
        r.checks.push_back(std::move(finiteResult));
        r.checks.push_back(checkDenormals(subject, options));
        r.checks.push_back(checkDeterminism(subject, options));
        r.checks.push_back(checkCpu(subject, options, r.cpuLoad));
    }
    catch (const std::exception& e)
    {
        r.checks.push_back(failed("exception", std::string("a check threw: ") + e.what()));
    }
    return r;
}

} // namespace crucible::sdk::check
