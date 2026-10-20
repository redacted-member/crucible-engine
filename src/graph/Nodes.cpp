// SPDX-License-Identifier: MIT
#include <crucible/graph/Node.h>
#include <crucible/graph/Program.h>

#include <crucible/AnalogEnvelope.h>
#include <crucible/AudioLfo.h>
#include <crucible/Dsp.h>
#include <crucible/Filters.h>
#include <crucible/module/ParamBuilder.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

// The built-in node types of the voice graph: wrappers of the existing oscillator modules, envelope, LFO, filter,
// VCA and the output mixer, plus the classic voice. The wrappers reuse the engine's DSP classes unchanged.

namespace crucible::graph
{
namespace
{
constexpr float kSqrt2 = 1.41421356f;

float dbToGain(float db)
{
    return db > -60.0f ? std::pow(10.0f, db * 0.05f) : 0.0f;
}

void panGains(float pan, float& left, float& right)
{
    const float angle = (std::clamp(pan, -1.0f, 1.0f) + 1.0f) * 0.78539816f;
    left = std::cos(angle) * kSqrt2;
    right = std::sin(angle) * kSqrt2;
}

// ---- Oscillator modules as nodes. The oscillator module keeps its "oscN_..." parameters; the node binds the slot.
class OscillatorNode final : public Node
{
public:
    OscillatorNode(const NodeCreate& c) : slot(c.slot), sp(c.slot, *c.model)
    {
        if (c.oscillator != nullptr && c.oscillator->createOscillator)
            instance = c.oscillator->createOscillator(c.slot, *c.model);
        level = sp.bind("level");
        semi = sp.bind("semi");
        fine = sp.bind("fine");
        pan = sp.bind("pan");
    }

    void prepare(const NodePrepare& p) override
    {
        rate = p.sampleRate;
        if (instance)
        {
            instance->prepare(p.sampleRate, static_cast<int>(p.quality));
            instance->arm(); // a node exists because its type is used
        }
    }

    void noteOn(const NodeNote& n) override
    {
        time = 0.0;
        if (!instance)
            return;
        OscNoteInfo info;
        info.note = n.note;
        info.velocity = n.velocity;
        info.seed = n.seed + static_cast<std::uint32_t>(slot) * 0x85ebca6bU;
        cardTune = 0.0f;
        if (n.card != nullptr && n.cardAmounts.osc > 0.0f)
        {
            info.card = n.card->osc[static_cast<size_t>(slot) % kCardOscSlots].data();
            info.cardAmount = n.cardAmounts.osc;
            if (!instance->usesVoiceCard())
                cardTune = n.card->pitchSemitones(n.cardAmounts.osc);
        }
        instance->noteOn(info);
    }

    void noteOff() override
    {
        if (instance)
            instance->noteOff();
    }

    void process(NodeIO& io, int n) override
    {
        float* l = io.out[0].left;
        float* r = io.out[0].right;
        std::fill(l, l + n, 0.0f);
        std::fill(r, r + n, 0.0f);
        if (!instance)
            return;
        OscRender rd;
        rd.values = io.values;
        rd.pitch = io.note + sp.value(io.values, nullptr, semi) + sp.value(io.values, nullptr, fine) * 0.01f +
                   24.0f * io.mod(0) + cardTune;
        rd.sampleRate = rate;
        rd.numSamples = n;
        rd.left = l;
        rd.right = r;
        if (io.shared != nullptr)
        {
            const auto s = static_cast<size_t>(slot);
            rd.sample = io.shared->sample[s];
            rd.slices = io.shared->slices[s];
            rd.zones = io.shared->zones[s];
            rd.cloud = io.shared->cloud[s];
        }
        rd.noteTime = time;
        // Audio-rate inputs: per sample for an oscillator that takes them, else the pitch follows the chunk mean.
        float pitchMod[kChunkSize];
        if (io.numRateIn > 0)
        {
            const bool perSample = instance->supportsAudioRate();
            if (io.rateConnected(0))
            {
                if (perSample)
                {
                    for (int i = 0; i < n; ++i)
                        pitchMod[i] = 24.0f * io.rate(0)[i];
                    rd.pitchMod = pitchMod;
                }
                else
                {
                    float mean = 0.0f;
                    for (int i = 0; i < n; ++i)
                        mean += io.rate(0)[i];
                    rd.pitch += 24.0f * mean / static_cast<float>(n);
                }
            }
            if (perSample && io.rateConnected(1))
                rd.widthMod = io.rate(1);
            if (perSample && io.rateConnected(2))
                rd.fmAmountMod = io.rate(2);
        }
        instance->render(rd);
        time += static_cast<double>(n) / rate;
        float gl = 0.0f, gr = 0.0f;
        panGains(sp.value(io.values, nullptr, pan), gl, gr);
        const float lv = std::clamp(sp.value(io.values, nullptr, level), 0.0f, 1.0f);
        for (int i = 0; i < n; ++i)
        {
            l[i] *= gl * lv;
            r[i] *= gr * lv;
        }
    }

private:
    int slot;
    SlotParams sp;
    int level = 0, semi = 0, fine = 0, pan = 0;
    std::unique_ptr<OscillatorInstance> instance;
    double rate = 48000.0;
    double time = 0.0;
    float cardTune = 0.0f; // semitones of the voice card, for an oscillator that does not draw its own tolerance
};

NodeType oscillatorType(const char* id, const char* name, const char* description)
{
    NodeType t;
    t.type = id;
    t.name = name;
    t.kind = ModuleKind::Oscillator;
    t.category = "Oscillator";
    t.description = description;
    t.usesSlot = true;
    // pitch_fm moves the pitch by 24 semitones per unit of signal; width_mod and fm_mod add to the pulse width and to the
    // cross-modulation amount (oscillators that take audio-rate input: the analog oscillator; the others ignore both).
    t.inputs = {{"pitch", PortType::Modulation},
                {"pitch_fm", PortType::AudioRate},
                {"width_mod", PortType::AudioRate},
                {"fm_mod", PortType::AudioRate}};
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate& c) { return std::make_unique<OscillatorNode>(c); };
    return t;
}

// ---- Envelope: the voice's ADSR (Dsp.h) as a modulation source.
class EnvelopeNode final : public Node
{
public:
    void prepare(const NodePrepare& p) override { env.prepare(p.sampleRate); }
    void reset() override { env.reset(); }
    void noteOn(const NodeNote& n) override
    {
        oneShot = n.oneShot;
        env.noteOn();
    }
    void noteOff() override
    {
        if (!oneShot)
            env.noteOff();
    }
    bool holdsVoice() const override { return !env.isIdle(); }
    void process(NodeIO& io, int n) override
    {
        AdsrParams a;
        a.attack = io.param(0);
        a.decay = io.param(1);
        a.sustain = io.param(2);
        a.release = io.param(3);
        a.hold = io.param(4);
        a.curve = io.param(5);
        a.attackCurve = io.param(6);
        env.setParams(a);
        // A one-shot envelope has no key to wait for: it is released as soon as it reaches its sustain stage.
        if (oneShot && env.isSustaining())
            env.noteOff();
        io.modOut[0] = env.advance(n);
    }

private:
    Adsr env;
    bool oneShot = false;
};

NodeType envelopeType()
{
    NodeType t;
    t.type = "env.adsr";
    t.name = "Envelope";
    t.kind = ModuleKind::Modulator;
    t.category = "Envelope";
    t.description = "Attack / hold / decay / sustain / release envelope with curve; ends the voice when it finishes.";
    const auto time = [](const char* id, const char* name, float lo, float def)
    { return params::floating(id, name, "env", lo, 10.0f, def, "s", 0.3f, Smoothing::None); };
    t.params.push_back(time("attack", "Attack", 0.001f, 0.005f));
    t.params.push_back(time("decay", "Decay", 0.001f, 0.3f));
    t.params.push_back(params::floating("sustain", "Sustain", "env", 0.0f, 1.0f, 0.8f, {}, 1.0f, Smoothing::None));
    t.params.push_back(time("release", "Release", 0.001f, 0.25f));
    t.params.push_back(time("hold", "Hold", 0.0f, 0.0f));
    t.params.push_back(params::floating("curve", "Curve", "env", -1.0f, 1.0f, -0.8f, {}, 1.0f, Smoothing::None));
    t.params.push_back(
        params::floating("attack_curve", "Attack Curve", "env", -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    t.outputs = {{"out", PortType::Modulation}};
    t.create = [](const NodeCreate&) { return std::make_unique<EnvelopeNode>(); };
    return t;
}

// ---- Analog envelope: the digital ADSR or an RC model (AnalogEnvelope.h) with retrigger modes, velocity and key time
// scaling, and the voice card's tolerance. env.adr is the same class with the retrigger fixed to attack-decay-release.
class AnalogEnvelopeNode final : public Node
{
public:
    explicit AnalogEnvelopeNode(bool adrOnly) : adr(adrOnly) {}
    void prepare(const NodePrepare& p) override
    {
        env.prepare(p.sampleRate);
        envIndex = p.index % kCardEnvs;
    }
    void reset() override
    {
        env.reset();
        pendingOn = pendingOff = false;
    }
    // The envelope needs the parameters (retrigger mode, time scaling) to start, and a node only sees them in process():
    // the note-on is applied at the first chunk.
    void noteOn(const NodeNote& n) override
    {
        oneShot = n.oneShot;
        velocity = n.velocity;
        note = static_cast<int>(n.note);
        pendingOn = true;
        pendingOff = false;
    }
    void noteOff() override
    {
        if (oneShot)
            return;
        if (pendingOn)
            pendingOff = true;
        else
            env.noteOff();
    }
    bool holdsVoice() const override { return pendingOn || !env.isIdle(); }
    void process(NodeIO& io, int n) override
    {
        cur = &io;
        env.setParams(adsr(), shapeOf(), timeScale(), sustainOffset());
        if (pendingOn)
        {
            env.noteOn(velocity, note);
            pendingOn = false;
            if (pendingOff)
                env.noteOff();
            pendingOff = false;
        }
        if (oneShot && env.isSustaining())
            env.noteOff();
        io.modOut[0] = env.advance(n);
    }

private:
    // The parameter layout differs: env.analog has sustain, hold and the two curves, env.adr has neither sustain nor curves.
    AdsrParams adsr() const
    {
        AdsrParams a;
        if (cur == nullptr)
            return a;
        a.attack = cur->param(0);
        a.decay = cur->param(1);
        a.release = cur->param(adr ? 2 : 3);
        a.sustain = adr ? 0.0f : cur->param(2);
        a.hold = cur->param(adr ? 3 : 4);
        a.curve = adr ? -0.8f : cur->param(5);
        a.attackCurve = adr ? 0.0f : cur->param(6);
        return a;
    }
    EnvShape shapeOf() const
    {
        EnvShape s;
        if (cur == nullptr)
            return s;
        const int base = adr ? 4 : 7;
        s.mode = cur->param(base) > 0.5f ? EnvMode::AnalogRc : EnvMode::Digital;
        s.retrigger =
            adr ? Retrigger::Adr : static_cast<Retrigger>(std::clamp(static_cast<int>(cur->param(base + 1)), 0, 3));
        s.overshoot = cur->param(adr ? base + 1 : base + 2);
        s.velocityTime = cur->param(adr ? base + 2 : base + 3);
        s.keyTime = cur->param(adr ? base + 3 : base + 4);
        return s;
    }
    float timeScale() const
    {
        return cur != nullptr && cur->card != nullptr && cur->cardAmounts.env > 0.0f
                   ? cur->card->envTimeScale(envIndex, cur->cardAmounts.env)
                   : 1.0f;
    }
    float sustainOffset() const
    {
        return cur != nullptr && cur->card != nullptr && cur->cardAmounts.env > 0.0f
                   ? cur->card->envSustainOffset(envIndex, cur->cardAmounts.env)
                   : 0.0f;
    }

    AnalogEnvelope env;
    bool adr;
    bool oneShot = false;
    int envIndex = 0;
    float velocity = 1.0f;
    int note = 60;
    bool pendingOn = false;
    bool pendingOff = false;
    const NodeIO* cur = nullptr;
};

NodeType analogEnvelopeType(bool adr)
{
    NodeType t;
    t.type = adr ? "env.adr" : "env.analog";
    t.name = adr ? "ADR Envelope" : "Analog Envelope";
    t.kind = ModuleKind::Modulator;
    t.category = "Envelope";
    t.description =
        adr ? "Attack / decay / release envelope without sustain (the decay runs while the key is held), digital or "
              "analog RC, with velocity and key time scaling and the voice card's tolerance."
            : "Envelope with a digital or analog RC circuit (the attack reaches the peak at the attack time, decay and "
              "release are exponential), retrigger modes, velocity and key time scaling and the voice card's tolerance.";
    const auto time = [](const char* id, const char* name, float lo, float def)
    { return params::floating(id, name, "env", lo, 10.0f, def, "s", 0.3f, Smoothing::None); };
    t.params.push_back(time("attack", "Attack", 0.001f, 0.005f));
    t.params.push_back(time("decay", "Decay", 0.001f, 0.3f));
    if (adr)
    {
        t.params.push_back(time("release", "Release", 0.001f, 0.25f));
        t.params.push_back(time("hold", "Hold", 0.0f, 0.0f));
    }
    else
    {
        t.params.push_back(params::floating("sustain", "Sustain", "env", 0.0f, 1.0f, 0.8f, {}, 1.0f, Smoothing::None));
        t.params.push_back(time("release", "Release", 0.001f, 0.25f));
        t.params.push_back(time("hold", "Hold", 0.0f, 0.0f));
        t.params.push_back(params::floating("curve", "Curve", "env", -1.0f, 1.0f, -0.8f, {}, 1.0f, Smoothing::None));
        t.params.push_back(
            params::floating("attack_curve", "Attack Curve", "env", -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    }
    t.params.push_back(params::choice("mode", "Mode", "env", {"Digital", "Analog RC"}, 1));
    if (!adr)
        t.params.push_back(params::choice("retrigger", "Retrigger", "env", {"From Level", "Reset", "Legato", "ADR"}, 0));
    t.params.push_back(params::floating("overshoot", "Overshoot", "env", 0.02f, 1.0f, 0.35f, {}, 1.0f, Smoothing::None));
    t.params.push_back(params::floating("vel_time", "Velocity Time", "env", -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    t.params.push_back(params::floating("key_time", "Key Time", "env", -1.0f, 1.0f, 0.0f, {}, 1.0f, Smoothing::None));
    t.outputs = {{"out", PortType::Modulation}};
    t.create = [adr](const NodeCreate&) { return std::make_unique<AnalogEnvelopeNode>(adr); };
    return t;
}

// ---- LFO (Dsp.h), bipolar output scaled by its amount.
class LfoNode final : public Node
{
public:
    void prepare(const NodePrepare& p) override { rate = p.sampleRate; }
    void reset() override { lfo.reset(1U, 0.0f); }
    void noteOn(const NodeNote& n) override { lfo.reset(n.seed, 0.0f); }
    void process(NodeIO& io, int n) override
    {
        LfoSettings s;
        s.rateHz = io.param(0);
        s.shape = static_cast<int>(io.param(1));
        io.modOut[0] = lfo.advance(s, n, rate, nullptr) * io.param(2);
    }

private:
    Lfo lfo;
    double rate = 48000.0;
};

NodeType lfoType()
{
    NodeType t;
    t.type = "lfo.basic";
    t.name = "LFO";
    t.kind = ModuleKind::Modulator;
    t.category = "LFO";
    t.description = "Low-frequency oscillator, restarted by every note.";
    t.params.push_back(params::floating("rate", "Rate", "lfo", 0.02f, 30.0f, 2.0f, "Hz", 0.4f, Smoothing::None));
    t.params.push_back(params::choice("shape", "Shape", "lfo", params::lfoShapeNames(), 0));
    t.params.push_back(params::floating("amount", "Amount", "lfo", 0.0f, 1.0f, 1.0f));
    t.outputs = {{"out", PortType::Modulation}};
    t.create = [](const NodeCreate&) { return std::make_unique<LfoNode>(); };
    return t;
}

// ---- Audio-rate LFO: the LFO shapes as a band-limited oscillator (AudioLfo.h), an audio output for the audio-rate inputs
// of other nodes (cutoff_fm, pitch_fm, width_mod, am). Restarted by every note; the frequency is fixed or a ratio of the
// note.
class AudioLfoNode final : public Node
{
public:
    void prepare(const NodePrepare& p) override
    {
        rate = p.sampleRate;
        FineBlep::instance(); // builds the tables on this (control) thread
    }
    void reset() override { lfo.reset(0.0); }
    void noteOn(const NodeNote&) override { lfo.reset(0.0); }
    void process(NodeIO& io, int n) override
    {
        static constexpr int shapes[] = {0, 1, 2, 3, 5, 7};
        const int shape = shapes[std::clamp(static_cast<int>(io.param(1)), 0, 5)];
        const double hz = io.param(2) > 0.5f
                              ? static_cast<double>(440.0f * std::exp2((io.note - 69.0f) / 12.0f)) * io.param(3)
                              : static_cast<double>(io.param(0));
        lfo.render(shape, hz / rate, io.param(4), n, io.out[0].left);
        std::memcpy(io.out[0].right, io.out[0].left, sizeof(float) * static_cast<size_t>(n));
    }

private:
    AudioLfo lfo;
    double rate = 48000.0;
};

NodeType audioLfoType()
{
    NodeType t;
    t.type = "lfo.audio";
    t.name = "Audio-Rate LFO";
    t.kind = ModuleKind::Modulator;
    t.category = "LFO";
    t.description =
        "Band-limited audio-rate oscillator with the LFO shapes (sine, triangle, saw, square, saw down, 25 % pulse), "
        "restarted by every note; its audio output feeds the audio-rate inputs of other nodes.";
    t.params.push_back(params::floating("rate", "Frequency", "lfo", 1.0f, 20000.0f, 440.0f, "Hz", 0.3f, Smoothing::None));
    t.params.push_back(
        params::choice("shape", "Shape", "lfo", {"Sine", "Triangle", "Saw Up", "Square", "Saw Down", "Pulse 25%"}, 0));
    t.params.push_back(params::choice("mode", "Mode", "lfo", {"Frequency", "Key Ratio"}, 0));
    t.params.push_back(params::floating("ratio", "Key Ratio", "lfo", 0.125f, 16.0f, 1.0f, {}, 0.4f, Smoothing::None));
    t.params.push_back(params::floating("amount", "Amount", "lfo", 0.0f, 1.0f, 1.0f));
    t.outputs = {{"out", PortType::AudioMono}};
    t.create = [](const NodeCreate&) { return std::make_unique<AudioLfoNode>(); };
    return t;
}

// ---- Filter: the legacy FilterUnit (Filters.h).
class FilterNode final : public Node
{
public:
    // The nonlinear models (type 17 and up) are allocated here, so selecting one never allocates on the audio thread.
    void prepare(const NodePrepare& p) override
    {
        rate = p.sampleRate;
        unit.prepare(p.sampleRate, p.quality);
    }
    void reset() override { unit.reset(); }
    void noteOn(const NodeNote&) override { unit.reset(); }
    void process(NodeIO& io, int n) override
    {
        FilterSettings s;
        s.model = static_cast<int>(io.param(0));
        const float key = io.param(4);
        s.cutoffHz = std::clamp(io.param(1) * std::exp2(5.0f * io.mod(0) + key * (io.note - 60.0f) / 12.0f), 20.0f,
                                0.45f * static_cast<float>(rate));
        s.resonance = io.param(2);
        unit.configure(s, io.param(3), rate);
        const float* inL = io.in[0].left;
        const float* inR = io.in[0].right;
        if (io.rateConnected(0))
        {
            // Audio-rate cutoff: 5 octaves per unit of signal, per sample.
            float octaves[kChunkSize];
            for (int i = 0; i < n; ++i)
                octaves[i] = 5.0f * io.rate(0)[i];
            std::memcpy(io.out[0].left, inL, sizeof(float) * static_cast<size_t>(n));
            std::memcpy(io.out[0].right, inR, sizeof(float) * static_cast<size_t>(n));
            unit.processBlock(io.out[0].left, io.out[0].right, n, octaves);
            return;
        }
        std::memcpy(io.out[0].left, inL, sizeof(float) * static_cast<size_t>(n));
        std::memcpy(io.out[0].right, inR, sizeof(float) * static_cast<size_t>(n));
        unit.processBlock(io.out[0].left, io.out[0].right, n, nullptr);
    }

private:
    FilterUnit unit;
    double rate = 48000.0;
};

NodeType filterType()
{
    NodeType t;
    t.type = "filter.svf";
    t.name = "Filter";
    t.kind = ModuleKind::Filter;
    t.category = "Filter";
    t.description = "The engine's classic filter models (state variable, ladder, diode, comb, formant, phaser).";
    t.params.push_back(params::choice("type", "Type", "filter", filterModelNames(), 0));
    t.params.push_back(params::floating("cutoff", "Cutoff", "filter", 20.0f, 20000.0f, 8000.0f, "Hz", 0.3f, Smoothing::Log));
    t.params.push_back(params::floating("res", "Resonance", "filter", 0.0f, 1.0f, 0.15f));
    t.params.push_back(params::floating("drive", "Drive", "filter", 0.0f, 1.0f, 0.0f));
    t.params.push_back(params::floating("key", "Keytrack", "filter", 0.0f, 1.0f, 0.0f));
    t.inputs = {{"in", PortType::AudioStereo}, {"cutoff", PortType::Modulation}, {"cutoff_fm", PortType::AudioRate}};
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate&) { return std::make_unique<FilterNode>(); };
    return t;
}

// ---- Analog filter: the nonlinear ZDF models (transistor ladder, OTA cascade, coupled diode ladder) with their drive,
// character and tolerance controls. The legacy models stay in filter.svf.
class AnalogFilterNode final : public Node
{
public:
    void prepare(const NodePrepare& p) override
    {
        rate = p.sampleRate;
        slotIndex = p.index % kCardFilters;
        batched = p.batchPairs;
        if (!batched)
        {
            unit.prepare(p.sampleRate, p.quality);
            return;
        }
        // The lead of a voice pair owns the four-lane object of each model: lanes 0 and 1 are its own left and right
        // channels, lanes 2 and 3 those of the next voice.
        if (!p.pairLead)
            return;
        using Factory = std::unique_ptr<AnalogFilter> (*)();
        static constexpr Factory factories[kModels] = {makeTransistorLadder, makeOtaCascade,        makeDiodeLadder,
                                                       makeSallenKeyLowpass, makeSallenKeyHighpass, makeSvfOta,
                                                       makeOta4Pole,         makePassiveHp};
        for (int m = 0; m < kModels; ++m)
        {
            quads[static_cast<size_t>(m)] = factories[m]();
            quads[static_cast<size_t>(m)]->prepare(p.sampleRate, p.quality, 4, kChunkSize);
        }
    }
    void reset() override
    {
        if (batched)
            stale = kAllModels;
        else
            unit.reset();
    }
    void noteOn(const NodeNote& n) override
    {
        // With a voice card the component values are those of the voice, the same note after note.
        const bool carded = n.card != nullptr && n.cardAmounts.filter > 0.0f;
        seed = carded ? n.card->filterSeed[static_cast<size_t>(slotIndex)] : n.seed;
        if (batched)
        {
            stale = kAllModels;
            return;
        }
        unit.setSeed(seed);
        unit.reset();
    }
    void process(NodeIO& io, int n) override
    {
        const float drive = io.param(3);
        unit.configure(settings(io), drive, rate);
        const float* inL = io.in[0].left;
        const float* inR = io.in[0].right;
        if (io.rateConnected(0))
        {
            // Audio-rate cutoff: 5 octaves per unit of signal, sampled every four samples by the nonlinear models.
            float octaves[kChunkSize];
            cutoffOctaves(io, n, octaves);
            std::memcpy(io.out[0].left, inL, sizeof(float) * static_cast<size_t>(n));
            std::memcpy(io.out[0].right, inR, sizeof(float) * static_cast<size_t>(n));
            unit.processBlock(io.out[0].left, io.out[0].right, n, octaves);
            return;
        }
        std::memcpy(io.out[0].left, inL, sizeof(float) * static_cast<size_t>(n));
        std::memcpy(io.out[0].right, inR, sizeof(float) * static_cast<size_t>(n));
        unit.processBlock(io.out[0].left, io.out[0].right, n, nullptr);
    }
    bool pairsVoices() const override { return batched; }
    bool processPair(NodeIO* ioA, Node* nodeB, NodeIO* ioB, int n) override
    {
        if (!batched || !quads[0])
            return false;
        auto* b = static_cast<AnalogFilterNode*>(nodeB);
        const NodeIO& ref = ioA != nullptr ? *ioA : *ioB;
        const int model = std::clamp(static_cast<int>(ref.param(0)), 0, kModels - 1);
        AnalogFilter& q = *quads[static_cast<size_t>(model)];
        float* lanes[4];
        const float* mods[4] = {nullptr, nullptr, nullptr, nullptr};
        float octaves[2][kChunkSize];
        bool modulated = false;
        const auto feed = [&](AnalogFilterNode& node, NodeIO* io, int first, int slot)
        {
            if (io == nullptr)
            {
                // A lane whose voice does not sound runs on silence.
                std::memset(spare[slot * 2], 0, sizeof(float) * kChunkSize);
                std::memset(spare[slot * 2 + 1], 0, sizeof(float) * kChunkSize);
                lanes[first] = spare[slot * 2];
                lanes[first + 1] = spare[slot * 2 + 1];
                return;
            }
            node.startLanes(q, first, model);
            const FilterSettings s = node.settings(*io);
            const float fc = std::clamp(s.cutoffHz, 20.0f, 0.45f * static_cast<float>(rate));
            const float res = std::clamp(s.resonance, 0.0f, 1.0f);
            for (int c = 0; c < 2; ++c)
                q.setLaneControls(first + c, fc, res, io->param(3), s.character, s.tolerance);
            if (highpass[static_cast<size_t>(model)] != s.highpass)
            {
                q.setHighpassStep(s.highpass);
                highpass[static_cast<size_t>(model)] = s.highpass;
            }
            std::memcpy(io->out[0].left, io->in[0].left, sizeof(float) * static_cast<size_t>(n));
            std::memcpy(io->out[0].right, io->in[0].right, sizeof(float) * static_cast<size_t>(n));
            lanes[first] = io->out[0].left;
            lanes[first + 1] = io->out[0].right;
            if (io->rateConnected(0))
            {
                cutoffOctaves(*io, n, octaves[slot]);
                mods[first] = mods[first + 1] = octaves[slot];
                modulated = true;
            }
        };
        feed(*this, ioA, 0, 0);
        if (b != nullptr)
            feed(*b, ioB, 2, 1);
        else
            feed(*this, nullptr, 2, 1);
        q.processQuad(lanes, n, modulated ? mods : nullptr);
        return true;
    }

private:
    static constexpr int kModels = FilterChannel::kAnalogModels;
    static constexpr unsigned kAllModels = (1U << kModels) - 1U;

    FilterSettings settings(const NodeIO& io) const
    {
        FilterSettings s;
        s.model = static_cast<int>(FilterModel::TransistorLadder) + static_cast<int>(io.param(0));
        const float key = io.param(6);
        s.cutoffHz = std::clamp(io.param(1) * std::exp2(5.0f * io.mod(0) + key * (io.note - 60.0f) / 12.0f), 20.0f,
                                0.45f * static_cast<float>(rate));
        s.resonance = io.param(2);
        s.character = io.param(4);
        s.tolerance = io.param(5);
        if (io.card != nullptr && io.cardAmounts.filter > 0.0f)
            s.tolerance = std::max(s.tolerance, io.card->filterTolerance(slotIndex, io.cardAmounts.filter));
        s.highpass = static_cast<int>(io.param(7));
        return s;
    }
    static void cutoffOctaves(const NodeIO& io, int n, float* octaves)
    {
        for (int i = 0; i < n; ++i)
            octaves[i] = 5.0f * io.rate(0)[i];
    }
    // The state a note (or a model change, as FilterChannel::configure resets on one) gives the two lanes of this voice
    // in the model's quad: the component draws of a mono filter under this voice's seed, and the start state.
    void startLanes(AnalogFilter& q, int first, int model)
    {
        if (lastModel != model)
        {
            lastModel = model;
            stale |= 1U << model;
        }
        if ((stale >> model & 1U) == 0U)
            return;
        stale &= ~(1U << model);
        for (int c = 0; c < 2; ++c)
        {
            q.setLaneSeed(first + c, seed + 16U * static_cast<std::uint32_t>(c) + static_cast<std::uint32_t>(model), 0);
            q.resetLane(first + c);
        }
    }

    FilterUnit unit;
    double rate = 48000.0;
    int slotIndex = 0;
    bool batched = false;
    std::uint32_t seed = 1;
    unsigned stale = kAllModels; // models whose lanes start afresh the next time they run
    int lastModel = -1;
    std::array<std::unique_ptr<AnalogFilter>, kModels> quads; // lead of a voice pair only
    std::array<int, kModels> highpass = {-1, -1, -1, -1, -1, -1, -1, -1};
    float spare[4][kChunkSize] = {}; // silent input of a lane without a sounding voice (lead only)
};

NodeType analogFilterType()
{
    NodeType t;
    t.type = "filter.analog";
    t.name = "Analog Filter";
    t.kind = ModuleKind::Filter;
    t.category = "Filter";
    t.description =
        "Nonlinear zero-delay-feedback filters: transistor ladder, OTA cascade, coupled diode ladder, Sallen-Key, OTA "
        "state-variable, four-pole OTA and passive high-pass, with drive, high-pass step, gain-compensation character and "
        "seeded component tolerance.";
    t.params.push_back(params::choice("model", "Model", "filter",
                                      {"Transistor Ladder", "OTA Cascade", "Coupled Diode Ladder", "Sallen-Key LP",
                                       "Sallen-Key HP", "OTA SVF", "OTA 4-Pole", "Passive HP"},
                                      0));
    t.params.push_back(params::floating("cutoff", "Cutoff", "filter", 20.0f, 20000.0f, 8000.0f, "Hz", 0.3f, Smoothing::Log));
    t.params.push_back(params::floating("res", "Resonance", "filter", 0.0f, 1.0f, 0.15f));
    t.params.push_back(params::floating("drive", "Drive", "filter", 0.0f, 1.0f, 0.0f));
    t.params.push_back(params::floating("character", "Character", "filter", 0.0f, 1.0f, 0.0f));
    t.params.push_back(params::floating("tolerance", "Tolerance", "filter", 0.0f, 1.0f, 0.0f));
    t.params.push_back(params::floating("key", "Keytrack", "filter", 0.0f, 1.0f, 0.0f));
    t.params.push_back(params::choice("hp", "High-pass", "filter", {"Off", "60 Hz", "180 Hz", "540 Hz"}, 0));
    t.inputs = {{"in", PortType::AudioStereo}, {"cutoff", PortType::Modulation}, {"cutoff_fm", PortType::AudioRate}};
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate&) { return std::make_unique<AnalogFilterNode>(); };
    return t;
}

// ---- VCA: level, velocity sensitivity and an optional gain modulation input (an envelope).
class VcaNode final : public Node
{
public:
    void process(NodeIO& io, int n) override
    {
        const float vs = std::clamp(io.param(1), 0.0f, 1.0f);
        float g = io.param(0) * ((1.0f - vs) + vs * io.velocity);
        if (io.card != nullptr && io.cardAmounts.level > 0.0f)
            g *= io.card->gain(io.cardAmounts.level);
        if (io.modConnected(0))
            g *= std::max(io.mod(0), 0.0f);
        if (io.rateConnected(0))
        {
            // Amplitude modulation at audio rate: the gain follows 1 + signal, per sample (never below zero).
            for (int i = 0; i < n; ++i)
            {
                const float gi = g * std::max(1.0f + io.rate(0)[i], 0.0f);
                io.out[0].left[i] = io.in[0].left[i] * gi;
                io.out[0].right[i] = io.in[0].right[i] * gi;
            }
            return;
        }
        for (int i = 0; i < n; ++i)
        {
            io.out[0].left[i] = io.in[0].left[i] * g;
            io.out[0].right[i] = io.in[0].right[i] * g;
        }
    }
};

NodeType vcaType()
{
    NodeType t;
    t.type = "amp.vca";
    t.name = "VCA";
    t.kind = ModuleKind::Modulator;
    t.category = "Amplifier";
    t.description = "Gain stage: level, velocity sensitivity and a gain modulation input.";
    t.params.push_back(params::floating("level", "Level", "vca", 0.0f, 2.0f, 1.0f));
    t.params.push_back(params::floating("velocity", "Velocity", "vca", 0.0f, 1.0f, 0.0f));
    t.inputs = {{"in", PortType::AudioStereo}, {"gain", PortType::Modulation}, {"am", PortType::AudioRate}};
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate&) { return std::make_unique<VcaNode>(); };
    return t;
}

// ---- Output: sums its inputs; level in dB and pan. Every voice graph (or layer) ends in one.
class OutNode final : public Node
{
public:
    void process(NodeIO& io, int n) override
    {
        float gl = 0.0f, gr = 0.0f;
        panGains(io.param(1), gl, gr);
        const float g = dbToGain(io.param(0));
        for (int i = 0; i < n; ++i)
        {
            io.out[0].left[i] = io.in[0].left[i] * gl * g;
            io.out[0].right[i] = io.in[0].right[i] * gr * g;
        }
    }
};

NodeType outType()
{
    NodeType t;
    t.type = "mix.out";
    t.name = "Output";
    t.kind = ModuleKind::Modulator;
    t.category = "Mixer";
    t.description = "Voice output: sums its inputs, with level and pan.";
    t.params.push_back(params::floating("level", "Level", "out", -60.0f, 6.0f, 0.0f, "dB", 2.5f));
    t.params.push_back(params::floating("pan", "Pan", "out", -1.0f, 1.0f, 0.0f));
    t.inputs = {{"in", PortType::AudioStereo}};
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate&) { return std::make_unique<OutNode>(); };
    return t;
}

NodeType classicType()
{
    NodeType t;
    t.type = "classic.voice";
    t.name = "Classic Voice";
    t.kind = ModuleKind::Voice;
    t.category = "Voice";
    t.description = "The engine's whole voice: wavetable and module oscillators, filters, envelopes, LFOs and the "
                    "modulation matrix. Its parameters are the product's flat ids.";
    t.outputs = {{"out", PortType::AudioStereo}};
    t.create = [](const NodeCreate&) { return std::make_unique<ClassicVoiceNode>(); };
    return t;
}
} // namespace

int portIndex(const std::vector<PortDef>& ports, const std::string& name)
{
    for (size_t i = 0; i < ports.size(); ++i)
        if (ports[i].name == name)
            return static_cast<int>(i);
    return -1;
}

void NodeRegistry::add(NodeType type)
{
    for (auto& t : list)
        if (t.type == type.type)
        {
            t = std::move(type);
            return;
        }
    list.push_back(std::move(type));
}

void NodeRegistry::addEffect(ModuleDescriptor effect)
{
    for (auto& e : effectList)
        if (e.id == effect.id)
        {
            e = std::move(effect);
            return;
        }
    effectList.push_back(std::move(effect));
}

const ModuleDescriptor* NodeRegistry::findEffect(const std::string& id) const
{
    for (const auto& e : effectList)
        if (e.id == id)
            return &e;
    return nullptr;
}

const NodeType* NodeRegistry::find(const std::string& type) const
{
    for (const auto& t : list)
        if (t.type == type)
            return &t;
    return nullptr;
}

const NodeRegistry& NodeRegistry::builtIn()
{
    static const NodeRegistry registry = []
    {
        NodeRegistry r;
        r.add(classicType());
        r.add(oscillatorType("osc.noise", "Noise", "The noise oscillator module (colours, tilt, width) as a node."));
        r.add(oscillatorType("osc.sub", "Sub", "The sub oscillator module (five waves, three octaves, drive) as a node."));
        r.add(oscillatorType("osc.sample", "Sample", "The sample and granular oscillator module as a node."));
        r.add(oscillatorType("osc.cloud", "Cloud", "The additive cloud oscillator module as a node."));
        r.add(oscillatorType("osc.analog", "Analog (VA)",
                             "The virtual analog oscillator module (saw, pulse, triangle, sine, sync, FM) as a node."));
        r.add(oscillatorType("osc.modal", "Modal",
                             "The modal resonator and string oscillator module (Rings / Elements designs) as a node."));
        r.add(oscillatorType(
            "osc.harmonic", "Harmonic",
            "The harmonic additive oscillator module (count, odd / even, stretch, tilt, formant, decay) as a node."));
        r.add(oscillatorType("osc.spectral", "Spectral",
                             "The time-stretching sample oscillator module (Signalsmith Stretch) as a node."));
        r.add(envelopeType());
        r.add(analogEnvelopeType(false));
        r.add(analogEnvelopeType(true));
        r.add(lfoType());
        r.add(audioLfoType());
        r.add(filterType());
        r.add(analogFilterType());
        r.add(vcaType());
        r.add(outType());
        return r;
    }();
    return registry;
}

} // namespace crucible::graph
