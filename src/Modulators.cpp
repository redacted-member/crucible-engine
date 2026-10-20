// SPDX-License-Identifier: MIT
#include <crucible/Modulators.h>

#include <crucible/Dsp.h>

#include <algorithm>
#include <cmath>

namespace crucible
{

namespace
{
constexpr float kEpsilon = 1.0e-5f;

bool readsInputA(const ModulatorConfig& c)
{
    switch (c.type)
    {
    case ModulatorType::Combine:
    case ModulatorType::Remap:
    case ModulatorType::Lag:
    case ModulatorType::Quantise:
    case ModulatorType::SampleHold:
        return true;
    case ModulatorType::Follower:
        return c.mode == static_cast<int>(FollowMode::Input);
    default:
        return false;
    }
}

bool readsInputB(const ModulatorConfig& c)
{
    switch (c.type)
    {
    case ModulatorType::Combine:
        return true;
    case ModulatorType::SampleHold:
    case ModulatorType::Turing:
    case ModulatorType::Binary:
        return c.mode == static_cast<int>(ClockMode::Input);
    default:
        return false;
    }
}

struct Rand
{
    std::uint32_t& s;
    std::uint32_t next()
    {
        s ^= s << 13U;
        s ^= s >> 17U;
        s ^= s << 5U;
        return s;
    }
    float unipolar() { return static_cast<float>(next() >> 8U) * (1.0f / 16777216.0f); }
    float bipolar() { return unipolar() * 2.0f - 1.0f; }
};

// The eight frames of the LFO table: shapes of Lfo::shapeValue sampled at 128 points, morphed by position.
constexpr int kTableFrames = 8;
constexpr int kTablePoints = 128;

struct LfoFrames
{
    std::array<std::array<float, kTablePoints + 1>, kTableFrames> frame{};
    LfoFrames()
    {
        static constexpr int shapes[kTableFrames] = {0, 1, 2, 3, 7, 8, 9, 10};
        const Lfo lfo;
        for (int f = 0; f < kTableFrames; ++f)
        {
            for (int i = 0; i < kTablePoints; ++i)
                frame[static_cast<size_t>(f)][static_cast<size_t>(i)] =
                    lfo.shapeValue(shapes[f], static_cast<float>(i) / kTablePoints, nullptr);
            frame[static_cast<size_t>(f)][kTablePoints] = frame[static_cast<size_t>(f)][0];
        }
    }
    static const LfoFrames& instance()
    {
        static const LfoFrames frames;
        return frames;
    }
    float read(float position, float phase) const
    {
        const float p = std::clamp(position, 0.0f, 1.0f) * static_cast<float>(kTableFrames - 1);
        const int a = std::min(static_cast<int>(p), kTableFrames - 2);
        const float mix = p - static_cast<float>(a);
        const float x = phase * kTablePoints;
        const int i = std::min(static_cast<int>(x), kTablePoints - 1);
        const float f = x - static_cast<float>(i);
        const auto at = [&](int frameIndex)
        {
            const auto& fr = frame[static_cast<size_t>(frameIndex)];
            return fr[static_cast<size_t>(i)] + (fr[static_cast<size_t>(i) + 1] - fr[static_cast<size_t>(i)]) * f;
        };
        return at(a) + (at(a + 1) - at(a)) * mix;
    }
};

float readInput(const float* sources, int selector, int index)
{
    if (selector == 0 || !ModulatorSet::validInput(selector, index))
        return 0.0f;
    return sources[selector];
}

bool inputBipolar(const ModulatorSet& set, int selector)
{
    if (selector <= 0)
        return false;
    if (selector < kNumModSources)
        return isBipolarSource(static_cast<ModSource>(selector));
    const int n = selector - kNumModSources;
    return n < kNumBankModulators && set.bipolar[static_cast<size_t>(n)];
}

// Clock events in this chunk for the clocked types.
int clockEvents(const ModulatorConfig& c, ModulatorState& st, const ModulatorContext& ctx, const float* sources, int index)
{
    switch (static_cast<ClockMode>(std::clamp(c.mode, 0, 2)))
    {
    case ClockMode::Rate:
    {
        st.phase += std::max(c.rate, 0.0f) * static_cast<float>(ctx.numSamples / ctx.sampleRate);
        const float whole = std::floor(st.phase);
        st.phase -= whole;
        return std::min(static_cast<int>(whole), 16);
    }
    case ClockMode::Input:
    {
        const bool high = readInput(sources, c.inB, index) > 0.5f;
        const bool edge = high && !st.wasHigh;
        st.wasHigh = high;
        return edge ? 1 : 0;
    }
    case ClockMode::NoteOn:
        return ctx.noteOn ? 1 : 0;
    }
    return 0;
}

float stepLorenz(ModulatorState& st, double span)
{
    constexpr double sigma = 10.0, rho = 28.0, beta = 8.0 / 3.0;
    const int steps = std::clamp(static_cast<int>(std::ceil(span / 0.01)), 1, 64);
    const double h = span / steps;
    const auto deriv = [&](const double* s, double* d)
    {
        d[0] = sigma * (s[1] - s[0]);
        d[1] = s[0] * (rho - s[2]) - s[1];
        d[2] = s[0] * s[1] - beta * s[2];
    };
    double s[3] = {st.lx, st.ly, st.lz};
    for (int k = 0; k < steps; ++k)
    {
        double k1[3], k2[3], k3[3], k4[3], t[3];
        deriv(s, k1);
        for (int i = 0; i < 3; ++i)
            t[i] = s[i] + 0.5 * h * k1[i];
        deriv(t, k2);
        for (int i = 0; i < 3; ++i)
            t[i] = s[i] + 0.5 * h * k2[i];
        deriv(t, k3);
        for (int i = 0; i < 3; ++i)
            t[i] = s[i] + h * k3[i];
        deriv(t, k4);
        for (int i = 0; i < 3; ++i)
            s[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    }
    if (!std::isfinite(s[0]) || !std::isfinite(s[1]) || !std::isfinite(s[2]) || std::abs(s[0]) > 200.0)
    {
        s[0] = s[1] = s[2] = 1.0;
    }
    st.lx = s[0];
    st.ly = s[1];
    st.lz = s[2];
    return std::clamp(static_cast<float>(s[0] / 20.0), -1.0f, 1.0f);
}

void trackPitch(const ModulatorConfig& c, ModulatorState& st, const ModulatorContext& ctx)
{
    const double rate = ctx.sampleRate;
    for (int i = 0; i < ctx.audioCount; ++i)
    {
        const float s = ctx.audio[i];
        // Hysteresis: the signal must go below -threshold before an upward zero crossing counts.
        constexpr float threshold = 0.002f;
        if (s < -threshold)
            st.armed = true;
        if (st.armed && s >= 0.0f && st.lastSample < 0.0f)
        {
            const double crossing =
                st.samples - 1.0 + static_cast<double>(-st.lastSample) / static_cast<double>(s - st.lastSample);
            if (st.lastCross >= 0.0)
            {
                const double period = crossing - st.lastCross;
                const double hz = rate / period;
                if (hz >= 20.0 && hz <= 8000.0)
                {
                    const float logHz = static_cast<float>(std::log2(hz));
                    if (st.logHz == 0.0f)
                        st.logHz = logHz;
                    else
                    {
                        // Smooth over `rise` seconds of crossings (one crossing per period).
                        const double crossingsPerSecond = hz;
                        const float tau = std::max(c.rise, 0.0f);
                        const float a =
                            tau <= 0.0f ? 1.0f : 1.0f - std::exp(-1.0f / (static_cast<float>(crossingsPerSecond) * tau));
                        st.logHz += (logHz - st.logHz) * a;
                    }
                }
            }
            st.lastCross = crossing;
            st.armed = false;
        }
        st.lastSample = s;
        st.samples += 1.0;
    }
    // Keep the clock numerically small.
    if (st.samples > 4.0e6)
    {
        st.samples -= 4.0e6;
        if (st.lastCross >= 0.0)
            st.lastCross -= 4.0e6;
    }
}

void begin(const ModulatorConfig& c, ModulatorState& st, const float* sources, int index)
{
    st = ModulatorState{};
    st.started = true;
    st.rng = (c.seed * 2654435761U) | 1U;
    Rand r{st.rng};
    switch (c.type)
    {
    case ModulatorType::Turing:
        st.reg = (r.next() >> 8U) & 0xffffU;
        if (st.reg == 0U)
            st.reg = 0x5a5aU;
        break;
    case ModulatorType::Chaos:
        st.held = 0.1f + 0.8f * r.unipolar();
        st.previous = st.held;
        st.lx = 1.0 + 0.2 * (static_cast<double>(r.unipolar()) - 0.5);
        st.ly = 1.0 + 0.2 * (static_cast<double>(r.unipolar()) - 0.5);
        st.lz = 1.0 + 0.2 * (static_cast<double>(r.unipolar()) - 0.5);
        break;
    case ModulatorType::Lag:
        st.value = readInput(sources, c.inA, index);
        break;
    case ModulatorType::LfoTable:
        st.phase = 0.0f;
        break;
    default:
        break;
    }
}
} // namespace

bool ModulatorSet::validInput(int selector, int index)
{
    if (selector == 0)
        return true;
    if (selector < 0)
        return false;
    if (selector < kNumModSources)
        return true;
    return selector - kNumModSources < index;
}

void ModulatorSet::finalize(std::uint32_t usedByRoutes)
{
    active = 0;
    for (int n = 0; n < kNumBankModulators; ++n)
        if (mod[static_cast<size_t>(n)].type != ModulatorType::Off)
            active |= 1U << n;
    needed = usedByRoutes & active;
    for (int n = kNumBankModulators - 1; n >= 0; --n)
    {
        if ((needed >> n & 1U) == 0U)
            continue;
        const auto& c = mod[static_cast<size_t>(n)];
        for (const auto& [read, selector] : {std::pair{readsInputA(c), c.inA}, std::pair{readsInputB(c), c.inB}})
            if (read && validInput(selector, n) && selector >= kNumModSources)
                needed |= (1U << (selector - kNumModSources)) & active;
    }
    bipolar.fill(false);
    needsAudio = false;
    anyVoice = anyShared = false;
    for (int n = 0; n < kNumBankModulators; ++n)
    {
        const auto& c = mod[static_cast<size_t>(n)];
        const auto sel = [&](int s) { return validInput(s, n) && inputBipolar(*this, s); };
        bool bip = false;
        switch (c.type)
        {
        case ModulatorType::Off:
            break;
        case ModulatorType::Combine:
            bip = c.mode == static_cast<int>(CombineMode::Difference) || sel(c.inA) || sel(c.inB);
            break;
        case ModulatorType::Remap:
            for (const float p : c.curve)
                bip = bip || p < 0.0f;
            break;
        case ModulatorType::Lag:
        case ModulatorType::Quantise:
            bip = sel(c.inA);
            break;
        case ModulatorType::SampleHold:
            bip = c.inA == 0 || sel(c.inA);
            break;
        case ModulatorType::Turing:
        case ModulatorType::Chaos:
        case ModulatorType::PitchTracker:
        case ModulatorType::LfoTable:
            bip = true;
            break;
        case ModulatorType::Binary:
        case ModulatorType::NoteGate:
        case ModulatorType::Follower:
        case ModulatorType::Count:
            break;
        }
        bipolar[static_cast<size_t>(n)] = bip;
        if ((needed >> n & 1U) != 0U)
        {
            if (c.type == ModulatorType::PitchTracker ||
                (c.type == ModulatorType::Follower && c.mode == static_cast<int>(FollowMode::Audio)))
                needsAudio = true;
            (c.scope == ModulatorScope::Voice ? anyVoice : anyShared) = true;
        }
    }
}

void restartVoiceModulators(const ModulatorSet& set, ModulatorState* states)
{
    for (int n = 0; n < kNumBankModulators; ++n)
        if (set.mod[static_cast<size_t>(n)].scope == ModulatorScope::Voice)
            states[n].started = false;
}

void evaluateModulators(const ModulatorSet& set, ModulatorState* states, const ModulatorContext& ctx, float* sources)
{
    if (set.needed == 0U)
        return;
    const float dt = static_cast<float>(ctx.numSamples / ctx.sampleRate);
    for (int n = 0; n < kNumBankModulators; ++n)
    {
        if ((set.needed >> n & 1U) == 0U)
            continue;
        const ModulatorConfig& c = set.mod[static_cast<size_t>(n)];
        float& out = sources[kExtModulatorFirst + n];
        const bool mine = (c.scope == ModulatorScope::Voice) == ctx.voice;
        if (!mine)
        {
            if (!ctx.voice)
                out = 0.0f; // a shared evaluation cannot see a voice modulator
            continue;       // a voice evaluation finds the shared value in place
        }
        ModulatorState& st = states[n];
        bool fresh = false;
        if (!st.started)
        {
            begin(c, st, sources, n);
            fresh = true;
        }
        Rand rand{st.rng};
        float value = st.value;
        switch (c.type)
        {
        case ModulatorType::Off:
        case ModulatorType::Count:
            value = 0.0f;
            break;
        case ModulatorType::Combine:
        {
            const float a = readInput(sources, c.inA, n), b = readInput(sources, c.inB, n);
            switch (static_cast<CombineMode>(std::clamp(c.mode, 0, 5)))
            {
            case CombineMode::Add:
                value = a + b;
                break;
            case CombineMode::Multiply:
                value = a * b;
                break;
            case CombineMode::Min:
                value = std::min(a, b);
                break;
            case CombineMode::Max:
                value = std::max(a, b);
                break;
            case CombineMode::Crossfade:
                value = a + (b - a) * std::clamp(c.amount, 0.0f, 1.0f);
                break;
            case CombineMode::Difference:
                value = a - b;
                break;
            }
            break;
        }
        case ModulatorType::Remap:
        {
            const float a = readInput(sources, c.inA, n);
            const float lo = inputBipolar(set, c.inA) && ModulatorSet::validInput(c.inA, n) ? -1.0f : 0.0f;
            const float t = std::clamp((a - lo) / (1.0f - lo), 0.0f, 1.0f) * 15.0f;
            const int i = std::min(static_cast<int>(t), 14);
            const float f = t - static_cast<float>(i);
            value = c.curve[static_cast<size_t>(i)] +
                    (c.curve[static_cast<size_t>(i) + 1] - c.curve[static_cast<size_t>(i)]) * f;
            break;
        }
        case ModulatorType::Lag:
        {
            const float x = readInput(sources, c.inA, n);
            // A non-finite input holds the state, so the lag recovers once the input does.
            if (std::isfinite(x))
            {
                const float tau = x > st.value ? c.rise : c.fall;
                const float coefficient = tau <= 0.0f ? 1.0f : 1.0f - std::exp(-dt / tau);
                st.value += (x - st.value) * coefficient;
            }
            value = st.value;
            break;
        }
        case ModulatorType::Quantise:
        {
            const float a = readInput(sources, c.inA, n);
            const float lo = inputBipolar(set, c.inA) && ModulatorSet::validInput(c.inA, n) ? -1.0f : 0.0f;
            const int steps = std::clamp(c.steps, 2, 64);
            const float t = (std::clamp(a, lo, 1.0f) - lo) / (1.0f - lo) * static_cast<float>(steps - 1);
            float q;
            switch (static_cast<QuantiseMode>(std::clamp(c.mode, 0, 2)))
            {
            case QuantiseMode::Down:
                q = std::floor(t + kEpsilon);
                break;
            case QuantiseMode::Up:
                q = std::ceil(t - kEpsilon);
                break;
            default:
                q = std::floor(t + 0.5f);
                break;
            }
            value = lo + q / static_cast<float>(steps - 1) * (1.0f - lo);
            break;
        }
        case ModulatorType::SampleHold:
        {
            int events = clockEvents(c, st, ctx, sources, n);
            if (fresh)
                events = std::max(events, 1);
            if (events > 0)
                st.held = c.inA == 0 ? rand.bipolar() : readInput(sources, c.inA, n);
            value = st.held;
            break;
        }
        case ModulatorType::Turing:
        {
            const int events = clockEvents(c, st, ctx, sources, n);
            const int length = std::clamp(c.steps, 1, 16);
            for (int e = 0; e < events; ++e)
            {
                std::uint32_t bit = (st.reg >> (length - 1)) & 1U;
                if (rand.unipolar() < std::clamp(c.amount, 0.0f, 1.0f))
                    bit = rand.next() >> 31U;
                st.reg = ((st.reg << 1U) | bit) & 0xffffU;
            }
            value = static_cast<float>(st.reg & 0xffU) * (2.0f / 255.0f) - 1.0f;
            break;
        }
        case ModulatorType::Binary:
        {
            int events = clockEvents(c, st, ctx, sources, n);
            if (fresh)
                events = std::max(events, 1);
            for (int e = 0; e < events; ++e)
                st.held = rand.unipolar() < std::clamp(c.amount, 0.0f, 1.0f) ? 1.0f : 0.0f;
            value = st.held;
            break;
        }
        case ModulatorType::Chaos:
        {
            if (c.mode == static_cast<int>(ChaosMap::Lorenz))
            {
                value = stepLorenz(st, static_cast<double>(std::max(c.rate, 0.0f)) * ctx.numSamples / ctx.sampleRate);
            }
            else
            {
                const float r = 3.5f + 0.5f * std::clamp(c.amount, 0.0f, 1.0f);
                st.phase += std::max(c.rate, 0.0f) * dt;
                const float whole = std::floor(st.phase);
                st.phase -= whole;
                for (int e = std::min(static_cast<int>(whole), 64); e > 0; --e)
                {
                    st.previous = st.held;
                    st.held = r * st.held * (1.0f - st.held);
                    if (st.held < 1.0e-4f || st.held > 1.0f - 1.0e-4f)
                        st.held = 0.1f + 0.8f * rand.unipolar();
                }
                value = (st.previous + (st.held - st.previous) * st.phase) * 2.0f - 1.0f;
            }
            break;
        }
        case ModulatorType::NoteGate:
        {
            switch (static_cast<GateMode>(std::clamp(c.mode, 0, 2)))
            {
            case GateMode::Gate:
                value = ctx.gate ? 1.0f : 0.0f;
                break;
            case GateMode::Velocity:
                value = ctx.gate ? ctx.velocity : 0.0f;
                break;
            case GateMode::Trigger:
                if (ctx.noteOn)
                    st.gateTime = std::max(c.rise, 0.001f);
                value = st.gateTime > 0.0f ? 1.0f : 0.0f;
                st.gateTime -= dt;
                break;
            }
            break;
        }
        case ModulatorType::Follower:
        {
            const float attack =
                c.rise <= 0.0f ? 1.0f : 1.0f - std::exp(-1.0f / (c.rise * static_cast<float>(ctx.sampleRate)));
            const float release =
                c.fall <= 0.0f ? 1.0f : 1.0f - std::exp(-1.0f / (c.fall * static_cast<float>(ctx.sampleRate)));
            if (c.mode == static_cast<int>(FollowMode::Input))
            {
                const float x = std::abs(readInput(sources, c.inA, n));
                if (std::isfinite(x))
                {
                    const float tau = x > st.env ? c.rise : c.fall;
                    st.env += (x - st.env) * (tau <= 0.0f ? 1.0f : 1.0f - std::exp(-dt / tau));
                }
            }
            else
            {
                for (int i = 0; i < ctx.audioCount; ++i)
                {
                    const float x = std::abs(ctx.audio[i]);
                    if (std::isfinite(x))
                        st.env += (x - st.env) * (x > st.env ? attack : release);
                }
            }
            value = st.env;
            break;
        }
        case ModulatorType::PitchTracker:
        {
            trackPitch(c, st, ctx);
            // The Key source's scale: (note - 60) / 60 over five octaves either way.
            const float note = st.logHz == 0.0f ? 60.0f : 69.0f + 12.0f * (st.logHz - std::log2(440.0f));
            value = std::clamp((note - 60.0f) / 60.0f, -1.0f, 1.0f);
            break;
        }
        case ModulatorType::LfoTable:
        {
            st.phase += std::max(c.rate, 0.0f) * dt;
            st.phase -= std::floor(st.phase);
            value = LfoFrames::instance().read(c.amount, st.phase);
            break;
        }
        }
        if (!std::isfinite(value))
            value = 0.0f;
        st.value = c.type == ModulatorType::Lag ? st.value : value;
        out = value;
    }
}

const char* modulatorTypeName(ModulatorType type)
{
    switch (type)
    {
    case ModulatorType::Off:
        return "Off";
    case ModulatorType::Combine:
        return "Combine";
    case ModulatorType::Remap:
        return "Remap";
    case ModulatorType::Lag:
        return "Lag";
    case ModulatorType::Quantise:
        return "Quantise";
    case ModulatorType::SampleHold:
        return "S&H";
    case ModulatorType::Turing:
        return "Turing";
    case ModulatorType::Binary:
        return "Binary";
    case ModulatorType::Chaos:
        return "Chaos";
    case ModulatorType::NoteGate:
        return "Note Gate";
    case ModulatorType::Follower:
        return "Follower";
    case ModulatorType::PitchTracker:
        return "Pitch Tracker";
    case ModulatorType::LfoTable:
        return "LFO Table";
    case ModulatorType::Count:
        break;
    }
    return "Off";
}

} // namespace crucible
