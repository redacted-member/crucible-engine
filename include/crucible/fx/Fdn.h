// SPDX-License-Identifier: MIT
#pragma once

// Algorithmic reverb suite (docs/plans/ENGINE-FX-PLAN.md, section 3.5, package FX6). Clean-room designs from the
// published literature:
//
//   Fdn               Jot's feedback delay network (Jot and Chaigne, "Digital delay networks for designing artificial
//                     reverberators", AES 1991): 16 lines (32 at High quality), co-prime base lengths, an orthogonal
//                     Hadamard or Householder feedback matrix, and in every line a two-section first-order shelving
//                     absorption filter whose gains are set from the line's own length so that the reverberation time
//                     is the requested one at DC, in the mid band and at Nyquist (low and high decay multipliers with
//                     their crossover frequencies). Reads are Kaiser-windowed sinc; each line's length follows a smooth
//                     random signal (chorusing), and a frozen network is held at constant energy.
//   EarlyReflections  Tapped delays for the first 20 to 120 ms: image-source shoebox taps (Allen and Berkley 1979) with
//                     velvet-noise polarity for Room and Chamber, jittered velvet-noise taps (Karjalainen and
//                     Jarvelainen 2007) for Hall and Ambience, none for Plate.
//   PlateTank         The figure-eight all-pass tank of Dattorro ("Effect Design Part 1: Reverberator and Other
//                     Filters", JAES 1997): input diffusion, two cross-coupled branches of a modulated all-pass, a delay,
//                     the same absorption filter as the FDN, a second all-pass and a delay, output taps of my own.
//   AlgorithmicReverb All of it with pre-delay, input diffusion, ducking, stereo width, wet filters, mix, freeze and a
//                     click-free algorithm change; this is what the module fx.reverb_algo drives.
//
// Nothing on the audio thread allocates. prepare() allocates; process() and deactivate() are real-time safe, and
// deactivate() clears the state a bounded number of samples per call.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace crucible::fx
{

enum class ReverbAlgorithm : int
{
    Room,
    Hall,
    Chamber,
    Plate,
    Ambience,
    Count
};

const char* reverbAlgorithmName(ReverbAlgorithm algorithm);

// ---------------------------------------------------------------------------------------------------------------
// Building blocks

constexpr int kSincTaps = 8;
constexpr int kSincPhases = 512;
// Kaiser-windowed sinc, kSincPhases fractional positions of kSincTaps taps each. Built once; the largest magnitude of
// any phase is scaled to 1, so a loop that reads through it never gains energy.
const float* sincKernel();

struct ClearSpan
{
    float* data = nullptr;
    std::size_t size = 0;
};

// Zeroes a list of memory spans a bounded number of samples at a time (the review-fix pattern for big delay lines).
class SpanClearer
{
public:
    void setSpans(std::vector<ClearSpan> list)
    {
        spans = std::move(list);
        restart();
    }
    void restart()
    {
        item = 0;
        offset = 0;
    }
    // Clears at most `budget` more samples (and takes them off `budget`); true once every span is clear.
    bool step(int& budget);
    void clearAll();
    std::size_t totalSamples() const;

private:
    std::vector<ClearSpan> spans;
    std::size_t item = 0, offset = 0;
};

// Power-of-two ring with a guard zone so that an eight-tap read never wraps.
class DelayBuffer
{
public:
    static constexpr int kGuard = 40;
    void allocate(int minimumSamples);
    void clear();
    ClearSpan span() { return {data.data(), data.size()}; }
    int capacity() const { return mask + 1; }
    int writeIndex() const { return pos; }
    void write(float x)
    {
        data[static_cast<std::size_t>(pos)] = x;
        if (pos < kGuard)
            data[static_cast<std::size_t>(mask + 1 + pos)] = x;
        pos = (pos + 1) & mask;
    }
    // The sample written `delay` samples before the next write; delay >= 1.
    float tap(int delay) const { return data[static_cast<std::size_t>((pos - delay) & mask)]; }
    // Pointer to the sample written `back` samples before the next write; the samples after it (up to kGuard - 1 of
    // them) are the newer ones, with no wrap.
    const float* window(int back) const { return data.data() + ((pos - back) & mask); }
    // Linear interpolation, delay >= 1.
    float readLinear(float delay) const
    {
        const int whole = static_cast<int>(delay);
        const float f = delay - static_cast<float>(whole);
        const float a = data[static_cast<std::size_t>((pos - whole) & mask)];
        const float b = data[static_cast<std::size_t>((pos - whole - 1) & mask)];
        return a + (b - a) * f;
    }
    // Windowed-sinc read, delay in 1/65536 samples, at least 6 samples and below capacity() - 8.
    float readSinc(std::int64_t delayQ16) const
    {
        const std::int64_t p = (static_cast<std::int64_t>(pos) << 16) - delayQ16;
        const int base = static_cast<int>(p >> 16);
        const float* k = sinc + static_cast<std::size_t>((p >> 7) & (kSincPhases - 1)) * kSincTaps;
        const float* d = data.data() + ((base - 3) & mask);
        float sum = 0.0f;
        for (int j = 0; j < kSincTaps; ++j)
            sum += k[j] * d[j];
        return sum;
    }
    // The same read with the two nearest kernel phases blended: the position is exact to 1/65536 sample instead of
    // 1/512, which matters while the delay glides (the phase quantisation is then audible as a steady grain).
    float readSincFine(std::int64_t delayQ16) const
    {
        const std::int64_t p = (static_cast<std::int64_t>(pos) << 16) - delayQ16;
        return sincFine(sinc, data.data(), p, mask);
    }
    static float sincFine(const float* ker, const float* data, std::int64_t p, int mask)
    {
        const int ph = static_cast<int>((p >> 7) & (kSincPhases - 1));
        const float fr = static_cast<float>(p & 127) * (1.0f / 128.0f);
        const int base = static_cast<int>(p >> 16) - 3;
        const float* d0 = data + (base & mask);
        const float* k0 = ker + static_cast<std::size_t>(ph) * kSincTaps;
        const bool wrap = ph == kSincPhases - 1;
        const float* d1 = wrap ? data + ((base + 1) & mask) : d0;
        const float* k1 = wrap ? ker : k0 + kSincTaps;
        float a = 0.0f, b = 0.0f;
        for (int j = 0; j < kSincTaps; ++j)
        {
            a += k0[j] * d0[j];
            b += k1[j] * d1[j];
        }
        return a + (b - a) * fr;
    }
    static std::int64_t toQ16(float samples) { return static_cast<std::int64_t>(samples * 65536.0f + 0.5f); }

private:
    std::vector<float> data{std::vector<float>(2 * kGuard + 4, 0.0f)};
    const float* sinc = sincKernel();
    int pos = 0;
    int mask = 3;
};

// Smooth random signal in [-1, 1]: random breakpoints joined by smoothstep segments (continuous first derivative).
class SmoothNoise
{
public:
    void seed(std::uint32_t s)
    {
        state = s == 0 ? 0x9E3779B9u : s;
        phase = 0.0f;
        prev = random();
        target = random();
    }
    // `segments` is how many breakpoints pass in this step (rate in Hz times the step length in seconds).
    float next(float segments)
    {
        phase += segments;
        while (phase >= 1.0f)
        {
            phase -= 1.0f;
            prev = target;
            target = random();
        }
        const float s = phase * phase * (3.0f - 2.0f * phase);
        return prev + (target - prev) * s;
    }
    float random()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return static_cast<float>(state >> 8) * (2.0f / 16777216.0f) - 1.0f;
    }

private:
    std::uint32_t state = 0x9E3779B9u;
    float phase = 0.0f, prev = 0.0f, target = 0.0f;
};

// Per-line decay target. decay is the reverberation time (RT60) of the middle band; the low and high multipliers
// scale it below lowHz and above highHz (first-order shelves, half the change at the crossover).
struct DecayTarget
{
    float decay = 2.5f;
    float lowMult = 1.0f;
    float highMult = 1.0f;
    float lowHz = 250.0f;
    float highHz = 5000.0f;
};

// Two first-order sections (low shelf with the broadband gain folded in, then high shelf), transposed direct form II.
struct AbsorptionCoefficients
{
    float lb0 = 1, lb1 = 0, la1 = 0;
    float hb0 = 1, hb1 = 0, ha1 = 0;
};

// Jot's absorption design for a loop of `delaySeconds`: the filter's gain at DC, the middle band and Nyquist is
// 10^(-3 delay / RT60) of the respective reverberation time. klow and khigh are tan(pi f / fs) of the crossovers.
AbsorptionCoefficients designAbsorption(float delaySeconds, const DecayTarget& target, float klow, float khigh);

// tan(pi f / fs) of the two crossovers after the clamping every user of the absorption design applies.
void absorptionCrossovers(const DecayTarget& target, double sampleRate, float& klow, float& khigh);
// The reverberation time that the absorption filter designed for a loop of `delaySeconds` implies at `hz`.
float absorptionDecayAt(float hz, double sampleRate, float delaySeconds, const DecayTarget& target);

// Keeps a frozen network at the level it had when it was frozen. The sinc reads and rounding would let the loop gain
// creep over minutes, and the beating of the few surviving modes moves the output level by about a decibel over a few
// seconds; the guard measures the output power, takes its mean over the first half second as the reference and
// then steers a loop gain of at most +-2 % with a time constant of a few seconds.
class FreezeGuard
{
public:
    void reset()
    {
        gain = 1.0f;
        smoothed = reference = 0.0f;
        sum = 0.0f;
        armedSeconds = 0.0f;
    }
    // Called once per chunk with the mean power of that chunk while frozen. Returns the loop gain factor.
    float update(float chunkPower, float chunkSeconds)
    {
        if (armedSeconds < 0.5f)
        {
            sum += chunkPower * chunkSeconds;
            armedSeconds += chunkSeconds;
            smoothed = reference = sum / armedSeconds;
            return gain;
        }
        smoothed += (chunkPower - smoothed) * std::min(1.0f, chunkSeconds / 0.5f);
        if (smoothed > 1.0e-30f && reference > 1.0e-30f)
            gain *= std::exp(0.5f * 1.0f * chunkSeconds * std::log(reference / smoothed));
        gain = gain < 0.98f ? 0.98f : (gain > 1.02f ? 1.02f : gain);
        return gain;
    }
    float current() const { return gain; }

private:
    float gain = 1.0f, smoothed = 0.0f, reference = 0.0f, sum = 0.0f, armedSeconds = 0.0f;
};

// Chain of Schroeder all-passes with fixed integer lengths.
class AllpassChain
{
public:
    static constexpr int kMaxStages = 8;
    void allocate(int stages, int maxLength);
    // Lengths in samples (each at most maxLength) and the gain of every stage.
    void setup(const int* lengths, const float* gains, int stages);
    // n > 0: the gains glide linearly to the new values over the next n samples (not on the first call after a rewind).
    void setGains(const float* gains, int stages, int n = 0);
    void clear();
    void rewind()
    {
        position.fill(0);
        gliding = false;
        remaining = 0;
    }
    void addSpans(std::vector<ClearSpan>& out);
    float process(float x)
    {
        if (remaining > 0)
        {
            for (int k = 0; k < active; ++k)
                gains[static_cast<std::size_t>(k)] += step[static_cast<std::size_t>(k)];
            if (--remaining == 0)
                gains = target;
        }
        for (int k = 0; k < active; ++k)
        {
            float* b = buffers[static_cast<std::size_t>(k)].data();
            const float d = b[position[k]];
            const float g = gains[static_cast<std::size_t>(k)];
            const float w = x + g * d;
            b[position[k]] = w;
            x = d - g * w;
            if (++position[k] >= length[k])
                position[k] = 0;
        }
        return x;
    }

private:
    std::array<std::vector<float>, kMaxStages> buffers;
    std::array<int, kMaxStages> length{}, position{};
    std::array<float, kMaxStages> gains{}, target{}, step{};
    int active = 0, remaining = 0;
    bool gliding = false;
};

// ---------------------------------------------------------------------------------------------------------------
// Feedback delay network

constexpr int kFdnMaxLines = 32;

class Fdn
{
public:
    struct Profile
    {
        int lines = 16;
        float minMs = 20.0f, maxMs = 120.0f; // line lengths at size scale 1
        bool householder = false;            // else fast Walsh-Hadamard
        std::uint32_t seed = 1;
    };
    struct Controls
    {
        float scale = 1.0f; // size: multiplies every line length
        DecayTarget decay;
        float modDepth = 0.3f; // 0..1
        float modRate = 1.0f;  // Hz
        int freeze = 0;        // 0 off, 1 engaging (loop closed), 2 held (energy guard on)
    };

    // maxLines is 16 or 32; maxDelaySeconds the longest line the profiles may ask for at the largest scale.
    void prepare(double sampleRate, int maxLines, double maxDelaySeconds);
    void configure(const Profile& profile);
    void clear();
    // Filter states, energy guard and modulation back to the start (the buffers are cleared separately).
    void resetState();
    void addSpans(std::vector<ClearSpan>& out);
    // Once per chunk of n samples, before process(): line lengths, modulation, absorption.
    void update(const Controls& controls, int n);
    void process(const float* inL, const float* inR, float* outL, float* outR, int n);
    // Gain that makes the energy of the impulse response equal to that of the input (at the middle-band decay).
    float wetGain() const { return gainNormal; }
    int lines() const { return active; }
    double meanLineSeconds() const { return meanSeconds; }

private:
    void designVectors(std::uint32_t seed);
    void absorptionFor(const Controls& controls);

    double sr = 48000.0;
    int maxLines = 16, active = 16, mask = 0, stride = 0, writePos = 0;
    bool householderMix = false, fresh = true;
    std::vector<float> storage;
    alignas(32) std::array<float, kFdnMaxLines> baseSamples{}, inGainL{}, inGainR{}, outGainL{}, outGainR{};
    alignas(32) std::array<float, kFdnMaxLines> lb0{}, lb1{}, la1{}, hb0{}, hb1{}, ha1{}, lz{}, hz{};
    // Absorption coefficients glide linearly to their new design over one chunk (no per-chunk staircase).
    std::array<std::array<float, kFdnMaxLines>, 6> coefTarget{}, coefStep{};
    bool coefMoving = false;
    bool fineRead = false; // size is gliding: blend kernel phases in the line reads
    std::array<std::int64_t, kFdnMaxLines> delayQ{}, stepQ{};
    std::array<SmoothNoise, kFdnMaxLines> noise;
    std::array<float, kFdnMaxLines> noiseSpeed{};
    float feedbackScale = 1.0f, gainNormal = 1.0f, chunkSeconds = 0.0f, energyAccum = 0.0f;
    double meanSeconds = 0.05;
    // The control values the absorption was last designed for.
    float lastScale = -1.0f, lastModDepth = 0.0f;
    DecayTarget lastDecay;
    int lastFreeze = -1;
    float klow = 0.0f, khigh = 0.0f;
    FreezeGuard guard;
    int freezeState = 0;
    int chunkSamples = 0;
    std::uint32_t profileSeed = 1;
};

// ---------------------------------------------------------------------------------------------------------------
// Early reflections

class EarlyReflections
{
public:
    static constexpr int kMaxTaps = 32;
    void prepare(double sampleRate, std::uint32_t seed);
    void select(ReverbAlgorithm algorithm);
    void clear() { buffer.clear(); }
    void addSpans(std::vector<ClearSpan>& out) { out.push_back(buffer.span()); }
    int tapCount() const { return taps; }
    // mono in, stereo out (written, not added). scale multiplies the room dimensions; density 0..1 fades taps in.
    void process(const float* mono, float* outL, float* outR, int n, float scale, float density);

private:
    struct Tap
    {
        float delayL = 0, delayR = 0; // seconds at scale 1
        float gainL = 0, gainR = 0;
    };
    struct Table
    {
        std::array<Tap, kMaxTaps> tap{};
        int count = 0;
    };
    void buildShoebox(Table& t, float lx, float ly, float lz, float reflect, std::uint32_t seed) const;
    void buildVelvet(Table& t, float startMs, float endMs, float tauMs, int count, std::uint32_t seed) const;

    double sr = 48000.0;
    std::array<Table, static_cast<std::size_t>(ReverbAlgorithm::Count)> tables;
    int selected = -1, taps = 0;
    // Scale and active-tap count of the previous chunk: the delays and weights glide from them (no per-chunk steps).
    float lastScale = 1.0f, lastActive = 0.0f;
    bool primed = false;
    DelayBuffer buffer;
};

// ---------------------------------------------------------------------------------------------------------------
// Plate

class PlateTank
{
public:
    struct Controls
    {
        float scale = 1.0f;
        DecayTarget decay;
        float modDepth = 0.3f;
        float modRate = 1.0f;
        float diffusion = 0.7f; // 0..1
        int freeze = 0;
    };
    void prepare(double sampleRate, std::uint32_t seed);
    void clear();
    void resetState();
    void addSpans(std::vector<ClearSpan>& out);
    void update(const Controls& controls, int n);
    // mono is the diffused input.
    void process(const float* mono, float* outL, float* outR, int n);
    float wetGain() const { return gainNormal; }
    // Seconds of one pass through a branch's loss filter at the given size scale (half the ring).
    double loopSeconds(float scale) const
    {
        double ring = 0.0;
        for (const float b : baseSamples)
            ring += static_cast<double>(b);
        return 0.5 * ring * static_cast<double>(scale) / sr;
    }

private:
    enum Node
    {
        Ap1L,
        D1L,
        Ap2L,
        D2L,
        Ap1R,
        D1R,
        Ap2R,
        D2R,
        NodeCount
    };
    struct OutTap
    {
        int node;
        float fraction;
        float sign;
    };
    static constexpr int kOutTaps = 7;

    double sr = 48000.0;
    std::array<DelayBuffer, NodeCount> line;
    std::array<float, NodeCount> baseSamples{};
    std::array<std::int64_t, NodeCount> delayQ{}, stepQ{};
    std::array<std::int64_t, NodeCount> tapDelayQ{};
    std::array<SmoothNoise, 2> noise;
    std::array<float, 2> noiseSpeed{};
    std::uint32_t seedValue = 1;
    std::array<float, 2> lb0{}, lb1{}, la1{}, hb0{}, hb1{}, ha1{}, lz{}, hz{};
    std::array<float, 2> feedback{};
    std::array<std::array<OutTap, kOutTaps>, 2> outputs{};
    float g1 = 0.7f, g2 = 0.5f;
    float gainNormal = 1.0f, feedbackScale = 1.0f, energyAccum = 0.0f, chunkSeconds = 0.0f;
    float lastScale = -1.0f;
    DecayTarget lastDecay;
    int lastFreeze = -1, freezeState = 0, chunkSamples = 0;
    float klow = 0.0f, khigh = 0.0f;
    bool fresh = true;
    FreezeGuard guard;
};

// ---------------------------------------------------------------------------------------------------------------
// The complete reverb

struct ReverbSettings
{
    ReverbAlgorithm algorithm = ReverbAlgorithm::Hall;
    float mix = 0.3f;
    float decaySeconds = 2.5f; // RT60 of the middle band
    float size = 0.5f;         // 0..1, 0.5 is the nominal room
    float predelayMs = 15.0f;
    float density = 0.7f;  // input diffusion and number of early reflections, 0..1
    float early = 0.5f;    // early reflection level, 0..1
    float lowMult = 1.0f;  // decay multiplier below lowHz
    float highMult = 0.7f; // decay multiplier above highHz
    float lowHz = 250.0f;
    float highHz = 4000.0f;
    float modDepth = 0.3f;      // 0..1
    float modRate = 1.0f;       // Hz
    float width = 1.0f;         // 0 mono .. 1 fully decorrelated
    float lowCutHz = 80.0f;     // wet high-pass; 20 Hz or less bypasses it
    float highCutHz = 20000.0f; // wet low-pass; 20 kHz or more bypasses it
    bool freeze = false;
    float duck = 0.0f; // 0..1: wet reduction (up to 30 dB) while the input is loud
    float duckReleaseMs = 250.0f;
};

class AlgorithmicReverb
{
public:
    static constexpr int kChunk = 32;

    // lines: 16, or 32 for the high-quality network. Control thread.
    void prepare(double sampleRate, int lines, std::uint32_t seed);
    // Control thread: forgets everything at once.
    void reset();
    // Audio thread, called instead of process() while the effect is bypassed: clears a bounded number of samples.
    void deactivate();
    void process(float* l, float* r, int n, const ReverbSettings& settings);

    // Time the wet signal needs to fall 60 dB after the input stops, at the last settings; infinite while frozen.
    double tailSeconds() const { return tail.load(std::memory_order_relaxed); }
    bool tailInfinite() const { return infinite.load(std::memory_order_relaxed); }
    int lines() const { return lineCount; }
    // Seconds one round of the current network takes (mean line length, or half the plate's ring); valid after process().
    double loopSeconds() const { return loopSecs; }
    // Samples of memory deactivate() clears in total, at most 65536 per call.
    std::size_t stateSamples() const { return clearer.totalSamples(); }
    // True when every smoothed control of the last block sat exactly on its target (the static path is in use).
    bool controlsSettled() const { return smoothAllSettled; }

private:
    enum class Phase
    {
        Running,
        FadingOut,
        Clearing,
        FadingIn
    };
    void configureAlgorithm(ReverbAlgorithm algorithm);
    void buildClearList();
    void finishClear();
    void processChunk(float* l, float* r, int n, const ReverbSettings& s);
    // Control smoothing: three cascaded one-poles per parameter at chunk rate (continuous slope, no ramp corners).
    struct Smooth3
    {
        double a = 0.0, b = 0.0, c = 0.0; // double: at the slow rates (size, predelay) a float one-pole stalls far from the target
    };
    enum SmoothSlot
    {
        kSmMix,
        kSmWidth,
        kSmDuck,
        kSmEarly,
        kSmLowMult,
        kSmHighMult,
        kSmLowHz,
        kSmHighHz,
        kSmLowCut,
        kSmHighCut,
        kSmSize,
        kSmPredelay,
        kSmDensity,
        kSmDecay,
        kSmModDepth,
        kSmCount
    };
    float smoothValue(SmoothSlot slot, float target, bool logDomain, float k);
    ReverbSettings smoothSettings(const ReverbSettings& s, int n);
    void stepPhase(const ReverbSettings& s, int n, float& fadeFrom, float& fadeTo);

    double sr = 48000.0;
    int lineCount = 16;
    std::uint32_t seed = 1;
    Fdn fdn;
    PlateTank plate;
    EarlyReflections er;
    AllpassChain diffL, diffR;
    DelayBuffer preL, preR;
    SpanClearer clearer;

    ReverbAlgorithm current = ReverbAlgorithm::Hall, pending = ReverbAlgorithm::Hall;
    bool configured = false;
    Phase phase = Phase::Running;
    float fade = 1.0f;

    bool primed = false, dirty = false, clearing = false;
    float mixNow = 0.0f, wetGainNow = 1.0f, erGainNow = 0.0f;
    float predelayNow = 0.0f;
    float scaleNow = 1.0f;
    float widthNow = 1.0f;
    float earlyNow = 0.0f;
    float onsetNow = 0.0f;
    float hpKNow = 0.0f, lpKNow = 0.0f;
    float inputGain = 1.0f;
    int freezeSettle = 0;
    int diffusionStages = 4;
    bool frozen = false;
    // Wet filters: high-pass and low-pass one-poles per channel (trapezoidal).
    float hpState[2] = {0, 0}, lpState[2] = {0, 0};
    float envelope = 0.0f, duckGainNow = 1.0f;
    Smooth3 smooth[kSmCount];
    bool smoothPrimed = false, smoothAllSettled = true;

    double loopSecs = 0.05;
    std::atomic<double> tail{2.5};
    std::atomic<bool> infinite{false};
};

} // namespace crucible::fx
