// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Sample.h>

#include <signalsmith-stretch/signalsmith-stretch.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>

// Spectral oscillator (P13): the slot's sample played through the Signalsmith Stretch phase-vocoder time stretcher (MIT, see
// third_party/signalsmith-stretch/), so that the playback speed (spec_stretch, 0.25x to 4x of the sample's duration) and the
// pitch are independent. The pitch follows the key (relative to spec_root) plus spec_semi, and spec_formant moves the
// formants against the pitch.
//
// Latency is removed at the start of a note: the stretcher is primed with outputSeek() from the first samples of the region
// so that the output starts at the sample's first sample, not one analysis block later. That priming runs a few blocks of
// the transform in one control cell (a short, bounded cost at note start; see docs/MEASUREMENTS.md). The stretcher (0.8 MB)
// is built by arm(), when the engine first sees the type selected on a slot, because every voice owns an instance of every
// type; the render path does not allocate, and no analysis outside the stretcher's own STFT is done.
//
// The stretcher reads the original-rate audio of level 0; if the sample's rate differs from the engine's, the input is
// resampled with a cubic interpolator before it enters the stretcher (the stretcher itself works at the engine rate).

namespace crucible::modules
{
namespace
{
constexpr int kMaxChunk = 32;
constexpr double kBlockSeconds = 0.1;
constexpr double kIntervalSeconds = 0.025;
constexpr float kMaxRate = 4.0f; // input samples per output sample at the fastest (0.25x stretch)

struct Channels
{
    std::array<float*, 2> p{};
    float* operator[](int c) const { return p[static_cast<size_t>(c)]; }
};

// One sample of the original audio at frame position p (cubic Hermite; zero outside unless looping).
float readSample(const SampleLevel& lv, int channel, double p, bool looping, double loopStart)
{
    const int frames = lv.frames;
    const auto& chosen = lv.data[static_cast<size_t>(channel)];
    const auto& data = chosen.empty() ? lv.data[0] : chosen; // mono samples feed both channels
    if (data.empty())
        return 0.0f;
    const double len = static_cast<double>(frames) - loopStart;
    if (looping && len > 4.0)
    {
        if (p >= static_cast<double>(frames - 1))
            p = loopStart + std::fmod(p - loopStart, len);
    }
    else if (p < 0.0 || p >= static_cast<double>(frames - 1))
    {
        return 0.0f;
    }
    const int i = static_cast<int>(p);
    const float f = static_cast<float>(p - i);
    // data is padded by kSamplePad zeros on both sides.
    const float* x = data.data() + kSamplePad + i;
    const float xm1 = x[-1], x0 = x[0], x1 = x[1], x2 = x[2];
    const float c = (x1 - xm1) * 0.5f;
    const float vv = x0 - x1;
    const float w = c + vv;
    const float a = w + vv + (x2 - x0) * 0.5f;
    const float bNeg = w + a;
    return (((a * f) - bNeg) * f + c) * f + x0;
}

// The audio of a region as a stretcher input: source[channel][i] is the sample i engine-rate samples after the start.
struct Source
{
    const SampleLevel* level;
    double start, ratio;
    bool looping;
    struct Channel
    {
        const Source* source;
        int channel;
        float operator[](int i) const
        {
            return readSample(*source->level, channel, source->start + static_cast<double>(i) * source->ratio,
                              source->looping, source->start);
        }
    };
    Channel operator[](int c) const { return {this, c}; }
};

// What a slot playing this type needs and the others do not: the stretcher (about 0.8 MB) and the chunk input. Built by
// arm().
struct Heavy
{
    signalsmith::stretch::SignalsmithStretch<float> stretch{1};
    std::array<std::vector<float>, 2> in;
};

class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        stretchP = sp.bind("spec_stretch");
        semi = sp.bind("spec_semi");
        root = sp.bind("spec_root");
        key = sp.bind("spec_key");
        start = sp.bind("spec_start");
        loop = sp.bind("spec_loop");
        formant = sp.bind("spec_formant");
    }
    ~Instance() override { delete heavy.load(); }

    void prepare(double rate, int) override
    {
        sr = rate;
        delete heavy.exchange(nullptr); // re-armed by the engine (audio is stopped while it prepares)
        fresh = true;
        idle = true;
    }

    void arm() override
    {
        if (heavy.load(std::memory_order_acquire) != nullptr || sr <= 0.0)
            return;
        auto* h = new Heavy;
        h->stretch.configure(2, static_cast<int>(sr * kBlockSeconds), static_cast<int>(sr * kIntervalSeconds));
        h->stretch.reset();
        for (auto& b : h->in)
            b.assign(static_cast<size_t>(kMaxRate * kMaxChunk) + 16, 0.0f);
        heavy.store(h, std::memory_order_release);
    }

    void noteOn(const OscNoteInfo&) override
    {
        fresh = true;
        idle = false;
        acc = 0.0;
        pos = 0.0;
    }

    void render(const OscRender& whole) override
    {
        for (int done = 0; done < whole.numSamples; done += kMaxChunk)
        {
            OscRender part = whole;
            part.numSamples = std::min(kMaxChunk, whole.numSamples - done);
            part.left = whole.left + done;
            part.right = whole.right + done;
            part.noteTime = whole.noteTime + static_cast<double>(done) / whole.sampleRate;
            renderChunk(part);
        }
    }

private:
    void renderChunk(const OscRender& r)
    {
        Heavy* h = heavy.load(std::memory_order_acquire);
        if (h == nullptr || idle || r.sample == nullptr || r.sample->levels.empty() || r.numSamples <= 0)
            return;
        auto& stretch = h->stretch;
        auto& in = h->in;
        const SampleData& d = *r.sample;
        const SampleLevel& lv = d.levels[0];
        if (lv.frames < 2)
            return;
        const auto v = [&](int p) { return sp.value(r.values, r.targetMod, p); };
        const int n = std::min(r.numSamples, kMaxChunk);
        const float stretchFactor = std::clamp(v(stretchP), 0.25f, 4.0f);
        const bool keytrack = v(key) > 0.5f;
        const float semitones = (keytrack ? r.pitch - v(root) : 0.0f) + v(semi);
        if (!std::isfinite(semitones))
            return;
        const bool looping = static_cast<int>(v(loop)) != 0;
        const double ratio = d.sampleRate / r.sampleRate; // source frames per engine-rate input sample
        const double startFrame = std::clamp(static_cast<double>(v(start)), 0.0, 1.0) * static_cast<double>(lv.frames - 1);

        stretch.setTransposeSemitones(std::clamp(semitones, -48.0f, 48.0f), static_cast<float>(8000.0 / r.sampleRate));
        stretch.setFormantSemitones(std::clamp(v(formant), -24.0f, 24.0f));

        const double rate = 1.0 / static_cast<double>(stretchFactor); // engine-rate input samples per output sample
        if (fresh)
        {
            // Prime: the first seekLength samples of the region (the stretcher takes the first inputLatency of them as the
            // input it has already seen and renders the rest ahead, which makes the output start with the first sample).
            // They are read on demand through a proxy, so no buffer is needed.
            const int seekLength = stretch.outputSeekLength(static_cast<float>(rate));
            Source source{&lv, startFrame, ratio, looping};
            pos = startFrame + static_cast<double>(seekLength) * ratio;
            stretch.outputSeek(source, seekLength);
            fresh = false;
            acc = 0.0;
        }

        // Input for this chunk: n / stretch samples (fractions carry over).
        acc += static_cast<double>(n) * rate;
        const int k = std::min(static_cast<int>(acc), static_cast<int>(in[0].size()));
        acc -= k;
        for (int c = 0; c < 2; ++c)
        {
            float* dst = in[static_cast<size_t>(c)].data();
            double p = pos;
            for (int i = 0; i < k; ++i)
            {
                dst[i] = readSample(lv, c, p, looping, startFrame);
                p += ratio;
            }
        }
        pos += static_cast<double>(k) * ratio;
        std::array<float, kMaxChunk> outL{}, outR{};
        Channels inputs{{in[0].data(), in[1].data()}};
        Channels outputs{{outL.data(), outR.data()}};
        stretch.process(inputs, k, outputs, n);
        for (int i = 0; i < n; ++i)
        {
            const float a = outL[static_cast<size_t>(i)], b = outR[static_cast<size_t>(i)];
            r.left[i] += std::isfinite(a) ? a : 0.0f;
            r.right[i] += std::isfinite(b) ? b : 0.0f;
        }
    }

    SlotParams sp;
    int stretchP = 0, semi = 0, root = 0, key = 0, start = 0, loop = 0, formant = 0;
    double sr = 0.0;
    double acc = 0.0, pos = 0.0;
    bool fresh = true, idle = true;
    std::atomic<Heavy*> heavy{nullptr};
};
} // namespace

ModuleDescriptor spectralOscillator(const VoiceGraphShape& shape)
{
    using namespace osc;
    ModuleDescriptor d;
    d.id = "osc.spectral";
    d.name = "Spectral";
    d.kind = ModuleKind::Oscillator;
    d.category = "Sample";
    d.cpu = CpuClass::Heavy;
    d.description = "The slot's sample through a phase-vocoder time stretcher (Signalsmith Stretch, MIT): speed (0.25x to "
                    "4x) and pitch are independent, with formant shift, a start position and a forward loop.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
    {
        const auto label = slotLabel(slot);
        const auto main = addSlotPage(d, slot, "spec", "Spectral", "LCD shows the sample and the playhead.");
        const auto extra =
            addSlotPage(d, slot, "spec_x", "Spectral Loop / Formant", "LCD shows the loop and the formant shift.");
        const auto add = [&d](ParamDef p) { d.params.push_back(std::move(p)); };
        add(voiceMod(params::floating(slotId(slot, "spec_stretch"), label + " Spectral Time Stretch", main, 0.25f, 4.0f,
                                      1.0f, "x", 0.5f)));
        add(voiceMod(
            params::floating(slotId(slot, "spec_semi"), label + " Spectral Transpose", main, -24.0f, 24.0f, 0.0f, "st")));
        add(params::integer(slotId(slot, "spec_root"), label + " Spectral Root Note", main, 0, 127, 60, "note"));
        add(params::toggle(slotId(slot, "spec_key"), label + " Spectral Keytrack", main, true));
        add(voiceMod(params::floating(slotId(slot, "spec_start"), label + " Spectral Start", extra, 0.0f, 1.0f, 0.0f)));
        add(params::choice(slotId(slot, "spec_loop"), label + " Spectral Loop", extra, {"Off", "Forward"}, 0));
        add(voiceMod(
            params::floating(slotId(slot, "spec_formant"), label + " Spectral Formant", extra, -12.0f, 12.0f, 0.0f, "st")));
    }
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

const char* spectralOscillatorName()
{
    return "Spectral";
}

} // namespace crucible::modules
