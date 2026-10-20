// SPDX-License-Identifier: MIT
#include "../Modules.h"
#include "OscSupport.h"

#include <crucible/Dsp.h>
#include <crucible/Sample.h>
#include <crucible/Zones.h>

#include <array>
#include <atomic>
#include <cmath>

// Sample / granular oscillator: pitched playback of user audio with start, end and loop points, or a granular
// cloud of windowed grains read from the same audio. Reads use band-limited copies of the audio (two per octave)
// and a windowed-sinc interpolator, so transposing up does not fold energy below 20 kHz. Everything is
// allocation free on the audio thread; the audio and its slice markers are published by SampleBank.
//
// Unison (sample mode): up to eight voices read the sample at detuned rates, panned across the stereo field, their
// starts a few milliseconds apart; in granular mode successive grains take the detune and pan of successive voices.
// Slicing: the markers of SampleBank::setSlices cut the sample into slices. In "Per Note" mode the note
// (note - base) picks the slice, in "Key Range" mode the keys base .. top are shared out evenly among the slices;
// notes without a slice are silent. The slice replaces start / end as the played region (loop points stay relative to
// it); with keytracking on the slice plays at its original pitch on its own key (the middle of its keys in key-range
// mode), otherwise always at the original pitch.
// Zones (smp_zone_mode = Zones and a ZoneSet published by ZoneBank, see Zones.h): the first render of a note picks up to
// four zones for its key and velocity; each plays through the sample path with its own sample, root, gain, tuning and
// loop points, and the layers add up. Slices are ignored; granular mode reads the first zone only.

namespace crucible::modules
{
namespace
{
constexpr int kMaxGrains = 32;
static_assert(kMaxGrains <= kOscVisualMaxGrains, "the read-back views reserve room for this many grains per voice");
constexpr int kMaxUnison = 8;
constexpr int kWindowSize = 1024;
constexpr float kPi = 3.14159265f;

enum Mode
{
    ModeSample = 0,
    ModeGranular = 1
};
enum LoopMode
{
    LoopOff = 0,
    LoopForward = 1,
    LoopPingPong = 2
};
enum SliceMode
{
    SliceOff = 0,
    SlicePerNote = 1,
    SliceKeyRange = 2
};

// Playheads of the unison voices of one sample layer.
struct PlayState
{
    std::array<double, kMaxUnison> vpos{};
    std::array<double, kMaxUnison> vdir{};
    std::array<bool, kMaxUnison> vdone{};
    bool fresh = true;

    void reset()
    {
        vpos.fill(-1.0);
        vdir.fill(1.0);
        vdone.fill(false);
        fresh = true;
    }
};

// What a zone changes about the sample path of its layer.
struct ZonePlay
{
    float gain = 1.0f;
    float root = 60.0f;
    float tuneCents = 0.0f;
    int loopMode = -1;
    float loopStart = 0.0f, loopEnd = 1.0f;
    bool first = false; // publishes the playhead
};

const std::vector<std::string>& windowNames()
{
    static const std::vector<std::string> n = {"Hann", "Gauss", "Triangle", "Trapezoid", "Blackman"};
    return n;
}

struct Windows
{
    std::array<std::array<float, kWindowSize + 1>, 5> table{};
    Windows()
    {
        for (int i = 0; i <= kWindowSize; ++i)
        {
            const float x = static_cast<float>(i) / kWindowSize;
            table[0][static_cast<size_t>(i)] = 0.5f - 0.5f * std::cos(2.0f * kPi * x);
            const float g = (x - 0.5f) / 0.17f;
            // Gaussian pulled to zero at the ends.
            table[1][static_cast<size_t>(i)] =
                std::max(0.0f, std::exp(-0.5f * g * g) - std::exp(-0.5f * (0.5f / 0.17f) * (0.5f / 0.17f))) /
                (1.0f - std::exp(-0.5f * (0.5f / 0.17f) * (0.5f / 0.17f)));
            table[2][static_cast<size_t>(i)] = 1.0f - std::abs(2.0f * x - 1.0f);
            table[3][static_cast<size_t>(i)] = std::min({1.0f, x / 0.25f, (1.0f - x) / 0.25f});
            table[4][static_cast<size_t>(i)] = 0.42f - 0.5f * std::cos(2.0f * kPi * x) + 0.08f * std::cos(4.0f * kPi * x);
        }
    }
    static const Windows& instance()
    {
        static const Windows w;
        return w;
    }
    float at(int type, float x) const
    {
        const float p = std::clamp(x, 0.0f, 1.0f) * kWindowSize;
        const int i = std::min(static_cast<int>(p), kWindowSize - 1);
        const float f = p - static_cast<float>(i);
        const auto& t = table[static_cast<size_t>(type)];
        return t[static_cast<size_t>(i)] + (t[static_cast<size_t>(i + 1)] - t[static_cast<size_t>(i)]) * f;
    }
};

struct Grain
{
    bool active = false;
    bool reverse = false;
    double pos = 0.0;  // source frames
    float step = 1.0f; // source frames per output sample (signed by the caller through `reverse`)
    int level = 0;     // band-limited copy chosen at spawn
    int age = 0;
    int length = 1;
    float amp = 0.0f;
    float gainL = 1.0f;
    float gainR = 1.0f;
};

class Instance final : public OscillatorInstance
{
public:
    Instance(int slot, const ParameterModel& model) : sp(slot, model)
    {
        mode = sp.bind("smp_mode");
        root = sp.bind("smp_root");
        start = sp.bind("smp_start");
        end = sp.bind("smp_end");
        loop = sp.bind("smp_loop");
        xfade = sp.bind("smp_xfade");
        loopStart = sp.bind("smp_loop_start");
        loopEnd = sp.bind("smp_loop_end");
        key = sp.bind("smp_key");
        reverse = sp.bind("smp_reverse");
        gpos = sp.bind("grain_pos");
        gsize = sp.bind("grain_size");
        gdensity = sp.bind("grain_density");
        gspray = sp.bind("grain_spray");
        gpitch = sp.bind("grain_pitch");
        gwindow = sp.bind("grain_window");
        grev = sp.bind("grain_rev");
        gpitchRand = sp.bind("grain_pitch_rand");
        gspread = sp.bind("grain_spread");
        gscan = sp.bind("grain_scan");
        gjitter = sp.bind("grain_jitter");
        uniVoices = sp.bind("smp_uni_voices");
        uniDetune = sp.bind("smp_uni_detune");
        uniSpread = sp.bind("smp_uni_spread");
        uniStart = sp.bind("smp_uni_start");
        sliceMode = sp.bind("smp_slice_mode");
        sliceBase = sp.bind("smp_slice_base");
        sliceTop = sp.bind("smp_slice_top");
        zoneMode = sp.bind("smp_zone_mode");
        zoneXfadeKey = sp.bind("smp_zone_xfade_key");
        zoneXfadeVel = sp.bind("smp_zone_xfade_vel");
        zoneRr = sp.bind("smp_zone_rr");
    }

    void prepare(double rate, int) override
    {
        sr = rate;
        SincKernel::instance();
        Windows::instance();
    }

    void noteOn(const OscNoteInfo& info) override
    {
        rng.state = (info.seed * 2246822519U) | 1U;
        baseNote = info.note;
        velocity = info.velocity;
        noteSeed = info.seed;
        main.reset();
        for (auto& z : zoneVoices)
            z.state.reset();
        zoneCount = 0;
        zoneChosen = false;
        finished = false;
        sliceIndex = 0;
        sliceRoot = baseNote;
        sliceChosen = false;
        uniCursor = 0;
        for (auto& g : grains)
            g.active = false;
        grainClock = 1.0;
        scan = 0.0;
        for (auto& a : vizActive)
            a.store(0.0f, std::memory_order_relaxed);
        for (auto& z : vizZone)
            z.store(-1, std::memory_order_relaxed);
        playhead.store(0.0f, std::memory_order_relaxed);
    }

    void render(const OscRender& r) override
    {
        const auto v = [&](int local) { return sp.value(r.values, r.targetMod, local); };
        if (r.zones != nullptr && r.zones->count() > 0 && v(zoneMode) > 0.5f)
        {
            renderZones(r, v);
            return;
        }
        const SampleData* data = r.sample;
        if (data == nullptr || finished)
        {
            playhead.store(-1.0f, std::memory_order_relaxed);
            return;
        }
        if (!sliceChosen && ((sliceChosen = true), !chooseSlice(r, *data, v)))
        {
            finished = true;
            playhead.store(-1.0f, std::memory_order_relaxed);
            return;
        }
        if (static_cast<int>(v(mode)) == ModeGranular)
            renderGranular(r, *data, v, nullptr);
        else
            finished = !renderSample(r, *data, v, main, nullptr);
    }

    void visual(OscVisual& out) const override
    {
        const float p = playhead.load(std::memory_order_relaxed);
        if (p >= 0.0f)
            out.playheads.push_back(p);
        for (const auto& z : vizZone)
            if (z.load(std::memory_order_relaxed) >= 0)
                out.zones.push_back(z.load(std::memory_order_relaxed));
        for (size_t i = 0; i < kMaxGrains; ++i)
            if (vizActive[i].load(std::memory_order_relaxed) > 0.0f)
                out.grains.push_back({vizPos[i].load(std::memory_order_relaxed), vizWidth[i].load(std::memory_order_relaxed),
                                      vizActive[i].load(std::memory_order_relaxed),
                                      vizPan[i].load(std::memory_order_relaxed)});
    }

private:
    struct Region
    {
        double begin, end; // source frames
    };

    static Region regionOf(const SampleData& d, float s, float e)
    {
        double a = std::clamp(static_cast<double>(s), 0.0, 1.0) * d.frames;
        double b = std::clamp(static_cast<double>(e), 0.0, 1.0) * d.frames;
        return minimumLength(d, a, b);
    }

    static Region minimumLength(const SampleData& d, double a, double b)
    {
        if (b < a)
            std::swap(a, b);
        if (b - a < 32.0)
        {
            b = std::min(static_cast<double>(d.frames), a + 32.0);
            a = std::max(0.0, b - 32.0);
        }
        return {a, b};
    }

    // The part of the sample this voice plays: the chosen slice when slicing is on, otherwise start..end.
    template <typename V>
    Region voiceRegion(const OscRender& r, const SampleData& d, const V& v, const ZonePlay* layer) const
    {
        if (layer != nullptr || static_cast<int>(v(sliceMode)) == SliceOff)
            return regionOf(d, v(start), v(end));
        double a = 0.0, b = static_cast<double>(d.frames);
        if (r.slices != nullptr)
            r.slices->region(sliceIndex, d.frames, a, b);
        return minimumLength(d, a, b);
    }

    // Slicing: picks the slice for this note (and the key its pitch is relative to). False when the note has no
    // slice, in which case the voice stays silent.
    template <typename V> bool chooseSlice(const OscRender& r, const SampleData&, const V& v)
    {
        const int sliceKind = static_cast<int>(v(sliceMode));
        if (sliceKind == SliceOff)
            return true;
        const int count = r.slices != nullptr ? r.slices->count() : 1;
        const int base = static_cast<int>(v(sliceBase));
        const int note = static_cast<int>(std::lround(baseNote));
        if (sliceKind == SlicePerNote)
        {
            const int index = note - base;
            if (index < 0 || index >= count)
                return false;
            sliceIndex = index;
            sliceRoot = static_cast<float>(note);
            return true;
        }
        // Key range: the keys base .. top are shared out evenly among the slices.
        const int top = std::max(base, static_cast<int>(v(sliceTop)));
        if (note < base || note > top)
            return false;
        const int keys = top - base + 1;
        sliceIndex = std::min(count - 1, (note - base) * count / keys);
        // The slice plays at its original pitch in the middle of its keys.
        const float first = static_cast<float>(base) + static_cast<float>(sliceIndex * keys) / static_cast<float>(count);
        const float last =
            static_cast<float>(base) + static_cast<float>((sliceIndex + 1) * keys) / static_cast<float>(count) - 1.0f;
        sliceRoot = 0.5f * (first + last);
        return true;
    }

    // Detune (cents), pan gains and weight of unison voice u of n.
    struct UnisonVoice
    {
        float cents = 0.0f;
        float gainL = 1.0f;
        float gainR = 1.0f;
    };
    static UnisonVoice unisonVoice(int u, int n, float detuneCents, float spread)
    {
        UnisonVoice out;
        if (n <= 1)
            return out;
        const float lin = static_cast<float>(u) / static_cast<float>(n - 1) * 2.0f - 1.0f;
        out.cents = lin * detuneCents;
        const float angle = (std::clamp(lin * spread, -1.0f, 1.0f) + 1.0f) * 0.78539816f;
        const float norm = 1.41421356f / std::sqrt(static_cast<float>(n));
        out.gainL = std::cos(angle) * norm;
        out.gainR = std::sin(angle) * norm;
        return out;
    }

    // Stereo read at a source position from the band-limited level chosen for the ratio.
    static void readAt(const SampleLevel& lv, int channels, double srcPos, float& l, float& rr)
    {
        const double p = srcPos / lv.decimation;
        const double fl = std::floor(p);
        const int idx = std::clamp(static_cast<int>(fl), 0, lv.frames - 1);
        const float frac = static_cast<float>(p - fl);
        const auto& k = SincKernel::instance();
        l = k.read<16>(lv.data[0].data() + kSamplePad, idx, frac);
        rr = channels > 1 ? k.read<16>(lv.data[1].data() + kSamplePad, idx, frac) : l;
    }

    // One sample layer: the whole voice, or a zone's layer. Returns true while a unison voice is still playing.
    template <typename V>
    bool renderSample(const OscRender& r, const SampleData& d, const V& v, PlayState& ps, const ZonePlay* layer)
    {
        const Region reg = voiceRegion(r, d, v, layer);
        const double len = reg.end - reg.begin;
        const bool rev = v(reverse) > 0.5f;
        const bool ownLoop = layer != nullptr && layer->loopMode >= 0;
        const int loopMode = ownLoop ? layer->loopMode : static_cast<int>(v(loop));
        const double ls =
            reg.begin + std::clamp(static_cast<double>(ownLoop ? layer->loopStart : v(loopStart)), 0.0, 1.0) * len;
        double le = reg.begin + std::clamp(static_cast<double>(ownLoop ? layer->loopEnd : v(loopEnd)), 0.0, 1.0) * len;
        double lo = std::min(ls, le), hi = std::max(ls, le);
        if (hi - lo < 16.0)
            hi = std::min(reg.end, lo + 16.0);
        const double loopLen = hi - lo;
        const double fade = std::clamp(static_cast<double>(v(xfade)), 0.0, 1.0) * 0.5 * loopLen;

        const bool sliced = layer == nullptr && static_cast<int>(v(sliceMode)) != SliceOff;
        float pitchOffset =
            v(key) > 0.5f ? r.pitch - (layer != nullptr ? layer->root : (sliced ? sliceRoot : static_cast<float>(v(root))))
                          : r.pitch - baseNote;
        if (layer != nullptr)
            pitchOffset += layer->tuneCents * 0.01f;
        const float layerGain = layer != nullptr ? layer->gain : 1.0f;
        const int voices = std::clamp(static_cast<int>(v(uniVoices)), 1, kMaxUnison);
        const float detuneCents = v(uniDetune);
        const float spread = v(uniSpread);
        auto& vpos = ps.vpos;
        auto& vdir = ps.vdir;
        auto& vdone = ps.vdone;

        if (ps.fresh)
        {
            // Unison voices start a few milliseconds apart so that they do not add up coherently at the attack.
            const double startSpan = std::min(0.5 * len, static_cast<double>(v(uniStart)) * 0.03 * d.sampleRate);
            for (int u = 0; u < voices; ++u)
            {
                const double offset = u == 0 ? 0.0 : static_cast<double>(rng.unipolar()) * startSpan;
                vpos[static_cast<size_t>(u)] = rev ? reg.end - 1.0 - offset : reg.begin + offset;
                vdir[static_cast<size_t>(u)] = rev ? -1.0 : 1.0;
                vdone[static_cast<size_t>(u)] = false;
            }
            ps.fresh = false;
        }
        bool anyActive = false;
        for (int u = 0; u < voices; ++u)
        {
            const auto ui = static_cast<size_t>(u);
            if (vdone[ui])
                continue;
            const UnisonVoice uv = unisonVoice(u, voices, detuneCents, spread);
            const double ratio = std::exp2(static_cast<double>(pitchOffset + uv.cents * 0.01f) / 12.0);
            const double step = ratio * d.sampleRate / r.sampleRate;
            const SampleLevel& lv = d.levelFor(ratio, r.sampleRate);
            double pos = vpos[ui];
            double dir = vdir[ui];
            for (int i = 0; i < r.numSamples; ++i)
            {
                float l, rr;
                readAt(lv, d.channels, pos, l, rr);
                if (loopMode == LoopForward && fade > 1.0)
                {
                    // Equal-power crossfade of the loop end into the material before the loop start.
                    const double into = dir > 0 ? pos - (hi - fade) : (lo + fade) - pos;
                    if (into > 0.0)
                    {
                        const float w = static_cast<float>(std::min(into / fade, 1.0));
                        const double other = dir > 0 ? pos - loopLen : pos + loopLen;
                        float l2, r2;
                        readAt(lv, d.channels, other, l2, r2);
                        const float a = std::cos(0.5f * kPi * w), b = std::sin(0.5f * kPi * w);
                        l = l * a + l2 * b;
                        rr = rr * a + r2 * b;
                    }
                }
                if (voices == 1)
                {
                    r.left[i] += l * layerGain;
                    r.right[i] += rr * layerGain;
                }
                else
                {
                    r.left[i] += l * uv.gainL * layerGain;
                    r.right[i] += rr * uv.gainR * layerGain;
                }

                pos += dir * step;
                if (loopMode == LoopOff)
                {
                    if (pos >= reg.end || pos < reg.begin)
                    {
                        vdone[ui] = true;
                        break;
                    }
                }
                else if (loopMode == LoopForward)
                {
                    if (pos >= hi)
                        pos -= loopLen;
                    else if (pos < lo)
                        pos += loopLen;
                }
                else
                {
                    if (pos >= hi)
                    {
                        pos = 2.0 * hi - pos;
                        dir = -1.0;
                    }
                    else if (pos < lo)
                    {
                        pos = 2.0 * lo - pos;
                        dir = 1.0;
                    }
                }
            }
            vpos[ui] = pos;
            vdir[ui] = dir;
            anyActive = anyActive || !vdone[ui];
        }
        if (layer == nullptr || layer->first)
            playhead.store(static_cast<float>(std::clamp(vpos[static_cast<size_t>(voices / 2)] / d.frames, 0.0, 1.0)),
                           std::memory_order_relaxed);
        return anyActive;
    }

    template <typename V> void renderGranular(const OscRender& r, const SampleData& d, const V& v, const ZonePlay* layer)
    {
        const Region reg = voiceRegion(r, d, v, layer);
        const double len = reg.end - reg.begin;
        const float sizeSec = v(gsize) * 0.001f;
        const float density = v(gdensity);
        const float spray = v(gspray);
        const float jitter = v(gjitter);
        const int win = std::clamp(static_cast<int>(v(gwindow)), 0, 4);
        const float revProb = v(grev);
        const float pitchRand = v(gpitchRand);
        const float spread = v(gspread);
        const bool keytrack = v(key) > 0.5f;
        const bool sliced = layer == nullptr && static_cast<int>(v(sliceMode)) != SliceOff;
        float pitchOffset =
            (keytrack ? r.pitch - (layer != nullptr ? layer->root : (sliced ? sliceRoot : static_cast<float>(v(root))))
                      : r.pitch - baseNote) +
            v(gpitch);
        if (layer != nullptr)
            pitchOffset += layer->tuneCents * 0.01f;
        const int voices = std::clamp(static_cast<int>(v(uniVoices)), 1, kMaxUnison);
        const float detuneCents = v(uniDetune);
        const float uniPanSpread = v(uniSpread);
        const int grainLen = std::max(16, static_cast<int>(sizeSec * static_cast<float>(r.sampleRate)));
        const float amp = 1.0f / std::sqrt(std::max(1.0f, density * sizeSec));
        const auto& windows = Windows::instance();
        const double scanSpeed = static_cast<double>(v(gscan));
        scan += scanSpeed * static_cast<double>(r.numSamples) / r.sampleRate;
        scan -= std::floor(scan);
        const double centre01 = std::clamp(static_cast<double>(v(gpos)), 0.0, 1.0);

        for (int i = 0; i < r.numSamples; ++i)
        {
            grainClock +=
                static_cast<double>(density) * (1.0 + jitter * 0.8 * static_cast<double>(rng.bipolar())) / r.sampleRate;
            while (grainClock >= 1.0)
            {
                grainClock -= 1.0;
                // Unison: successive grains take the detune and pan of successive unison voices.
                float cents = 0.0f, uniPan = 0.0f;
                if (voices > 1)
                {
                    const int u = uniCursor++ % voices;
                    const float lin = static_cast<float>(u) / static_cast<float>(voices - 1) * 2.0f - 1.0f;
                    cents = lin * detuneCents;
                    uniPan = lin * uniPanSpread;
                }
                spawn(d, reg, len, centre01, spray, pitchOffset + cents * 0.01f, pitchRand, revProb, spread, uniPan,
                      grainLen, amp, r.sampleRate);
            }
            float outL = 0.0f, outR = 0.0f;
            for (auto& g : grains)
            {
                if (!g.active)
                    continue;
                const float w = windows.at(win, (static_cast<float>(g.age) + 0.5f) / static_cast<float>(g.length));
                const SampleLevel& lv =
                    d.levels[static_cast<size_t>(std::min(g.level, static_cast<int>(d.levels.size()) - 1))];
                const double p = g.pos / lv.decimation;
                const double fl = std::floor(p);
                const int idx = std::clamp(static_cast<int>(fl), 0, lv.frames - 1);
                const float frac = static_cast<float>(p - fl);
                const auto& k = SincKernel::instance();
                const float l = k.read<8>(lv.data[0].data() + kSamplePad, idx, frac);
                const float rr = d.channels > 1 ? k.read<8>(lv.data[1].data() + kSamplePad, idx, frac) : l;
                outL += l * w * g.gainL * g.amp;
                outR += rr * w * g.gainR * g.amp;
                g.pos += g.reverse ? -static_cast<double>(g.step) : static_cast<double>(g.step);
                if (++g.age >= g.length || g.pos < 0.0 || g.pos >= d.frames)
                    g.active = false;
            }
            r.left[i] += outL;
            r.right[i] += outR;
        }
        // Visual state.
        size_t n = 0;
        for (const auto& g : grains)
        {
            if (!g.active || n >= kMaxGrains)
                continue;
            vizPos[n].store(static_cast<float>(g.pos / d.frames), std::memory_order_relaxed);
            vizWidth[n].store(static_cast<float>(g.step) * static_cast<float>(g.length) / static_cast<float>(d.frames),
                              std::memory_order_relaxed);
            vizActive[n].store(g.amp * windows.at(win, (static_cast<float>(g.age) + 0.5f) / static_cast<float>(g.length)) +
                                   1.0e-6f,
                               std::memory_order_relaxed);
            vizPan[n].store(g.gainR > g.gainL ? 1.0f : -1.0f, std::memory_order_relaxed);
            ++n;
        }
        for (; n < kMaxGrains; ++n)
            vizActive[n].store(0.0f, std::memory_order_relaxed);
        playhead.store(static_cast<float>((reg.begin + (centre01 + scan) * len) / d.frames), std::memory_order_relaxed);
    }

    // Zone path: picks the zones of the note at its first render, then runs one sample layer per zone.
    template <typename V> void renderZones(const OscRender& r, const V& v)
    {
        if (finished)
        {
            playhead.store(-1.0f, std::memory_order_relaxed);
            return;
        }
        if (!zoneChosen)
        {
            zoneChosen = true;
            ZoneSelect options;
            options.xfadeKey = v(zoneXfadeKey);
            options.xfadeVel = v(zoneXfadeVel);
            options.roundRobin = static_cast<ZoneRoundRobin>(std::clamp(static_cast<int>(v(zoneRr)), 0, 2));
            options.seed = noteSeed;
            std::array<crucible::ZoneLayer, kMaxZoneLayers> picked;
            const int n = selectZones(*r.zones, std::clamp(static_cast<int>(std::lround(baseNote)), 0, 127),
                                      std::clamp(static_cast<int>(std::lround(velocity * 127.0f)), 0, 127), options, picked);
            zoneCount = 0;
            for (int i = 0; i < n; ++i)
            {
                const Zone& z = r.zones->zones[static_cast<size_t>(picked[static_cast<size_t>(i)].zone)];
                if (r.zones->sampleFor(z, r.sample) == nullptr)
                    continue;
                auto& zv = zoneVoices[static_cast<size_t>(zoneCount)];
                zv.zone = picked[static_cast<size_t>(i)].zone;
                zv.layer.gain = picked[static_cast<size_t>(i)].weight * std::pow(10.0f, z.gainDb * 0.05f);
                zv.layer.root = z.root;
                zv.layer.tuneCents = z.tuneCents;
                zv.layer.loopMode = z.loopMode;
                zv.layer.loopStart = z.loopStart;
                zv.layer.loopEnd = z.loopEnd;
                zv.layer.first = zoneCount == 0;
                vizZone[static_cast<size_t>(zoneCount)].store(zv.zone, std::memory_order_relaxed);
                ++zoneCount;
            }
            if (zoneCount == 0)
                finished = true;
        }
        if (zoneCount == 0)
        {
            playhead.store(-1.0f, std::memory_order_relaxed);
            return;
        }
        if (static_cast<int>(v(mode)) == ModeGranular)
        {
            const auto& zv = zoneVoices[0];
            renderGranular(r, *r.zones->sampleFor(r.zones->zones[static_cast<size_t>(zv.zone)], r.sample), v, &zv.layer);
            return;
        }
        bool any = false;
        for (int i = 0; i < zoneCount; ++i)
        {
            auto& zv = zoneVoices[static_cast<size_t>(i)];
            const SampleData* d = r.zones->sampleFor(r.zones->zones[static_cast<size_t>(zv.zone)], r.sample);
            if (d != nullptr)
                any = renderSample(r, *d, v, zv.state, &zv.layer) || any;
        }
        finished = !any;
    }

    void spawn(const SampleData& d, const Region& reg, double len, double centre01, float spray, float pitchOffset,
               float pitchRand, float revProb, float spread, float uniPan, int grainLen, float amp, double outRate)
    {
        Grain* g = nullptr;
        for (auto& c : grains)
            if (!c.active)
            {
                g = &c;
                break;
            }
        if (g == nullptr)
            return;
        double p = centre01 + scan + static_cast<double>(spray) * static_cast<double>(rng.bipolar()) * 0.5;
        p -= std::floor(p); // positions wrap inside the region
        const float semis = pitchOffset + pitchRand * 0.01f * rng.bipolar();
        const double ratio = std::exp2(static_cast<double>(semis) / 12.0);
        g->level = d.levelIndexFor(ratio, outRate);
        g->step = static_cast<float>(ratio * d.sampleRate / outRate);
        g->reverse = rng.unipolar() < revProb;
        const double span = static_cast<double>(g->step) * grainLen;
        g->pos = reg.begin + p * len;
        if (g->reverse)
            g->pos = std::min(g->pos + span, static_cast<double>(d.frames - 1));
        g->age = 0;
        g->length = grainLen;
        g->amp = amp;
        const float pan = std::clamp(spread * rng.bipolar() + uniPan, -1.0f, 1.0f);
        const float angle = (pan + 1.0f) * 0.78539816f;
        g->gainL = std::cos(angle) * 1.41421356f;
        g->gainR = std::sin(angle) * 1.41421356f;
        g->active = true;
    }

    SlotParams sp;
    int mode = 0, root = 0, start = 0, end = 0, loop = 0, xfade = 0, loopStart = 0, loopEnd = 0, key = 0, reverse = 0;
    int gpos = 0, gsize = 0, gdensity = 0, gspray = 0, gpitch = 0, gwindow = 0, grev = 0, gpitchRand = 0, gspread = 0;
    int gscan = 0, gjitter = 0;
    int uniVoices = 0, uniDetune = 0, uniSpread = 0, uniStart = 0, sliceMode = 0, sliceBase = 0, sliceTop = 0;
    int zoneMode = 0, zoneXfadeKey = 0, zoneXfadeVel = 0, zoneRr = 0;
    double sr = 48000.0;
    Rng rng;
    float baseNote = 60.0f;
    float velocity = 1.0f;
    std::uint32_t noteSeed = 1;
    PlayState main;
    struct ZoneVoice
    {
        PlayState state;
        ZonePlay layer;
        int zone = -1;
    };
    std::array<ZoneVoice, kMaxZoneLayers> zoneVoices{};
    int zoneCount = 0;
    bool zoneChosen = false;
    std::array<std::atomic<int>, kMaxZoneLayers> vizZone{};
    int sliceIndex = 0;
    float sliceRoot = 60.0f;
    bool sliceChosen = false;
    int uniCursor = 0;
    bool finished = false;
    double grainClock = 1.0;
    double scan = 0.0;
    std::array<Grain, kMaxGrains> grains{};
    std::atomic<float> playhead{-1.0f};
    std::array<std::atomic<float>, kMaxGrains> vizPos{}, vizWidth{}, vizActive{}, vizPan{};
};

void addSampleSlot(ModuleDescriptor& d, int slot)
{
    using namespace osc;
    auto& p = d.params;
    const auto label = slotLabel(slot);
    const auto main = addSlotPage(d, slot, "smp", "Sample", "LCD shows the sample waveform with start, end and playhead.");
    const auto loopPage = addSlotPage(d, slot, "smp_2", "Loop", "LCD shows the loop region over the waveform.");
    const auto grain = addSlotPage(d, slot, "grain", "Grains", "LCD shows the grain positions over the waveform.");
    const auto grain2 = addSlotPage(d, slot, "grain_2", "Grain Shape", "LCD shows the grain window and scatter.");
    const auto unison = addSlotPage(d, slot, "smp_uni", "Sample Unison",
                                    "LCD shows the detuned voices across the stereo field; strips set voices and detune.");
    const auto slices = addSlotPage(d, slot, "smp_slice", "Slices",
                                    "LCD shows the waveform with slice markers and the key range of each slice.");
    const auto zonesPage =
        addSlotPage(d, slot, "smp_zone", "Zones",
                    "LCD shows the key and velocity map of the zones; strips set the crossfades and round robin.");

    p.push_back(params::choice(slotId(slot, "smp_mode"), label + " Sample Mode", main, {"Sample", "Granular"}, 0));
    p.push_back(params::integer(slotId(slot, "smp_root"), label + " Root Note", main, 0, 127, 60, "note"));
    p.push_back(voiceMod(params::floating(slotId(slot, "smp_start"), label + " Sample Start", main, 0.0f, 1.0f, 0.0f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "smp_end"), label + " Sample End", main, 0.0f, 1.0f, 1.0f)));
    p.push_back(params::choice(slotId(slot, "smp_loop"), label + " Loop", main, {"Off", "Forward", "Ping-Pong"}, 0));
    p.push_back(voiceMod(params::floating(slotId(slot, "smp_xfade"), label + " Loop Crossfade", main, 0.0f, 1.0f, 0.0f)));

    p.push_back(
        voiceMod(params::floating(slotId(slot, "smp_loop_start"), label + " Loop Start", loopPage, 0.0f, 1.0f, 0.0f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "smp_loop_end"), label + " Loop End", loopPage, 0.0f, 1.0f, 1.0f)));
    p.push_back(params::toggle(slotId(slot, "smp_key"), label + " Keytrack", loopPage, true));
    p.push_back(params::toggle(slotId(slot, "smp_reverse"), label + " Reverse", loopPage, false));

    p.push_back(voiceMod(params::floating(slotId(slot, "grain_pos"), label + " Grain Position", grain, 0.0f, 1.0f, 0.5f)));
    p.push_back(voiceMod(
        params::floating(slotId(slot, "grain_size"), label + " Grain Size", grain, 1.0f, 500.0f, 80.0f, "ms", 0.4f)));
    p.push_back(voiceMod(
        params::floating(slotId(slot, "grain_density"), label + " Grain Density", grain, 0.5f, 300.0f, 25.0f, "Hz", 0.4f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "grain_spray"), label + " Grain Spray", grain, 0.0f, 1.0f, 0.1f)));
    p.push_back(
        voiceMod(params::floating(slotId(slot, "grain_pitch"), label + " Grain Pitch", grain, -24.0f, 24.0f, 0.0f, "st")));
    p.push_back(params::choice(slotId(slot, "grain_window"), label + " Grain Window", grain, windowNames(), 0));

    p.push_back(voiceMod(params::floating(slotId(slot, "grain_rev"), label + " Grain Reverse", grain2, 0.0f, 1.0f, 0.0f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "grain_pitch_rand"), label + " Grain Pitch Random", grain2, 0.0f,
                                          1200.0f, 0.0f, "ct", 0.5f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "grain_spread"), label + " Grain Spread", grain2, 0.0f, 1.0f, 0.5f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "grain_scan"), label + " Grain Scan", grain2, -2.0f, 2.0f, 0.0f)));
    p.push_back(voiceMod(params::floating(slotId(slot, "grain_jitter"), label + " Grain Jitter", grain2, 0.0f, 1.0f, 0.3f)));

    // Unison and slicing come last: they are plain (not modulatable) parameters, so the generic modulation
    // destination numbers of existing patches do not move.
    p.push_back(params::integer(slotId(slot, "smp_uni_voices"), label + " Sample Unison Voices", unison, 1, kMaxUnison, 1));
    p.push_back(params::floating(slotId(slot, "smp_uni_detune"), label + " Sample Unison Detune", unison, 0.0f, 100.0f,
                                 15.0f, "ct"));
    p.push_back(params::floating(slotId(slot, "smp_uni_spread"), label + " Sample Unison Spread", unison, 0.0f, 1.0f, 0.6f));
    p.push_back(
        params::floating(slotId(slot, "smp_uni_start"), label + " Sample Unison Start Offset", unison, 0.0f, 1.0f, 0.3f));

    p.push_back(
        params::choice(slotId(slot, "smp_slice_mode"), label + " Slice Mode", slices, {"Off", "Per Note", "Key Range"}, 0));
    p.push_back(params::integer(slotId(slot, "smp_slice_base"), label + " Slice Base Note", slices, 0, 127, 36, "note"));
    p.push_back(params::integer(slotId(slot, "smp_slice_top"), label + " Slice Top Note", slices, 0, 127, 96, "note"));

    // Zones (content in ZoneBank): registered last like the slice parameters, so existing destination numbers do not move.
    p.push_back(params::choice(slotId(slot, "smp_zone_mode"), label + " Zone Mode", zonesPage, {"Off", "Zones"}, 0));
    p.push_back(params::floating(slotId(slot, "smp_zone_xfade_key"), label + " Zone Key Crossfade", zonesPage, 0.0f, 12.0f,
                                 0.0f, "st"));
    p.push_back(params::floating(slotId(slot, "smp_zone_xfade_vel"), label + " Zone Velocity Crossfade", zonesPage, 0.0f,
                                 0.5f, 0.0f));
    p.push_back(
        params::choice(slotId(slot, "smp_zone_rr"), label + " Zone Round Robin", zonesPage, {"Off", "Cycle", "Random"}, 0));
}
} // namespace

ModuleDescriptor sampleOscillator(const VoiceGraphShape& shape)
{
    ModuleDescriptor d;
    d.id = "osc.sample";
    d.name = "Sample / Granular";
    d.kind = ModuleKind::Oscillator;
    d.category = "Sampler";
    d.cpu = CpuClass::Heavy;
    d.version = 2; // 2: zone parameters (smp_zone_*); patches without them load with zones off and play as before
    d.description =
        "Pitched sample playback with start, end and loop points, or a granular cloud of grains read "
        "from the same audio. Unison stacks up to eight detuned voices; slicing plays one slice of the "
        "sample per note (slice per key, or the slices spread over a key range). Zones play a set of samples by key and "
        "velocity range with crossfades and round robin.";
    for (int slot = 0; slot < shape.oscillators; ++slot)
        addSampleSlot(d, slot);
    d.createOscillator = [](int slot, const ParameterModel& model) { return std::make_unique<Instance>(slot, model); };
    d.perOscillator = true;
    return d;
}

} // namespace crucible::modules
