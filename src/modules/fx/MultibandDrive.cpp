// SPDX-License-Identifier: MIT
// fx.multiband_drive: three-band distortion. The input is split by the LR4 crossovers of include/crucible/fx/Crossover.h
// (minimum phase, flat sum), every band runs through its own oversampled saturation model of include/crucible/Saturation.h
// with its own drive, output, mix, solo and bypass, and the bands are summed again.
#include "FxModuleSupport.h"
#include "SaturatorParts.h"

#include <crucible/Saturation.h>
#include <crucible/fx/Crossover.h>
#include <crucible/fx/HalfBandChain.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/LatencyReporter.h>

#include <array>

namespace crucible::modules::fx
{
namespace
{
constexpr float kMaxDriveDb = 36.0f;
constexpr int kNumModels = 10;

// Model order of the "Model" choice, the same as fx.saturator's. The numbers are part of the patch format: append only.
struct ModelEntry
{
    const char* name;
    saturation::Model model;
};
constexpr ModelEntry kModels[] = {
    {"Soft Clip", saturation::Model::Soft},
    {"Hard Clip", saturation::Model::Hard},
    {"Op-Amp Rail", saturation::Model::Rail},
    {"Diode", saturation::Model::Diode},
    {"Wavefolder", saturation::Model::Fold},
    {"Tube Triode", saturation::Model::TriodeSE},
    {"Tube Push-Pull", saturation::Model::TriodePP},
    {"Tube Pentode", saturation::Model::Pentode},
    {"Tape", saturation::Model::Tape},
    {"Transformer", saturation::Model::Transformer},
};

// The models that run on antiderivative anti-aliasing (the hysteresis and the transformer do not).
bool usesAdaa(int modelIndex)
{
    const saturation::Model m = kModels[std::clamp(modelIndex, 0, kNumModels - 1)].model;
    return m != saturation::Model::Tape && m != saturation::Model::Transformer;
}

// One setting of the saturation library for a band: the drive in dB with the shorter ranges of the wavefolder (a third,
// where fx.saturator takes a half: four to eight times oversampling is all a band gets, and the folder's alias rises with
// its fold count) and the tape model (two thirds, as in fx.saturator), no bias, the character control fixed at its middle
// (tube sag 0.25, tape loop 0.5, flux corner 12 Hz). integrator: the tape loop's step (RK2 below 8x, Euler at 8x and above).
saturation::Settings libraryFor(int modelIndex, float driveDb, int integrator)
{
    using saturation::Model;
    saturation::Settings s;
    s.model = kModels[std::clamp(modelIndex, 0, kNumModels - 1)].model;
    s.driveDb =
        s.model == Model::Fold ? driveDb * (1.0f / 3.0f) : (s.model == Model::Tape ? driveDb * (2.0f / 3.0f) : driveDb);
    s.integrator = integrator;
    s.tapeFast = true;
    s.adaaOrder = 2;
    s.sag = s.model == Model::TriodePP ? 0.0f : 0.25f;
    s.hysteresis = 0.5f;
    s.cornerHz = 12.0f;
    return s;
}

// A control that reaches its target through two one-pole stages, per sample, and lands exactly on it: a stage that can no
// longer move (fx/Smoothing.h) or is within 1e-6 of what it follows (-120 dB, which also keeps a fade to zero out of the
// denormals) takes that value.
struct Glide
{
    static constexpr float kLanding = 1.0e-6f;
    float stage1 = 0.0f, stage2 = 0.0f, target = 0.0f;
    void reset(float v) { stage1 = stage2 = target = v; }
    bool settled() const { return stage1 == target && stage2 == target; }
    static float follow(float v, float goal, float coef)
    {
        const float next = crucible::fx::onePoleStepSnap(v, goal, coef);
        return std::abs(next - goal) < kLanding ? goal : next;
    }
    void fill(float* out, int n, float coef)
    {
        if (settled())
        {
            std::fill(out, out + n, target);
            return;
        }
        for (int i = 0; i < n; ++i)
        {
            stage1 = follow(stage1, target, coef);
            stage2 = follow(stage2, stage1, coef);
            out[i] = stage2;
        }
    }
};

// One band, both channels: half-band oversampling, the shaper, the flattening filter of the antiderivative curves, and
// padding that brings every model to the same whole-sample latency. A fresh state renders what a reset one does.
class BandPath
{
public:
    static constexpr int kChunk = 128;

    void prepare(double rate, int stages, double stopBandDb)
    {
        sr = rate;
        crucible::fx::HalfBandChain::Design designs[crucible::fx::HalfBandChain::kMaxStages];
        for (int i = 0; i < stages; ++i)
            designs[i] = crucible::fx::HalfBandChain::designFor(sr, i, stopBandDb);
        for (size_t c = 0; c < 2; ++c)
        {
            chain[c].prepare(stages, designs, kChunk);
            wet[c].assign((static_cast<size_t>(kChunk) << stages) + detail::Pad::kMax, 0.0f);
        }
        factor = 1 << stages;
        osRate = sr * static_cast<double>(factor);
        // The oversampling filters, plus for the antiderivative models one oversampled sample of the second-order curve
        // and three base-rate samples of the flattening filter; padded to a whole number of base-rate samples.
        const int firOs = chain[0].latencyFine();
        const int adaaOs = firOs + 1 + detail::Flatten::kTaps * factor;
        const int total = (adaaOs + factor - 1) / factor * factor;
        padAdaa = total - adaaOs;
        padPlain = total - firOs;
        latencyBase = total / factor;
        for (auto& f : flatten)
            f.design(sr, osRate);
        // Prepared once here (control thread); reset() on the audio thread only copies them back.
        for (auto& s : fresh)
        {
            s = saturation::Saturator{};
            s.prepare(osRate);
        }
        reset();
    }

    int latency() const { return latencyBase; }

    void reset()
    {
        for (auto& c : chain)
            c.reset();
        sat = fresh;
        for (auto& f : flatten)
            f.clear();
        for (auto& d : pad)
            d.clear();
        haveSettings = false;
    }

    // Settings the shaper already holds are not applied again (the call is two pow and tan per channel).
    void set(const saturation::Settings& s)
    {
        if (haveSettings && same(s, applied))
            return;
        for (auto& x : sat)
            x.set(s);
        applied = s;
        haveSettings = true;
    }

    // In place, n <= kChunk.
    void run(float* l, float* r, int n, bool adaa)
    {
        float* io[2] = {l, r};
        const int count = n * factor;
        float* head[2] = {wet[0].data() + detail::Pad::kMax, wet[1].data() + detail::Pad::kMax};
        for (size_t c = 0; c < 2; ++c)
            chain[c].up(io[c], n, head[c]);
        saturation::Saturator::processPair(sat[0], sat[1], head[0], head[1], count);
        const int padding = adaa ? padAdaa : padPlain;
        for (size_t c = 0; c < 2; ++c)
        {
            const float* delayed = padding > 0 ? pad[c].delayed(head[c], count, padding) : head[c];
            chain[c].down(delayed, n, io[c]);
            if (adaa)
                flatten[c].run(io[c], n);
        }
    }

private:
    static bool same(const saturation::Settings& a, const saturation::Settings& b)
    {
        return a.model == b.model && a.driveDb == b.driveDb && a.bias == b.bias && a.dcCornerHz == b.dcCornerHz &&
               a.sag == b.sag && a.cornerHz == b.cornerHz && a.hysteresis == b.hysteresis && a.adaa == b.adaa &&
               a.adaaOrder == b.adaaOrder && a.integrator == b.integrator && a.tapeFast == b.tapeFast;
    }

    saturation::Settings applied;
    bool haveSettings = false;
    double sr = 48000.0, osRate = 48000.0;
    int factor = 1, padAdaa = 0, padPlain = 0, latencyBase = 0;
    std::array<crucible::fx::HalfBandChain, 2> chain;
    std::array<std::vector<float>, 2> wet;
    std::array<saturation::Saturator, 2> sat, fresh;
    std::array<detail::Flatten, 2> flatten;
    std::array<detail::Pad, 2> pad;
};

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_mbdrive_mix")), xlow(localIndex(d, "fx_mbdrive_xlow")),
          xhigh(localIndex(d, "fx_mbdrive_xhigh")), output(localIndex(d, "fx_mbdrive_output"))
    {
        static const char* const names[3] = {"low", "mid", "high"};
        for (size_t b = 0; b < kBands; ++b)
        {
            const std::string id = std::string("fx_mbdrive_") + names[b] + "_";
            ctl[b] = {localIndex(d, id + "model"), localIndex(d, id + "drive"), localIndex(d, id + "output"),
                      localIndex(d, id + "mix"),   localIndex(d, id + "solo"),  localIndex(d, id + "bypass")};
        }
    }

    void prepare(const ModuleContext& c) override
    {
        sr = c.sampleRate;
        // Oversampling stages per band and quality: 2x in Draft, 4x in Standard, 8x in High; the high band, where the
        // harmonics of a driven tone fold back first, one stage more in Standard and High.
        const int base = c.quality == Quality::Draft ? 1 : (c.quality == Quality::Standard ? 2 : 3);
        const double stopBand = c.quality == Quality::Draft ? 60.0 : (c.quality == Quality::Standard ? 100.0 : 110.0);
        integrator = base >= 3 ? 1 : 2;
        latency = 0;
        for (size_t b = 0; b < kBands; ++b)
        {
            const int stages = base + (b == kBands - 1 && c.quality != Quality::Draft ? 1 : 0);
            path[b].prepare(sr, stages, stopBand);
            latency = std::max(latency, path[b].latency());
        }
        coef = 1.0f - static_cast<float>(std::exp(-1.0 / (kGlideSeconds * sr)));
        crossover.prepare(sr);
        size_t size = 64;
        while (size < static_cast<size_t>(latency + kChunk + kWarmup + 8))
            size *= 2;
        ringMask = size - 1;
        for (auto& ch : inRing)
            ch.assign(size, 0.0f);
        for (auto& b : bandRing)
            for (auto& ch : b)
                ch.assign(size, 0.0f);
        for (auto& b : wetRing)
            for (auto& ch : b)
                ch.assign(size, 0.0f);
        reset();
    }

    void reset() override
    {
        for (auto& p : path)
            p.reset();
        crossover.reset();
        for (auto& ch : inRing)
            std::fill(ch.begin(), ch.end(), 0.0f);
        for (auto& b : bandRing)
            for (auto& ch : b)
                std::fill(ch.begin(), ch.end(), 0.0f);
        for (auto& b : wetRing)
            for (auto& ch : b)
                std::fill(ch.begin(), ch.end(), 0.0f);
        ringPos = 0;
        gridPos = 0;
        primed = false;
        cold.fill(true);
    }

    void deactivate() override
    {
        if (primed)
            reset();
    }

    int latencySamples() const override { return latency; }

    // The work is cut at multiples of kChunk samples of the instance's own time, whatever the host block size: the
    // parameters are read on that grid and the crossover frequencies move over the chunk, so the output does not depend
    // on how the host splits its blocks.
    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        for (int pos = 0; pos < n;)
        {
            const int m = std::min(kChunk - gridPos, n - pos);
            if (gridPos == 0)
                beginGrid(in);
            fragment(l + pos, r + pos, m);
            gridPos = (gridPos + m) % kChunk;
            pos += m;
        }
    }

private:
    static constexpr size_t kBands = 3;
    static constexpr int kChunk = BandPath::kChunk;
    static constexpr int kWarmup = 64;
    static constexpr double kGlideSeconds = 0.010; // each of the two stages of every gain, mix and share

    struct BandIds
    {
        int model, drive, output, mix, solo, bypass;
    };

    // Start of a grid chunk: parameters, model change, crossover, targets of the glides.
    void beginGrid(const EffectInputs& in)
    {
        bool anySolo = false;
        for (const auto& c : ctl)
            anySolo = anySolo || in.on(c.solo);
        for (size_t b = 0; b < kBands; ++b)
        {
            const BandIds& c = ctl[b];
            const int wanted = std::clamp(in.choice(c.model), 0, kNumModels - 1);
            const float driveDb = std::clamp(in.value(c.drive), 0.0f, kMaxDriveDb);
            const bool bypass = in.on(c.bypass);
            if (!primed)
                active[b] = wanted;
            // Drive at its minimum is a bypass of the shaper: the wet share rises over the first half decibel, so the
            // output nulls exactly. A model change first takes the wet share to zero, then the new model takes over
            // (a changed model resets the shaper's state) and the share rises again.
            float shareT = bypass ? 0.0f : in.value(c.mix) * std::min(1.0f, driveDb * 2.0f);
            if (wanted != active[b])
            {
                // The wet share is below -80 dB: the switch (the shaper restarts) cannot be heard.
                if (share[b].target == 0.0f && share[b].stage2 < 1.0e-4f)
                    active[b] = wanted;
                else
                    shareT = 0.0f;
            }
            const float level = bypass ? 1.0f : std::pow(10.0f, in.value(c.output) / 20.0f);
            const float gainT = anySolo && !in.on(c.solo) ? 0.0f : level;
            if (!primed)
            {
                share[b].reset(shareT);
                gain[b].reset(gainT);
            }
            share[b].target = shareT;
            gain[b].target = gainT;
            settings[b] = libraryFor(active[b], driveDb, integrator);
            // Decided once per chunk: a band whose wet share and gain are both settled at zero is not run at all.
            const bool wetOff = share[b].settled() && share[b].target == 0.0f;
            const bool muted = gain[b].settled() && gain[b].target == 0.0f;
            run[b] = !wetOff && !muted;
        }
        const float mixT = in.value(mix);
        const float outT = std::pow(10.0f, in.value(output) / 20.0f);
        if (!primed)
        {
            mixGlide.reset(mixT);
            outGlide.reset(outT);
            freqLo = freqLoMid = lastLo = std::clamp(in.value(xlow), 20.0f, 0.4f * static_cast<float>(sr));
            freqHi = freqHiMid = lastHi = std::clamp(in.value(xhigh), freqLo * 1.05f, 0.45f * static_cast<float>(sr));
            const double f[2] = {freqLo, freqHi};
            crossover.set(3, f, crucible::fx::CrossoverSlope::Db24);
            primed = true;
        }
        mixGlide.target = mixT;
        outGlide.target = outT;
        // The crossover frequencies approach their targets through two one-poles per chunk (about 25 ms each); the filters
        // move over the chunk.
        const float fcoef = 1.0f - static_cast<float>(std::exp(-static_cast<double>(kChunk) / (0.025 * sr)));
        const float loT = std::clamp(in.value(xlow), 20.0f, 0.4f * static_cast<float>(sr));
        const float hiT = std::clamp(in.value(xhigh), loT * 1.05f, 0.45f * static_cast<float>(sr));
        freqLoMid = crucible::fx::onePoleStepSnap(freqLoMid, loT, fcoef);
        freqHiMid = crucible::fx::onePoleStepSnap(freqHiMid, hiT, fcoef);
        freqLo = crucible::fx::onePoleStepSnap(freqLo, freqLoMid, fcoef);
        freqHi = crucible::fx::onePoleStepSnap(freqHi, freqHiMid, fcoef);
        const float hiFix = std::max(freqHi, freqLo * 1.05f);
        if (freqLo != lastLo || hiFix != lastHi)
        {
            const double f[2] = {freqLo, hiFix};
            crossover.set(3, f, crucible::fx::CrossoverSlope::Db24, kChunk);
            lastLo = freqLo;
            lastHi = hiFix;
        }
    }

    // m samples of the grid chunk, starting at gridPos.
    void fragment(float* l, float* r, int m)
    {
        float* io[2] = {l, r};
        const size_t lat = static_cast<size_t>(latency);
        float* bl[kBands] = {band[0][0].data(), band[1][0].data(), band[2][0].data()};
        float* br[kBands] = {band[0][1].data(), band[1][1].data(), band[2][1].data()};
        crossover.process(l, r, m, bl, br);
        for (size_t b = 0; b < kBands; ++b)
        {
            share[b].fill(sharePath[b].data(), m, coef);
            gain[b].fill(gainPath[b].data(), m, coef);
        }
        mixGlide.fill(mixPath.data(), m, coef);
        outGlide.fill(outPath.data(), m, coef);
        for (size_t c = 0; c < 2; ++c)
        {
            std::fill(sum[c].begin(), sum[c].begin() + m, 0.0f);
            std::fill(dryAll[c].begin(), dryAll[c].begin() + m, 0.0f);
        }
        for (size_t b = 0; b < kBands; ++b)
        {
            for (size_t c = 0; c < 2; ++c)
                ringWrite(bandRing[b][c], ringPos, band[b][c].data(), m);
            const size_t extra = lat - static_cast<size_t>(path[b].latency());
            if (run[b])
            {
                if (cold[b])
                    warmUp(b);
                path[b].set(settings[b]);
                for (size_t c = 0; c < 2; ++c)
                    std::copy(band[b][c].begin(), band[b][c].begin() + m, wetBuf[c].begin());
                path[b].run(wetBuf[0].data(), wetBuf[1].data(), m, usesAdaa(active[b]));
                // A band with fewer oversampling stages is delayed to the latency of the slowest.
                if (extra > 0)
                    for (size_t c = 0; c < 2; ++c)
                    {
                        ringWrite(wetRing[b][c], ringPos, wetBuf[c].data(), m);
                        ringRead(wetRing[b][c], ringPos - extra, wetBuf[c].data(), m);
                    }
            }
            for (size_t c = 0; c < 2; ++c)
            {
                ringRead(bandRing[b][c], ringPos - lat, dryBuf.data(), m);
                const float* dry = dryBuf.data();
                const float* wet = run[b] ? wetBuf[c].data() : dry;
                const float* shr = sharePath[b].data();
                const float* gn = gainPath[b].data();
                float* da = dryAll[c].data();
                float* sm = sum[c].data();
                for (int i = 0; i < m; ++i)
                {
                    da[i] += dry[i];
                    sm[i] += (dry[i] + (wet[i] - dry[i]) * shr[i]) * gn[i];
                }
            }
            cold[b] = !run[b];
        }
        for (size_t c = 0; c < 2; ++c)
            for (int i = 0; i < m; ++i)
            {
                const size_t k = static_cast<size_t>(i);
                // The dry share is the undriven band sum: it passes the crossover's all-pass like the wet path, so the
                // mix cannot notch at the crossovers.
                io[c][i] = (sum[c][k] + (dryAll[c][k] - sum[c][k]) * (1.0f - mixPath[k])) * outPath[k];
            }
        ringPos += static_cast<size_t>(m);
    }

    // m samples into / out of a ring, starting at position pos (wrapping at the ring's size).
    void ringWrite(std::vector<float>& ring, size_t pos, const float* src, int m) const
    {
        const size_t start = pos & ringMask, len = static_cast<size_t>(m), first = std::min(len, ring.size() - start);
        std::copy(src, src + first, ring.begin() + static_cast<std::ptrdiff_t>(start));
        std::copy(src + first, src + len, ring.begin());
    }
    void ringRead(const std::vector<float>& ring, size_t pos, float* dst, int m) const
    {
        const size_t start = pos & ringMask, len = static_cast<size_t>(m), first = std::min(len, ring.size() - start);
        std::copy(ring.begin() + static_cast<std::ptrdiff_t>(start), ring.begin() + static_cast<std::ptrdiff_t>(start + first),
                  dst);
        std::copy(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(len - first), dst + first);
    }

    // A band waking up runs its path over the last stretch of the band first (output discarded), so that the filters
    // and the shaper's memory are warm when the wet share starts to rise.
    void warmUp(size_t b)
    {
        path[b].reset();
        path[b].set(settings[b]);
        const size_t len = std::min<size_t>(static_cast<size_t>(kWarmup) + static_cast<size_t>(latency), kChunk);
        std::array<float, kChunk> w[2];
        for (size_t c = 0; c < 2; ++c)
            for (size_t i = 0; i < len; ++i)
                w[c][i] = bandRing[b][c][(ringPos - len + i) & ringMask];
        path[b].run(w[0].data(), w[1].data(), static_cast<int>(len), usesAdaa(active[b]));
        for (size_t c = 0; c < 2; ++c)
            std::fill(wetRing[b][c].begin(), wetRing[b][c].end(), 0.0f);
    }

    const int mix, xlow, xhigh, output;
    std::array<BandIds, kBands> ctl{};
    std::array<BandPath, kBands> path;
    crucible::fx::Crossover crossover;
    std::array<Glide, kBands> share, gain;
    Glide mixGlide, outGlide;
    std::array<std::array<std::array<float, kChunk>, 2>, kBands> band{};
    std::array<std::array<float, kChunk>, kBands> sharePath{}, gainPath{};
    std::array<std::array<float, kChunk>, 2> sum{}, wetBuf{}, dryAll{};
    std::array<float, kChunk> dryBuf{};
    std::array<float, kChunk> mixPath{}, outPath{};
    std::array<std::vector<float>, 2> inRing;
    std::array<std::array<std::vector<float>, 2>, kBands> bandRing, wetRing;
    std::array<saturation::Settings, kBands> settings;
    std::array<int, kBands> active{};
    std::array<bool, kBands> cold{}, run{};
    size_t ringMask = 63, ringPos = 0;
    double sr = 48000.0;
    int latency = 0, integrator = 2, gridPos = 0;
    float coef = 0.0f;
    float freqLo = 0.0f, freqHi = 0.0f, freqLoMid = 0.0f, freqHiMid = 0.0f, lastLo = 0.0f, lastHi = 0.0f;
    bool primed = false;
};
} // namespace

ModuleDescriptor multibandDriveEffect()
{
    auto d = effectDescriptor(
        "fx.multiband_drive", "Multiband Drive", "Distortion", CpuClass::Heavy,
        "Three-band distortion. Linkwitz-Riley crossovers (24 dB per octave, minimum phase, band sum flat) split the "
        "input into low, mid and high bands; each band runs through its own anti-aliased saturation model "
        "(the ten of fx.saturator, second-order antiderivative curves inside a linear-phase half-band "
        "oversampler: 2x in Draft, 4x in Standard and 8x in High, the high band one stage higher) with "
        "its own drive (0 to 36 dB), output, mix, solo and bypass. The bands are summed, mixed against "
        "the delayed input and trimmed by the output. A band at 0 dB drive or bypassed is not shaped at "
        "all: with every band so, the sum is the crossover's all-pass sum, flat within 0.1 dB from 20 Hz "
        "to 20 kHz and phase-coherent with it. Drives, outputs, mixes, solo and bypass glide (two 10 ms "
        "one-poles) and the crossover frequencies move over about 50 ms; a model change takes the band's "
        "wet share to zero, switches and brings it back. The wavefolder's drive range is a third of the "
        "control, the tape model's two thirds. Latency is the oversampling filters, reported; the "
        "bands and the dry path share it.");
    auto& p = d.params;
    addOn(p, "mbdrive", "Multiband Drive", "fx_mbdrive", 1.0f);
    addHz(p, "mbdrive_xlow", "Multiband Drive Low X-Over", "fx_mbdrive", 40.0f, 1000.0f, 200.0f, 0.4f);
    addHz(p, "mbdrive_xhigh", "Multiband Drive High X-Over", "fx_mbdrive", 800.0f, 12000.0f, 3000.0f, 0.4f);
    addF(p, "mbdrive_output", "Multiband Drive Output", "fx_mbdrive", -24.0f, 3.0f, 0.0f, "dB");
    p.push_back(params::integer("fx_pos_mbdrive", "Multiband Drive Slot", "fx_mbdrive", 1, kNumModuleFx, kNumModuleFx));
    std::vector<std::string> names;
    for (const auto& m : kModels)
        names.emplace_back(m.name);
    const char* const bandIds[3] = {"low", "mid", "high"};
    const char* const bandNames[3] = {"Low", "Mid", "High"};
    const float bandDrive[3] = {6.0f, 12.0f, 12.0f};
    for (int b = 0; b < 3; ++b)
    {
        const std::string id = std::string("mbdrive_") + bandIds[b], name = std::string("Multiband Drive ") + bandNames[b],
                          page = std::string("fx_mbdrive_") + bandIds[b];
        p.push_back(params::choice("fx_" + id + "_model", name + " Model", page, names, 0));
        addF(p, id + "_drive", name + " Drive", page, 0.0f, kMaxDriveDb, bandDrive[b], "dB");
        addF(p, id + "_output", name + " Output", page, -24.0f, 6.0f, 0.0f, "dB");
        addF(p, id + "_mix", name + " Mix", page, 0.0f, 1.0f, 1.0f);
        p.push_back(params::toggle("fx_" + id + "_solo", name + " Solo", page, false));
        p.push_back(params::toggle("fx_" + id + "_bypass", name + " Bypass", page, false));
        d.pages.push_back(
            {page, name + " Band", "fx", name + " Band", "Model, drive, output, mix, solo and bypass of the band."});
    }
    d.onParam = "fx_mbdrive_on";
    d.positionParam = "fx_pos_mbdrive";
    d.capabilities = {"latency"};
    d.pages.insert(d.pages.begin(), {"fx_mbdrive", "Multiband Drive", "fx", "Multiband Drive",
                                     "Crossover frequencies, output, mix and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.multiband_drive")); };
    return d;
}

} // namespace crucible::modules::fx
