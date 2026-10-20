// SPDX-License-Identifier: MIT
// fx.exciter: a harmonic exciter (High mode) and a bass enhancer (Low mode).
//
// The signal path is  x -> band -> drive and saturator -> high-pass -> amount -> added to x.
//   High mode  band = everything above the frequency (Butterworth high-pass, 12 dB per octave)
//   Low mode   band = everything below the bass frequency (Butterworth low-pass, 12 dB per octave)
// The band is driven into the rail model of include/crucible/Saturation.h (a cubic with a smooth knee: first-order
// antiderivative anti-aliased), oversampled 1x for Draft quality and 2x (with the longer filters of High) otherwise. Its
// output goes through a 36 dB per octave Butterworth high-pass at the same frequency, so the added energy sits above the
// original band: in High mode the harmonics of the band, in Low mode the harmonics of the bass (the low tone is heard
// through its overtones). The added signal is scaled by the amount and summed with the input delayed by the oversampling
// filters' latency (reported); with the amount at zero the output is the delayed input exactly.
//
// The law of the generated harmonics: for a driven band of peak a (the band's amplitude times the drive gain) and the
// asymmetry's offset o, the saturator output is f(a sin t + o) with f(x) = x - x^3 / 6.75 for |x| < 1.5 (the knee) and the
// rail 1 beyond. Inside the knee that is a cubic polynomial, so the generated harmonics are exactly the third (and, with an
// offset, the second) and nothing above, and they cannot alias at all; the knee only comes into play when a * g + |o|
// exceeds 1.5. The added signal is that output times the high-pass response at each harmonic, so the first harmonic adds to
// the input's own tone. Below the exciter's frequency the two filters leave the added signal at least 50 dB down one octave
// below.
//
// Mode changes fade the added signal out and in (two smoothing poles of 5 ms each). Channels are processed independently.
#include "FxModuleSupport.h"

#include <crucible/Saturation.h>
#include <crucible/fx/AnalogMatchedFilters.h>
#include <crucible/fx/OversampledBlock.h>
#include <crucible/module/LatencyReporter.h>

#include <array>
#include <atomic>
#include <vector>

namespace crucible::modules::fx
{
namespace
{
constexpr int kChunk = 128;
constexpr double kMaxAsymmetryBias =
    0.2; // saturator bias at 100 % asymmetry (the saturator offsets its driven input by half of it: 0.1)
constexpr int kPreOrder = 2, kPostOrder = 6;
constexpr float kMaxStep = 1.12f; // largest change of the corner frequency per kChunk samples
constexpr double kHoldMs = 30.0;  // the added signal stays off this long after the filters are cleared
constexpr double kGainMs = 5.0;   // time constant of each of the two smoothing poles of the added signal's gain

class Instance final : public EffectModule, public LatencyReporter
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mode(localIndex(d, "fx_exciter_mode")), frequency(localIndex(d, "fx_exciter_frequency")),
          bassFrequency(localIndex(d, "fx_exciter_bass_frequency")), drive(localIndex(d, "fx_exciter_drive")),
          asymmetry(localIndex(d, "fx_exciter_asymmetry")), amount(localIndex(d, "fx_exciter_amount")),
          listen(localIndex(d, "fx_exciter_listen"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        gainAlpha = 1.0f - static_cast<float>(std::exp(-1.0 / (kGainMs * 0.001 * rate)));
        // The rail curve is a cubic inside its knee, so 2x with the filters of the quality setting is enough for Standard
        // and High; Draft runs at the base rate.
        const int stages = c.quality == Quality::Draft ? 0 : 1;
        block.prepare(2, stages, c.oversampling, c.quality, kChunk, crucible::fx::DryPath::Delay);
        lag = block.latencySamples();
        ring = 1;
        while (ring < lag + kChunk + 1)
            ring <<= 1;
        for (auto& r : dry)
            r.assign(static_cast<size_t>(ring), 0.0f);
        reset();
    }
    void reset() override
    {
        block.reset();
        // Back to the state of a new instance, including the glide of the drive from unity.
        for (auto& s : saturator)
        {
            s.set(crucible::saturation::Settings{});
            s.prepare(rate * block.factor());
        }
        pre.reset();
        post.reset();
        for (auto& r : dry)
            std::fill(r.begin(), r.end(), 0.0f);
        writePos = 0;
        gainIn = gainOut = 0.0f;
        gridPos = 0;
        holdLeft = static_cast<int>(kHoldMs * 0.001 * rate);
        highNow = true;
        lastFrequency = -1.0f;
        frequencyNow = 1000.0f;
        designed = false;
        latency.store(lag, std::memory_order_relaxed);
        latencyChanged.store(false, std::memory_order_relaxed);
    }
    void deactivate() override
    {
        if (dirty)
        {
            reset();
            dirty = false;
        }
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        dirty = true;
        const bool wantHigh = in.choice(mode) == 0;
        const float hz = wantHigh ? in.value(frequency) : in.value(bassFrequency);
        const float amountValue = in.value(amount) * 0.01f;
        const bool listening = in.on(listen);

        crucible::saturation::Settings s;
        s.model = crucible::saturation::Model::Rail;
        s.driveDb = in.value(drive);
        s.bias = static_cast<float>(kMaxAsymmetryBias * 0.01 * in.value(asymmetry));
        s.dcCornerHz = 0.0f; // the high-pass behind the saturator takes the offset out
        s.adaa = true;
        for (auto& sat : saturator)
            sat.set(s);

        // The work is cut at multiples of kChunk samples of the instance's own time, whatever the host block size: the
        // corner design, its ramp and the mode change happen on that grid, and the gains below move per sample, so the
        // output does not depend on how the host splits its blocks.
        for (int pos = 0; pos < n;)
        {
            const int m = std::min(kChunk - gridPos, n - pos);
            float* chunk[2] = {l + pos, r + pos};

            if (gridPos == 0)
            {
                // A mode change waits until the added signal has faded out.
                if (wantHigh != highNow && gainOut == 0.0f)
                {
                    highNow = wantHigh;
                    for (auto& sat : saturator)
                        sat.reset();
                    block.reset();
                    pre.reset();
                    post.reset();
                    designed = false;
                    holdLeft = static_cast<int>(kHoldMs * 0.001 * rate);
                }

                // The corner moves toward the set frequency by at most kMaxStep per kChunk samples, so that a jump becomes a
                // short sweep: the coefficients of a steep filter interpolated across a large jump would pass through badly
                // damped filters.
                if (!designed)
                    frequencyNow = hz;
                else
                    frequencyNow *= std::clamp(hz / frequencyNow, 1.0f / kMaxStep, kMaxStep);
                if (!designed || frequencyNow != lastFrequency)
                {
                    const double f = std::clamp(static_cast<double>(frequencyNow), 20.0, 0.45 * rate);
                    const int ramp = designed ? kChunk : 0;
                    design(pre, highNow ? crucible::fx::SectionKind::HighPass : crucible::fx::SectionKind::LowPass,
                           kPreOrder, f, ramp);
                    design(post, crucible::fx::SectionKind::HighPass, kPostOrder, f, ramp);
                    lastFrequency = frequencyNow;
                    designed = true;
                }
            }
            const float target = wantHigh == highNow ? amountValue : 0.0f;

            // The dry path: the input delayed by the latency of the oversampling filters.
            for (int c = 0; c < 2; ++c)
            {
                auto& ringBuffer = dry[static_cast<size_t>(c)];
                const size_t mask = static_cast<size_t>(ring - 1);
                for (int i = 0; i < m; ++i)
                    ringBuffer[(writePos + static_cast<size_t>(i)) & mask] = chunk[c][i];
                for (int i = 0; i < m; ++i)
                    delayed[static_cast<size_t>(c)][static_cast<size_t>(i)] =
                        ringBuffer[(writePos + static_cast<size_t>(i) + static_cast<size_t>(ring - lag)) & mask];
            }
            writePos += static_cast<size_t>(m);

            // Band, saturator, high-pass.
            for (int i = 0; i < m; ++i)
            {
                work[0][static_cast<size_t>(i)] = chunk[0][i];
                work[1][static_cast<size_t>(i)] = chunk[1][i];
            }
            pre.process(work[0].data(), work[1].data(), m);
            for (int c = 0; c < 2; ++c)
                for (int i = 0; i < m; ++i)
                    band[static_cast<size_t>(c)][static_cast<size_t>(i)] =
                        static_cast<float>(work[static_cast<size_t>(c)][static_cast<size_t>(i)]);
            float* wet[2] = {band[0].data(), band[1].data()};
            block.process(wet, m, 1.0f,
                          [this](float* const* x, int channels, int count)
                          {
                              for (int c = 0; c < channels; ++c)
                                  saturator[static_cast<size_t>(c)].process(x[c], x[c], count);
                          });
            for (int c = 0; c < 2; ++c)
                for (int i = 0; i < m; ++i)
                    work[static_cast<size_t>(c)][static_cast<size_t>(i)] = wet[c][i];
            post.process(work[0].data(), work[1].data(), m);

            // Two cascaded one-poles (about 5 ms each) move the gain per sample: smooth in value and slope, and the same at
            // any block size.
            float gains[kChunk];
            for (int i = 0; i < m; ++i)
            {
                // After a mode change the gain stays at zero while the freshly cleared filters and oversampler fill with the
                // signal.
                float goal = target;
                if (holdLeft > 0)
                {
                    --holdLeft;
                    goal = 0.0f;
                }
                gainIn += (goal - gainIn) * gainAlpha;
                gainOut += (gainIn - gainOut) * gainAlpha;
                if (std::abs(goal - gainOut) < 1.0e-7f && std::abs(goal - gainIn) < 1.0e-7f)
                    gainIn = gainOut = goal;
                gains[i] = gainOut;
            }
            for (int c = 0; c < 2; ++c)
            {
                const double* h = work[static_cast<size_t>(c)].data();
                const float* d = delayed[static_cast<size_t>(c)].data();
                for (int i = 0; i < m; ++i)
                {
                    const float added = gains[i] * static_cast<float>(h[i]);
                    chunk[c][i] = listening ? added : d[i] + added;
                }
            }
            gridPos = gridPos + m == kChunk ? 0 : gridPos + m;
            pos += m;
        }
    }

    int latencySamples() const override { return latency.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

private:
    const int mode, frequency, bassFrequency, drive, asymmetry, amount, listen;
    double rate = 48000.0;
    crucible::fx::OversampledBlock block;
    std::array<crucible::saturation::Saturator, 2> saturator;
    // A cascade of Butterworth sections of one kind (the building block of the FX5 filters, ramped to new coefficients).
    struct Cascade
    {
        crucible::fx::BiquadChain chain;
        void reset() { chain.reset(); }
        void process(double* a, double* b, int n) { chain.process(a, b, n); }
    };
    void design(Cascade& c, crucible::fx::SectionKind kind, int order, double hz, int ramp)
    {
        crucible::fx::PolePair poles[8];
        const int count = crucible::fx::butterworthPoles(order, poles);
        crucible::fx::Biquad sections[8];
        for (int i = 0; i < count; ++i)
            sections[i] = crucible::fx::bilinearSection(
                crucible::fx::analogPrototype(kind, 0.0, poles[i].q, poles[i].real ? 1 : 2), hz, rate);
        c.chain.set(sections, count, ramp);
    }
    Cascade pre, post;
    std::array<std::vector<float>, 2> dry;
    std::array<std::array<float, kChunk>, 2> band{};
    std::array<std::array<double, kChunk>, 2> work{};
    std::array<std::array<float, kChunk>, 2> delayed{};
    size_t writePos = 0;
    int ring = 256, lag = 0, gridPos = 0, holdLeft = 0;
    float gainIn = 0.0f, gainOut = 0.0f, gainAlpha = 0.004f, lastFrequency = -1.0f, frequencyNow = 1000.0f;
    bool highNow = true, designed = false, dirty = false;
    std::atomic<int> latency{0};
    std::atomic<bool> latencyChanged{false};
};
} // namespace

ModuleDescriptor exciterEffect()
{
    auto d = effectDescriptor(
        "fx.exciter", "Exciter", "Distortion", CpuClass::Light,
        "Harmonic exciter and bass enhancer. High mode takes the band above the frequency (1 to 10 kHz), Low "
        "mode the band below the bass frequency (40 to 250 Hz); the band is driven into an anti-aliased "
        "soft saturator (oversampled by quality), high-passed again at the same frequency so that the added "
        "energy sits above the original band, scaled by the amount and added to the input. An asymmetry adds "
        "even harmonics to the odd ones of the symmetric curve; listen outputs only the added signal. Below the "
        "frequency the output changes by at most 0.5 dB. Latency is the oversampling filters, reported; at "
        "amount 0 the output is the delayed input exactly.");
    auto& p = d.params;
    p.push_back(params::toggle("fx_exciter_on", "Exciter On", "fx_exciter", false));
    p.push_back(params::choice("fx_exciter_mode", "Exciter Mode", "fx_exciter", {"High", "Low"}, 0));
    addHz(p, "exciter_frequency", "Exciter Frequency", "fx_exciter", 1000.0f, 10000.0f, 3500.0f, 0.5f);
    addHz(p, "exciter_bass_frequency", "Exciter Bass Frequency", "fx_exciter", 40.0f, 250.0f, 100.0f, 0.5f);
    addF(p, "exciter_drive", "Exciter Drive", "fx_exciter", 0.0f, 9.0f, 6.0f, "dB");
    addF(p, "exciter_asymmetry", "Exciter Asymmetry", "fx_exciter_2", 0.0f, 100.0f, 0.0f, "%");
    addF(p, "exciter_amount", "Exciter Amount", "fx_exciter_2", 0.0f, 100.0f, 50.0f, "%");
    p.push_back(params::toggle("fx_exciter_listen", "Exciter Listen", "fx_exciter_2", false));
    p.push_back(params::integer("fx_pos_exciter", "Exciter Slot", "fx_exciter_2", 1, kNumModuleFx, kNumModuleFx));
    d.onParam = "fx_exciter_on";
    d.positionParam = "fx_pos_exciter";
    d.capabilities = {"latency"};
    d.pages.push_back({"fx_exciter", "Exciter", "fx", "Exciter", "On, mode, the High and Low frequencies and the drive."});
    d.pages.push_back(
        {"fx_exciter_2", "Exciter 2", "fx", "Exciter Output", "Asymmetry, amount, listen and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.exciter")); };
    return d;
}

} // namespace crucible::modules::fx
