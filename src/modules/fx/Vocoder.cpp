// SPDX-License-Identifier: MIT
#include "FxModuleSupport.h"

#include <crucible/Simd.h>

#include <array>
#include <cstdint>

// Channel vocoder. The modulator (the engine's sidechain input) is split into 16 to 32 log-spaced bands by band-pass
// filters; each band's envelope drives the level of the same band of the carrier (the synth bus, an internal saw,
// noise, or saw plus noise), filtered by a second bank whose centre frequencies can be shifted (formant shift). The
// carrier's own band level is divided out (Flatten), so the result follows the modulator's spectrum whatever the
// carrier's tilt is. A high-passed copy of the modulator can be mixed in so consonants stay intelligible.

namespace crucible::modules::fx
{
namespace
{
constexpr int kMaxBands = 32;
constexpr int kBlock = 64;
constexpr float kPi = 3.14159265358979f;

enum Carrier
{
    CarrierSynth = 0,
    CarrierSaw,
    CarrierNoise,
    CarrierSawNoise
};

// Band-pass biquad with 0 dB gain at its centre (RBJ "constant peak gain"), transposed direct form II.
struct BandPass
{
    float b0 = 0.0f, a1 = 0.0f, a2 = 0.0f; // b1 = 0, b2 = -b0

    void design(double centreHz, double bandwidthOctaves, double rate)
    {
        const double w0 = 2.0 * 3.14159265358979 * centreHz / rate;
        const double s = std::sin(w0);
        const double alpha = s * std::sinh(0.34657359027997264 * bandwidthOctaves * w0 / s);
        const double a0 = 1.0 + alpha;
        b0 = static_cast<float>(alpha / a0);
        a1 = static_cast<float>(-2.0 * std::cos(w0) / a0);
        a2 = static_cast<float>((1.0 - alpha) / a0);
    }
};

struct BiquadState
{
    float z1 = 0.0f, z2 = 0.0f;

    CRUCIBLE_ALWAYS_INLINE float run(const BandPass& c, float x)
    {
        const float y = c.b0 * x + z1;
        z1 = z2 - c.a1 * y;
        z2 = -c.b0 * x - c.a2 * y;
        return y;
    }
};

// 4th-order Butterworth-style high-pass as two biquads (sibilance passthrough).
struct HighPass
{
    float b0 = 1.0f, b1 = -2.0f, b2 = 1.0f, a1 = 0.0f, a2 = 0.0f;

    void design(double hz, double q, double rate)
    {
        const double w0 = 2.0 * 3.14159265358979 * std::min(hz, 0.45 * rate) / rate;
        const double alpha = std::sin(w0) / (2.0 * q);
        const double c = std::cos(w0);
        const double a0 = 1.0 + alpha;
        b0 = static_cast<float>((1.0 + c) * 0.5 / a0);
        b1 = static_cast<float>(-(1.0 + c) / a0);
        b2 = b0;
        a1 = static_cast<float>(-2.0 * c / a0);
        a2 = static_cast<float>((1.0 - alpha) / a0);
    }
    CRUCIBLE_ALWAYS_INLINE float run(BiquadState& s, float x) const
    {
        const float y = b0 * x + s.z1;
        s.z1 = b1 * x - a1 * y + s.z2;
        s.z2 = b2 * x - a2 * y;
        return y;
    }
};

float polyBlep(float t, float dt)
{
    if (t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.0f;
    }
    if (t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

class Instance final : public EffectModule
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : on(localIndex(d, "fx_voc_on")), mix(localIndex(d, "fx_voc_mix")), bands(localIndex(d, "fx_voc_bands")),
          carrier(localIndex(d, "fx_voc_carrier")), pitch(localIndex(d, "fx_voc_pitch")),
          input(localIndex(d, "fx_voc_input")), formant(localIndex(d, "fx_voc_formant")), gain(localIndex(d, "fx_voc_gain")),
          attack(localIndex(d, "fx_voc_attack")), release(localIndex(d, "fx_voc_release")),
          flatten(localIndex(d, "fx_voc_flatten")), sibilance(localIndex(d, "fx_voc_sibilance")),
          sibFreq(localIndex(d, "fx_voc_sib_freq"))
    {
    }

    void prepare(const ModuleContext& c) override
    {
        rate = c.sampleRate;
        seed = c.seed;
        designed = false;
        reset();
    }

    void reset() override
    {
        for (auto& b : bank)
            b = Band{};
        sibState = {};
        sawPhase = {0.0f, 0.37f};
        rng.state = (seed * 2654435761U) | 1U;
        lastMix = lastGain = lastSib = -1.0f;
    }

    void deactivate() override { lastMix = lastGain = lastSib = -1.0f; }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        const int count = std::clamp(in.rounded(bands), 16, kMaxBands);
        const float shift = in.value(formant);
        const float sibHz = in.value(sibFreq);
        if (!designed || count != bandCount || shift != designedShift || sibHz != designedSib)
            design(count, shift, sibHz);

        const int source = in.rounded(carrier);
        const int channel = in.rounded(input);
        const float mixNow = in.value(mix);
        const float gainNow = std::pow(10.0f, in.value(gain) * 0.05f);
        const float sibNow = in.value(sibilance);
        const float flat = in.value(flatten);
        const float sawStep = 440.0f * std::exp2((in.value(pitch) - 69.0f) / 12.0f) / static_cast<float>(rate);
        const float attackCoef =
            1.0f - std::exp(-1.0f / (std::max(in.value(attack), 0.05f) * 0.001f * static_cast<float>(rate)));
        const float releaseCoef =
            1.0f - std::exp(-1.0f / (std::max(in.value(release), 0.5f) * 0.001f * static_cast<float>(rate)));
        if (lastMix < 0.0f)
        {
            lastMix = mixNow;
            lastGain = gainNow;
            lastSib = sibNow;
        }

        const float* auxL = in.auxLeft();
        const float* auxR = in.auxRight();
        for (int start = 0; start < n; start += kBlock)
        {
            const int m = std::min(kBlock, n - start);
            float* dl = l + start;
            float* dr = r + start;

            // Modulator and carrier for this block.
            float mod[kBlock], carL[kBlock], carR[kBlock], outL[kBlock] = {}, outR[kBlock] = {};
            for (int i = 0; i < m; ++i)
            {
                float x = 0.0f;
                if (auxL != nullptr)
                {
                    const float a = auxL[start + i], b = auxR[start + i];
                    x = channel == 0 ? a : (channel == 1 ? b : 0.5f * (a + b));
                }
                mod[i] = x;
            }
            for (int i = 0; i < m; ++i)
            {
                float cl = 0.0f, cr = 0.0f;
                if (source == CarrierSynth)
                {
                    cl = dl[i];
                    cr = dr[i];
                }
                else
                {
                    if (source == CarrierSaw || source == CarrierSawNoise)
                    {
                        // Two saws a few cents apart, one per channel.
                        for (int ch = 0; ch < 2; ++ch)
                        {
                            const float step = sawStep * (ch == 0 ? 0.9985f : 1.0015f);
                            float& ph = sawPhase[static_cast<size_t>(ch)];
                            const float v = 2.0f * ph - 1.0f - polyBlep(ph, step);
                            (ch == 0 ? cl : cr) += v * 0.5f;
                            ph += step;
                            if (ph >= 1.0f)
                                ph -= 1.0f;
                        }
                    }
                    if (source == CarrierNoise || source == CarrierSawNoise)
                    {
                        const float level = source == CarrierNoise ? 0.5f : 0.2f;
                        cl += rng.bipolar() * level;
                        cr += rng.bipolar() * level;
                    }
                }
                carL[i] = cl;
                carR[i] = cr;
            }

            // The bands. Constants: pi/2 turns a rectified, smoothed sine into its amplitude.
            for (int b = 0; b < bandCount; ++b)
            {
                Band& band = bank[static_cast<size_t>(b)];
                const BandPass& ana = analysis[static_cast<size_t>(b)];
                const BandPass& syn = synthesis[static_cast<size_t>(b)];
                float env = band.env, car = band.carEnv;
                for (int i = 0; i < m; ++i)
                {
                    const float a = band.ana[1].run(ana, band.ana[0].run(ana, mod[i]));
                    const float mag = std::abs(a) * (kPi * 0.5f);
                    env += (mag - env) * (mag > env ? attackCoef : releaseCoef);
                    const float sl = band.synL[1].run(syn, band.synL[0].run(syn, carL[i]));
                    const float sr = band.synR[1].run(syn, band.synR[0].run(syn, carR[i]));
                    const float cmag = 0.5f * (std::abs(sl) + std::abs(sr)) * (kPi * 0.5f);
                    car += (cmag - car) * (cmag > car ? attackCoef : releaseCoef);
                    const float g = env / ((1.0f - flat) * 0.1f + flat * car + 1.0e-4f);
                    outL[i] += sl * g;
                    outR[i] += sr * g;
                }
                band.env = env;
                band.carEnv = car;
            }

            // Sibilance passthrough, level and dry / wet (ramped over the block).
            for (int i = 0; i < m; ++i)
            {
                const float t = (static_cast<float>(i) + 1.0f) / static_cast<float>(m);
                const float mx = lastMix + (mixNow - lastMix) * t;
                const float gn = lastGain + (gainNow - lastGain) * t;
                const float sb = lastSib + (sibNow - lastSib) * t;
                const float hp = sibHp.run(sibState[1], sibHp.run(sibState[0], mod[i]));
                const float wetL = (outL[i] * kBandNorm + hp * sb) * gn;
                const float wetR = (outR[i] * kBandNorm + hp * sb) * gn;
                dl[i] += (wetL - dl[i]) * mx;
                dr[i] += (wetR - dr[i]) * mx;
            }
            lastMix = mixNow;
            lastGain = gainNow;
            lastSib = sibNow;
        }
    }

private:
    // Output level of a complete bank relative to the modulator (overlapping bands add).
    static constexpr float kBandNorm = 0.85f;

    struct Band
    {
        std::array<BiquadState, 2> ana{};
        std::array<BiquadState, 2> synL{};
        std::array<BiquadState, 2> synR{};
        float env = 0.0f;
        float carEnv = 0.0f;
    };

    void design(int count, float shiftSemitones, float sibHz)
    {
        bandCount = count;
        designedShift = shiftSemitones;
        designedSib = sibHz;
        designed = true;
        // Log-spaced centres from 90 Hz up to 8 kHz; the synthesis bank is moved by the formant shift.
        const double lo = 90.0;
        const double hi = std::min(8000.0, 0.42 * rate);
        const double ratio = count > 1 ? std::pow(hi / lo, 1.0 / (count - 1)) : 1.0;
        // Each band is two cascaded biquads, which narrows the response; widen each so neighbours cross near -3 dB.
        const double octaves = std::log2(ratio) * 1.55;
        const double factor = std::exp2(static_cast<double>(shiftSemitones) / 12.0);
        for (int b = 0; b < count; ++b)
        {
            const double fc = lo * std::pow(ratio, b);
            analysis[static_cast<size_t>(b)].design(fc, octaves, rate);
            synthesis[static_cast<size_t>(b)].design(std::clamp(fc * factor, 40.0, 0.45 * rate), octaves, rate);
        }
        sibHp.design(sibHz, 0.7071, rate);
        // Only the filter memories of changed bands matter; a redesign keeps the envelopes running.
    }

    const int on, mix, bands, carrier, pitch, input, formant, gain, attack, release, flatten, sibilance, sibFreq;
    double rate = 48000.0;
    std::uint32_t seed = 1;
    bool designed = false;
    int bandCount = 0;
    float designedShift = 0.0f, designedSib = 0.0f;
    std::array<Band, kMaxBands> bank{};
    std::array<BandPass, kMaxBands> analysis{}, synthesis{};
    HighPass sibHp;
    std::array<BiquadState, 2> sibState{};
    std::array<float, 2> sawPhase{0.0f, 0.37f};
    Rng rng;
    float lastMix = -1.0f, lastGain = 1.0f, lastSib = 0.0f;
};
} // namespace

ModuleDescriptor vocoderEffect()
{
    auto d = effectDescriptor(
        "fx.vocoder", "Vocoder", "Modulation", CpuClass::Medium,
        "Channel vocoder (16 to 32 bands): the spectrum of the sidechain input (the aux buffers of Engine::process / "
        "crucible_process_aux) shapes the synth bus, an internal saw, noise or saw plus noise. Formant shift moves the "
        "carrier bands, sibilance passes the modulator's highs. Without a sidechain input the wet signal is silent.");
    auto& p = d.params;
    addOn(p, "voc", "Vocoder", "fx_voc", 1.0f);
    p.push_back(params::integer("fx_voc_bands", "Vocoder Bands", "fx_voc", 16, 32, 24));
    p.push_back(params::choice("fx_voc_carrier", "Vocoder Carrier", "fx_voc", {"Synth", "Saw", "Noise", "Saw + Noise"}, 0));
    addF(p, "voc_pitch", "Vocoder Saw Pitch", "fx_voc", 24.0f, 96.0f, 48.0f, "st");
    p.push_back(params::choice("fx_voc_input", "Vocoder Input", "fx_voc", {"Left", "Right", "Sum"}, 2));
    addF(p, "voc_formant", "Vocoder Formant Shift", "fx_voc_2", -12.0f, 12.0f, 0.0f, "st");
    addF(p, "voc_gain", "Vocoder Gain", "fx_voc_2", -24.0f, 24.0f, 0.0f, "dB");
    addF(p, "voc_flatten", "Vocoder Flatten", "fx_voc_2", 0.0f, 1.0f, 0.8f);
    addF(p, "voc_sibilance", "Vocoder Sibilance", "fx_voc_2", 0.0f, 1.0f, 0.3f);
    addHz(p, "voc_sib_freq", "Vocoder Sibilance Freq", "fx_voc_2", 2000.0f, 12000.0f, 5500.0f, 0.5f);
    // The vocoder runs after the original twenty effects by default; its slot may go beyond theirs.
    p.push_back(params::integer("fx_pos_voc", "Vocoder Slot", "fx_voc_2", 1, kNumModuleFx, kNumFx + 1));
    addF(p, "voc_attack", "Vocoder Attack", "fx_voc_3", 0.1f, 100.0f, 4.0f, "ms", 0.4f);
    addF(p, "voc_release", "Vocoder Release", "fx_voc_3", 1.0f, 500.0f, 50.0f, "ms", 0.4f);
    d.onParam = "fx_voc_on";
    d.positionParam = "fx_pos_voc";
    d.pages.push_back({"fx_voc", "Vocoder", "fx", "Vocoder",
                       "LCD draws the band envelopes of the modulator; strips set band count and mix."});
    d.pages.push_back({"fx_voc_2", "Vocoder 2", "fx", "Vocoder Formant and Sibilance",
                       "LCD draws the band envelopes of the modulator; strips set formant shift and sibilance."});
    d.pages.push_back({"fx_voc_3", "Vocoder 3", "fx", "Vocoder Dynamics",
                       "LCD draws the band envelopes of the modulator; strips set attack and release."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.vocoder")); };
    return d;
}

} // namespace crucible::modules::fx
