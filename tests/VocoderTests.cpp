// SPDX-License-Identifier: MIT
// Vocoder module: intelligibility proxy (band envelope correlation between the modulator and the output),
// numerical safety, determinism, dry / wet, sibilance and the sidechain plumbing through the engine facade and the C API.

#include <crucible/Engine.h>
#include <crucible/crucible.h>
#include <crucible/module/ModuleRack.h>
#include "TestSupport.h"

#include <array>
#include <numbers>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr int kBlock = 32;

// Speech-like modulator: a 110 Hz glottal pulse train through three resonators whose centre frequencies glide
// between vowel targets every 220 ms, with short pauses.
std::vector<float> speechLike(int samples, int offset)
{
    static const double vowels[5][3] = {
        {730, 1090, 2440}, {270, 2290, 3010}, {300, 870, 2240}, {530, 1840, 2480}, {660, 1720, 2410}};
    std::vector<float> out(static_cast<size_t>(samples));
    double z1[3] = {}, z2[3] = {};
    double phase = 0.0;
    for (int i = 0; i < samples; ++i)
    {
        const double t = (static_cast<double>(i) + offset) / kRate;
        const int seg = static_cast<int>(t / 0.22);
        const double frac = t / 0.22 - seg;
        const double* a = vowels[seg % 5];
        const double* b = vowels[(seg + 1) % 5];
        phase += 110.0 / kRate;
        const double pulse = phase >= 1.0 ? (phase -= 1.0, 1.0) : 0.0;
        double y = 0.0;
        for (int f = 0; f < 3; ++f)
        {
            const double fc = a[f] + (b[f] - a[f]) * std::min(1.0, frac * 1.5);
            const double r = std::exp(-std::numbers::pi * (60.0 + 0.05 * fc) / kRate);
            const double c1 = 2.0 * r * std::cos(2.0 * std::numbers::pi * fc / kRate), c2 = -r * r;
            const double v = pulse * 0.2 + c1 * z1[f] + c2 * z2[f];
            z2[f] = z1[f];
            z1[f] = v;
            y += v * (f == 0 ? 1.0 : (f == 1 ? 0.6 : 0.3));
        }
        const bool pause = std::fmod(t, 0.44) > 0.40;
        out[static_cast<size_t>(i)] = static_cast<float>(pause ? y * 0.02 : y);
    }
    return out;
}

// Log band energies per 40 ms frame (20 ms hop): each of 12 half-octave bands between 300 Hz and 6 kHz sums the
// Hann-windowed DFT power at eight frequencies across the band (independent of the vocoder's own filters).
std::vector<std::vector<double>> bandTrajectories(const std::vector<float>& x, int skip)
{
    const int frame = 1920, hop = 960, bands = 12;
    std::vector<std::vector<double>> tr(static_cast<size_t>(bands));
    for (int start = skip; start + frame <= static_cast<int>(x.size()); start += hop)
        for (int b = 0; b < bands; ++b)
        {
            const double lo = 300.0 * std::pow(2.0, b * (std::log2(6000.0 / 300.0) - 0.5) / (bands - 1));
            double power = 0.0;
            for (int q = 0; q < 8; ++q)
            {
                const double fc = lo * std::pow(2.0, 0.5 * (q + 0.5) / 8.0);
                double re = 0.0, im = 0.0;
                for (int k = 0; k < frame; ++k)
                {
                    const double w = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * k / frame);
                    const double ph = 2.0 * std::numbers::pi * fc * k / kRate;
                    re += w * x[static_cast<size_t>(start + k)] * std::cos(ph);
                    im += w * x[static_cast<size_t>(start + k)] * std::sin(ph);
                }
                power += re * re + im * im;
            }
            tr[static_cast<size_t>(b)].push_back(10.0 * std::log10(power + 1.0e-9));
        }
    return tr;
}

double meanBandCorrelation(const std::vector<float>& a, const std::vector<float>& b, int skip)
{
    const auto ta = bandTrajectories(a, skip), tb = bandTrajectories(b, skip);
    double total = 0.0;
    for (size_t band = 0; band < ta.size(); ++band)
    {
        const auto& x = ta[band];
        const auto& y = tb[band];
        double mx = 0.0, my = 0.0;
        for (size_t i = 0; i < x.size(); ++i)
            mx += x[i], my += y[i];
        mx /= static_cast<double>(x.size());
        my /= static_cast<double>(y.size());
        double sxy = 0.0, sxx = 0.0, syy = 0.0;
        for (size_t i = 0; i < x.size(); ++i)
        {
            sxy += (x[i] - mx) * (y[i] - my);
            sxx += (x[i] - mx) * (x[i] - mx);
            syy += (y[i] - my) * (y[i] - my);
        }
        total += sxy / std::sqrt(std::max(sxx * syy, 1.0e-12));
    }
    return total / static_cast<double>(ta.size());
}

// A rack with only the vocoder switched on.
struct VocoderRig
{
    ModuleRack rack{ModuleRegistry::instance()};
    std::vector<float> values = ParameterMap::defaultValues();
    std::vector<float> legacy = std::vector<float>(static_cast<size_t>(kNumModDests), 0.0f);

    VocoderRig()
    {
        // Speech-like timing for the proxy measurements (the defaults are slower, for sustained pads).
        ModuleContext ctx;
        ctx.sampleRate = kRate;
        ctx.seed = 7;
        rack.prepare(ctx);
        for (int i = 0; i < rack.size(); ++i)
            values[static_cast<size_t>(ParameterMap::indexOf(rack.descriptor(i).onParam))] = 0.0f;
        set("fx_voc_on", 1.0f);
        set("fx_voc_attack", 2.0f);
        set("fx_voc_release", 12.0f);
    }
    void set(const std::string& id, float v) { setValue(values, id, v); }

    // Runs the carrier bus (in place) with the modulator as sidechain input; aux null means no input.
    void run(std::vector<float>& l, std::vector<float>& r, const std::vector<float>* aux,
             const std::vector<float>* auxRight = nullptr, int block = kBlock)
    {
        for (size_t pos = 0; pos < l.size(); pos += static_cast<size_t>(block))
        {
            const int n = static_cast<int>(std::min(static_cast<size_t>(block), l.size() - pos));
            rack.process(l.data() + pos, r.data() + pos, n, values.data(), legacy.data(), nullptr, 120.0,
                         aux != nullptr ? aux->data() + pos : nullptr,
                         auxRight != nullptr ? auxRight->data() + pos : nullptr);
        }
    }
};

double rms(const std::vector<float>& x, size_t from = 0)
{
    double s = 0.0;
    for (size_t i = from; i < x.size(); ++i)
        s += static_cast<double>(x[i]) * x[i];
    return std::sqrt(s / static_cast<double>(x.size() - from));
}
} // namespace

class VocoderTests : public UnitTest
{
public:
    VocoderTests() : UnitTest("Vocoder", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;
        const int samples = static_cast<int>(kRate * 2.2);
        const auto speech = speechLike(samples, 0);
        const auto otherSpeech = speechLike(samples, static_cast<int>(kRate * 0.55)); // same voice, other vowel order

        beginTest("band envelopes of the output follow the modulator (intelligibility proxy)");
        {
            for (const int carrier : {1, 2, 3})
                for (const int bands : {16, 24, 32})
                {
                    VocoderRig rig;
                    rig.set("fx_voc_carrier", static_cast<float>(carrier));
                    rig.set("fx_voc_bands", static_cast<float>(bands));
                    rig.set("fx_voc_sibilance", 0.0f);
                    std::vector<float> l(static_cast<size_t>(samples), 0.0f), r(l);
                    rig.run(l, r, &speech);
                    const double matched = meanBandCorrelation(speech, l, 4800);
                    VocoderRig control;
                    control.set("fx_voc_carrier", static_cast<float>(carrier));
                    control.set("fx_voc_bands", static_cast<float>(bands));
                    control.set("fx_voc_sibilance", 0.0f);
                    std::vector<float> cl(static_cast<size_t>(samples), 0.0f), cr(cl);
                    control.run(cl, cr, &otherSpeech);
                    const double unrelated = meanBandCorrelation(speech, cl, 4800);
                    const std::string what = "carrier " + std::to_string(carrier) + ", " + std::to_string(bands) +
                                             " bands: matched " + std::to_string(matched) + ", unrelated " +
                                             std::to_string(unrelated);
                    expect(matched > 0.55, what);
                    expect(matched > unrelated + 0.5, what);
                }
        }

        beginTest("the synth bus works as the carrier and the modulator's level sets the output level");
        {
            VocoderRig rig;
            rig.set("fx_voc_sibilance", 0.0f);
            std::vector<float> l(static_cast<size_t>(samples)), r(l);
            double ph = 0.0;
            for (size_t i = 0; i < l.size(); ++i)
            {
                ph += 110.0 / kRate;
                ph -= std::floor(ph);
                l[i] = r[i] = static_cast<float>(0.3 * (2.0 * ph - 1.0));
            }
            rig.run(l, r, &speech);
            const double corr = meanBandCorrelation(speech, l, 4800);
            expect(corr > 0.55, "bus carrier correlation " + std::to_string(corr));
            const double ratio = rms(l, 9600) / rms(speech, 9600);
            expect(ratio > 0.25 && ratio < 4.0, "output / modulator level " + std::to_string(ratio));
        }

        beginTest("sidechain input: channel choice, mono input, no input is silent");
        {
            std::vector<float> quiet(static_cast<size_t>(samples), 0.0f);
            const auto run = [&](int input, const std::vector<float>* left, const std::vector<float>* right)
            {
                VocoderRig rig;
                rig.set("fx_voc_carrier", 2.0f);
                rig.set("fx_voc_input", static_cast<float>(input));
                std::vector<float> l(static_cast<size_t>(samples), 0.0f), r(l);
                rig.run(l, r, left, right);
                return rms(l, 9600);
            };
            expect(run(2, nullptr, nullptr) < 1.0e-6, "no sidechain input");
            const double mono = run(2, &speech, nullptr);
            expect(mono > 0.001, "mono input " + std::to_string(mono));
            expect(run(0, &speech, &quiet) > 0.001 && run(1, &quiet, &speech) > 0.001, "left / right pick");
            expect(run(0, &quiet, &speech) < 1.0e-6 && run(1, &speech, &quiet) < 1.0e-6, "unused channel is ignored");
        }

        beginTest("dry / wet: mix 0 leaves the bus untouched, sibilance passes high frequencies");
        {
            VocoderRig dry;
            dry.set("fx_voc_mix", 0.0f);
            std::vector<float> l(static_cast<size_t>(samples)), r(l);
            Rng rng;
            for (size_t i = 0; i < l.size(); ++i)
                l[i] = r[i] = 0.2f * rng.bipolar();
            const auto before = l;
            dry.run(l, r, &speech);
            expect(l == before && r == before, "mix 0 is bit-exact");

            // A noise burst of the modulator above 6 kHz only comes through with the sibilance control up.
            std::vector<float> hiss(static_cast<size_t>(samples), 0.0f);
            double a = 0.0;
            for (size_t i = 0; i < hiss.size(); ++i)
            {
                const double x = 0.3 * rng.bipolar();
                hiss[i] = static_cast<float>(x - a); // first difference: tilted towards the highs
                a = x;
            }
            const auto energy = [&](float sib)
            {
                VocoderRig rig;
                rig.set("fx_voc_carrier", 1.0f);
                rig.set("fx_voc_sibilance", sib);
                rig.set("fx_voc_sib_freq", 5000.0f);
                std::vector<float> ol(static_cast<size_t>(samples), 0.0f), orr(ol);
                rig.run(ol, orr, &hiss);
                return rms(ol, 9600);
            };
            expect(energy(1.0f) > energy(0.0f) * 1.5, "sibilance adds the high band");
        }

        beginTest("formant shift moves the spectral envelope");
        {
            const auto centroid = [&](float shift)
            {
                VocoderRig rig;
                rig.set("fx_voc_carrier", 3.0f);
                rig.set("fx_voc_sibilance", 0.0f);
                rig.set("fx_voc_formant", shift);
                std::vector<float> l(static_cast<size_t>(samples), 0.0f), r(l);
                rig.run(l, r, &speech);
                // Energy-weighted mean of the zero-crossing rate proxy: use the first difference's share.
                double diff = 0.0, total = 0.0;
                for (size_t i = 9601; i < l.size(); ++i)
                {
                    const double d = l[i] - l[i - 1];
                    diff += d * d;
                    total += static_cast<double>(l[i]) * l[i];
                }
                return diff / std::max(total, 1.0e-12);
            };
            expect(centroid(7.0f) > centroid(-7.0f) * 1.2, "shifting up brightens the output");
        }

        beginTest("finite and bounded at extreme settings, block sizes and silent / loud inputs");
        {
            Rng rng;
            for (int round = 0; round < 12; ++round)
                for (const int block : {1, 7, 32, 64, 200})
                {
                    VocoderRig rig;
                    for (const auto& p : ModuleRegistry::instance().find("fx.vocoder")->params)
                    {
                        if (p.id == "fx_voc_on" || p.id == "fx_pos_voc")
                            continue;
                        const auto& d = ParameterMap::params()[static_cast<size_t>(ParameterMap::indexOf(p.id))];
                        float v = round < 2 ? (round == 0 ? d.min : d.max) : d.min + (d.max - d.min) * rng.unipolar();
                        if (d.type != ParamType::Float)
                            v = std::round(v);
                        rig.set(p.id, v);
                    }
                    std::vector<float> l(6000), r(l), aux(l.size());
                    for (size_t i = 0; i < l.size(); ++i)
                    {
                        l[i] = r[i] = 0.5f * rng.bipolar();
                        aux[i] = (i / 1500) % 2 == 0 ? 0.0f : 4.0f * rng.bipolar();
                    }
                    rig.run(l, r, &aux, nullptr, block);
                    bool ok = true;
                    for (size_t i = 0; i < l.size(); ++i)
                        ok = ok && std::isfinite(l[i]) && std::isfinite(r[i]) && std::abs(l[i]) < 200.0f;
                    expect(ok, "round " + std::to_string(round) + " block " + std::to_string(block));
                }
        }

        beginTest("deterministic: same input and seed give identical output, reset restarts the sequence");
        {
            const auto render = [&]
            {
                VocoderRig rig;
                rig.set("fx_voc_carrier", 3.0f);
                std::vector<float> l(static_cast<size_t>(samples), 0.0f), r(l);
                rig.run(l, r, &speech);
                return l;
            };
            expect(render() == render());
            VocoderRig rig;
            rig.set("fx_voc_carrier", 3.0f);
            std::vector<float> a(9600, 0.0f), ar(a), b(a), br(a);
            rig.run(a, ar, &speech);
            rig.rack.reset();
            rig.run(b, br, &speech);
            expect(a == b, "reset");
            // Block size does not change the result beyond the parameter ramps inside a block.
            VocoderRig big;
            big.set("fx_voc_carrier", 3.0f);
            std::vector<float> c(9600, 0.0f), cr(c);
            big.run(c, cr, &speech, nullptr, 128);
            expect(errorDb(a, c) < -90.0, "block independence " + std::to_string(errorDb(a, c)));
        }

        beginTest("engine facade and C API: sidechain input reaches the vocoder");
        {
            const auto render = [&](bool withAux, bool viaC)
            {
                Engine e;
                e.prepare(kRate, 512);
                e.setParam("fx_voc_on", 1.0f);
                e.setParam("fx_voc_carrier", 2.0f);
                e.setParam("fx_voc_mix", 1.0f);
                e.setParam("amp_sustain", 1.0f);
                e.noteOn(48, 1.0f);
                std::vector<float> l(static_cast<size_t>(samples)), r(l);
                for (size_t pos = 0; pos < l.size(); pos += 512)
                {
                    const int n = static_cast<int>(std::min<size_t>(512, l.size() - pos));
                    e.process(l.data() + pos, r.data() + pos, n, nullptr, 0, nullptr,
                              withAux ? speech.data() + pos : nullptr, nullptr);
                }
                (void)viaC;
                return l;
            };
            const auto with = render(true, false);
            const auto without = render(false, false);
            expect(rms(with, 9600) > 0.001, "vocoded output " + std::to_string(rms(with, 9600)));
            expect(rms(without, 9600) < 1.0e-6, "silent without a sidechain input");
            expect(render(true, false) == with, "deterministic through the engine");

            CrucibleEngine* ce = crucible_create(nullptr);
            crucible_prepare(ce, kRate, 512);
            crucible_set_param_id(ce, "fx_voc_on", 1.0f);
            crucible_set_param_id(ce, "fx_voc_carrier", 2.0f);
            crucible_set_param_id(ce, "amp_sustain", 1.0f);
            crucible_note_on(ce, 48, 1.0f);
            std::vector<float> l(static_cast<size_t>(samples)), r(l);
            for (size_t pos = 0; pos < l.size(); pos += 512)
            {
                const int n = static_cast<int>(std::min<size_t>(512, l.size() - pos));
                crucible_process_aux(ce, l.data() + pos, r.data() + pos, speech.data() + pos, nullptr, n, nullptr, 0, 0.0,
                                     -1.0, 0);
            }
            crucible_destroy(ce);
            expect(l == with, "C API matches the C++ facade");

            // crucible_process_events_aux: the parameter events of a block and the sidechain in one call.
            ce = crucible_create(nullptr);
            crucible_prepare(ce, kRate, 512);
            crucible_set_param_id(ce, "fx_voc_carrier", 2.0f);
            crucible_note_on(ce, 48, 1.0f);
            const CrucibleParamEvent first[] = {{0, crucible_param_index(ce, "fx_voc_on"), 1.0f},
                                                {0, crucible_param_index(ce, "amp_sustain"), 1.0f}};
            CrucibleTransport transport{};
            transport.struct_size = sizeof transport;
            transport.ppq = -1.0;
            std::vector<float> l2(l.size()), r2(l.size());
            for (size_t pos = 0; pos < l2.size(); pos += 512)
            {
                const int n = static_cast<int>(std::min<size_t>(512, l2.size() - pos));
                crucible_process_events_aux(ce, l2.data() + pos, r2.data() + pos, speech.data() + pos, nullptr, n,
                                            nullptr, 0, pos == 0 ? first : nullptr, pos == 0 ? 2 : 0, &transport);
            }
            crucible_destroy(ce);
            expect(l2 == with, "crucible_process_events_aux applies the block's parameter events and the sidechain");
        }
    }

private:
    static double errorDb(const std::vector<float>& a, const std::vector<float>& b)
    {
        double err = 0.0, sig = 0.0;
        for (size_t i = 0; i < a.size(); ++i)
        {
            err += (static_cast<double>(a[i]) - b[i]) * (static_cast<double>(a[i]) - b[i]);
            sig += static_cast<double>(a[i]) * a[i];
        }
        return 10.0 * std::log10(std::max(err, 1.0e-30) / std::max(sig, 1.0e-30));
    }
};

static VocoderTests vocoderTests;
