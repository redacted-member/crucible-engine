// SPDX-License-Identifier: MIT
#include <crucible/Filters.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <mutex>

namespace crucible
{
namespace
{
constexpr float kPi = 3.14159265358979f;

struct Vowel
{
    float f[3];
    float g[3];
};

constexpr std::array<Vowel, 5> kVowels = {{
    {{800.0f, 1150.0f, 2900.0f}, {1.0f, 0.5f, 0.25f}},
    {{400.0f, 1600.0f, 2700.0f}, {1.0f, 0.6f, 0.3f}},
    {{350.0f, 1700.0f, 2700.0f}, {1.0f, 0.55f, 0.3f}},
    {{450.0f, 800.0f, 2830.0f}, {1.0f, 0.6f, 0.2f}},
    {{325.0f, 700.0f, 2530.0f}, {1.0f, 0.4f, 0.2f}},
}};

// One formant of a user table at position pos01 (0..1 along the vowels) for resonance res. Frequency and bandwidth
// interpolate geometrically between the two neighbouring vowels, the gain linearly in dB. A bandwidth of 0 on both sides
// keeps the resonance-derived damping of the built-in table; a 0 on one side stands for the bandwidth that damping would
// give there, so the two kinds blend. With a bandwidth the formant is a unity-peak band-pass of Q = freq / bw * 0.5 * 16^res
// (at least 0.3).
template <typename T> struct UserBand
{
    T freq, k, gain;
};

template <typename T> UserBand<T> userFormantBand(const FormantTable& t, int b, T pos01, T res)
{
    const T pos = pos01 * static_cast<T>(t.vowels - 1);
    const int i = std::min(static_cast<int>(pos), t.vowels - 2);
    const T f = pos - static_cast<T>(i);
    const FormantBand& a = t.at(i, b);
    const FormantBand& c = t.at(i + 1, b);
    const T fa = static_cast<T>(a.freqHz), fc = static_cast<T>(c.freqHz);
    UserBand<T> out;
    out.freq = fa * std::pow(fc / fa, f);
    const T legacyK = static_cast<T>(1) - static_cast<T>(0.93) * res + static_cast<T>(0.07);
    const T qScale = static_cast<T>(0.5) * std::pow(static_cast<T>(16), res);
    if (a.bandwidthHz > 0.0f || c.bandwidthHz > 0.0f)
    {
        const T ba = a.bandwidthHz > 0.0f ? static_cast<T>(a.bandwidthHz) : legacyK * fa / qScale;
        const T bc = c.bandwidthHz > 0.0f ? static_cast<T>(c.bandwidthHz) : legacyK * fc / qScale;
        const T bw = ba * std::pow(bc / ba, f);
        out.k = static_cast<T>(1) / std::max(out.freq / bw * qScale, static_cast<T>(0.3));
    }
    else
        out.k = legacyK;
    const T db = static_cast<T>(a.gainDb) + (static_cast<T>(c.gainDb) - static_cast<T>(a.gainDb)) * f;
    out.gain = std::pow(static_cast<T>(10), db / static_cast<T>(20));
    return out;
}

// Damping of the state variable sections. Up to resonance 0.9 it falls linearly from 2 to 0.236 (Q about 4); the
// last tenth falls exponentially to 0.006 (Q about 330, a 50 dB peak) so the top of the dial reaches the
// near-self-oscillating peaks of the reference synths while the rest of the range is unchanged.
float damping(float res)
{
    if (res <= 0.9f)
        return 2.0f - 1.96f * res;
    return 0.236f * std::pow(0.006f / 0.236f, (res - 0.9f) / 0.1f);
}

float smoothStep(float a, float b, float x)
{
    const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Where the resonant peak of the low-pass sections lies relative to their cutoff parameter. A resonant state
// variable low-pass peaks below its cutoff (a 12 dB section at sqrt(1 - k^2 / 2)); the cascaded 24 dB filter's
// ratio is found numerically. configure() divides the cutoff by the ratio so the peak sits on the dial.
struct PeakTable
{
    static constexpr int steps = 64;
    std::array<float, steps + 1> lp12{};
    std::array<float, steps + 1> lp24{};

    PeakTable()
    {
        for (int i = 0; i <= steps; ++i)
        {
            const float k = damping(static_cast<float>(i) / steps);
            lp12[static_cast<size_t>(i)] = k < 1.41421356f ? std::sqrt(1.0f - 0.5f * k * k) : 0.0f;
            // |H|^2 of the cascade 1 / ((1 - w^2)^2 + k1^2 w^2) * 1 / ((1 - w^2)^2 + k2^2 w^2); maximum over w.
            const double k1 = 1.45, k2 = k;
            double best = 0.0, bestValue = -1.0;
            for (int n = 1; n <= 4000; ++n)
            {
                const double w = 0.3 + 1.2 * n / 4000.0;
                const double a = (1.0 - w * w) * (1.0 - w * w);
                const double v = 1.0 / ((a + k1 * k1 * w * w) * (a + k2 * k2 * w * w));
                if (v > bestValue)
                {
                    bestValue = v;
                    best = w;
                }
            }
            // Refine around the maximum.
            double lo = best - 0.0004, hi = best + 0.0004;
            for (int it = 0; it < 40; ++it)
            {
                const auto f = [&](double w)
                {
                    const double a = (1.0 - w * w) * (1.0 - w * w);
                    return 1.0 / ((a + k1 * k1 * w * w) * (a + k2 * k2 * w * w));
                };
                const double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
                if (f(m1) < f(m2))
                    lo = m1;
                else
                    hi = m2;
            }
            best = 0.5 * (lo + hi);
            lp24[static_cast<size_t>(i)] = best > 0.31 ? static_cast<float>(best) : 0.0f;
        }
    }

    static const PeakTable& instance()
    {
        static const PeakTable t;
        return t;
    }

    // Factor for the cutoff parameter so the peak lands on the requested frequency.
    float scale(const std::array<float, steps + 1>& table, float res) const
    {
        const float pos = std::clamp(res, 0.0f, 1.0f) * steps;
        const int i = std::min(static_cast<int>(pos), steps - 1);
        const float f = pos - static_cast<float>(i);
        const float ratio =
            table[static_cast<size_t>(i)] + (table[static_cast<size_t>(i + 1)] - table[static_cast<size_t>(i)]) * f;
        if (ratio < 0.3f)
            return 1.0f;
        const float w = smoothStep(0.6f, 0.85f, res);
        return 1.0f + (1.0f / ratio - 1.0f) * w;
    }
};

// Built when the library loads, so no audio thread ever runs the construction.
[[maybe_unused]] const PeakTable& gPeakTable = PeakTable::instance();

float softClip(float x)
{
    // Rational tanh approximation, bounded and smooth.
    const float c = std::clamp(x, -3.0f, 3.0f);
    return c * (27.0f + c * c) / (27.0f + 9.0f * c * c);
}
} // namespace

const std::vector<std::string>& filterModelNames()
{
    static const std::vector<std::string> names = {
        "Lowpass 12",    "Highpass 12",
        "Bandpass 12",   "Notch",
        "Peak",          "Lowpass 24",
        "Highpass 24",   "Bandpass 24",
        "Ladder LP",     "Ladder HP",
        "Ladder BP",     "Diode LP",
        "Comb +",        "Comb -",
        "Formant",       "Phaser",
        "Dirty LP",      "Transistor Ladder",
        "OTA Cascade",   "Coupled Diode Ladder",
        "Sallen-Key LP", "Sallen-Key HP",
        "OTA SVF",       "OTA 4-Pole",
        "Passive HP",
    };
    return names;
}

FilterChannel::FilterChannel(const FilterChannel& other)
{
    *this = other;
}

FilterChannel& FilterChannel::operator=(const FilterChannel& other)
{
    if (this == &other)
        return *this;
    model = other.model;
    g = other.g;
    k1 = other.k1;
    k2 = other.k2;
    a1 = other.a1;
    a2 = other.a2;
    a3 = other.a3;
    b1 = other.b1;
    b2 = other.b2;
    b3 = other.b3;
    s0 = other.s0;
    s1 = other.s1;
    G = other.G;
    kRes = other.kRes;
    ladderComp = other.ladderComp;
    ls = other.ls;
    comb = other.comb;
    combPos = other.combPos;
    combDelay = other.combDelay;
    combFb = other.combFb;
    combLp = other.combLp;
    combDamp = other.combDamp;
    fs = other.fs;
    fa1 = other.fa1;
    fa2 = other.fa2;
    fa3 = other.fa3;
    fk = other.fk;
    fgain = other.fgain;
    fcount = other.fcount;
    fout = other.fout;
    ap = other.ap;
    apG = other.apG;
    phaserFb = other.phaserFb;
    phaserLast = other.phaserLast;
    analogRate = other.analogRate;
    analogQuality = other.analogQuality;
    analogSeed = other.analogSeed;
    analogIndex = other.analogIndex;
    for (size_t i = 0; i < analog.size(); ++i)
        analog[i] = other.analog[i] ? other.analog[i]->clone() : nullptr;
    return *this;
}

void FilterChannel::prepare(double sampleRate, Quality quality)
{
    analogRate = sampleRate;
    analogQuality = quality;
    using Factory = std::unique_ptr<AnalogFilter> (*)();
    static constexpr Factory factories[kAnalogModels] = {makeTransistorLadder, makeOtaCascade,        makeDiodeLadder,
                                                         makeSallenKeyLowpass, makeSallenKeyHighpass, makeSvfOta,
                                                         makeOta4Pole,         makePassiveHp};
    for (size_t i = 0; i < analog.size(); ++i)
    {
        analog[i] = factories[i]();
        analog[i]->prepare(sampleRate, quality, 1, 32);
        analog[i]->setSeed(analogSeed + static_cast<std::uint32_t>(i));
    }
}

void FilterChannel::setSeed(std::uint32_t seed)
{
    analogSeed = seed;
    for (size_t i = 0; i < analog.size(); ++i)
        if (analog[i])
            analog[i]->setSeed(seed + static_cast<std::uint32_t>(i));
}

void FilterChannel::reset()
{
    for (auto& a : analog)
        if (a)
            a->reset();
    s0 = s1 = {};
    ls.fill(0.0f);
    comb.fill(0.0f);
    combPos = 0;
    combLp = 0.0f;
    fs.fill({});
    ap.fill(0.0f);
    phaserLast = 0.0f;
}

void FilterChannel::configure(const FilterSettings& s, double sampleRate)
{
    lastSettings = s;
    lastRate = sampleRate;
    const auto newModel = static_cast<FilterModel>(std::clamp(s.model, 0, static_cast<int>(FilterModel::Count) - 1));
    if (newModel != model)
    {
        model = newModel;
        reset();
    }
    const float sr = static_cast<float>(sampleRate);
    const float fc = std::clamp(s.cutoffHz, 20.0f, 0.45f * sr);
    const float res = std::clamp(s.resonance, 0.0f, 1.0f);
    if (isAnalogModel(static_cast<int>(model)))
    {
        if (!analog[0] || analogRate != sampleRate)
            prepare(sampleRate, analogQuality);
        analogIndex = static_cast<int>(model) - static_cast<int>(FilterModel::TransistorLadder);
        auto& active = *analog[static_cast<size_t>(analogIndex)];
        active.setControls(fc, res, s.drive, s.character, s.tolerance);
        active.setHighpassStep(s.highpass);
        return;
    }
    g = std::tan(kPi * fc / sr);
    if (model == FilterModel::Lowpass12)
        g *= PeakTable::instance().scale(PeakTable::instance().lp12, res);
    else if (model == FilterModel::Lowpass24)
        g *= PeakTable::instance().scale(PeakTable::instance().lp24, res);

    switch (model)
    {
    case FilterModel::Lowpass12:
    case FilterModel::Highpass12:
    case FilterModel::Bandpass12:
    case FilterModel::Notch:
    case FilterModel::Peak:
    case FilterModel::DirtyLowpass:
        k1 = damping(res);
        a1 = 1.0f / (1.0f + g * (g + k1));
        a2 = g * a1;
        a3 = g * a2;
        break;
    case FilterModel::Lowpass24:
    case FilterModel::Highpass24:
    case FilterModel::Bandpass24:
        k1 = 1.45f;
        k2 = damping(res);
        a1 = 1.0f / (1.0f + g * (g + k1));
        a2 = g * a1;
        a3 = g * a2;
        b1 = 1.0f / (1.0f + g * (g + k2));
        b2 = g * b1;
        b3 = g * b2;
        break;
    case FilterModel::LadderLowpass:
    case FilterModel::LadderHighpass:
    case FilterModel::LadderBandpass:
        G = g / (1.0f + g);
        kRes = 4.0f * res;
        ladderComp = 1.0f + 0.45f * kRes;
        break;
    case FilterModel::DiodeLowpass:
        G = g / (1.0f + g);
        kRes = 3.6f * res;
        ladderComp = 1.0f + 0.45f * kRes;
        break;
    case FilterModel::CombPlus:
    case FilterModel::CombMinus:
        combDelay = std::clamp(sr / fc, 2.0f, static_cast<float>(combSize - 4));
        combFb = 0.985f * res;
        combDamp = 0.35f;
        break;
    case FilterModel::Formant:
    {
        const float pos01 = std::clamp(std::log(fc / 20.0f) / std::log(1000.0f), 0.0f, 1.0f);
        if (s.formants != nullptr && s.formants->vowels >= 2)
        {
            const FormantTable& t = *s.formants;
            fcount = t.bands;
            fout = t.outputGain;
            fsat = t.saturate;
            for (int b = 0; b < t.bands; ++b)
            {
                const auto ub = userFormantBand<float>(t, b, pos01, res);
                const size_t bi = static_cast<size_t>(b);
                fgain[bi] = ub.gain;
                const float gg = std::tan(kPi * std::min(ub.freq, 0.45f * sr) / sr);
                fk[bi] = ub.k;
                fa1[bi] = 1.0f / (1.0f + gg * (gg + fk[bi]));
                fa2[bi] = gg * fa1[bi];
                fa3[bi] = gg * fa2[bi];
            }
            break;
        }
        fcount = 3;
        fout = 1.4f;
        fsat = false;
        const float pos = pos01 * 4.0f;
        const int i = std::min(static_cast<int>(pos), 3);
        const float f = pos - static_cast<float>(i);
        for (size_t b = 0; b < 3; ++b)
        {
            const float freq =
                kVowels[static_cast<size_t>(i)].f[b] * (1.0f - f) + kVowels[static_cast<size_t>(i + 1)].f[b] * f;
            fgain[b] = kVowels[static_cast<size_t>(i)].g[b] * (1.0f - f) + kVowels[static_cast<size_t>(i + 1)].g[b] * f;
            const float gg = std::tan(kPi * std::min(freq, 0.45f * sr) / sr);
            fk[b] = 1.0f - 0.93f * res + 0.07f;
            fa1[b] = 1.0f / (1.0f + gg * (gg + fk[b]));
            fa2[b] = gg * fa1[b];
            fa3[b] = gg * fa2[b];
        }
        break;
    }
    case FilterModel::Phaser:
        apG = g / (1.0f + g);
        phaserFb = 0.95f * res;
        break;
    case FilterModel::TransistorLadder:
    case FilterModel::OtaCascade:
    case FilterModel::CoupledDiodeLadder:
    case FilterModel::SallenKeyLowpass:
    case FilterModel::SallenKeyHighpass:
    case FilterModel::SvfOta:
    case FilterModel::Ota4Pole:
    case FilterModel::PassiveHighpass:
    case FilterModel::Count:
        break;
    }
}

void FilterChannel::trackModel(int modelIndex)
{
    const auto newModel = static_cast<FilterModel>(std::clamp(modelIndex, 0, static_cast<int>(FilterModel::Count) - 1));
    if (newModel != model)
    {
        model = newModel;
        reset();
    }
}

float FilterChannel::svf(SvfState& st, float x, float c1, float c2, float c3, float k, int out) const
{
    const float v3 = x - st.ic2;
    const float v1 = c1 * st.ic1 + c2 * v3;
    const float v2 = st.ic2 + c2 * st.ic1 + c3 * v3;
    st.ic1 = 2.0f * v1 - st.ic1;
    st.ic2 = 2.0f * v2 - st.ic2;
    switch (out)
    {
    case 0:
        return v2;
    case 1:
        return x - k * v1 - v2;
    case 2:
        return k * v1;
    case 3:
        return x - k * v1;
    default:
        return 2.0f * v2 + k * v1 - x;
    }
}

float FilterChannel::process(float x)
{
    switch (model)
    {
    case FilterModel::Lowpass12:
        return svf(s0, x, a1, a2, a3, k1, 0);
    case FilterModel::Highpass12:
        return svf(s0, x, a1, a2, a3, k1, 1);
    case FilterModel::Bandpass12:
        return svf(s0, x, a1, a2, a3, k1, 2);
    case FilterModel::Notch:
        return svf(s0, x, a1, a2, a3, k1, 3);
    case FilterModel::Peak:
        return svf(s0, x, a1, a2, a3, k1, 4);
    case FilterModel::Lowpass24:
        return svf(s1, svf(s0, x, a1, a2, a3, k1, 0), b1, b2, b3, k2, 0);
    case FilterModel::Highpass24:
        return svf(s1, svf(s0, x, a1, a2, a3, k1, 1), b1, b2, b3, k2, 1);
    case FilterModel::Bandpass24:
        return svf(s1, svf(s0, x, a1, a2, a3, k1, 2), b1, b2, b3, k2, 2);
    case FilterModel::DirtyLowpass:
    {
        const float v3 = x - s0.ic2;
        const float v1 = softClip(a1 * s0.ic1 + a2 * v3);
        const float v2 = s0.ic2 + a2 * s0.ic1 + a3 * v3;
        s0.ic1 = 2.0f * v1 - s0.ic1;
        s0.ic2 = 2.0f * v2 - s0.ic2;
        return v2;
    }
    case FilterModel::LadderLowpass:
    case FilterModel::LadderHighpass:
    case FilterModel::LadderBandpass:
    case FilterModel::DiodeLowpass:
    {
        const bool diode = model == FilterModel::DiodeLowpass;
        const float oneMinusG = 1.0f - G;
        const float g2 = G * G;
        const float S = oneMinusG * (g2 * G * ls[0] + g2 * ls[1] + G * ls[2] + ls[3]);
        const float uLin = (x * ladderComp - kRes * S) / (1.0f + kRes * g2 * g2);
        float u = softClip(uLin * 0.5f) * 2.0f;
        if (diode)
            u = softClip(u + 0.12f) - softClip(0.12f);
        float stage[4];
        float in = u;
        for (size_t i = 0; i < 4; ++i)
        {
            const float drive = diode && i > 0 ? softClip(in) : in;
            const float v = G * (drive - ls[i]);
            const float y = v + ls[i];
            ls[i] = y + v;
            stage[i] = y;
            in = y;
        }
        switch (model)
        {
        case FilterModel::LadderHighpass:
            return u - 4.0f * stage[0] + 6.0f * stage[1] - 4.0f * stage[2] + stage[3];
        case FilterModel::LadderBandpass:
            return 2.0f * (stage[1] - stage[3]);
        case FilterModel::DiodeLowpass:
            return 0.35f * stage[2] + 0.65f * stage[3];
        case FilterModel::Lowpass12:
        case FilterModel::Highpass12:
        case FilterModel::Bandpass12:
        case FilterModel::Notch:
        case FilterModel::Peak:
        case FilterModel::Lowpass24:
        case FilterModel::Highpass24:
        case FilterModel::Bandpass24:
        case FilterModel::LadderLowpass:
        case FilterModel::CombPlus:
        case FilterModel::CombMinus:
        case FilterModel::Formant:
        case FilterModel::Phaser:
        case FilterModel::DirtyLowpass:
        case FilterModel::TransistorLadder:
        case FilterModel::OtaCascade:
        case FilterModel::CoupledDiodeLadder:
        case FilterModel::SallenKeyLowpass:
        case FilterModel::SallenKeyHighpass:
        case FilterModel::SvfOta:
        case FilterModel::Ota4Pole:
        case FilterModel::PassiveHighpass:
        case FilterModel::Count:
        default:
            return stage[3];
        }
    }
    case FilterModel::CombPlus:
    case FilterModel::CombMinus:
    {
        const float pos = static_cast<float>(combPos) - combDelay;
        const float fl = std::floor(pos);
        const float frac = pos - fl;
        const int i0 = (static_cast<int>(fl) + combSize) & (combSize - 1);
        const int i1 = (i0 + 1) & (combSize - 1);
        const float delayed = comb[static_cast<size_t>(i0)] * (1.0f - frac) + comb[static_cast<size_t>(i1)] * frac;
        combLp += (delayed - combLp) * (1.0f - combDamp);
        const float sign = model == FilterModel::CombPlus ? 1.0f : -1.0f;
        const float y = x + sign * combFb * combLp;
        comb[static_cast<size_t>(combPos)] = std::clamp(y, -8.0f, 8.0f);
        combPos = (combPos + 1) & (combSize - 1);
        return y * (1.0f - 0.5f * combFb);
    }
    case FilterModel::Formant:
    {
        float sum = 0.0f;
        if (fsat)
        {
            // State saturation: the band-pass state is soft-clipped, the low-pass state clamped.
            for (size_t b = 0; b < static_cast<size_t>(fcount); ++b)
            {
                SvfState& st = fs[b];
                const float v3 = x - st.ic2;
                const float v1 = fa1[b] * st.ic1 + fa2[b] * v3;
                const float v2 = st.ic2 + fa2[b] * st.ic1 + fa3[b] * v3;
                const float u = std::clamp(v1 + v1 - st.ic1, -6.0f, 6.0f) * 0.5f; // 0.5 * (2 v1 - st0), clipped to +-3
                st.ic1 = 2.0f * (u * (27.0f + u * u) / (27.0f + 9.0f * u * u));
                st.ic2 = std::clamp(2.0f * v2 - st.ic2, -8.0f, 8.0f);
                sum += fk[b] * v1 * fgain[b];
            }
            return sum * fout;
        }
        for (size_t b = 0; b < static_cast<size_t>(fcount); ++b)
            sum += svf(fs[b], x, fa1[b], fa2[b], fa3[b], fk[b], 2) * fgain[b];
        return sum * fout;
    }
    case FilterModel::Phaser:
    {
        float v = x + phaserFb * phaserLast;
        const float input = v;
        for (auto& s : ap)
        {
            const float t = apG * (v - s);
            const float lp = t + s;
            s = lp + t;
            v = 2.0f * lp - v;
        }
        phaserLast = std::clamp(v, -8.0f, 8.0f);
        return 0.5f * (input + v);
    }
    case FilterModel::TransistorLadder:
    case FilterModel::OtaCascade:
    case FilterModel::CoupledDiodeLadder:
    case FilterModel::SallenKeyLowpass:
    case FilterModel::SallenKeyHighpass:
    case FilterModel::SvfOta:
    case FilterModel::Ota4Pole:
    case FilterModel::PassiveHighpass:
        if (analogIndex >= 0 && analog[static_cast<size_t>(analogIndex)])
            analog[static_cast<size_t>(analogIndex)]->process(&x, 1, nullptr);
        return x;
    case FilterModel::Count:
        break;
    }
    return x;
}

void FilterChannel::processBlock(float* x, int n, const float* cutoffOct)
{
    if (isAnalogModel(static_cast<int>(model)))
    {
        if (analogIndex >= 0 && analog[static_cast<size_t>(analogIndex)])
            analog[static_cast<size_t>(analogIndex)]->process(x, n, cutoffOct);
        return;
    }
    if (cutoffOct == nullptr)
    {
        for (int i = 0; i < n; ++i)
            x[i] = process(x[i]);
        return;
    }
    // Coefficients every four samples, at the cutoff the modulation has reached at the end of the group.
    const FilterSettings base = lastSettings;
    const double rate = lastRate;
    for (int j = 0; j < n; j += 4)
    {
        const int m = std::min(4, n - j);
        FilterSettings s = base;
        s.cutoffHz = base.cutoffHz * std::exp2(cutoffOct[j + m - 1]);
        configure(s, rate);
        for (int i = 0; i < m; ++i)
            x[j + i] = process(x[j + i]);
    }
    lastSettings = base;
}

void FilterUnit::prepare(double sampleRate, Quality quality)
{
    for (auto& c : ch)
        c.prepare(sampleRate, quality);
}

void FilterUnit::setSeed(std::uint32_t seed)
{
    for (size_t i = 0; i < ch.size(); ++i)
        ch[i].setSeed(seed + static_cast<std::uint32_t>(i) * 16U);
}

void FilterUnit::reset()
{
    for (auto& c : ch)
        c.reset();
    for (auto& a : adaa)
        a.reset();
}

void FilterUnit::configure(const FilterSettings& s, float drive, double sampleRate)
{
    // The nonlinear models drive their own input stage.
    const bool analogModel = FilterChannel::isAnalogModel(s.model);
    driven = drive > 0.001f && !analogModel;
    driveGain = 1.0f + drive * 8.0f;
    driveComp = 1.0f / std::sqrt(driveGain);
    FilterSettings withDrive = s;
    if (analogModel)
        withDrive.drive = drive;
    for (auto& c : ch)
        c.configure(withDrive, sampleRate);
}

void FilterUnit::trackModel(int modelIndex)
{
    driven = false;
    for (auto& c : ch)
        c.trackModel(modelIndex);
}

void FilterUnit::process(float& l, float& r)
{
    if (driven)
    {
        l = adaa[0].process(l * driveGain) * driveComp;
        r = adaa[1].process(r * driveGain) * driveComp;
    }
    l = ch[0].process(l);
    r = ch[1].process(r);
}

void FilterUnit::processBlock(float* l, float* r, int n, const float* cutoffOct)
{
    if (driven)
        for (int i = 0; i < n; ++i)
        {
            l[i] = adaa[0].process(l[i] * driveGain) * driveComp;
            r[i] = adaa[1].process(r[i] * driveGain) * driveComp;
        }
    ch[0].processBlock(l, n, cutoffOct);
    ch[1].processBlock(r, n, cutoffOct);
}

// ---------------------------------------------------------------------------------------------------------------
// Small-signal response.

namespace
{
using Cx = std::complex<double>;

double toDb(Cx h)
{
    return 20.0 * std::log10(std::max(std::abs(h), 1.0e-9));
}

// The state variable section in the normalised frequency p = j w: out 0 low-pass, 1 high-pass, 2 band-pass (k times the
// band signal), 3 notch, 4 peak.
Cx svfGain(int out, Cx p, double k)
{
    const Cx d = p * p + k * p + 1.0;
    switch (out)
    {
    case 0:
        return 1.0 / d;
    case 1:
        return p * p / d;
    case 2:
        return k * p / d;
    case 3:
        return (p * p + 1.0) / d;
    default:
        return (1.0 - p * p) / d;
    }
}

// Slope of softClip at c.
double softClipSlope(double c)
{
    const double c2 = c * c;
    const double d = 27.0 + 9.0 * c2;
    return ((27.0 + 3.0 * c2) * d - c * (27.0 + c2) * 18.0 * c) / (d * d);
}

Cx legacyGain(const FilterSettings& s, double sr, double f)
{
    constexpr double pi = 3.14159265358979323846;
    const auto model = static_cast<FilterModel>(std::clamp(s.model, 0, static_cast<int>(FilterModel::Count) - 1));
    const double fc = std::clamp(static_cast<double>(s.cutoffHz), 20.0, 0.45 * sr);
    const float res = std::clamp(s.resonance, 0.0f, 1.0f);
    double g = std::tan(pi * fc / sr);
    if (model == FilterModel::Lowpass12)
        g *= PeakTable::instance().scale(PeakTable::instance().lp12, res);
    else if (model == FilterModel::Lowpass24)
        g *= PeakTable::instance().scale(PeakTable::instance().lp24, res);
    const double w = std::tan(pi * std::min(f, 0.4999 * sr) / sr);
    const Cx p(0.0, w / g);
    switch (model)
    {
    case FilterModel::Lowpass12:
    case FilterModel::DirtyLowpass:
        return svfGain(0, p, damping(res));
    case FilterModel::Highpass12:
        return svfGain(1, p, damping(res));
    case FilterModel::Bandpass12:
        return svfGain(2, p, damping(res));
    case FilterModel::Notch:
        return svfGain(3, p, damping(res));
    case FilterModel::Peak:
        return svfGain(4, p, damping(res));
    case FilterModel::Lowpass24:
        return svfGain(0, p, 1.45) * svfGain(0, p, damping(res));
    case FilterModel::Highpass24:
        return svfGain(1, p, 1.45) * svfGain(1, p, damping(res));
    case FilterModel::Bandpass24:
        return svfGain(2, p, 1.45) * svfGain(2, p, damping(res));
    case FilterModel::LadderLowpass:
    case FilterModel::LadderHighpass:
    case FilterModel::LadderBandpass:
    case FilterModel::DiodeLowpass:
    {
        // Four trapezoidal one-pole stages h = 1 / (1 + p) with the input compensation and the resonance feedback from
        // the last stage. The shaper ahead of the stages is linear for small signals; the diode ladder's offset shaper
        // has the slope s0 at its working point, and because the zero-delay solve assumes unit slope the loop reads
        // 1 + k G^4 + s0 k (h^4 - G^4) with G the instantaneous gain of one stage.
        const bool diode = model == FilterModel::DiodeLowpass;
        const double k = (diode ? 3.6 : 4.0) * static_cast<double>(res);
        const double comp = 1.0 + 0.45 * k;
        const double s0 = diode ? softClipSlope(0.12) : 1.0;
        const double stage = g / (1.0 + g);
        const double g4 = stage * stage * stage * stage;
        const Cx h = 1.0 / (1.0 + p);
        const Cx h2 = h * h, h3 = h2 * h, h4 = h2 * h2;
        const Cx u = s0 * comp / (1.0 + k * g4 + s0 * k * (h4 - g4));
        switch (model)
        {
        case FilterModel::LadderHighpass:
            return u * (1.0 - h) * (1.0 - h) * (1.0 - h) * (1.0 - h);
        case FilterModel::LadderBandpass:
            return u * 2.0 * (h2 - h4);
        case FilterModel::DiodeLowpass:
            return u * (0.35 * h3 + 0.65 * h4);
        default:
            return u * h4;
        }
    }
    case FilterModel::CombPlus:
    case FilterModel::CombMinus:
    {
        const double delay = std::clamp(sr / fc, 2.0, static_cast<double>(FilterChannel::combSize - 4));
        const double fb = 0.985 * static_cast<double>(res);
        const double damp = 0.35;
        const double whole = std::floor(delay), frac = delay - whole;
        const Cx zinv = std::polar(1.0, -2.0 * pi * f / sr);
        const Cx zn = std::pow(zinv, whole);
        const Cx delayed = zn * ((1.0 - frac) + frac * zinv);
        const Cx lp = (1.0 - damp) / (1.0 - damp * zinv);
        const double sign = model == FilterModel::CombPlus ? 1.0 : -1.0;
        return (1.0 - 0.5 * fb) / (1.0 - sign * fb * lp * delayed);
    }
    case FilterModel::Formant:
    {
        const double fcl = std::clamp(static_cast<double>(s.cutoffHz), 20.0, 0.45 * sr);
        const float pos01 = std::clamp(static_cast<float>(std::log(fcl / 20.0) / std::log(1000.0)), 0.0f, 1.0f);
        if (s.formants != nullptr && s.formants->vowels >= 2)
        {
            const FormantTable& t = *s.formants;
            Cx usum = 0.0;
            for (int b = 0; b < t.bands; ++b)
            {
                const auto ub = userFormantBand<double>(t, b, static_cast<double>(pos01), static_cast<double>(res));
                const double gg = std::tan(pi * std::min(ub.freq, 0.45 * sr) / sr);
                usum += svfGain(2, Cx(0.0, w / gg), ub.k) * ub.gain;
            }
            return usum * static_cast<double>(t.outputGain);
        }
        const float pos = pos01 * 4.0f;
        const int i = std::min(static_cast<int>(pos), 3);
        const double fr = static_cast<double>(pos - static_cast<float>(i));
        Cx sum = 0.0;
        for (size_t b = 0; b < 3; ++b)
        {
            const double freq =
                kVowels[static_cast<size_t>(i)].f[b] * (1.0 - fr) + kVowels[static_cast<size_t>(i + 1)].f[b] * fr;
            const double gain =
                kVowels[static_cast<size_t>(i)].g[b] * (1.0 - fr) + kVowels[static_cast<size_t>(i + 1)].g[b] * fr;
            const double gg = std::tan(pi * std::min(freq, 0.45 * sr) / sr);
            const double k = 1.0 - 0.93 * static_cast<double>(res) + 0.07;
            sum += svfGain(2, Cx(0.0, w / gg), k) * gain;
        }
        return sum * 1.4;
    }
    case FilterModel::Phaser:
    {
        const Cx allpass = (1.0 - p) / (1.0 + p);
        const Cx a6 = std::pow(allpass, 6);
        const Cx zinv = std::polar(1.0, -2.0 * pi * f / sr);
        const double fb = 0.95 * static_cast<double>(res);
        return 0.5 * (1.0 + a6) / (1.0 - fb * zinv * a6);
    }
    case FilterModel::TransistorLadder:
    case FilterModel::OtaCascade:
    case FilterModel::CoupledDiodeLadder:
    case FilterModel::SallenKeyLowpass:
    case FilterModel::SallenKeyHighpass:
    case FilterModel::SvfOta:
    case FilterModel::Ota4Pole:
    case FilterModel::PassiveHighpass:
    case FilterModel::Count:
        break;
    }
    return 1.0;
}

// Prepared analog models for the response, one set for the rate and quality last asked for.
struct AnalogResponseModels
{
    std::mutex lock;
    double rate = 0.0;
    Quality quality = Quality::Standard;
    std::array<std::unique_ptr<AnalogFilter>, FilterChannel::kAnalogModels> models;
};
} // namespace

void filterResponseDb(const FilterSettings& s, float drive, double sampleRate, Quality quality, const float* freqHz, int n,
                      float* outDb)
{
    if (n <= 0)
        return;
    const int index = std::clamp(s.model, 0, static_cast<int>(FilterModel::Count) - 1);
    if (FilterChannel::isAnalogModel(index))
    {
        static AnalogResponseModels cache;
        const std::lock_guard<std::mutex> guard(cache.lock);
        if (cache.rate != sampleRate || cache.quality != quality)
        {
            cache.models = {};
            cache.rate = sampleRate;
            cache.quality = quality;
        }
        const auto a = static_cast<size_t>(index - static_cast<int>(FilterModel::TransistorLadder));
        if (!cache.models[a])
        {
            using Factory = std::unique_ptr<AnalogFilter> (*)();
            static constexpr Factory factories[FilterChannel::kAnalogModels] = {
                makeTransistorLadder,  makeOtaCascade, makeDiodeLadder, makeSallenKeyLowpass,
                makeSallenKeyHighpass, makeSvfOta,     makeOta4Pole,    makePassiveHp};
            cache.models[a] = factories[a]();
            cache.models[a]->prepare(sampleRate, quality, 1, 32);
        }
        auto& model = *cache.models[a];
        const float fc = std::clamp(s.cutoffHz, 20.0f, 0.45f * static_cast<float>(sampleRate));
        model.setControls(fc, std::clamp(s.resonance, 0.0f, 1.0f), drive, s.character, 0.0f);
        model.setHighpassStep(s.highpass);
        model.responseDb(freqHz, n, outDb);
        return;
    }
    // The drive stage ahead of the models 0..16 is an antiderivative-anti-aliased tanh: for small signals a gain of
    // sqrt(1 + 8 drive) and the two-sample average of the method, cos(pi f / rate) in magnitude.
    const bool driven = drive > 0.001f;
    const double driveGain = driven ? std::sqrt(1.0 + 8.0 * static_cast<double>(drive)) : 1.0;
    for (int i = 0; i < n; ++i)
    {
        const double f = static_cast<double>(freqHz[i]);
        const double average = driven ? std::cos(3.14159265358979323846 * std::min(f, 0.5 * sampleRate) / sampleRate) : 1.0;
        outDb[i] = static_cast<float>(toDb(legacyGain(s, sampleRate, f) * (driveGain * average)));
    }
}

} // namespace crucible

namespace crucible
{
bool makeFormantTable(const FormantBand* bands, int vowels, int bandsPerVowel, float outputGain, FormantTable& out,
                      std::string& error)
{
    if (vowels < 2 || vowels > kMaxFormantVowels)
    {
        error = "vowel_count must be 2..8";
        return false;
    }
    if (bandsPerVowel < 1 || bandsPerVowel > kMaxFormantBands)
    {
        error = "formant_count must be 1..4";
        return false;
    }
    if (bands == nullptr)
    {
        error = "formants is null";
        return false;
    }
    if (!std::isfinite(outputGain) || outputGain <= 0.0f)
    {
        error = "output_gain must be finite and > 0";
        return false;
    }
    for (int v = 0; v < vowels; ++v)
        for (int b = 0; b < bandsPerVowel; ++b)
        {
            const FormantBand& e = bands[v * bandsPerVowel + b];
            const std::string where = " (vowel " + std::to_string(v) + ", formant " + std::to_string(b) + ")";
            if (!std::isfinite(e.freqHz) || e.freqHz < 20.0f || e.freqHz > 20000.0f)
            {
                error = "freq_hz must be finite and within 20..20000" + where;
                return false;
            }
            if (!std::isfinite(e.bandwidthHz) || e.bandwidthHz < 0.0f)
            {
                error = "bandwidth_hz must be finite and >= 0" + where;
                return false;
            }
            if (!std::isfinite(e.gainDb) || e.gainDb > kMaxFormantGainDb)
            {
                error = "gain_db must be finite and <= +24" + where;
                return false;
            }
        }
    out = FormantTable{};
    out.vowels = vowels;
    out.bands = bandsPerVowel;
    out.outputGain = outputGain;
    for (int v = 0; v < vowels; ++v)
        for (int b = 0; b < bandsPerVowel; ++b)
            out.band[static_cast<size_t>(v * kMaxFormantBands + b)] = bands[v * bandsPerVowel + b];
    return true;
}
} // namespace crucible
