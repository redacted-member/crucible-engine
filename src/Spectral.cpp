// SPDX-License-Identifier: MIT
#include <crucible/Spectral.h>

#include <crucible/Wavetables.h>

#include <algorithm>
#include <bit>
#include <cmath>

namespace crucible::spectral
{
namespace
{
std::atomic<long> gAnalyses{0};
std::atomic<std::uint64_t> gSerial{0};
constexpr float kPi = 3.14159265358979f;
constexpr float kPeakFloor = 0.003f; // relative to the largest smoothed magnitude (-50 dB)

// Complex value of a spectrum at a fractional harmonic position: the magnitude interpolates linearly, the direction from
// the linear blend of the two neighbouring bins (so a peak that moves keeps a phase from its source).
inline void sampleAt(const float* re, const float* im, float p, float& outRe, float& outIm)
{
    if (!(p >= 0.0f) || p >= static_cast<float>(kBins - 1))
    {
        outRe = outIm = 0.0f;
        return;
    }
    const int i0 = static_cast<int>(p);
    const float f = p - static_cast<float>(i0);
    const float r0 = re[i0], i0v = im[i0], r1 = re[i0 + 1], i1 = im[i0 + 1];
    const float m0 = std::sqrt(r0 * r0 + i0v * i0v), m1 = std::sqrt(r1 * r1 + i1 * i1);
    const float m = m0 + (m1 - m0) * f;
    float dr = r0 + (r1 - r0) * f, di = i0v + (i1 - i0v) * f;
    const float dm = std::sqrt(dr * dr + di * di);
    if (dm < 1.0e-20f)
    {
        // The neighbours cancel: take the direction of the larger one.
        dr = m0 >= m1 ? r0 : r1;
        di = m0 >= m1 ? i0v : i1;
        const float d2 = std::sqrt(dr * dr + di * di);
        if (d2 < 1.0e-20f)
        {
            outRe = outIm = 0.0f;
            return;
        }
        outRe = dr / d2 * m;
        outIm = di / d2 * m;
        return;
    }
    outRe = dr / dm * m;
    outIm = di / dm * m;
}

// Maps an output harmonic to a source position through the piecewise linear map with the given anchors (output side
// `to`, source side `from`, both ascending). `seg` is advanced monotonically by the caller.
inline float mapPosition(const float* from, const float* to, int count, int& seg, float k)
{
    while (seg + 1 < count - 1 && k > to[seg + 1])
        ++seg;
    const float dt = to[seg + 1] - to[seg];
    if (dt <= 1.0e-6f)
        return from[seg];
    return from[seg] + (k - to[seg]) * (from[seg + 1] - from[seg]) / dt;
}
} // namespace

const char* warpName(Warp w)
{
    switch (w)
    {
    case Warp::Shift:
        return "Shift";
    case Warp::Stretch:
        return "Stretch";
    case Warp::Blur:
        return "Blur";
    case Warp::Comb:
        return "Comb";
    case Warp::Dispersion:
        return "Dispersion";
    case Warp::Tilt:
        return "Tilt";
    default:
        return "Off";
    }
}

std::vector<Peak> findPeaks(const float* magnitude)
{
    std::array<float, kBins + 2> s{};
    for (int k = 1; k <= kHarmonics; ++k)
        s[static_cast<size_t>(k)] =
            (magnitude[k - 1] * (k == 1 ? 0.0f : 1.0f) + 2.0f * magnitude[k] + magnitude[k + 1 < kBins ? k + 1 : k]) * 0.25f;
    float top = 0.0f;
    for (int k = 1; k <= kHarmonics; ++k)
        top = std::max(top, s[static_cast<size_t>(k)]);
    std::vector<Peak> peaks;
    if (top <= 0.0f)
        return peaks;
    const float limit = top * kPeakFloor;
    for (int k = 1; k <= kHarmonics; ++k)
    {
        const float c = s[static_cast<size_t>(k)];
        if (c < limit)
            continue;
        const float l = s[static_cast<size_t>(k - 1)], r = s[static_cast<size_t>(k + 1)];
        if (!(c > l && c >= r))
            continue;
        if (k >= 2 && s[static_cast<size_t>(k - 2)] > c)
            continue;
        if (k + 2 <= kHarmonics && s[static_cast<size_t>(k + 2)] > c)
            continue;
        float pos = static_cast<float>(k);
        if (l > 0.0f && r > 0.0f)
        {
            const float ll = std::log(l), lc = std::log(c), lr = std::log(r);
            const float den = ll - 2.0f * lc + lr;
            if (den < -1.0e-6f)
                pos += std::clamp(0.5f * (ll - lr) / den, -0.5f, 0.5f);
        }
        peaks.push_back({pos, c});
    }
    if (peaks.size() > static_cast<size_t>(kMaxPeaks))
    {
        std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return a.magnitude > b.magnitude; });
        peaks.resize(static_cast<size_t>(kMaxPeaks));
        std::sort(peaks.begin(), peaks.end(), [](const Peak& a, const Peak& b) { return a.position < b.position; });
    }
    return peaks;
}

// Monotone alignment of two peak lists (Needleman-Wunsch): matching costs the distance in octaves (plus a little of the
// level difference), leaving a peak unmatched costs one octave, so peaks more than two octaves apart never pair.
std::vector<Match> matchPeaks(const std::vector<Peak>& a, const std::vector<Peak>& b)
{
    const size_t n = a.size(), m = b.size();
    std::vector<Match> out;
    if (n == 0 || m == 0)
        return out;
    constexpr float skip = 1.0f;
    std::vector<float> cost((n + 1) * (m + 1), 0.0f);
    const auto at = [&](size_t i, size_t j) -> float& { return cost[i * (m + 1) + j]; };
    for (size_t i = 1; i <= n; ++i)
        at(i, 0) = skip * static_cast<float>(i);
    for (size_t j = 1; j <= m; ++j)
        at(0, j) = skip * static_cast<float>(j);
    const auto pairCost = [&](size_t i, size_t j)
    {
        const float octaves = std::abs(std::log2(a[i].position / b[j].position));
        const float level = std::abs(std::log2((a[i].magnitude + 1.0e-12f) / (b[j].magnitude + 1.0e-12f)));
        return octaves + 0.1f * std::min(level, 8.0f);
    };
    for (size_t i = 1; i <= n; ++i)
        for (size_t j = 1; j <= m; ++j)
        {
            float best = std::min(at(i - 1, j), at(i, j - 1)) + skip;
            const float c = pairCost(i - 1, j - 1);
            if (c < 2.0f * skip)
                best = std::min(best, at(i - 1, j - 1) + c);
            at(i, j) = best;
        }
    size_t i = n, j = m;
    while (i > 0 && j > 0)
    {
        const float here = at(i, j);
        const float c = pairCost(i - 1, j - 1);
        if (c < 2.0f * skip && std::abs(here - (at(i - 1, j - 1) + c)) < 1.0e-5f)
        {
            out.push_back({static_cast<int>(i - 1), static_cast<int>(j - 1)});
            --i;
            --j;
        }
        else if (std::abs(here - (at(i - 1, j) + skip)) < 1.0e-5f)
            --i;
        else
            --j;
    }
    std::reverse(out.begin(), out.end());
    return out;
}

long SpectralTable::analysisCount() noexcept
{
    return gAnalyses.load(std::memory_order_relaxed);
}

std::unique_ptr<SpectralTable> SpectralTable::analyse(int numFrames, const std::function<const float*(int)>& frame)
{
    gAnalyses.fetch_add(1, std::memory_order_relaxed);
    auto t = std::make_unique<SpectralTable>();
    t->serial = gSerial.fetch_add(1, std::memory_order_relaxed) + 1;
    t->count = std::clamp(numFrames, 0, kMaxFrames);
    t->data.assign(static_cast<size_t>(t->count) * 2 * kBins, 0.0f);
    t->energies.assign(static_cast<size_t>(t->count), 0.0f);
    t->peakList.resize(static_cast<size_t>(t->count));
    t->matchList.resize(static_cast<size_t>(std::max(t->count - 1, 0)));
    Fft fft(11);
    std::vector<float> work(2 * kCycle);
    std::array<float, kBins> mag{};
    for (int f = 0; f < t->count; ++f)
    {
        const float* x = frame(f);
        std::fill(work.begin(), work.end(), 0.0f);
        std::copy(x, x + kCycle, work.begin());
        fft.forward(work.data());
        float* re = t->data.data() + static_cast<size_t>(f) * 2 * kBins;
        float* im = re + kBins;
        double e = 0.0;
        const float norm = 2.0f / static_cast<float>(kCycle);
        for (int k = 1; k < kBins; ++k)
        {
            re[k] = work[static_cast<size_t>(2 * k)] * norm;
            im[k] = work[static_cast<size_t>(2 * k + 1)] * norm;
            mag[static_cast<size_t>(k)] = std::sqrt(re[k] * re[k] + im[k] * im[k]);
            e += static_cast<double>(mag[static_cast<size_t>(k)]) * mag[static_cast<size_t>(k)];
        }
        mag[0] = 0.0f;
        t->energies[static_cast<size_t>(f)] = static_cast<float>(e);
        t->peakList[static_cast<size_t>(f)] = findPeaks(mag.data());
    }
    for (int f = 0; f + 1 < t->count; ++f)
        t->matchList[static_cast<size_t>(f)] =
            matchPeaks(t->peakList[static_cast<size_t>(f)], t->peakList[static_cast<size_t>(f) + 1]);
    return t;
}

// ---------------------------------------------------------------------------------------------------------------------
struct SpectralSynth::Impl
{
    std::array<std::unique_ptr<Fft>, 6> fft; // sizes 128 .. 4096
    std::vector<float> work = std::vector<float>(2 * kMaxSynthSize);
    Spectrum a, b, c;
    std::array<float, kMaxPeaks + 2> posA{}, posB{}, target{};
    std::array<float, kBins> magnitude{};
    std::array<float, kBins + 1> prefix{};
    Impl()
    {
        for (size_t i = 0; i < fft.size(); ++i)
            fft[i] = std::make_unique<Fft>(7 + static_cast<int>(i));
    }
};

SpectralSynth::SpectralSynth() : impl(std::make_unique<Impl>()) {}
SpectralSynth::~SpectralSynth() = default;

namespace
{
double spectrumEnergy(const Spectrum& s)
{
    double e = 0.0;
    for (int k = 1; k < kBins; ++k)
        e += static_cast<double>(s.re[static_cast<size_t>(k)]) * s.re[static_cast<size_t>(k)] +
             static_cast<double>(s.im[static_cast<size_t>(k)]) * s.im[static_cast<size_t>(k)];
    return e;
}
} // namespace

void SpectralSynth::interpolate(const SpectralTable& table, int frame, float mix, Interp interp, Spectrum& out)
{
    const int frames = table.frames();
    if (frames <= 0)
    {
        out = Spectrum{};
        return;
    }
    const int f0 = std::clamp(frame, 0, std::max(frames - 2, 0));
    const int f1 = std::min(f0 + 1, frames - 1);
    const float x = std::clamp(mix, 0.0f, 1.0f);
    const float *ar = table.re(f0), *ai = table.im(f0), *br = table.re(f1), *bi = table.im(f1);
    out.re[0] = out.im[0] = 0.0f;
    static const std::vector<Match> none;
    const auto& matches = f1 > f0 ? table.matches(f0) : none;
    if (interp == Interp::Crossfade || f1 == f0 || matches.empty())
    {
        for (int k = 1; k < kBins; ++k)
        {
            out.re[static_cast<size_t>(k)] = ar[k] + (br[k] - ar[k]) * x;
            out.im[static_cast<size_t>(k)] = ai[k] + (bi[k] - ai[k]) * x;
        }
        return;
    }

    // Anchors: both axes meet where the matched peaks are at this mix.
    auto& m = *impl;
    const auto& pa = table.peaks(f0);
    const auto& pb = table.peaks(f1);
    int n = 0;
    m.posA[0] = m.posB[0] = m.target[0] = 0.0f;
    n = 1;
    for (const auto& pair : matches)
    {
        const float a = pa[static_cast<size_t>(pair.a)].position, b = pb[static_cast<size_t>(pair.b)].position;
        const float t = a + (b - a) * x;
        if (t <= m.target[static_cast<size_t>(n - 1)] + 1.0e-3f || a <= m.posA[static_cast<size_t>(n - 1)] + 1.0e-3f ||
            b <= m.posB[static_cast<size_t>(n - 1)] + 1.0e-3f)
            continue;
        m.posA[static_cast<size_t>(n)] = a;
        m.posB[static_cast<size_t>(n)] = b;
        m.target[static_cast<size_t>(n)] = t;
        ++n;
    }
    m.posA[static_cast<size_t>(n)] = m.posB[static_cast<size_t>(n)] = m.target[static_cast<size_t>(n)] =
        static_cast<float>(kHarmonics);
    ++n;
    if (m.target[static_cast<size_t>(n - 2)] >= static_cast<float>(kHarmonics) - 1.0e-3f)
    {
        // The last matched peak sits at the top of the band: no room for the end anchor, plain crossfade.
        for (int k = 1; k < kBins; ++k)
        {
            out.re[static_cast<size_t>(k)] = ar[k] + (br[k] - ar[k]) * x;
            out.im[static_cast<size_t>(k)] = ai[k] + (bi[k] - ai[k]) * x;
        }
        return;
    }

    int segA = 0, segB = 0;
    double outEnergy = 0.0;
    for (int k = 1; k < kBins; ++k)
    {
        const float kk = static_cast<float>(k);
        const float sa = mapPosition(m.posA.data(), m.target.data(), n, segA, kk);
        const float sb = mapPosition(m.posB.data(), m.target.data(), n, segB, kk);
        float cr, ci, dr, di;
        sampleAt(ar, ai, sa, cr, ci);
        sampleAt(br, bi, sb, dr, di);
        const float magA = std::sqrt(cr * cr + ci * ci), magB = std::sqrt(dr * dr + di * di);
        const float mag = (1.0f - x) * magA + x * magB;
        float zr = (1.0f - x) * cr + x * dr, zi = (1.0f - x) * ci + x * di;
        float zm = std::sqrt(zr * zr + zi * zi);
        if (zm < 1.0e-6f * (magA + magB) || zm < 1.0e-30f)
        {
            // Opposite phases: keep the direction of the stronger source.
            zr = (1.0f - x) * magA >= x * magB ? cr : dr;
            zi = (1.0f - x) * magA >= x * magB ? ci : di;
            zm = std::sqrt(zr * zr + zi * zi);
        }
        const float g = zm > 1.0e-30f ? mag / zm : 0.0f;
        out.re[static_cast<size_t>(k)] = zr * g;
        out.im[static_cast<size_t>(k)] = zi * g;
        outEnergy += static_cast<double>(mag) * mag;
    }
    // Level: the energy of the interpolation of the two frame energies.
    const double target = static_cast<double>(1.0f - x) * table.energy(f0) + static_cast<double>(x) * table.energy(f1);
    if (outEnergy > 1.0e-20 && target > 0.0)
    {
        const float gain = std::min(static_cast<float>(std::sqrt(target / outEnergy)), 4.0f);
        for (int k = 1; k < kBins; ++k)
        {
            out.re[static_cast<size_t>(k)] *= gain;
            out.im[static_cast<size_t>(k)] *= gain;
        }
    }
}

void SpectralSynth::warp(const WarpSettings& settings, const Spectrum& in, Spectrum& out)
{
    out.re[0] = out.im[0] = 0.0f;
    const float amt = std::clamp(settings.amount, -1.0f, 1.0f);
    if (settings.mode == Warp::Off || settings.mode >= Warp::Count || amt == 0.0f)
    {
        out = in;
        out.re[0] = out.im[0] = 0.0f;
        return;
    }
    auto& m = *impl;
    switch (settings.mode)
    {
    case Warp::Shift:
    {
        const float s = amt * 32.0f;
        for (int k = 1; k < kBins; ++k)
        {
            const float p = static_cast<float>(k) - s;
            if (p < 1.0f)
                out.re[static_cast<size_t>(k)] = out.im[static_cast<size_t>(k)] = 0.0f;
            else
                sampleAt(in.re.data(), in.im.data(), p, out.re[static_cast<size_t>(k)], out.im[static_cast<size_t>(k)]);
        }
        break;
    }
    case Warp::Stretch:
    {
        const float r = std::exp2(amt);
        for (int k = 1; k < kBins; ++k)
        {
            const float p = static_cast<float>(k) / r;
            if (p < 1.0f)
                out.re[static_cast<size_t>(k)] = out.im[static_cast<size_t>(k)] = 0.0f;
            else
                sampleAt(in.re.data(), in.im.data(), p, out.re[static_cast<size_t>(k)], out.im[static_cast<size_t>(k)]);
        }
        break;
    }
    case Warp::Blur:
    {
        const int w = static_cast<int>(std::lround(std::abs(amt) * 16.0f));
        m.prefix[0] = 0.0f;
        for (int k = 0; k < kBins; ++k)
        {
            m.magnitude[static_cast<size_t>(k)] = std::sqrt(in.re[static_cast<size_t>(k)] * in.re[static_cast<size_t>(k)] +
                                                            in.im[static_cast<size_t>(k)] * in.im[static_cast<size_t>(k)]);
            m.prefix[static_cast<size_t>(k) + 1] = m.prefix[static_cast<size_t>(k)] + m.magnitude[static_cast<size_t>(k)];
        }
        for (int k = 1; k < kBins; ++k)
        {
            const int lo = std::max(1, k - w), hi = std::min(kBins - 1, k + w);
            const float avg = (m.prefix[static_cast<size_t>(hi) + 1] - m.prefix[static_cast<size_t>(lo)]) /
                              static_cast<float>(hi - lo + 1);
            const float mg = m.magnitude[static_cast<size_t>(k)];
            const float g = mg > 1.0e-20f ? avg / mg : 0.0f;
            out.re[static_cast<size_t>(k)] = in.re[static_cast<size_t>(k)] * g;
            out.im[static_cast<size_t>(k)] = in.im[static_cast<size_t>(k)] * g;
            if (mg <= 1.0e-20f)
            {
                out.re[static_cast<size_t>(k)] = avg;
                out.im[static_cast<size_t>(k)] = 0.0f;
            }
        }
        break;
    }
    case Warp::Comb:
    {
        const float period = 2.0f + 14.0f * std::abs(amt);
        for (int k = 1; k < kBins; ++k)
        {
            const float notch = 0.5f - 0.5f * std::cos(2.0f * kPi * static_cast<float>(k) / period);
            const float g = amt > 0.0f ? 1.0f - 0.95f * notch : 1.0f + 2.0f * notch;
            out.re[static_cast<size_t>(k)] = in.re[static_cast<size_t>(k)] * g;
            out.im[static_cast<size_t>(k)] = in.im[static_cast<size_t>(k)] * g;
        }
        break;
    }
    case Warp::Dispersion:
    {
        for (int k = 1; k < kBins; ++k)
        {
            const float ph = amt * kPi * static_cast<float>(k) * static_cast<float>(k) / 256.0f;
            const float c = std::cos(ph), s = std::sin(ph);
            out.re[static_cast<size_t>(k)] = in.re[static_cast<size_t>(k)] * c - in.im[static_cast<size_t>(k)] * s;
            out.im[static_cast<size_t>(k)] = in.re[static_cast<size_t>(k)] * s + in.im[static_cast<size_t>(k)] * c;
        }
        break;
    }
    case Warp::Tilt:
    {
        for (int k = 1; k < kBins; ++k)
        {
            const float g = std::pow(static_cast<float>(k), 2.0f * amt);
            out.re[static_cast<size_t>(k)] = in.re[static_cast<size_t>(k)] * g;
            out.im[static_cast<size_t>(k)] = in.im[static_cast<size_t>(k)] * g;
        }
        break;
    }
    default:
        out = in;
        return;
    }
    // Level-matched: the warp changes the colour, not the loudness.
    const double ein = spectrumEnergy(in), eout = spectrumEnergy(out);
    if (eout > 1.0e-20 && ein > 0.0)
    {
        const float g = std::min(static_cast<float>(std::sqrt(ein / eout)), 8.0f);
        for (int k = 1; k < kBins; ++k)
        {
            out.re[static_cast<size_t>(k)] *= g;
            out.im[static_cast<size_t>(k)] *= g;
        }
    }
}

bool SpectralSynth::synthesise(const Spectrum& spectrum, int harmonics, int size, float* out)
{
    if (size < kMinSynthSize || size > kMaxSynthSize || !std::has_single_bit(static_cast<unsigned>(size)))
        return false;
    auto& m = *impl;
    const int order = std::countr_zero(static_cast<unsigned>(size));
    float* w = m.work.data();
    std::fill(w, w + 2 * size, 0.0f);
    const int top = std::min({harmonics, size / 2 - 1, kHarmonics});
    const float scale = static_cast<float>(size) * 0.5f;
    for (int k = 1; k <= top; ++k)
    {
        w[2 * k] = spectrum.re[static_cast<size_t>(k)] * scale;
        w[2 * k + 1] = spectrum.im[static_cast<size_t>(k)] * scale;
    }
    m.fft[static_cast<size_t>(order - 7)]->inverse(w);
    std::copy(w, w + size, out);
    out[-2] = out[size - 2];
    out[-1] = out[size - 1];
    out[size] = out[0];
    out[size + 1] = out[1];
    out[size + 2] = out[2];
    return true;
}

bool SpectralSynth::render(const SpectralTable& table, int frame, float mix, Interp interp, const WarpSettings& warpSettings,
                           int harmonics, int size, float* out)
{
    if (size < kMinSynthSize || size > kMaxSynthSize || !std::has_single_bit(static_cast<unsigned>(size)))
        return false;
    auto& m = *impl;
    interpolate(table, frame, mix, interp, m.a);
    if (warpSettings.mode != Warp::Off && warpSettings.amount != 0.0f)
    {
        warp(warpSettings, m.a, m.b);
        return synthesise(m.b, harmonics, size, out);
    }
    return synthesise(m.a, harmonics, size, out);
}

// ---------------------------------------------------------------------------------------------------------------------
FactoryTables& FactoryTables::instance()
{
    static FactoryTables t;
    return t;
}

void FactoryTables::ensure(int table)
{
    if (table < 0 || table >= WavetableBank::numTables)
        return;
    std::lock_guard<std::mutex> lock(mutex);
    if (tables[static_cast<size_t>(table)].load(std::memory_order_acquire) != nullptr)
        return;
    const auto& bank = WavetableBank::instance();
    bank.ensure(table);
    auto built = SpectralTable::analyse(WavetableBank::numFrames, [&](int f) { return bank.frame(table, 0, f); });
    tables[static_cast<size_t>(table)].store(built.get(), std::memory_order_release);
    owned.push_back(std::move(built));
}

const SpectralTable* FactoryTables::get(int table) const noexcept
{
    if (table < 0 || table >= WavetableBank::numTables)
        return nullptr;
    return tables[static_cast<size_t>(table)].load(std::memory_order_acquire);
}

} // namespace crucible::spectral
