// SPDX-License-Identifier: MIT
// Feedback delay network, the shared reverb building blocks and the complete algorithmic reverb (include/crucible/fx/Fdn.h).
// prepare(), configure() and the designs run on the control thread; update() and process() never allocate.

#include <crucible/fx/Fdn.h>
#include <crucible/fx/Smoothing.h>

#include <algorithm>
#include <cmath>
#include <complex>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr float kTiny = 1.0e-18f;          // keeps recursive paths out of the denormal range; -360 dBFS
constexpr float kExcursionRatio = 0.016f;  // line length excursion at modulation depth 1, as a part of the line's own length
constexpr float kExcursionLimit = 0.0015f; // and its largest value in seconds
constexpr float kRt60Log = 6.907755279f;   // ln(1000): -60 dB

double besselI0(double x)
{
    double sum = 1.0, term = 1.0;
    for (int k = 1; k < 60; ++k)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
    }
    return sum;
}

bool isPrime(int n)
{
    if (n < 2)
        return false;
    for (int d = 2; d * d <= n; ++d)
        if (n % d == 0)
            return false;
    return true;
}

int nextPrime(int n)
{
    while (!isPrime(n))
        ++n;
    return n;
}

std::uint32_t nextRandom(std::uint32_t& state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float uniform01(std::uint32_t& state)
{
    return static_cast<float>(nextRandom(state) >> 8) * (1.0f / 16777216.0f);
}
} // namespace

const char* reverbAlgorithmName(ReverbAlgorithm a)
{
    switch (a)
    {
    case ReverbAlgorithm::Room:
        return "Room";
    case ReverbAlgorithm::Hall:
        return "Hall";
    case ReverbAlgorithm::Chamber:
        return "Chamber";
    case ReverbAlgorithm::Plate:
        return "Plate";
    case ReverbAlgorithm::Ambience:
        return "Ambience";
    default:
        return "?";
    }
}

// ---------------------------------------------------------------------------------------------------------------
const float* sincKernel()
{
    static const std::vector<float> table = []
    {
        const double beta = 6.5;
        const double norm = besselI0(beta);
        std::vector<double> h(static_cast<std::size_t>(kSincPhases * kSincTaps));
        for (int p = 0; p < kSincPhases; ++p)
        {
            const double f = static_cast<double>(p) / kSincPhases;
            double sum = 0.0;
            for (int k = 0; k < kSincTaps; ++k)
            {
                const double t = static_cast<double>(k - 3) - f;
                const double x = std::abs(t) < 1.0e-12 ? 1.0 : std::sin(kPi * t) / (kPi * t);
                const double r = t / 4.0;
                const double w = std::abs(r) >= 1.0 ? 0.0 : besselI0(beta * std::sqrt(1.0 - r * r)) / norm;
                h[static_cast<std::size_t>(p * kSincTaps + k)] = x * w;
                sum += x * w;
            }
            for (int k = 0; k < kSincTaps; ++k)
                h[static_cast<std::size_t>(p * kSincTaps + k)] /= sum;
        }
        // Largest magnitude of any phase over frequency; scale the whole table so that it is at most 1.
        double peak = 1.0;
        for (int p = 0; p < kSincPhases; p += 4)
            for (int m = 0; m <= 128; ++m)
            {
                const double w = kPi * m / 128.0;
                double re = 0.0, im = 0.0;
                for (int k = 0; k < kSincTaps; ++k)
                {
                    re += h[static_cast<std::size_t>(p * kSincTaps + k)] * std::cos(w * k);
                    im -= h[static_cast<std::size_t>(p * kSincTaps + k)] * std::sin(w * k);
                }
                peak = std::max(peak, std::hypot(re, im));
            }
        std::vector<float> out(h.size());
        for (std::size_t i = 0; i < h.size(); ++i)
            out[i] = static_cast<float>(h[i] / peak);
        return out;
    }();
    return table.data();
}

// ---------------------------------------------------------------------------------------------------------------
bool SpanClearer::step(int& budget)
{
    while (item < spans.size() && budget > 0)
    {
        ClearSpan& s = spans[item];
        const std::size_t n = std::min(static_cast<std::size_t>(budget), s.size - offset);
        std::fill_n(s.data + offset, n, 0.0f);
        offset += n;
        budget -= static_cast<int>(n);
        if (offset >= s.size)
        {
            ++item;
            offset = 0;
        }
    }
    return item >= spans.size();
}

void SpanClearer::clearAll()
{
    for (auto& s : spans)
        std::fill_n(s.data, s.size, 0.0f);
    item = spans.size();
    offset = 0;
}

std::size_t SpanClearer::totalSamples() const
{
    std::size_t n = 0;
    for (const auto& s : spans)
        n += s.size;
    return n;
}

// ---------------------------------------------------------------------------------------------------------------
void DelayBuffer::allocate(int minimumSamples)
{
    int size = 16;
    while (size < minimumSamples + 16)
        size <<= 1;
    data.assign(static_cast<std::size_t>(size + kGuard), 0.0f);
    mask = size - 1;
    pos = 0;
}

void DelayBuffer::clear()
{
    std::fill(data.begin(), data.end(), 0.0f);
    pos = 0;
}

// ---------------------------------------------------------------------------------------------------------------
void AllpassChain::allocate(int stages, int maxLength)
{
    active = std::min(stages, kMaxStages);
    for (int k = 0; k < kMaxStages; ++k)
    {
        buffers[static_cast<std::size_t>(k)].assign(static_cast<std::size_t>(k < active ? maxLength : 2), 0.0f);
        length[static_cast<std::size_t>(k)] = std::max(2, k < active ? maxLength : 2);
    }
    position.fill(0);
    gains.fill(0.5f);
}

void AllpassChain::setup(const int* lengths, const float* g, int stages)
{
    active = std::min(stages, kMaxStages);
    for (int k = 0; k < active; ++k)
    {
        const auto i = static_cast<std::size_t>(k);
        length[i] = std::clamp(lengths[k], 2, static_cast<int>(buffers[i].size()));
        gains[i] = g[k];
        position[i] = 0;
    }
}

void AllpassChain::setGains(const float* g, int stages, int n)
{
    const int count = std::min(stages, active);
    if (remaining > 0)
        gains = target; // the previous glide is cut short (the chunk length changed)
    remaining = 0;
    bool moves = false;
    for (int k = 0; k < count; ++k)
        moves = moves || g[k] != gains[static_cast<std::size_t>(k)];
    if (!moves)
    {
        gliding = n > 0;
        return;
    }
    if (n <= 0 || !gliding)
    {
        for (int k = 0; k < count; ++k)
            gains[static_cast<std::size_t>(k)] = g[k];
        gliding = n > 0;
        return;
    }
    target = gains;
    step.fill(0.0f);
    for (int k = 0; k < count; ++k)
    {
        const auto i = static_cast<std::size_t>(k);
        target[i] = g[k];
        step[i] = (g[k] - gains[i]) / static_cast<float>(n);
    }
    remaining = n;
    gliding = true;
}

void AllpassChain::clear()
{
    for (auto& b : buffers)
        std::fill(b.begin(), b.end(), 0.0f);
    position.fill(0);
}

void AllpassChain::addSpans(std::vector<ClearSpan>& out)
{
    for (auto& b : buffers)
        out.push_back({b.data(), b.size()});
}

// ---------------------------------------------------------------------------------------------------------------
AbsorptionCoefficients designAbsorption(float u, const DecayTarget& t, float klow, float khigh)
{
    const float lm = -kRt60Log / t.decay;
    const float ll = -kRt60Log / (t.decay * t.lowMult);
    const float lh = -kRt60Log / (t.decay * t.highMult);
    float gm = std::exp(lm * u);
    const float a = std::exp((ll - lm) * u); // low shelf gain: DC against the middle
    const float b = std::exp((lh - lm) * u); // high shelf gain: Nyquist against the middle
    // Overlapping shelves that both boost could lift the middle above one: keep the loop strictly lossy.
    const float peak = gm * std::max(1.0f, a) * std::max(1.0f, b);
    if (peak > 0.9999f)
        gm *= 0.9999f / peak;
    const float sa = std::exp(0.5f * (ll - lm) * u);
    const float sb = std::exp(0.5f * (lh - lm) * u);
    AbsorptionCoefficients c;
    {
        const float kz = klow * sa, kp = klow / sa;
        const float a0 = 1.0f / (kp + 1.0f);
        c.lb0 = gm * (kz + 1.0f) * a0;
        c.lb1 = gm * (kz - 1.0f) * a0;
        c.la1 = (kp - 1.0f) * a0;
    }
    {
        const float pk = sb * khigh;
        const float a0 = 1.0f / (1.0f + pk);
        c.hb0 = (b + pk) * a0;
        c.hb1 = (pk - b) * a0;
        c.ha1 = (pk - 1.0f) * a0;
    }
    return c;
}

void absorptionCrossovers(const DecayTarget& t, double sampleRate, float& klow, float& khigh)
{
    const float nyquist = static_cast<float>(sampleRate) * 0.5f;
    const float lowHz = std::clamp(t.lowHz, 20.0f, nyquist * 0.45f);
    const float highHz = std::clamp(t.highHz, lowHz * 1.5f, nyquist * 0.92f);
    klow = std::tan(static_cast<float>(kPi) * lowHz / static_cast<float>(sampleRate));
    khigh = std::tan(static_cast<float>(kPi) * highHz / static_cast<float>(sampleRate));
}

float absorptionDecayAt(float hz, double sampleRate, float delaySeconds, const DecayTarget& target)
{
    float klow, khigh;
    absorptionCrossovers(target, sampleRate, klow, khigh);
    const AbsorptionCoefficients c = designAbsorption(delaySeconds, target, klow, khigh);
    const double w = 2.0 * kPi * static_cast<double>(hz) / sampleRate;
    const std::complex<double> z1 = std::polar(1.0, -w);
    const std::complex<double> low =
        (static_cast<double>(c.lb0) + static_cast<double>(c.lb1) * z1) / (1.0 + static_cast<double>(c.la1) * z1);
    const std::complex<double> high =
        (static_cast<double>(c.hb0) + static_cast<double>(c.hb1) * z1) / (1.0 + static_cast<double>(c.ha1) * z1);
    const double magnitude = std::abs(low) * std::abs(high);
    return static_cast<float>(-6.907755279 * static_cast<double>(delaySeconds) / std::log(magnitude));
}

// ---------------------------------------------------------------------------------------------------------------
void Fdn::prepare(double sampleRate, int maxLinesWanted, double maxDelaySeconds)
{
    sr = sampleRate;
    maxLines = std::clamp(maxLinesWanted, 8, kFdnMaxLines);
    int cap = 64;
    while (cap < static_cast<int>(maxDelaySeconds * sampleRate) + 64)
        cap <<= 1;
    mask = cap - 1;
    stride = cap + DelayBuffer::kGuard;
    storage.assign(static_cast<std::size_t>(maxLines) * static_cast<std::size_t>(stride), 0.0f);
    (void)sincKernel();
    Profile p;
    p.lines = maxLines;
    configure(p);
}

void Fdn::addSpans(std::vector<ClearSpan>& out)
{
    out.push_back({storage.data(), storage.size()});
}

void Fdn::clear()
{
    std::fill(storage.begin(), storage.end(), 0.0f);
    resetState();
}

void Fdn::resetState()
{
    lb0.fill(1.0f);
    lb1.fill(0.0f);
    la1.fill(0.0f);
    hb0.fill(1.0f);
    hb1.fill(0.0f);
    ha1.fill(0.0f);
    lz.fill(0.0f);
    hz.fill(0.0f);
    coefMoving = false;
    writePos = 0;
    fresh = true;
    guard.reset();
    feedbackScale = 1.0f;
    energyAccum = 0.0f;
    lastScale = -1.0f;
    lastFreeze = -1;
    freezeState = 0;
    std::uint32_t s = profileSeed * 2654435761u + 12345u;
    for (int i = 0; i < kFdnMaxLines; ++i)
    {
        const auto k = static_cast<std::size_t>(i);
        noise[k].seed(s + static_cast<std::uint32_t>(i) * 97u + 1u);
        noiseSpeed[k] = 0.6f + 0.8f * uniform01(s);
    }
}

void Fdn::configure(const Profile& profile)
{
    active = std::clamp(profile.lines, 8, maxLines);
    householderMix = profile.householder;
    profileSeed = profile.seed;
    std::uint32_t rng = profile.seed * 747796405u + 2891336453u;
    // Geometrically spaced lengths with a little jitter, snapped to primes at 48 kHz, then shuffled.
    std::array<int, kFdnMaxLines> primes{};
    int previous = 0;
    for (int i = 0; i < active; ++i)
    {
        const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(active);
        float ms = profile.minMs * std::pow(profile.maxMs / profile.minMs, t);
        ms *= 1.0f + 0.05f * (uniform01(rng) - 0.5f);
        int p = nextPrime(static_cast<int>(ms * 48.0f + 0.5f));
        if (p <= previous)
            p = nextPrime(previous + 1);
        primes[static_cast<std::size_t>(i)] = p;
        previous = p;
    }
    for (int i = active - 1; i > 0; --i)
        std::swap(primes[static_cast<std::size_t>(i)],
                  primes[static_cast<std::size_t>(nextRandom(rng) % static_cast<std::uint32_t>(i + 1))]);
    double total = 0.0;
    for (int i = 0; i < active; ++i)
    {
        baseSamples[static_cast<std::size_t>(i)] = static_cast<float>(primes[static_cast<std::size_t>(i)] * sr / 48000.0);
        total += baseSamples[static_cast<std::size_t>(i)];
    }
    meanSeconds = total / active / sr;
    designVectors(profile.seed);
    resetState();
}

void Fdn::designVectors(std::uint32_t seed)
{
    const int n = active;
    std::uint32_t rng = seed * 2246822519u + 3266489917u;
    const auto walshPeak = [n](const std::array<float, kFdnMaxLines>& v)
    {
        std::array<float, kFdnMaxLines> w = v;
        for (int len = 1; len < n; len <<= 1)
            for (int i = 0; i < n; i += len << 1)
                for (int j = i; j < i + len; ++j)
                {
                    const float a = w[static_cast<std::size_t>(j)], b = w[static_cast<std::size_t>(j + len)];
                    w[static_cast<std::size_t>(j)] = a + b;
                    w[static_cast<std::size_t>(j + len)] = a - b;
                }
        float peak = 0.0f;
        for (int i = 0; i < n; ++i)
            peak = std::max(peak, std::abs(w[static_cast<std::size_t>(i)]));
        return peak / static_cast<float>(n);
    };
    // A random sign vector that is not close to a row of the Hadamard matrix (it would then fall on one line after the
    // first mixing), and a second one orthogonal to it (half of the signs flipped).
    const auto signs = [&](std::array<float, kFdnMaxLines>& a, std::array<float, kFdnMaxLines>& b)
    {
        for (int tries = 0; tries < 200; ++tries)
        {
            for (int i = 0; i < n; ++i)
                a[static_cast<std::size_t>(i)] = (nextRandom(rng) & 0x100u) != 0 ? 1.0f : -1.0f;
            if (walshPeak(a) < 0.55f)
                break;
        }
        std::array<float, kFdnMaxLines> flip{};
        for (int i = 0; i < n; ++i)
            flip[static_cast<std::size_t>(i)] = i < n / 2 ? 1.0f : -1.0f;
        for (int i = n - 1; i > 0; --i)
            std::swap(flip[static_cast<std::size_t>(i)],
                      flip[static_cast<std::size_t>(nextRandom(rng) % static_cast<std::uint32_t>(i + 1))]);
        for (int i = 0; i < n; ++i)
            b[static_cast<std::size_t>(i)] = a[static_cast<std::size_t>(i)] * flip[static_cast<std::size_t>(i)];
    };
    std::array<float, kFdnMaxLines> a{}, b{}, c{}, d{};
    signs(a, b);
    signs(c, d);
    const float norm = 1.0f / std::sqrt(static_cast<float>(n));
    for (int i = 0; i < n; ++i)
    {
        const auto k = static_cast<std::size_t>(i);
        inGainL[k] = a[k] * norm * 0.70710678f;
        inGainR[k] = b[k] * norm * 0.70710678f;
        outGainL[k] = c[k] * norm;
        outGainR[k] = d[k] * norm;
    }
}

void Fdn::absorptionFor(const Controls& c)
{
    absorptionCrossovers(c.decay, sr, klow, khigh);
    DecayTarget t = c.decay;
    t.decay = std::max(t.decay, 0.02f);
    t.lowMult = std::clamp(t.lowMult, 0.02f, 8.0f);
    t.highMult = std::clamp(t.highMult, 0.02f, 8.0f);
    const float paramDecay = t.decay;
    if (c.freeze != 0)
    {
        t.decay = 1.0e5f;
        t.lowMult = t.highMult = 1.0f;
    }
    float loss = 0.0f;
    for (int i = 0; i < active; ++i)
    {
        const auto k = static_cast<std::size_t>(i);
        const float u = baseSamples[k] * c.scale / static_cast<float>(sr);
        const AbsorptionCoefficients a = designAbsorption(u, t, klow, khigh);
        coefTarget[0][k] = a.lb0;
        coefTarget[1][k] = a.lb1;
        coefTarget[2][k] = a.la1;
        coefTarget[3][k] = a.hb0;
        coefTarget[4][k] = a.hb1;
        coefTarget[5][k] = a.ha1;
        const float g2 = std::exp(-2.0f * kRt60Log * u / paramDecay);
        loss += 1.0f - g2;
    }
    float* const cur[6] = {lb0.data(), lb1.data(), la1.data(), hb0.data(), hb1.data(), ha1.data()};
    const float inv = 1.0f / static_cast<float>(std::max(1, chunkSamples));
    for (int j = 0; j < 6; ++j)
        for (int i = 0; i < active; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            if (fresh)
                cur[j][k] = coefTarget[static_cast<std::size_t>(j)][k];
            else
                coefStep[static_cast<std::size_t>(j)][k] = (coefTarget[static_cast<std::size_t>(j)][k] - cur[j][k]) * inv;
        }
    coefMoving = !fresh;
    // Energy of the impulse response is 1 / (N mean(1 - g^2)) for unit input and output vectors.
    gainNormal = std::sqrt(std::max(loss, 1.0e-6f));
}

void Fdn::update(const Controls& c, int n)
{
    chunkSamples = n;
    chunkSeconds = static_cast<float>(n) / static_cast<float>(sr);
    freezeState = c.freeze;
    if (c.freeze == 2)
    {
        if (energyAccum > 0.0f)
            feedbackScale = guard.update(energyAccum / static_cast<float>(2 * std::max(1, chunkSamples)), chunkSeconds);
    }
    else
    {
        guard.reset();
        feedbackScale = 1.0f;
    }
    energyAccum = 0.0f;
    fineRead = !fresh && (c.scale != lastScale || c.modDepth != lastModDepth);
    lastModDepth = c.modDepth;
    const bool changed = fresh || c.scale != lastScale || c.decay.decay != lastDecay.decay ||
                         c.decay.lowMult != lastDecay.lowMult || c.decay.highMult != lastDecay.highMult ||
                         c.decay.lowHz != lastDecay.lowHz || c.decay.highHz != lastDecay.highHz ||
                         (c.freeze != 0) != (lastFreeze != 0);
    if (changed)
    {
        absorptionFor(c);
        lastScale = c.scale;
        lastDecay = c.decay;
        lastFreeze = c.freeze;
    }
    lastFreeze = c.freeze;
    const float depth = std::clamp(c.modDepth, 0.0f, 1.0f);
    const float limit = kExcursionLimit * static_cast<float>(sr);
    const float rate = std::clamp(c.modRate, 0.0f, 20.0f);
    const float low = 8.0f, high = static_cast<float>(mask) - 16.0f;
    for (int i = 0; i < active; ++i)
    {
        const auto k = static_cast<std::size_t>(i);
        const float m = noise[k].next(rate * noiseSpeed[k] * chunkSeconds);
        const float excursion = std::min(limit, kExcursionRatio * baseSamples[k] * c.scale) * depth;
        const float target = std::clamp(baseSamples[k] * c.scale + excursion * m, low, high);
        const std::int64_t targetQ = static_cast<std::int64_t>(target * 65536.0f);
        if (fresh)
        {
            delayQ[k] = targetQ;
            stepQ[k] = 0;
        }
        else
            stepQ[k] = (targetQ - delayQ[k]) / std::max(1, n);
    }
    fresh = false;
}

void Fdn::process(const float* xl, const float* xr, float* ol, float* orr, int n)
{
    const float* ker = sincKernel();
    const int count = active;
    alignas(32) float y[kFdnMaxLines];
    alignas(32) float v[kFdnMaxLines];
    alignas(32) float m[kFdnMaxLines];
    const bool track = freezeState == 2;
    const float scale = householderMix ? feedbackScale : feedbackScale / std::sqrt(static_cast<float>(count));
    const float spread = 2.0f / static_cast<float>(count);
    const int cap = mask + 1;
    for (int s = 0; s < n; ++s)
    {
        for (int i = 0; i < count; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            delayQ[k] += stepQ[k];
            const std::int64_t p = (static_cast<std::int64_t>(writePos) << 16) - delayQ[k];
            if (fineRead)
            {
                y[i] = DelayBuffer::sincFine(ker, storage.data() + static_cast<std::size_t>(i) * static_cast<std::size_t>(stride),
                                             p, mask);
                continue;
            }
            const float* kr = ker + static_cast<std::size_t>((p >> 7) & (kSincPhases - 1)) * kSincTaps;
            const float* d = storage.data() + static_cast<std::size_t>(i) * static_cast<std::size_t>(stride) +
                             static_cast<std::size_t>((static_cast<int>(p >> 16) - 3) & mask);
            float sum = 0.0f;
            for (int j = 0; j < kSincTaps; ++j)
                sum += kr[j] * d[j];
            y[i] = sum;
        }
        float oL = 0.0f, oR = 0.0f;
        for (int i = 0; i < count; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            if (coefMoving)
            {
                lb0[k] += coefStep[0][k];
                lb1[k] += coefStep[1][k];
                la1[k] += coefStep[2][k];
                hb0[k] += coefStep[3][k];
                hb1[k] += coefStep[4][k];
                ha1[k] += coefStep[5][k];
            }
            const float t = lb0[k] * y[i] + lz[k];
            lz[k] = lb1[k] * y[i] - la1[k] * t;
            const float u = hb0[k] * t + hz[k];
            hz[k] = hb1[k] * t - ha1[k] * u;
            v[i] = u;
            oL += outGainL[k] * u;
            oR += outGainR[k] * u;
        }
        if (track)
            energyAccum += oL * oL + oR * oR;
        if (householderMix)
        {
            float sum = 0.0f;
            for (int i = 0; i < count; ++i)
                sum += v[i];
            const float h = sum * spread;
            for (int i = 0; i < count; ++i)
                m[i] = (v[i] - h) * scale;
        }
        else
        {
            for (int i = 0; i < count; ++i)
                m[i] = v[i];
            for (int len = 1; len < count; len <<= 1)
                for (int i = 0; i < count; i += len << 1)
                    for (int j = i; j < i + len; ++j)
                    {
                        const float a = m[j], b = m[j + len];
                        m[j] = a + b;
                        m[j + len] = a - b;
                    }
            for (int i = 0; i < count; ++i)
                m[i] *= scale;
        }
        const float a = xl[s], b = xr[s];
        const bool mirror = writePos < DelayBuffer::kGuard;
        for (int i = 0; i < count; ++i)
        {
            const auto k = static_cast<std::size_t>(i);
            const float x = m[i] + inGainL[k] * a + inGainR[k] * b + kTiny;
            float* line = storage.data() + static_cast<std::size_t>(i) * static_cast<std::size_t>(stride);
            line[writePos] = x;
            if (mirror)
                line[cap + writePos] = x;
        }
        writePos = (writePos + 1) & mask;
        ol[s] = oL;
        orr[s] = oR;
    }
    if (coefMoving)
    {
        float* const cur[6] = {lb0.data(), lb1.data(), la1.data(), hb0.data(), hb1.data(), ha1.data()};
        for (int j = 0; j < 6; ++j)
            for (int i = 0; i < count; ++i)
                cur[j][static_cast<std::size_t>(i)] = coefTarget[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)];
        coefMoving = false;
    }
}

// ---------------------------------------------------------------------------------------------------------------
namespace
{
struct AlgorithmProfile
{
    float minMs, maxMs;
    bool householder;
    int stages;
    int diffusion[8]; // all-pass lengths at 48 kHz, left channel
};

constexpr AlgorithmProfile kProfiles[] = {
    {6.0f, 42.0f, true, 6, {13, 23, 41, 71, 113, 181, 0, 0}},      // Room
    {9.0f, 135.0f, false, 8, {11, 19, 31, 53, 89, 151, 257, 419}}, // Hall
    {11.0f, 80.0f, false, 6, {13, 23, 43, 79, 139, 241, 0, 0}},    // Chamber
    {0.0f, 0.0f, false, 8, {17, 29, 47, 79, 131, 229, 373, 611}},  // Plate (own topology); paper input all-passes last
    {3.5f, 28.0f, true, 4, {13, 23, 37, 61, 0, 0, 0, 0}},          // Ambience
};
constexpr int kRightOffset[8] = {6, 10, 8, 14, 12, 18, 16, 22};
constexpr float kFadeSeconds = 0.020f;
// The plate's published lengths give a 0.73 s round trip, long against the usual decays (the energy then falls in steps);
// at the nominal size it runs at half those lengths.
constexpr float kPlateScale = 0.5f;
constexpr float kPlateRingSeconds = 0.7254f; // 21589 samples at 29761 Hz
constexpr float kPlateRingToDecay = 0.2f;
constexpr int kClearBudget = 65536; // samples of memory one deactivate() or chunk clears: 256 KiB
} // namespace

void AlgorithmicReverb::prepare(double sampleRate, int lines, std::uint32_t seedValue)
{
    sr = sampleRate;
    lineCount = lines >= 32 ? 32 : 16;
    seed = seedValue;
    fdn.prepare(sr, lineCount, 0.31);
    plate.prepare(sr, seed);
    er.prepare(sr, seed);
    const int diffusionMax = static_cast<int>(800.0 * sr / 48000.0) + 8;
    diffL.allocate(8, diffusionMax);
    diffR.allocate(8, diffusionMax);
    preL.allocate(static_cast<int>(0.52 * sr) + 64);
    preR.allocate(static_cast<int>(0.52 * sr) + 64);
    buildClearList();
    reset();
}

void AlgorithmicReverb::buildClearList()
{
    std::vector<ClearSpan> spans;
    fdn.addSpans(spans);
    plate.addSpans(spans);
    er.addSpans(spans);
    diffL.addSpans(spans);
    diffR.addSpans(spans);
    spans.push_back(preL.span());
    spans.push_back(preR.span());
    clearer.setSpans(std::move(spans));
}

void AlgorithmicReverb::reset()
{
    clearer.clearAll();
    finishClear();
    primed = false;
    smoothPrimed = false;
    configured = false;
    dirty = false;
}

void AlgorithmicReverb::finishClear()
{
    fdn.resetState();
    plate.resetState();
    diffL.rewind();
    diffR.rewind();
    hpState[0] = hpState[1] = lpState[0] = lpState[1] = 0.0f;
    envelope = 0.0f;
    duckGainNow = 1.0f;
    frozen = false;
    inputGain = 1.0f;
    freezeSettle = 0;
    phase = Phase::Running;
    fade = 1.0f;
    clearing = false;
    primed = false;
    configured = false;
}

void AlgorithmicReverb::deactivate()
{
    primed = false;
    if (dirty)
    {
        dirty = false;
        clearing = true;
        clearer.restart();
    }
    if (clearing)
    {
        int budget = kClearBudget;
        if (clearer.step(budget))
            finishClear();
    }
}

void AlgorithmicReverb::configureAlgorithm(ReverbAlgorithm a)
{
    current = a;
    configured = true;
    er.select(a);
    const AlgorithmProfile& p = kProfiles[static_cast<int>(a)];
    if (a != ReverbAlgorithm::Plate)
    {
        Fdn::Profile fp;
        fp.lines = lineCount;
        fp.minMs = p.minMs;
        fp.maxMs = p.maxMs;
        fp.householder = p.householder;
        fp.seed = seed * 31u + static_cast<std::uint32_t>(a) * 7919u + 1u;
        fdn.configure(fp);
    }
    int left[8], right[8];
    const float ratio = static_cast<float>(sr / 48000.0);
    for (int k = 0; k < p.stages; ++k)
    {
        left[k] = static_cast<int>(static_cast<float>(p.diffusion[k]) * ratio + 0.5f);
        right[k] = static_cast<int>(static_cast<float>(p.diffusion[k] + kRightOffset[k]) * ratio + 0.5f);
    }
    const float g[8] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
    diffL.setup(left, g, p.stages);
    diffR.setup(right, g, p.stages);
    diffusionStages = p.stages;
}

void AlgorithmicReverb::process(float* l, float* r, int n, const ReverbSettings& s)
{
    if (clearing)
    {
        // Switched on again before the spread-out clear finished: keep clearing a bounded amount per callback and pass
        // the dry input until the memory is clean, instead of zeroing it all in one callback.
        int budget = kClearBudget;
        if (!clearer.step(budget))
            return;
        finishClear();
    }
    dirty = true;
    const double slowest = static_cast<double>(std::max({1.0f, s.lowMult, s.highMult}));
    tail.store(static_cast<double>(s.predelayMs) * 0.001 + static_cast<double>(s.decaySeconds) * slowest,
               std::memory_order_relaxed);
    infinite.store(s.freeze, std::memory_order_relaxed);
    for (int pos = 0; pos < n; pos += kChunk)
    {
        const int len = std::min(kChunk, n - pos);
        processChunk(l + pos, r + pos, len, smoothSettings(s, len));
    }
}

namespace
{
constexpr float kHighPassOffHz = 0.002f; // coefficient about 1e-7: the integrator stays at zero, the filter is an identity
constexpr float kLowPassOffHz = 1.0e6f;  // beyond the clamp below: coefficient 1 - 2e-5
} // namespace

float AlgorithmicReverb::smoothValue(SmoothSlot slot, float target, bool logDomain, float k)
{
    Smooth3& m = smooth[slot];
    const float t = logDomain ? std::log(std::max(target, 1.0e-3f)) : target;
    if (!smoothPrimed)
        m.a = m.b = m.c = t;
    // Settled on the target: pass the raw value through (constant settings stay bit-identical, no exp / log round trip).
    const double eps = logDomain ? 2.0e-5 : 1.0e-6 * std::max(1.0, std::abs(static_cast<double>(t)));
    if (std::abs(m.a - t) <= eps && std::abs(m.b - t) <= eps && std::abs(m.c - t) <= eps)
    {
        m.a = m.b = m.c = t;
        return target;
    }
    smoothAllSettled = false;
    // Double state: at the slow rates (size, predelay) a float step stalls far from the target, and snapping from there
    // would be a step. The snap on "no progress" only fires within an ulp of the target.
    const auto step = [k](double v, double to)
    {
        const double next = v + static_cast<double>(k) * (to - v);
        return next == v ? to : next;
    };
    m.a = step(m.a, t);
    m.b = step(m.b, m.a);
    m.c = step(m.c, m.b);
    const float out = static_cast<float>(m.c);
    return logDomain ? std::exp(out) : out;
}

ReverbSettings AlgorithmicReverb::smoothSettings(const ReverbSettings& s, int n)
{
    // 8 ms per stage: about 25 ms in all.
    const float k = 1.0f - std::exp(-static_cast<float>(n) / (0.008f * static_cast<float>(sr)));
    ReverbSettings o = s;
    smoothAllSettled = true;
    o.mix = smoothValue(kSmMix, s.mix, false, k);
    o.width = smoothValue(kSmWidth, s.width, false, k);
    o.duck = smoothValue(kSmDuck, s.duck, false, k);
    o.early = smoothValue(kSmEarly, s.early, false, k);
    o.lowMult = smoothValue(kSmLowMult, s.lowMult, true, k);
    o.highMult = smoothValue(kSmHighMult, s.highMult, true, k);
    o.lowHz = smoothValue(kSmLowHz, s.lowHz, true, k);
    o.highHz = smoothValue(kSmHighHz, s.highHz, true, k);
    // Bypassed cutoffs (20 Hz or less / 20 kHz or more) glide to a virtual cutoff where the filter is an identity, so
    // that switching a cut on or off never steps.
    const float lowCut = s.lowCutHz > 20.0f ? s.lowCutHz : kHighPassOffHz;
    const float highCut = s.highCutHz < 19990.0f ? s.highCutHz : kLowPassOffHz;
    // The cut coefficients move steeply with the cutoff, so they glide more slowly (20 ms per stage).
    const float kCut = 1.0f - std::exp(-static_cast<float>(n) / (0.035f * static_cast<float>(sr)));
    o.lowCutHz = smoothValue(kSmLowCut, lowCut, true, kCut);
    o.highCutHz = smoothValue(kSmHighCut, highCut, true, kCut);
    o.size = smoothValue(kSmSize, s.size, false, k * 0.02f);
    o.predelayMs = smoothValue(kSmPredelay, s.predelayMs, true, k * 0.03f);
    o.density = smoothValue(kSmDensity, s.density, false, k * 0.25f);
    o.decaySeconds = smoothValue(kSmDecay, s.decaySeconds, true, k);
    o.modDepth = smoothValue(kSmModDepth, s.modDepth, false, k);
    smoothPrimed = true;
    return o;
}

void AlgorithmicReverb::processChunk(float* l, float* r, int n, const ReverbSettings& s)
{
    const float chunkSeconds = static_cast<float>(n) / static_cast<float>(sr);
    const float size = std::clamp(s.size, 0.0f, 1.0f);
    const float scaleTarget = 0.5f * std::pow(4.0f, size);
    const float maxPredelay = static_cast<float>(preL.capacity() - 2 * DelayBuffer::kGuard - 2);
    const float predelayTarget = std::clamp(s.predelayMs * 0.001f * static_cast<float>(sr), 5.0f, maxPredelay);
    const bool snap = !primed;
    if (snap)
    {
        mixNow = s.mix;
        scaleNow = scaleTarget;
        predelayNow = predelayTarget;
        widthNow = s.width;
        earlyNow = std::clamp(s.early, 0.0f, 1.0f);
        erGainNow = 0.0f;
        wetGainNow = 0.0f;
        primed = true;
        if (!configured)
            configureAlgorithm(s.algorithm);
        pending = current;
        phase = Phase::Running;
        fade = 1.0f;
    }

    // ---- algorithm change: fade out, clear, fade in
    pending = s.algorithm;
    if ((phase == Phase::Running || phase == Phase::FadingIn) && pending != current)
        phase = Phase::FadingOut;
    const float fadeStart = fade;
    const float fadeStep = static_cast<float>(n) / (kFadeSeconds * static_cast<float>(sr));
    bool skip = false;
    switch (phase)
    {
    case Phase::Running:
        fade = 1.0f;
        break;
    case Phase::FadingOut:
        fade -= fadeStep;
        if (fade <= 0.0f)
        {
            fade = 0.0f;
            phase = Phase::Clearing;
            clearer.restart();
        }
        break;
    case Phase::Clearing:
    {
        skip = true;
        int budget = kClearBudget;
        if (clearer.step(budget))
        {
            const ReverbAlgorithm target = pending;
            finishClear();
            primed = true;
            configureAlgorithm(target);
            phase = Phase::FadingIn;
            fade = 0.0f;
            inputGain = 0.0f; // the input enters the cleared network over 30 ms, not with a step
            mixNow = s.mix;
            scaleNow = scaleTarget;
            predelayNow = predelayTarget;
        }
        break;
    }
    case Phase::FadingIn:
        fade += fadeStep;
        if (fade >= 1.0f)
        {
            fade = 1.0f;
            phase = Phase::Running;
        }
        break;
    }
    const float fadeEnd = fade;
    float dryL[kChunk], dryR[kChunk];
    for (int i = 0; i < n; ++i)
    {
        dryL[i] = l[i];
        dryR[i] = r[i];
    }

    // ---- control values for this chunk
    const float follow = std::min(1.0f, chunkSeconds / 0.08f);
    scaleNow += (scaleTarget - scaleNow) * follow;
    const float predelayStart = predelayNow;
    predelayNow += (predelayTarget - predelayNow) * std::min(1.0f, chunkSeconds / 0.05f);
    const float predelayEnd = predelayNow;
    const float mixStart = mixNow;
    mixNow = s.mix;
    const float widthStart = widthNow;
    widthNow = s.width;

    // ---- freeze
    if (s.freeze && !frozen)
    {
        frozen = true;
        freezeSettle = 0;
    }
    else if (!s.freeze && frozen)
        frozen = false;
    int freezeState = 0;
    float gainTarget = 1.0f;
    if (frozen)
    {
        gainTarget = 0.0f;
        freezeSettle += n;
        freezeState = static_cast<double>(freezeSettle) > 0.1 * sr ? 2 : 1;
    }
    const float inputStart = inputGain;
    const float inputStep = chunkSeconds / 0.03f;
    inputGain =
        inputGain < gainTarget ? std::min(gainTarget, inputGain + inputStep) : std::max(gainTarget, inputGain - inputStep);
    const float inputEnd = inputGain;

    float wetL[kChunk], wetR[kChunk];
    const bool isPlate = current == ReverbAlgorithm::Plate;
    float wetGainTarget = wetGainNow, erGainTarget = 0.0f;
    if (skip)
    {
        for (int i = 0; i < n; ++i)
            wetL[i] = wetR[i] = 0.0f;
    }
    else
    {
        float pl[kChunk], pr[kChunk], mono[kChunk], lateL[kChunk], lateR[kChunk];
        const float inv = 1.0f / static_cast<float>(n);
        for (int i = 0; i < n; ++i)
        {
            preL.write(dryL[i]);
            preR.write(dryR[i]);
            const float t = static_cast<float>(i + 1) * inv;
            const float delay = predelayStart + (predelayEnd - predelayStart) * t + 1.0f;
            const float g = inputStart + (inputEnd - inputStart) * t;
            const std::int64_t q = DelayBuffer::toQ16(delay);
            pl[i] = g * (predelayStart != predelayEnd ? preL.readSincFine(q) : preL.readSinc(q));
            pr[i] = g * (predelayStart != predelayEnd ? preR.readSincFine(q) : preR.readSinc(q));
            mono[i] = 0.5f * (pl[i] + pr[i]);
        }
        DecayTarget decay;
        decay.decay = std::clamp(s.decaySeconds, 0.05f, 120.0f);
        decay.lowMult = s.lowMult;
        decay.highMult = s.highMult;
        decay.lowHz = s.lowHz;
        decay.highHz = s.highHz;
        // Input diffusion (both algorithm families): a chain of all-passes per channel. Its output also joins the wet signal
        // as the dense onset of the reverberation (before the first return of the network).
        const float dg = 0.30f + 0.50f * std::clamp(s.density, 0.0f, 1.0f);
        const float gains[8] = {dg, dg, dg, dg, dg * 0.9f, dg * 0.9f, dg * 0.9f, dg * 0.9f};
        diffL.setGains(gains, diffusionStages, n);
        diffR.setGains(gains, diffusionStages, n);
        float dl[kChunk], dr[kChunk];
        for (int i = 0; i < n; ++i)
        {
            dl[i] = diffL.process(pl[i] + 1.0e-18f);
            dr[i] = diffR.process(pr[i] + 1.0e-18f);
        }
        const float onset = 0.5f * std::clamp(s.density, 0.0f, 1.0f);
        if (snap)
            onsetNow = onset;
        const float onsetStart = onsetNow;
        onsetNow = onset;
        if (isPlate)
        {
            for (int i = 0; i < n; ++i)
                mono[i] = dl[i];
            PlateTank::Controls c;
            // The energy of a loop falls in steps of one round trip: keep the round trip within a third of the decay time so
            // that the steps stay small against the decay (a short plate for a short decay).
            c.scale = scaleNow * kPlateScale;
            const float ring = kPlateRingSeconds * c.scale;
            const float ringLimit = kPlateRingToDecay * decay.decay;
            if (ring > ringLimit)
                c.scale *= ringLimit / ring;
            c.decay = decay;
            c.modDepth = s.modDepth;
            c.modRate = s.modRate;
            c.diffusion = s.density;
            c.freeze = freezeState;
            plate.update(c, n);
            plate.process(mono, lateL, lateR, n);
            wetGainTarget = plate.wetGain();
            loopSecs = plate.loopSeconds(c.scale);
            for (int i = 0; i < n; ++i)
            {
                wetL[i] = lateL[i];
                wetR[i] = lateR[i];
                const float on = onsetStart + (onset - onsetStart) * (static_cast<float>(i + 1) * inv);
                lateL[i] = on * dl[i];
                lateR[i] = on * dr[i];
            }
            erGainTarget = 1.0f;
        }
        else
        {
            Fdn::Controls c;
            c.scale = scaleNow;
            c.decay = decay;
            c.modDepth = s.modDepth;
            c.modRate = s.modRate;
            c.freeze = freezeState;
            fdn.update(c, n);
            fdn.process(dl, dr, lateL, lateR, n);
            wetGainTarget = fdn.wetGain();
            loopSecs = fdn.meanLineSeconds() * static_cast<double>(scaleNow);
            float erL[kChunk], erR[kChunk];
            er.process(mono, erL, erR, n, scaleNow, s.density);
            const float levelTarget = std::clamp(s.early, 0.0f, 1.0f);
            const float levelStart = earlyNow;
            earlyNow = levelTarget;
            for (int i = 0; i < n; ++i)
            {
                const float level = levelStart + (levelTarget - levelStart) * (static_cast<float>(i + 1) * inv);
                wetL[i] = lateL[i];
                wetR[i] = lateR[i];
                const float on = onsetStart + (onset - onsetStart) * (static_cast<float>(i + 1) * inv);
                lateL[i] = level * erL[i] + on * dl[i];
                lateR[i] = level * erR[i] + on * dr[i];
            }
            erGainTarget = 1.0f;
        }
        if (snap)
        {
            wetGainNow = wetGainTarget;
            erGainNow = erGainTarget;
        }
        const float wgStart = wetGainNow, erStart = erGainNow;
        wetGainNow = wetGainTarget;
        erGainNow = erGainTarget;
        for (int i = 0; i < n; ++i)
        {
            const float t = static_cast<float>(i + 1) * inv;
            const float wg = wgStart + (wetGainNow - wgStart) * t;
            const float eg = erStart + (erGainNow - erStart) * t;
            wetL[i] = wg * wetL[i] + eg * lateL[i];
            wetR[i] = wg * wetR[i] + eg * lateR[i];
        }
    }

    // ---- width, wet filters, ducking, mix
    const float nyquist = static_cast<float>(sr) * 0.5f;
    const bool hpOn = s.lowCutHz > 0.01f;
    // On until the smoothed cutoff has reached the virtual bypass value: switching a filter that is almost, but not quite,
    // an identity (coefficient 1 - 2e-5) off would leave a small step.
    const bool lpOn = s.highCutHz < kLowPassOffHz;
    // The coefficients follow the cutoff even while a filter is bypassed, so that it can fade in without a step.
    const float hpG = std::tan(static_cast<float>(kPi) * std::clamp(s.lowCutHz, 1.0e-3f, nyquist * 0.45f) / static_cast<float>(sr));
    const float lpG =
        std::tan(static_cast<float>(kPi) * std::clamp(s.highCutHz, 1000.0f, nyquist * 0.99998f) / static_cast<float>(sr));
    const float hpKTarget = hpG / (1.0f + hpG), lpKTarget = lpG / (1.0f + lpG);
    if (snap)
    {
        hpKNow = hpKTarget;
        lpKNow = lpKTarget;
    }
    const float hpKStart = hpKNow, lpKStart = lpKNow;
    hpKNow = hpKTarget;
    lpKNow = lpKTarget;
    const float attack = 1.0f - std::exp(-1.0f / (0.002f * static_cast<float>(sr)));
    const float release = 1.0f - std::exp(-1.0f / (std::max(s.duckReleaseMs, 1.0f) * 0.001f * static_cast<float>(sr)));
    const float duckDepthDb = 30.0f * std::clamp(s.duck, 0.0f, 1.0f);
    const float duckStart = duckGainNow;
    for (int i = 0; i < n; ++i)
    {
        const float a = std::max(std::abs(dryL[i]), std::abs(dryR[i]));
        envelope += (a - envelope) * (a > envelope ? attack : release);
    }
    if (envelope < 1.0e-20f)
        envelope = 0.0f;
    // Full depth above -20 dBFS, none below -45 dBFS, linear in dB between.
    const float duckLevel = envelope > 0.0f ? std::clamp((20.0f * std::log10(envelope) + 45.0f) / 25.0f, 0.0f, 1.0f) : 0.0f;
    const float duckEnd = std::exp(-0.115129255f * duckDepthDb * duckLevel);
    duckGainNow = duckEnd;
    const float invN = 1.0f / static_cast<float>(n);
    for (int i = 0; i < n; ++i)
    {
        const float t = static_cast<float>(i + 1) * invN;
        float wl = wetL[i], wr = wetR[i];
        const float w = widthStart + (widthNow - widthStart) * t;
        const float mid = 0.5f * (wl + wr), side = 0.5f * (wl - wr) * w;
        wl = mid + side;
        wr = mid - side;
        const float hpK = hpKStart + (hpKTarget - hpKStart) * t;
        const float lpK = lpKStart + (lpKTarget - lpKStart) * t;
        float* out[2] = {&wl, &wr};
        for (int c = 0; c < 2; ++c)
        {
            float x = *out[c];
            // high-pass (one-pole, trapezoidal): state is the low-pass integrator
            if (hpOn)
            {
                const float v = (x - hpState[c]) * hpK;
                const float lp = v + hpState[c];
                hpState[c] = lp + v;
                if (std::abs(hpState[c]) < 1.0e-30f)
                    hpState[c] = 0.0f;
                x -= lp;
            }
            else
                hpState[c] = 0.0f; // bypassed: the integrator of a low cutoff sits at the mean, not at the sample
            // The low-pass integrator always runs (bypassed, its coefficient is 1 - 2e-5) so that the cut fades in seamlessly.
            const float v = (x - lpState[c]) * lpK;
            const float lp = v + lpState[c];
            lpState[c] = lp + v;
            if (std::abs(lpState[c]) < 1.0e-30f)
                lpState[c] = 0.0f;
            if (lpOn)
                x = lp;
            *out[c] = x;
        }
        const float duck = duckStart + (duckEnd - duckStart) * t;
        const float fl = fadeStart + (fadeEnd - fadeStart) * t;
        const float f = fl * fl * (3.0f - 2.0f * fl);
        const float m = (mixStart + (mixNow - mixStart) * t) * f;
        l[i] = dryL[i] + (wl * duck - dryL[i]) * m;
        r[i] = dryR[i] + (wr * duck - dryR[i]) * m;
    }
}

} // namespace crucible::fx
