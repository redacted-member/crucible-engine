// SPDX-License-Identifier: MIT
#include <crucible/Sample.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double besselI0(double x)
{
    double sum = 1.0;
    double term = 1.0;
    for (int k = 1; k < 60; ++k)
    {
        term *= (x / (2.0 * k)) * (x / (2.0 * k));
        sum += term;
        if (term < 1.0e-14 * sum)
            break;
    }
    return sum;
}

double kaiser(double r, double beta) // r in -1..1
{
    if (std::abs(r) >= 1.0)
        return 0.0;
    return besselI0(beta * std::sqrt(1.0 - r * r)) / besselI0(beta);
}

double sinc(double x)
{
    return std::abs(x) < 1.0e-12 ? 1.0 : std::sin(kPi * x) / (kPi * x);
}

// Zero-phase low-pass FIR with unit DC gain; cutoff in cycles per input sample.
std::vector<float> lowpass(double cutoff, int halfLength)
{
    std::vector<float> h(static_cast<size_t>(2 * halfLength + 1));
    double sum = 0.0;
    for (int n = -halfLength; n <= halfLength; ++n)
    {
        const double v = 2.0 * cutoff * sinc(2.0 * cutoff * n) * kaiser(static_cast<double>(n) / (halfLength + 1), 8.0);
        h[static_cast<size_t>(n + halfLength)] = static_cast<float>(v);
        sum += v;
    }
    for (auto& v : h)
        v = static_cast<float>(v / sum);
    return h;
}

// y[m] = sum h[k] x[m * q + k - half], x zero outside [0, n).
void filterDecimate(const std::vector<float>& x, int n, const std::vector<float>& h, int q, std::vector<float>& y,
                    int outFrames)
{
    const int half = static_cast<int>(h.size() / 2);
    y.assign(static_cast<size_t>(outFrames + 2 * kSamplePad), 0.0f);
    for (int m = 0; m < outFrames; ++m)
    {
        const int centre = m * q;
        const int lo = std::max(0, centre - half);
        const int hi = std::min(n - 1, centre + half);
        double acc = 0.0;
        for (int i = lo; i <= hi; ++i)
            acc += static_cast<double>(x[static_cast<size_t>(i + kSamplePad)]) * h[static_cast<size_t>(i - centre + half)];
        y[static_cast<size_t>(m + kSamplePad)] = static_cast<float>(acc);
    }
}

float detectPitch(const std::vector<float>& mono, double rate)
{
    const int n = static_cast<int>(mono.size());
    if (n < 4096)
        return -1.0f;
    // Loudest 8192-sample window, searched on a coarse grid.
    int best = 0;
    double bestEnergy = 0.0;
    for (int s = 0; s + 8192 <= n; s += 4096)
    {
        double e = 0.0;
        for (int i = s; i < s + 8192; i += 4)
            e += static_cast<double>(mono[static_cast<size_t>(i)]) * mono[static_cast<size_t>(i)];
        if (e > bestEnergy)
        {
            bestEnergy = e;
            best = s;
        }
    }
    if (bestEnergy < 1.0e-9)
        return -1.0f;
    const int minLag = std::max(8, static_cast<int>(rate / 4200.0));
    const int maxLag = std::min(static_cast<int>(rate / 24.0), 3000);
    const int len = 4096;
    if (best + len + maxLag >= n)
        best = std::max(0, n - len - maxLag - 1);
    double e0 = 0.0;
    for (int i = 0; i < len; ++i)
        e0 += static_cast<double>(mono[static_cast<size_t>(best + i)]) * mono[static_cast<size_t>(best + i)];
    std::vector<double> ac(static_cast<size_t>(maxLag + 2), 0.0);
    for (int lag = minLag; lag <= maxLag; ++lag)
    {
        double c = 0.0, e1 = 0.0;
        for (int i = 0; i < len; ++i)
        {
            const double a = mono[static_cast<size_t>(best + i)];
            const double b = mono[static_cast<size_t>(best + i + lag)];
            c += a * b;
            e1 += b * b;
        }
        ac[static_cast<size_t>(lag)] = c / std::sqrt(std::max(e0 * e1, 1.0e-30));
    }
    // First local maximum above 0.9 of the global maximum (avoids octave errors).
    double top = 0.0;
    for (int lag = minLag; lag <= maxLag; ++lag)
        top = std::max(top, ac[static_cast<size_t>(lag)]);
    if (top < 0.6)
        return -1.0f;
    for (int lag = minLag + 1; lag < maxLag; ++lag)
    {
        const double v = ac[static_cast<size_t>(lag)];
        if (v >= 0.9 * top && v >= ac[static_cast<size_t>(lag - 1)] && v >= ac[static_cast<size_t>(lag + 1)])
        {
            const double a = ac[static_cast<size_t>(lag - 1)], b = v, c = ac[static_cast<size_t>(lag + 1)];
            const double denom = a - 2.0 * b + c;
            const double shift = std::abs(denom) > 1.0e-12 ? 0.5 * (a - c) / denom : 0.0;
            const double period = lag + shift;
            return static_cast<float>(69.0 + 12.0 * std::log2(rate / period / 440.0));
        }
    }
    return -1.0f;
}
} // namespace

float detectSamplePitch(const std::vector<float>& mono, double rate)
{
    return detectPitch(mono, rate);
}

namespace
{
// Channel mix sample at an index of the original level (padding handled), 0 outside.
float mixAt(const SampleData& s, int i)
{
    if (i < 0 || i >= s.frames || s.levels.empty())
        return 0.0f;
    const auto& l = s.levels[0];
    float v = l.data[0][static_cast<size_t>(i + kSamplePad)];
    if (s.channels > 1 && !l.data[1].empty())
        v = 0.5f * (v + l.data[1][static_cast<size_t>(i + kSamplePad)]);
    return v;
}

// Rising crossing between i and i+1 (value < 0 before, >= 0 after); returns the interpolated index or -1.
double crossingAt(const SampleData& s, int i)
{
    const float a = mixAt(s, i), b = mixAt(s, i + 1);
    if (!(a < 0.0f && b >= 0.0f))
        return -1.0;
    return i + static_cast<double>(-a) / static_cast<double>(b - a);
}
} // namespace

double nearestZeroCrossing(const SampleData& s, double pos01, int direction)
{
    if (s.frames < 2 || s.levels.empty())
        return pos01;
    const double centre = std::clamp(pos01, 0.0, 1.0) * s.frames;
    const int c = std::min(static_cast<int>(std::floor(centre)), s.frames - 2);
    double before = -1.0, after = -1.0;
    if (direction <= 0)
        for (int i = c; i >= 0; --i)
            if (const double x = crossingAt(s, i); x >= 0.0 && x <= centre)
            {
                before = x;
                break;
            }
    if (direction >= 0)
        for (int i = c; i < s.frames - 1; ++i)
            if (const double x = crossingAt(s, i); x >= 0.0 && x >= centre)
            {
                after = x;
                break;
            }
    double best = before;
    if (best < 0.0 || (after >= 0.0 && after - centre < centre - best))
        best = after;
    return best < 0.0 ? pos01 : std::clamp(best / s.frames, 0.0, 1.0);
}

bool findLoopPoints(const SampleData& s, double start01, double end01, double& loopStart01, double& loopEnd01)
{
    start01 = std::clamp(start01, 0.0, 1.0);
    end01 = std::clamp(end01, 0.0, 1.0);
    loopStart01 = start01;
    loopEnd01 = end01;
    if (s.frames < 16 || s.levels.empty() || end01 - start01 <= 0.0)
        return false;
    const int a0 = static_cast<int>(start01 * s.frames);
    const int b0 = static_cast<int>(end01 * s.frames);
    const int len = b0 - a0;
    if (len < 8)
        return false;
    const int reach = std::max(2, len / 20);
    // Candidate rising crossings near each end.
    std::vector<double> starts, ends;
    for (int i = std::max(0, a0 - reach); i <= std::min(s.frames - 2, a0 + reach); ++i)
        if (const double x = crossingAt(s, i); x >= 0.0)
            starts.push_back(x);
    for (int i = std::max(0, b0 - reach); i <= std::min(s.frames - 2, b0 + reach); ++i)
        if (const double x = crossingAt(s, i); x >= 0.0)
            ends.push_back(x);
    if (starts.empty() || ends.empty())
        return false;
    // Prefer the pair whose short waveform windows after the start and after the end line up best (a seam without
    // a step), and whose region length stays close to the requested one.
    const int window = std::clamp(len / 16, 8, 256);
    double bestCost = 1.0e300, bestA = starts[0], bestB = ends[0];
    for (const double x : starts)
        for (const double y : ends)
        {
            if (y - x < 8.0)
                continue;
            double cost = 0.0;
            const int xi = static_cast<int>(std::ceil(x)), yi = static_cast<int>(std::ceil(y));
            for (int k = 0; k < window; ++k)
            {
                const double d = mixAt(s, xi + k) - mixAt(s, yi + k);
                cost += d * d;
            }
            cost = cost / window + 1.0e-6 * std::abs((y - x) - len) / len;
            if (cost < bestCost)
            {
                bestCost = cost;
                bestA = x;
                bestB = y;
            }
        }
    loopStart01 = bestA / s.frames;
    loopEnd01 = bestB / s.frames;
    return loopEnd01 > loopStart01;
}

int SampleData::levelIndexFor(double pitchRatio, double outputRate) const
{
    // Content must stay below (outputRate - 20 kHz) / pitchRatio so that anything folding over Nyquist lands above
    // 20 kHz; below that limit the original is used.
    const double allowed = std::max(outputRate - 20000.0, 2000.0) / std::max(pitchRatio, 1.0e-3);
    for (size_t i = 0; i < levels.size(); ++i)
        if (levels[i].cutoff * sampleRate <= allowed)
            return static_cast<int>(i);
    return static_cast<int>(levels.size()) - 1;
}

std::unique_ptr<SampleData> SampleData::build(const AudioData& audio, std::string name)
{
    auto s = std::make_unique<SampleData>();
    s->name = std::move(name);
    s->sampleRate = audio.sampleRate > 0.0 ? audio.sampleRate : 48000.0;
    s->channels = std::clamp(audio.numChannels(), 1, kSampleMaxChannels);
    s->frames = std::min(audio.numFrames(), kSampleMaxFrames);
    if (s->frames < 2)
        return nullptr;

    SampleLevel base;
    base.decimation = 1;
    base.cutoff = 0.5;
    base.frames = s->frames;
    for (int c = 0; c < s->channels; ++c)
    {
        base.data[static_cast<size_t>(c)].assign(static_cast<size_t>(s->frames + 2 * kSamplePad), 0.0f);
        const auto& src = audio.channel(c);
        for (int i = 0; i < s->frames; ++i)
        {
            const float v = src[static_cast<size_t>(i)];
            base.data[static_cast<size_t>(c)][static_cast<size_t>(i + kSamplePad)] = std::isfinite(v) ? std::clamp(v, -kSampleMaxValue, kSampleMaxValue) : 0.0f;
        }
    }
    s->levels.push_back(std::move(base));

    // Two levels per octave, each derived from the previous one.
    for (int k = 1; k <= 12; ++k)
    {
        const auto& prev = s->levels.back();
        const int q = (k % 2 == 0) ? 2 : 1;
        const int decimation = prev.decimation * q;
        if (prev.frames / q < 64)
            break;
        SampleLevel lv;
        lv.decimation = decimation;
        lv.cutoff = 0.5 * std::pow(2.0, -k / 2.0);
        lv.frames = prev.frames / q;
        // Cutoff relative to the previous level's rate, with a margin for the transition band.
        const double rel = std::min(lv.cutoff * prev.decimation * 0.92, 0.46 / q);
        const auto h = lowpass(rel, 72);
        for (int c = 0; c < s->channels; ++c)
            filterDecimate(prev.data[static_cast<size_t>(c)], prev.frames, h, q, lv.data[static_cast<size_t>(c)], lv.frames);
        s->levels.push_back(std::move(lv));
    }

    // Overview and pitch.
    s->peakMin.assign(kSamplePeakColumns, 0.0f);
    s->peakMax.assign(kSamplePeakColumns, 0.0f);
    std::vector<float> mono(static_cast<size_t>(s->frames));
    const auto& l0 = s->levels[0];
    for (int i = 0; i < s->frames; ++i)
    {
        float v = l0.data[0][static_cast<size_t>(i + kSamplePad)];
        if (s->channels > 1)
            v = 0.5f * (v + l0.data[1][static_cast<size_t>(i + kSamplePad)]);
        mono[static_cast<size_t>(i)] = v;
    }
    for (int c = 0; c < kSamplePeakColumns; ++c)
    {
        const int a = static_cast<int>(static_cast<std::int64_t>(c) * s->frames / kSamplePeakColumns);
        const int b = std::max(a + 1, static_cast<int>(static_cast<std::int64_t>(c + 1) * s->frames / kSamplePeakColumns));
        float lo = 0.0f, hi = 0.0f;
        for (int i = a; i < std::min(b, s->frames); ++i)
        {
            lo = std::min(lo, mono[static_cast<size_t>(i)]);
            hi = std::max(hi, mono[static_cast<size_t>(i)]);
        }
        s->peakMin[static_cast<size_t>(c)] = lo;
        s->peakMax[static_cast<size_t>(c)] = hi;
    }
    s->detectedNote = detectPitch(mono, s->sampleRate);
    return s;
}

SincKernel::SincKernel()
{
    const auto build = [](int taps, double beta, std::vector<float>& out)
    {
        out.assign(static_cast<size_t>((phases + 1) * 2 * taps), 0.0f);
        std::vector<double> prev(static_cast<size_t>(taps));
        for (int p = phases; p >= 0; --p)
        {
            std::vector<double> c(static_cast<size_t>(taps));
            const double frac = static_cast<double>(p) / phases;
            double sum = 0.0;
            for (int j = 0; j < taps; ++j)
            {
                const double d = frac + (taps / 2 - 1) - j;
                c[static_cast<size_t>(j)] = sinc(d) * kaiser(d / (taps / 2), beta);
                sum += c[static_cast<size_t>(j)];
            }
            for (auto& v : c)
                v /= sum; // unit DC gain at every phase
            for (int j = 0; j < taps; ++j)
            {
                out[static_cast<size_t>(p * 2 * taps + j)] = static_cast<float>(c[static_cast<size_t>(j)]);
                out[static_cast<size_t>(p * 2 * taps + taps + j)] =
                    p == phases ? 0.0f : static_cast<float>(prev[static_cast<size_t>(j)] - c[static_cast<size_t>(j)]);
            }
            prev = c;
        }
    };
    build(16, 8.5, t16);
    build(8, 6.0, t8);
}

const SincKernel& SincKernel::instance()
{
    static const SincKernel k;
    return k;
}

// ---------------------------------------------------------------------------------------------------------------
bool SampleBank::load(int slot, const AudioData& audio, const std::string& name, std::string& error)
{
    if (slot < 0 || slot >= static_cast<int>(slots.size()))
    {
        error = "no such sample slot";
        return false;
    }
    auto built = SampleData::build(audio, name);
    if (!built)
    {
        error = "the audio is too short";
        return false;
    }
    return install(slot, std::move(built));
}

bool SampleBank::install(int slot, std::unique_ptr<SampleData> built)
{
    if (slot < 0 || slot >= static_cast<int>(slots.size()) || !built)
        return false;
    SincKernel::instance();
    slots[static_cast<size_t>(slot)].publish(std::move(built));
    setSliceSet(slot, nullptr);
    return true;
}

bool SampleBank::loadFile(int slot, const std::string& wavPath, std::string& error)
{
    AudioData audio;
    if (!readWavFile(wavPath, audio, error))
        return false;
    const auto slash = wavPath.find_last_of("/\\");
    return load(slot, audio, slash == std::string::npos ? wavPath : wavPath.substr(slash + 1), error);
}

void SampleBank::clear(int slot)
{
    if (slot >= 0 && slot < static_cast<int>(slots.size()))
    {
        slots[static_cast<size_t>(slot)].publish(nullptr);
        setSliceSet(slot, nullptr);
    }
}

bool SampleBank::isLoaded(int slot) const
{
    return sample(slot) != nullptr;
}

const SampleData* SampleBank::sample(int slot) const
{
    return slot >= 0 && slot < static_cast<int>(slots.size()) ? slots[static_cast<size_t>(slot)].latest() : nullptr;
}

bool SampleBank::original(int slot, AudioData& out, std::string& name) const
{
    const auto* s = sample(slot);
    if (s == nullptr)
        return false;
    out = AudioData{};
    out.sampleRate = s->sampleRate;
    out.channels.resize(static_cast<size_t>(s->channels));
    for (int c = 0; c < s->channels; ++c)
    {
        const auto& d = s->levels[0].data[static_cast<size_t>(c)];
        out.channels[static_cast<size_t>(c)].assign(d.begin() + kSamplePad, d.begin() + kSamplePad + s->frames);
    }
    name = s->name;
    return true;
}

std::vector<SampleBank::Record> SampleBank::serialise() const
{
    std::vector<Record> out;
    for (int slot = 0; slot < static_cast<int>(slots.size()); ++slot)
    {
        AudioData audio;
        Record r;
        if (!original(slot, audio, r.name))
            continue;
        r.slot = slot;
        r.rate = audio.sampleRate;
        r.channels = audio.numChannels();
        r.frames.resize(static_cast<size_t>(audio.numFrames()) * static_cast<size_t>(r.channels));
        for (int c = 0; c < r.channels; ++c)
            for (int i = 0; i < audio.numFrames(); ++i)
                r.frames[static_cast<size_t>(i * r.channels + c)] =
                    audio.channels[static_cast<size_t>(c)][static_cast<size_t>(i)];
        out.push_back(std::move(r));
    }
    return out;
}

void SampleBank::deserialise(const std::vector<Record>& records)
{
    for (int slot = 0; slot < static_cast<int>(slots.size()); ++slot)
        clear(slot);
    for (const auto& r : records)
    {
        if (r.channels < 1 || r.frames.empty())
            continue;
        AudioData audio;
        audio.sampleRate = r.rate > 0.0 ? r.rate : 48000.0;
        const size_t frames = r.frames.size() / static_cast<size_t>(r.channels);
        audio.channels.assign(static_cast<size_t>(std::min(r.channels, kSampleMaxChannels)), std::vector<float>(frames));
        for (size_t c = 0; c < audio.channels.size(); ++c)
            for (size_t i = 0; i < frames; ++i)
                audio.channels[c][i] = r.frames[i * static_cast<size_t>(r.channels) + c];
        std::string error;
        load(r.slot, audio, r.name, error);
    }
}

void SampleBank::collectGarbage()
{
    for (auto& s : slots)
        s.collectGarbage();
    for (auto& s : sliceSlots)
        s.collectGarbage();
}

void SliceSet::region(int index, int frames, double& begin, double& end) const
{
    const int i = std::clamp(index, 0, count() - 1);
    const double total = static_cast<double>(frames);
    begin = i == 0 ? 0.0 : static_cast<double>(markers[static_cast<size_t>(i - 1)]) * total;
    end = i == count() - 1 ? total : static_cast<double>(markers[static_cast<size_t>(i)]) * total;
}

std::unique_ptr<SliceSet> SliceSet::make(std::vector<float> markers)
{
    auto set = std::make_unique<SliceSet>();
    constexpr float kMinGap = 1.0f / 2048.0f;
    std::sort(markers.begin(), markers.end());
    for (const float m : markers)
    {
        if (!std::isfinite(m) || m < kMinGap || m > 1.0f - kMinGap)
            continue;
        if (!set->markers.empty() && m - set->markers.back() < kMinGap)
            continue;
        if (static_cast<int>(set->markers.size()) >= kMaxSlices - 1)
            break;
        set->markers.push_back(m);
    }
    return set;
}

void SampleBank::setSliceSet(int slot, std::unique_ptr<SliceSet> set)
{
    if (slot >= 0 && slot < static_cast<int>(sliceSlots.size()))
    {
        if (set && set->markers.empty())
            set.reset();
        sliceSlots[static_cast<size_t>(slot)].publish(std::move(set));
    }
}

bool SampleBank::setSlices(int slot, std::vector<float> markers)
{
    if (!isLoaded(slot))
        return false;
    setSliceSet(slot, SliceSet::make(std::move(markers)));
    return true;
}

std::vector<float> SampleBank::slices(int slot) const
{
    if (slot < 0 || slot >= static_cast<int>(sliceSlots.size()))
        return {};
    std::vector<float> out;
    sliceSlots[static_cast<size_t>(slot)].withLatest([&](const SliceSet& s) { out = s.markers; });
    return out;
}

int SampleBank::sliceEqually(int slot, int n)
{
    n = std::clamp(n, 1, kMaxSlices);
    std::vector<float> markers;
    for (int i = 1; i < n; ++i)
        markers.push_back(static_cast<float>(i) / static_cast<float>(n));
    if (!setSlices(slot, std::move(markers)))
        return 0;
    return n;
}

int SampleBank::sliceByTransients(int slot, int maxSlices, float sensitivity)
{
    const SampleData* d = sample(slot);
    if (d == nullptr)
        return 0;
    maxSlices = std::clamp(maxSlices, 1, kMaxSlices);
    // Energy per 2 ms frame of the channel mix.
    const int hop = std::max(16, static_cast<int>(d->sampleRate * 0.002));
    const int frames = d->frames / hop;
    std::vector<double> energy(static_cast<size_t>(std::max(frames, 0)), 0.0);
    const SampleLevel& lv = d->levels[0];
    for (int k = 0; k < frames; ++k)
    {
        double sum = 0.0;
        for (int i = 0; i < hop; ++i)
        {
            const size_t at = static_cast<size_t>(kSamplePad + k * hop + i);
            double x = lv.data[0][at];
            if (d->channels > 1)
                x = 0.5 * (x + lv.data[1][at]);
            sum += x * x;
        }
        energy[static_cast<size_t>(k)] = sum / hop;
    }
    double peakEnergy = 0.0;
    for (const double e : energy)
        peakEnergy = std::max(peakEnergy, e);
    // Onset strength: log rise of a frame's energy over the mean of the eight frames before it.
    const double floorEnergy = std::max(peakEnergy * 1.0e-6, 1.0e-12);
    const double threshold = 1.0 + (1.0 - std::clamp(static_cast<double>(sensitivity), 0.0, 1.0)) * 4.0;
    struct Onset
    {
        int frame;
        double strength;
    };
    std::vector<Onset> found;
    const int minGap = std::max(1, static_cast<int>(0.04 / (static_cast<double>(hop) / d->sampleRate)));
    for (int k = 4; k < frames; ++k)
    {
        double before = 0.0;
        const int from = std::max(0, k - 8);
        for (int j = from; j < k; ++j)
            before += energy[static_cast<size_t>(j)];
        before /= (k - from);
        const double strength = std::log((energy[static_cast<size_t>(k)] + floorEnergy) / (before + floorEnergy));
        if (strength < threshold || energy[static_cast<size_t>(k)] < peakEnergy * 1.0e-4)
            continue;
        if (!found.empty() && k - found.back().frame < minGap)
        {
            if (strength > found.back().strength)
                found.back() = {k, strength};
            continue;
        }
        found.push_back({k, strength});
    }
    std::sort(found.begin(), found.end(), [](const Onset& a, const Onset& b) { return a.strength > b.strength; });
    if (static_cast<int>(found.size()) > maxSlices - 1)
        found.resize(static_cast<size_t>(maxSlices - 1));
    std::vector<float> markers;
    for (const auto& o : found)
        markers.push_back(static_cast<float>(std::max(0, o.frame - 1) * hop) / static_cast<float>(d->frames));
    setSlices(slot, std::move(markers));
    const SliceSet* now = sliceSlots[static_cast<size_t>(slot)].latest();
    return now != nullptr ? now->count() : 1;
}

void SampleBank::acquire(Pinned& out) noexcept
{
    for (size_t i = 0; i < slots.size(); ++i)
        out[i] = slots[i].acquire();
}

void SampleBank::release() noexcept
{
    for (auto& s : slots)
        s.release();
}

void SampleBank::acquireSlices(PinnedSlices& out) noexcept
{
    for (size_t i = 0; i < sliceSlots.size(); ++i)
        out[i] = sliceSlots[i].acquire();
}

void SampleBank::releaseSlices() noexcept
{
    for (auto& s : sliceSlots)
        s.release();
}

} // namespace crucible
