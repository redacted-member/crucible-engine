// SPDX-License-Identifier: MIT
#include <crucible/UserWavetables.h>

#include <crucible/Fft.h>
#include <crucible/Sample.h>
#include <crucible/Spectral.h>

#include <algorithm>
#include <array>
#include <memory>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>

namespace crucible
{
namespace
{
constexpr int kSize = WavetableBank::tableSize;
constexpr int kFrames = WavetableBank::numFrames;
constexpr int kMips = WavetableBank::numMips;
constexpr int kMaxHarmonic = kSize / 2;
constexpr double kTwoPi = 6.283185307179586;

using Spectrum = std::vector<std::complex<double>>; // index h = harmonic, 0..kMaxHarmonic - 1

bool isPow2(size_t n)
{
    return n >= 2 && (n & (n - 1)) == 0;
}

// Harmonic coefficients c_h = (1 / L) sum x[n] e^(-2 pi i h n / L) for h < min(L / 2, kMaxHarmonic).
Spectrum analyse(const std::vector<float>& cycle)
{
    const size_t len = cycle.size();
    Spectrum s(static_cast<size_t>(kMaxHarmonic), {0.0, 0.0});
    const size_t hmax = std::min(len / 2, static_cast<size_t>(kMaxHarmonic));
    if (isPow2(len) && len <= (1U << 16))
    {
        int order = 0;
        while ((size_t{1} << order) < len)
            ++order;
        const Fft fft(order);
        std::vector<float> buf(2 * len, 0.0f);
        std::copy(cycle.begin(), cycle.end(), buf.begin());
        fft.forwardPortable(buf.data());
        for (size_t h = 1; h < hmax; ++h)
            s[h] = {buf[2 * h] / static_cast<double>(len), buf[2 * h + 1] / static_cast<double>(len)};
        return s;
    }
    for (size_t h = 1; h < hmax; ++h)
    {
        std::complex<double> acc{0.0, 0.0};
        const double w = kTwoPi * static_cast<double>(h) / static_cast<double>(len);
        for (size_t n = 0; n < len; ++n)
            acc += static_cast<double>(cycle[n]) * std::polar(1.0, -w * static_cast<double>(n));
        s[h] = acc / static_cast<double>(len);
    }
    return s;
}

// Inverse real FFT of the harmonics up to maxHarmonic at `size` samples per cycle (DC and everything above are zero).
void synthesise(const Spectrum& s, int maxHarmonic, std::vector<float>& out, int size = kSize)
{
    static thread_local std::array<std::unique_ptr<Fft>, 14> ffts;
    int order = 0;
    while ((1 << order) < size)
        ++order;
    if (!ffts[static_cast<size_t>(order)])
        ffts[static_cast<size_t>(order)] = std::make_unique<Fft>(order);
    std::vector<float> buf(static_cast<size_t>(2 * size), 0.0f);
    for (int h = 1; h <= maxHarmonic && h < kMaxHarmonic && h < size / 2; ++h)
    {
        // Inverse convention: x[n] = (1 / N) sum X[k] e^(+...), so X[k] = N * c_k for a real signal of amplitude c.
        buf[static_cast<size_t>(2 * h)] = static_cast<float>(s[static_cast<size_t>(h)].real() * size);
        buf[static_cast<size_t>(2 * h + 1)] = static_cast<float>(s[static_cast<size_t>(h)].imag() * size);
    }
    // The inverse mirrors the conjugate bins, so out = 2 Re(sum c_h e^(2 pi i h n / N)): the original cycle.
    ffts[static_cast<size_t>(order)]->inversePortable(buf.data());
    out.assign(buf.begin(), buf.begin() + size);
}
} // namespace

const std::vector<std::string>& allTableNames()
{
    static const std::vector<std::string> names = []
    {
        auto n = WavetableBank::tableNames();
        for (int i = 1; i <= kUserTables; ++i)
            n.push_back("User " + std::to_string(i));
        return n;
    }();
    return names;
}

UserWavetable::UserWavetable() = default;
UserWavetable::~UserWavetable() = default;

std::unique_ptr<UserWavetable> UserWavetable::fromCycles(const std::vector<std::vector<float>>& cycles,
                                                         WavetableImport::Normalise normalise, std::string name)
{
    return build({{cycles, 1.0}}, normalise, std::move(name), false, 48000.0);
}

std::unique_ptr<UserWavetable> UserWavetable::fromZones(const std::vector<WavetableZone>& zones,
                                                        WavetableImport::Normalise normalise, std::string name,
                                                        double sampleRate)
{
    if (zones.empty() || zones.size() > static_cast<size_t>(kMaxWavetableZones) || !(sampleRate > 0.0))
        return nullptr;
    double prev = 0.0;
    for (const auto& z : zones)
    {
        if (!std::isfinite(z.maxHz) || !(z.maxHz > prev))
            return nullptr;
        prev = z.maxHz;
        if (std::none_of(z.cycles.begin(), z.cycles.end(), [](const auto& c) { return c.size() >= 4; }))
            return nullptr;
    }
    return build(zones, normalise, std::move(name), false, sampleRate);
}

std::unique_ptr<UserWavetable> UserWavetable::rebuiltForRate(double sampleRate) const
{
    if (zoneSources.size() < 2 || sampleRate == builtRate)
        return nullptr;
    return build(zoneSources, normalisation, label, false, sampleRate);
}

std::unique_ptr<UserWavetable> UserWavetable::fromBaseFrames(const std::vector<float>& frames, std::string name)
{
    std::vector<std::vector<float>> cycles(static_cast<size_t>(kFrames));
    for (int f = 0; f < kFrames; ++f)
        cycles[static_cast<size_t>(f)].assign(frames.begin() + f * kSize, frames.begin() + (f + 1) * kSize);
    return build({{cycles, 1.0}}, WavetableImport::Normalise::None, std::move(name), true, 48000.0);
}

namespace
{
// The zone whose range holds the representative fundamental of mip level `mip` at the output rate: the geometric
// centre of the fundamentals for which WavetableBank::mipForIncrement picks that level at the output band limit.
// Level m is chosen for increments in (limit / harmonics[m - 1], limit / harmonics[m]]; the outermost levels have no
// outer edge, so they use the ratio to their neighbour.
size_t zoneForMip(const std::vector<WavetableZone>& zones, int mip, double rate)
{
    const double limit = static_cast<double>(WavetableBank::outputBandLimit(rate));
    const auto h = [](int m) { return static_cast<double>(WavetableBank::mipHarmonics(m)); };
    double hi = limit * rate / h(mip);
    double lo = mip > 0 ? limit * rate / h(mip - 1) : 0.0;
    if (mip == 0)
        lo = hi * h(1) / h(0);
    else if (mip == kMips - 1)
        hi = lo * h(mip - 1) / h(mip);
    const double centre = std::sqrt(lo * hi);
    for (size_t z = 0; z + 1 < zones.size(); ++z)
        if (centre <= zones[z].maxHz)
            return z;
    return zones.size() - 1;
}
} // namespace

std::unique_ptr<UserWavetable> UserWavetable::build(const std::vector<WavetableZone>& zones,
                                                    WavetableImport::Normalise normalise, std::string name, bool verbatim,
                                                    double sampleRate)
{
    auto t = std::make_unique<UserWavetable>();
    t->label = std::move(name);
    t->normalisation = normalise;
    t->builtRate = sampleRate;
    t->zoneN = static_cast<int>(zones.size());
    if (zones.size() > 1)
        t->zoneSources = zones;
    const size_t nz = zones.size();

    // Per zone: the source frames spread over 64 frames (spectral crossfade = time-domain crossfade of band-limited
    // cycles), then the full-band frames for the normalisation.
    std::vector<std::vector<Spectrum>> frames(nz);
    std::vector<std::vector<std::vector<float>>> full(nz);
    for (size_t z = 0; z < nz; ++z)
    {
        std::vector<Spectrum> src;
        for (const auto& c : zones[z].cycles)
            if (c.size() >= 4)
                src.push_back(analyse(c));
        if (z == 0)
            t->sourceFrames = static_cast<int>(src.size());
        if (src.empty())
            src.emplace_back(static_cast<size_t>(kMaxHarmonic), std::complex<double>{0.0, 0.0});
        frames[z].resize(static_cast<size_t>(kFrames));
        full[z].resize(static_cast<size_t>(kFrames));
        for (int f = 0; f < kFrames; ++f)
        {
            const double pos =
                src.size() == 1 ? 0.0 : static_cast<double>(f) * static_cast<double>(src.size() - 1) / (kFrames - 1);
            const size_t a = std::min(static_cast<size_t>(pos), src.size() - 1);
            const size_t b = std::min(a + 1, src.size() - 1);
            const double mix = pos - static_cast<double>(a);
            auto& out = frames[z][static_cast<size_t>(f)];
            out.resize(static_cast<size_t>(kMaxHarmonic));
            for (size_t h = 0; h < out.size(); ++h)
                out[h] = src[a][h] * (1.0 - mix) + src[b][h] * mix;
            synthesise(out, kMaxHarmonic - 1, full[z][static_cast<size_t>(f)]);
        }
    }
    // One table peak over every zone (Table mode), or each frame's own peak (Frame mode).
    std::vector<std::vector<double>> gain(nz, std::vector<double>(static_cast<size_t>(kFrames), 1.0));
    if (normalise != WavetableImport::Normalise::None)
    {
        double tablePeak = 0.0;
        std::vector<std::vector<double>> framePeak(nz, std::vector<double>(static_cast<size_t>(kFrames), 0.0));
        for (size_t z = 0; z < nz; ++z)
            for (int f = 0; f < kFrames; ++f)
            {
                auto& fp = framePeak[z][static_cast<size_t>(f)];
                for (const float x : full[z][static_cast<size_t>(f)])
                    fp = std::max(fp, static_cast<double>(std::abs(x)));
                tablePeak = std::max(tablePeak, fp);
            }
        for (size_t z = 0; z < nz; ++z)
            for (int f = 0; f < kFrames; ++f)
            {
                const double peak =
                    normalise == WavetableImport::Normalise::Frame ? framePeak[z][static_cast<size_t>(f)] : tablePeak;
                gain[z][static_cast<size_t>(f)] = peak > 1.0e-9 ? 1.0 / peak : 1.0;
            }
    }

    t->mipOffset.resize(kMips);
    size_t total = 0;
    for (int m = 0; m < kMips; ++m)
    {
        t->mipOffset[static_cast<size_t>(m)] = total;
        total += static_cast<size_t>(kFrames) *
                 static_cast<size_t>(WavetableBank::mipSize(m) + WavetableBank::guardBefore + WavetableBank::guardAfter);
    }
    t->data.assign(total, 0.0f);
    t->base.assign(static_cast<size_t>(kFrames * kSize), 0.0f);
    std::vector<size_t> mipZone(kMips, 0);
    if (nz > 1)
        for (int m = 0; m < kMips; ++m)
            mipZone[static_cast<size_t>(m)] = zoneForMip(zones, m, sampleRate);
    const size_t baseZone = mipZone[0]; // what the frames() / spectral views show: the lowest level's zone
    std::vector<float> work;
    for (int f = 0; f < kFrames; ++f)
    {
        // A restored table keeps its saved frames exactly: resynthesising them is not a fixed point.
        const auto gb = static_cast<float>(gain[baseZone][static_cast<size_t>(f)]);
        for (int i = 0; i < kSize; ++i)
            t->base[static_cast<size_t>(f * kSize + i)] =
                verbatim ? zones[0].cycles[static_cast<size_t>(f)][static_cast<size_t>(i)]
                         : full[baseZone][static_cast<size_t>(f)][static_cast<size_t>(i)] * gb;
        for (int mip = 0; mip < kMips; ++mip)
        {
            // Same levels as the factory tables: the first mipHarmonics(mip) partials at mipSize(mip) samples.
            const size_t z = mipZone[static_cast<size_t>(mip)];
            const auto g = static_cast<float>(gain[z][static_cast<size_t>(f)]);
            const int size = WavetableBank::mipSize(mip);
            synthesise(frames[z][static_cast<size_t>(f)], WavetableBank::mipHarmonics(mip), work, size);
            float* dst = t->data.data() + (t->frame(mip, f) - t->data.data());
            for (int i = 0; i < size; ++i)
                dst[i] = work[static_cast<size_t>(i)] * g;
            dst[-2] = dst[size - 2];
            dst[-1] = dst[size - 1];
            dst[size] = dst[0];
            dst[size + 1] = dst[1];
            dst[size + 2] = dst[2];
        }
    }
    t->spectralData =
        spectral::SpectralTable::analyse(kFrames, [&t](int f) { return t->base.data() + static_cast<size_t>(f) * kSize; });
    return t;
}

FrameDetection detectWavetableFrames(const AudioData& audio, const WavetableImport& options)
{
    const int n = audio.numFrames();
    FrameDetection d;
    if (options.frameSize > 0)
    {
        d.frameSize = options.frameSize;
        d.reason = FrameDetection::Reason::Forced;
    }
    else if (audio.cycleLength > 0)
    {
        d.frameSize = audio.cycleLength;
        d.reason = FrameDetection::Reason::ClmMarker;
    }
    else if (n >= 2048 && n % 2048 == 0)
    {
        d.frameSize = 2048;
        d.reason = FrameDetection::Reason::Multiple2048;
    }
    else if (n <= 8192)
    {
        d.frameSize = n;
        d.reason = FrameDetection::Reason::SingleCycle;
    }
    else
    {
        d.frameSize = 2048;
        d.reason = FrameDetection::Reason::Fallback;
    }
    d.frames = d.frameSize >= 4 ? n / d.frameSize : 0;
    return d;
}

int estimateCycleLength(const AudioData& audio)
{
    if (audio.numFrames() == 0)
        return 0;
    const float note = detectSamplePitch(audio.mono(), audio.sampleRate);
    if (note < 0.0f)
        return 0;
    const double hz = 440.0 * std::pow(2.0, (static_cast<double>(note) - 69.0) / 12.0);
    return std::max(0, static_cast<int>(std::lround(audio.sampleRate / hz)));
}

std::vector<std::vector<float>> sliceCycles(const AudioData& audio, const WavetableImport& options)
{
    std::vector<float> mono = options.channel < 0 ? audio.mono() : audio.channel(options.channel);
    const size_t n = mono.size();
    const size_t frame = static_cast<size_t>(std::max(detectWavetableFrames(audio, options).frameSize, 0));
    std::vector<std::vector<float>> cycles;
    if (frame < 4)
        return cycles;
    for (size_t start = 0; start + frame <= n; start += frame)
        cycles.emplace_back(mono.begin() + static_cast<std::ptrdiff_t>(start),
                            mono.begin() + static_cast<std::ptrdiff_t>(start + frame));
    return cycles;
}

bool UserWavetableBank::load(int slot, const AudioData& audio, const WavetableImport& options, std::string name,
                             std::string& error)
{
    if (slot < 0 || slot >= kUserTables)
    {
        error = "slot out of range";
        return false;
    }
    const auto cycles = sliceCycles(audio, options);
    if (cycles.empty())
    {
        error = "no complete cycle in the audio";
        return false;
    }
    slots[static_cast<size_t>(slot)].publish(UserWavetable::fromCycles(cycles, options.normalise, std::move(name)));
    return true;
}

bool UserWavetableBank::loadFile(int slot, const std::string& wavPath, const WavetableImport& options, std::string& error)
{
    AudioData audio;
    if (!readWavFile(wavPath, audio, error))
        return false;
    auto name = wavPath;
    const auto cut = name.find_last_of("/\\");
    if (cut != std::string::npos)
        name = name.substr(cut + 1);
    return load(slot, audio, options, name, error);
}

bool UserWavetableBank::loadCycles(int slot, const std::vector<std::vector<float>>& cycles,
                                   WavetableImport::Normalise normalise, std::string name)
{
    if (slot < 0 || slot >= kUserTables || cycles.empty())
        return false;
    slots[static_cast<size_t>(slot)].publish(UserWavetable::fromCycles(cycles, normalise, std::move(name)));
    return true;
}

bool UserWavetableBank::loadZones(int slot, const std::vector<WavetableZone>& zones, WavetableImport::Normalise normalise,
                                  std::string name)
{
    if (slot < 0 || slot >= kUserTables)
        return false;
    auto t = UserWavetable::fromZones(zones, normalise, std::move(name), sampleRate);
    if (t == nullptr)
        return false;
    slots[static_cast<size_t>(slot)].publish(std::move(t));
    return true;
}

void UserWavetableBank::setSampleRate(double rate)
{
    if (!(rate > 0.0) || rate == sampleRate)
        return;
    sampleRate = rate;
    for (auto& s : slots)
    {
        std::unique_ptr<UserWavetable> rebuilt;
        if (const auto* t = s.latest())
            rebuilt = t->rebuiltForRate(rate);
        if (rebuilt != nullptr)
            s.publish(std::move(rebuilt));
    }
}

void UserWavetableBank::clear(int slot)
{
    if (slot >= 0 && slot < kUserTables)
        slots[static_cast<size_t>(slot)].publish(nullptr);
}

bool UserWavetableBank::isLoaded(int slot) const
{
    return table(slot) != nullptr;
}

std::string UserWavetableBank::name(int slot) const
{
    const auto* t = table(slot);
    return t != nullptr ? t->name() : std::string{};
}

const UserWavetable* UserWavetableBank::table(int slot) const
{
    if (slot < 0 || slot >= kUserTables)
        return nullptr;
    return slots[static_cast<size_t>(slot)].latest();
}

void UserWavetableBank::collectGarbage()
{
    for (auto& s : slots)
        s.collectGarbage();
}

std::vector<UserWavetableBank::Record> UserWavetableBank::serialise() const
{
    std::vector<Record> out;
    for (int s = 0; s < kUserTables; ++s)
        if (const auto* t = table(s))
        {
            Record r{s, t->name(), t->baseFrames(), {}, 0};
            if (t->zoneCount() > 1)
            {
                r.zones = t->zones();
                r.normalise = static_cast<int>(t->normalisationMode());
            }
            out.push_back(std::move(r));
        }
    return out;
}

void UserWavetableBank::deserialise(const std::vector<Record>& records)
{
    for (int s = 0; s < kUserTables; ++s)
        clear(s);
    for (const auto& r : records)
    {
        if (r.slot < 0 || r.slot >= kUserTables || r.frames.size() != static_cast<size_t>(kFrames * kSize))
            continue;
        std::unique_ptr<UserWavetable> t;
        if (r.zones.size() >= 2 && r.normalise >= 0 && r.normalise <= static_cast<int>(WavetableImport::Normalise::None))
            t = UserWavetable::fromZones(r.zones, static_cast<WavetableImport::Normalise>(r.normalise), r.name, sampleRate);
        if (t == nullptr)
            t = UserWavetable::fromBaseFrames(r.frames, r.name);
        slots[static_cast<size_t>(r.slot)].publish(std::move(t));
    }
}

std::vector<float> UserWavetableBank::packZones(const Record& r)
{
    std::vector<float> b;
    b.push_back(1.0f); // version
    b.push_back(static_cast<float>(r.normalise));
    b.push_back(static_cast<float>(r.zones.size()));
    for (const auto& z : r.zones)
    {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &z.maxHz, sizeof bits);
        for (int k = 0; k < 4; ++k)
            b.push_back(static_cast<float>((bits >> (16 * k)) & 0xFFFFU));
        b.push_back(static_cast<float>(z.cycles.size()));
        for (const auto& c : z.cycles)
            b.push_back(static_cast<float>(c.size()));
        for (const auto& c : z.cycles)
            b.insert(b.end(), c.begin(), c.end());
    }
    return b;
}

bool UserWavetableBank::unpackZones(const std::vector<float>& b, Record& r)
{
    size_t pos = 0;
    const auto count = [&](float v, size_t limit, size_t& out)
    {
        if (!(v >= 0.0f) || v > static_cast<float>(limit) || v != std::floor(v))
            return false;
        out = static_cast<size_t>(v);
        return true;
    };
    size_t norm = 0, zoneCount = 0;
    if (b.size() < 3 || b[pos++] != 1.0f || !count(b[pos++], 2, norm) || !count(b[pos++], kMaxWavetableZones, zoneCount) ||
        zoneCount < 2)
        return false;
    std::vector<WavetableZone> zones(zoneCount);
    for (auto& z : zones)
    {
        if (pos + 5 > b.size())
            return false;
        std::uint64_t bits = 0;
        for (int k = 0; k < 4; ++k)
        {
            size_t piece = 0;
            if (!count(b[pos++], 0xFFFF, piece))
                return false;
            bits |= static_cast<std::uint64_t>(piece) << (16 * k);
        }
        std::memcpy(&z.maxHz, &bits, sizeof bits);
        size_t cycleCount = 0;
        // 2^24 is the most the C API accepts (64M samples in all, cycles of at least 4) and the last count a float holds exactly.
        if (!count(b[pos++], 1U << 24, cycleCount) || cycleCount == 0 || pos + cycleCount > b.size())
            return false;
        std::vector<size_t> lengths(cycleCount);
        size_t total = 0;
        for (auto& len : lengths)
        {
            if (!count(b[pos++], 65536, len) || len < 4)
                return false;
            total += len;
        }
        if (total > b.size() - pos)
            return false;
        for (const size_t len : lengths)
        {
            z.cycles.emplace_back(b.begin() + static_cast<std::ptrdiff_t>(pos),
                                  b.begin() + static_cast<std::ptrdiff_t>(pos + len));
            pos += len;
        }
    }
    if (pos != b.size())
        return false;
    r.zones = std::move(zones);
    r.normalise = static_cast<int>(norm);
    return true;
}

void UserWavetableBank::acquire(Pinned& out) noexcept
{
    for (size_t s = 0; s < slots.size(); ++s)
        out[s] = slots[s].acquire();
}

void UserWavetableBank::release() noexcept
{
    for (auto& s : slots)
        s.release();
}

} // namespace crucible
