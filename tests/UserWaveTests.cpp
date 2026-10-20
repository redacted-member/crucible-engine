// SPDX-License-Identifier: MIT
#include <crucible/AudioFile.h>
#include <crucible/Cloud.h>
#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/PresetLibrary.h>
#include <crucible/UserWavetables.h>
#include "TestSupport.h"

#include <complex>
#include <cstdio>
#include <filesystem>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kTwoPi = 6.283185307179586;

std::vector<float> squareCycle(int length)
{
    std::vector<float> v(static_cast<size_t>(length));
    for (int i = 0; i < length; ++i)
        v[static_cast<size_t>(i)] = i < length / 2 ? 0.8f : -0.8f;
    return v;
}

std::vector<float> sineCycle(int length, int harmonic = 1, float amp = 1.0f)
{
    std::vector<float> v(static_cast<size_t>(length));
    for (int i = 0; i < length; ++i)
        v[static_cast<size_t>(i)] = amp * static_cast<float>(std::sin(kTwoPi * harmonic * i / length));
    return v;
}

constexpr int kL0 = WavetableBank::mipSize(0); // samples of the finest level

// Amplitude of harmonic h of a frame of `size` samples.
double harmonicAmp(const float* frame, int size, int h)
{
    double re = 0.0, im = 0.0;
    for (int i = 0; i < size; ++i)
    {
        re += frame[i] * std::cos(kTwoPi * h * i / size);
        im += frame[i] * std::sin(kTwoPi * h * i / size);
    }
    return 2.0 * std::sqrt(re * re + im * im) / size;
}

Rendered renderNote(SynthEngine& e, int note, int samples)
{
    e.noteOn(note, 1.0f);
    return renderBlocks(e, samples, 256, true);
}

// Spectral centroid (Hz) and RMS of the last 16384 samples of a render (Hann window, radix-2 FFT).
struct Spectrum
{
    double centroid = 0.0;
    double rms = 0.0;
};

Spectrum analyse(const std::vector<float>& x, double rate)
{
    constexpr size_t n = 16384;
    Spectrum s;
    if (x.size() < n)
        return s;
    std::vector<std::complex<double>> a(n);
    for (size_t i = 0; i < n; ++i)
    {
        const double v = x[x.size() - n + i];
        s.rms += v * v;
        a[i] = v * (0.5 - 0.5 * std::cos(kTwoPi * static_cast<double>(i) / n));
    }
    s.rms = std::sqrt(s.rms / n);
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1)
    {
        const std::complex<double> wl = std::polar(1.0, -kTwoPi / static_cast<double>(len));
        for (size_t i = 0; i < n; i += len)
        {
            std::complex<double> w = 1.0;
            for (size_t k = 0; k < len / 2; ++k, w *= wl)
            {
                const auto u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v;
                a[i + k + len / 2] = u - v;
            }
        }
    }
    double num = 0.0, den = 0.0;
    for (size_t k = 1; k < n / 2; ++k)
    {
        const double m = std::norm(a[k]);
        num += m * static_cast<double>(k) * rate / n;
        den += m;
    }
    s.centroid = den > 0.0 ? num / den : 0.0;
    return s;
}
} // namespace

class UserWaveTests : public UnitTest
{
public:
    UserWaveTests() : UnitTest("User wavetables and cloud", "Engine") {}

    void runTest() override
    {
        const ScopedFlushDenormals noDenormals;

        beginTest("WAV round trip (16-bit, float, stereo) and the clm frame marker");
        {
            AudioData a;
            a.sampleRate = 44100.0;
            a.channels = {sineCycle(1000), squareCycle(1000)};
            std::string error;
            for (const int bits : {16, 32})
            {
                const auto bytes = writeWav(a, bits);
                AudioData b;
                expect(readWav(bytes.data(), bytes.size(), b, error), error);
                expect(b.numChannels() == 2 && b.numFrames() == 1000 && b.sampleRate == 44100.0);
                float maxDiff = 0.0f;
                for (int c = 0; c < 2; ++c)
                    for (int i = 0; i < 1000; ++i)
                        maxDiff = std::max(
                            maxDiff, std::abs(b.channel(c)[static_cast<size_t>(i)] - a.channel(c)[static_cast<size_t>(i)]));
                expect(maxDiff < (bits == 16 ? 1.0e-4f : 1.0e-7f), "bits " + String(bits) + " diff " + String(maxDiff));
            }
            auto bytes = writeWav(a, 32);
            const std::string clm = "<!>256 10000000 wavetable";
            std::vector<std::uint8_t> chunk = {'c', 'l', 'm', ' ', static_cast<std::uint8_t>(clm.size()), 0, 0, 0};
            chunk.insert(chunk.end(), clm.begin(), clm.end());
            if (clm.size() % 2 == 1)
                chunk.push_back(0);
            bytes.insert(bytes.begin() + 12, chunk.begin(), chunk.end());
            const auto riff = static_cast<std::uint32_t>(bytes.size() - 8);
            for (int i = 0; i < 4; ++i)
                bytes[static_cast<size_t>(4 + i)] = static_cast<std::uint8_t>((riff >> (8 * i)) & 0xffU);
            AudioData c;
            expect(readWav(bytes.data(), bytes.size(), c, error) && c.cycleLength == 256, error);
            AudioData bad;
            const std::uint8_t junk[16] = {'R', 'I', 'F', 'F'};
            expect(!readWav(junk, sizeof(junk), bad, error));
        }

        beginTest("import: any cycle length is resampled spectrally, band-limited per mip, DC removed, normalised");
        {
            auto cycle = squareCycle(600); // not a power of two
            for (auto& x : cycle)
                x += 0.3f; // DC offset
            auto t = UserWavetable::fromCycles({cycle}, WavetableImport::Normalise::Table, "square");
            const float* f0 = t->frame(0, 0);
            double mean = 0.0;
            float peak = 0.0f;
            for (int i = 0; i < kL0; ++i)
            {
                mean += f0[i];
                peak = std::max(peak, std::abs(f0[i]));
            }
            expect(std::abs(mean / static_cast<double>(kL0)) < 1.0e-4, "DC removed");
            expectWithinAbsoluteError(peak, 1.0f, 1.0e-3f);
            const double h1 = harmonicAmp(f0, kL0, 1);
            expectWithinAbsoluteError(harmonicAmp(f0, kL0, 3) / h1, 1.0 / 3.0, 0.01);
            expect(harmonicAmp(f0, kL0, 2) / h1 < 1.0e-3, "even harmonics absent");
            expect(harmonicAmp(f0, kL0, 301) < 1.0e-5, "nothing above the source's Nyquist (300)");
            // The level with 34 partials keeps harmonics up to 34 only (the source has 299).
            const float* m5 = t->frame(11, 0);
            const int size5 = WavetableBank::mipSize(11);
            expectEquals(WavetableBank::mipHarmonics(11), 34);
            expect(harmonicAmp(m5, size5, 31) > 1.0e-3 && harmonicAmp(m5, size5, 35) < 1.0e-4);
            expectEquals(t->sourceFrameCount(), 1);
        }

        beginTest("import: several frames spread over 64 with crossfades; slicing honours the frame size");
        {
            AudioData a;
            a.channels.resize(1);
            for (const int h : {1, 2, 3})
            {
                const auto c = sineCycle(2048, h);
                a.channels[0].insert(a.channels[0].end(), c.begin(), c.end());
            }
            WavetableImport opt;
            const auto cycles = sliceCycles(a, opt);
            expectEquals(static_cast<int>(cycles.size()), 3);
            auto t = UserWavetable::fromCycles(cycles, WavetableImport::Normalise::Frame, "three");
            expect(harmonicAmp(t->frame(0, 0), kL0, 1) > 0.99 && harmonicAmp(t->frame(0, 0), kL0, 2) < 1.0e-3);
            expect(harmonicAmp(t->frame(0, 63), kL0, 3) > 0.99);
            const float* mid = t->frame(0, 31); // between sources 0 and 1 (position 0.98 of the way)
            expect(harmonicAmp(mid, kL0, 1) > 0.01 && harmonicAmp(mid, kL0, 2) > 0.5);
            opt.frameSize = 1024;
            expectEquals(static_cast<int>(sliceCycles(a, opt).size()), 6);
            AudioData small;
            small.channels = {sineCycle(700)};
            expectEquals(static_cast<int>(sliceCycles(small, WavetableImport{}).size()), 1);
        }

        beginTest("user slots play through the oscillator; empty slots are silent; serialisation round-trips");
        {
            auto v = ParameterMap::defaultValues();
            setValue(v, "osc1_table", static_cast<float>(WavetableBank::numTables + 2)); // User 3
            setValue(v, "filter_cutoff", 20000.0f);
            SynthEngine e;
            prepare(e, v);
            expect(renderNote(e, 60, 4800).peak == 0.0f, "empty user slot is silent");
            e.allNotesOff();
            renderBlocks(e, 48000);
            expect(e.userWavetables().loadCycles(2, {squareCycle(512)}, WavetableImport::Normalise::Table, "sq"));
            expect(e.userWavetables().isLoaded(2) && e.userWavetables().name(2) == "sq");
            const auto r = renderNote(e, 60, 9600);
            expect(r.finite && r.peak > 0.05f, "user square plays " + String(r.peak));
            const auto records = e.userWavetables().serialise();
            expectEquals(static_cast<int>(records.size()), 1);
            UserWavetableBank other;
            other.deserialise(records);
            const auto* a = e.userWavetables().table(2);
            const auto* b = other.table(2);
            expect(b != nullptr && b->name() == "sq");
            float diff = 0.0f;
            for (int mip = 0; mip < WavetableBank::numMips; ++mip)
                for (int f = 0; f < WavetableBank::numFrames; f += 9)
                    for (int i = 0; i < WavetableBank::mipSize(mip); ++i)
                        diff = std::max(diff, std::abs(a->frame(mip, f)[i] - b->frame(mip, f)[i]));
            expect(diff < 1.0e-4f, "round trip diff " + String(diff));
            e.userWavetables().clear(2);
            expect(!e.userWavetables().isLoaded(2));
            expectEquals(allTableNames().back(), std::string("User 8"));
        }

        beginTest("zones: one zone is exactly the plain table; two zones pick their source per mip and per sample rate");
        {
            const auto cyc = std::vector<std::vector<float>>{squareCycle(600), sineCycle(512, 5)};
            auto plain = UserWavetable::fromCycles(cyc, WavetableImport::Normalise::Table, "p");
            auto one = UserWavetable::fromZones({{cyc, 1000.0}}, WavetableImport::Normalise::Table, "p", 96000.0);
            expect(plain != nullptr && one != nullptr);
            bool same = one->baseFrames() == plain->baseFrames();
            for (int mip = 0; mip < WavetableBank::numMips && same; ++mip)
                for (int f = 0; f < WavetableBank::numFrames && same; ++f)
                    for (int i = 0; i < WavetableBank::mipSize(mip); ++i)
                        if (one->frame(mip, f)[i] != plain->frame(mip, f)[i])
                        {
                            same = false;
                            break;
                        }
            expect(same, "one zone is bit-identical to fromCycles");
            expectEquals(one->zoneCount(), 1);
            expect(one->rebuiltForRate(48000.0) == nullptr, "a one-zone table does not depend on the rate");

            // Zone 0 (up to 500 Hz) is the 2nd harmonic, zone 1 the 3rd. Frames are normalised together, so the
            // zones keep equal levels.
            const std::vector<WavetableZone> zones = {{{sineCycle(2048, 2)}, 500.0}, {{sineCycle(2048, 3)}, 4000.0}};
            for (const double rate : {48000.0, 96000.0})
            {
                auto z = UserWavetable::fromZones(zones, WavetableImport::Normalise::Table, "z", rate);
                expect(z != nullptr && z->zoneCount() == 2);
                const float limit = WavetableBank::outputBandLimit(rate);
                for (const double hz : {65.4, 130.8, 2093.0, 8000.0})
                {
                    const int mip = WavetableBank::mipForIncrement(static_cast<float>(hz / rate), limit);
                    const int size = WavetableBank::mipSize(mip);
                    const double a2 = harmonicAmp(z->frame(mip, 0), size, 2);
                    const double a3 = harmonicAmp(z->frame(mip, 0), size, 3);
                    const bool low = hz < 500.0;
                    expect(low ? (a2 > 0.9 && a3 < 1.0e-3) : (a3 > 0.9 && a2 < 1.0e-3),
                           "rate " + String(rate) + " hz " + String(hz) + " mip " + String(mip) + " h2 " + String(a2) +
                               " h3 " + String(a3));
                }
            }
            // The map from level to pitch moves with the rate: a table built for 48 kHz is rebuilt for 96 kHz.
            auto z48 = UserWavetable::fromZones(zones, WavetableImport::Normalise::Table, "z", 48000.0);
            auto z96 = z48->rebuiltForRate(96000.0);
            expect(z96 != nullptr && z96->sampleRate() == 96000.0 && z96->zoneCount() == 2);
            auto ref96 = UserWavetable::fromZones(zones, WavetableImport::Normalise::Table, "z", 96000.0);
            bool eq = true;
            for (int mip = 0; mip < WavetableBank::numMips; ++mip)
                for (int i = 0; i < WavetableBank::mipSize(mip); ++i)
                    eq = eq && z96->frame(mip, 3)[i] == ref96->frame(mip, 3)[i];
            expect(eq, "rebuilt table equals one built at that rate");
            // Refusals.
            expect(UserWavetable::fromZones({}, WavetableImport::Normalise::Table, "x", 48000.0) == nullptr);
            expect(UserWavetable::fromZones({{{sineCycle(64)}, 500.0}, {{sineCycle(64)}, 400.0}},
                                            WavetableImport::Normalise::Table, "x", 48000.0) == nullptr, "unsorted");
            expect(UserWavetable::fromZones({{{sineCycle(64)}, -1.0}}, WavetableImport::Normalise::Table, "x",
                                            48000.0) == nullptr, "non-positive");
            expect(UserWavetable::fromZones({{{}, 500.0}}, WavetableImport::Normalise::Table, "x", 48000.0) == nullptr,
                   "empty zone");
        }

        beginTest("zones play through the oscillator: low notes read zone 0, high notes zone 1, at 48 and 96 kHz");
        {
            const std::vector<WavetableZone> zones = {{{sineCycle(2048, 2)}, 500.0}, {{sineCycle(2048, 3)}, 4000.0}};
            for (const double rate : {48000.0, 96000.0})
            {
                auto v = ParameterMap::defaultValues();
                setValue(v, "osc1_table", static_cast<float>(WavetableBank::numTables));
                setValue(v, "filter_cutoff", 20000.0f);
                SynthEngine e;
                e.prepare(rate);
                e.ensureSelectedTables(v.data(), static_cast<int>(v.size()));
                e.setParameterValues(v.data(), static_cast<int>(v.size()));
                expect(e.userWavetables().loadZones(0, zones, WavetableImport::Normalise::Table, "z"));
                for (const int note : {36, 96})
                {
                    const double f0 = 440.0 * std::pow(2.0, (note - 69) / 12.0);
                    e.noteOn(note, 1.0f);
                    const auto r = renderBlocks(e, static_cast<int>(rate), 256, true);
                    const size_t n = static_cast<size_t>(rate / 2);
                    const size_t start = r.left.size() - n;
                    auto power = [&](double hz)
                    {
                        double re = 0.0, im = 0.0;
                        for (size_t i = 0; i < n; ++i)
                        {
                            const double w = 6.283185307179586 * hz * static_cast<double>(i) / rate;
                            re += r.left[start + i] * std::cos(w);
                            im += r.left[start + i] * std::sin(w);
                        }
                        return std::sqrt(re * re + im * im);
                    };
                    const double p2 = power(2.0 * f0), p3 = power(3.0 * f0);
                    expect(note < 60 ? p2 > 8.0 * p3 : p3 > 8.0 * p2,
                           "rate " + String(rate) + " note " + String(note) + " p2 " + String(p2) + " p3 " + String(p3));
                    e.allNotesOff();
                    renderBlocks(e, static_cast<int>(rate));
                }
            }
        }

        beginTest("zones in patches: bit-stable save / load / save, other rates, old patches, malformed zones, asset store");
        {
            // Irrational max_hz, cycles of different lengths and counts: nothing may be rounded on the way.
            const std::vector<WavetableZone> zones = {{{sineCycle(1000, 2), squareCycle(777)}, 261.6255653005986},
                                                      {{sineCycle(2048, 3, 0.5f)}, 1046.5022612023945},
                                                      {{sineCycle(300, 5), sineCycle(300, 7)}, 7000.123456789}};
            Engine a;
            a.prepare(kRate, 256);
            expect(a.userWavetables().loadZones(1, zones, WavetableImport::Normalise::Frame, "zoned"));
            expect(a.userWavetables().loadCycles(0, {sineCycle(512)}, WavetableImport::Normalise::Table, "plain"));
            const std::string first = patchToJson(capturePatch(a));
            expect(first.find("\"zones\"") != std::string::npos, "the zoned table writes its zones");
            std::string previous = first;
            for (int round = 0; round < 3; ++round)
            {
                Patch p;
                std::string err;
                expect(patchFromJson(previous, p, err), err);
                Engine b;
                b.prepare(kRate, 256);
                applyPatch(b, p);
                const std::string next = patchToJson(capturePatch(b));
                expect(next == previous, "save / load / save identical (round " + std::to_string(round) + ")");
                previous = next;
            }
            Patch p;
            std::string err;
            expect(patchFromJson(first, p, err), err);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            const auto* orig = a.userWavetables().table(1);
            const auto* back = b.userWavetables().table(1);
            expect(orig != nullptr && back != nullptr && back->zoneCount() == 3 && back->zones().size() == 3, "zoned table restored");
            if (orig != nullptr && back != nullptr && back->zones().size() == 3)
            {
                bool same = back->baseFrames() == orig->baseFrames();
                for (int m = 0; m < WavetableBank::numMips && same; ++m)
                    same = std::memcmp(back->frame(m, 10), orig->frame(m, 10),
                                       sizeof(float) * static_cast<size_t>(WavetableBank::mipSize(m))) == 0;
                expect(same, "frames and every mip level are bit-identical after load");
                expect(back->zones()[0].maxHz == zones[0].maxHz && back->zones()[2].maxHz == zones[2].maxHz,
                       "max_hz is exact");
                expect(back->zones()[0].cycles == zones[0].cycles && back->zones()[2].cycles == zones[2].cycles,
                       "zone cycles are exact");
            }
            expect(b.userWavetables().table(0) != nullptr && b.userWavetables().table(0)->zoneCount() == 1, "plain table stays plain");

            // Loaded into an engine at another rate: built for that rate, like a direct load.
            Engine c;
            c.prepare(96000.0, 256);
            applyPatch(c, p);
            auto want96 = UserWavetable::fromZones(zones, WavetableImport::Normalise::Frame, "zoned", 96000.0);
            const auto* got96 = c.userWavetables().table(1);
            expect(got96 != nullptr && want96 != nullptr && got96->sampleRate() == 96000.0 &&
                       got96->baseFrames() == want96->baseFrames(),
                   "a patch loads built for the engine's rate");

            // Old patches: a one-zone table never writes zones; a patch without them loads as before.
            Engine one;
            one.prepare(kRate, 256);
            expect(one.userWavetables().loadCycles(2, {sineCycle(512), squareCycle(512)}, WavetableImport::Normalise::Table, "one"));
            const std::string oldText = patchToJson(capturePatch(one));
            expect(oldText.find("\"zones\"") == std::string::npos, "one-zone tables write the old format");
            Patch oldPatch;
            expect(patchFromJson(oldText, oldPatch, err), err);
            Engine two;
            two.prepare(kRate, 256);
            applyPatch(two, oldPatch);
            expect(patchToJson(capturePatch(two)) == oldText, "an old-format patch loads and saves unchanged");

            // A zoned patch with its zones removed, or damaged, still loads (as the one-zone frames).
            const auto key = first.find("\"zones\"");
            expect(key != std::string::npos, "zones key found");
            if (key != std::string::npos)
            {
                std::string damaged = first;
                const auto open = damaged.find('"', damaged.find(':', key) + 1);
                damaged.replace(open + 1, 4, "AAAA");
                Patch dp;
                expect(patchFromJson(damaged, dp, err), err);
                expect(dp.userWaves.size() == 2, "damaged patch keeps both tables");
                Engine d;
                d.prepare(kRate, 256);
                applyPatch(d, dp);
                const auto* t = d.userWavetables().table(1);
                expect(t != nullptr && t->zoneCount() == 1 && t->baseFrames() == orig->baseFrames(),
                       "a damaged zones blob falls back to the saved frames");
            }

            // The preset library's asset store keeps the zones too.
            const std::string store = (std::filesystem::temp_directory_path() / "crucible_wtzone_assets").string();
            std::filesystem::remove_all(store);
            Patch ext = capturePatch(a);
            expect(externaliseAssets(ext, AssetStore(store)), "externalise");
            Patch ext2;
            expect(patchFromJson(patchToJson(ext), ext2, err), err);
            std::vector<std::string> missing;
            AssetStore as(store);
            resolveAssets(ext2, {&as}, missing);
            expect(missing.empty() && ext2.assetRefs.empty(), "all asset refs resolve");
            Engine f;
            f.prepare(kRate, 256);
            applyPatch(f, ext2);
            expect(patchToJson(capturePatch(f)) == first, "asset-store round trip is identical");
            std::filesystem::remove_all(store);
        }

        beginTest("a zoned table with more than 2^20 frames in a zone saves and reloads (load and unpack limits agree)");
        {
            const int many = (1 << 20) + 5;
            std::vector<WavetableZone> big(2);
            big[0].maxHz = 400.0;
            big[0].cycles.assign(static_cast<size_t>(many), sineCycle(4));
            big[1].maxHz = 3000.0;
            big[1].cycles = {sineCycle(256)};
            Engine a;
            a.prepare(kRate, 256);
            expect(a.userWavetables().loadZones(0, big, WavetableImport::Normalise::Table, "big"), "the load accepts it");
            const std::string text = patchToJson(capturePatch(a));
            Patch p;
            std::string err;
            expect(patchFromJson(text, p, err), err);
            Engine b;
            b.prepare(kRate, 256);
            applyPatch(b, p);
            const auto* t = b.userWavetables().table(0);
            expect(t != nullptr && t->zoneCount() == 2 && t->zones().size() == 2 &&
                       t->zones()[0].cycles.size() == static_cast<size_t>(many),
                   "the saved zoned table reloads as zoned");
        }

        beginTest("cloud: weights morph along z, wavetable generation, serialisation, determinism");
        {
            CloudModel m;
            m.spread = 0.1f;
            m.points = {{1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f}, {5.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f}};
            const auto cycles = cloudToCycles(m);
            expectEquals(static_cast<int>(cycles.size()), 64);
            const auto t = UserWavetable::fromCycles(cycles, WavetableImport::Normalise::Frame, "cloud");
            expect(harmonicAmp(t->frame(0, 0), kL0, 1) > 0.9 && harmonicAmp(t->frame(0, 0), kL0, 5) < 1.0e-3);
            expect(harmonicAmp(t->frame(0, 63), kL0, 5) > 0.9 && harmonicAmp(t->frame(0, 63), kL0, 1) < 1.0e-3);

            CloudModel back;
            m.name = "two points";
            expect(CloudModel::deserialise(m.serialise(), back));
            expect(back.points.size() == 2 && back.name == "two points" && std::abs(back.spread - 0.1f) < 1.0e-6f);
            expect(!CloudModel::deserialise("nonsense", back));
            CloudModel wild;
            wild.points = {{-5.0f, 9.0f, 3.0f, 7.25f, 4.0f, -1.0f}};
            wild.sanitise();
            const auto& w = wild.points[0];
            expect(w.x == 1.0f && w.y == 1.0f && w.z == 1.0f && std::abs(w.phase - 0.25f) < 1.0e-6f && w.pan == 1.0f &&
                   w.random == 0.0f);
            const auto r1 = CloudModel::random(100, 42);
            const auto r2 = CloudModel::random(100, 42);
            expect(r1.serialise() == r2.serialise(), "seeded clouds are deterministic");
        }

        beginTest("cloud oscillator: additive render, band-limited, snapshot for a 3D view, deterministic");
        {
            CloudOscillator osc;
            osc.prepare(kRate);
            auto model = CloudModel::harmonicMorph(200);
            osc.setModel(model);
            const auto render = [&](float freq, float pos, std::uint32_t seed)
            {
                CloudOscillator o;
                o.prepare(kRate);
                o.setModel(model);
                o.noteOn(freq, seed);
                std::vector<float> l(20480, 0.0f), r(20480, 0.0f);
                for (int p = 0; p < 20480; p += 128)
                    o.render(l.data() + p, r.data() + p, 128, pos, freq);
                return l;
            };
            const auto a = render(220.0f, 0.0f, 7);
            const auto b = render(220.0f, 0.0f, 7);
            expect(a == b, "same seed, same output");
            float peak = 0.0f;
            for (const float x : a)
                peak = std::max(peak, std::abs(x));
            expect(peak > 0.05f && peak <= 1.5f, "peak " + String(peak));
            // At 2 kHz only partials below 21.6 kHz sound (the 200-partial saw keeps 10): no aliases.
            model.points.resize(200); // the harmonic series only (the bell partials are inharmonic by design)
            const auto hi = render(2000.0f, 0.0f, 7);
            model = CloudModel::harmonicMorph(200);
            const double alias = inharmonicEnergyDb(hi, 2000.0, kRate, 0);
            expect(alias < -80.0, "inharmonic energy " + String(alias));
            osc.noteOn(220.0f, 3);
            std::vector<float> l(256, 0.0f), r(256, 0.0f);
            osc.render(l.data(), r.data(), 256, 0.9f, 220.0f);
            const auto snap = osc.snapshot();
            expect(snap.points.size() == model.points.size() && snap.energy.size() == snap.points.size());
            expectWithinAbsoluteError(snap.position, 0.9f, 1.0e-6f);
            // Near z = 1 the bell partials (the last six points) dominate the energy.
            float bell = 0.0f, saw = 0.0f;
            for (size_t i = 0; i < snap.energy.size(); ++i)
                (i + 6 >= snap.energy.size() ? bell : saw) += snap.energy[i];
            expect(bell > saw, "bell " + String(bell) + " saw " + String(saw));
            expect(snap.activeNotes == 1);
        }

        beginTest("zones keep their zone when the oscillator renders oversampled (FM, sync, Quality High)");
        {
            // Alternating dark (sine) and bright (square) zones, one per octave: reading the wrong zone shows as a
            // change of the centroid.
            std::vector<WavetableZone> zones;
            double maxHz = 100.0;
            for (int i = 0; i < 6; ++i, maxHz *= 2.0)
                zones.push_back({{i % 2 == 0 ? sineCycle(2048, 2) : squareCycle(2048)}, maxHz});
            for (const bool zoned : {true, false})
                for (const Quality quality : {Quality::Standard, Quality::High})
                    for (const int note : {36, 48, 60, 72})
                    {
                        EngineConfig cfg;
                        cfg.quality = quality;
                        const auto render = [&](int xmode, float amt)
                        {
                            auto v = ParameterMap::defaultValues();
                            setValue(v, "osc1_level", 0.0f);
                            setValue(v, "osc2_level", 0.5f);
                            setValue(v, "osc2_table", static_cast<float>(WavetableBank::numTables));
                            setValue(v, "osc2_xmod_mode", static_cast<float>(xmode));
                            setValue(v, "osc2_xmod_src", 0.0f);
                            setValue(v, "osc2_xmod_amt", amt);
                            setValue(v, "filter_on", 0.0f);
                            setValue(v, "amp_attack", 0.001f);
                            setValue(v, "amp_sustain", 1.0f);
                            SynthEngine e(cfg);
                            e.prepare(48000.0);
                            e.ensureSelectedTables(v.data(), static_cast<int>(v.size()));
                            e.setParameterValues(v.data(), static_cast<int>(v.size()));
                            if (zoned)
                                e.userWavetables().loadZones(0, zones, WavetableImport::Normalise::Table, "z");
                            else
                                e.userWavetables().loadCycles(0, {sineCycle(2048, 2), squareCycle(2048)},
                                                              WavetableImport::Normalise::Table, "one");
                            e.noteOn(note, 1.0f);
                            return analyse(renderBlocks(e, 48000, 256, true).left, 48000.0);
                        };
                        const Spectrum plain = render(0, 0.0f);
                        for (const int xmode : {1, 3})
                        {
                            const Spectrum x = render(xmode, 0.005f);
                            const String tag = String(zoned ? "zoned" : "one zone") +
                                               (quality == Quality::High ? " High" : "") + " note " + String(note) +
                                               (xmode == 1 ? " FM" : " sync") + ": centroid " + String(plain.centroid, 0) +
                                               " -> " + String(x.centroid, 0) + ", rms " + String(plain.rms, 4) + " -> " +
                                               String(x.rms, 4);
                            expect(plain.rms > 1.0e-3, "plain renders " + tag);
                            if (xmode == 1)
                            {
                                expect(std::abs(x.centroid / plain.centroid - 1.0) < 0.05, tag);
                                expect(std::abs(x.rms / plain.rms - 1.0) < 0.15, tag);
                            }
                            else
                            {
                                // Sync keeps its tighter limit (fewer partials): never brighter, no level jump.
                                expect(x.centroid <= plain.centroid * 1.03, tag);
                                expect(std::abs(20.0 * std::log10(x.rms / plain.rms)) < 3.0, tag);
                            }
                        }
                    }
        }
    }
};

static UserWaveTests userWaveTests;
