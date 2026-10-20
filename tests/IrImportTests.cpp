// SPDX-License-Identifier: MIT
#include <crucible/fx/IrImport.h>
#include "TestSupport.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

using namespace crucible;
using namespace crucible::testing;

namespace
{
constexpr double kPi = 3.14159265358979323846;

void put32(std::vector<std::uint8_t>& b, std::uint32_t v)
{
    for (int s = 24; s >= 0; s -= 8)
        b.push_back(static_cast<std::uint8_t>(v >> s));
}
void put16(std::vector<std::uint8_t>& b, std::uint16_t v)
{
    b.push_back(static_cast<std::uint8_t>(v >> 8));
    b.push_back(static_cast<std::uint8_t>(v));
}
void putTag(std::vector<std::uint8_t>& b, const char* tag)
{
    b.insert(b.end(), tag, tag + 4);
}

// 80-bit extended precision for positive integer rates.
void putExtended(std::vector<std::uint8_t>& b, double v)
{
    int exponent = 0;
    const double m = std::frexp(v, &exponent); // v = m * 2^exponent, m in [0.5, 1)
    const std::uint64_t mantissa = static_cast<std::uint64_t>(std::ldexp(m, 64));
    put16(b, static_cast<std::uint16_t>(exponent - 1 + 16383));
    for (int s = 56; s >= 0; s -= 8)
        b.push_back(static_cast<std::uint8_t>(mantissa >> s));
}

// kind: "NONE" (big-endian PCM), "sowt" (little-endian PCM), "fl32"; bits for PCM kinds. Samples are interleaved.
std::vector<std::uint8_t> makeAiff(const std::vector<float>& interleaved, int channels, double rate, int bits,
                                   const char* kind)
{
    const bool aifc = std::string(kind) != "AIFF";
    const bool isFloat = std::string(kind) == "fl32";
    const bool little = std::string(kind) == "sowt";
    const int bytes = isFloat ? 4 : bits / 8;
    std::vector<std::uint8_t> sound;
    for (const float v : interleaved)
    {
        std::vector<std::uint8_t> s;
        if (isFloat)
        {
            std::uint32_t u;
            std::memcpy(&u, &v, 4);
            put32(s, u);
        }
        else
        {
            const double scale = std::ldexp(1.0, bits - 1);
            const std::int64_t q = static_cast<std::int64_t>(std::llround(static_cast<double>(v) * scale));
            for (int i = bytes - 1; i >= 0; --i)
                s.push_back(static_cast<std::uint8_t>(q >> (8 * i)));
        }
        if (little)
            s = std::vector<std::uint8_t>(s.rbegin(), s.rend());
        sound.insert(sound.end(), s.begin(), s.end());
    }
    std::vector<std::uint8_t> comm;
    put16(comm, static_cast<std::uint16_t>(channels));
    put32(comm, static_cast<std::uint32_t>(interleaved.size() / static_cast<size_t>(channels)));
    put16(comm, static_cast<std::uint16_t>(isFloat ? 32 : bits));
    putExtended(comm, rate);
    if (aifc)
    {
        putTag(comm, std::string(kind) == "sowt" ? "sowt" : isFloat ? "fl32" : "NONE");
        comm.push_back(0); // empty pascal string, padded to even
        comm.push_back(0);
    }
    std::vector<std::uint8_t> file;
    putTag(file, "FORM");
    put32(file, 0);
    putTag(file, aifc ? "AIFC" : "AIFF");
    putTag(file, "COMM");
    put32(file, static_cast<std::uint32_t>(comm.size()));
    file.insert(file.end(), comm.begin(), comm.end());
    putTag(file, "SSND");
    put32(file, static_cast<std::uint32_t>(8 + sound.size()));
    put32(file, 0);
    put32(file, 0);
    file.insert(file.end(), sound.begin(), sound.end());
    if (file.size() & 1u)
        file.push_back(0);
    const std::uint32_t total = static_cast<std::uint32_t>(file.size() - 8);
    for (int i = 0; i < 4; ++i)
        file[4 + static_cast<size_t>(i)] = static_cast<std::uint8_t>(total >> (24 - 8 * i));
    return file;
}

double hann(double x)
{
    return x <= 0.0 || x >= 1.0 ? 0.0 : 0.5 - 0.5 * std::cos(2.0 * kPi * x);
}

// Relative error (dB) of the resampled Hann-windowed tone against the same tone evaluated analytically at the output
// instants.
double toneErrorDb(double srcRate, double dstRate, double freq)
{
    const size_t n = static_cast<size_t>(srcRate / 2);
    const double duration = static_cast<double>(n) / srcRate;
    std::vector<float> in(n);
    for (size_t i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) / srcRate;
        in[i] = static_cast<float>(hann(t / duration) * std::sin(2.0 * kPi * freq * t));
    }
    const std::vector<float> out = resampleKaiser(in, srcRate, dstRate);
    double err = 0.0, ref = 0.0;
    for (size_t m = 0; m < out.size(); ++m)
    {
        const double t = static_cast<double>(m) / dstRate;
        const double ideal = hann(t / duration) * std::sin(2.0 * kPi * freq * t);
        err += (static_cast<double>(out[m]) - ideal) * (static_cast<double>(out[m]) - ideal);
        ref += ideal * ideal;
    }
    return 10.0 * std::log10((err + 1.0e-300) / ref);
}

// Level (dB) of the output of a tone above the target Nyquist frequency, relative to the input tone.
double stopbandDb(double srcRate, double dstRate, double freq)
{
    const size_t n = static_cast<size_t>(srcRate / 2);
    const double duration = static_cast<double>(n) / srcRate;
    std::vector<float> in(n);
    double inPower = 0.0;
    for (size_t i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) / srcRate;
        in[i] = static_cast<float>(hann(t / duration) * std::sin(2.0 * kPi * freq * t));
        inPower += static_cast<double>(in[i]) * in[i];
    }
    const std::vector<float> out = resampleKaiser(in, srcRate, dstRate);
    double outPower = 0.0;
    for (const float v : out)
        outPower += static_cast<double>(v) * v;
    // Power per second of audio on each side.
    return 10.0 * std::log10((outPower * dstRate + 1.0e-300) / (inPower * srcRate));
}
} // namespace

class IrImportTests : public UnitTest
{
public:
    IrImportTests() : UnitTest("Impulse response import", "Fx") {}

    void runTest() override
    {
        beginTest("resampled images and aliases stay below -120 dB");
        {
            struct Pair
            {
                double from, to;
            };
            const std::vector<Pair> pairs = {{44100, 48000},  {48000, 44100}, {96000, 48000},   {192000, 48000},
                                             {22050, 48000},  {48000, 96000}, {44100, 96000},   {88200, 44100},
                                             {44100, 192000}, {32000, 44100}, {44100.5, 48000}, {48000, 47999.25}};
            double worst = -400.0;
            for (const auto& p : pairs)
            {
                const double lowNyquist = 0.5 * std::min(p.from, p.to);
                const double db = toneErrorDb(p.from, p.to, 0.9 * lowNyquist);
                const double dbEdge = toneErrorDb(p.from, p.to, 0.945 * lowNyquist);
                worst = std::max({worst, db, dbEdge});
                expect(db < -120.0 && dbEdge < -120.0, String(p.from) + " -> " + String(p.to) + " Hz: tone error " +
                                                           String(db) + " / " + String(dbEdge) + " dB");
            }
            std::cout << "  worst resampling error (tones at 0.9 and 0.945 of the lower Nyquist): " << worst << " dB\n";

            // Downsampling: content above the new Nyquist frequency (it would alias into the band) is removed.
            double worstStop = -400.0;
            for (const auto& p : {Pair{96000, 48000}, Pair{192000, 48000}, Pair{48000, 44100}, Pair{88200, 44100}})
            {
                const double stop = stopbandDb(p.from, p.to, 1.055 * 0.5 * p.to);
                worstStop = std::max(worstStop, stop);
                expect(stop < -120.0,
                       String(p.from) + " -> " + String(p.to) + " Hz: stopband tone leaves " + String(stop) + " dB");
            }
            std::cout << "  worst stopband leakage: " << worstStop << " dB\n";
        }

        beginTest("DC gain is one, equal rates are untouched, lengths follow the rate ratio");
        {
            const std::vector<float> dc(48000, 1.0f);
            const auto up = resampleKaiser(dc, 48000.0, 96000.0);
            expectEquals(static_cast<int>(up.size()), 96000);
            expect(std::abs(up[48000] - 1.0f) < 1.0e-6f, "DC at 2x");
            const auto down = resampleKaiser(dc, 48000.0, 44100.0);
            expectEquals(static_cast<int>(down.size()), 44100);
            expect(std::abs(down[20000] - 1.0f) < 1.0e-6f, "DC at 44.1 kHz");
            expect(resampleKaiser(dc, 48000.0, 48000.0) == dc, "equal rates");
            expect(resampleKaiser({}, 44100.0, 48000.0).empty(), "empty");
            std::vector<float> impulse(4410, 0.0f);
            impulse[441] = 1.0f; // 10 ms
            const auto moved = resampleKaiser(impulse, 44100.0, 48000.0);
            size_t peak = 0;
            for (size_t i = 0; i < moved.size(); ++i)
                if (std::abs(moved[i]) > std::abs(moved[peak]))
                    peak = i;
            expectEquals(static_cast<int>(peak), 480, "an impulse keeps its time");
            expect(std::abs(moved[peak] - 1.0f) < 1.0e-3f, "and its height");
        }

        beginTest("AIFF and AIFC decoding");
        {
            const std::vector<float> samples = {0.0f, 0.5f, -0.5f, 0.25f, -1.0f, 0.125f};
            for (const auto& c : {std::make_pair("AIFF", 16), std::make_pair("AIFF", 24), std::make_pair("AIFF", 32),
                                  std::make_pair("AIFF", 8), std::make_pair("sowt", 16), std::make_pair("fl32", 32)})
            {
                const auto file = makeAiff(samples, 2, 44100.0, c.second, c.first);
                AudioData a;
                std::string error;
                const bool ok = readAiff(file.data(), file.size(), a, error);
                expect(ok, String(c.first) + " " + String(c.second) + " bit: " + String(error));
                if (!ok)
                    continue;
                expectEquals(a.numChannels(), 2);
                expectEquals(a.numFrames(), 3);
                expectWithinAbsoluteError(a.sampleRate, 44100.0, 1.0e-9);
                double worst = 0.0;
                for (int ch = 0; ch < 2; ++ch)
                    for (int i = 0; i < 3; ++i)
                        worst = std::max(worst, std::abs(static_cast<double>(
                                                             a.channels[static_cast<size_t>(ch)][static_cast<size_t>(i)]) -
                                                         samples[static_cast<size_t>(2 * i + ch)]));
                expect(worst < (c.second == 8 ? 1.0e-2 : 1.0e-4),
                       String(c.first) + " " + String(c.second) + " bit values, error " + String(worst));
            }
            const std::uint8_t junk[16] = {'F', 'O', 'R', 'M', 0, 0, 0, 4, 'A', 'I', 'F', 'F', 0, 0, 0, 0};
            AudioData a;
            std::string error;
            expect(!readAiff(junk, sizeof(junk), a, error) && !error.empty(), "no COMM chunk");
            expect(!readAiff(junk, 6, a, error), "truncated");
        }

        beginTest("import: layouts, rate conversion, limits");
        {
            ImpulseResponse ir;
            std::string error;
            // Stereo AIFF at 44.1 kHz: an impulse in each channel at different positions.
            std::vector<float> st(2 * 4410, 0.0f);
            st[2 * 441] = 1.0f;
            st[2 * 882 + 1] = 0.5f;
            auto file = makeAiff(st, 2, 44100.0, 24, "AIFF");
            expect(importImpulseResponse(file.data(), file.size(), 48000.0, ir, error), error);
            expect(ir.layout == IrLayout::Stereo && ir.channels.size() == 2, "stereo layout");
            expectEquals(ir.frames(), 4800);
            expectWithinAbsoluteError(ir.sampleRate, 48000.0, 1.0e-9);
            size_t pl = 0, pr = 0;
            for (size_t i = 0; i < 4800; ++i)
            {
                if (std::abs(ir.channels[0][i]) > std::abs(ir.channels[0][pl]))
                    pl = i;
                if (std::abs(ir.channels[1][i]) > std::abs(ir.channels[1][pr]))
                    pr = i;
            }
            expectEquals(static_cast<int>(pl), 480, "left impulse at 10 ms");
            expectEquals(static_cast<int>(pr), 960, "right impulse at 20 ms");

            // WAV, mono and four channels (LL, LR, RL, RR), no rate change.
            AudioData wav;
            wav.sampleRate = 48000.0;
            wav.channels.assign(1, std::vector<float>(1000, 0.0f));
            wav.channels[0][10] = 0.75f;
            auto bytes = writeWav(wav, 32);
            expect(importImpulseResponse(bytes.data(), bytes.size(), 48000.0, ir, error), error);
            expect(ir.layout == IrLayout::Mono && ir.channels.size() == 1 && ir.channels[0] == wav.channels[0],
                   "mono, same rate, exact");
            wav.channels.assign(4, std::vector<float>(1000, 0.0f));
            for (size_t c = 0; c < 4; ++c)
                wav.channels[c][20 + c] = 1.0f;
            bytes = writeWav(wav, 32);
            expect(importImpulseResponse(bytes.data(), bytes.size(), 96000.0, ir, error), error);
            expect(ir.layout == IrLayout::TrueStereo && ir.channels.size() == 4, "true stereo layout");
            for (size_t c = 0; c < 4; ++c)
            {
                size_t peak = 0;
                for (size_t i = 0; i < ir.channels[c].size(); ++i)
                    if (std::abs(ir.channels[c][i]) > std::abs(ir.channels[c][peak]))
                        peak = i;
                expectEquals(static_cast<int>(peak), static_cast<int>(2 * (20 + c)),
                             "channel " + String(static_cast<int>(c)) + " keeps its order");
            }

            wav.channels.assign(3, std::vector<float>(100, 0.0f));
            bytes = writeWav(wav, 32);
            expect(!importImpulseResponse(bytes.data(), bytes.size(), 48000.0, ir, error) && !error.empty(),
                   "3 channels rejected");

            AudioData longIr;
            longIr.sampleRate = 8000.0;
            longIr.channels.assign(1, std::vector<float>(static_cast<size_t>(31.0 * 8000.0), 0.0f));
            bytes = writeWav(longIr, 16);
            expect(!importImpulseResponse(bytes.data(), bytes.size(), 48000.0, ir, error), "31 s rejected");
            longIr.channels[0].resize(static_cast<size_t>(30.0 * 8000.0));
            bytes = writeWav(longIr, 16);
            expect(importImpulseResponse(bytes.data(), bytes.size(), 48000.0, ir, error), "30 s accepted");
            expectEquals(ir.frames(), 30 * 48000);

            const std::uint8_t text[16] = {'h', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd', '!', '!', '!', '!', '!'};
            expect(!importImpulseResponse(text, sizeof(text), 48000.0, ir, error), "unknown type");
            expect(!importImpulseResponse(bytes.data(), bytes.size(), 100.0, ir, error), "bad target rate");
            expect(!importImpulseResponseFile("/nonexistent/ir.wav", 48000.0, ir, error), "missing file");
        }
    }
};

static IrImportTests irImportTests;
