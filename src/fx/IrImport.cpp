// SPDX-License-Identifier: MIT
#include <crucible/fx/IrImport.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <numeric>

#include "Utf8Path.h"

namespace crucible
{
namespace
{
constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kStopbandDb = 140.0; // design attenuation (the requirement is 120 dB)
constexpr double kTransition = 0.1;   // transition band as a fraction of the lower Nyquist frequency
constexpr int kMaxPhases = 16384;     // largest exact polyphase table

std::uint32_t be32(const std::uint8_t* p)
{
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}
std::uint16_t be16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

// 80-bit IEEE extended precision, big endian.
double readExtended(const std::uint8_t* p)
{
    const int exponent = ((p[0] & 0x7f) << 8) | p[1];
    std::uint64_t mantissa = 0;
    for (int i = 0; i < 8; ++i)
        mantissa = (mantissa << 8) | p[2 + i];
    if (exponent == 0 && mantissa == 0)
        return 0.0;
    const double v = std::ldexp(static_cast<double>(mantissa), exponent - 16383 - 63);
    return (p[0] & 0x80) != 0 ? -v : v;
}

bool fail(std::string& error, const char* message)
{
    error = message;
    return false;
}

// Modified Bessel function of the first kind, order 0 (power series; converges for the arguments used here).
double besselI0(double x)
{
    double sum = 1.0, term = 1.0;
    const double q = 0.25 * x * x;
    for (int k = 1; k < 200; ++k)
    {
        term *= q / (static_cast<double>(k) * static_cast<double>(k));
        sum += term;
        if (term < 1.0e-20 * sum)
            break;
    }
    return sum;
}

double kaiserBeta(double attenuationDb)
{
    return 0.1102 * (attenuationDb - 8.7);
}

double sinc(double x)
{
    return std::abs(x) < 1.0e-12 ? 1.0 : std::sin(kPi * x) / (kPi * x);
}

// Prototype low-pass in source-sample units: unit DC gain, cutoff at `scale` * Nyquist of the source, Kaiser window of
// half-width `halfWidth` source samples.
struct Prototype
{
    double scale, halfWidth, beta, invI0;
    double at(double u) const
    {
        const double r = u / halfWidth;
        if (std::abs(r) >= 1.0)
            return 0.0;
        return scale * sinc(scale * u) * besselI0(beta * std::sqrt(1.0 - r * r)) * invI0;
    }
};

Prototype makePrototype(double srcRate, double dstRate)
{
    Prototype p{};
    p.scale = std::min(1.0, dstRate / srcRate);
    p.beta = kaiserBeta(kStopbandDb);
    p.invI0 = 1.0 / besselI0(p.beta);
    // Kaiser's length formula N = (A - 7.95) / (2.285 * dw), dw = 2 pi * transition (cycles / source sample).
    const double transition = kTransition * 0.5 * p.scale;
    const double taps = (kStopbandDb - 7.95) / (2.285 * 2.0 * kPi * transition);
    p.halfWidth = std::ceil(0.5 * taps) + 1.0;
    return p;
}

bool nearInteger(double v)
{
    return std::abs(v - std::round(v)) < 1.0e-9 * std::max(1.0, std::abs(v));
}
} // namespace

bool readAiff(const std::uint8_t* data, std::size_t size, AudioData& out, std::string& error)
{
    out = AudioData{};
    if (data == nullptr || size < 12 || std::memcmp(data, "FORM", 4) != 0 ||
        (std::memcmp(data + 8, "AIFF", 4) != 0 && std::memcmp(data + 8, "AIFC", 4) != 0))
        return fail(error, "not an AIFF file");
    const bool aifc = std::memcmp(data + 8, "AIFC", 4) == 0;
    int channels = 0, bits = 0;
    std::uint32_t frames = 0;
    double rate = 0.0;
    bool littleEndian = false, isFloat = false, haveComm = false;
    const std::uint8_t* sound = nullptr;
    std::size_t soundBytes = 0;

    std::size_t at = 12;
    while (at + 8 <= size)
    {
        const std::uint32_t chunkSize = be32(data + at + 4);
        const std::uint8_t* body = data + at + 8;
        const std::size_t available = size - (at + 8);
        const std::size_t length = std::min<std::size_t>(chunkSize, available);
        if (std::memcmp(data + at, "COMM", 4) == 0)
        {
            if (length < 18)
                return fail(error, "AIFF: short COMM chunk");
            channels = be16(body);
            frames = be32(body + 2);
            bits = be16(body + 6);
            rate = readExtended(body + 8);
            if (aifc)
            {
                if (length < 22)
                    return fail(error, "AIFC: missing compression type");
                if (std::memcmp(body + 18, "NONE", 4) == 0 || std::memcmp(body + 18, "twos", 4) == 0)
                {
                }
                else if (std::memcmp(body + 18, "sowt", 4) == 0)
                    littleEndian = true;
                else if (std::memcmp(body + 18, "fl32", 4) == 0 || std::memcmp(body + 18, "FL32", 4) == 0 ||
                         std::memcmp(body + 18, "fl64", 4) == 0 || std::memcmp(body + 18, "FL64", 4) == 0)
                    isFloat = true;
                else
                    return fail(error, "AIFC: unsupported compression");
            }
            haveComm = true;
        }
        else if (std::memcmp(data + at, "SSND", 4) == 0)
        {
            if (length < 8)
                return fail(error, "AIFF: short SSND chunk");
            const std::uint32_t offset = be32(body);
            if (8 + static_cast<std::size_t>(offset) > length)
                return fail(error, "AIFF: bad SSND offset");
            sound = body + 8 + offset;
            soundBytes = length - 8 - offset;
        }
        at += 8 + static_cast<std::size_t>(chunkSize) + (chunkSize & 1u);
    }
    if (!haveComm || sound == nullptr)
        return fail(error, "AIFF: missing COMM or SSND chunk");
    if (channels < 1 || channels > 64 || rate <= 0.0 || !std::isfinite(rate))
        return fail(error, "AIFF: bad format");
    if (isFloat ? (bits != 32 && bits != 64) : (bits != 8 && bits != 16 && bits != 24 && bits != 32))
        return fail(error, "AIFF: unsupported sample size");
    const std::size_t bytesPerSample = static_cast<std::size_t>(bits) / 8;
    const std::size_t frameBytes = bytesPerSample * static_cast<std::size_t>(channels);
    const std::size_t available = soundBytes / frameBytes;
    const std::size_t count = std::min<std::size_t>(frames, available);

    out.sampleRate = rate;
    out.channels.assign(static_cast<std::size_t>(channels), std::vector<float>(count));
    for (std::size_t f = 0; f < count; ++f)
    {
        for (int c = 0; c < channels; ++c)
        {
            std::uint8_t b[8];
            const std::uint8_t* src = sound + f * frameBytes + static_cast<std::size_t>(c) * bytesPerSample;
            for (std::size_t i = 0; i < bytesPerSample; ++i)
                b[i] = littleEndian ? src[bytesPerSample - 1 - i] : src[i]; // b is big endian from here on
            float v;
            if (isFloat && bits == 32)
            {
                const std::uint32_t u = be32(b);
                std::memcpy(&v, &u, 4);
            }
            else if (isFloat)
            {
                std::uint64_t u = 0;
                for (int i = 0; i < 8; ++i)
                    u = (u << 8) | b[i];
                double d;
                std::memcpy(&d, &u, 8);
                v = static_cast<float>(d);
            }
            else if (bits == 8)
                v = static_cast<float>(static_cast<std::int8_t>(b[0])) / 128.0f;
            else if (bits == 16)
                v = static_cast<float>(static_cast<std::int16_t>(be16(b))) / 32768.0f;
            else if (bits == 24)
                v = static_cast<float>(static_cast<std::int32_t>((static_cast<std::uint32_t>(b[0]) << 24) |
                                                                 (static_cast<std::uint32_t>(b[1]) << 16) |
                                                                 (static_cast<std::uint32_t>(b[2]) << 8)) >>
                                       8) /
                    8388608.0f;
            else
                v = static_cast<float>(static_cast<double>(static_cast<std::int32_t>(be32(b))) / 2147483648.0);
            out.channels[static_cast<std::size_t>(c)][f] = v;
        }
    }
    return true;
}

std::vector<float> resampleKaiser(const std::vector<float>& in, double srcRate, double dstRate)
{
    if (in.empty() || srcRate <= 0.0 || dstRate <= 0.0 || srcRate == dstRate)
        return in;
    const double ratio = srcRate / dstRate; // source samples per output sample
    const std::size_t outCount = static_cast<std::size_t>(std::llround(static_cast<double>(in.size()) / ratio));
    std::vector<float> out(outCount, 0.0f);
    if (outCount == 0)
        return out;
    const Prototype proto = makePrototype(srcRate, dstRate);
    const int reach = static_cast<int>(proto.halfWidth); // taps on each side of the centre, source samples
    const long long n = static_cast<long long>(in.size());

    // Exact polyphase table when both rates are integers: the output instant m * q / p source samples repeats its
    // fractional part every p outputs.
    long long p = 0, q = 0;
    if (nearInteger(srcRate) && nearInteger(dstRate))
    {
        const long long a = std::llround(srcRate), b = std::llround(dstRate);
        const long long g = std::gcd(a, b);
        q = a / g;
        p = b / g;
    }
    if (p > 0 && p <= kMaxPhases)
    {
        const int width = 2 * reach + 1;
        std::vector<float> table(static_cast<std::size_t>(p) * static_cast<std::size_t>(width));
        for (long long ph = 0; ph < p; ++ph)
        {
            const double frac =
                static_cast<double>(ph) / static_cast<double>(p); // fractional part of the centre (row = (m * q) mod p)
            double sum = 0.0;
            std::vector<double> row(static_cast<std::size_t>(width));
            for (int t = 0; t < width; ++t)
            {
                row[static_cast<std::size_t>(t)] = proto.at(frac - static_cast<double>(t - reach));
                sum += row[static_cast<std::size_t>(t)];
            }
            for (int t = 0; t < width; ++t)
                table[static_cast<std::size_t>(ph) * static_cast<std::size_t>(width) + static_cast<std::size_t>(t)] =
                    static_cast<float>(row[static_cast<std::size_t>(t)] / sum);
        }
        for (std::size_t m = 0; m < outCount; ++m)
        {
            const long long num = static_cast<long long>(m) * q;
            const long long centre = num / p; // floor of the centre position
            const float* row = table.data() + static_cast<std::size_t>(num % p) * static_cast<std::size_t>(width);
            double acc = 0.0;
            for (int t = 0; t < width; ++t)
            {
                const long long idx = centre + t - reach;
                if (idx >= 0 && idx < n)
                    acc += static_cast<double>(row[t]) * static_cast<double>(in[static_cast<std::size_t>(idx)]);
            }
            out[m] = static_cast<float>(acc);
        }
        return out;
    }

    // General ratio: evaluate the window per tap.
    for (std::size_t m = 0; m < outCount; ++m)
    {
        const double pos = static_cast<double>(m) * ratio;
        const long long centre = static_cast<long long>(std::floor(pos));
        double acc = 0.0, sum = 0.0;
        for (int t = -reach; t <= reach + 1; ++t)
        {
            const long long idx = centre + t;
            const double w = proto.at(pos - static_cast<double>(idx));
            sum += w;
            if (idx >= 0 && idx < n)
                acc += w * static_cast<double>(in[static_cast<std::size_t>(idx)]);
        }
        out[m] = static_cast<float>(acc / sum);
    }
    return out;
}

bool importImpulseResponse(const std::uint8_t* data, std::size_t size, double targetRate, ImpulseResponse& out,
                           std::string& error)
{
    out = ImpulseResponse{};
    if (!(targetRate >= 1000.0 && targetRate <= 1.0e6))
        return fail(error, "bad target sample rate");
    AudioData audio;
    if (data != nullptr && size >= 12 && std::memcmp(data, "RIFF", 4) == 0)
    {
        if (!readWav(data, size, audio, error))
            return false;
    }
    else if (data != nullptr && size >= 12 && std::memcmp(data, "FORM", 4) == 0)
    {
        if (!readAiff(data, size, audio, error))
            return false;
    }
    else
        return fail(error, "unknown file type (WAV or AIFF expected)");

    const int channels = audio.numChannels();
    if (channels != 1 && channels != 2 && channels != 4)
        return fail(error, "impulse response must have 1, 2 or 4 channels");
    if (audio.numFrames() == 0 || !(audio.sampleRate >= 1000.0 && audio.sampleRate <= 1.0e6))
        return fail(error, "empty impulse response or bad sample rate");
    if (static_cast<double>(audio.numFrames()) > kMaxImpulseResponseSeconds * audio.sampleRate + 0.5)
        return fail(error, "impulse response longer than 30 seconds");

    out.sampleRate = targetRate;
    out.layout = channels == 1 ? IrLayout::Mono : channels == 2 ? IrLayout::Stereo : IrLayout::TrueStereo;
    out.channels.clear();
    for (const auto& ch : audio.channels)
        out.channels.push_back(resampleKaiser(ch, audio.sampleRate, targetRate));
    return true;
}

bool importImpulseResponseFile(const std::string& path, double targetRate, ImpulseResponse& out, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return fail(error, "cannot read file");
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return importImpulseResponse(bytes.data(), bytes.size(), targetRate, out, error);
}

} // namespace crucible
