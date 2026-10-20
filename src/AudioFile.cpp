// SPDX-License-Identifier: MIT
#include <crucible/AudioFile.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>

#include "Utf8Path.h"

namespace crucible
{
namespace
{
std::uint32_t u32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::uint16_t u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

void put32(std::vector<std::uint8_t>& out, std::uint32_t v)
{
    for (int i = 0; i < 4; ++i)
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffU));
}

void put16(std::vector<std::uint8_t>& out, std::uint16_t v)
{
    out.push_back(static_cast<std::uint8_t>(v & 0xffU));
    out.push_back(static_cast<std::uint8_t>(v >> 8));
}

bool readFile(const std::string& path, std::vector<std::uint8_t>& bytes)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}
} // namespace

const std::vector<float>& AudioData::channel(int c) const
{
    static const std::vector<float> empty;
    if (channels.empty())
        return empty;
    return channels[static_cast<size_t>(std::clamp(c, 0, numChannels() - 1))];
}

std::vector<float> AudioData::mono() const
{
    std::vector<float> m(static_cast<size_t>(numFrames()), 0.0f);
    if (channels.empty())
        return m;
    for (const auto& ch : channels)
        for (size_t i = 0; i < m.size(); ++i)
            m[i] += ch[i];
    const float scale = 1.0f / static_cast<float>(channels.size());
    for (auto& x : m)
        x *= scale;
    return m;
}

bool readWav(const std::uint8_t* data, std::size_t size, AudioData& out, std::string& error)
{
    out = AudioData{};
    if (data == nullptr || size < 12 || std::memcmp(data, "RIFF", 4) != 0 || std::memcmp(data + 8, "WAVE", 4) != 0)
    {
        error = "not a RIFF/WAVE file";
        return false;
    }
    int format = 0, channels = 0, bits = 0, cycle = 0;
    double rate = 0.0;
    const std::uint8_t* pcm = nullptr;
    std::size_t pcmSize = 0;
    std::size_t pos = 12;
    while (pos + 8 <= size)
    {
        const std::uint8_t* chunk = data + pos;
        const std::size_t len = u32(chunk + 4);
        const std::size_t body = pos + 8;
        const std::size_t available = std::min(len, size - body);
        if (std::memcmp(chunk, "fmt ", 4) == 0 && available >= 16)
        {
            format = u16(data + body);
            channels = u16(data + body + 2);
            rate = static_cast<double>(u32(data + body + 4));
            bits = u16(data + body + 14);
            if (format == 0xFFFE && available >= 26)
                format = u16(data + body + 24); // sub-format GUID starts with the format tag
        }
        else if (std::memcmp(chunk, "clm ", 4) == 0 && available >= 4)
        {
            const std::string text(reinterpret_cast<const char*>(data + body), available);
            const auto at = text.find("<!>");
            if (at != std::string::npos)
                cycle = std::atoi(text.c_str() + at + 3);
        }
        else if (std::memcmp(chunk, "data", 4) == 0)
        {
            pcm = data + body;
            pcmSize = available;
        }
        pos = body + len + (len & 1U);
    }
    if (pcm == nullptr || channels <= 0 || rate <= 0.0)
    {
        error = "missing fmt or data chunk";
        return false;
    }
    const bool isFloat = format == 3;
    if (!(format == 1 || isFloat) || !(bits == 8 || bits == 16 || bits == 24 || bits == 32 || bits == 64) ||
        (isFloat && bits != 32 && bits != 64) || (!isFloat && bits == 64))
    {
        error = "unsupported WAV encoding (format " + std::to_string(format) + ", " + std::to_string(bits) + " bit)";
        return false;
    }
    const std::size_t bytesPerSample = static_cast<std::size_t>(bits / 8);
    const std::size_t frames = pcmSize / (bytesPerSample * static_cast<std::size_t>(channels));
    out.sampleRate = rate;
    out.cycleLength = cycle > 0 && cycle <= 65536 ? cycle : 0;
    out.channels.assign(static_cast<size_t>(channels), std::vector<float>(frames));
    for (std::size_t f = 0; f < frames; ++f)
        for (int c = 0; c < channels; ++c)
        {
            const std::uint8_t* s =
                pcm + (f * static_cast<std::size_t>(channels) + static_cast<std::size_t>(c)) * bytesPerSample;
            float v = 0.0f;
            if (isFloat && bits == 32)
            {
                const std::uint32_t u = u32(s);
                std::memcpy(&v, &u, 4);
            }
            else if (isFloat)
            {
                const std::uint64_t u = static_cast<std::uint64_t>(u32(s)) | (static_cast<std::uint64_t>(u32(s + 4)) << 32);
                double d;
                std::memcpy(&d, &u, 8);
                v = static_cast<float>(d);
            }
            else if (bits == 8)
                v = (static_cast<float>(s[0]) - 128.0f) / 128.0f;
            else if (bits == 16)
                v = static_cast<float>(static_cast<std::int16_t>(u16(s))) / 32768.0f;
            else if (bits == 24)
            {
                std::int32_t x = static_cast<std::int32_t>(s[0] | (s[1] << 8) | (s[2] << 16));
                if ((x & 0x800000) != 0)
                    x -= 0x1000000;
                v = static_cast<float>(x) / 8388608.0f;
            }
            else
                v = static_cast<float>(static_cast<double>(static_cast<std::int32_t>(u32(s))) / 2147483648.0);
            out.channels[static_cast<size_t>(c)][f] = std::isfinite(v) ? v : 0.0f;
        }
    return true;
}

bool readWavFile(const std::string& path, AudioData& out, std::string& error)
{
    std::vector<std::uint8_t> bytes;
    if (!readFile(path, bytes))
    {
        error = "cannot read " + path;
        return false;
    }
    return readWav(bytes.data(), bytes.size(), out, error);
}

bool readRawFloat32(const std::uint8_t* data, std::size_t size, double sampleRate, AudioData& out, std::string& error)
{
    out = AudioData{};
    if (data == nullptr || size < 4 || size % 4 != 0)
    {
        error = "raw float32 data must be a non-empty multiple of 4 bytes";
        return false;
    }
    out.sampleRate = sampleRate;
    out.channels.assign(1, std::vector<float>(size / 4));
    for (std::size_t i = 0; i < size / 4; ++i)
    {
        const std::uint32_t u = u32(data + 4 * i);
        float v;
        std::memcpy(&v, &u, 4);
        out.channels[0][i] = std::isfinite(v) ? v : 0.0f;
    }
    return true;
}

std::vector<std::uint8_t> writeWav(const AudioData& audio, int bitsPerSample)
{
    const int bits = bitsPerSample == 16 ? 16 : 32;
    const auto channels = static_cast<std::uint16_t>(std::max(audio.numChannels(), 1));
    const auto frames = static_cast<std::uint32_t>(audio.numFrames());
    const std::uint32_t dataBytes = frames * channels * static_cast<std::uint32_t>(bits / 8);
    std::vector<std::uint8_t> out;
    out.reserve(44 + dataBytes);
    out.insert(out.end(), {'R', 'I', 'F', 'F'});
    put32(out, 36 + dataBytes);
    out.insert(out.end(), {'W', 'A', 'V', 'E', 'f', 'm', 't', ' '});
    put32(out, 16);
    put16(out, bits == 32 ? 3 : 1);
    put16(out, channels);
    const auto rate = static_cast<std::uint32_t>(std::lround(audio.sampleRate));
    put32(out, rate);
    put32(out, rate * channels * static_cast<std::uint32_t>(bits / 8));
    put16(out, static_cast<std::uint16_t>(channels * bits / 8));
    put16(out, static_cast<std::uint16_t>(bits));
    out.insert(out.end(), {'d', 'a', 't', 'a'});
    put32(out, dataBytes);
    for (std::uint32_t f = 0; f < frames; ++f)
        for (int c = 0; c < channels; ++c)
        {
            const float v = audio.channels.empty() ? 0.0f : audio.channel(c)[f];
            if (bits == 32)
            {
                std::uint32_t u;
                std::memcpy(&u, &v, 4);
                put32(out, u);
            }
            else
            {
                const auto s = static_cast<std::int16_t>(std::lround(std::clamp(v, -1.0f, 1.0f) * 32767.0f));
                put16(out, static_cast<std::uint16_t>(s));
            }
        }
    return out;
}

bool writeWavFile(const std::string& path, const AudioData& audio, int bitsPerSample)
{
    const auto bytes = writeWav(audio, bitsPerSample);
    std::ofstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return static_cast<bool>(f);
}

} // namespace crucible
