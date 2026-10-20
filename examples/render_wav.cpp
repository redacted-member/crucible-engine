// SPDX-License-Identifier: MIT
// Offline render with CrucibleEngine and nothing else: plays a short phrase through the default patch with a little
// reverb and writes a 16-bit stereo WAV file.
//
// Usage: render_wav [output.wav]   (default: crucible_engine_demo.wav)

#include <crucible/Engine.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace
{
void put16(std::ofstream& f, std::uint16_t v)
{
    const char b[2] = {static_cast<char>(v & 0xff), static_cast<char>(v >> 8)};
    f.write(b, 2);
}

void put32(std::ofstream& f, std::uint32_t v)
{
    const char b[4] = {static_cast<char>(v & 0xff), static_cast<char>((v >> 8) & 0xff), static_cast<char>((v >> 16) & 0xff),
                       static_cast<char>(v >> 24)};
    f.write(b, 4);
}

bool writeWav(const std::string& path, const std::vector<float>& left, const std::vector<float>& right, int rate)
{
    std::ofstream f(path, std::ios::binary);
    if (!f)
        return false;
    const auto frames = static_cast<std::uint32_t>(left.size());
    const std::uint32_t dataBytes = frames * 2 * 2;
    f.write("RIFF", 4);
    put32(f, 36 + dataBytes);
    f.write("WAVEfmt ", 8);
    put32(f, 16);
    put16(f, 1); // PCM
    put16(f, 2);
    put32(f, static_cast<std::uint32_t>(rate));
    put32(f, static_cast<std::uint32_t>(rate) * 4);
    put16(f, 4);
    put16(f, 16);
    f.write("data", 4);
    put32(f, dataBytes);
    for (std::uint32_t i = 0; i < frames; ++i)
        for (const float s : {left[i], right[i]})
            put16(f,
                  static_cast<std::uint16_t>(static_cast<std::int16_t>(std::lround(std::clamp(s, -1.0f, 1.0f) * 32767.0f))));
    return static_cast<bool>(f);
}
} // namespace

int main(int argc, char** argv)
{
    const std::string path = argc > 1 ? argv[1] : "crucible_engine_demo.wav";
    constexpr int rate = 48000;
    constexpr int block = 256;

    crucible::Engine engine;
    engine.prepare(rate, block);
    engine.setParam("osc1_uni_voices", 4.0f);
    engine.setParam("osc1_uni_detune", 20.0f);
    engine.setParam("filter_cutoff", 2500.0f);
    engine.setParam("amp_release", 0.6f);
    engine.setParam("fx_ra_on", 1.0f);
    engine.setParam("fx_ra_mix", 0.25f);
    engine.setParam("master_gain", -8.0f);

    // (start time in seconds, note, velocity, length in seconds)
    struct Note
    {
        double at;
        int note;
        float velocity;
        double length;
    };
    const Note phrase[] = {
        {0.0, 48, 0.9f, 1.5}, {0.0, 55, 0.7f, 1.5}, {0.0, 60, 0.8f, 1.5}, {0.5, 64, 0.8f, 1.0}, {1.0, 67, 0.8f, 0.5}};
    const double seconds = 3.5;

    const int total = static_cast<int>(seconds * rate);
    std::vector<float> left(static_cast<size_t>(total)), right(static_cast<size_t>(total));
    std::vector<crucible::MidiInput> midi;
    for (int pos = 0; pos < total; pos += block)
    {
        const int n = std::min(block, total - pos);
        midi.clear();
        for (const auto& note : phrase)
        {
            const int on = static_cast<int>(note.at * rate);
            const int off = static_cast<int>((note.at + note.length) * rate);
            if (on >= pos && on < pos + n)
                midi.push_back({on - pos, 0x90, static_cast<std::uint8_t>(note.note),
                                static_cast<std::uint8_t>(note.velocity * 127.0f)});
            if (off >= pos && off < pos + n)
                midi.push_back({off - pos, 0x80, static_cast<std::uint8_t>(note.note), 0});
        }
        std::sort(midi.begin(), midi.end(), [](const auto& a, const auto& b) { return a.position < b.position; });
        engine.process(left.data() + pos, right.data() + pos, n, midi.data(), static_cast<int>(midi.size()));
    }

    float peak = 0.0f;
    for (int i = 0; i < total; ++i)
        peak = std::max({peak, std::abs(left[static_cast<size_t>(i)]), std::abs(right[static_cast<size_t>(i)])});
    if (!writeWav(path, left, right, rate))
    {
        std::fprintf(stderr, "could not write %s\n", path.c_str());
        return 1;
    }
    std::printf("wrote %s: %.2f s, peak %.3f (%.1f dBFS)\n", path.c_str(), seconds, peak,
                peak > 0.0f ? 20.0 * std::log10(peak) : -120.0);
    return peak > 0.001f ? 0 : 2;
}
