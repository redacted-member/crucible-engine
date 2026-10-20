// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace crucible
{

// Decoded audio: one float vector per channel, -1..1 for integer sources.
struct AudioData
{
    double sampleRate = 48000.0;
    std::vector<std::vector<float>> channels;
    // Samples per wavetable frame when the file says so (the 'clm ' chunk written by common wavetable editors,
    // "<!>2048 ..."); 0 when unknown.
    int cycleLength = 0;

    int numChannels() const { return static_cast<int>(channels.size()); }
    int numFrames() const { return channels.empty() ? 0 : static_cast<int>(channels[0].size()); }
    // Channel c, or the last channel when c is out of range (mono sources read the same data for left and right).
    const std::vector<float>& channel(int c) const;
    // Average of all channels.
    std::vector<float> mono() const;
};

// WAV (RIFF / RF64 not supported): PCM 8 / 16 / 24 / 32-bit integer, IEEE float 32 / 64, including
// WAVE_FORMAT_EXTENSIBLE. Control-thread helpers; they allocate. On failure they return false and set error.
bool readWav(const std::uint8_t* data, std::size_t size, AudioData& out, std::string& error);
bool readWavFile(const std::string& path, AudioData& out, std::string& error);
// Raw headerless float32 little-endian samples (for single-cycle waves): one channel.
bool readRawFloat32(const std::uint8_t* data, std::size_t size, double sampleRate, AudioData& out, std::string& error);

// Encodes 32-bit float WAV (bitsPerSample 32) or 16-bit PCM (bitsPerSample 16).
std::vector<std::uint8_t> writeWav(const AudioData& audio, int bitsPerSample = 32);
bool writeWavFile(const std::string& path, const AudioData& audio, int bitsPerSample = 32);

} // namespace crucible
