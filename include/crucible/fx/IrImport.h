// SPDX-License-Identifier: MIT
#pragma once

#include <crucible/AudioFile.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace crucible
{

// Longest impulse response accepted by the importer, in seconds of the source file.
constexpr double kMaxImpulseResponseSeconds = 30.0;

// Channel layout of an imported impulse response.
//   Mono        1 channel: the same kernel for both sides.
//   Stereo      2 channels: left and right kernels.
//   TrueStereo  4 channels in the order LL, LR, RL, RR (input side first, output side second).
enum class IrLayout
{
    Mono,
    Stereo,
    TrueStereo
};

struct ImpulseResponse
{
    double sampleRate = 48000.0;
    IrLayout layout = IrLayout::Mono;
    std::vector<std::vector<float>> channels;

    int frames() const { return channels.empty() ? 0 : static_cast<int>(channels[0].size()); }
};

// AIFF / AIFC: PCM 8 / 16 / 24 / 32-bit big-endian ('NONE'), little-endian ('sowt'), IEEE float 32 / 64 ('fl32', 'FL32',
// 'fl64', 'FL64'). Control thread; allocates. On failure returns false and sets error.
bool readAiff(const std::uint8_t* data, std::size_t size, AudioData& out, std::string& error);

// Offline sample-rate conversion of one channel by a Kaiser-windowed sinc (stopband at least 130 dB, transition band
// from 0.95 to 1.05 of the lower Nyquist frequency; images and aliases stay below -120 dB). The output has
// round(n * dstRate / srcRate) samples; the signal outside the input is treated as silence. Control thread; allocates.
// Returns the input unchanged when the rates are equal.
std::vector<float> resampleKaiser(const std::vector<float>& in, double srcRate, double dstRate);

// Decodes a WAV or AIFF file image (detected from its header), checks the channel count (1, 2 or 4) and the length
// (at most kMaxImpulseResponseSeconds), and resamples to `targetRate`. Control thread; allocates.
bool importImpulseResponse(const std::uint8_t* data, std::size_t size, double targetRate, ImpulseResponse& out,
                           std::string& error);
bool importImpulseResponseFile(const std::string& path, double targetRate, ImpulseResponse& out, std::string& error);

} // namespace crucible
