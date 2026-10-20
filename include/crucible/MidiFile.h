// SPDX-License-Identifier: MIT
#pragma once

// Standard MIDI File reader (format 0 and 1, tempo map, running status) for offline rendering.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace crucible
{

struct TimedMidi
{
    double seconds = 0.0;
    std::uint8_t status = 0;
    std::uint8_t data1 = 0;
    std::uint8_t data2 = 0;
};

struct MidiFileInfo
{
    int format = 0;
    int tracks = 0;
    int ticksPerQuarter = 480;
    double firstTempoBpm = 120.0;
    double lengthSeconds = 0.0;
};

// Channel-voice messages of all tracks merged in time order (ties keep file order: note-offs are not reordered).
bool readMidiFile(const std::uint8_t* data, std::size_t size, std::vector<TimedMidi>& events, MidiFileInfo& info,
                  std::string& error);
bool readMidiFile(const std::string& path, std::vector<TimedMidi>& events, MidiFileInfo& info, std::string& error);

} // namespace crucible
