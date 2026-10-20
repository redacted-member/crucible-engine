// SPDX-License-Identifier: MIT
#pragma once

// Import and export of arpeggiator pattern grids.
//
// JSON ("crucible-arp-pattern", version 1) keeps every field of every step, including probability groups.
// Standard MIDI files carry the playable part of a pattern as notes, so patterns can be edited in a DAW or taken from
// a MIDI clip: steps become notes on a 1/4-beat grid by default (stepsPerBeat), the pitch is relative to a base note
// (transpose and octave), velocity and gate come from the note, a ratcheted step is several short notes and a tie
// is a long note. Probability and groups have no MIDI representation (they are not exported and an import sets them
// to the defaults); gates above one step come back as tied steps.

#include <crucible/Arpeggiator.h>
#include <crucible/Json.h>
#include <crucible/MidiFile.h>

#include <cstdint>
#include <string>
#include <vector>

namespace crucible
{

json::Value arpPatternToJsonValue(const ArpPattern& pattern);
std::string arpPatternToJson(const ArpPattern& pattern);
bool arpPatternFromJson(const std::string& text, ArpPattern& out, std::string& error);
bool saveArpPatternFile(const std::string& path, const ArpPattern& pattern);
bool loadArpPatternFile(const std::string& path, ArpPattern& out, std::string& error);

// Format 0 file, 480 ticks per quarter, one tempo event.
std::vector<std::uint8_t> arpPatternToMidi(const ArpPattern& pattern, int baseNote = 60, double bpm = 120.0,
                                           int stepsPerBeat = 4);
// baseNote < 0: the lowest note of the clip is the reference (transpose 0). The pattern length is the clip length
// in steps (at most kArpMaxSteps); clips with chords keep the lowest note of each step.
bool arpPatternFromMidi(const std::vector<TimedMidi>& events, double bpm, int stepsPerBeat, int baseNote, ArpPattern& out,
                        std::string& error);
bool arpPatternFromMidiFile(const std::string& path, int stepsPerBeat, int baseNote, ArpPattern& out, std::string& error);

} // namespace crucible
