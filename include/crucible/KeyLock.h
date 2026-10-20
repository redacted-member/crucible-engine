// SPDX-License-Identifier: MIT
#pragma once

// Key lock: a scale quantiser in the note path so a synth cannot be played out of key.
//
// Placement. Every note that reaches the voices passes through a NoteQuantiser first: keyboard and MIDI notes,
// sequencer notes and arpeggiator output. (Arpeggiator input is quantised too, so chords are built from in-key
// notes.) Pitch bend, MPE expression, controllers and the tuning are not touched: only the note number changes.
//
// Modes
//   Off    notes pass unchanged (bit-identical to an engine without key lock);
//   Snap   an out-of-key note plays the nearest in-key note (ties go down or up, `tieUp`);
//   Block  out-of-key notes are muted;
//   Remap  the white keys play the scale degrees, so every key is in key: white key w (counted from middle C)
//          plays degree w of the scale, wrapping by the scale size per octave (a pentatonic scale spans an octave
//          in five white keys plus the next octave's first two); a black key plays the degree of the white key
//          below it. Degree 0 sits on note 60 + root.
//
// Note-offs follow the note that was actually started for that key (also when the scale or mode changed in
// between), and two keys that snap onto the same note release it when the last of them is released.
//
// The scale is a 12-bit mask of intervals from the root (bit 0 = root). Built-in scales: keyScaleMask(); custom
// masks and the mask derived from a loaded Scala tuning (scaleMaskFromTuning) use the same form.

#include <array>
#include <cstdint>
#include <string>

namespace crucible
{

class Tuning;

enum class KeyMode : int
{
    Off,
    Snap,
    Block,
    Remap
};

constexpr int kKeyScaleCustom = 16;
constexpr int kKeyScaleTuning = 17;
constexpr int kNumKeyScales = 18;

// Name and interval mask of a built-in scale (index < kKeyScaleCustom); custom / tuning entries have mask 0 here.
const char* keyScaleName(int scale);
std::uint16_t keyScaleMask(int scale);
// Mask of the semitone positions nearest to the degrees of a Scala scale (relative to its 1/1).
std::uint16_t scaleMaskFromTuning(const Tuning& tuning);

struct KeyLockSettings
{
    KeyMode mode = KeyMode::Off;
    int root = 0;                // pitch class 0..11
    std::uint16_t mask = 0x0AB5; // intervals from the root; major
    bool tieUp = false;
    bool operator==(const KeyLockSettings&) const = default;
};

// What a UI needs to highlight in-key notes: absolute pitch classes (0 = C) plus the root. Read-only.
struct KeyScaleView
{
    KeyMode mode = KeyMode::Off;
    int root = 0;
    std::array<bool, 12> inKey{};
    bool tieUp = false;
    bool following = false; // the scale comes from the key bus
    std::uint32_t busGeneration = 0;
};

bool inKey(const KeyLockSettings& s, int note);
// The note a key plays under the settings, or -1 when it is muted.
int quantiseNote(const KeyLockSettings& s, int note);
KeyScaleView makeScaleView(const KeyLockSettings& s);

class NoteQuantiser
{
public:
    void configure(const KeyLockSettings& s) { settings = s; }
    const KeyLockSettings& config() const { return settings; }

    struct Start
    {
        int out = -1;  // note to play, -1 when muted
        int stop = -1; // a note this key was still holding that has to be released first
    };
    struct Stop
    {
        int out = -1;
        bool release = false; // true when the note should now be released
        bool known = false;
    };
    Start noteOn(int note) noexcept;
    Stop noteOff(int note) noexcept;
    void reset() noexcept;

private:
    KeyLockSettings settings;
    std::array<std::int8_t, 128> started{};  // per key: note that was started, -1 none
    std::array<std::uint8_t, 128> holders{}; // per output note: keys holding it
    bool initialised = false;
    void init() noexcept;
};

} // namespace crucible
