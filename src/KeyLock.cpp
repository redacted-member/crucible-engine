// SPDX-License-Identifier: MIT
#include <crucible/KeyLock.h>

#include <crucible/Tuning.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace crucible
{
namespace
{
constexpr std::uint16_t maskOf(std::initializer_list<int> intervals)
{
    std::uint16_t m = 0;
    for (const int i : intervals)
        m = static_cast<std::uint16_t>(m | (1U << i));
    return m;
}

struct ScaleDef
{
    const char* name;
    std::uint16_t mask;
};

constexpr std::array<ScaleDef, kKeyScaleCustom> kScales = {{
    {"Chromatic", 0x0FFF},
    {"Major (Ionian)", maskOf({0, 2, 4, 5, 7, 9, 11})},
    {"Natural Minor (Aeolian)", maskOf({0, 2, 3, 5, 7, 8, 10})},
    {"Harmonic Minor", maskOf({0, 2, 3, 5, 7, 8, 11})},
    {"Melodic Minor", maskOf({0, 2, 3, 5, 7, 9, 11})},
    {"Dorian", maskOf({0, 2, 3, 5, 7, 9, 10})},
    {"Phrygian", maskOf({0, 1, 3, 5, 7, 8, 10})},
    {"Lydian", maskOf({0, 2, 4, 6, 7, 9, 11})},
    {"Mixolydian", maskOf({0, 2, 4, 5, 7, 9, 10})},
    {"Locrian", maskOf({0, 1, 3, 5, 6, 8, 10})},
    {"Major Pentatonic", maskOf({0, 2, 4, 7, 9})},
    {"Minor Pentatonic", maskOf({0, 3, 5, 7, 10})},
    {"Blues", maskOf({0, 3, 5, 6, 7, 10})},
    {"Whole Tone", maskOf({0, 2, 4, 6, 8, 10})},
    {"Diminished (Whole-Half)", maskOf({0, 2, 3, 5, 6, 8, 9, 11})},
    {"Diminished (Half-Whole)", maskOf({0, 1, 3, 4, 6, 7, 9, 10})},
}};

int floorDiv(int a, int b)
{
    int q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0)))
        --q;
    return q;
}

std::uint16_t effectiveMask(const KeyLockSettings& s)
{
    const std::uint16_t m = s.mask & 0x0FFF;
    return m == 0 ? static_cast<std::uint16_t>(0x0FFF) : m;
}

bool pcInKey(const KeyLockSettings& s, int pc)
{
    return ((effectiveMask(s) >> ((pc - s.root + 120) % 12)) & 1U) != 0;
}

int whiteIndex(int pc, bool& black)
{
    static constexpr int idx[12] = {0, 0, 1, 1, 2, 3, 3, 4, 4, 5, 5, 6};
    black = pc == 1 || pc == 3 || pc == 6 || pc == 8 || pc == 10;
    return idx[pc];
}
} // namespace

const char* keyScaleName(int scale)
{
    if (scale >= 0 && scale < kKeyScaleCustom)
        return kScales[static_cast<size_t>(scale)].name;
    return scale == kKeyScaleCustom ? "Custom" : scale == kKeyScaleTuning ? "Tuning Scale" : "Unknown";
}

std::uint16_t keyScaleMask(int scale)
{
    return scale >= 0 && scale < kKeyScaleCustom ? kScales[static_cast<size_t>(scale)].mask : static_cast<std::uint16_t>(0);
}

std::uint16_t scaleMaskFromTuning(const Tuning& tuning)
{
    std::uint16_t m = 1; // the 1/1
    const auto& d = tuning.degreeCents();
    for (size_t i = 1; i < d.size(); ++i)
    {
        const int semitone = static_cast<int>(std::lround(d[i] / 100.0));
        m = static_cast<std::uint16_t>(m | (1U << (((semitone % 12) + 12) % 12)));
    }
    return m;
}

bool inKey(const KeyLockSettings& s, int note)
{
    return note >= 0 && note < 128 && pcInKey(s, note % 12);
}

int quantiseNote(const KeyLockSettings& s, int note)
{
    if (note < 0 || note > 127)
        return -1;
    switch (s.mode)
    {
    case KeyMode::Off:
        return note;
    case KeyMode::Block:
        return inKey(s, note) ? note : -1;
    case KeyMode::Snap:
    {
        if (inKey(s, note))
            return note;
        for (int d = 1; d < 12; ++d)
        {
            const int first = s.tieUp ? note + d : note - d;
            const int second = s.tieUp ? note - d : note + d;
            if (inKey(s, first))
                return first;
            if (inKey(s, second))
                return second;
        }
        return note;
    }
    case KeyMode::Remap:
    {
        std::array<int, 12> degrees{};
        int count = 0;
        for (int i = 0; i < 12; ++i)
            if (((effectiveMask(s) >> i) & 1U) != 0)
                degrees[static_cast<size_t>(count++)] = i;
        bool black = false;
        const int w = whiteIndex(note % 12, black) + 7 * (note / 12) - 7 * 5; // middle C (60) is key 0
        const int out = 60 + s.root + 12 * floorDiv(w, count) + degrees[static_cast<size_t>(w - count * floorDiv(w, count))];
        return out >= 0 && out <= 127 ? out : -1;
    }
    }
    return note;
}

KeyScaleView makeScaleView(const KeyLockSettings& s)
{
    KeyScaleView v;
    v.mode = s.mode;
    v.root = s.root;
    v.tieUp = s.tieUp;
    for (int pc = 0; pc < 12; ++pc)
        v.inKey[static_cast<size_t>(pc)] = pcInKey(s, pc);
    return v;
}

void NoteQuantiser::init() noexcept
{
    started.fill(-1);
    holders.fill(0);
    initialised = true;
}

void NoteQuantiser::reset() noexcept
{
    init();
}

NoteQuantiser::Start NoteQuantiser::noteOn(int note) noexcept
{
    if (!initialised)
        init();
    Start r;
    if (note < 0 || note > 127)
        return r;
    const int out = quantiseNote(settings, note);
    const int previous = started[static_cast<size_t>(note)];
    if (previous >= 0 && previous != out)
    {
        // The key was still down (a repeated note-on): release what it held before starting the new note.
        if (holders[static_cast<size_t>(previous)] > 0 && --holders[static_cast<size_t>(previous)] == 0)
            r.stop = previous;
    }
    else if (previous >= 0 && previous == out)
    {
        r.out = out; // retrigger of the same note
        return r;
    }
    started[static_cast<size_t>(note)] = static_cast<std::int8_t>(out);
    if (out >= 0)
        ++holders[static_cast<size_t>(out)];
    r.out = out;
    return r;
}

NoteQuantiser::Stop NoteQuantiser::noteOff(int note) noexcept
{
    if (!initialised)
        init();
    Stop r;
    if (note < 0 || note > 127)
        return r;
    const int out = started[static_cast<size_t>(note)];
    if (out < 0)
        return r; // muted or never started
    r.known = true;
    r.out = out;
    started[static_cast<size_t>(note)] = -1;
    if (holders[static_cast<size_t>(out)] > 0 && --holders[static_cast<size_t>(out)] == 0)
        r.release = true;
    return r;
}

} // namespace crucible
