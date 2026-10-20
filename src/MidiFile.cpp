// SPDX-License-Identifier: MIT
#include <crucible/MidiFile.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iterator>

#include "Utf8Path.h"

namespace crucible
{
namespace
{
struct Raw
{
    std::uint64_t tick = 0;
    int order = 0;
    bool tempo = false;
    std::uint32_t microsPerQuarter = 500000;
    std::uint8_t status = 0, d1 = 0, d2 = 0;
};

std::uint32_t be32(const std::uint8_t* p)
{
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | p[3];
}
} // namespace

bool readMidiFile(const std::uint8_t* data, std::size_t size, std::vector<TimedMidi>& events, MidiFileInfo& info,
                  std::string& error)
{
    events.clear();
    info = MidiFileInfo{};
    if (data == nullptr || size < 14 || std::memcmp(data, "MThd", 4) != 0 || be32(data + 4) < 6)
    {
        error = "not a standard MIDI file";
        return false;
    }
    info.format = (data[8] << 8) | data[9];
    info.tracks = (data[10] << 8) | data[11];
    const int division = (data[12] << 8) | data[13];
    if ((division & 0x8000) != 0)
    {
        error = "SMPTE time division is not supported";
        return false;
    }
    info.ticksPerQuarter = std::max(division, 1);
    std::vector<Raw> raw;
    std::size_t pos = 8 + be32(data + 4);
    int order = 0;
    for (int t = 0; t < info.tracks && pos + 8 <= size; ++t)
    {
        if (std::memcmp(data + pos, "MTrk", 4) != 0)
        {
            error = "missing track chunk";
            return false;
        }
        const std::size_t len = be32(data + pos + 4);
        std::size_t p = pos + 8;
        const std::size_t end = std::min(size, p + len);
        std::uint64_t tick = 0;
        std::uint8_t running = 0;
        const auto varlen = [&](std::uint32_t& v)
        {
            v = 0;
            for (int k = 0; k < 4 && p < end; ++k)
            {
                const std::uint8_t b = data[p++];
                v = (v << 7) | (b & 0x7fU);
                if ((b & 0x80U) == 0)
                    return true;
            }
            return false;
        };
        while (p < end)
        {
            std::uint32_t delta;
            if (!varlen(delta))
                break;
            tick += delta;
            if (p >= end)
                break;
            std::uint8_t status = data[p];
            if (status == 0xFF)
            {
                if (p + 2 > end)
                    break;
                const std::uint8_t type = data[p + 1];
                p += 2;
                std::uint32_t mlen;
                if (!varlen(mlen) || p + mlen > end)
                    break;
                if (type == 0x51 && mlen == 3)
                {
                    Raw r;
                    r.tick = tick;
                    r.order = order++;
                    r.tempo = true;
                    r.microsPerQuarter = (static_cast<std::uint32_t>(data[p]) << 16) | (data[p + 1] << 8) | data[p + 2];
                    raw.push_back(r);
                }
                p += mlen;
                continue;
            }
            if (status == 0xF0 || status == 0xF7)
            {
                ++p;
                std::uint32_t slen;
                if (!varlen(slen))
                    break;
                p += slen;
                continue;
            }
            if ((status & 0x80U) != 0)
            {
                running = status;
                ++p;
            }
            else
                status = running;
            if (status < 0x80)
                break;
            const std::uint8_t type = status & 0xF0U;
            const int needed = (type == 0xC0 || type == 0xD0) ? 1 : 2;
            if (p + static_cast<std::size_t>(needed) > end)
                break;
            Raw r;
            r.tick = tick;
            r.order = order++;
            r.status = status;
            r.d1 = data[p] & 0x7fU;
            r.d2 = needed == 2 ? (data[p + 1] & 0x7fU) : 0;
            p += static_cast<std::size_t>(needed);
            raw.push_back(r);
        }
        pos = pos + 8 + len;
    }
    std::stable_sort(raw.begin(), raw.end(), [](const Raw& a, const Raw& b) { return a.tick < b.tick; });
    // Tick to seconds through the tempo map.
    double seconds = 0.0;
    std::uint64_t lastTick = 0;
    std::uint32_t tempo = 500000;
    bool firstTempo = true;
    for (const auto& r : raw)
    {
        seconds += static_cast<double>(r.tick - lastTick) * tempo / 1.0e6 / info.ticksPerQuarter;
        lastTick = r.tick;
        if (r.tempo)
        {
            tempo = std::max<std::uint32_t>(r.microsPerQuarter, 1);
            if (firstTempo)
                info.firstTempoBpm = 60.0e6 / tempo;
            firstTempo = false;
            continue;
        }
        events.push_back({seconds, r.status, r.d1, r.d2});
    }
    info.lengthSeconds = seconds;
    return true;
}

bool readMidiFile(const std::string& path, std::vector<TimedMidi>& events, MidiFileInfo& info, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
    {
        error = "cannot read " + path;
        return false;
    }
    const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return readMidiFile(bytes.data(), bytes.size(), events, info, error);
}

} // namespace crucible
