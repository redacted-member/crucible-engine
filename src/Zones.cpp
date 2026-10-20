// SPDX-License-Identifier: MIT
#include <crucible/Zones.h>

#include <crucible/Json.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

#include "Utf8Path.h"

namespace crucible
{
namespace
{
constexpr float kHalfPi = 1.57079633f;

bool rangesOverlap(const Zone& a, const Zone& b)
{
    return a.keyLo <= b.keyHi && b.keyLo <= a.keyHi && a.velLo <= b.velHi && b.velLo <= a.velHi;
}

std::uint32_t mix(std::uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

float finiteOr(float v, float fallback)
{
    return std::isfinite(v) ? v : fallback;
}

// "C3", "f#2", "Bb-1" (C3 = 60); -1 when the text is not a note name.
int noteFromName(const std::string& s)
{
    if (s.size() < 2)
        return -1;
    static const int semis[7] = {9, 11, 0, 2, 4, 5, 7}; // A B C D E F G
    const char letter = static_cast<char>(std::toupper(static_cast<unsigned char>(s[0])));
    if (letter < 'A' || letter > 'G')
        return -1;
    int semi = semis[letter - 'A'];
    size_t i = 1;
    if (s[i] == '#')
        ++semi, ++i;
    else if (s[i] == 'b' || s[i] == 'B')
        --semi, ++i;
    if (i >= s.size())
        return -1;
    size_t used = 0;
    int octave = 0;
    try
    {
        octave = std::stoi(s.substr(i), &used);
    }
    catch (...)
    {
        return -1;
    }
    if (i + used != s.size())
        return -1;
    const int note = 12 * (octave + 2) + semi;
    return note >= 0 && note <= 127 ? note : -1;
}

bool allDigits(const std::string& s)
{
    return !s.empty() &&
           std::all_of(s.begin(), s.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; });
}

std::string lower(std::string s)
{
    for (auto& c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> splitOn(const std::string& s, char sep)
{
    std::vector<std::string> out;
    std::string cur;
    for (const char c : s)
    {
        if (c == sep)
        {
            out.push_back(cur);
            cur.clear();
        }
        else
            cur.push_back(c);
    }
    out.push_back(cur);
    return out;
}

std::vector<float> interleaved(const SampleData& s)
{
    std::vector<float> out(static_cast<size_t>(s.frames) * static_cast<size_t>(s.channels));
    for (int c = 0; c < s.channels; ++c)
    {
        const auto& d = s.levels[0].data[static_cast<size_t>(c)];
        for (int i = 0; i < s.frames; ++i)
            out[static_cast<size_t>(i * s.channels + c)] = d[static_cast<size_t>(kSamplePad + i)];
    }
    return out;
}
} // namespace

bool Zone::operator==(const Zone& o) const
{
    return sampleId == o.sampleId && keyLo == o.keyLo && keyHi == o.keyHi && velLo == o.velLo && velHi == o.velHi &&
           root == o.root && gainDb == o.gainDb && tuneCents == o.tuneCents && loopStart == o.loopStart &&
           loopEnd == o.loopEnd && loopMode == o.loopMode && group == o.group;
}

const SampleData* ZoneSet::sampleFor(const Zone& z, const SampleData* slotSample) const
{
    if (z.sampleId == 0)
        return slotSample;
    const auto i = static_cast<size_t>(z.sampleId - 1);
    return i < samples.size() ? samples[i].get() : nullptr;
}

float zoneEdgeWeight(float x, int lo, int hi, float width)
{
    if (width <= 0.0f)
        return x >= static_cast<float>(lo) && x <= static_cast<float>(hi) ? 1.0f : 0.0f;
    float w = 1.0f;
    if (lo > 0)
    {
        const float t = std::min((x - (static_cast<float>(lo) - 0.5f)) / width + 0.5f, 1.0f);
        if (t <= 0.0f)
            return 0.0f;
        if (t < 1.0f)
            w *= std::sin(kHalfPi * t);
    }
    if (hi < 127)
    {
        const float t = std::max((x - (static_cast<float>(hi) + 0.5f)) / width + 0.5f, 0.0f);
        if (t >= 1.0f)
            return 0.0f;
        if (t > 0.0f)
            w *= std::cos(kHalfPi * t);
    }
    return w;
}

int selectZones(const ZoneSet& set, int key, int velocity, const ZoneSelect& options,
                std::array<ZoneLayer, kMaxZoneLayers>& out)
{
    struct Candidate
    {
        int zone;
        float weight;
    };
    std::array<Candidate, kMaxZones> cand;
    int n = 0;
    const float xk = std::clamp(options.xfadeKey, 0.0f, 12.0f);
    const float xv = std::clamp(options.xfadeVel, 0.0f, 0.5f) * 127.0f;
    const int total = std::min(set.count(), kMaxZones);
    for (int i = 0; i < total; ++i)
    {
        const Zone& z = set.zones[static_cast<size_t>(i)];
        const float w = zoneEdgeWeight(static_cast<float>(key), z.keyLo, z.keyHi, xk) *
                        zoneEdgeWeight(static_cast<float>(velocity), z.velLo, z.velHi, xv);
        if (w > 0.0f)
            cand[static_cast<size_t>(n++)] = {i, w};
    }
    int count = 0;
    if (options.roundRobin == ZoneRoundRobin::Off)
    {
        for (int i = 0; i < n && count < kMaxZoneLayers; ++i)
            out[static_cast<size_t>(count++)] = {cand[static_cast<size_t>(i)].zone, cand[static_cast<size_t>(i)].weight};
        return count;
    }
    // Alternatives: candidates of one group whose ranges overlap the first member's. One of them sounds.
    std::array<bool, kMaxZones> used{};
    std::array<int, kMaxZones> members;
    std::uint32_t base = 0;
    bool haveBase = false;
    std::array<Candidate, kMaxZoneLayers> picked;
    int nPicked = 0;
    for (int a = 0; a < n; ++a)
    {
        if (used[static_cast<size_t>(a)])
            continue;
        const Zone& za = set.zones[static_cast<size_t>(cand[static_cast<size_t>(a)].zone)];
        int m = 0;
        for (int b = a; b < n; ++b)
        {
            if (used[static_cast<size_t>(b)])
                continue;
            const Zone& zb = set.zones[static_cast<size_t>(cand[static_cast<size_t>(b)].zone)];
            if (b == a || (zb.group == za.group && rangesOverlap(za, zb)))
            {
                members[static_cast<size_t>(m++)] = b;
                used[static_cast<size_t>(b)] = true;
            }
        }
        int pick = 0;
        if (m > 1)
        {
            if (options.roundRobin == ZoneRoundRobin::Cycle)
            {
                if (!haveBase)
                {
                    base = set.cycle.fetch_add(1, std::memory_order_relaxed);
                    haveBase = true;
                }
                pick = static_cast<int>(base % static_cast<std::uint32_t>(m));
            }
            else
                pick = static_cast<int>(
                    mix(options.seed * 0x9e3779b1U + static_cast<std::uint32_t>(cand[static_cast<size_t>(a)].zone) + 1U) %
                    static_cast<std::uint32_t>(m));
        }
        if (nPicked < kMaxZoneLayers)
            picked[static_cast<size_t>(nPicked++)] = cand[static_cast<size_t>(members[static_cast<size_t>(pick)])];
    }
    std::sort(picked.begin(), picked.begin() + nPicked,
              [](const Candidate& x, const Candidate& y) { return x.zone < y.zone; });
    for (int i = 0; i < nPicked; ++i)
        out[static_cast<size_t>(i)] = {picked[static_cast<size_t>(i)].zone, picked[static_cast<size_t>(i)].weight};
    return nPicked;
}

ZoneFileName parseZoneFileName(const std::string& fileName)
{
    ZoneFileName out;
    std::string stem = fileName;
    const auto slash = stem.find_last_of("/\\");
    if (slash != std::string::npos)
        stem = stem.substr(slash + 1);
    const auto dot = stem.find_last_of('.');
    if (dot != std::string::npos)
        stem = stem.substr(0, dot);
    const auto parts = splitOn(stem, '_');
    for (size_t i = 1; i < parts.size(); ++i)
    {
        const std::string& p = parts[i];
        if (p.size() >= 2 && (p[0] == 'v' || p[0] == 'V') && allDigits(p.substr(1)) && p.size() <= 4)
        {
            out.velocity = std::clamp(std::stoi(p.substr(1)), 0, 127);
            continue;
        }
        const int note = noteFromName(p);
        if (note >= 0)
            out.root = static_cast<float>(note);
        else if (allDigits(p) && p.size() <= 3 && std::stoi(p) <= 127)
            out.root = static_cast<float>(std::stoi(p));
    }
    return out;
}

void assignAutomaticRanges(std::vector<Zone>& zones, const std::vector<int>& velocity)
{
    std::vector<int> layers;
    for (size_t i = 0; i < zones.size(); ++i)
        layers.push_back(i < velocity.size() ? velocity[i] : -1);
    std::vector<int> distinct;
    for (const int v : layers)
        if (v >= 0 && std::find(distinct.begin(), distinct.end(), v) == distinct.end())
            distinct.push_back(v);
    std::sort(distinct.begin(), distinct.end());
    for (size_t i = 0; i < zones.size(); ++i)
    {
        Zone& z = zones[i];
        z.velLo = 0;
        z.velHi = 127;
        if (layers[i] < 0)
            continue;
        const auto at = static_cast<size_t>(std::find(distinct.begin(), distinct.end(), layers[i]) - distinct.begin());
        if (at > 0)
            z.velLo = static_cast<std::uint8_t>((distinct[at - 1] + distinct[at]) / 2 + 1);
        if (at + 1 < distinct.size())
            z.velHi = static_cast<std::uint8_t>((distinct[at] + distinct[at + 1]) / 2);
    }
    // Key ranges per velocity layer (-1 = zones named without a velocity form one layer).
    std::vector<int> keys = layers;
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    for (const int layer : keys)
    {
        std::vector<int> roots;
        for (size_t i = 0; i < zones.size(); ++i)
            if (layers[i] == layer)
                roots.push_back(static_cast<int>(std::lround(zones[i].root)));
        std::sort(roots.begin(), roots.end());
        roots.erase(std::unique(roots.begin(), roots.end()), roots.end());
        for (size_t i = 0; i < zones.size(); ++i)
        {
            if (layers[i] != layer)
                continue;
            const auto at = static_cast<size_t>(
                std::find(roots.begin(), roots.end(), static_cast<int>(std::lround(zones[i].root))) - roots.begin());
            zones[i].keyLo = at == 0 ? 0 : static_cast<std::uint8_t>((roots[at - 1] + roots[at]) / 2 + 1);
            zones[i].keyHi = at + 1 == roots.size() ? 127 : static_cast<std::uint8_t>((roots[at] + roots[at + 1]) / 2);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
Zone ZoneBank::sanitiseZone(Zone z, int sampleCount)
{
    z.sampleId = std::clamp(z.sampleId, 0, std::max(sampleCount, 0));
    z.keyLo = std::min<std::uint8_t>(z.keyLo, 127);
    z.keyHi = std::min<std::uint8_t>(z.keyHi, 127);
    z.velLo = std::min<std::uint8_t>(z.velLo, 127);
    z.velHi = std::min<std::uint8_t>(z.velHi, 127);
    if (z.keyLo > z.keyHi)
        std::swap(z.keyLo, z.keyHi);
    if (z.velLo > z.velHi)
        std::swap(z.velLo, z.velHi);
    z.root = std::clamp(finiteOr(z.root, 60.0f), 0.0f, 127.0f);
    z.gainDb = std::clamp(finiteOr(z.gainDb, 0.0f), -60.0f, 24.0f);
    z.tuneCents = std::clamp(finiteOr(z.tuneCents, 0.0f), -1200.0f, 1200.0f);
    z.loopStart = std::clamp(finiteOr(z.loopStart, 0.0f), 0.0f, 1.0f);
    z.loopEnd = std::clamp(finiteOr(z.loopEnd, 1.0f), 0.0f, 1.0f);
    if (z.loopStart > z.loopEnd)
        std::swap(z.loopStart, z.loopEnd);
    z.loopMode = std::clamp(z.loopMode, -1, 2);
    return z;
}

void ZoneBank::publish(int slot)
{
    auto& stage = stages[static_cast<size_t>(slot)];
    if (stage.zones.empty())
    {
        slots[static_cast<size_t>(slot)].publish(nullptr);
        return;
    }
    auto set = std::make_unique<ZoneSet>();
    set->zones = stage.zones;
    set->samples = stage.samples;
    slots[static_cast<size_t>(slot)].publish(std::move(set));
}

int ZoneBank::addSample(int slot, const AudioData& audio, const std::string& name, std::string& error)
{
    if (slot < 0 || slot >= kMaxZoneSlots)
    {
        error = "no such sample slot";
        return 0;
    }
    auto& stage = stages[static_cast<size_t>(slot)];
    if (static_cast<int>(stage.samples.size()) + 1 >= kMaxSamplesPerSlot)
    {
        error = "too many samples in the slot";
        return 0;
    }
    auto built = SampleData::build(audio, name);
    if (!built)
    {
        error = "the audio is too short";
        return 0;
    }
    if (stage.frames + built->frames > kZoneMaxFrames)
    {
        error = "the slot's samples use too much memory";
        return 0;
    }
    SincKernel::instance();
    stage.frames += built->frames;
    stage.samples.push_back(std::shared_ptr<const SampleData>(std::move(built)));
    return static_cast<int>(stage.samples.size());
}

bool ZoneBank::setZones(int slot, std::vector<Zone> zones)
{
    if (slot < 0 || slot >= kMaxZoneSlots)
        return false;
    auto& stage = stages[static_cast<size_t>(slot)];
    if (static_cast<int>(zones.size()) > kMaxZones)
        zones.resize(kMaxZones);
    for (auto& z : zones)
        z = sanitiseZone(z, static_cast<int>(stage.samples.size()));
    stage.zones = std::move(zones);
    publish(slot);
    return true;
}

std::vector<Zone> ZoneBank::zones(int slot) const
{
    std::vector<Zone> out;
    if (slot >= 0 && slot < kMaxZoneSlots)
        slots[static_cast<size_t>(slot)].withLatest([&](const ZoneSet& set) { out = set.zones; });
    return out;
}

int ZoneBank::sampleCount(int slot) const
{
    return slot >= 0 && slot < kMaxZoneSlots ? static_cast<int>(stages[static_cast<size_t>(slot)].samples.size()) : 0;
}

const SampleData* ZoneBank::sample(int slot, int id) const
{
    if (slot < 0 || slot >= kMaxZoneSlots || id < 1)
        return nullptr;
    const auto& s = stages[static_cast<size_t>(slot)].samples;
    return static_cast<size_t>(id) <= s.size() ? s[static_cast<size_t>(id - 1)].get() : nullptr;
}

void ZoneBank::clear(int slot)
{
    if (slot < 0 || slot >= kMaxZoneSlots)
        return;
    stages[static_cast<size_t>(slot)] = Stage{};
    publish(slot);
}

void ZoneBank::clearAll()
{
    for (int s = 0; s < kMaxZoneSlots; ++s)
        clear(s);
}

bool ZoneBank::hasZones(int slot) const
{
    return slot >= 0 && slot < kMaxZoneSlots && !stages[static_cast<size_t>(slot)].zones.empty();
}

void ZoneBank::acquire(Pinned& out) noexcept
{
    for (size_t i = 0; i < slots.size(); ++i)
        out[i] = slots[i].acquire();
}

void ZoneBank::release() noexcept
{
    for (auto& s : slots)
        s.release();
}

void ZoneBank::collectGarbage()
{
    for (auto& s : slots)
        s.collectGarbage();
}

// ---- Persistence.
std::vector<ZoneBank::Record> ZoneBank::serialiseSamples() const
{
    std::vector<Record> out;
    for (int slot = 0; slot < kMaxZoneSlots; ++slot)
    {
        const auto& samples = stages[static_cast<size_t>(slot)].samples;
        for (size_t i = 0; i < samples.size(); ++i)
        {
            if (!samples[i])
                continue;
            Record r;
            r.slot = slot;
            r.index = static_cast<int>(i) + 1;
            r.name = samples[i]->name;
            r.rate = samples[i]->sampleRate;
            r.channels = samples[i]->channels;
            r.frames = interleaved(*samples[i]);
            out.push_back(std::move(r));
        }
    }
    return out;
}

std::string ZoneBank::zonesToText(const std::vector<Zone>& zones)
{
    json::Value arr = json::Value::array();
    for (const auto& z : zones)
    {
        json::Value o = json::Value::object();
        o.set("sample", z.sampleId);
        o.set("keyLo", static_cast<int>(z.keyLo));
        o.set("keyHi", static_cast<int>(z.keyHi));
        o.set("velLo", static_cast<int>(z.velLo));
        o.set("velHi", static_cast<int>(z.velHi));
        o.set("root", z.root);
        o.set("gainDb", z.gainDb);
        o.set("tuneCents", z.tuneCents);
        o.set("loopStart", z.loopStart);
        o.set("loopEnd", z.loopEnd);
        o.set("loopMode", z.loopMode);
        o.set("group", static_cast<int>(z.group));
        arr.push(o);
    }
    return arr.dump(-1);
}

bool ZoneBank::zonesFromText(const std::string& text, std::vector<Zone>& out)
{
    json::Value doc;
    std::string error;
    out.clear();
    if (!json::parse(text, doc, error) || !doc.isArray())
        return false;
    for (const auto& o : doc.elements())
    {
        if (out.size() >= static_cast<size_t>(kMaxZones))
            break;
        Zone z;
        const auto u8 = [&](const char* key, int fallback)
        { return static_cast<std::uint8_t>(std::clamp(static_cast<int>(o[key].asNumber(fallback)), 0, 255)); };
        z.sampleId = static_cast<int>(o["sample"].asNumber(0.0));
        z.keyLo = u8("keyLo", 0);
        z.keyHi = u8("keyHi", 127);
        z.velLo = u8("velLo", 0);
        z.velHi = u8("velHi", 127);
        z.root = static_cast<float>(o["root"].asNumber(60.0));
        z.gainDb = static_cast<float>(o["gainDb"].asNumber(0.0));
        z.tuneCents = static_cast<float>(o["tuneCents"].asNumber(0.0));
        z.loopStart = static_cast<float>(o["loopStart"].asNumber(0.0));
        z.loopEnd = static_cast<float>(o["loopEnd"].asNumber(1.0));
        z.loopMode = static_cast<int>(o["loopMode"].asNumber(-1.0));
        z.group = u8("group", 0);
        out.push_back(z);
    }
    return true;
}

std::string ZoneBank::serialiseZones(int slot) const
{
    if (slot < 0 || slot >= kMaxZoneSlots || stages[static_cast<size_t>(slot)].zones.empty())
        return {};
    return zonesToText(stages[static_cast<size_t>(slot)].zones);
}

void ZoneBank::deserialise(const std::vector<Record>& samples, const std::vector<std::pair<int, std::string>>& zoneTexts)
{
    clearAll();
    for (const auto& r : samples)
    {
        if (r.slot < 0 || r.slot >= kMaxZoneSlots || r.index < 1 || r.index >= kMaxSamplesPerSlot || r.channels < 1 ||
            r.frames.empty())
            continue;
        AudioData audio;
        audio.sampleRate = r.rate > 0.0 ? r.rate : 48000.0;
        const size_t frames = r.frames.size() / static_cast<size_t>(r.channels);
        audio.channels.assign(static_cast<size_t>(std::min(r.channels, kSampleMaxChannels)), std::vector<float>(frames));
        for (size_t c = 0; c < audio.channels.size(); ++c)
            for (size_t i = 0; i < frames; ++i)
                audio.channels[c][i] = r.frames[i * static_cast<size_t>(r.channels) + c];
        auto built = SampleData::build(audio, r.name);
        auto& stage = stages[static_cast<size_t>(r.slot)];
        if (!built || stage.frames + built->frames > kZoneMaxFrames)
            continue;
        SincKernel::instance();
        if (stage.samples.size() < static_cast<size_t>(r.index))
            stage.samples.resize(static_cast<size_t>(r.index));
        stage.frames += built->frames;
        stage.samples[static_cast<size_t>(r.index - 1)] = std::shared_ptr<const SampleData>(std::move(built));
    }
    for (const auto& [slot, text] : zoneTexts)
    {
        std::vector<Zone> zones;
        if (slot >= 0 && slot < kMaxZoneSlots && zonesFromText(text, zones))
            setZones(slot, std::move(zones));
    }
    for (int s = 0; s < kMaxZoneSlots; ++s)
        publish(s);
}

// ---- Import.
namespace
{
std::string readTextFile(const std::string& path)
{
    std::ifstream in(crucible::utf8Path(path), std::ios::binary);
    std::ostringstream os;
    os << in.rdbuf();
    return os.str();
}

double number(const std::map<std::string, std::string>& op, const char* key, double fallback)
{
    const auto it = op.find(key);
    if (it == op.end())
        return fallback;
    try
    {
        return std::stod(it->second);
    }
    catch (...)
    {
        return fallback;
    }
}

// A key opcode value: a number or a note name.
int keyValue(const std::map<std::string, std::string>& op, const char* key, int fallback)
{
    const auto it = op.find(key);
    if (it == op.end())
        return fallback;
    if (allDigits(it->second))
        return std::clamp(std::stoi(it->second), 0, 127);
    const int n = noteFromName(it->second);
    return n >= 0 ? n : fallback;
}
} // namespace

int ZoneBank::importFolder(int slot, const std::string& directory, std::string& error)
{
    namespace fs = std::filesystem;
    if (slot < 0 || slot >= kMaxZoneSlots)
    {
        error = "no such sample slot";
        return 0;
    }
    std::error_code ec;
    std::vector<fs::path> files;
    for (fs::directory_iterator it(crucible::utf8Path(directory), ec), end; !ec && it != end; it.increment(ec))
        if (it->is_regular_file(ec) && lower(it->path().extension().string()) == ".wav")
            files.push_back(it->path());
    if (files.empty())
    {
        error = "no WAV files in the folder";
        return 0;
    }
    std::sort(files.begin(), files.end());
    clear(slot);
    std::vector<Zone> zones;
    std::vector<int> velocity;
    for (const auto& f : files)
    {
        if (static_cast<int>(zones.size()) >= kMaxZones)
            break;
        AudioData audio;
        std::string err;
        if (!readWavFile(crucible::pathToUtf8(f), audio, err))
            continue;
        const int id = addSample(slot, audio, crucible::pathToUtf8(f.filename()), err);
        if (id == 0)
            continue;
        const ZoneFileName named = parseZoneFileName(crucible::pathToUtf8(f.filename()));
        Zone z;
        z.sampleId = id;
        const float detected = sample(slot, id)->detectedNote;
        z.root = named.root >= 0.0f ? named.root : (detected >= 0.0f ? std::round(detected) : 60.0f);
        zones.push_back(z);
        velocity.push_back(named.velocity);
    }
    if (zones.empty())
    {
        clear(slot);
        error = "none of the files could be read";
        return 0;
    }
    assignAutomaticRanges(zones, velocity);
    setZones(slot, std::move(zones));
    return static_cast<int>(stages[static_cast<size_t>(slot)].zones.size());
}

int ZoneBank::importSfz(int slot, const std::string& sfzPath, std::string& error)
{
    namespace fs = std::filesystem;
    if (slot < 0 || slot >= kMaxZoneSlots)
    {
        error = "no such sample slot";
        return 0;
    }
    const std::string text = readTextFile(sfzPath);
    if (text.empty())
    {
        error = "cannot read the SFZ file";
        return 0;
    }
    // Tokens: <header> or opcode=value; a token without '=' continues the previous value (file names with spaces).
    std::map<std::string, std::string> groupOps;
    std::vector<std::map<std::string, std::string>> regions;
    std::map<std::string, std::string>* current = nullptr;
    std::map<std::string, std::string> regionOps;
    bool inRegion = false;
    const auto closeRegion = [&]
    {
        if (inRegion)
            regions.push_back(regionOps);
        inRegion = false;
    };
    std::string lastKey;
    std::istringstream lines(text);
    std::string line;
    while (std::getline(lines, line))
    {
        const auto comment = line.find("//");
        if (comment != std::string::npos)
            line.erase(comment);
        std::istringstream tokens(line);
        std::string tok;
        while (tokens >> tok)
        {
            if (tok[0] == '<')
            {
                const std::string header = lower(tok);
                closeRegion();
                if (header == "<region>")
                {
                    inRegion = true;
                    regionOps = groupOps;
                    current = &regionOps;
                }
                else if (header == "<group>")
                {
                    groupOps.clear();
                    current = &groupOps;
                }
                else
                    current = nullptr;
                lastKey.clear();
                continue;
            }
            const auto eq = tok.find('=');
            if (eq != std::string::npos && current != nullptr)
            {
                lastKey = lower(tok.substr(0, eq));
                (*current)[lastKey] = tok.substr(eq + 1);
            }
            else if (!lastKey.empty() && current != nullptr)
                (*current)[lastKey] += " " + tok;
        }
    }
    closeRegion();
    const fs::path base = crucible::utf8Path(sfzPath).parent_path();
    clear(slot);
    std::map<std::string, int> ids;
    std::vector<Zone> zones;
    for (const auto& op : regions)
    {
        if (static_cast<int>(zones.size()) >= kMaxZones)
            break;
        const auto file = op.find("sample");
        if (file == op.end())
            continue;
        std::string rel = file->second;
        std::replace(rel.begin(), rel.end(), '\\', '/');
        const std::string full = crucible::pathToUtf8(base / crucible::utf8Path(rel));
        auto found = ids.find(full);
        if (found == ids.end())
        {
            AudioData audio;
            std::string err;
            int id = 0;
            if (readWavFile(full, audio, err))
                id = addSample(slot, audio, crucible::pathToUtf8(crucible::utf8Path(rel).filename()), err);
            found = ids.emplace(full, id).first;
        }
        if (found->second == 0)
            continue;
        const SampleData* data = sample(slot, found->second);
        Zone z;
        z.sampleId = found->second;
        const int single = keyValue(op, "key", -1);
        const int lo = keyValue(op, "lokey", single >= 0 ? single : 0);
        const int hi = keyValue(op, "hikey", single >= 0 ? single : 127);
        z.keyLo = static_cast<std::uint8_t>(lo);
        z.keyHi = static_cast<std::uint8_t>(hi);
        z.velLo = static_cast<std::uint8_t>(std::clamp(static_cast<int>(number(op, "lovel", 0.0)), 0, 127));
        z.velHi = static_cast<std::uint8_t>(std::clamp(static_cast<int>(number(op, "hivel", 127.0)), 0, 127));
        const int center = keyValue(op, "pitch_keycenter", single >= 0 ? single : -1);
        z.root =
            center >= 0 ? static_cast<float>(center) : (data->detectedNote >= 0.0f ? std::round(data->detectedNote) : 60.0f);
        z.gainDb = static_cast<float>(number(op, "volume", 0.0));
        z.tuneCents = static_cast<float>(number(op, "tune", 0.0) + 100.0 * number(op, "transpose", 0.0));
        const double frames = std::max(1, data->frames);
        if (op.count("loop_start") || op.count("loop_end"))
        {
            z.loopStart = static_cast<float>(number(op, "loop_start", 0.0) / frames);
            z.loopEnd = static_cast<float>(number(op, "loop_end", frames) / frames);
            z.loopMode = 1;
        }
        const auto mode = op.find("loop_mode");
        if (mode != op.end())
            z.loopMode = lower(mode->second) == "no_loop" || lower(mode->second) == "one_shot" ? 0 : 1;
        zones.push_back(z);
    }
    if (zones.empty())
    {
        clear(slot);
        error = "the SFZ file has no usable regions";
        return 0;
    }
    setZones(slot, std::move(zones));
    return static_cast<int>(stages[static_cast<size_t>(slot)].zones.size());
}

} // namespace crucible
