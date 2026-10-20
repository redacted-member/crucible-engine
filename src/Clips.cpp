// SPDX-License-Identifier: MIT
#include <crucible/Clips.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
// Deterministic 0..1 value per (seed, cycle, index, salt); used for humanise and randomise.
float hash01(std::uint32_t seed, std::uint32_t cycle, int index, std::uint32_t salt)
{
    std::uint32_t h =
        seed * 0x9E3779B1U ^ cycle * 0x85EBCA77U ^ (static_cast<std::uint32_t>(index) + 1U) * 0xC2B2AE3DU ^ salt;
    h ^= h >> 16;
    h *= 0x7feb352dU;
    h ^= h >> 15;
    h *= 0x846ca68bU;
    h ^= h >> 16;
    return static_cast<float>(h >> 8) * (1.0f / 16777216.0f);
}

// Humanise: the segment starts up to half a segment late and catches up by its end.
float humaniseFraction(const ClipLane& lane, std::uint32_t cycle, int index, float f)
{
    if (lane.humanise <= 0.0f)
        return f;
    const float d = hash01(lane.seed, cycle, index, 0x51ED270BU) * lane.humanise * 0.5f;
    return f <= d ? 0.0f : (f - d) / (1.0f - d);
}

float randomised(const ClipLane& lane, std::uint32_t cycle, int index, float y)
{
    if (lane.randomise <= 0.0f)
        return y;
    return std::clamp(y + lane.randomise * (2.0f * hash01(lane.seed, cycle, index, 0xA3C59AC3U) - 1.0f), 0.0f, 1.0f);
}

const char* modeName(ClipPlayMode m)
{
    switch (m)
    {
    case ClipPlayMode::OneShot:
        return "oneshot";
    case ClipPlayMode::Loop:
        return "loop";
    case ClipPlayMode::PingPong:
        return "pingpong";
    }
    return "loop";
}

const char* triggerName(ClipTrigger t)
{
    switch (t)
    {
    case ClipTrigger::Voice:
        return "voice";
    case ClipTrigger::GlobalRetrigger:
        return "global";
    case ClipTrigger::GlobalFree:
        return "free";
    case ClipTrigger::Transport:
        return "transport";
    }
    return "voice";
}

const char* shapeName(SegmentShape s)
{
    switch (s)
    {
    case SegmentShape::Curve:
        return "curve";
    case SegmentShape::Hold:
        return "hold";
    case SegmentShape::SCurve:
        return "scurve";
    case SegmentShape::Stairs:
        return "stairs";
    }
    return "curve";
}

template <typename E, size_t N>
E parseName(const std::string& s, const std::array<E, N>& values, const char* (*name)(E), E fallback)
{
    for (const E v : values)
        if (s == name(v))
            return v;
    return fallback;
}
} // namespace

int Clip::lengthSteps() const
{
    return grid > 0.0 ? static_cast<int>(std::lround(length / grid)) : 0;
}

void Clip::setLengthSteps(int steps)
{
    if (grid <= 0.0 || steps <= 0)
        return;
    const bool loopAtEnd = std::abs(loopEnd - length) < 1.0e-9;
    length = steps * grid;
    if (loopAtEnd || loopEnd > length)
        loopEnd = length;
}

double Clip::snap(double x) const
{
    return grid > 0.0 ? std::round(x / grid) * grid : x;
}

void Clip::sanitise()
{
    if (!std::isfinite(length))
        length = 4.0;
    length = std::clamp(length, 1.0e-3, 4096.0);
    grid = std::isfinite(grid) ? std::clamp(grid, 0.0, length) : 0.0;
    rate = std::isfinite(rate) ? std::clamp(rate, 1.0 / 64.0, 64.0) : 1.0;
    loopStart = std::isfinite(loopStart) ? std::clamp(loopStart, 0.0, length) : 0.0;
    loopEnd = std::isfinite(loopEnd) ? std::clamp(loopEnd, 0.0, length) : length;
    if (loopEnd - loopStart < 1.0e-6)
    {
        loopStart = 0.0;
        loopEnd = length;
    }
    if (static_cast<int>(mode) > 2)
        mode = ClipPlayMode::Loop;
    if (static_cast<int>(trigger) > 3)
        trigger = ClipTrigger::Voice;
    if (lanes.size() > static_cast<size_t>(kMaxClipLanes))
        lanes.resize(static_cast<size_t>(kMaxClipLanes));
    for (auto& l : lanes)
    {
        if (static_cast<int>(l.kind) > 1)
            l.kind = ClipLaneKind::Curve;
        std::stable_sort(l.points.begin(), l.points.end(), [](const ClipPoint& a, const ClipPoint& b) { return a.x < b.x; });
        if (l.points.size() > static_cast<size_t>(kMaxClipPoints))
            l.points.resize(static_cast<size_t>(kMaxClipPoints));
        for (auto& p : l.points)
        {
            p.x = std::isfinite(p.x) ? std::clamp(p.x, 0.0, length) : 0.0;
            p.y = std::isfinite(p.y) ? std::clamp(p.y, 0.0f, 1.0f) : 0.0f;
            p.curve = std::isfinite(p.curve) ? std::clamp(p.curve, -1.0f, 1.0f) : 0.0f;
            if (static_cast<int>(p.shape) > 3)
                p.shape = SegmentShape::Curve;
        }
        if (l.steps.size() > static_cast<size_t>(kMaxClipSteps))
            l.steps.resize(static_cast<size_t>(kMaxClipSteps));
        for (auto& s : l.steps)
            s = std::isfinite(s) ? std::clamp(s, 0.0f, 1.0f) : 0.0f;
        const auto unit = [](float v) { return std::isfinite(v) ? std::clamp(v, 0.0f, 1.0f) : 0.0f; };
        l.stepGlide = unit(l.stepGlide);
        l.humanise = unit(l.humanise);
        l.randomise = unit(l.randomise);
        l.smoothingMs = std::isfinite(l.smoothingMs) ? std::clamp(l.smoothingMs, 0.0f, 5000.0f) : 0.0f;
    }
}

Clip makeMseg(const std::vector<ClipPoint>& points, double lengthBeats, bool sync, double loopStart, double loopEnd)
{
    Clip c;
    c.name = "MSEG";
    c.sync = sync;
    c.length = lengthBeats;
    c.trigger = ClipTrigger::Voice;
    if (loopStart >= 0.0 && loopEnd > loopStart)
    {
        c.mode = ClipPlayMode::Loop;
        c.sustainLoop = true;
        c.loopStart = loopStart;
        c.loopEnd = loopEnd;
    }
    else
    {
        c.mode = ClipPlayMode::OneShot;
        c.loopStart = 0.0;
        c.loopEnd = lengthBeats;
    }
    ClipLane lane;
    lane.name = "MSEG";
    lane.points = points;
    c.lanes.push_back(lane);
    c.sanitise();
    return c;
}

namespace
{
double loopedPosition(const Clip& c, double elapsed, std::uint32_t& cycle)
{
    cycle = 0;
    const double len = c.length;
    if (c.mode == ClipPlayMode::OneShot)
        return std::clamp(elapsed, 0.0, len);
    const double ls = c.loopStart;
    const double le = c.loopEnd;
    const double span = le - ls;
    if (elapsed < le || span <= 0.0)
        return std::max(elapsed, 0.0);
    const double t = elapsed - ls;
    const double k = std::floor(t / span);
    double r = t - k * span;
    if (r < 0.0)
        r = 0.0;
    if (r >= span)
        r = 0.0;
    cycle = static_cast<std::uint32_t>(std::min(k, 4.0e9));
    if (c.mode == ClipPlayMode::PingPong && (static_cast<std::uint64_t>(k) & 1U) != 0)
        return le - r;
    return ls + r;
}
} // namespace

double clipPosition(const Clip& c, double elapsed, double releasedAt, std::uint32_t& cycle)
{
    if (c.sustainLoop && c.mode != ClipPlayMode::OneShot && releasedAt >= 0.0 && elapsed >= releasedAt)
    {
        const double atRelease = loopedPosition(c, releasedAt, cycle);
        return std::min(atRelease + (elapsed - releasedAt), c.length);
    }
    return loopedPosition(c, elapsed, cycle);
}

float segmentShape(SegmentShape shape, float curve, float f)
{
    f = std::clamp(f, 0.0f, 1.0f);
    switch (shape)
    {
    case SegmentShape::Curve:
        return curve == 0.0f ? f : std::pow(f, std::exp2(curve * 3.0f));
    case SegmentShape::Hold:
        return 0.0f;
    case SegmentShape::SCurve:
    {
        const float k = std::exp2(1.0f + curve * 2.0f);
        return f < 0.5f ? 0.5f * std::pow(2.0f * f, k) : 1.0f - 0.5f * std::pow(2.0f - 2.0f * f, k);
    }
    case SegmentShape::Stairs:
    {
        const float n = 2.0f + std::round((curve + 1.0f) * 7.0f);
        return std::min(std::floor(f * n) / (n - 1.0f), 1.0f);
    }
    }
    return f;
}

float evaluateLane(const Clip& c, const ClipLane& lane, double position, std::uint32_t cycle)
{
    if (lane.kind == ClipLaneKind::Steps)
    {
        const int n = static_cast<int>(lane.steps.size());
        if (n == 0)
            return 0.0f;
        const double stepLen = c.length / n;
        const double q = position / stepLen;
        const int i = std::clamp(static_cast<int>(std::floor(q)), 0, n - 1);
        float f = std::clamp(static_cast<float>(q - i), 0.0f, 1.0f);
        const auto value = [&](int k) { return randomised(lane, cycle, k, lane.steps[static_cast<size_t>(k)]); };
        if (lane.humanise > 0.0f && i > 0)
        {
            const float d = hash01(lane.seed, cycle, i, 0x51ED270BU) * lane.humanise * 0.5f;
            if (f < d)
                return value(i - 1);
            f = (f - d) / (1.0f - d);
        }
        float v = value(i);
        if (lane.stepGlide > 0.0f && f > 1.0f - lane.stepGlide)
        {
            const int next = i + 1 < n ? i + 1 : (c.mode == ClipPlayMode::OneShot ? i : 0);
            const float t = (f - (1.0f - lane.stepGlide)) / lane.stepGlide;
            v += (value(next) - v) * t;
        }
        return v;
    }

    const auto& pts = lane.points;
    const int n = static_cast<int>(pts.size());
    if (n == 0)
        return 0.0f;
    if (position <= pts[0].x)
        return randomised(lane, cycle, 0, pts[0].y);
    if (position >= pts[static_cast<size_t>(n - 1)].x)
        return randomised(lane, cycle, n - 1, pts[static_cast<size_t>(n - 1)].y);
    // Largest i with x_i <= position.
    int lo = 0, hi = n - 1;
    while (hi - lo > 1)
    {
        const int mid = (lo + hi) / 2;
        if (pts[static_cast<size_t>(mid)].x <= position)
            lo = mid;
        else
            hi = mid;
    }
    const auto& a = pts[static_cast<size_t>(lo)];
    const auto& b = pts[static_cast<size_t>(lo + 1)];
    const double dx = b.x - a.x;
    float f = dx > 0.0 ? static_cast<float>((position - a.x) / dx) : 1.0f;
    f = humaniseFraction(lane, cycle, lo, f);
    const float ya = randomised(lane, cycle, lo, a.y);
    const float yb = randomised(lane, cycle, lo + 1, b.y);
    return ya + (yb - ya) * segmentShape(a.shape, a.curve, f);
}

// ---- ClipPlayer.

void ClipPlayer::prepare(double rate, int voiceCount)
{
    sampleRate = rate;
    voices = std::max(voiceCount, 0);
    voiceHeads.assign(static_cast<size_t>(voices) * kMaxClips, ClipPlayhead{});
    voiceSmooth.assign(static_cast<size_t>(voices) * laneSlots, -1.0f);
    voiceAge.assign(static_cast<size_t>(voices), 0);
    publishedVoice = std::make_unique<std::atomic<double>[]>(static_cast<size_t>(voices) * kMaxClips);
    for (size_t i = 0; i < static_cast<size_t>(voices) * kMaxClips; ++i)
        publishedVoice[i].store(-1.0, std::memory_order_relaxed);
    restart();
}

void ClipPlayer::restart()
{
    for (auto& h : heads)
        h = ClipPlayhead{};
    smooth.fill(-1.0f);
    ppq = 0.0;
    gains.fill(1.0f);
    wantedPlaying.fill(true);
}

void ClipPlayer::applyTransport(std::uint32_t wanted, std::uint32_t restartBits, int numSamples)
{
    const float step = static_cast<float>(numSamples) / (0.01f * static_cast<float>(sampleRate));
    for (int i = 0; i < kMaxClips; ++i)
    {
        const auto k = static_cast<size_t>(i);
        const std::uint32_t bit = 1U << static_cast<unsigned>(i);
        if ((restartBits & bit) != 0)
        {
            heads[k] = ClipPlayhead{};
            heads[k].start();
            for (int v = 0; v < voices; ++v)
                voiceHeads[static_cast<size_t>(v) * kMaxClips + k].start();
            std::fill_n(smooth.begin() + static_cast<std::ptrdiff_t>(i) * kMaxClipLanes, kMaxClipLanes, -1.0f);
            for (int v = 0; v < voices; ++v)
                std::fill_n(voiceSmooth.begin() + (static_cast<std::ptrdiff_t>(v) * laneSlots + i * kMaxClipLanes),
                            kMaxClipLanes, -1.0f);
        }
        wantedPlaying[k] = (wanted & bit) != 0;
        const float target = wantedPlaying[k] ? 1.0f : 0.0f;
        gains[k] = target > gains[k] ? std::min(gains[k] + step, target) : std::max(gains[k] - step, target);
    }
}

void ClipPlayer::setHostTransport(bool hasPos, bool playing, double position)
{
    hostHasPpq = hasPos;
    hostPlaying = playing;
    if (hasPos)
        ppq = position;
}

void ClipPlayer::noteOn()
{
    pendingNoteOn = true;
}

void ClipPlayer::beginChunk(const ClipBank* b, int numSamples, double bpm, bool anyNoteHeld)
{
    bank = b;
    chunk = numSamples;
    tempo = bpm;
    if (bank == nullptr)
        return;
    for (size_t i = 0; i < bank->clips.size(); ++i)
    {
        const Clip& c = bank->clips[i];
        auto& h = heads[i];
        if (parked(static_cast<int>(i)))
            continue;
        switch (c.trigger)
        {
        case ClipTrigger::Voice:
        case ClipTrigger::GlobalRetrigger:
            if (!anyNoteHeld)
                h.release();
            if (pendingNoteOn)
                h.start();
            break;
        case ClipTrigger::GlobalFree:
            if (!h.running)
                h.start();
            break;
        case ClipTrigger::Transport:
            if (!h.running)
                h.start();
            if (hostPlaying && hostHasPpq)
            {
                const double beats = std::max(ppq, 0.0);
                h.elapsed = (c.sync ? beats : beats * 60.0 / tempo) * c.rate;
            }
            break;
        }
    }
    pendingNoteOn = false;
}

void ClipPlayer::beginVoice(int voice, std::uint64_t age, bool active, bool releasing)
{
    if (bank == nullptr || voice < 0 || voice >= voices)
        return;
    auto* vh = &voiceHeads[static_cast<size_t>(voice) * kMaxClips];
    if (!active)
    {
        for (int i = 0; i < kMaxClips; ++i)
            vh[i].running = false;
        voiceAge[static_cast<size_t>(voice)] = 0;
        return;
    }
    if (voiceAge[static_cast<size_t>(voice)] != age)
    {
        voiceAge[static_cast<size_t>(voice)] = age;
        for (int i = 0; i < kMaxClips; ++i)
            vh[i].start();
        std::fill_n(voiceSmooth.begin() + static_cast<std::ptrdiff_t>(voice) * laneSlots, laneSlots, -1.0f);
    }
    if (releasing)
        for (int i = 0; i < kMaxClips; ++i)
            vh[i].release();
}

float ClipPlayer::evaluate(const ClipPlayhead& head, int source, float& smoothState)
{
    if (bank == nullptr)
        return 0.0f;
    const Clip* c = bank->clipOf(source);
    const ClipLane* l = bank->lane(source);
    if (c == nullptr || l == nullptr)
        return 0.0f;
    std::uint32_t cycle = 0;
    const double pos = clipPosition(*c, head.elapsed, head.releasedAt, cycle);
    float v = evaluateLane(*c, *l, pos, cycle);
    if (l->smoothingMs > 0.0f)
    {
        if (smoothState < 0.0f)
            smoothState = v;
        const float a =
            1.0f - std::exp(-static_cast<float>(chunk) / (l->smoothingMs * 0.001f * static_cast<float>(sampleRate)));
        smoothState += (v - smoothState) * a;
        v = smoothState;
    }
    return v;
}

float ClipPlayer::globalValue(int source)
{
    if (!isClipSource(source))
        return 0.0f;
    const int k = source - kSourceClip;
    return evaluate(heads[static_cast<size_t>(k / kMaxClipLanes)], source, smooth[static_cast<size_t>(k)]);
}

float ClipPlayer::voiceValue(int voice, int source)
{
    if (!isClipSource(source) || voice < 0 || voice >= voices)
        return 0.0f;
    const int k = source - kSourceClip;
    return evaluate(voiceHeads[static_cast<size_t>(voice) * kMaxClips + static_cast<size_t>(k / kMaxClipLanes)], source,
                    voiceSmooth[static_cast<size_t>(voice) * laneSlots + static_cast<size_t>(k)]);
}

void ClipPlayer::endChunk()
{
    const double seconds = chunk / sampleRate;
    if (bank != nullptr)
    {
        for (size_t i = 0; i < bank->clips.size(); ++i)
        {
            const Clip& c = bank->clips[i];
            if (parked(static_cast<int>(i)))
                continue;
            const double units = clipUnits(c, seconds, tempo);
            auto& h = heads[i];
            if (h.running && !(c.trigger == ClipTrigger::Transport && hostPlaying && hostHasPpq))
                h.elapsed += units;
            for (int v = 0; v < voices; ++v)
            {
                auto& vh = voiceHeads[static_cast<size_t>(v) * kMaxClips + i];
                if (vh.running)
                    vh.elapsed += units;
            }
        }
    }
    if (hostPlaying)
        ppq += seconds * tempo / 60.0;
}

double ClipPlayer::globalPosition(int clip) const
{
    if (bank == nullptr || clip < 0 || clip >= static_cast<int>(bank->clips.size()))
        return 0.0;
    std::uint32_t cycle = 0;
    const auto& h = heads[static_cast<size_t>(clip)];
    return clipPosition(bank->clips[static_cast<size_t>(clip)], h.elapsed, h.releasedAt, cycle);
}

void ClipPlayer::publishPositions()
{
    if (publishedVoice == nullptr)
        return;
    for (int c = 0; c < kMaxClips; ++c)
    {
        publishedGlobal[static_cast<size_t>(c)].store(globalPosition(c), std::memory_order_relaxed);
        const bool voiceClip = bank != nullptr && c < static_cast<int>(bank->clips.size()) &&
                               bank->clips[static_cast<size_t>(c)].trigger == ClipTrigger::Voice;
        for (int v = 0; v < voices; ++v)
        {
            double pos = -1.0;
            const auto& h = voiceHeads[static_cast<size_t>(v) * kMaxClips + static_cast<size_t>(c)];
            if (voiceClip && h.running)
            {
                std::uint32_t cycle = 0;
                pos = clipPosition(bank->clips[static_cast<size_t>(c)], h.elapsed, h.releasedAt, cycle);
            }
            publishedVoice[static_cast<size_t>(v) * kMaxClips + static_cast<size_t>(c)].store(pos,
                                                                                              std::memory_order_relaxed);
        }
    }
}

double ClipPlayer::publishedGlobalPosition(int clip) const
{
    return clip >= 0 && clip < kMaxClips ? publishedGlobal[static_cast<size_t>(clip)].load(std::memory_order_relaxed) : 0.0;
}

double ClipPlayer::publishedVoicePosition(int voice, int clip) const
{
    if (publishedVoice == nullptr || voice < 0 || voice >= voices || clip < 0 || clip >= kMaxClips)
        return -1.0;
    return publishedVoice[static_cast<size_t>(voice) * kMaxClips + static_cast<size_t>(clip)].load(
        std::memory_order_relaxed);
}

const ClipLane* ClipBank::lane(int source) const
{
    if (!isClipSource(source))
        return nullptr;
    const int k = source - kSourceClip;
    const int c = k / kMaxClipLanes;
    const int l = k % kMaxClipLanes;
    if (c >= static_cast<int>(clips.size()) || l >= laneCount(c))
        return nullptr;
    return &clips[static_cast<size_t>(c)].lanes[static_cast<size_t>(l)];
}

const Clip* ClipBank::clipOf(int source) const
{
    if (!isClipSource(source))
        return nullptr;
    const int c = (source - kSourceClip) / kMaxClipLanes;
    return c < static_cast<int>(clips.size()) ? &clips[static_cast<size_t>(c)] : nullptr;
}

ClipStore::ClipStore()
{
    shared.publish(std::make_unique<ClipBank>());
}

void ClipStore::publishLocked()
{
    if (bank.clips.size() > static_cast<size_t>(kMaxClips))
        bank.clips.resize(static_cast<size_t>(kMaxClips));
    for (auto& c : bank.clips)
        c.sanitise();
    shared.publish(std::make_unique<ClipBank>(bank));
    rev.fetch_add(1, std::memory_order_acq_rel);
}

void ClipStore::edit(const std::function<void(ClipBank&)>& change)
{
    std::lock_guard<std::mutex> lock(mutex);
    change(bank);
    publishLocked();
}

void ClipStore::replace(const ClipBank& b)
{
    std::lock_guard<std::mutex> lock(mutex);
    bank = b;
    publishLocked();
}

ClipBank ClipStore::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return bank;
}

void ClipStore::clear()
{
    replace(ClipBank{});
}

json::Value clipBankToJson(const ClipBank& bank)
{
    using json::Value;
    Value arr = Value::array();
    for (const auto& c : bank.clips)
    {
        Value o = Value::object();
        o.set("name", c.name);
        o.set("sync", c.sync);
        o.set("length", c.length);
        o.set("grid", c.grid);
        Value loop = Value::array();
        loop.push(c.loopStart);
        loop.push(c.loopEnd);
        o.set("loop", loop);
        o.set("mode", modeName(c.mode));
        o.set("trigger", triggerName(c.trigger));
        o.set("rate", c.rate);
        o.set("sustainLoop", c.sustainLoop);
        Value lanes = Value::array();
        for (const auto& l : c.lanes)
        {
            Value lo = Value::object();
            lo.set("name", l.name);
            lo.set("kind", l.kind == ClipLaneKind::Steps ? "steps" : "curve");
            Value pts = Value::array();
            for (const auto& p : l.points)
            {
                Value pv = Value::array();
                pv.push(p.x);
                pv.push(static_cast<double>(p.y));
                pv.push(static_cast<double>(p.curve));
                pv.push(shapeName(p.shape));
                pts.push(pv);
            }
            lo.set("points", pts);
            Value steps = Value::array();
            for (const float s : l.steps)
                steps.push(static_cast<double>(s));
            lo.set("steps", steps);
            lo.set("stepGlide", static_cast<double>(l.stepGlide));
            lo.set("smoothingMs", static_cast<double>(l.smoothingMs));
            lo.set("humanise", static_cast<double>(l.humanise));
            lo.set("randomise", static_cast<double>(l.randomise));
            lo.set("seed", static_cast<double>(l.seed));
            lanes.push(lo);
        }
        o.set("lanes", lanes);
        arr.push(o);
    }
    return arr;
}

bool clipBankFromJson(const json::Value& value, ClipBank& out)
{
    ClipBank bank;
    if (!value.isArray())
    {
        out = bank;
        return value.isNull();
    }
    static constexpr std::array<ClipPlayMode, 3> modes = {ClipPlayMode::OneShot, ClipPlayMode::Loop, ClipPlayMode::PingPong};
    static constexpr std::array<ClipTrigger, 4> triggers = {ClipTrigger::Voice, ClipTrigger::GlobalRetrigger,
                                                            ClipTrigger::GlobalFree, ClipTrigger::Transport};
    static constexpr std::array<SegmentShape, 4> shapes = {SegmentShape::Curve, SegmentShape::Hold, SegmentShape::SCurve,
                                                           SegmentShape::Stairs};
    for (const auto& o : value.elements())
    {
        if (!o.isObject())
            continue;
        Clip c;
        c.name = o["name"].asString("");
        c.sync = o["sync"].asBool(true);
        c.length = o["length"].asNumber(4.0);
        c.grid = o["grid"].asNumber(0.25);
        c.loopStart = o["loop"][0].asNumber(0.0);
        c.loopEnd = o["loop"][1].asNumber(c.length);
        c.mode = parseName(o["mode"].asString("loop"), modes, &modeName, ClipPlayMode::Loop);
        c.trigger = parseName(o["trigger"].asString("voice"), triggers, &triggerName, ClipTrigger::Voice);
        c.rate = o["rate"].asNumber(1.0);
        c.sustainLoop = o["sustainLoop"].asBool(false);
        for (const auto& lo : o["lanes"].elements())
        {
            ClipLane l;
            l.name = lo["name"].asString("");
            l.kind = lo["kind"].asString("curve") == "steps" ? ClipLaneKind::Steps : ClipLaneKind::Curve;
            for (const auto& pv : lo["points"].elements())
            {
                ClipPoint p;
                p.x = pv[0].asNumber(0.0);
                p.y = static_cast<float>(pv[1].asNumber(0.0));
                p.curve = static_cast<float>(pv[2].asNumber(0.0));
                p.shape = parseName(pv[3].asString("curve"), shapes, &shapeName, SegmentShape::Curve);
                l.points.push_back(p);
            }
            for (const auto& s : lo["steps"].elements())
                l.steps.push_back(static_cast<float>(s.asNumber(0.0)));
            l.stepGlide = static_cast<float>(lo["stepGlide"].asNumber(0.0));
            l.smoothingMs = static_cast<float>(lo["smoothingMs"].asNumber(0.0));
            l.humanise = static_cast<float>(lo["humanise"].asNumber(0.0));
            l.randomise = static_cast<float>(lo["randomise"].asNumber(0.0));
            l.seed = static_cast<std::uint32_t>(lo["seed"].asNumber(1.0));
            c.lanes.push_back(std::move(l));
        }
        c.sanitise();
        bank.clips.push_back(std::move(c));
        if (static_cast<int>(bank.clips.size()) >= kMaxClips)
            break;
    }
    out = std::move(bank);
    return true;
}

} // namespace crucible
