// SPDX-License-Identifier: MIT
#include <crucible/MotionRecorder.h>

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
constexpr int kHoldChunks = 12; // a touch ends after this many unchanged control chunks (about 8 ms)
constexpr float kTouchEpsilon = 1.0e-5f;

// Ramer-Douglas-Peucker on (x / length, y), keeps the ends.
void thin(const std::vector<ClipPoint>& in, double length, float tolerance, std::vector<ClipPoint>& out)
{
    out.clear();
    if (in.size() <= 2)
    {
        out = in;
        return;
    }
    std::vector<char> keep(in.size(), 0);
    keep.front() = keep.back() = 1;
    std::vector<std::pair<size_t, size_t>> stack;
    stack.emplace_back(0, in.size() - 1);
    const double inv = 1.0 / std::max(length, 1.0e-9);
    while (!stack.empty())
    {
        const auto [a, b] = stack.back();
        stack.pop_back();
        if (b <= a + 1)
            continue;
        const double ax = in[a].x * inv, bx = in[b].x * inv;
        double worst = 0.0;
        size_t at = a;
        for (size_t i = a + 1; i < b; ++i)
        {
            const double f = bx > ax ? (in[i].x * inv - ax) / (bx - ax) : 0.0;
            const double y = in[a].y + (in[b].y - in[a].y) * f;
            const double d = std::abs(in[i].y - y);
            if (d > worst)
            {
                worst = d;
                at = i;
            }
        }
        if (worst > tolerance)
        {
            keep[at] = 1;
            stack.emplace_back(a, at);
            stack.emplace_back(at, b);
        }
    }
    for (size_t i = 0; i < in.size(); ++i)
        if (keep[i] != 0)
            out.push_back(in[i]);
}
} // namespace

double detectClipLength(const Clip& clip, double duration)
{
    const double unit = clip.grid > 0.0 ? clip.grid : (clip.sync ? 1.0 : 0.25);
    return std::max(unit, std::round(duration / unit) * unit);
}

void quantiseLane(const Clip& clip, ClipLane& lane, float strength)
{
    if (clip.grid <= 0.0)
        return;
    strength = std::clamp(strength, 0.0f, 1.0f);
    for (auto& p : lane.points)
        p.x += (clip.snap(p.x) - p.x) * strength;
    std::stable_sort(lane.points.begin(), lane.points.end(),
                     [](const ClipPoint& a, const ClipPoint& b) { return a.x < b.x; });
    // Points on the same place collapse into the later one.
    std::vector<ClipPoint> merged;
    for (const auto& p : lane.points)
    {
        if (!merged.empty() && std::abs(merged.back().x - p.x) < 1.0e-9)
            merged.back() = p;
        else
            merged.push_back(p);
    }
    lane.points = std::move(merged);
}

MotionRecorder::MotionRecorder(ClipStore& clips, ModRouteStore& routes)
    : clipStore(clips), routeStore(routes), model(&routes.parameterModel())
{
}

bool MotionRecorder::arm(const MotionSettings& s, float baseNormalised, float baseNormalised2)
{
    const int pIdx = model->indexOf(s.param);
    if (pIdx < 0 || s.clip < 0 || s.clip >= kMaxClips || s.lane < 0 || s.lane >= kMaxClipLanes ||
        !routeStore.accepts(s.param))
        return false;
    const bool two = !s.param2.empty();
    const int pIdx2 = two ? model->indexOf(s.param2) : -1;
    if (two && (pIdx2 < 0 || pIdx2 == pIdx || s.lane2 < 0 || s.lane2 >= kMaxClipLanes || s.lane2 == s.lane ||
                !routeStore.accepts(s.param2)))
        return false;
    if (active.load())
        stop();
    cfg = s;
    base[0] = std::clamp(baseNormalised, 0.0f, 1.0f);
    base[1] = std::clamp(baseNormalised2, 0.0f, 1.0f);
    for (auto& r : run)
        r.clear();
    ring.reset();
    lost.store(0);

    ClipTrigger trigger = ClipTrigger::Transport;
    double len = 4.0;
    bool isSync = true;
    double clipRate = 1.0;
    bool loops = true;
    clipStore.edit(
        [&](ClipBank& bank)
        {
            while (static_cast<int>(bank.clips.size()) <= s.clip)
            {
                Clip c;
                c.name = "Clip " + std::to_string(bank.clips.size() + 1);
                c.trigger = ClipTrigger::Transport;
                bank.clips.push_back(c);
            }
            Clip& c = bank.clips[static_cast<size_t>(s.clip)];
            while (static_cast<int>(c.lanes.size()) <= std::max(s.lane, two ? s.lane2 : 0))
            {
                ClipLane l;
                l.name = "Lane " + std::to_string(c.lanes.size() + 1);
                c.lanes.push_back(l);
            }
            for (const int lane : {s.lane, two ? s.lane2 : -1})
                if (lane >= 0 && !s.overdub)
                {
                    c.lanes[static_cast<size_t>(lane)].points.clear();
                    c.lanes[static_cast<size_t>(lane)].kind = ClipLaneKind::Curve;
                }
            trigger = c.trigger;
            len = c.length;
            isSync = c.sync;
            clipRate = c.rate;
            loops = c.mode != ClipPlayMode::OneShot;
        });

    const auto ensureRoute = [&](int lane, const std::string& param)
    {
        const int source = clipLaneSource(s.clip, lane);
        for (const auto& r : routeStore.routesFor(param))
            if (r.source == source)
                return true;
        ModRoute r;
        r.source = source;
        r.target = param;
        r.amount = 1.0f;
        r.polarity = static_cast<int>(ModPolarity::Bipolar);
        return routeStore.add(r);
    };
    if (!ensureRoute(s.lane, s.param) || (two && !ensureRoute(s.lane2, s.param2)))
        return false;

    const bool headFollowed = !s.detectLength && (trigger == ClipTrigger::Transport || trigger == ClipTrigger::GlobalFree);
    paramIndex[0].store(pIdx);
    paramIndex[1].store(pIdx2);
    clipIndex.store(s.clip);
    useHead.store(headFollowed);
    wrap.store(!s.detectLength && loops);
    sync.store(isSync);
    rate.store(clipRate);
    length.store(len);
    recTime.store(0.0);
    restartClock.store(true);
    active.store(true, std::memory_order_release);
    return true;
}

void MotionRecorder::commitRun(int track)
{
    auto& samples = run[static_cast<size_t>(track)];
    if (samples.empty())
        return;
    const double x0 = samples.front().time;
    const double x1 = samples.back().time;
    std::vector<ClipPoint> raw;
    raw.reserve(samples.size());
    for (const auto& s : samples)
    {
        ClipPoint p;
        p.x = s.time;
        p.y = std::clamp(0.5f + 0.5f * (s.value - base[static_cast<size_t>(track)]), 0.0f, 1.0f);
        raw.push_back(p);
    }
    samples.clear();
    const int laneIndex = track == 0 ? cfg.lane : cfg.lane2;
    thin(raw, length.load(), cfg.tolerance, runPoints);

    clipStore.edit(
        [&](ClipBank& bank)
        {
            if (cfg.clip >= static_cast<int>(bank.clips.size()))
                return;
            Clip& c = bank.clips[static_cast<size_t>(cfg.clip)];
            if (laneIndex >= static_cast<int>(c.lanes.size()))
                return;
            ClipLane& lane = c.lanes[static_cast<size_t>(laneIndex)];
            std::vector<ClipPoint> fresh = runPoints;
            if (cfg.quantise && c.grid > 0.0)
            {
                ClipLane tmp;
                tmp.points = fresh;
                quantiseLane(c, tmp);
                fresh = std::move(tmp.points);
            }
            // The new touch replaces what was there between its first and last sample.
            const double lo = fresh.empty() ? x0 : std::min(x0, fresh.front().x);
            const double hi = fresh.empty() ? x1 : std::max(x1, fresh.back().x);
            std::erase_if(lane.points, [&](const ClipPoint& p) { return p.x >= lo - 1.0e-9 && p.x <= hi + 1.0e-9; });
            lane.points.insert(lane.points.end(), fresh.begin(), fresh.end());
            std::stable_sort(lane.points.begin(), lane.points.end(),
                             [](const ClipPoint& a, const ClipPoint& b) { return a.x < b.x; });
        });
}

int MotionRecorder::drain()
{
    int added = 0;
    MotionSample s;
    while (ring.pop(s))
    {
        auto& samples = run[std::min<size_t>(s.track, 1)];
        const int track = static_cast<int>(std::min<size_t>(s.track, 1));
        if (s.touched)
        {
            // A new sample that is earlier than the last one means the clock wrapped: close the previous touch.
            if (!samples.empty() && s.time < samples.back().time)
            {
                added += static_cast<int>(samples.size());
                commitRun(track);
            }
            samples.push_back(s);
        }
        else
        {
            samples.push_back(s);
            added += static_cast<int>(samples.size());
            commitRun(track);
        }
    }
    return added;
}

void MotionRecorder::finish()
{
    for (int t = 0; t < 2; ++t)
        if (!run[static_cast<size_t>(t)].empty())
            commitRun(t);
}

void MotionRecorder::stop()
{
    if (!active.exchange(false, std::memory_order_acq_rel))
        return;
    drain();
    finish();
    if (cfg.detectLength)
    {
        const double duration = recTime.load();
        clipStore.edit(
            [&](ClipBank& bank)
            {
                if (cfg.clip >= static_cast<int>(bank.clips.size()))
                    return;
                Clip& c = bank.clips[static_cast<size_t>(cfg.clip)];
                const bool loopAtEnd = std::abs(c.loopEnd - c.length) < 1.0e-9;
                c.length = detectClipLength(c, duration);
                if (loopAtEnd || c.loopEnd > c.length)
                    c.loopEnd = c.length;
                c.loopStart = std::min(c.loopStart, c.length);
            });
    }
}

void MotionRecorder::capture(int numSamples, double bpm, const float* values, double headPosition)
{
    const bool on = active.load(std::memory_order_acquire);
    if (!on)
    {
        wasActive = false;
        return;
    }
    const auto& defs = model->params();
    std::array<int, 2> pIdx{paramIndex[0].load(std::memory_order_relaxed), paramIndex[1].load(std::memory_order_relaxed)};
    if (pIdx[0] < 0 || static_cast<size_t>(pIdx[0]) >= defs.size())
        return;
    for (auto& p : pIdx)
        if (p >= 0 && static_cast<size_t>(p) >= defs.size())
            p = -1;
    if (!wasActive || restartClock.exchange(false))
    {
        wasActive = true;
        for (size_t t = 0; t < 2; ++t)
        {
            touching[t] = false;
            idleChunks[t] = 0;
            lastNorm[t] = pIdx[t] >= 0 ? toNormalised(defs[static_cast<size_t>(pIdx[t])], values[pIdx[t]]) : -1.0f;
        }
        clock = 0.0;
    }
    const double len = length.load(std::memory_order_relaxed);
    const double t = (useHead.load(std::memory_order_relaxed) && headPosition >= 0.0) ? headPosition : clock;
    recTime.store(t, std::memory_order_relaxed);

    std::array<float, 2> norm{0.0f, 0.0f};
    for (size_t k = 0; k < 2; ++k)
    {
        if (pIdx[k] < 0)
            continue;
        norm[k] = toNormalised(defs[static_cast<size_t>(pIdx[k])], values[pIdx[k]]);
        const bool moved = std::abs(norm[k] - lastNorm[k]) > kTouchEpsilon;
        lastNorm[k] = norm[k];
        if (moved)
        {
            idleChunks[k] = 0;
            touching[k] = true;
            if (!ring.push({t, static_cast<std::uint8_t>(k), norm[k], true}))
                lost.fetch_add(1, std::memory_order_relaxed);
        }
        else if (touching[k] && ++idleChunks[k] >= kHoldChunks)
        {
            touching[k] = false;
            if (!ring.push({t, static_cast<std::uint8_t>(k), norm[k], false}))
                lost.fetch_add(1, std::memory_order_relaxed);
        }
    }

    if (!useHead.load(std::memory_order_relaxed) || headPosition < 0.0)
    {
        clock += (numSamples / sampleRate) * rate.load(std::memory_order_relaxed) *
                 (sync.load(std::memory_order_relaxed) ? bpm / 60.0 : 1.0);
        if (wrap.load(std::memory_order_relaxed) && clock >= len)
        {
            for (size_t k = 0; k < 2; ++k)
                if (touching[k])
                {
                    touching[k] = false;
                    if (!ring.push({len, static_cast<std::uint8_t>(k), norm[k], false}))
                        lost.fetch_add(1, std::memory_order_relaxed);
                }
            clock = std::fmod(clock, len);
        }
    }
}

} // namespace crucible
