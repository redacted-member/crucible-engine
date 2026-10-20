// SPDX-License-Identifier: MIT
#include <crucible/module/ModuleRack.h>

#include <crucible/module/LatencyReporter.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace crucible
{

namespace
{
constexpr double kOnOffFadeSeconds = 0.010;
}

namespace graph
{
FxPlan FxPlan::classic(int slotCount)
{
    FxPlan plan;
    FxLane lane;
    lane.inputs.push_back({FxSourceKind::Voices, -1, 1.0f});
    lane.byPosition = true;
    for (int i = 0; i < slotCount; ++i)
        lane.slots.push_back(i);
    plan.lanes.push_back(std::move(lane));
    return plan;
}

bool FxPlan::isInPlace() const
{
    if (lanes.size() != 1 || masterLevel != 1.0f)
        return false;
    const FxLane& l = lanes[0];
    return l.toMaster && l.level == 1.0f && l.inputs.size() == 1 && l.inputs[0].kind == FxSourceKind::Voices &&
           l.inputs[0].gain == 1.0f;
}
} // namespace graph

ModuleRack::ModuleRack(const ModuleRegistry& registry) : model(&registry.parameters())
{
    const auto& targets = model->modTargets();
    for (const auto* d : registry.ofKind(ModuleKind::Effect))
    {
        if (!d->createEffect)
            continue;
        Slot s;
        s.descriptor = d;
        s.copy = !d->instanceOf.empty();
        s.instance = d->createEffect();
        s.reporter = dynamic_cast<const LatencyReporter*>(s.instance.get());
        for (const auto& p : d->params)
        {
            const int g = model->indexOf(p.id);
            s.globalIndex.push_back(g);
            int t = -1;
            for (size_t k = 0; k < targets.size(); ++k)
                if (targets[k] == g)
                    t = static_cast<int>(k);
            s.genericTarget.push_back(t);
        }
        s.onIndex = model->indexOf(d->onParam);
        s.positionIndex = model->indexOf(d->positionParam);
        slots.push_back(std::move(s));
    }
    order.resize(slots.size());
    classicPlan = graph::FxPlan::classic(static_cast<int>(slots.size()));
    zeros.assign(64, 0.0f);
}

ModuleRack::~ModuleRack() = default;

void ModuleRack::prepare(const ModuleContext& context)
{
    for (auto& s : slots)
        s.instance->prepare(context);
    blockCapacity = std::max(context.maxBlockSize, 32);
    laneMemory.assign(static_cast<size_t>(kMaxLanes + 1) * 2 * static_cast<size_t>(blockCapacity), 0.0f);
    fadeMemory.assign(2 * static_cast<size_t>(blockCapacity), 0.0f);
    fadeStep = 1.0f / static_cast<float>(std::max(1.0, std::round(kOnOffFadeSeconds * context.sampleRate)));
    for (auto& s : slots)
        s.seeded = false;
}

void ModuleRack::reset()
{
    for (auto& s : slots)
    {
        s.instance->reset();
        s.seeded = false;
    }
}

bool ModuleRack::slotOn(const Slot& s, const float* current)
{
    return s.onIndex < 0 || current[s.onIndex] > 0.5f;
}

int ModuleRack::latencySamples(const float* current, const graph::FxPlan* plan) const
{
    const graph::FxPlan& p = plan != nullptr ? *plan : active != nullptr ? *active : classicPlan;
    std::array<int, kMaxLanes> laneLatency{};
    int total = 0;
    for (size_t l = 0; l < p.lanes.size() && l < laneLatency.size(); ++l)
    {
        const auto& lane = p.lanes[l];
        int latency = 0;
        for (const auto& in : lane.inputs)
            if (in.kind == graph::FxSourceKind::Lane && in.lane >= 0 && in.lane < static_cast<int>(l))
                latency = std::max(latency, laneLatency[static_cast<size_t>(in.lane)]);
        for (const int slot : lane.slots)
        {
            const auto& s = slots[static_cast<size_t>(slot)];
            if (!slotOn(s, current))
                continue;
            if (const auto* r = s.reporter)
                latency += std::max(0, r->latencySamples());
        }
        laneLatency[l] = latency;
        if (lane.toMaster)
            total = std::max(total, latency);
    }
    return total;
}

double ModuleRack::tailSeconds(const float* current) const
{
    double tail = 0.0;
    for (const auto& s : slots)
    {
        if (!slotOn(s, current))
            continue;
        if (const auto* r = s.reporter)
        {
            if (r->tailInfinite())
                return HUGE_VAL;
            tail = std::max(tail, r->tailSeconds());
        }
    }
    return tail;
}

void ModuleRack::publishReported(const float* current, const graph::FxPlan* plan)
{
    pubLatency.store(latencySamples(current, plan), std::memory_order_relaxed);
    pubTail.store(tailSeconds(current), std::memory_order_relaxed);
}

void ModuleRack::adoptLatencyParams(const float* current)
{
    EffectInputs in;
    in.current = current;
    in.defs = model->params().data();
    in.legacyDests = zeros.data();
    for (auto& s : slots)
    {
        if (s.reporter == nullptr)
            continue;
        in.globalIndex = s.globalIndex.data();
        in.genericTarget = nullptr;
        if (auto* lr = dynamic_cast<LatencyReporter*>(s.instance.get()))
            lr->adoptLatencyParams(in);
    }
}

EffectModule* ModuleRack::find(const std::string& moduleId)
{
    for (auto& s : slots)
        if (s.descriptor->id == moduleId)
            return s.instance.get();
    return nullptr;
}

int ModuleRack::slotOf(const std::string& moduleId) const
{
    for (size_t i = 0; i < slots.size(); ++i)
        if (slots[i].descriptor->id == moduleId)
            return static_cast<int>(i);
    return -1;
}

bool ModuleRack::validPlan(const graph::FxPlan& plan) const
{
    if (plan.lanes.empty() || plan.lanes.size() > static_cast<size_t>(kMaxLanes))
        return false;
    std::vector<char> used(slots.size(), 0);
    for (size_t i = 0; i < plan.lanes.size(); ++i)
    {
        const auto& lane = plan.lanes[i];
        for (const int s : lane.slots)
        {
            if (s < 0 || static_cast<size_t>(s) >= slots.size() || used[static_cast<size_t>(s)] != 0)
                return false;
            used[static_cast<size_t>(s)] = 1;
        }
        for (const auto& in : lane.inputs)
            if (in.kind == graph::FxSourceKind::Lane && (in.lane < 0 || static_cast<size_t>(in.lane) >= i))
                return false;
    }
    return true;
}

void ModuleRack::runLane(const graph::FxLane& lane, float* left, float* right, int n, const Context& c)
{
    const int count = static_cast<int>(lane.slots.size());
    int* ord = order.data();
    for (int i = 0; i < count; ++i)
        ord[i] = lane.slots[static_cast<size_t>(i)];
    if (lane.byPosition)
    {
        const auto position = [&](int slot)
        {
            const int idx = slots[static_cast<size_t>(slot)].positionIndex;
            return idx >= 0 ? static_cast<int>(c.current[idx]) : slot;
        };
        // Insertion sort: stable and allocation free.
        for (int i = 1; i < count; ++i)
        {
            const int key = ord[i];
            const int keyPos = position(key);
            int j = i - 1;
            while (j >= 0 && position(ord[j]) > keyPos)
            {
                ord[j + 1] = ord[j];
                --j;
            }
            ord[j + 1] = key;
        }
    }

    const auto& defs = model->params();
    EffectInputs in;
    in.current = c.current;
    in.defs = defs.data();
    in.genericDest = c.genericDest;
    in.tempo = c.bpm;
    in.auxL = c.auxL;
    in.auxR = c.auxR != nullptr ? c.auxR : c.auxL;
    in.noteValue = c.note;
    for (int k = 0; k < count; ++k)
    {
        auto& s = slots[static_cast<size_t>(ord[k])];
        const bool on = s.onIndex < 0 || c.current[s.onIndex] > 0.5f;
        if (!s.seeded)
        {
            s.fade = on ? 1.0f : 0.0f;
            s.seeded = true;
        }
        if (!on && s.fade <= 0.0f)
        {
            s.instance->deactivate();
            continue;
        }
        in.legacyDests = s.copy ? zeros.data() : c.legacyDests;
        in.globalIndex = s.globalIndex.data();
        in.genericTarget = c.genericDest != nullptr ? s.genericTarget.data() : nullptr;
        if (on && s.fade >= 1.0f)
        {
            s.instance->process(left, right, n, in);
            continue;
        }
        if (blockCapacity <= 0)
        {
            // Not prepared for fades: switch at once.
            s.fade = on ? 1.0f : 0.0f;
            if (on)
                s.instance->process(left, right, n, in);
            else
                s.instance->deactivate();
            continue;
        }
        // Switching on or off: the module's output and its input crossfade over 10 ms (BFX4). A module that is
        // switched on starts from cleared state; one that is switched off is deactivated when the fade has ended.
        const size_t cap = static_cast<size_t>(blockCapacity);
        float* dryL = fadeMemory.data();
        float* dryR = fadeMemory.data() + cap;
        for (int off = 0; off < n; off += blockCapacity)
        {
            const int m = std::min(blockCapacity, n - off);
            float* l = left + off;
            float* r = right + off;
            std::memcpy(dryL, l, static_cast<size_t>(m) * sizeof(float));
            std::memcpy(dryR, r, static_cast<size_t>(m) * sizeof(float));
            in.auxL = c.auxL != nullptr ? c.auxL + off : nullptr;
            in.auxR = c.auxR != nullptr ? c.auxR + off : in.auxL;
            s.instance->process(l, r, m, in);
            for (int i = 0; i < m; ++i)
            {
                s.fade = on ? std::min(1.0f, s.fade + fadeStep) : std::max(0.0f, s.fade - fadeStep);
                const float w = s.fade * s.fade * (3.0f - 2.0f * s.fade);
                l[i] = dryL[i] + (l[i] - dryL[i]) * w;
                r[i] = dryR[i] + (r[i] - dryR[i]) * w;
            }
        }
        in.auxL = c.auxL;
        in.auxR = c.auxR != nullptr ? c.auxR : c.auxL;
        if (!on && s.fade <= 0.0f)
            s.instance->deactivate();
    }
}

void ModuleRack::processSegment(float* left, float* right, int n, const Context& c)
{
    const graph::FxPlan& plan = active != nullptr && blockCapacity > 0 ? *active : classicPlan;
    if (plan.isInPlace())
    {
        runLane(plan.lanes[0], left, right, n, c);
        return;
    }

    const size_t cap = static_cast<size_t>(blockCapacity);
    const auto laneL = [&](size_t i) { return laneMemory.data() + (2 * i) * cap; };
    const auto laneR = [&](size_t i) { return laneMemory.data() + (2 * i + 1) * cap; };
    float* dryL = laneL(kMaxLanes);
    float* dryR = laneR(kMaxLanes);
    const size_t bytes = static_cast<size_t>(n) * sizeof(float);
    std::memcpy(dryL, left, bytes);
    std::memcpy(dryR, right, bytes);

    for (size_t i = 0; i < plan.lanes.size(); ++i)
    {
        const auto& lane = plan.lanes[i];
        float* l = laneL(i);
        float* r = laneR(i);
        std::fill(l, l + n, 0.0f);
        std::fill(r, r + n, 0.0f);
        for (const auto& src : lane.inputs)
        {
            const float *sl = nullptr, *sr = nullptr;
            switch (src.kind)
            {
            case graph::FxSourceKind::Voices:
                sl = dryL;
                sr = dryR;
                break;
            case graph::FxSourceKind::Aux:
                sl = c.auxL;
                sr = c.auxR != nullptr ? c.auxR : c.auxL;
                break;
            case graph::FxSourceKind::Lane:
                sl = laneL(static_cast<size_t>(src.lane));
                sr = laneR(static_cast<size_t>(src.lane));
                break;
            }
            if (sl == nullptr)
                continue;
            for (int k = 0; k < n; ++k)
            {
                l[k] += sl[k] * src.gain;
                r[k] += sr[k] * src.gain;
            }
        }
        runLane(lane, l, r, n, c);
    }

    std::fill(left, left + n, 0.0f);
    std::fill(right, right + n, 0.0f);
    for (size_t i = 0; i < plan.lanes.size(); ++i)
    {
        const auto& lane = plan.lanes[i];
        if (!lane.toMaster)
            continue;
        const float g = lane.level * plan.masterLevel;
        const float* l = laneL(i);
        const float* r = laneR(i);
        for (int k = 0; k < n; ++k)
        {
            left[k] += l[k] * g;
            right[k] += r[k] * g;
        }
    }
}

void ModuleRack::process(float* left, float* right, int numSamples, const float* current, const float* legacyDests,
                         const float* genericDest, double bpm, const float* auxLeft, const float* auxRight)
{
    Context c{current, legacyDests, genericDest, bpm, auxLeft, auxRight, note};
    const graph::FxPlan& plan = active != nullptr && blockCapacity > 0 ? *active : classicPlan;
    if (plan.isInPlace())
    {
        // The classic chain works on blocks of any size, in place.
        processSegment(left, right, numSamples, c);
        return;
    }
    for (int off = 0; off < numSamples; off += blockCapacity)
    {
        const int n = std::min(blockCapacity, numSamples - off);
        Context seg = c;
        seg.auxL = auxLeft != nullptr ? auxLeft + off : nullptr;
        seg.auxR = auxRight != nullptr ? auxRight + off : nullptr;
        processSegment(left + off, right + off, n, seg);
    }
}

} // namespace crucible
