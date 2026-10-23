// SPDX-License-Identifier: MIT
#include <crucible/Sequencer.h>
#include <crucible/module/Module.h>

#include <crucible/ParameterMap.h>

#include "TextFormat.h"

#include <algorithm>
#include <cmath>

namespace crucible
{
namespace
{
// Host tempo is clamped to a musical range, a host position beyond kMaxPositionBeats is ignored and a block never visits
// more than kMaxStepsPerBlock steps, so a bad host value cannot spin the audio thread.
constexpr double kMinBpm = 1.0;
constexpr double kMaxBpm = 1000.0;
constexpr double kMaxPositionBeats = 1.0e12;
constexpr int kMaxStepsPerBlock = 4096;
} // namespace

// ---------------------------------------------------------------------------------------------------------------
SeqStore::SeqStore()
{
    publish(std::make_unique<SeqBank>());
}

void SeqStore::publish(std::unique_ptr<SeqBank> bank)
{
    // Caller holds the mutex (or is the constructor).
    // seq_cst pairs with the reader's pin-and-recheck in acquire(): a release store followed by an acquire load of
    // inUse may be reordered, which would let the writer free a bank the reader has just pinned.
    published.store(bank.get(), std::memory_order_seq_cst);
    banks.push_back(std::move(bank));
}

void SeqStore::edit(const std::function<void(SeqBank&)>& change)
{
    std::lock_guard<std::mutex> lock(mutex);
    auto copy = std::make_unique<SeqBank>(*published.load());
    change(*copy);
    publish(std::move(copy));
    // Drop retired banks the audio thread is not holding.
    const SeqBank* pinned = inUse.load(std::memory_order_seq_cst);
    const SeqBank* current = published.load(std::memory_order_seq_cst);
    banks.erase(std::remove_if(banks.begin(), banks.end(),
                               [&](const std::unique_ptr<SeqBank>& b) { return b.get() != pinned && b.get() != current; }),
                banks.end());
}

void SeqStore::replace(const SeqBank& bank)
{
    edit([&bank](SeqBank& b) { b = bank; });
}

SeqBank SeqStore::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex);
    return *published.load();
}

SeqPattern SeqStore::snapshotPattern(int slot) const
{
    std::lock_guard<std::mutex> lock(mutex);
    return published.load()->pattern[static_cast<size_t>(std::clamp(slot, 0, kSeqSlots - 1))];
}

void SeqStore::collectGarbage()
{
    std::lock_guard<std::mutex> lock(mutex);
    const SeqBank* pinned = inUse.load(std::memory_order_seq_cst);
    const SeqBank* current = published.load(std::memory_order_seq_cst);
    banks.erase(std::remove_if(banks.begin(), banks.end(),
                               [&](const std::unique_ptr<SeqBank>& b) { return b.get() != pinned && b.get() != current; }),
                banks.end());
}

const SeqBank* SeqStore::acquire() noexcept
{
    for (;;)
    {
        SeqBank* p = published.load(std::memory_order_acquire);
        inUse.store(p, std::memory_order_seq_cst);
        if (published.load(std::memory_order_seq_cst) == p)
            return p;
    }
}

// ---------------------------------------------------------------------------------------------------------------
std::vector<PatternRecord> seqBankSerialise(const SeqBank& bank, const ParameterModel* modelPtr)
{
    const ParameterModel& model = modelPtr != nullptr ? *modelPtr : ModuleRegistry::instance().parameters();
    std::vector<PatternRecord> out;
    for (int slot = 0; slot < kSeqSlots; ++slot)
    {
        const auto& pat = bank.pattern[static_cast<size_t>(slot)];
        std::vector<std::string> rows;
        for (int i = 0; i < kSeqMaxSteps; ++i)
        {
            const auto& s = pat.steps[static_cast<size_t>(i)];
            const SeqStep defaults;
            const bool hasLocks = std::any_of(s.locks.begin(), s.locks.end(), [](const SeqLock& l) { return l.param >= 0; });
            if (!s.on && !hasLocks && s.note == defaults.note && s.velocity == defaults.velocity)
                continue;
            std::string row = std::to_string(i) + "," + std::to_string(s.on ? 1 : 0) + "," +
                              std::to_string(static_cast<int>(s.note)) + "," + std::to_string(static_cast<int>(s.velocity)) +
                              "," + text::fixed(s.gate, 4) + "," + text::fixed(s.probability, 4) + "," +
                              std::to_string(static_cast<int>(s.ratchet));
            for (const auto& l : s.locks)
                if (l.param >= 0)
                    row += "," + model.params()[static_cast<size_t>(l.param)].id + "=" + text::fixed(l.value, 6);
            rows.push_back(row);
        }
        out.push_back({slot, pat.lengthBars, text::join(rows, ';')});
    }
    return out;
}

void seqBankDeserialise(const std::vector<PatternRecord>& records, SeqBank& bank, const ParameterModel* modelPtr)
{
    const ParameterModel& model = modelPtr != nullptr ? *modelPtr : ModuleRegistry::instance().parameters();
    bank = SeqBank();
    for (const auto& record : records)
    {
        const int slot = record.slot;
        if (slot < 0 || slot >= kSeqSlots)
            continue;
        auto& pat = bank.pattern[static_cast<size_t>(slot)];
        pat.lengthBars = std::clamp(record.bars, 1, kSeqMaxBars);
        for (const auto& row : text::split(record.steps, ';'))
        {
            const auto f = text::split(row, ',');
            if (f.size() < 7)
                continue;
            const int idx = text::toInt(f[0]);
            if (idx < 0 || idx >= kSeqMaxSteps)
                continue;
            auto& s = pat.steps[static_cast<size_t>(idx)];
            s.on = text::toInt(f[1]) != 0;
            s.note = static_cast<std::uint8_t>(std::clamp(text::toInt(f[2]), 0, 127));
            s.velocity = static_cast<std::uint8_t>(std::clamp(text::toInt(f[3]), 1, 127));
            s.gate = std::clamp(text::toFloat(f[4]), 0.0f, 8.0f);
            s.probability = std::clamp(text::toFloat(f[5]), 0.0f, 1.0f);
            s.ratchet = static_cast<std::uint8_t>(std::clamp(text::toInt(f[6]), 1, 4));
            int lock = 0;
            for (size_t i = 7; i < f.size() && lock < kSeqLocks; ++i)
            {
                const auto eq = f[i].find('=');
                if (eq == std::string::npos)
                    continue;
                const int pi = model.indexOf(f[i].substr(0, eq));
                if (pi < 0)
                    continue;
                s.locks[static_cast<size_t>(lock++)] = {static_cast<std::int16_t>(pi), text::toFloat(f[i].substr(eq + 1))};
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
double seqRateBeats(int rate)
{
    static constexpr double table[] = {0.25, 0.5, 0.125, 1.0 / 6.0, 1.0 / 3.0, 1.0 / 12.0};
    return table[std::clamp(rate, 0, 5)];
}

int seqChainSlots(int chain)
{
    static constexpr int table[] = {0, 2, 3, 4, 6, 8, 12};
    return table[std::clamp(chain, 0, 6)];
}

float Sequencer::random01(std::uint32_t seed, std::int64_t k)
{
    std::uint64_t x = static_cast<std::uint64_t>(k) * 0x9e3779b97f4a7c15ULL + seed;
    x ^= x >> 30;
    x *= 0xbf58476d1ce4e5b9ULL;
    x ^= x >> 27;
    x *= 0x94d049bb133111ebULL;
    x ^= x >> 31;
    return static_cast<float>(x >> 40) * (1.0f / 16777216.0f);
}

Sequencer::Position Sequencer::positionFor(const SeqBank& bank, const SeqSettings& s, std::int64_t k, std::uint32_t seed)
{
    const int chainN = seqChainSlots(s.chain);
    int slot = std::clamp(s.pattern, 0, kSeqSlots - 1);
    std::int64_t local = k;
    int length = bank.pattern[static_cast<size_t>(slot)].lengthSteps();
    std::int64_t cycle = 0;
    if (chainN > 0)
    {
        std::int64_t total = 0;
        for (int i = 0; i < chainN; ++i)
            total += bank.pattern[static_cast<size_t>(i)].lengthSteps();
        const std::int64_t kk = k % total;
        std::int64_t base = 0;
        for (int i = 0; i < chainN; ++i)
        {
            const int len = bank.pattern[static_cast<size_t>(i)].lengthSteps();
            if (kk < base + len)
            {
                slot = i;
                local = kk - base;
                length = len;
                break;
            }
            base += len;
        }
        cycle = k / total;
    }
    else
    {
        cycle = local / length;
        local %= length;
    }

    int idx = static_cast<int>(local);
    switch (s.direction)
    {
    case 1:
        idx = length - 1 - idx;
        break;
    case 2:
        if (length > 1)
        {
            const int m = static_cast<int>((cycle * length + local) % (2 * (length - 1)));
            idx = m < length ? m : 2 * (length - 1) - m;
        }
        break;
    case 3:
        idx = static_cast<int>(random01(seed ^ 0x5bd1e995U, k) * static_cast<float>(length)) % length;
        break;
    default:
        break;
    }
    return {slot, idx};
}

void Sequencer::prepare(double rate)
{
    sampleRate = rate;
    reset();
}

void Sequencer::reset()
{
    active = false;
    internalBeat = 0.0;
    lastEnd = -1.0;
    pendingCount = 0;
    activeLockCount = 0;
    lastHandled = -1;
    last = {};
}

void Sequencer::push(const Pending& p)
{
    if (pendingCount < static_cast<int>(pending.size()))
        pending[static_cast<size_t>(pendingCount++)] = p;
}

void Sequencer::handleStep(const SeqBank& bank, const SeqSettings& s, std::int64_t k, double beat, double stepBeats)
{
    const Position pos = positionFor(bank, s, k, seed);
    last = pos;
    const SeqStep& step = bank.pattern[static_cast<size_t>(pos.slot)].steps[static_cast<size_t>(pos.index)];

    push({beat, SeqEvent::Type::Position, pos.index + 256 * pos.slot, 0.0f, false});

    const bool passes = random01(seed, k) < step.probability;
    const bool applyLocks = !step.on || passes;

    // Locks: clear those no longer present, then set the step's own.
    std::array<int, kSeqLocks> now{};
    int nowCount = 0;
    if (applyLocks)
        for (const auto& l : step.locks)
            if (l.param >= 0)
                now[static_cast<size_t>(nowCount++)] = l.param;
    for (int i = 0; i < activeLockCount; ++i)
    {
        const int p = activeLocks[static_cast<size_t>(i)];
        if (std::find(now.begin(), now.begin() + nowCount, p) == now.begin() + nowCount)
            push({beat, SeqEvent::Type::LockClear, p, 0.0f, false});
    }
    activeLockCount = 0;
    if (applyLocks)
        for (const auto& l : step.locks)
            if (l.param >= 0 && activeLockCount < static_cast<int>(activeLocks.size()))
            {
                activeLocks[static_cast<size_t>(activeLockCount++)] = l.param;
                push({beat, SeqEvent::Type::LockSet, l.param, l.value, false});
            }

    if (!step.on || !passes)
        return;

    const int note = std::clamp(static_cast<int>(step.note) + s.transpose + (s.keyTrigger ? s.keyTranspose : 0), 0, 127);
    const int ratchet = std::clamp(static_cast<int>(step.ratchet), 1, 4);
    const double sub = stepBeats / ratchet;
    const float vel = static_cast<float>(step.velocity) / 127.0f;
    for (int j = 0; j < ratchet; ++j)
    {
        const double on = beat + j * sub;
        const bool tie = ratchet == 1 && step.gate > 1.0f;
        const double len = ratchet == 1 ? step.gate * stepBeats : std::min(step.gate, 1.0f) * sub;
        const double off = on + std::max(len, 0.0005);

        // A tied note from the previous step continues instead of retriggering.
        bool tied = false;
        for (int i = 0; i < pendingCount; ++i)
        {
            auto& p = pending[static_cast<size_t>(i)];
            if (p.type == SeqEvent::Type::NoteOff && p.a == note && p.tied && p.beat > on)
            {
                p.beat = off;
                p.tied = tie;
                tied = true;
                break;
            }
        }
        if (tied)
            continue;
        push({on, SeqEvent::Type::NoteOn, note, vel, false});
        push({off, SeqEvent::Type::NoteOff, note, 0.0f, tie});
    }
}

int Sequencer::emit(double p0, double p1, double bps, int numSamples, SeqEvent* out, int cap)
{
    // Select and order the due events.
    std::array<int, 512> due{};
    int dueCount = 0;
    for (int i = 0; i < pendingCount; ++i)
        if (pending[static_cast<size_t>(i)].beat < p1)
            due[static_cast<size_t>(dueCount++)] = i;
    const auto before = [this](int a, int b)
    {
        const auto& x = pending[static_cast<size_t>(a)];
        const auto& y = pending[static_cast<size_t>(b)];
        if (x.beat < y.beat)
            return true;
        if (x.beat > y.beat)
            return false;
        return static_cast<int>(x.type) < static_cast<int>(y.type);
    };
    for (int i = 1; i < dueCount; ++i)
    {
        const int key = due[static_cast<size_t>(i)];
        int j = i - 1;
        while (j >= 0 && before(key, due[static_cast<size_t>(j)]))
        {
            due[static_cast<size_t>(j + 1)] = due[static_cast<size_t>(j)];
            --j;
        }
        due[static_cast<size_t>(j + 1)] = key;
    }

    int n = 0;
    for (int d = 0; d < dueCount && n < cap; ++d)
    {
        const auto& p = pending[static_cast<size_t>(due[static_cast<size_t>(d)])];
        const int offset = std::clamp(static_cast<int>(std::floor((p.beat - p0) / bps)), 0, numSamples - 1);
        out[n++] = {p.type, offset, p.a, p.value};
    }

    int kept = 0;
    for (int i = 0; i < pendingCount; ++i)
        if (pending[static_cast<size_t>(i)].beat >= p1)
            pending[static_cast<size_t>(kept++)] = pending[static_cast<size_t>(i)];
    pendingCount = kept;
    return n;
}

int Sequencer::flushAll(SeqEvent* out, int cap)
{
    int n = 0;
    // Release every sounding note and clear locks; scheduled note-ons are dropped.
    for (int i = 0; i < pendingCount && n < cap; ++i)
        if (pending[static_cast<size_t>(i)].type == SeqEvent::Type::NoteOff)
            out[n++] = {SeqEvent::Type::NoteOff, 0, pending[static_cast<size_t>(i)].a, 0.0f};
    for (int i = 0; i < activeLockCount && n < cap; ++i)
        out[n++] = {SeqEvent::Type::LockClear, 0, activeLocks[static_cast<size_t>(i)], 0.0f};
    pendingCount = 0;
    activeLockCount = 0;
    return n;
}

int Sequencer::process(const SeqBank& bank, const SeqSettings& s, const SeqHostTime& host, int numSamples, SeqEvent* out,
                       int cap)
{
    // A host position that is not finite or absurdly large cannot be followed (and would overflow the step index).
    const bool positionOk = !s.hostSync || (host.hasPpq && std::isfinite(host.ppq) && std::abs(host.ppq) < kMaxPositionBeats);
    const bool shouldRun =
        s.run && (!s.keyTrigger || s.keysHeld) && (!s.hostSync || (host.playing && host.hasPpq && positionOk));
    if (!shouldRun)
    {
        int n = 0;
        if (active)
            n = flushAll(out, cap);
        active = false;
        lastEnd = -1.0;
        internalBeat = 0.0;
        lastHandled = -1;
        return n;
    }

    const double rawBpm = host.hasBpm ? host.bpm : static_cast<double>(s.tempo);
    const double bpm = std::isfinite(rawBpm) ? std::clamp(rawBpm, kMinBpm, kMaxBpm) : static_cast<double>(s.tempo);
    const double bps = bpm / 60.0 / sampleRate;
    double p0;
    if (s.hostSync)
        p0 = host.ppq;
    else
        p0 = internalBeat;
    const double p1 = p0 + numSamples * bps;

    int n = 0;
    if (active && s.hostSync && lastEnd >= 0.0 && std::abs(p0 - lastEnd) > 2.0 * numSamples * bps + 1.0e-6)
    {
        n += flushAll(out, cap); // transport jumped (loop or locate)
        lastHandled = -1;
    }

    const double stepBeats = seqRateBeats(s.rate);
    const double swing = std::clamp(static_cast<double>(s.swingPercent), 0.0, 75.0) / 100.0 * stepBeats;
    std::int64_t k = static_cast<std::int64_t>(std::floor((p0 - swing) / stepBeats));
    if (k < 0)
        k = 0;
    for (int visited = 0; visited < kMaxStepsPerBlock; ++k, ++visited)
    {
        const double t = static_cast<double>(k) * stepBeats + ((k & 1) != 0 ? swing : 0.0);
        if (t >= p1)
            break;
        if (t < p0 - 1.0e-9 || k <= lastHandled)
            continue;
        lastHandled = k;
        handleStep(bank, s, k, t, stepBeats);
    }

    n += emit(p0, p1, bps, numSamples, out + n, cap - n);
    active = true;
    lastEnd = p1;
    if (!s.hostSync)
        internalBeat = p1;
    return n;
}

} // namespace crucible
