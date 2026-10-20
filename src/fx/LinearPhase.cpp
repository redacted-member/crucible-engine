// SPDX-License-Identifier: MIT
// Linear-phase kernel design, partitioned convolution and the design worker. See include/crucible/fx/LinearPhase.h.

#include <crucible/fx/LinearPhase.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>

namespace crucible::fx
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

int orderOf(int size)
{
    int o = 0;
    while ((1 << o) < size)
        ++o;
    return o;
}

Fft& fftFor(std::vector<std::unique_ptr<Fft>>& cache, int order)
{
    if (static_cast<int>(cache.size()) <= order)
        cache.resize(static_cast<size_t>(order) + 1);
    if (!cache[static_cast<size_t>(order)])
        cache[static_cast<size_t>(order)] = std::make_unique<Fft>(order);
    return *cache[static_cast<size_t>(order)];
}

// y += a * b over planar complex spectra.
inline void macPlanar(float* __restrict yr, float* __restrict yi, const float* __restrict xr, const float* __restrict xi,
                      const float* __restrict kr, const float* __restrict ki, int n) noexcept
{
    for (int b = 0; b < n; ++b)
    {
        yr[b] += xr[b] * kr[b] - xi[b] * ki[b];
        yi[b] += xr[b] * ki[b] + xi[b] * kr[b];
    }
}
} // namespace

// ---------------------------------------------------------------------------------------------------------------
void LinearPhaseDesigner::prepare(int maxLog2)
{
    maxLog2 = std::clamp(maxLog2, kMinKernelLog2, kMaxKernelLog2);
    for (int k = kMinKernelLog2; k <= maxLog2; ++k)
        fftFor(fft, k + 2);
    work.assign(static_cast<size_t>(2) << (maxLog2 + 2), 0.0f);
}

double LinearPhaseDesigner::gridOmega(int log2, int j)
{
    return kPi * static_cast<double>(j) / static_cast<double>(2 << log2);
}

void LinearPhaseDesigner::design(int log2, const float* magnitude, float* taps)
{
    const int m = 4 << log2; // transform size
    const int bins = m / 2 + 1;
    Fft& f = fftFor(fft, log2 + 2);
    if (work.size() < static_cast<size_t>(2) * static_cast<size_t>(m))
        work.assign(static_cast<size_t>(2) * static_cast<size_t>(m), 0.0f);
    std::fill(work.begin(), work.begin() + 2 * static_cast<std::ptrdiff_t>(m), 0.0f);
    for (int j = 0; j < bins; ++j)
        work[static_cast<size_t>(2 * j)] = magnitude[j];
    f.inverse(work.data());
    const int length = kernelTaps(log2), centre = kernelCentre(log2);
    // Raised-cosine taper over the outer 30 % of each side.
    const double start = 0.7 * centre;
    for (int k = 0; k <= centre; ++k)
    {
        const int idxA = k; // h[k]
        const int idxB = (m - k) % m;
        const double h = 0.5 * (static_cast<double>(work[static_cast<size_t>(idxA)]) +
                                static_cast<double>(work[static_cast<size_t>(idxB)]));
        double w = 1.0;
        if (k > start)
            w = 0.5 * (1.0 + std::cos(kPi * (k - start) / (centre - start)));
        const float v = static_cast<float>(h * w);
        taps[centre + k] = v;
        taps[centre - k] = v;
    }
    (void)length;
}

// ---------------------------------------------------------------------------------------------------------------
std::unique_ptr<KernelSet> KernelSetBuilder::build(int log2, int outputs, const float* const* taps, std::uint64_t generation)
{
    auto set = std::make_unique<KernelSet>();
    set->log2 = log2;
    set->block = convolverBlock(log2);
    set->bins = set->block + 1;
    set->partitions = (1 << log2) / set->block;
    set->outputs = std::clamp(outputs, 1, kMaxOutputs);
    set->pairStride = set->partitions * 2 * set->bins;
    set->generation = generation;
    set->active.assign(static_cast<size_t>(set->outputs) * 2, 0);
    set->data.assign(static_cast<size_t>(set->outputs) * 2 * static_cast<size_t>(set->pairStride), 0.0f);
    const int n = 2 * set->block;
    Fft& f = fftFor(fft, orderOf(n));
    work.assign(static_cast<size_t>(2) * static_cast<size_t>(n), 0.0f);
    const int length = kernelTaps(log2);
    for (int o = 0; o < set->outputs; ++o)
        for (int i = 0; i < 2; ++i)
        {
            const float* kernel = taps[o * 2 + i];
            if (kernel == nullptr)
                continue;
            set->active[static_cast<size_t>(o * 2 + i)] = 1;
            for (int p = 0; p < set->partitions; ++p)
            {
                std::fill(work.begin(), work.end(), 0.0f);
                const int from = p * set->block;
                const int count = std::max(0, std::min(set->block, length - from));
                if (count > 0)
                    std::memcpy(work.data(), kernel + from, static_cast<size_t>(count) * sizeof(float));
                f.forward(work.data());
                float* re = set->reMutable(o, i, p);
                float* im = re + set->bins;
                for (int b = 0; b < set->bins; ++b)
                {
                    re[b] = work[static_cast<size_t>(2 * b)];
                    im[b] = work[static_cast<size_t>(2 * b + 1)];
                }
            }
        }
    return set;
}

// ---------------------------------------------------------------------------------------------------------------
KernelMailbox::~KernelMailbox()
{
    delete pending.exchange(nullptr);
    for (auto& slot : retired)
        delete slot.exchange(nullptr);
}

void KernelMailbox::post(std::unique_ptr<KernelSet> set)
{
    delete pending.exchange(set.release(), std::memory_order_acq_rel);
}

void KernelMailbox::collect()
{
    for (auto& slot : retired)
        delete slot.exchange(nullptr, std::memory_order_acq_rel);
}

// ---------------------------------------------------------------------------------------------------------------
void PartitionedConvolver::prepare(int maxOutputs)
{
    outputCapacity = std::clamp(maxOutputs, 1, kMaxOutputs);
    for (int k = kMinKernelLog2; k <= kMaxKernelLog2; ++k)
        fftFor(fft, orderOf(2 * convolverBlock(k)));
    const int maxBlock = convolverBlock(kMaxKernelLog2);
    const int maxBins = maxBlock + 1;
    work.assign(static_cast<size_t>(4) * static_cast<size_t>(maxBlock), 0.0f);
    fadeBlock.assign(static_cast<size_t>(maxBlock), 0.0f);
    for (int c = 0; c < 2; ++c)
    {
        history[c].assign(static_cast<size_t>(maxBlock), 0.0f);
        collect[c].assign(static_cast<size_t>(maxBlock), 0.0f);
        ringRe[c].assign(static_cast<size_t>(kPartitions) * static_cast<size_t>(maxBins), 0.0f);
        ringIm[c].assign(static_cast<size_t>(kPartitions) * static_cast<size_t>(maxBins), 0.0f);
    }
    for (int o = 0; o < outputCapacity; ++o)
    {
        outBlock[o].assign(static_cast<size_t>(maxBlock), 0.0f);
        nextBlock[o].assign(static_cast<size_t>(maxBlock), 0.0f);
    }
    accRe.assign(static_cast<size_t>(maxBins), 0.0f);
    accIm.assign(static_cast<size_t>(maxBins), 0.0f);
    oldRe.assign(static_cast<size_t>(maxBins), 0.0f);
    oldIm.assign(static_cast<size_t>(maxBins), 0.0f);
    reset();
}

PartitionedConvolver::~PartitionedConvolver()
{
    delete current;
    delete previous;
    delete deferred;
}

void PartitionedConvolver::reset()
{
    for (int c = 0; c < 2; ++c)
    {
        std::fill(history[c].begin(), history[c].end(), 0.0f);
        std::fill(collect[c].begin(), collect[c].end(), 0.0f);
        std::fill(ringRe[c].begin(), ringRe[c].end(), 0.0f);
        std::fill(ringIm[c].begin(), ringIm[c].end(), 0.0f);
    }
    for (int o = 0; o < outputCapacity; ++o)
    {
        std::fill(outBlock[o].begin(), outBlock[o].end(), 0.0f);
        std::fill(nextBlock[o].begin(), nextBlock[o].end(), 0.0f);
    }
    positions = 0;
    ringHead = 0;
    total = 0;
}

void PartitionedConvolver::dropKernel()
{
    delete current;
    delete previous;
    delete deferred;
    current = nullptr;
    previous = nullptr;
    deferred = nullptr;
    fading = false;
    box.discard();
    blockSize = 256;
    reset();
}

void PartitionedConvolver::install(KernelSet* set) noexcept
{
    // A different length class: new block size, so the history is meaningless. The old set goes back to the worker (the
    // audio thread never frees memory); if its mailbox is full it is held and handed back at the next block. When two sets
    // are already held and the mailbox is still full, the new set waits (`deferred`) and the installed one keeps running;
    // nothing is dropped or freed here.
    if (previous != nullptr && box.retire(previous))
        previous = nullptr;
    if (current != nullptr)
    {
        if (box.retire(current))
            current = nullptr;
        else if (previous == nullptr)
        {
            previous = current;
            current = nullptr;
        }
        else
        {
            deferred = set;
            return;
        }
    }
    fading = false; // a crossfade in progress ends: its old set belongs to another block size or to a dropped session
    current = set;
    blockSize = set->block;
    positions = 0;
    ringHead = 0;
    for (int c = 0; c < 2; ++c)
    {
        std::fill(history[c].begin(), history[c].end(), 0.0f);
        std::fill(ringRe[c].begin(), ringRe[c].end(), 0.0f);
        std::fill(ringIm[c].begin(), ringIm[c].end(), 0.0f);
    }
    for (int o = 0; o < outputCapacity; ++o)
    {
        std::fill(outBlock[o].begin(), outBlock[o].end(), 0.0f);
        std::fill(nextBlock[o].begin(), nextBlock[o].end(), 0.0f);
    }
}

bool PartitionedConvolver::adopt(int log2) noexcept
{
    if (KernelSet* incoming = nextIncoming())
        install(incoming);
    return current != nullptr && current->log2 == log2;
}

void PartitionedConvolver::process(const float* left, const float* right, float* const* out, int numOut, int n) noexcept
{
    const float* in[2] = {left, right};
    // The first kernel is adopted at once, so that nothing the stream carries is dropped waiting for a block boundary.
    if (current == nullptr)
        if (KernelSet* first = nextIncoming())
            install(first);
    int done = 0;
    while (done < n)
    {
        const int chunk = std::min(n - done, blockSize - positions);
        for (int c = 0; c < 2; ++c)
            std::memcpy(collect[c].data() + positions, in[c] + done, static_cast<size_t>(chunk) * sizeof(float));
        for (int o = 0; o < numOut; ++o)
        {
            if (o < outputCapacity)
                std::memcpy(out[o] + done, outBlock[o].data() + positions, static_cast<size_t>(chunk) * sizeof(float));
            else
                std::memset(out[o] + done, 0, static_cast<size_t>(chunk) * sizeof(float));
        }
        positions += chunk;
        done += chunk;
        total += static_cast<std::uint64_t>(chunk);
        if (positions == blockSize)
        {
            runBlock();
            if (positions >= blockSize) // not reset by an install
                positions = 0;
        }
    }
}

void PartitionedConvolver::runBlock() noexcept
{
    // Hand back a set that could not be retired before.
    if (previous != nullptr && !fading && box.retire(previous))
        previous = nullptr;
    bool crossfade = false;
    if (previous == nullptr)
    {
        if (KernelSet* incoming = nextIncoming())
        {
            if (current != nullptr && incoming->log2 == current->log2 && incoming->outputs == current->outputs)
            {
                previous = current;
                current = incoming;
                fading = true;
                fadeIdx = 0;
                fadeTotal = std::max(1, (fadeSamples + incoming->block / 2) / incoming->block);
            }
            else
            {
                install(incoming);
                return; // restart on the new block size; this block's input is dropped
            }
        }
    }
    crossfade = fading;
    const int b = blockSize, bins = b + 1, n = 2 * b;
    if (current == nullptr)
    {
        for (int o = 0; o < outputCapacity; ++o)
            std::fill(outBlock[o].begin(), outBlock[o].begin() + b, 0.0f);
        return;
    }
    const int parts = current->partitions;
    Fft& f = fftFor(fft, orderOf(n));

    ringHead = (ringHead + 1) % parts;
    for (int c = 0; c < 2; ++c)
    {
        std::memcpy(work.data(), history[c].data(), static_cast<size_t>(b) * sizeof(float));
        std::memcpy(work.data() + b, collect[c].data(), static_cast<size_t>(b) * sizeof(float));
        std::memcpy(history[c].data(), collect[c].data(), static_cast<size_t>(b) * sizeof(float));
        f.forward(work.data());
        float* re = ringRe[c].data() + static_cast<size_t>(ringHead) * static_cast<size_t>(bins);
        float* im = ringIm[c].data() + static_cast<size_t>(ringHead) * static_cast<size_t>(bins);
        for (int k = 0; k < bins; ++k)
        {
            re[k] = work[static_cast<size_t>(2 * k)];
            im[k] = work[static_cast<size_t>(2 * k + 1)];
        }
    }

    const auto accumulate = [&](const KernelSet& set, int o, std::vector<float>& ar, std::vector<float>& ai)
    {
        std::fill(ar.begin(), ar.begin() + bins, 0.0f);
        std::fill(ai.begin(), ai.begin() + bins, 0.0f);
        bool any = false;
        for (int i = 0; i < 2; ++i)
        {
            if (!set.isActive(o, i))
                continue;
            any = true;
            for (int p = 0; p < parts; ++p)
            {
                const int slot = (ringHead - p + parts) % parts;
                macPlanar(ar.data(), ai.data(), ringRe[i].data() + static_cast<size_t>(slot) * static_cast<size_t>(bins),
                          ringIm[i].data() + static_cast<size_t>(slot) * static_cast<size_t>(bins), set.re(o, i, p),
                          set.im(o, i, p), bins);
            }
        }
        return any;
    };
    const auto toTime = [&](const std::vector<float>& ar, const std::vector<float>& ai, float* out)
    {
        for (int k = 0; k < bins; ++k)
        {
            work[static_cast<size_t>(2 * k)] = ar[static_cast<size_t>(k)];
            work[static_cast<size_t>(2 * k + 1)] = ai[static_cast<size_t>(k)];
        }
        f.inverse(work.data());
        std::memcpy(out, work.data() + b, static_cast<size_t>(b) * sizeof(float));
    };

    for (int o = 0; o < outputCapacity; ++o)
    {
        float* out = nextBlock[o].data();
        if (o >= current->outputs)
        {
            std::fill(out, out + b, 0.0f);
            continue;
        }
        if (accumulate(*current, o, accRe, accIm))
            toTime(accRe, accIm, out);
        else
            std::fill(out, out + b, 0.0f);
        if (crossfade)
        {
            // The old kernel's block, blended out with a raised-cosine ramp while the new one blends in.
            const bool hasOld = accumulate(*previous, o, oldRe, oldIm);
            if (hasOld)
                toTime(oldRe, oldIm, fadeBlock.data());
            for (int k = 0; k < b; ++k)
            {
                const float w = 0.5f - 0.5f * std::cos(3.14159265f * (static_cast<float>(fadeIdx * b + k) + 0.5f) /
                                                       static_cast<float>(fadeTotal * b));
                out[k] = (hasOld ? fadeBlock[static_cast<size_t>(k)] * (1.0f - w) : 0.0f) + out[k] * w;
            }
        }
    }
    for (int o = 0; o < outputCapacity; ++o)
        std::swap(outBlock[o], nextBlock[o]);
    if (fading && ++fadeIdx >= fadeTotal)
        fading = false;
    if (previous != nullptr && !fading && box.retire(previous))
        previous = nullptr;
}

// ---------------------------------------------------------------------------------------------------------------
LinearPhaseWorker& LinearPhaseWorker::instance()
{
    static LinearPhaseWorker worker;
    return worker;
}

LinearPhaseWorker::~LinearPhaseWorker()
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
    }
    condition.notify_all();
    if (thread.joinable())
        thread.join();
}

void LinearPhaseWorker::startLocked()
{
    if (threadless())
        return; // serviced by pump() on the control thread
    if (!thread.joinable())
        thread = std::thread([this] { run(); });
}

void LinearPhaseWorker::add(Client* client)
{
    {
        std::lock_guard<std::mutex> lock(mutex);
        clients.push_back(client);
        startLocked();
    }
    condition.notify_all();
}

void LinearPhaseWorker::remove(Client* client)
{
    std::lock_guard<std::mutex> lock(mutex);
    clients.erase(std::remove(clients.begin(), clients.end(), client), clients.end());
}

bool LinearPhaseWorker::servicePass()
{
    if (!tools)
        tools = std::make_unique<Tools>();
    bool worked = false;
    for (size_t i = 0; i < clients.size(); ++i)
        worked = clients[i]->service(*tools) || worked;
    return worked;
}

void LinearPhaseWorker::pump()
{
    if (!threadless())
        return;
    std::lock_guard<std::mutex> lock(mutex);
    servicePass();
}

void LinearPhaseWorker::wake()
{
    if (threadless())
    {
        pump();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex);
        poked = true;
    }
    condition.notify_all();
}

void LinearPhaseWorker::run()
{
    std::unique_lock<std::mutex> lock(mutex);
    int idle = 0; // polls since the last piece of work
    while (!stopping)
    {
        if (clients.empty())
        {
            condition.wait(lock, [this] { return stopping || !clients.empty(); });
            continue;
        }
        const bool worked = servicePass();
        poked = false;
        idle = worked ? 0 : std::min(idle + 1, 1000);
        // Quick polling right after work (a parameter is being moved), slow when nothing has happened for a second.
        const int ms = idle < 250 ? 4 : 40;
        condition.wait_for(lock, std::chrono::milliseconds(ms), [this] { return stopping || poked; });
    }
}

} // namespace crucible::fx
