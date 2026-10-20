// SPDX-License-Identifier: MIT
#pragma once

// Linear-phase filtering by FFT convolution (docs/plans/ENGINE-FX-PLAN.md, sections 3.3 and 4.2): a symmetric FIR built
// from a target magnitude, executed as uniformly partitioned convolution, with the kernel redesigned on a worker thread
// and handed to the audio thread without locks or allocation.
//
//   LinearPhaseDesigner   target magnitude on a uniform grid -> symmetric float taps (frequency sampling on a 4x finer
//                         grid than the kernel, so the wrap-around of the inverse transform stays below the taper;
//                         the ends are tapered by a raised cosine)
//   KernelSetBuilder      taps -> the spectra of the partitions (what the convolver multiplies by)
//   PartitionedConvolver  stereo-input matrix convolution (out_o = sum_i kernel(o, i) * in_i for up to 16 outputs),
//                         overlap-save with block size B and up to 16 partitions; kernel changes crossfade over one block
//   KernelMailbox         worker -> audio hand-over of finished kernel sets and audio -> worker return of the old ones;
//                         the audio thread never allocates, frees or waits
//   LinearPhaseWorker     one background thread per process that calls Client::service() of every registered client
//                         every few milliseconds
//
// A kernel of 2^k - 1 taps (k = 10..15) is symmetric about tap c = 2^(k-1) - 1, so a signal is delayed by exactly c
// samples for every frequency; the convolver adds its block size B (256 for k <= 12, then 2^(k-4)). The latency a module
// reports is therefore latencySamples(k) = c + B.
//
// Only the audio side of these classes is real-time safe; design, building and construction run on the control or worker
// thread.

#include <crucible/Fft.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace crucible::fx
{

constexpr int kMinKernelLog2 = 10;
constexpr int kMaxKernelLog2 = 15;
constexpr int kPartitions = 16;
constexpr int kMaxOutputs = 16;

constexpr int kernelTaps(int log2)
{
    return (1 << log2) - 1;
}
constexpr int kernelCentre(int log2)
{
    return (1 << (log2 - 1)) - 1;
}
constexpr int convolverBlock(int log2)
{
    return log2 <= 12 ? 256 : (1 << (log2 - 4));
}
constexpr int linearPhaseLatency(int log2)
{
    return kernelCentre(log2) + convolverBlock(log2);
}

// ---------------------------------------------------------------------------------------------------------------
class LinearPhaseDesigner
{
public:
    // Control or worker thread: allocates for kernels up to 2^maxLog2 - 1 taps.
    void prepare(int maxLog2 = kMaxKernelLog2);

    // Number of grid points (0 .. pi inclusive) the magnitude must be given at for a kernel of 2^log2 - 1 taps.
    static int gridPoints(int log2) { return (4 << log2) / 2 + 1; }
    // Angular frequency of grid point j.
    static double gridOmega(int log2, int j);

    // magnitude[j] >= 0 at gridOmega(log2, j), j < gridPoints(log2). taps receives kernelTaps(log2) floats, exactly
    // symmetric. The grid is not touched.
    void design(int log2, const float* magnitude, float* taps);

private:
    std::vector<std::unique_ptr<Fft>> fft; // index = order
    std::vector<float> work;
};

// ---------------------------------------------------------------------------------------------------------------
// The spectra of the partitions of the kernels of a convolution matrix: output o (0 .. outputs-1) from input i (0 left,
// 1 right).
struct KernelSet
{
    // Diagnostic: how many sets exist (tests check that a hand-over path frees every one).
    static std::atomic<int>& liveCount() noexcept
    {
        static std::atomic<int> n{0};
        return n;
    }
    KernelSet() noexcept { liveCount().fetch_add(1, std::memory_order_relaxed); }
    KernelSet(const KernelSet&) = delete;
    KernelSet& operator=(const KernelSet&) = delete;
    ~KernelSet() { liveCount().fetch_sub(1, std::memory_order_relaxed); }

    int log2 = 0;
    int block = 0;
    int bins = 0; // block + 1
    int partitions = 0;
    int outputs = 2;
    std::vector<char> active; // outputs * 2: pair (o, i) holds a kernel
    int pairStride = 0;       // floats between pairs
    std::vector<float> data;  // planar: pair (o * 2 + i), partition p: re[bins], im[bins]
    std::uint64_t generation = 0;

    bool isActive(int o, int i) const { return active[static_cast<size_t>(o * 2 + i)] != 0; }
    const float* re(int o, int i, int p) const
    {
        return data.data() + static_cast<size_t>(o * 2 + i) * static_cast<size_t>(pairStride) +
               static_cast<size_t>(p) * 2u * static_cast<size_t>(bins);
    }
    const float* im(int o, int i, int p) const { return re(o, i, p) + bins; }
    float* reMutable(int o, int i, int p) { return const_cast<float*>(re(o, i, p)); }
};

class KernelSetBuilder
{
public:
    // taps[o * 2 + i]: kernelTaps(log2) floats or null (pair inactive); outputs <= kMaxOutputs.
    std::unique_ptr<KernelSet> build(int log2, int outputs, const float* const* taps, std::uint64_t generation);

private:
    std::vector<std::unique_ptr<Fft>> fft; // index = order
    std::vector<float> work;
};

// ---------------------------------------------------------------------------------------------------------------
class KernelMailbox
{
public:
    static constexpr int kRetired = 4;
    ~KernelMailbox();

    // Worker thread: replaces a set the audio thread has not taken yet.
    void post(std::unique_ptr<KernelSet> set);
    // Worker or control thread: frees what the audio thread has handed back.
    void collect();
    // Control thread, audio stopped: frees the set that is waiting, if any.
    void discard()
    {
        delete pending.exchange(nullptr, std::memory_order_acq_rel);
        collect();
    }
    // Audio thread: the newest posted set (ownership passes to the caller) or null.
    KernelSet* take() noexcept { return pending.exchange(nullptr, std::memory_order_acq_rel); }
    // Audio thread: hands a finished set back; false when all slots are occupied (try again next block).
    bool retire(KernelSet* set) noexcept
    {
        for (auto& slot : retired)
        {
            KernelSet* expected = nullptr;
            if (slot.compare_exchange_strong(expected, set, std::memory_order_acq_rel))
                return true;
        }
        return false;
    }

private:
    std::atomic<KernelSet*> pending{nullptr};
    std::atomic<KernelSet*> retired[kRetired] = {};
};

// ---------------------------------------------------------------------------------------------------------------
// Newest-request hand-over from an audio thread to the worker (one slot, the worker takes only the latest). The slot is
// guarded by a flag that the worker also takes on every poll, so a real-time post must not wait: tryPost() fails when it
// meets the worker. An offline render must not lose a request (its result would depend on thread timing): post() spins.
template <class T> class LatestRequestSlot
{
public:
    bool tryPost(const T& r)
    {
        if (busy.test_and_set(std::memory_order_acquire))
            return false;
        store(r);
        return true;
    }
    // Not for the real-time path: waits for the worker's brief hold on the flag.
    void post(const T& r)
    {
        while (busy.test_and_set(std::memory_order_acquire))
            std::this_thread::yield();
        store(r);
    }
    bool fetch(T& out, std::uint64_t& seen)
    {
        while (busy.test_and_set(std::memory_order_acquire))
            std::this_thread::yield();
        const bool fresh = version.load(std::memory_order_acquire) != seen;
        if (fresh)
        {
            out = data;
            seen = version.load(std::memory_order_acquire);
        }
        busy.clear(std::memory_order_release);
        return fresh;
    }
    std::uint64_t posted() const { return version.load(std::memory_order_acquire); }

private:
    void store(const T& r)
    {
        data = r;
        version.fetch_add(1, std::memory_order_release);
        busy.clear(std::memory_order_release);
    }

    std::atomic_flag busy; // clear on construction (C++20)
    T data{};
    std::atomic<std::uint64_t> version{0};
};

// ---------------------------------------------------------------------------------------------------------------
// Stereo 2x2 partitioned convolver. A set for the length class it runs is required; until one arrives it outputs silence.
class PartitionedConvolver
{
public:
    ~PartitionedConvolver();
    // Control thread. Allocates for the largest length class and for up to maxOutputs outputs.
    void prepare(int maxOutputs = kMaxOutputs);
    void reset();
    // Control thread, audio stopped: forgets the installed kernel and the one waiting (they belong to the settings or sample
    // rate before a prepare); until a new set arrives the convolver is silent.
    void dropKernel();

    KernelMailbox& mailbox() { return box; }
    // Control thread: length in samples of the crossfade between two kernels of the same length class (default one block).
    void setFadeSamples(int samples) { fadeSamples = samples; }
    // True once a kernel set is installed and the input history has had one full block + kernel to fill.
    bool hasKernel() const { return current != nullptr; }
    // Length class of the installed set (0 before the first).
    int lengthLog2() const { return current != nullptr ? current->log2 : 0; }
    // Whole samples from input to output (convolver block only: the kernel's own delay is added by the caller).
    int blockLatency() const { return blockSize; }
    // Samples processed since the last reset (priming).
    std::uint64_t processed() const { return total; }

    // Reads two input channels and writes the outputs of the installed set (out[o], o < outputs(); outputs beyond the set
    // are filled with silence); any n. An output may alias an input.
    void process(const float* left, const float* right, float* const* out, int numOut, int n) noexcept;
    // In place on two channels (a two-output set).
    void process(float* left, float* right, int n) noexcept
    {
        float* io[2] = {left, right};
        process(left, right, io, 2, n);
    }
    int outputs() const { return current != nullptr ? current->outputs : 0; }
    // Audio thread, while the convolver is not carrying a signal (before a path switch): takes a posted set if one is
    // waiting and installs it, so that a kernel can be adopted without running process(). True when the installed set has
    // length class log2.
    bool adopt(int log2) noexcept;

private:
    void runBlock() noexcept;
    void install(KernelSet* set) noexcept;
    // The set to install next: a deferred one first, else the newest posted (null when none).
    KernelSet* nextIncoming() noexcept
    {
        if (KernelSet* d = deferred)
        {
            deferred = nullptr;
            return d;
        }
        return box.take();
    }

    KernelMailbox box;
    KernelSet* current = nullptr;
    KernelSet* previous = nullptr;
    KernelSet* deferred = nullptr; // taken from the mailbox but not installed: no slot to hold the installed set
    // Kernel change crossfade: whole blocks (at least one) nearest to fadeSamples; the old set is kept until it ends.
    int fadeSamples = 0, fadeIdx = 0, fadeTotal = 1;
    bool fading = false;
    int blockSize = 256, positions = 0;
    std::uint64_t total = 0;

    std::vector<std::unique_ptr<Fft>> fft; // index = order of 2 * block
    std::vector<float> work;
    // Per input channel: the previous block, the block being collected, the spectra ring.
    std::vector<float> history[2], collect[2], ringRe[2], ringIm[2];
    // Per output channel: the block being read, the next block.
    std::vector<float> outBlock[kMaxOutputs], nextBlock[kMaxOutputs];
    std::vector<float> accRe, accIm, oldRe, oldIm, fadeBlock;
    int ringHead = 0;
    int outputCapacity = kMaxOutputs;
};

// ---------------------------------------------------------------------------------------------------------------
class LinearPhaseWorker
{
public:
    // Design tools shared by all clients (allocated on first use by the worker thread) and scratch memory for them.
    struct Tools
    {
        LinearPhaseDesigner designer;
        KernelSetBuilder builder;
        std::vector<float> scratch[8];
        bool ready = false;
        // Allocates the transforms on first use.
        void ensure()
        {
            if (!ready)
            {
                designer.prepare(kMaxKernelLog2);
                ready = true;
            }
        }
    };

    class Client
    {
    public:
        virtual ~Client() = default;
        // Worker thread, every poll: do any pending design work. Returns true when there was work (the worker polls faster
        // for a while after work).
        virtual bool service(Tools& tools) = 0;
    };

    static LinearPhaseWorker& instance();
    // Control thread. remove() returns once service() is not running on the client.
    void add(Client* client);
    void remove(Client* client);
    // Wakes the worker at once (used by offline rendering to wait for a design). Where threadless() is true this pumps, so
    // it must only be called on the control thread, never from process().
    void wake();
    // Control thread. Where background threads are unavailable (WebAssembly without pthreads) the clients are serviced here,
    // one pass per call, on the calling thread; everywhere else this does nothing (the worker thread does the work).
    void pump();
    static constexpr bool threadless()
    {
#if defined(__EMSCRIPTEN__) && !defined(__EMSCRIPTEN_PTHREADS__)
        return true;
#else
        return false;
#endif
    }

    LinearPhaseWorker(const LinearPhaseWorker&) = delete;
    LinearPhaseWorker& operator=(const LinearPhaseWorker&) = delete;

private:
    LinearPhaseWorker() = default;
    ~LinearPhaseWorker();
    void run();
    bool servicePass();
    void startLocked();
    std::unique_ptr<Tools> tools;

    std::mutex mutex;
    std::condition_variable condition;
    std::vector<Client*> clients;
    std::thread thread;
    bool stopping = false;
    bool poked = false;
};

} // namespace crucible::fx
