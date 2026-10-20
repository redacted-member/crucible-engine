// SPDX-License-Identifier: MIT
#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace crucible
{

// Lock-free single-producer / single-consumer ring of N slots holding at most N - 1 items.
// push() is called from exactly one thread and pop() from exactly one (possibly different) thread.
// Neither allocates nor blocks, so either side may be the audio thread.
template <typename T, std::size_t N> class SpscRing
{
    static_assert(N >= 2, "ring needs at least two slots");

public:
    bool push(const T& item) noexcept
    {
        const auto w = writePos.load(std::memory_order_relaxed);
        const auto next = (w + 1) % N;
        if (next == readPos.load(std::memory_order_acquire))
            return false; // full
        slots[w] = item;
        writePos.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& out) noexcept
    {
        const auto r = readPos.load(std::memory_order_relaxed);
        if (r == writePos.load(std::memory_order_acquire))
            return false; // empty
        out = slots[r];
        readPos.store((r + 1) % N, std::memory_order_release);
        return true;
    }

    // Discards everything. Only safe while neither side is running.
    void reset() noexcept
    {
        readPos.store(0, std::memory_order_relaxed);
        writePos.store(0, std::memory_order_relaxed);
    }

    std::size_t size() const noexcept
    {
        const auto w = writePos.load(std::memory_order_acquire);
        const auto r = readPos.load(std::memory_order_acquire);
        return (w + N - r) % N;
    }

    static constexpr std::size_t capacity() noexcept { return N - 1; }

private:
    std::array<T, N> slots{};
    std::atomic<std::size_t> readPos{0};
    std::atomic<std::size_t> writePos{0};
};

} // namespace crucible
