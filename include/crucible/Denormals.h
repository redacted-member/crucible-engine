// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#if defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
#include <xmmintrin.h>
#endif

namespace crucible
{

// Flushes denormal floats to zero on the current thread for the lifetime of the object, then restores the
// previous mode. Recursive DSP (filters, reverbs) can otherwise slow down badly as signals decay.
class ScopedFlushDenormals
{
public:
    ScopedFlushDenormals() noexcept
    {
#if defined(__aarch64__)
        asm volatile("mrs %0, fpcr" : "=r"(saved));
        const std::uint64_t flushed = saved | (1ULL << 24);
        asm volatile("msr fpcr, %0" : : "r"(flushed));
#elif defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
        saved = _mm_getcsr();
        _mm_setcsr(static_cast<unsigned>(saved) | 0x8040U); // FTZ | DAZ
#endif
    }

    ~ScopedFlushDenormals()
    {
#if defined(__aarch64__)
        asm volatile("msr fpcr, %0" : : "r"(saved));
#elif defined(__SSE__) || defined(_M_X64) || defined(_M_IX86)
        _mm_setcsr(static_cast<unsigned>(saved));
#endif
    }

    ScopedFlushDenormals(const ScopedFlushDenormals&) = delete;
    ScopedFlushDenormals& operator=(const ScopedFlushDenormals&) = delete;

private:
    std::uint64_t saved = 0;
};

} // namespace crucible
