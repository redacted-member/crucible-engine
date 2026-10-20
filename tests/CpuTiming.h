// SPDX-License-Identifier: MIT
#pragma once

#include <ctime>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#if defined(_M_X64) || defined(_M_IX86)
#include <intrin.h>
#endif
#endif

#if defined(__APPLE__)
#include <pthread.h>
#include <pthread/qos.h>
#endif

namespace crucible::testing
{

// CPU seconds consumed by the calling thread. Unlike a wall clock it does not grow while the thread is preempted, so
// the CPU checks hold on a busy machine.
#if defined(_WIN32) && (defined(_M_X64) || defined(_M_IX86))
// Time-stamp counter ticks per second, measured once against QueryPerformanceCounter (20 ms).
inline double tscTicksPerSecond()
{
    static const double rate = []
    {
        LARGE_INTEGER f{}, q0{}, q1{};
        QueryPerformanceFrequency(&f);
        QueryPerformanceCounter(&q0);
        const unsigned long long c0 = __rdtsc();
        do
            QueryPerformanceCounter(&q1);
        while (q1.QuadPart - q0.QuadPart < f.QuadPart / 50);
        const unsigned long long c1 = __rdtsc();
        return static_cast<double>(c1 - c0) * static_cast<double>(f.QuadPart) /
               static_cast<double>(q1.QuadPart - q0.QuadPart);
    }();
    return rate;
}
#endif

inline double threadCpuSeconds()
{
#if defined(_WIN32) && (defined(_M_X64) || defined(_M_IX86))
    // GetThreadTimes only advances in whole scheduler ticks (15.625 ms), coarser than most of the measured intervals;
    // the thread's cycle count (time-stamp counter ticks while the thread runs) is exact.
    ULONG64 cycles = 0;
    QueryThreadCycleTime(GetCurrentThread(), &cycles);
    return static_cast<double>(cycles) / tscTicksPerSecond();
#elif defined(_WIN32)
    FILETIME creation{}, exit{}, kernel{}, user{};
    GetThreadTimes(GetCurrentThread(), &creation, &exit, &kernel, &user);
    const auto ticks = [](const FILETIME& f)
    { return static_cast<double>((static_cast<unsigned long long>(f.dwHighDateTime) << 32) | f.dwLowDateTime); };
    return (ticks(kernel) + ticks(user)) * 1.0e-7; // 100 ns units
#else
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + 1.0e-9 * static_cast<double>(ts.tv_nsec);
#endif
}

// Asks macOS to keep the calling thread on a performance core for the lifetime of the object. A no-op elsewhere.
class ScopedUserInteractiveQos
{
public:
    ScopedUserInteractiveQos()
    {
#if defined(__APPLE__)
        qos_class_t cls = QOS_CLASS_DEFAULT;
        int rel = 0;
        pthread_get_qos_class_np(pthread_self(), &cls, &rel);
        previous = cls;
        pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
#endif
    }
    ~ScopedUserInteractiveQos()
    {
#if defined(__APPLE__)
        pthread_set_qos_class_self_np(previous, 0);
#endif
    }
    ScopedUserInteractiveQos(const ScopedUserInteractiveQos&) = delete;
    ScopedUserInteractiveQos& operator=(const ScopedUserInteractiveQos&) = delete;

private:
#if defined(__APPLE__)
    qos_class_t previous = QOS_CLASS_DEFAULT;
#endif
};

} // namespace crucible::testing
