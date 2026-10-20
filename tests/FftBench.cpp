// SPDX-License-Identifier: MIT
// FFT backend benchmark (P13): the portable double-precision radix-2 reference, pffft and the backend behind
// Fft::forward() (Accelerate on Apple), per size. Prints a Markdown table. With --check the exit status is 1 unless pffft is
// at least 1.5 times faster than the portable code at every size from 64 up: the adoption rule of the plan (pffft is the
// backend of Fft.h on platforms without Accelerate only if it passes). Timing is meaningless in debug and sanitizer builds.
//
//   crucible_engine_fft_bench [--check] [--seconds S]

#include <crucible/Fft.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

namespace
{
template <typename F> double microseconds(F&& f, double budgetSeconds)
{
    using clock = std::chrono::steady_clock;
    // Calibrate the repetition count, then take the best of five timed batches.
    int reps = 1;
    for (;;)
    {
        const auto t0 = clock::now();
        for (int i = 0; i < reps; ++i)
            f();
        if (std::chrono::duration<double>(clock::now() - t0).count() > budgetSeconds / 10.0 || reps > (1 << 20))
            break;
        reps *= 2;
    }
    double best = 1.0e30;
    for (int batch = 0; batch < 5; ++batch)
    {
        const auto t0 = clock::now();
        for (int i = 0; i < reps; ++i)
            f();
        best = std::min(best, std::chrono::duration<double>(clock::now() - t0).count() / reps);
    }
    return best * 1.0e6;
}
} // namespace

int main(int argc, char** argv)
{
    bool check = false;
    double seconds = 0.2;
    for (int i = 1; i < argc; ++i)
    {
        if (std::strcmp(argv[i], "--check") == 0)
            check = true;
        else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc)
            seconds = std::atof(argv[++i]);
    }
    std::printf("| size | portable (us) | pffft (us) | forward() = %s (us) | pffft vs portable | pffft vs forward() |\n",
                crucible::Fft::backendName());
    std::printf("| ---: | ---: | ---: | ---: | ---: | ---: |\n");
    std::mt19937 rng(1);
    std::uniform_real_distribution<float> uniform(-1.0f, 1.0f);
    bool passed = true;
    for (const int order : {6, 8, 10, 11, 12, 13, 15})
    {
        const int n = 1 << order;
        crucible::Fft fft(order);
        std::vector<float> input(static_cast<size_t>(n)), data(static_cast<size_t>(2 * n));
        for (auto& v : input)
            v = uniform(rng);
        const auto run = [&](auto&& transform)
        {
            return microseconds(
                [&]
                {
                    std::copy(input.begin(), input.end(), data.begin());
                    transform();
                },
                seconds);
        };
        const double portable = run([&] { fft.forwardPortable(data.data()); });
        const double pffft = run([&] { fft.forwardPffft(data.data()); });
        const double backend = run([&] { fft.forward(data.data()); });
        std::printf("| %d | %.2f | %.2f | %.2f | %.1fx | %.2fx |\n", n, portable, pffft, backend, portable / pffft,
                    backend / pffft);
        if (portable / pffft < 1.5)
            passed = false;
    }
    std::printf("\npffft is %s the 1.5x adoption rule against the portable code.\n", passed ? "meeting" : "NOT meeting");
    return check && !passed ? 1 : 0;
}
