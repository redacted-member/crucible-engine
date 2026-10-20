// SPDX-License-Identifier: MIT
#pragma once

#include <memory>

namespace crucible
{

// Real-input FFT of size 2^order, in place, with the conventional layout used throughout the engine:
//
// forward(): data holds 2 * size floats; the first size floats are the real input. On return data holds size
//            interleaved complex bins (re, im), X[k] = sum x[n] e^(-2 pi i k n / size), unscaled.
// inverse(): data holds interleaved complex bins 0..size/2 (the rest is ignored and treated as the conjugate
//            mirror). On return the first size floats are the real signal, scaled by 1 / size, so that
//            inverse(forward(x)) == x.
//
// Not real-time safe to construct (it allocates); forward() and inverse() do not allocate. An instance is not thread safe
// (the transforms use per-instance scratch memory); give every thread its own.
// Backends: on Apple platforms the Accelerate framework, elsewhere pffft (vendored, BSD-like licence, SIMD), unless the
// build selects another with CRUCIBLE_ENGINE_FFT_BACKEND (ACCELERATE, PFFFT or PORTABLE: a double-precision radix-2
// implementation of our own, always available as the reference). Sizes below 32 use the portable code with every backend.
class Fft
{
public:
    explicit Fft(int order);
    ~Fft();
    Fft(const Fft&) = delete;
    Fft& operator=(const Fft&) = delete;

    int size() const noexcept { return n; }
    void forward(float* data) const noexcept;
    void inverse(float* data) const noexcept;

    // The portable implementation, always available (the reference in tests, and the backend of tiny sizes).
    void forwardPortable(float* data) const noexcept;
    void inversePortable(float* data) const noexcept;
    // The pffft implementation, always compiled (forward() and inverse() use it on non-Apple platforms). Same layout and
    // scaling as forward() / inverse(); agrees with the portable one to about 1e-7 of the largest bin.
    void forwardPffft(float* data) const noexcept;
    void inversePffft(float* data) const noexcept;
    // Name of the backend behind forward() / inverse(): "accelerate", "pffft" or "portable".
    static const char* backendName() noexcept;

private:
    struct Impl;
    int order = 0;
    int n = 1;
    std::unique_ptr<Impl> impl;
};

} // namespace crucible
