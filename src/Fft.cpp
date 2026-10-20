// SPDX-License-Identifier: MIT
#include <crucible/Fft.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

#include <pffft.h>

// Backend of forward() / inverse(): Accelerate on Apple, pffft elsewhere, the portable code when CRUCIBLE_FFT_PORTABLE is
// defined; CRUCIBLE_FFT_PFFFT picks pffft on Apple too.
#if defined(__APPLE__) && !defined(CRUCIBLE_FFT_PORTABLE) && !defined(CRUCIBLE_FFT_PFFFT)
#include <Accelerate/Accelerate.h>
#define CRUCIBLE_FFT_ACCELERATE 1
#elif !defined(CRUCIBLE_FFT_PORTABLE)
#define CRUCIBLE_FFT_USE_PFFFT 1
#endif

namespace crucible
{

struct Fft::Impl
{
    std::vector<std::complex<double>> twiddles; // e^(-2 pi i k / n), k < n / 2
    std::vector<int> bitReverse;
    mutable std::vector<std::complex<double>> scratch;
#if CRUCIBLE_FFT_ACCELERATE
    FFTSetup setup = nullptr;
#endif
    // pffft: setup and 16-byte aligned buffers (null for sizes pffft does not support).
    PFFFT_Setup* pf = nullptr;
    float* pfIn = nullptr;
    float* pfOut = nullptr;
    float* pfWork = nullptr;

    void transform(bool inverse) const noexcept
    {
        const auto count = scratch.size();
        for (size_t i = 0; i < count; ++i)
        {
            const auto j = static_cast<size_t>(bitReverse[i]);
            if (j > i)
                std::swap(scratch[i], scratch[j]);
        }
        for (size_t len = 2; len <= count; len <<= 1)
        {
            const size_t half = len / 2;
            const size_t stride = count / len;
            for (size_t start = 0; start < count; start += len)
                for (size_t k = 0; k < half; ++k)
                {
                    auto w = twiddles[k * stride];
                    if (inverse)
                        w = std::conj(w);
                    const auto a = scratch[start + k];
                    const auto b = scratch[start + k + half] * w;
                    scratch[start + k] = a + b;
                    scratch[start + k + half] = a - b;
                }
        }
    }
};

Fft::Fft(int o) : order(o), n(1 << o), impl(std::make_unique<Impl>())
{
    const double pi = 3.14159265358979323846;
    impl->twiddles.resize(static_cast<size_t>(std::max(n / 2, 1)));
    for (int k = 0; k < n / 2; ++k)
        impl->twiddles[static_cast<size_t>(k)] = std::polar(1.0, -2.0 * pi * k / n);
    impl->bitReverse.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i)
    {
        int r = 0;
        for (int b = 0; b < order; ++b)
            if ((i >> b) & 1)
                r |= 1 << (order - 1 - b);
        impl->bitReverse[static_cast<size_t>(i)] = r;
    }
    impl->scratch.resize(static_cast<size_t>(n));
#if CRUCIBLE_FFT_ACCELERATE
    impl->setup = vDSP_create_fftsetup(static_cast<vDSP_Length>(order), 2);
#endif
    if (n >= 32)
    {
        impl->pf = pffft_new_setup(n, PFFFT_REAL);
        if (impl->pf != nullptr)
        {
            const auto bytes = sizeof(float) * static_cast<size_t>(n);
            impl->pfIn = static_cast<float*>(pffft_aligned_malloc(bytes));
            impl->pfOut = static_cast<float*>(pffft_aligned_malloc(bytes));
            impl->pfWork = static_cast<float*>(pffft_aligned_malloc(bytes));
            if (impl->pfIn == nullptr || impl->pfOut == nullptr || impl->pfWork == nullptr)
            {
                pffft_aligned_free(impl->pfIn);
                pffft_aligned_free(impl->pfOut);
                pffft_aligned_free(impl->pfWork);
                pffft_destroy_setup(impl->pf);
                impl->pf = nullptr;
                impl->pfIn = impl->pfOut = impl->pfWork = nullptr;
            }
        }
    }
}

Fft::~Fft()
{
#if CRUCIBLE_FFT_ACCELERATE
    if (impl->setup != nullptr)
        vDSP_destroy_fftsetup(impl->setup);
#endif
    if (impl->pf != nullptr)
    {
        pffft_aligned_free(impl->pfIn);
        pffft_aligned_free(impl->pfOut);
        pffft_aligned_free(impl->pfWork);
        pffft_destroy_setup(impl->pf);
    }
}

const char* Fft::backendName() noexcept
{
#if CRUCIBLE_FFT_ACCELERATE
    return "accelerate";
#elif CRUCIBLE_FFT_USE_PFFFT
    return "pffft";
#else
    return "portable";
#endif
}

void Fft::forwardPffft(float* data) const noexcept
{
    if (impl->pf == nullptr)
    {
        forwardPortable(data);
        return;
    }
    std::copy(data, data + n, impl->pfIn);
    pffft_transform_ordered(impl->pf, impl->pfIn, impl->pfOut, impl->pfWork, PFFFT_FORWARD);
    // pffft: [re0, re(n/2), re1, im1, ...]; the engine layout is n interleaved bins with the conjugate mirror.
    const float* z = impl->pfOut;
    const int half = n >> 1;
    data[0] = z[0];
    data[1] = 0.0f;
    data[2 * half] = z[1];
    data[2 * half + 1] = 0.0f;
    for (int k = 1; k < half; ++k)
    {
        data[2 * k] = z[2 * k];
        data[2 * k + 1] = z[2 * k + 1];
        data[2 * (n - k)] = z[2 * k];
        data[2 * (n - k) + 1] = -z[2 * k + 1];
    }
}

void Fft::inversePffft(float* data) const noexcept
{
    if (impl->pf == nullptr)
    {
        inversePortable(data);
        return;
    }
    float* z = impl->pfIn;
    const int half = n >> 1;
    z[0] = data[0];
    z[1] = data[2 * half];
    for (int k = 1; k < half; ++k)
    {
        z[2 * k] = data[2 * k];
        z[2 * k + 1] = data[2 * k + 1];
    }
    pffft_transform_ordered(impl->pf, z, impl->pfOut, impl->pfWork, PFFFT_BACKWARD);
    const float scale = 1.0f / static_cast<float>(n);
    for (int i = 0; i < n; ++i)
        data[i] = impl->pfOut[i] * scale;
    for (int i = n; i < 2 * n; ++i)
        data[i] = 0.0f;
}

void Fft::forwardPortable(float* data) const noexcept
{
    auto& s = impl->scratch;
    for (int i = 0; i < n; ++i)
        s[static_cast<size_t>(i)] = {data[i], 0.0};
    impl->transform(false);
    for (int i = 0; i < n; ++i)
    {
        data[2 * i] = static_cast<float>(s[static_cast<size_t>(i)].real());
        data[2 * i + 1] = static_cast<float>(s[static_cast<size_t>(i)].imag());
    }
}

void Fft::inversePortable(float* data) const noexcept
{
    auto& s = impl->scratch;
    for (int i = 0; i <= n / 2; ++i)
        s[static_cast<size_t>(i)] = {data[2 * i], data[2 * i + 1]};
    for (int i = n / 2 + 1; i < n; ++i)
        s[static_cast<size_t>(i)] = std::conj(s[static_cast<size_t>(n - i)]);
    impl->transform(true);
    const double scale = 1.0 / n;
    for (int i = 0; i < n; ++i)
        data[i] = static_cast<float>(s[static_cast<size_t>(i)].real() * scale);
    for (int i = n; i < 2 * n; ++i)
        data[i] = 0.0f;
}

#if CRUCIBLE_FFT_ACCELERATE
namespace
{
DSPSplitComplex split(float* data)
{
    return {data, data + 1};
}
} // namespace

void Fft::forward(float* data) const noexcept
{
    // Packed real FFT (stride 2 over interleaved storage), then unpack DC / Nyquist and mirror.
    auto sp = split(data);
    data[n] = 0.0f;
    vDSP_fft_zrip(impl->setup, &sp, 2, static_cast<vDSP_Length>(order), kFFTDirection_Forward);
    const float half = 0.5f;
    vDSP_vsmul(data, 1, &half, data, 1, static_cast<vDSP_Length>(n << 1));
    int i = n >> 1;
    data[2 * i] = data[1];
    data[2 * i + 1] = 0.0f;
    data[1] = 0.0f;
    for (++i; i < n; ++i)
    {
        data[2 * i] = data[2 * (n - i)];
        data[2 * i + 1] = -data[2 * (n - i) + 1];
    }
}

void Fft::inverse(float* data) const noexcept
{
    auto sp = split(data);
    if (n != 1)
        data[1] = data[2 * (n >> 1)];
    vDSP_fft_zrip(impl->setup, &sp, 2, static_cast<vDSP_Length>(order), kFFTDirection_Inverse);
    const float scale = 1.0f / static_cast<float>(n);
    vDSP_vsmul(data, 1, &scale, data, 1, static_cast<vDSP_Length>(n << 1));
    vDSP_vclr(data + n, 1, static_cast<vDSP_Length>(n));
}
#elif CRUCIBLE_FFT_USE_PFFFT
void Fft::forward(float* data) const noexcept
{
    forwardPffft(data);
}

void Fft::inverse(float* data) const noexcept
{
    inversePffft(data);
}
#else
void Fft::forward(float* data) const noexcept
{
    forwardPortable(data);
}

void Fft::inverse(float* data) const noexcept
{
    inversePortable(data);
}
#endif

} // namespace crucible
