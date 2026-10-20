// SPDX-License-Identifier: MIT
// Implements the audiofft::AudioFFT interface of the vendored FFTConvolver sources (third_party/fftconvolver/AudioFFT.h)
// on top of crucible::Fft, so that the convolver code never uses the bundled Ooura / FFTW / Accelerate backends.
#include "AudioFFT.h"

#include <crucible/Fft.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace audiofft
{
namespace detail
{
class AudioFFTImpl
{
public:
    void init(size_t size)
    {
        int order = 0;
        while ((static_cast<size_t>(1) << order) < size)
            ++order;
        n = static_cast<size_t>(1) << order;
        fft = std::make_unique<crucible::Fft>(order);
        work.assign(2 * n, 0.0f);
    }

    // Real input of n samples -> bins 0..n/2 as separate real and imaginary arrays.
    void forward(const float* data, float* re, float* im)
    {
        std::memcpy(work.data(), data, n * sizeof(float));
        std::fill(work.begin() + static_cast<std::ptrdiff_t>(n), work.end(), 0.0f);
        fft->forward(work.data());
        for (size_t k = 0; k <= n / 2; ++k)
        {
            re[k] = work[2 * k];
            im[k] = work[2 * k + 1];
        }
    }

    // Bins 0..n/2 -> n real samples (scaled so that inverse(forward(x)) == x).
    void inverse(float* data, const float* re, const float* im)
    {
        for (size_t k = 0; k <= n / 2; ++k)
        {
            work[2 * k] = re[k];
            work[2 * k + 1] = im[k];
        }
        // DC and Nyquist are real for a real signal.
        work[1] = 0.0f;
        work[n + 1] = 0.0f;
        fft->inverse(work.data());
        std::memcpy(data, work.data(), n * sizeof(float));
    }

private:
    size_t n = 0;
    std::unique_ptr<crucible::Fft> fft;
    std::vector<float> work;
};
} // namespace detail

AudioFFT::AudioFFT() : _impl(new detail::AudioFFTImpl) {}
AudioFFT::~AudioFFT() = default;
void AudioFFT::init(size_t size)
{
    _impl->init(size);
}
void AudioFFT::fft(const float* data, float* re, float* im)
{
    _impl->forward(data, re, im);
}
void AudioFFT::ifft(float* data, const float* re, const float* im)
{
    _impl->inverse(data, re, im);
}
size_t AudioFFT::ComplexSize(size_t size)
{
    return size / 2 + 1;
}

} // namespace audiofft
