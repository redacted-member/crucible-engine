// SPDX-License-Identifier: MIT
#include <crucible/fx/PartitionedConvolver.h>

// Vendored FFTConvolver helpers (private to the library): split-complex buffers and the multiply-accumulate kernel. The
// FFT behind AudioFFT is crucible::Fft (see AudioFftAdapter.cpp).
#include <crucible/Simd.h>
#include "AudioFFT.h"
#include "Utilities.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace crucible
{
namespace
{
constexpr int kLeafSize = 1024;  // largest real FFT run in one step
constexpr int kMergeChunk = 512; // bins per radix-2 merge step
constexpr int kMaxChunk = 512;   // longest run of samples handled between two scheduler checks
constexpr int kMinBlock = 16;
constexpr int kMaxBlock = 1 << 16;

bool isPow2(int v)
{
    return v > 0 && (v & (v - 1)) == 0;
}

size_t nextPow2(size_t v)
{
    size_t p = 1;
    while (p < v)
        p <<= 1;
    return p;
}

// acc += a * b over `len` split-complex bins (len a multiple of 4; all arrays 16-byte aligned or at least valid for
// unaligned loads). The same four-bin kernel as the first-generation convolver (removed), on simd::Float4 (NEON / SSE).
void multiplyAccumulate(float* accRe, float* accIm, const float* aRe, const float* aIm, const float* bRe, const float* bIm,
                        int len) noexcept
{
    using simd::Float4;
    for (int i = 0; i < len; i += 4)
    {
        const Float4 a = Float4::load(aRe + i), ai = Float4::load(aIm + i);
        const Float4 h = Float4::load(bRe + i), g = Float4::load(bIm + i);
        Float4::mulSub(Float4::mulAdd(Float4::load(accRe + i), a, h), ai, g).store(accRe + i);
        Float4::mulAdd(Float4::mulAdd(Float4::load(accIm + i), a, g), ai, h).store(accIm + i);
    }
}

float dotProduct(const float* a, const float* b, int n) noexcept
{
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
    int i = 0;
    for (; i + 4 <= n; i += 4)
    {
        s0 += a[i] * b[i];
        s1 += a[i + 1] * b[i + 1];
        s2 += a[i + 2] * b[i + 2];
        s3 += a[i + 3] * b[i + 3];
    }
    for (; i < n; ++i)
        s0 += a[i] * b[i];
    return (s0 + s1) + (s2 + s3);
}
} // namespace

bool ConvolverPlan::valid() const noexcept
{
    if (latency < 0 || head < 0 || blocks.empty())
        return false;
    int previous = 0;
    for (const int b : blocks)
    {
        if (!isPow2(b) || b < kMinBlock || b > kMaxBlock || b <= previous)
            return false;
        previous = b;
    }
    return latency + head >= 2 * blocks.front();
}

struct NonUniformConvolver::Impl
{
    // One stage: partitions of `block` samples covering kernel taps [start, start + parts * block). The work for the
    // block that completes at time (k + 1) * block is cut in steps (forward FFT, multiply-accumulate chunks, inverse FFT)
    // that run at evenly spaced times of the following block period; the result is due `offset` samples after the
    // block's first sample, and offset >= 2 * block.
    //
    // The 2 * block point real FFTs are split so that no step is longer than a 2048 point FFT: with P = 2^levels, the
    // signal is cut in P interleaved sub-signals (x[P m + r]) that get their own real FFT, and the sub-spectra are
    // merged level by level by radix-2 decimation-in-time butterflies (the inverse runs the same tree backwards). Every
    // merge is cut in chunks of kMergeChunk bins.
    struct Step
    {
        enum Kind : int
        {
            Start,
            FwdLeaf,
            FwdMerge,
            Mac,
            InvMerge,
            InvLeaf
        };
        Kind kind = Start;
        int level = 0, node = 0, from = 0, to = 0;
    };

    struct Stage
    {
        int block = 0, parts = 0, bins = 0, stride = 0, perSlice = 0;
        int size = 0, levels = 0, leaves = 1, leafSize = 0;
        int offset = 0;
        size_t cur = 0, slice = 0;
        std::uint64_t jobStart = 0, nextTime = 0;
        audiofft::AudioFFT fft;
        fftconvolver::SampleBuffer kernelSpectra, inputSpectra, acc, time, leaf;
        std::vector<std::unique_ptr<fftconvolver::SampleBuffer>>
            levelSpectra; // [1 .. levels]: 2^level split-complex half spectra
        std::vector<int> levelStride;
        std::vector<std::vector<float>> twCos,
            twSin; // per merge level l: exp(-2 pi i k / (size >> l)), k = 0 .. (size >> (l + 2))
        std::vector<Step> steps;
        std::vector<float> ring;
        size_t ringMask = 0;

        float* re(fftconvolver::SampleBuffer& b, size_t p) { return b.data() + p * 2 * static_cast<size_t>(stride); }
        float* im(fftconvolver::SampleBuffer& b, size_t p) { return re(b, p) + stride; }
        size_t slices() const { return steps.size(); }
        std::uint64_t sliceTime(size_t i) const { return jobStart + (i * static_cast<size_t>(block)) / steps.size(); }

        // Half spectrum (split complex) of node `node` of tree level `level`; level 0 is the whole signal: `top` (the
        // frequency delay line slot going forward, the accumulator going backward).
        float* nodeRe(int level, int node, float* top)
        {
            return level == 0
                       ? top
                       : levelSpectra[static_cast<size_t>(level)]->data() +
                             static_cast<size_t>(node) * 2 * static_cast<size_t>(levelStride[static_cast<size_t>(level)]);
        }
        float* nodeIm(int level, int node, float* top)
        {
            return level == 0 ? top + stride : nodeRe(level, node, top) + levelStride[static_cast<size_t>(level)];
        }

        void restart()
        {
            inputSpectra.setZero();
            acc.setZero();
            time.setZero();
            std::fill(ring.begin(), ring.end(), 0.0f);
            cur = 0;
            slice = 0;
            jobStart = static_cast<std::uint64_t>(block);
            nextTime = jobStart;
        }

        void buildSteps(int mergeChunk)
        {
            steps.clear();
            steps.push_back({Step::Start, 0, 0, 0, 0});
            for (int r = 0; r < leaves; ++r)
                steps.push_back({Step::FwdLeaf, levels, r, 0, 0});
            for (int l = levels - 1; l >= 0; --l)
            {
                const int m = size >> (l + 1); // merged nodes have `size >> l` samples, bins 0 .. m
                for (int r = 0; r < (1 << l); ++r)
                    for (int k = 0; k <= m / 2; k += mergeChunk)
                        steps.push_back({Step::FwdMerge, l, r, k, std::min(m / 2 + 1, k + mergeChunk)});
            }
            for (int j = 0; j < parts; j += perSlice)
                steps.push_back({Step::Mac, 0, 0, j, std::min(parts, j + perSlice)});
            for (int l = 0; l < levels; ++l)
            {
                const int m = size >> (l + 1);
                for (int r = 0; r < (1 << l); ++r)
                    for (int k = 0; k <= m / 2; k += mergeChunk)
                        steps.push_back({Step::InvMerge, l, r, k, std::min(m / 2 + 1, k + mergeChunk)});
            }
            for (int r = 0; r < leaves; ++r)
                steps.push_back({Step::InvLeaf, levels, r, 0, 0});
        }
    };

    ConvolverPlan plan;
    bool prepared = false;
    int kernelLen = 0;
    int headTaps = 0;
    std::vector<float> reversedHead;
    std::vector<float> hist; // mirrored ring of the input: sample t at t & mask and (t & mask) + histSize
    size_t histSize = 0, histMask = 0;
    std::uint64_t now = 0;
    std::vector<std::unique_ptr<Stage>> stages;

    void runSlice(Stage& s) noexcept
    {
        const Step& st = s.steps[s.slice];
        float* const slotRe = s.re(s.inputSpectra, s.cur);
        switch (st.kind)
        {
        case Step::Start:
        {
            // The last `block` input samples, newest partition first in the frequency delay line.
            const size_t start = (static_cast<size_t>(now) + histSize - static_cast<size_t>(s.block)) & histMask;
            std::memcpy(s.time.data(), hist.data() + start, static_cast<size_t>(s.block) * sizeof(float));
            std::memset(s.time.data() + s.block, 0, static_cast<size_t>(s.block) * sizeof(float));
            s.cur = (s.cur + static_cast<size_t>(s.parts) - 1) % static_cast<size_t>(s.parts);
            s.acc.setZero();
            break;
        }
        case Step::FwdLeaf:
        {
            float* const top = s.re(s.inputSpectra, s.cur);
            for (int i = 0; i < s.leafSize; ++i)
                s.leaf.data()[i] = s.time.data()[i * s.leaves + st.node];
            s.fft.fft(s.leaf.data(), s.nodeRe(st.level, st.node, top), s.nodeIm(st.level, st.node, top));
            break;
        }
        case Step::FwdMerge:
        {
            // S[k] = E[k] + W^k O[k] and S[m - k] = conj(E[k] - W^k O[k]) for k = 0 .. m / 2, with E, O the spectra of the
            // even / odd samples of the node's signal (m = half the node's length).
            float* const top = slotRe;
            const int l = st.level, r = st.node;
            const int m = s.size >> (l + 1);
            const float* er = s.nodeRe(l + 1, r, top);
            const float* ei = s.nodeIm(l + 1, r, top);
            const float* orr = s.nodeRe(l + 1, r + (1 << l), top);
            const float* oi = s.nodeIm(l + 1, r + (1 << l), top);
            float* sr = s.nodeRe(l, r, top);
            float* si = s.nodeIm(l, r, top);
            const float* tc = s.twCos[static_cast<size_t>(l)].data();
            const float* ts = s.twSin[static_cast<size_t>(l)].data();
            for (int k = st.from; k < st.to; ++k)
            {
                const float tR = tc[k] * orr[k] + ts[k] * oi[k];
                const float tI = tc[k] * oi[k] - ts[k] * orr[k];
                sr[k] = er[k] + tR;
                si[k] = ei[k] + tI;
                sr[m - k] = er[k] - tR;
                si[m - k] = tI - ei[k];
            }
            break;
        }
        case Step::Mac:
            for (int j = st.from; j < st.to; ++j)
            {
                const size_t x = (s.cur + static_cast<size_t>(j)) % static_cast<size_t>(s.parts);
                multiplyAccumulate(s.acc.data(), s.acc.data() + s.stride, s.re(s.kernelSpectra, static_cast<size_t>(j)),
                                   s.im(s.kernelSpectra, static_cast<size_t>(j)), s.re(s.inputSpectra, x),
                                   s.im(s.inputSpectra, x), s.stride);
            }
            break;
        case Step::InvMerge:
        {
            // E[k] = (S[k] + conj S[m - k]) / 2, O[k] = (S[k] - conj S[m - k]) / 2 * W^-k, k = 0 .. m / 2.
            float* const top = s.acc.data();
            const int l = st.level, r = st.node;
            const int m = s.size >> (l + 1);
            const float* sr = s.nodeRe(l, r, top);
            const float* si = s.nodeIm(l, r, top);
            float* er = s.nodeRe(l + 1, r, top);
            float* ei = s.nodeIm(l + 1, r, top);
            float* orr = s.nodeRe(l + 1, r + (1 << l), top);
            float* oi = s.nodeIm(l + 1, r + (1 << l), top);
            for (int k = st.from; k < st.to; ++k)
            {
                const float aR = sr[k], aI = si[k], bR = sr[m - k], bI = -si[m - k];
                er[k] = 0.5f * (aR + bR);
                ei[k] = 0.5f * (aI + bI);
                const float dR = 0.5f * (aR - bR), dI = 0.5f * (aI - bI);
                const float c = s.twCos[static_cast<size_t>(l)][static_cast<size_t>(k)],
                            sn = s.twSin[static_cast<size_t>(l)][static_cast<size_t>(k)];
                orr[k] = dR * c - dI * sn;
                oi[k] = dI * c + dR * sn;
            }
            break;
        }
        case Step::InvLeaf:
        {
            float* const top = s.acc.data();
            s.fft.ifft(s.leaf.data(), s.nodeRe(st.level, st.node, top), s.nodeIm(st.level, st.node, top));
            const std::uint64_t base = s.jobStart - static_cast<std::uint64_t>(s.block) +
                                       static_cast<std::uint64_t>(s.offset) + static_cast<std::uint64_t>(st.node);
            // The same additions, in runs between the ring's wrap points without the 64-bit index and mask per sample.
            float* const ring = s.ring.data();
            const float* const leaf = s.leaf.data();
            const size_t stride = static_cast<size_t>(s.leaves), ringSize = s.ringMask + 1;
            size_t r = static_cast<size_t>(base) & s.ringMask;
            for (int i = 0; i < s.leafSize;)
            {
                const int run =
                    static_cast<int>(std::min(static_cast<size_t>(s.leafSize - i), (ringSize - r + stride - 1) / stride));
                float* dst = ring + r;
                for (int j = 0; j < run; ++j)
                    dst[static_cast<size_t>(j) * stride] += leaf[i + j];
                i += run;
                r = (r + static_cast<size_t>(run) * stride) & s.ringMask;
            }
            break;
        }
        }
        if (++s.slice == s.slices())
        {
            s.slice = 0;
            s.jobStart += static_cast<std::uint64_t>(s.block);
        }
        s.nextTime = s.sliceTime(s.slice);
    }
};

NonUniformConvolver::NonUniformConvolver() : impl(new Impl) {}
NonUniformConvolver::~NonUniformConvolver() = default;

int NonUniformConvolver::latency() const noexcept
{
    return impl->prepared ? impl->plan.latency : 0;
}
int NonUniformConvolver::kernelLength() const noexcept
{
    return impl->prepared ? impl->kernelLen : 0;
}
bool NonUniformConvolver::ready() const noexcept
{
    return impl->prepared;
}

bool NonUniformConvolver::prepare(const ConvolverPlan& plan, const float* kernel, int length)
{
    Impl& m = *impl;
    m.prepared = false;
    m.stages.clear();
    if (!plan.valid() || kernel == nullptr || length <= 0)
        return false;
    m.plan = plan;
    m.kernelLen = length;
    const int latency = plan.latency;

    // Direct head: output[t] += h[j] * x[t - latency - j], j < headTaps.
    m.headTaps = std::min(plan.head, length);
    m.reversedHead.assign(static_cast<size_t>(m.headTaps), 0.0f);
    for (int j = 0; j < m.headTaps; ++j)
        m.reversedHead[static_cast<size_t>(m.headTaps - 1 - j)] = kernel[j];

    // Stages: each one covers the kernel until the next block size is allowed to start.
    int at = plan.head;
    for (size_t s = 0; s < plan.blocks.size() && at < length; ++s)
    {
        const int b = plan.blocks[s];
        int end = length;
        if (s + 1 < plan.blocks.size())
        {
            const int need = 2 * plan.blocks[s + 1] - (latency + at);
            const int count = need > 0 ? (need + b - 1) / b : 0;
            if (count == 0)
                continue; // the next stage can already start here
            end = std::min(length, at + count * b);
        }
        auto st = std::make_unique<Impl::Stage>();
        st->block = b;
        st->bins = b + 1;
        st->stride = (st->bins + 3) & ~3;
        st->parts = (end - at + b - 1) / b;
        st->offset = latency + at;
        st->perSlice = std::max(1, 2048 / st->bins);
        st->size = 2 * b;
        while ((st->size >> st->levels) > kLeafSize)
            ++st->levels;
        st->leaves = 1 << st->levels;
        st->leafSize = st->size >> st->levels;
        st->fft.init(static_cast<size_t>(st->leafSize));
        st->leaf.resize(static_cast<size_t>(st->leafSize));
        st->levelSpectra.resize(static_cast<size_t>(st->levels) + 1);
        st->levelStride.assign(static_cast<size_t>(st->levels) + 1, 0);
        for (int l = 1; l <= st->levels; ++l)
        {
            const int nodeBins = (st->size >> (l + 1)) + 1;
            const int stride = (nodeBins + 3) & ~3;
            st->levelStride[static_cast<size_t>(l)] = stride;
            st->levelSpectra[static_cast<size_t>(l)] = std::make_unique<fftconvolver::SampleBuffer>();
            st->levelSpectra[static_cast<size_t>(l)]->resize(2 * static_cast<size_t>(stride) * static_cast<size_t>(1) << l);
        }
        st->twCos.resize(static_cast<size_t>(st->levels));
        st->twSin.resize(static_cast<size_t>(st->levels));
        for (int l = 0; l < st->levels; ++l)
        {
            const int node = st->size >> l;
            auto& c = st->twCos[static_cast<size_t>(l)];
            auto& sn = st->twSin[static_cast<size_t>(l)];
            c.resize(static_cast<size_t>(node / 4) + 1);
            sn.resize(c.size());
            for (size_t k = 0; k < c.size(); ++k)
            {
                const double a = 6.283185307179586476925287 * static_cast<double>(k) / static_cast<double>(node);
                c[k] = static_cast<float>(std::cos(a));
                sn[k] = static_cast<float>(std::sin(a));
            }
        }
        st->buildSteps(kMergeChunk);
        const size_t spectrum = 2 * static_cast<size_t>(st->stride);
        st->kernelSpectra.resize(spectrum * static_cast<size_t>(st->parts));
        st->inputSpectra.resize(spectrum * static_cast<size_t>(st->parts));
        st->acc.resize(spectrum);
        st->time.resize(static_cast<size_t>(2 * b));
        st->ring.assign(nextPow2(static_cast<size_t>(st->offset) + 3 * static_cast<size_t>(b) + 1), 0.0f);
        st->ringMask = st->ring.size() - 1;
        audiofft::AudioFFT full;
        full.init(static_cast<size_t>(2 * b));
        for (int p = 0; p < st->parts; ++p)
        {
            const int first = at + p * b;
            const int n = std::min(b, length - first);
            std::memcpy(st->time.data(), kernel + first, static_cast<size_t>(n) * sizeof(float));
            std::memset(st->time.data() + n, 0, static_cast<size_t>(2 * b - n) * sizeof(float));
            full.fft(st->time.data(), st->re(st->kernelSpectra, static_cast<size_t>(p)),
                     st->im(st->kernelSpectra, static_cast<size_t>(p)));
        }
        m.stages.push_back(std::move(st));
        at = end;
    }

    size_t longest = static_cast<size_t>(latency + m.headTaps);
    for (const auto& st : m.stages)
        longest = std::max(longest, static_cast<size_t>(st->block));
    m.histSize = nextPow2(longest + static_cast<size_t>(kMaxChunk) + 1);
    m.histMask = m.histSize - 1;
    m.hist.assign(2 * m.histSize, 0.0f);
    m.prepared = true;
    reset();
    return true;
}

void NonUniformConvolver::reset() noexcept
{
    Impl& m = *impl;
    std::fill(m.hist.begin(), m.hist.end(), 0.0f);
    m.now = 0;
    for (auto& st : m.stages)
        st->restart();
}

void NonUniformConvolver::process(const float* in, float* out, int n) noexcept
{
    Impl& m = *impl;
    if (!m.prepared)
    {
        if (n > 0)
            std::memset(out, 0, static_cast<size_t>(n) * sizeof(float));
        return;
    }
    const int latency = m.plan.latency;
    while (n > 0)
    {
        std::uint64_t nextEvent = ~static_cast<std::uint64_t>(0);
        for (auto& st : m.stages)
        {
            while (st->nextTime == m.now)
                m.runSlice(*st);
            nextEvent = std::min(nextEvent, st->nextTime);
        }
        // Pairwise (MSVC calls a library routine for the initializer-list form).
        const int c = static_cast<int>(
            std::min(std::min(static_cast<std::uint64_t>(n), static_cast<std::uint64_t>(kMaxChunk)), nextEvent - m.now));

        // Input into the mirrored history first (the output may alias the input).
        for (int i = 0; i < c; ++i)
        {
            const size_t p = static_cast<size_t>(m.now + static_cast<std::uint64_t>(i)) & m.histMask;
            m.hist[p] = in[i];
            m.hist[p + m.histSize] = in[i];
        }
        if (m.headTaps > 0)
        {
            const size_t back = static_cast<size_t>(latency + m.headTaps - 1);
            for (int i = 0; i < c; ++i)
            {
                const size_t u = static_cast<size_t>(m.now + static_cast<std::uint64_t>(i));
                const size_t start = (u + m.histSize - back) & m.histMask;
                out[i] = dotProduct(m.reversedHead.data(), m.hist.data() + start, m.headTaps);
            }
        }
        else
        {
            std::memset(out, 0, static_cast<size_t>(c) * sizeof(float));
        }
        for (auto& st : m.stages)
        {
            // The ring positions of the c samples are one or two contiguous runs; as plain loops they vectorise (each
            // out[i] still gets the stages' contributions added in stage order).
            float* ring = st->ring.data();
            if (static_cast<size_t>(c) > st->ringMask + 1) // not with any plan the convolver builds; kept general
            {
                for (int i = 0; i < c; ++i)
                {
                    const size_t r = static_cast<size_t>(m.now + static_cast<std::uint64_t>(i)) & st->ringMask;
                    out[i] += ring[r];
                    ring[r] = 0.0f;
                }
                continue;
            }
            const size_t r0 = static_cast<size_t>(m.now) & st->ringMask;
            const int first = static_cast<int>(std::min(static_cast<size_t>(c), st->ringMask + 1 - r0));
            for (int i = 0; i < first; ++i)
            {
                out[i] += ring[r0 + static_cast<size_t>(i)];
                ring[r0 + static_cast<size_t>(i)] = 0.0f;
            }
            for (int i = first; i < c; ++i)
            {
                out[i] += ring[static_cast<size_t>(i - first)];
                ring[static_cast<size_t>(i - first)] = 0.0f;
            }
        }
        m.now += static_cast<std::uint64_t>(c);
        in += c;
        out += c;
        n -= c;
    }
}

} // namespace crucible
