// SPDX-License-Identifier: MIT
// Early reflections of the algorithmic reverbs (include/crucible/fx/Fdn.h): image-source shoebox taps for Room and
// Chamber (Allen and Berkley, "Image method for efficiently simulating small-room acoustics", JASA 1979) and jittered
// velvet-noise taps for Hall and Ambience (Karjalainen and Jarvelainen, "Reverberation modeling using velvet noise",
// AES 30th Conference 2007). Every tap gets a pseudo-random polarity so the sum has no comb pattern. The tables are built
// in prepare(); selecting one and processing never allocate.

#include <crucible/fx/Fdn.h>

#include <algorithm>
#include <cmath>

namespace crucible::fx
{
namespace
{
std::uint32_t nextRandom(std::uint32_t& state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

float uniform01(std::uint32_t& state)
{
    return static_cast<float>(nextRandom(state) >> 8) * (1.0f / 16777216.0f);
}

struct Vec3
{
    float x, y, z;
};

float distance(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}
} // namespace

void EarlyReflections::buildShoebox(Table& t, float lx, float ly, float lz, float reflect, std::uint32_t seed) const
{
    constexpr float kSpeed = 343.0f;
    const Vec3 source{0.27f * lx, 0.31f * ly, 0.48f * lz};
    const Vec3 listener{0.62f * lx, 0.58f * ly, 0.50f * lz};
    const Vec3 earL{listener.x - 0.09f, listener.y, listener.z}, earR{listener.x + 0.09f, listener.y, listener.z};
    const float r0L = distance(source, earL), r0R = distance(source, earR);
    struct Image
    {
        float rc, rl, rr, amp;
    };
    std::vector<Image> images;
    const float dims[3] = {lx, ly, lz};
    const float src[3] = {source.x, source.y, source.z};
    for (int qx = 0; qx < 2; ++qx)
        for (int jx = -2; jx <= 2; ++jx)
            for (int qy = 0; qy < 2; ++qy)
                for (int jy = -2; jy <= 2; ++jy)
                    for (int qz = 0; qz < 2; ++qz)
                        for (int jz = -2; jz <= 2; ++jz)
                        {
                            const int hits = std::abs(jx - qx) + std::abs(jx) + std::abs(jy - qy) + std::abs(jy) +
                                             std::abs(jz - qz) + std::abs(jz);
                            if (hits == 0 || hits > 6)
                                continue;
                            const Vec3 img{(1 - 2 * qx) * src[0] + 2.0f * static_cast<float>(jx) * dims[0],
                                           (1 - 2 * qy) * src[1] + 2.0f * static_cast<float>(jy) * dims[1],
                                           (1 - 2 * qz) * src[2] + 2.0f * static_cast<float>(jz) * dims[2]};
                            images.push_back({distance(img, listener), distance(img, earL), distance(img, earR),
                                              std::pow(reflect, static_cast<float>(hits))});
                        }
    std::sort(images.begin(), images.end(), [](const Image& a, const Image& b) { return a.rc < b.rc; });
    std::uint32_t rng = seed * 2654435761u + 77u;
    const int count = std::min<int>(kMaxTaps, static_cast<int>(images.size()));
    t.count = count;
    float el = 0.0f, er = 0.0f;
    for (int k = 0; k < count; ++k)
    {
        const Image& im = images[static_cast<std::size_t>(k)];
        const float sign = (nextRandom(rng) & 0x200u) != 0 ? 1.0f : -1.0f;
        Tap& tap = t.tap[static_cast<std::size_t>(k)];
        tap.delayL = std::max(0.0005f, (im.rl - r0L) / kSpeed);
        tap.delayR = std::max(0.0005f, (im.rr - r0R) / kSpeed);
        tap.gainL = sign * im.amp * r0L / im.rl;
        tap.gainR = sign * im.amp * r0R / im.rr;
        el += tap.gainL * tap.gainL;
        er += tap.gainR * tap.gainR;
    }
    for (int k = 0; k < count; ++k)
    {
        Tap& tap = t.tap[static_cast<std::size_t>(k)];
        tap.gainL /= std::sqrt(std::max(el, 1.0e-12f));
        tap.gainR /= std::sqrt(std::max(er, 1.0e-12f));
    }
}

void EarlyReflections::buildVelvet(Table& t, float startMs, float endMs, float tauMs, int count, std::uint32_t seed) const
{
    std::uint32_t rng = seed * 2246822519u + 99u;
    t.count = std::min(count, kMaxTaps);
    float el = 0.0f, er = 0.0f;
    for (int k = 0; k < t.count; ++k)
    {
        Tap& tap = t.tap[static_cast<std::size_t>(k)];
        for (int ear = 0; ear < 2; ++ear)
        {
            const float u = (static_cast<float>(k) + uniform01(rng)) / static_cast<float>(t.count);
            const float ms = startMs + (endMs - startMs) * std::pow(u, 0.9f);
            const float gain =
                std::exp(-ms / tauMs) * (0.6f + 0.4f * uniform01(rng)) * ((nextRandom(rng) & 0x200u) != 0 ? 1.0f : -1.0f);
            (ear == 0 ? tap.delayL : tap.delayR) = ms * 0.001f;
            (ear == 0 ? tap.gainL : tap.gainR) = gain;
            (ear == 0 ? el : er) += gain * gain;
        }
    }
    for (int k = 0; k < t.count; ++k)
    {
        Tap& tap = t.tap[static_cast<std::size_t>(k)];
        tap.gainL /= std::sqrt(std::max(el, 1.0e-12f));
        tap.gainR /= std::sqrt(std::max(er, 1.0e-12f));
    }
}

void EarlyReflections::prepare(double sampleRate, std::uint32_t seed)
{
    sr = sampleRate;
    for (auto& t : tables)
        t.count = 0;
    buildShoebox(tables[static_cast<std::size_t>(ReverbAlgorithm::Room)], 5.2f, 3.9f, 2.9f, 0.78f, seed + 1);
    buildShoebox(tables[static_cast<std::size_t>(ReverbAlgorithm::Chamber)], 10.4f, 7.0f, 4.6f, 0.88f, seed + 2);
    buildVelvet(tables[static_cast<std::size_t>(ReverbAlgorithm::Hall)], 4.0f, 100.0f, 40.0f, 32, seed + 3);
    buildVelvet(tables[static_cast<std::size_t>(ReverbAlgorithm::Ambience)], 1.5f, 34.0f, 14.0f, 32, seed + 4);
    buffer.allocate(static_cast<int>(0.26 * sampleRate) + 8);
    selected = -1;
    taps = 0;
}

void EarlyReflections::select(ReverbAlgorithm algorithm)
{
    selected = static_cast<int>(algorithm);
    taps = tables[static_cast<std::size_t>(selected)].count;
    primed = false;
}

void EarlyReflections::process(const float* mono, float* outL, float* outR, int n, float scale, float density)
{
    for (int s = 0; s < n; ++s)
    {
        outL[s] = 0.0f;
        outR[s] = 0.0f;
    }
    for (int s = 0; s < n; ++s)
        buffer.write(mono[s]);
    if (selected < 0 || taps == 0)
        return;
    const Table& table = tables[static_cast<std::size_t>(selected)];
    // Taps fade in with density; the first few are always there.
    const float active = 12.0f + std::clamp(density, 0.0f, 1.0f) * static_cast<float>(taps - 12) + 2.0f;
    const float sampleRate = static_cast<float>(sr);
    const float maxDelay = static_cast<float>(buffer.capacity() - 2 * DelayBuffer::kGuard);
    if (!primed)
    {
        lastScale = scale;
        lastActive = active;
        primed = true;
    }
    const float scaleStart = lastScale, activeStart = lastActive;
    lastScale = scale;
    lastActive = active;
    const bool moving = scaleStart != scale || activeStart != active;
    const float inv = 1.0f / static_cast<float>(n);
    for (int k = 0; k < taps; ++k)
    {
        const Tap& tap = table.tap[static_cast<std::size_t>(k)];
        const float w = std::clamp((active - static_cast<float>(k)) / 3.0f, 0.0f, 1.0f);
        const float wStart = std::clamp((activeStart - static_cast<float>(k)) / 3.0f, 0.0f, 1.0f);
        if (w <= 0.0f && wStart <= 0.0f)
            break;
        for (int ear = 0; ear < 2; ++ear)
        {
            const float seconds = ear == 0 ? tap.delayL : tap.delayR;
            const float baseGain = ear == 0 ? tap.gainL : tap.gainR;
            float* out = ear == 0 ? outL : outR;
            const float delay = std::clamp(seconds * scale * sampleRate, 1.0f, maxDelay);
            if (!moving)
            {
                const float gain = baseGain * w;
                const int whole = static_cast<int>(delay);
                const float f = delay - static_cast<float>(whole);
                // Sample s of this chunk is `n - s` writes before the next one; its delayed value is `whole` further back.
                const float* newer = buffer.window(n + whole);     // delay `whole`, for the first sample of the chunk
                const float* older = buffer.window(n + whole + 1); // one sample older
                for (int s = 0; s < n; ++s)
                    out[s] += gain * (newer[s] + (older[s] - newer[s]) * f);
                continue;
            }
            const float delayStart = std::clamp(seconds * scaleStart * sampleRate, 1.0f, maxDelay);
            for (int s = 0; s < n; ++s)
            {
                const float t = static_cast<float>(s + 1) * inv;
                const float d = delayStart + (delay - delayStart) * t;
                const float gain = baseGain * (wStart + (w - wStart) * t);
                const int whole = static_cast<int>(d);
                const float f = d - static_cast<float>(whole);
                const int back = n - s + whole;
                const float a = buffer.window(back)[0], b = buffer.window(back + 1)[0];
                out[s] += gain * (a + (b - a) * f);
            }
        }
    }
}

} // namespace crucible::fx
