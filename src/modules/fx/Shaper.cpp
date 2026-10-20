// SPDX-License-Identifier: MIT
// fx.shaper: a waveshaper whose transfer curve the user draws (include/crucible/fx/ShaperCurve.h): 2..64 points with a
// curvature per segment and an optional symmetric (odd, mirrored) mode.
//
// The control thread validates a curve and publishes it as an immutable table (RtShared; the pulled "curve" view reads a control-side copy); at the start of a block the
// audio thread copies it into the instance's own state (a plain array copy, no allocation) and cross-fades from the curve
// it was playing to the new one over 10 ms, so a curve edit never clicks. Drive (pre gain), output and mix move through two
// cascaded 5 ms one-poles per sample (smooth in value and slope, the same whatever the host block size). The shaper runs at 1x, 2x, 4x or 8x behind the half-band chains of
// include/crucible/fx/OversampledBlock.h; the dry path of the mix is the input delayed by the reported latency. At the default identity
// curve with 0 dB drive the wet path is left out altogether, so the module is a bit-exact (delayed) pass-through there.
#include "FxModuleSupport.h"

#include <crucible/RtShared.h>
#include <crucible/fx/OversampledBlock.h>
#include <crucible/fx/ShaperCurve.h>
#include <crucible/fx/Smoothing.h>
#include <crucible/module/LatencyReporter.h>
#include <crucible/module/ShaperCurve.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>

namespace crucible::modules::fx
{
namespace
{
using crucible::fx::DryPath;
using crucible::fx::OversampledBlock;

constexpr float kMaxDriveDb = 24.0f, kMinDriveDb = -24.0f;
constexpr double kFadeSeconds = 0.010;
constexpr double kSmoothSeconds = 0.005; // time constant of each of the two smoothing poles
constexpr double kDcCornerHz = 10.0;
constexpr int kMaxStages = 3; // 8x
constexpr int kMaxFactor = 1 << kMaxStages;

// Two cascaded one-poles: smooth in value and slope, and settling exactly on the target.
class Smoother
{
public:
    void setRate(double rate) { coefficient = static_cast<float>(1.0 - std::exp(-1.0 / (kSmoothSeconds * rate))); }
    void reset(float value) { a = b = value; }
    float next(float target)
    {
        a = step(a, target);
        b = step(b, a);
        return b;
    }
    float value() const { return b; }

private:
    float step(float v, float target) const
    {
        const float d = target - v;
        return std::abs(d) <= 1.0e-7f * std::max(1.0f, std::abs(target)) ? target : crucible::fx::onePoleStepSnap(v, target, coefficient);
    }
    float a = 0.0f, b = 0.0f, coefficient = 0.01f;
};

class Instance final : public EffectModule, public LatencyReporter, public ShaperCurve
{
public:
    explicit Instance(const ModuleDescriptor& d)
        : mix(localIndex(d, "fx_shaper_mix")), drive(localIndex(d, "fx_shaper_drive")),
          output(localIndex(d, "fx_shaper_output")), dcBlock(localIndex(d, "fx_shaper_dc")),
          oversample(localIndex(d, "fx_shaper_os"))
    {
    }

    // ---- EffectModule
    void prepare(const ModuleContext& c) override
    {
        sr = c.sampleRate;
        quality = c.quality;
        for (int s = 0; s <= kMaxStages; ++s)
            blocks[static_cast<size_t>(s)].prepare(2, s, c.oversampling, c.quality, c.maxBlockSize, DryPath::Delay);
        const size_t scratchSize = static_cast<size_t>(std::max(c.maxBlockSize, 32)) * kMaxFactor;
        driveGain.assign(scratchSize, 0.0f);
        fadeWeight.assign(scratchSize, 0.0f);
        fadeLength = std::max(16, static_cast<int>(std::lround(kFadeSeconds * sr)));
        size_t ring = 64;
        while (ring < static_cast<size_t>(std::max(c.maxBlockSize, 32)) + 512)
            ring *= 2;
        for (auto& r : dryRing)
            r.assign(ring, 0.0f);
        ringMask = ring - 1;
        stages = defaultStages();
        reportedLatency.store(blocks[static_cast<size_t>(stages)].latencySamples(), std::memory_order_relaxed);
        reset();
    }

    void reset() override
    {
        for (auto& b : blocks)
            b.reset();
        const ShaperCurveTable* published = tables.acquire();
        current = published != nullptr ? *published : ShaperCurveTable{};
        tables.release();
        previous = current;
        fadePos = fadeTotal = 1;
        fresh = true;
        smoothed = false;
        for (auto& s : dc)
            s = {};
        for (auto& r : dryRing)
            std::fill(r.begin(), r.end(), 0.0f);
        ringPos = 0;
        dirty = false;
    }

    void deactivate() override
    {
        if (dirty)
        {
            reset();
            dirty = false;
        }
    }

    void process(float* l, float* r, int n, const EffectInputs& in) override
    {
        if (n <= 0)
            return;
        dirty = true;
        selectStages(in.choice(oversample));
        OversampledBlock& block = blocks[static_cast<size_t>(stages)];
        const int factor = block.factor();
        takeCurve(factor);

        const float driveDb = std::clamp(in.value(drive), kMinDriveDb, kMaxDriveDb);
        driveTarget = std::pow(10.0f, driveDb / 20.0f);
        const float outTarget = std::pow(10.0f, std::clamp(in.value(output), -24.0f, 12.0f) / 20.0f);
        // The identity curve at 0 dB drive is the input itself: leave the wet path out (the mix glides to it).
        const bool unity = current.identity && std::abs(driveDb) < 1.0e-6f;
        const float mixTarget = unity ? 0.0f : std::clamp(in.value(mix), 0.0f, 1.0f);
        if (!smoothed)
        {
            driveSmooth.reset(driveTarget);
            mixSmooth.reset(mixTarget);
            outSmooth.reset(outTarget);
            smoothed = true;
        }
        driveSmooth.setRate(sr * factor);
        mixSmooth.setRate(sr);
        outSmooth.setRate(sr);
        useDc = in.on(dcBlock);
        if (!useDc)
            for (auto& s : dc)
                s = {};
        dcR = static_cast<float>(1.0 - 2.0 * 3.14159265358979323846 * kDcCornerHz / (sr * factor));

        float* io[2] = {l, r};
        for (size_t c = 0; c < 2; ++c)
            for (int i = 0; i < n; ++i)
                dryRing[c][(ringPos + static_cast<size_t>(i)) & ringMask] = io[c][i];
        block.process(io, n, 1.0f, [this](float* const* x, int ch, int count) { shape(x, ch, count); });
        const size_t lat = static_cast<size_t>(block.latencySamples());
        for (int i = 0; i < n; ++i)
        {
            const float m = mixSmooth.next(mixTarget);
            const float g = outSmooth.next(outTarget);
            for (size_t c = 0; c < 2; ++c)
            {
                const float dry = dryRing[c][(ringPos + static_cast<size_t>(i) - lat) & ringMask];
                io[c][i] = (dry + (io[c][i] - dry) * m) * g;
            }
        }
        ringPos += static_cast<size_t>(n);
    }

    // ---- LatencyReporter
    int latencySamples() const override { return reportedLatency.load(std::memory_order_relaxed); }
    bool takeLatencyChanged() override { return latencyChanged.exchange(false, std::memory_order_relaxed); }

    // ---- ShaperCurve (control thread, except fillCurveView)
    bool setCurve(const ShaperPoint* points, int count, bool symmetric, std::string& error) override
    {
        auto table = std::make_unique<ShaperCurveTable>();
        if (!makeShaperCurveTable(points, count, symmetric, *table, error))
            return false;
        std::lock_guard<std::mutex> lock(sourceMutex);
        table->serial = ++serialCounter;
        controlTable = *table;
        source.assign(points, points + count);
        sourceSymmetric = symmetric;
        sourceSet = true;
        tables.publish(std::move(table));
        return true;
    }
    void resetCurve() override
    {
        std::lock_guard<std::mutex> lock(sourceMutex);
        source.clear();
        sourceSet = false;
        auto table = std::make_unique<ShaperCurveTable>();
        table->serial = ++serialCounter;
        controlTable = *table;
        tables.publish(std::move(table));
    }
    bool curve(std::vector<ShaperPoint>& points, bool& symmetric) const override
    {
        std::lock_guard<std::mutex> lock(sourceMutex);
        if (!sourceSet)
            return false;
        points = source;
        symmetric = sourceSymmetric;
        return true;
    }
    void fillCurveView(float* out) override
    {
        ShaperCurveTable table;
        {
            std::lock_guard<std::mutex> lock(sourceMutex);
            table = controlTable;
        }
        for (int i = 0; i < kViewPoints; ++i)
            out[i] = table.eval(-1.0f + 2.0f * static_cast<float>(i) / static_cast<float>(kViewPoints - 1));
    }

private:
    int defaultStages() const { return quality == Quality::Draft ? 1 : (quality == Quality::Standard ? 2 : 3); }

    // Block boundary: a newly published table becomes the target of a cross-fade (over fadeLength base-rate samples, counted
    // here at the oversampled rate). The first block after a reset takes it at once.
    void takeCurve(int factor)
    {
        const ShaperCurveTable* published = tables.acquire();
        if (published != nullptr && published->serial != current.serial)
        {
            previous = current;
            current = *published;
            fadePos = 0;
            fadeTotal = fresh ? 0 : fadeLength * factor;
        }
        tables.release();
        fresh = false;
    }

    void adoptLatencyParams(const EffectInputs& in) override
    {
        const int wanted = stagesFor(in.choice(oversample));
        if (wanted == stages)
            return;
        stages = wanted;
        reportedLatency.store(blocks[static_cast<size_t>(stages)].latencySamples(), std::memory_order_relaxed);
    }

    int stagesFor(int choice) const { return choice <= 0 ? defaultStages() : std::clamp(choice - 1, 0, kMaxStages); }

    // 0 = Auto (by quality), 1 = 1x, 2 = 2x, 3 = 4x, 4 = 8x. A change of factor changes the latency: the new path starts from
    // silence and the change is announced, as the clipper does.
    void selectStages(int choice)
    {
        const int wanted = stagesFor(choice);
        if (wanted == stages)
            return;
        stages = wanted;
        blocks[static_cast<size_t>(stages)].reset();
        for (auto& s : dc)
            s = {};
        reportedLatency.store(blocks[static_cast<size_t>(stages)].latencySamples(), std::memory_order_relaxed);
        latencyChanged.store(true, std::memory_order_relaxed);
    }

    // The kernel at the oversampled rate: drive, curve (cross-faded while a new one comes in), optional DC block.
    void shape(float* const* x, int channels, int count)
    {
        float* g = driveGain.data();
        for (int i = 0; i < count; ++i)
            g[i] = driveSmooth.next(driveTarget);
        const bool fading = fadePos < fadeTotal;
        if (fading)
        {
            float* w = fadeWeight.data();
            const int len = fadeTotal;
            for (int i = 0; i < count; ++i)
            {
                const int pos = fadePos + i + 1;
                w[i] = pos >= len ? 1.0f : crucible::fx::smootherstep(static_cast<float>(pos) / static_cast<float>(len));
            }
        }
        for (int c = 0; c < channels; ++c)
        {
            float* p = x[c];
            if (fading)
            {
                const float* w = fadeWeight.data();
                for (int i = 0; i < count; ++i)
                {
                    const float v = p[i] * g[i];
                    const float a = previous.eval(v);
                    p[i] = a + w[i] * (current.eval(v) - a);
                }
            }
            else
                for (int i = 0; i < count; ++i)
                    p[i] = current.eval(p[i] * g[i]);
            if (useDc)
            {
                Dc& s = dc[static_cast<size_t>(c)];
                for (int i = 0; i < count; ++i)
                {
                    const float y = p[i] - s.x + dcR * s.y;
                    s.x = p[i];
                    s.y = y;
                    p[i] = y;
                }
            }
        }
        if (fading)
            fadePos = std::min(fadePos + count, fadeTotal);
    }

    struct Dc
    {
        float x = 0.0f, y = 0.0f;
    };

    const int mix, drive, output, dcBlock, oversample;
    double sr = 48000.0;
    Quality quality = Quality::Standard;
    std::array<OversampledBlock, kMaxStages + 1> blocks;
    int stages = 2;
    std::vector<float> driveGain, fadeWeight;
    Smoother driveSmooth, mixSmooth, outSmooth;
    float driveTarget = 1.0f;
    bool smoothed = false, fresh = true, useDc = false, dirty = false;
    std::array<std::vector<float>, 2> dryRing;
    size_t ringMask = 0, ringPos = 0;
    float dcR = 0.999f;
    std::array<Dc, 2> dc{};
    ShaperCurveTable current, previous;
    int fadeLength = 480, fadePos = 1, fadeTotal = 1;
    std::atomic<int> reportedLatency{0};
    std::atomic<bool> latencyChanged{false};

    RtShared<ShaperCurveTable> tables;
    mutable std::mutex sourceMutex;
    std::vector<ShaperPoint> source;
    ShaperCurveTable controlTable; // what was published last, for the pulled curve view
    bool sourceSymmetric = false, sourceSet = false;
    std::uint64_t serialCounter = 0;
};

void shortNames(std::vector<ParamDef>& p)
{
    static const std::pair<const char*, const char*> names[] = {
        {"fx_shaper_on", "On"},    {"fx_shaper_mix", "Mix"}, {"fx_shaper_drive", "Drive"},
        {"fx_shaper_output", "Out"}, {"fx_shaper_dc", "DC"},   {"fx_shaper_os", "OvrSmp"},
        {"fx_pos_shaper", "Slot"},
    };
    for (auto& d : p)
        for (const auto& n : names)
            if (d.id == n.first)
                d.shortName = n.second;
}
} // namespace

ModuleDescriptor shaperEffect()
{
    auto d = effectDescriptor(
        "fx.shaper", "Shaper", "Distortion", CpuClass::Medium,
        "Waveshaper with a user-drawn transfer curve: 2 to 64 points (x and y within -1..1, x ascending) with a "
        "curvature per segment (y = y0 + (y1 - y0) * t^(4^curve), 0 is straight), optionally symmetric (the positive half "
        "is drawn and mirrored). The curve is set with crucible_set_shaper_curve, published to the audio thread as an "
        "immutable table, cross-faded in over 10 ms and saved in patches; the \"curve\" view returns it for drawing. "
        "Drive (pre gain), output, a latency-aligned mix, an optional DC block and 1x to 8x oversampling in the "
        "half-band chains. The default curve is the identity: with 0 dB drive the module passes the input through "
        "bit for bit (delayed by the reported latency). A curve that ends short of +-1 or below it clips there. Latency is "
        "the oversampling filters, reported.");
    auto& p = d.params;
    addOn(p, "shaper", "Shaper", "fx_shaper", 1.0f);
    addF(p, "shaper_drive", "Shaper Drive", "fx_shaper", kMinDriveDb, kMaxDriveDb, 0.0f, "dB");
    addF(p, "shaper_output", "Shaper Output", "fx_shaper", -24.0f, 12.0f, 0.0f, "dB");
    p.push_back(params::toggle("fx_shaper_dc", "Shaper DC Block", "fx_shaper_2", false));
    p.push_back(params::choice("fx_shaper_os", "Shaper Oversampling", "fx_shaper_2", {"Auto", "1x", "2x", "4x", "8x"}, 0));
    p.push_back(params::integer("fx_pos_shaper", "Shaper Slot", "fx_shaper_2", 1, kNumModuleFx, kNumModuleFx));
    shortNames(p);
    d.onParam = "fx_shaper_on";
    d.positionParam = "fx_pos_shaper";
    d.capabilities = {"latency", "shaper_curve"};
    d.views = {"curve"};
    d.pages.push_back({"fx_shaper", "Shaper", "fx", "Shaper", "Mix, drive and output; the curve is set through the API."});
    d.pages.push_back({"fx_shaper_2", "Shaper 2", "fx", "Shaper Quality", "DC block, oversampling and the chain slot."});
    d.createEffect = [] { return std::make_unique<Instance>(*ModuleRegistry::instance().find("fx.shaper")); };
    return d;
}

} // namespace crucible::modules::fx
