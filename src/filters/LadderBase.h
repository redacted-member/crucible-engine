// SPDX-License-Identifier: MIT
#pragma once

// Shared machinery of the ladder family: control smoothing, tolerances, the drive stage, oversampling around a
// per-sample kernel, the scalar and four-lane drivers. A model derives from LadderBase and supplies the kernel for one
// lane and for four.
//
// Signal path per control chunk of 16 host-rate samples (all stages are streaming, so chunks join seamlessly):
//
//   host rate -> up to 2x the filter rate -> drive shaper -> down to the filter rate -> ladder kernel -> down to host rate
//
// The drive shaper tanh(a x) runs at twice the rate of the ladder because a hard-clipped input is what aliases; the
// ladder itself only sees the shaped, bounded signal. Without drive the shaper is the identity (a = 0).

#include <crucible/filters/FilterModel.h>

#include "NonlinearSolve.h"

#include <array>
#include <complex>
#include <vector>

namespace crucible::filters
{

// Parameters of one chunk of oversampled samples, four lanes wide. Values with a d prefix are per-sample steps; the
// kernel starts from the value and the driver advances it by n steps after the call.
struct LaneBlock
{
    alignas(16) float g[4]{}, dg[4]{};      // integrator gain tan(pi fc / fs) (times the model's scale)
    alignas(16) float k[4]{}, dk[4]{};      // feedback gain
    alignas(16) float chr[4]{};             // gain compensation blend
    alignas(16) float gm[4][4]{};           // per stage integrator gain factor (capacitor mismatch), [stage][lane]
    alignas(16) float gMid[4]{}, kMid[4]{}; // values in the middle of the control chunk, for the linear predictor
    bool rebuild = true;                    // the targets changed: the kernel rebuilds its linear predictor
};

class LadderBase : public AnalogFilter
{
public:
    void prepare(double sampleRate, Quality quality, int lanes, int maxBlock) override;
    void reset() override;
    void resetLane(int lane) override;
    void setSeed(std::uint32_t seed) override;
    void setLaneSeed(int lane, std::uint32_t seed, int draw) override;
    void setStartupNoise(bool on) override { noiseOn = on; }
    // The shared high-pass stage ahead of the model (60 / 180 / 540 Hz), for the models that report supportsHighpass().
    void setHighpassStep(int step) override;
    void setControls(float cutoffHz, float resonance, float drive, float character, float tolerance) override
    {
        setLaneControls(0, cutoffHz, resonance, drive, character, tolerance);
    }
    void setLaneControls(int lane, float cutoffHz, float resonance, float drive, float character, float tolerance) override;
    void process(float* io, int n, const float* cutoffOctMod) override;
    void processQuad(float* const io[4], int n, const float* const cutoffOctMod[4]) override;
    int lanes() const override { return laneCount; }
    float latency() const override { return latencySamples; }

    void responseDb(const float* freqHz, int n, float* outDb) const override;

    // The pitch-correction table is measured on the model itself; this switches it off (used by the measurement).
    void disablePitchCorrection()
    {
        pitchOff = true;
        table = nullptr;
        tableHigh = nullptr;
    }

protected:
    static constexpr int kChunk = 16; // control rate in host-rate samples
    // Control rate while a per-sample cutoff modulation is passed in: the modulation is sampled every kFmChunk samples
    // (12 kHz at 48 kHz) and the gains ramp between those points.
    static constexpr int kFmChunk = 4;
    static constexpr int kMaxStages = 2; // log2 of the highest filter oversampling (High: 4x)
    static constexpr int kModelIds = 8;

    // Kernels: n oversampled samples; in and out may be the same array. run1 reads lane 0 with stride 1, run4 reads
    // four lanes interleaved.
    virtual void run1(const float* in, float* out, int n, const LaneBlock& b) = 0;
    virtual void run4(const float* in, float* out, int n, const LaneBlock& b) = 0;
    virtual void resetState() = 0;
    // The same for one lane of the kernel state; the other lanes are left alone.
    virtual void resetLaneState(int lane) = 0;
    // Called when the control grid restarts (new controls were set): the next sample begins a control chunk.
    virtual void onRestart() {}
    // Multiplies the prewarped integrator gain so the dial frequency lands on the self-oscillation frequency.
    virtual float gScale() const { return 1.0f; }
    // 0 .. kModelIds - 1, indexes the cache of the pitch-correction tables.
    virtual int modelId() const = 0;
    // False for the models that never oscillate (state-variable filter, passive high-pass): they get no pitch table.
    virtual bool selfOscillates() const { return true; }
    // True when the limit cycle carries strong harmonics and its frequency depends on `character` (the Sallen-Key's
    // clipper): the pitch table then reads the frequency from zero crossings and is measured at character 0 and 1.
    virtual bool distortedOscillation() const { return false; }
    // True when setHighpassStep() puts the shared one-pole high-pass in front of the model (the OTA cascade has its own).
    virtual bool supportsHighpass() const { return false; }
    // Called after prepare(): the oversampled rate and the tier are known.
    virtual void onPrepared() {}
    // Small-signal transfer of the kernel (input of the model to output, nominal parts) at the normalised frequency p = j w,
    // w = tan(pi f / rate) over the integrator gain; k is the model's loop quantity (feedbackForResonance, limited below
    // self-oscillation) and chr the character.
    virtual std::complex<double> kernelGain(std::complex<double> p, double k, double chr) const = 0;
    // Corner of the one-pole high-pass ahead of the kernel in Hz, 0 = none.
    virtual float frontHighpassHz() const;

    double hostRate = 48000.0;
    double osRate = 48000.0;
    int stages = 1;
    int osFactor = 2;
    SolverTier tier = SolverTier::Standard;
    Quality quality = Quality::Standard;
    int laneCount = 1;
    int maxBlockSize = 512;

private:
    // Polyphase all-pass half-band stages (4 lanes wide; the scalar path uses lane 0), see Oversampler.cpp for the design.
    struct HalfBand
    {
        alignas(16) float x[8][4]{}, y[8][4]{};
        alignas(16) float prev[4]{};
    };
    // Measured self-oscillation at full resonance: the cutoff that was set and the frequency that came out, both log2
    // of the ratio to the sample rate. Read backwards it gives the cutoff to set for a wanted frequency.
    struct PitchTable
    {
        static constexpr int kPoints = 28;
        std::array<float, kPoints> logSet{}, logOsc{};
    };
    struct Control
    {
        float cutoffHz = 1000.0f, resonance = 0.0f, drive = 0.0f, character = 0.0f, tolerance = 0.0f;
    };

    static const PitchTable* pitchTableFor(const LadderBase& prototype, Quality quality, int anchor);
    static PitchTable measurePitchTable(const LadderBase& prototype, Quality quality, float character);
    float pitchRatio(float cutoffHz, float resonance, float character) const;
    float tableOffset(const PitchTable& t, float cutoffHz) const;

    template <class V> void upAt(size_t stage, const float* in, int n, float* out);
    template <class V> void downAt(size_t stage, const float* in, int n, float* out);
    template <class V, int Coefficients> void upStage(HalfBand& st, const float* in, int n, float* out);
    template <class V, int Coefficients> void downStage(HalfBand& st, const float* in, int n, float* out);
    template <class V> void runChunk(float* data, int m, bool identity);
    void measureLatency();
    void applyHighpass();
    template <class V> void highpassRun(float* data, int n);

    void updateTargets(int lane, const float* mod, int samplesLeft, int length);
    void drawTolerances();
    void drawLane(int lane);

    std::array<HalfBand, kMaxStages + 1> upState{}, downState{};
    std::array<float, 8> coefficient{};
    int coefficients = 6;
    float latencySamples = 0.0f;
    std::array<Control, 4> ctl{};
    LaneBlock blk;
    alignas(16) float shaperA[4]{}, shaperScale[4]{}; // drive shaper tanh(a x) * scale per lane
    bool shaperActive = false;
    std::array<std::array<float, 6>, 4> rnd{}; // seeded draws per lane, in [-1, 1)
    std::array<nl::SplitMix, 4> noise{};
    std::array<bool, 4> snap{true, true, true, true};
    std::array<bool, 4> dirty{true, true, true, true}; // controls changed since the targets were last computed
    alignas(16) float hpState[4]{};
    float hpCoefficient = 0.0f; // g / (1 + g) of the one-pole high-pass at the filter rate, 0 = off
    int hpStep = 0;
    const PitchTable* table = nullptr;
    const PitchTable* tableHigh = nullptr; // measured at character 1, for the models whose pitch depends on it
    bool pitchOff = false;
    std::uint32_t seedValue = 1;
    std::array<std::uint32_t, 4> laneSeed{1U, 1U, 1U, 1U};
    std::array<int, 4> laneDraw{0, 1, 2, 3};
    bool noiseOn = false;
    bool restart = true;
    std::array<int, 4> laneAge{kChunk, kChunk, kChunk, kChunk}; // host-rate samples since a lane last retargeted (quad)
    int phase = 0;       // host-rate samples already used up in the current control chunk
    int ctlLen = kChunk; // length of the current control chunk (kChunk, or kFmChunk while the cutoff is modulated)
};

} // namespace crucible::filters
