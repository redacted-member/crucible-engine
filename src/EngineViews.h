// SPDX-License-Identifier: MIT
#pragma once

// The built-in views of an engine (see crucible/Views.h and docs/ARCHITECTURE.md, "Views"): friend of EngineCore and
// SynthEngine so that the producers can read what the old accessors read.

#include <crucible/EngineCore.h>

#include <string>

namespace crucible
{
struct EngineViews
{
    enum Op
    {
        Meter,
        Loudness,
        Scope,
        Spectrum,
        SoundingNotes,
        SequencerStep,
        SequencerRunning,
        ClipPositions,
        VectorPosition,
        ActiveVoices,
        ParamModulated,
        ParamVoice,
        ParamBand,
        ParamVoiceN,
        SamplePlayheads,
        SampleGrains,
        SampleZonesActive,
        SampleInfo,
        SamplePeaks,
        SampleSlices,
        SampleZones,
        CloudPoints,
        CloudEnergy,
        CloudState,
        GainReduction,
        ModuleLoudness,
        ModuleLevels,
        ModuleResponse,
        ModuleSpectrum,
        FilterResponse,
        OscWavetable,
        ModuleCurve
    };

    static constexpr int kZoneFloats = 12; // floats per point of a sample_zones view
    static constexpr int kCloudPointFloats = 6;
    static constexpr int kWavetablePoints = 256;      // points per frame of an oscN/wavetable view
    static constexpr int kFilterResponsePoints = 256; // points of a filter response view, 20 Hz to 20 kHz

    static void install(EngineCore& core);
    static void produce(void* context, const ViewArgs& args, ViewWriter& out);
    static int resolve(void* context, ViewHub& hub, const std::string& instance, const std::string& id);
};
} // namespace crucible
