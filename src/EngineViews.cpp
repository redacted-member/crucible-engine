// SPDX-License-Identifier: MIT
#include "EngineViews.h"

#include <crucible/Cloud.h>
#include <crucible/Filters.h>
#include <crucible/UserWavetables.h>
#include <crucible/Wavetables.h>
#include <crucible/module/MeterReadout.h>
#include <crucible/module/ShaperCurve.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

namespace crucible
{
namespace
{
constexpr int kScope = MasterAnalyzer::scopeSize;

// "<param>.voice@<n>" -> parameter id and voice, or false.
bool parseChosenVoice(const std::string& id, std::string& param, int& voice)
{
    const std::string tag = ".voice@";
    const auto at = id.rfind(tag);
    if (at == std::string::npos || at == 0)
        return false;
    const std::string digits = id.substr(at + tag.size());
    if (digits.empty() || digits.size() > 3 || digits.find_first_not_of("0123456789") != std::string::npos)
        return false;
    param = id.substr(0, at);
    voice = std::atoi(digits.c_str());
    return true;
}

void pushMask(ViewWriter& out, const std::array<std::uint64_t, 2>& bits)
{
    for (const auto word : bits)
    {
        out.pushWord(static_cast<std::uint32_t>(word & 0xffffffffU));
        out.pushWord(static_cast<std::uint32_t>(word >> 32));
    }
}
} // namespace

void EngineViews::install(EngineCore& core)
{
    ViewHub& hub = core.viewHub;
    SynthEngine& synth = core.synth;
    const ParameterModel& model = synth.model();

    const auto add = [&](const std::string& instance, const std::string& id, const char* description, ViewKind kind,
                         int maxElements, int stride, bool live, Op op, int index = 0, int extra = 0, int dependsOn = -1)
    {
        ViewHub::Spec spec;
        spec.info = {instance, id, description, kind, maxElements, stride, live};
        spec.produce = &EngineViews::produce;
        spec.context = &core;
        spec.args = {op, index, extra};
        spec.dependsOn = dependsOn;
        return hub.add(std::move(spec));
    };

    add("master", "meter", "peak left dB, peak right dB, clip (0 / 1); OutputMeter::snapshot()", ViewKind::Vector, 3, 1,
        true, Meter);
    add("master", "loudness",
        "momentary LUFS, short-term LUFS, true peak dB, max true peak dB, correlation; MasterAnalyzer::loudness()",
        ViewKind::Vector, 5, 1, true, Loudness);
    core.viewScope = add("master", "scope", "the last 4096 samples, left channel then right channel, oldest first",
                         ViewKind::Vector, 2 * kScope, 1, true, Scope);
    for (const int size : {256, 512, 1024, 2048, 4096})
        add("master", "spectrum_" + std::to_string(size), "magnitude spectrum in dB, size / 2 + 1 bins from 0 Hz to Nyquist",
            ViewKind::Vector, size / 2 + 1, 1, false, Spectrum, size, 0, core.viewScope);
    add("engine", "sounding_notes", "bit mask of the notes sounding (word k holds notes 32k..32k+31)", ViewKind::Mask, 4, 1,
        true, SoundingNotes);
    add("engine", "active_voices", "voices playing", ViewKind::Scalar, 1, 1, true, ActiveVoices);
    add("engine", "vector_position", "vector field puck x, y (0..1) and whether the field is active (0 / 1)",
        ViewKind::Vector, 3, 1, true, VectorPosition);
    add("sequencer", "step", "current step of the step sequencer", ViewKind::Scalar, 1, 1, true, SequencerStep);
    add("sequencer", "running", "1 while the step sequencer plays", ViewKind::Scalar, 1, 1, true, SequencerRunning);
    add("clips", "positions", "playhead of every clip in clip units (kMaxClips values)", ViewKind::Vector, kMaxClips, 1,
        true, ClipPositions);

    // Per-parameter modulated values. "<id>.voice@<n>" (the value under voice n) is made on demand by the resolver.
    const auto& params = model.params();
    for (size_t i = 0; i < params.size(); ++i)
    {
        const int p = static_cast<int>(i);
        add("params", params[i].id + ".mod", "value after global modulation", ViewKind::Scalar, 1, 1, true, ParamModulated,
            p);
        add("params", params[i].id + ".voice", "value under the newest voice's modulation", ViewKind::Scalar, 1, 1, true,
            ParamVoice, p);
        add("params", params[i].id + ".band", "lowest and highest value across the active voices, and how many are active",
            ViewKind::Vector, 3, 1, true, ParamBand, p);
    }

    // Sample, granular and cloud oscillator slots.
    const int cloudType = [&]
    {
        const auto types = synth.reg->oscillatorTypes();
        for (size_t t = 0; t < types.size(); ++t)
            if (types[t] != nullptr && types[t]->id == "osc.cloud")
                return static_cast<int>(t);
        return -1;
    }();
    const int voiceCount = static_cast<int>(synth.voices.size());
    synth.viewVisual.playheads.reserve(static_cast<size_t>(voiceCount));
    synth.viewVisual.grains.reserve(static_cast<size_t>(voiceCount) * kOscVisualMaxGrains);
    synth.viewVisual.zones.reserve(static_cast<size_t>(voiceCount) * kMaxZoneLayers);
    synth.viewEnergy.reserve(kCloudMaxPoints);
    for (int slot = 0; slot < synth.numOsc && slot < 8; ++slot)
    {
        const std::string osc = "osc" + std::to_string(slot + 1);
        add(osc, "wavetable",
            "the table the slot selects (factory or user): the frame count, then 64 frames of 256 points (every 8th sample "
            "of the 2048-sample base cycle), frame-major; empty for an empty user slot",
            ViewKind::Vector, 1 + WavetableBank::numFrames * kWavetablePoints, 1, false, OscWavetable, slot);
        add(osc, "sample_info", "frames, sample rate, detected note (-1: none); empty without a sample", ViewKind::Vector, 3,
            1, false, SampleInfo, slot);
        add(osc, "sample_peaks", "waveform overview: kSamplePeakColumns minima then as many maxima", ViewKind::Vector,
            2 * kSamplePeakColumns, 1, false, SamplePeaks, slot);
        add(osc, "sample_slices", "slice markers, 0..1", ViewKind::Vector, kMaxSlices, 1, false, SampleSlices, slot);
        add(osc, "sample_zones",
            "multisample zones: sample id, key lo, key hi, velocity lo, velocity hi, root, gain dB, tune cents, loop start, "
            "loop "
            "end, loop mode, group",
            ViewKind::Points, kMaxZones * kZoneFloats, kZoneFloats, false, SampleZones, slot);
        add(osc, "sample_playheads", "playheads (0..1) of the voices playing the sample", ViewKind::Vector, voiceCount, 1,
            true, SamplePlayheads, slot);
        add(osc, "sample_grains", "grains of all voices: position, width, amplitude, pan", ViewKind::Points,
            voiceCount * kOscVisualMaxGrains * 4, 4, true, SampleGrains, slot);
        add(osc, "sample_zones_active", "indices of the zones sounding in the active voices", ViewKind::Vector,
            voiceCount * kMaxZoneLayers, 1, true, SampleZonesActive, slot);
        add(osc, "cloud_points", "cloud model points: x, y, z, phase, pan, random", ViewKind::Points,
            kCloudMaxPoints * kCloudPointFloats, kCloudPointFloats, false, CloudPoints, slot);
        add(osc, "cloud_energy", "energy (0..1) of each cloud point, the loudest active voice's", ViewKind::Vector,
            kCloudMaxPoints, 1, true, CloudEnergy, slot, cloudType);
        add(osc, "cloud_state", "morph position and number of active cloud notes", ViewKind::Vector, 2, 1, true, CloudState,
            slot, cloudType);
    }

    // Gain reduction of the dynamics modules (dB, positive: turned down), one scalar per module that exposes the meter.
    for (const auto* module : synth.reg->ofKind(ModuleKind::Effect))
    {
        if (std::find(module->capabilities.begin(), module->capabilities.end(), "gain_reduction") ==
            module->capabilities.end())
            continue;
        const int slot = synth.fx.slotOf(module->id);
        if (slot < 0 || dynamic_cast<GainReductionMeter*>(synth.fx.module(slot)) == nullptr)
            continue;
        add(module->id, "gain_reduction", "gain reduction in dB of the loudest channel (positive: turned down)",
            ViewKind::Scalar, 1, 1, true, GainReduction, slot);
    }

    // Loudness and level readouts of the metering modules (see crucible/module/MeterReadout.h).
    for (const auto* module : synth.reg->ofKind(ModuleKind::Effect))
    {
        const auto has = [&](const char* capability)
        {
            return std::find(module->capabilities.begin(), module->capabilities.end(), capability) !=
                   module->capabilities.end();
        };
        const int slot = synth.fx.slotOf(module->id);
        auto* readout = slot >= 0 ? dynamic_cast<MeterReadout*>(synth.fx.module(slot)) : nullptr;
        if (readout == nullptr)
            continue;
        if (has("loudness"))
            add(module->id, "loudness",
                "momentary LUFS, short-term LUFS, integrated LUFS, loudness range LU, peak dB (true peak when the module "
                "measures it), "
                "largest momentary LUFS, range low and high LUFS; -120 means nothing yet",
                ViewKind::Vector, MeterReadout::kLoudnessValues, 1, true, ModuleLoudness, slot);
        if (has("levels"))
            add(module->id, "levels",
                "sample peak dB left and right, true peak dB left and right (decaying), RMS dB left and right, PPM dB left "
                "and right, VU "
                "dB left and right, correlation",
                ViewKind::Vector, MeterReadout::kLevelValues, 1, true, ModuleLevels, slot);
    }

    // Frequency response and output spectrum of the equalisers (modules implementing ResponseView).
    for (const auto* module : synth.reg->ofKind(ModuleKind::Effect))
    {
        if (std::find(module->capabilities.begin(), module->capabilities.end(), "response_view") ==
            module->capabilities.end())
            continue;
        const int slot = synth.fx.slotOf(module->id);
        auto* view = slot < 0 ? nullptr : dynamic_cast<ResponseView*>(synth.fx.module(slot));
        if (view == nullptr)
            continue;
        add(module->id, "response", "magnitude response in dB at points spaced logarithmically from 20 Hz to 20 kHz",
            ViewKind::Vector, view->responsePoints(), 1, true, ModuleResponse, slot);
        if (view->spectrumBins() > 0)
            add(module->id, "spectrum", "output magnitude spectrum in dB, bins from 0 Hz to Nyquist", ViewKind::Vector,
                view->spectrumBins(), 1, true, ModuleSpectrum, slot);
    }

    // Transfer curve of the waveshapers (modules implementing ShaperCurve).
    for (const auto* module : synth.reg->ofKind(ModuleKind::Effect))
    {
        if (std::find(module->capabilities.begin(), module->capabilities.end(), "shaper_curve") ==
            module->capabilities.end())
            continue;
        const int slot = synth.fx.slotOf(module->id);
        if (slot < 0 || dynamic_cast<ShaperCurve*>(synth.fx.module(slot)) == nullptr)
            continue;
        add(module->id, "curve",
            "transfer curve at 257 points from x = -1 to +1 (the drawn curve, without drive), as set by crucible_set_shaper_curve",
            ViewKind::Vector, ShaperCurve::kViewPoints, 1, false, ModuleCurve, slot);
    }

    // Magnitude response of the voice filters: "filter" for the first, "filter2", "filter3"... for the others (the parameter
    // prefixes). "response" is computed from the parameter values, "response_mod" from the values after modulation (the
    // ".mod" values); both are pulled and cost nothing until read.
    for (int f = 0; f < synth.numFilt; ++f)
    {
        const std::string instance = f == 0 ? "filter" : "filter" + std::to_string(f + 1);
        add(instance, "response",
            "small-signal magnitude response in dB at 256 points spaced logarithmically from 20 Hz to 20 kHz, from the "
            "type, "
            "cutoff, resonance, drive, character and high-pass parameters",
            ViewKind::Vector, kFilterResponsePoints, 1, false, FilterResponse, f, 0);
        add(instance, "response_mod", "the same from the parameter values after modulation (the \".mod\" values)",
            ViewKind::Vector, kFilterResponsePoints, 1, false, FilterResponse, f, 1);
    }

    hub.setResolver(&EngineViews::resolve, &core);
}

int EngineViews::resolve(void* context, ViewHub& hub, const std::string& instance, const std::string& id)
{
    auto& core = *static_cast<EngineCore*>(context);
    std::string param;
    int voice = 0;
    if (instance != "params" || !parseChosenVoice(id, param, voice))
        return -1;
    const int p = core.synth.model().indexOf(param);
    if (p < 0 || voice >= static_cast<int>(core.synth.voices.size()))
        return -1;
    ViewHub::Spec spec;
    spec.info = {instance,
                 id,
                 "value under the modulation of one voice (the unmodulated one when it is not playing)",
                 ViewKind::Scalar,
                 1,
                 1,
                 true};
    spec.produce = &EngineViews::produce;
    spec.context = context;
    spec.args = {ParamVoiceN, p, voice};
    return hub.add(std::move(spec));
}

void EngineViews::produce(void* context, const ViewArgs& args, ViewWriter& out)
{
    EngineCore& core = *static_cast<EngineCore*>(context);
    SynthEngine& synth = core.synth;
    const int index = args.index;
    switch (static_cast<Op>(args.op))
    {
    case Meter:
    {
        const auto s = core.outputMeter.snapshot();
        out.push(s.peakLeftDb);
        out.push(s.peakRightDb);
        out.push(s.clip ? 1.0f : 0.0f);
        break;
    }
    case Loudness:
    {
        const auto l = core.masterAnalyzer.loudness();
        out.push(l.momentaryLufs);
        out.push(l.shortTermLufs);
        out.push(l.truePeakDb);
        out.push(l.maxTruePeakDb);
        out.push(l.correlation);
        break;
    }
    case Scope:
        core.masterAnalyzer.copyScope(out.data(), out.data() + kScope, kScope);
        out.setSize(2 * kScope);
        break;
    case Spectrum:
    {
        std::vector<float> both(static_cast<size_t>(2 * kScope));
        if (core.viewHub.read(core.viewScope, both.data(), 2 * kScope, nullptr) != 2 * kScope)
            break; // the scope has not been published: nothing to show
        const int length = MasterAnalyzer::fftLength(index);
        const auto bins =
            MasterAnalyzer::spectrumOf(both.data() + (kScope - length), both.data() + kScope + (kScope - length), length);
        out.copy(bins.data(), static_cast<int>(bins.size()));
        break;
    }
    case SoundingNotes:
    {
        std::array<std::uint64_t, 2> bits{};
        core.soundingNotes(bits);
        pushMask(out, bits);
        break;
    }
    case SequencerStep:
        out.scalar(static_cast<float>(core.sequencerStep()));
        break;
    case SequencerRunning:
        out.scalar(core.sequencerRunning() ? 1.0f : 0.0f);
        break;
    case ClipPositions:
    {
        std::array<double, kMaxClips> positions{};
        synth.clipPositions(positions);
        for (const double p : positions)
            out.push(static_cast<float>(p));
        break;
    }
    case VectorPosition:
    {
        float x = 0.0f, y = 0.0f;
        synth.vectorPosition(x, y);
        out.push(x);
        out.push(y);
        out.push(synth.vectorActive() ? 1.0f : 0.0f);
        break;
    }
    case GainReduction:
    {
        auto* meter = dynamic_cast<GainReductionMeter*>(synth.fx.module(index));
        out.scalar(meter != nullptr ? meter->gainReductionDb(0) : 0.0f);
        break;
    }
    case ModuleLoudness:
    case ModuleLevels:
    {
        auto* readout = dynamic_cast<MeterReadout*>(synth.fx.module(index));
        if (readout == nullptr)
            break;
        float values[16] = {};
        if (static_cast<Op>(args.op) == ModuleLoudness)
        {
            readout->loudness(values);
            out.copy(values, MeterReadout::kLoudnessValues);
        }
        else if (readout->levels(values))
            out.copy(values, MeterReadout::kLevelValues);
        break;
    }
    case ModuleResponse:
    {
        if (auto* view = dynamic_cast<ResponseView*>(synth.fx.module(index)))
        {
            view->fillResponse(out.data());
            out.setSize(view->responsePoints());
        }
        break;
    }
    case ModuleCurve:
    {
        if (auto* shaper = dynamic_cast<ShaperCurve*>(synth.fx.module(index)))
        {
            shaper->fillCurveView(out.data());
            out.setSize(ShaperCurve::kViewPoints);
        }
        break;
    }
    case ModuleSpectrum:
    {
        if (auto* view = dynamic_cast<ResponseView*>(synth.fx.module(index)))
        {
            view->fillSpectrum(out.data());
            out.setSize(view->spectrumBins());
        }
        break;
    }
    case FilterResponse:
    {
        static const std::array<float, kFilterResponsePoints> grid = []
        {
            std::array<float, kFilterResponsePoints> hz{};
            for (int i = 0; i < kFilterResponsePoints; ++i)
                hz[static_cast<size_t>(i)] =
                    static_cast<float>(20.0 * std::pow(1000.0, static_cast<double>(i) / (kFilterResponsePoints - 1)));
            return hz;
        }();
        const auto& ix = synth.idx.filter[static_cast<size_t>(index)];
        const bool modulated = args.extra != 0;
        const auto value = [&](int p, float fallback)
        { return p < 0 ? fallback : (modulated ? synth.modulatedValue(p) : synth.baseValue(p)); };
        FilterSettings fs;
        fs.model = static_cast<int>(value(ix.type, 0.0f));
        fs.cutoffHz = value(ix.cutoff, 8000.0f);
        fs.resonance = value(ix.res, 0.0f);
        fs.character = value(ix.character, 0.0f);
        fs.highpass = static_cast<int>(value(ix.highpass, 0.0f));
        FormantTable formant;
        if (synth.formantTable(index, formant))
            fs.formants = &formant;
        filterResponseDb(fs, value(ix.drive, 0.0f), synth.sampleRate, synth.cfg.quality, grid.data(), kFilterResponsePoints,
                         out.data());
        out.setSize(kFilterResponsePoints);
        break;
    }
    case OscWavetable:
    {
        constexpr int step = WavetableBank::tableSize / kWavetablePoints;
        const int table = static_cast<int>(synth.baseValue(synth.idx.osc[static_cast<size_t>(index)].table));
        const auto emit = [&](const auto& frameAt)
        {
            out.push(static_cast<float>(WavetableBank::numFrames));
            for (int f = 0; f < WavetableBank::numFrames; ++f)
            {
                const float* frame = frameAt(f);
                for (int i = 0; i < kWavetablePoints; ++i)
                    out.push(frame[i * step]);
            }
        };
        if (table >= 0 && table < WavetableBank::numTables)
        {
            const auto& bank = WavetableBank::instance();
            if (bank.resident(table))
                emit([&](int f) { return bank.frame(table, 0, f); });
        }
        else if (table >= WavetableBank::numTables && table < kTotalTables)
            synth.userWavetables().withTable(
                table - WavetableBank::numTables, [&](const UserWavetable& t)
                { emit([&](int f) { return t.baseFrames().data() + f * WavetableBank::tableSize; }); });
        break;
    }
    case ActiveVoices:
        out.scalar(static_cast<float>(synth.activeVoiceCount()));
        break;
    case ParamModulated:
        out.scalar(synth.modulatedValue(index));
        break;
    case ParamVoice:
        out.scalar(synth.voiceModulatedValue(index));
        break;
    case ParamBand:
    case ParamVoiceN:
    {
        // Per-voice modulation of the parameter: a voice target (ParameterMap::voiceTargets()) or a fixed matrix
        // destination, exactly as voiceModulatedValue() picks it.
        const float base = synth.modulatedValue(index);
        const int target = index >= 0 && static_cast<size_t>(index) < synth.paramDest.size()
                               ? synth.params_->voiceTargetOfParam(index)
                               : -1;
        const int dest = target >= 0 || index < 0 || static_cast<size_t>(index) >= synth.paramDest.size()
                             ? 0
                             : synth.paramDest[static_cast<size_t>(index)];
        const bool perVoice = target >= 0 || (dest > 0 && dest < 64);
        const auto valueOf = [&](const Voice& v)
        {
            return target >= 0 ? synth.applyVoiceMod(index, base, 0, v.lastTargetMod()[static_cast<size_t>(target)])
                               : synth.applyVoiceMod(index, base, dest, v.lastDestMod()[static_cast<size_t>(dest)]);
        };
        if (static_cast<Op>(args.op) == ParamVoiceN)
        {
            const auto v = static_cast<size_t>(args.extra);
            out.scalar(perVoice && v < synth.voices.size() && synth.voices[v].isActive() ? valueOf(synth.voices[v]) : base);
            break;
        }
        float lo = base, hi = base;
        int active = 0;
        bool first = true;
        for (const auto& v : synth.voices)
        {
            if (!v.isActive())
                continue;
            ++active;
            if (!perVoice)
                continue;
            const float value = valueOf(v);
            lo = first ? value : std::min(lo, value);
            hi = first ? value : std::max(hi, value);
            first = false;
        }
        out.push(lo);
        out.push(hi);
        out.push(static_cast<float>(active));
        break;
    }
    case SampleInfo:
    case SamplePeaks:
    case SampleSlices:
    case SampleZones:
    {
        SynthEngine::SampleView view;
        if (!synth.sampleSnapshot(index, view))
            break;
        if (static_cast<Op>(args.op) == SampleInfo)
        {
            out.push(static_cast<float>(view.frames));
            out.push(static_cast<float>(view.sampleRate));
            out.push(view.detectedNote);
        }
        else if (static_cast<Op>(args.op) == SamplePeaks)
        {
            for (const float v : view.peakMin)
                out.push(v);
            for (const float v : view.peakMax)
                out.push(v);
        }
        else if (static_cast<Op>(args.op) == SampleSlices)
        {
            for (const float v : view.slices)
                out.push(v);
        }
        else
        {
            for (const Zone& z : view.zones)
            {
                if (out.size() + kZoneFloats > out.capacity())
                    break;
                out.push(static_cast<float>(z.sampleId));
                out.push(static_cast<float>(z.keyLo));
                out.push(static_cast<float>(z.keyHi));
                out.push(static_cast<float>(z.velLo));
                out.push(static_cast<float>(z.velHi));
                out.push(z.root);
                out.push(z.gainDb);
                out.push(z.tuneCents);
                out.push(z.loopStart);
                out.push(z.loopEnd);
                out.push(static_cast<float>(z.loopMode));
                out.push(static_cast<float>(z.group));
            }
        }
        break;
    }
    case SamplePlayheads:
    case SampleGrains:
    case SampleZonesActive:
    {
        OscVisual& visual = synth.viewVisual;
        visual.playheads.clear();
        visual.grains.clear();
        visual.zones.clear();
        for (const auto& v : synth.voices)
            if (v.isActive())
                if (const auto* inst = v.oscillatorInstance(index, 1))
                    inst->visual(visual);
        if (static_cast<Op>(args.op) == SamplePlayheads)
            for (const float p : visual.playheads)
                out.push(p);
        else if (static_cast<Op>(args.op) == SampleGrains)
            for (const auto& g : visual.grains)
            {
                if (out.size() + 4 > out.capacity())
                    break;
                out.push(g.position);
                out.push(g.width);
                out.push(g.amplitude);
                out.push(g.pan);
            }
        else
            for (const int z : visual.zones)
                out.push(static_cast<float>(z));
        break;
    }
    case CloudPoints:
    {
        synth.clouds.withModel(index,
                               [&](const CloudModel& m)
                               {
                                   for (const auto& p : m.points)
                                   {
                                       if (out.size() + kCloudPointFloats > out.capacity())
                                           break;
                                       out.push(p.x);
                                       out.push(p.y);
                                       out.push(p.z);
                                       out.push(p.phase);
                                       out.push(p.pan);
                                       out.push(p.random);
                                   }
                               });
        break;
    }
    case CloudEnergy:
    case CloudState:
    {
        CloudBank::Pinned pinned{};
        synth.clouds.acquire(pinned);
        const CloudModel* m = pinned[static_cast<size_t>(index)];
        if (m != nullptr)
        {
            const size_t points = std::min(m->points.size(), static_cast<size_t>(kCloudMaxPoints));
            float* energy = out.data();
            if (static_cast<Op>(args.op) == CloudEnergy)
                std::fill(energy, energy + points, 0.0f);
            float position = 0.0f;
            int notes = 0;
            for (const auto& v : synth.voices)
            {
                if (!v.isActive())
                    continue;
                auto* view = dynamic_cast<CloudOscillatorView*>(v.oscillatorInstance(index, args.extra));
                if (view == nullptr)
                    continue;
                auto& osc = view->cloudOscillator();
                if (!osc.isActive())
                    continue;
                if (static_cast<Op>(args.op) == CloudEnergy)
                {
                    osc.energies(synth.viewEnergy, points);
                    for (size_t i = 0; i < points; ++i)
                        energy[i] = std::max(energy[i], synth.viewEnergy[i]);
                }
                position = osc.lastMorphPosition();
                ++notes;
            }
            if (static_cast<Op>(args.op) == CloudEnergy)
                out.setSize(static_cast<int>(points));
            else
            {
                out.push(position);
                out.push(static_cast<float>(notes));
            }
        }
        synth.clouds.release();
        break;
    }
    }
}

} // namespace crucible
