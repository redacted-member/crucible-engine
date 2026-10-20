// SPDX-License-Identifier: MIT
#include <crucible/Engine.h>
#include <crucible/Spectral.h>
#include <crucible/fx/IrImport.h>
#include <crucible/fx/LinearPhase.h>
#include <crucible/graph/Compiler.h>
#include <crucible/module/ConvolutionIr.h>
#include <crucible/module/ShaperCurve.h>

#include <crucible/Denormals.h>
#include <crucible/Wavetables.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <optional>

namespace crucible
{
namespace
{
bool isTableParam(const ParamDef& d)
{
    static const std::string suffix = "_table";
    return d.id.size() > suffix.size() && d.id.rfind("osc", 0) == 0 &&
           d.id.compare(d.id.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Slot (0-based) of an oscillator type parameter ("osc2_type" is 1), or -1.
int oscillatorTypeSlot(const ParamDef& d)
{
    const std::string& id = d.id;
    if (id.size() < 9 || id.rfind("osc", 0) != 0 || id.compare(id.size() - 5, 5, "_type") != 0)
        return -1;
    int slot = 0;
    for (size_t i = 3; i + 5 < id.size(); ++i)
    {
        if (id[i] < '0' || id[i] > '9')
            return -1;
        slot = slot * 10 + (id[i] - '0');
    }
    return slot - 1;
}
} // namespace

std::unique_ptr<Engine> Engine::fromProduct(const std::string& json, std::string& error, EngineConfig base,
                                            const graph::NodeRegistry* nodes)
{
    // Nothing here throws to the caller: a product the compiler or the parameter model cannot hold (for example too many
    // per-voice modulation targets) is reported through `error`.
    try
    {
        graph::ProductDef def;
        if (!graph::productFromText(json, def, error))
            return nullptr;
        base.compiled = graph::compile(def, error, nodes != nullptr ? *nodes : graph::NodeRegistry::builtIn());
        if (!base.compiled)
            return nullptr;
        base.registry.reset();
        base.product.reset();
        return std::make_unique<Engine>(base);
    }
    catch (const std::exception& ex)
    {
        error = ex.what();
        return nullptr;
    }
}

Engine::Engine(const EngineConfig& config)
    : core(std::make_unique<EngineCore>(config)), sceneBank(&core->model()), ccStore(&core->model())
{
    seqLengthIndex = model().indexOf("seq_length");
    seqPatternIndex = model().indexOf("seq_pattern");
    maxBlock = core->engine().config().maxBlockSize;
    core->attachScenes(&sceneBank);
    // A scene can select an oscillator type the host parameters do not hold; the vector pad then switches to it on the
    // audio thread, so its heavy state has to exist by then.
    sceneBank.setStoredHook([this](const std::vector<float>& values) { armSceneOscillators(values); });
    const auto n = static_cast<size_t>(model().size());
    params = std::make_unique<std::atomic<float>[]>(n);
    snapshot.assign(n, 0.0f);
    midiSlice.assign(kMidiSliceCapacity, MidiInput{});
    pending = std::make_unique<std::atomic<bool>[]>(n);
    needsService.assign(n, false);
    changedBy = std::make_unique<std::atomic<std::uint8_t>[]>(n);
    for (size_t i = 0; i < n; ++i)
        changedBy[i].store(0, std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i)
    {
        const auto& d = model().params()[i];
        needsService[i] = isTableParam(d) || oscillatorTypeSlot(d) >= 0 || static_cast<int>(i) == seqLengthIndex ||
                          static_cast<int>(i) == seqPatternIndex;
    }
    resetParamsToDefaults();
    anyChanged.store(false, std::memory_order_relaxed);
    for (size_t i = 0; i < n; ++i)
        changedBy[i].store(0, std::memory_order_relaxed);
    core->engine().effects().publishReported(getAllParams().data());
    reportedLatency = latencySamples();
    reportedTail = tailSeconds();
}

Engine::~Engine()
{
    sceneBank.setStoredHook({});
}

void Engine::armSceneOscillators(const std::vector<float>& values)
{
    for (int i = 0; i < model().size() && static_cast<size_t>(i) < values.size(); ++i)
    {
        if (isTableParam(info(i)))
        {
            WavetableBank::instance().ensure(static_cast<int>(values[static_cast<size_t>(i)]));
            spectral::FactoryTables::instance().ensure(static_cast<int>(values[static_cast<size_t>(i)]));
        }
        if (const int slot = oscillatorTypeSlot(info(i)); slot >= 0)
            core->engine().armOscillator(slot, static_cast<int>(values[static_cast<size_t>(i)]));
    }
}

void Engine::prepare(double sampleRate, int maxBlockSize)
{
    rate = sampleRate > 0.0 ? sampleRate : 48000.0;
    maxBlock = std::max(1, std::min(maxBlockSize, core->engine().config().maxBlockSize));
    core->prepare(rate);
    for (int i = 0; i < model().size(); ++i)
        if (isTableParam(info(i)))
        {
            const int table = static_cast<int>(params[static_cast<size_t>(i)].load(std::memory_order_relaxed));
            WavetableBank::instance().ensure(table);
            spectral::FactoryTables::instance().ensure(table);
        }
    // Oscillator types with heavy state build it for the types the slots are set to now (OscillatorInstance::arm).
    for (int i = 0; i < model().size(); ++i)
        if (const int slot = oscillatorTypeSlot(info(i)); slot >= 0)
            core->engine().armOscillator(slot,
                                         static_cast<int>(params[static_cast<size_t>(i)].load(std::memory_order_relaxed)));
    // prepare() rebuilt the heavy state: arm the types the stored scenes select as well.
    std::vector<float> values(static_cast<size_t>(model().size()));
    for (int slot = 0; slot < kNumScenes; ++slot)
        if (sceneBank.recall(slot, values))
            armSceneOscillators(values);
    core->engine().effects().adoptLatencyParams(getAllParams().data());
    core->engine().effects().publishReported(getAllParams().data());
}

const ParamDef& Engine::info(int index) const
{
    const auto& p = model().params();
    return p[static_cast<size_t>(std::clamp(index, 0, static_cast<int>(p.size()) - 1))];
}

// The value a parameter stores for a natural value: clamped to its range, and rounded to the nearest whole number for
// int, choice and bool parameters. setParam and setParamRt both use it, so a value means the same on every path.
static float storedValue(const ParamDef& d, float natural)
{
    const float v = std::clamp(natural, d.min, d.max);
    return d.type == ParamType::Float ? v : std::clamp(std::round(v), d.min, d.max);
}

bool Engine::setParam(int index, float natural)
{
    if (index < 0 || index >= model().size() || std::isnan(natural))
        return false; // a NaN would pass through std::clamp and poison every filter and smoother it reaches
    const auto& d = info(index);
    const float value = storedValue(d, natural);
    // The factory wavetables are generated on demand: do it here, on the caller's (control) thread, rather than on
    // the audio thread when the oscillator first reads the table.
    if (isTableParam(d))
    {
        WavetableBank::instance().ensure(static_cast<int>(value));
        spectral::FactoryTables::instance().ensure(static_cast<int>(value));
    }
    // Selecting an oscillator type builds its lazily allocated state first (control thread), so the audio thread finds it.
    if (const int slot = oscillatorTypeSlot(d); slot >= 0)
        core->engine().armOscillator(slot, static_cast<int>(value));
    storeParam(index, value, mainSource);
    syncSequencerParam(index, value);
    return true;
}

void Engine::markChanged(int index, int source)
{
    changedBy[static_cast<size_t>(index)].store(static_cast<std::uint8_t>(source + 1), std::memory_order_release);
    anyChanged.store(true, std::memory_order_release);
}

bool Engine::storeParam(int index, float value, int source)
{
    params[static_cast<size_t>(index)].store(value, std::memory_order_relaxed);
    markChanged(index, source);
    return true;
}

void Engine::syncSequencerParam(int index, float value)
{
    // seq_length sizes the selected pattern (the pattern owns its length); selecting another pattern reports that
    // pattern's length back, so the two stay in step for hosts that only use the parameters.
    const int lengthIndex = seqLengthIndex;
    const int patternIndex = seqPatternIndex;
    if (index == lengthIndex)
    {
        const int slot = std::clamp(
            static_cast<int>(params[static_cast<size_t>(patternIndex)].load(std::memory_order_relaxed)), 0, kSeqSlots - 1);
        const int bars = std::clamp(static_cast<int>(std::lround(value)), 1, kSeqMaxBars);
        if (core->store().snapshotPattern(slot).lengthBars != bars)
            core->store().edit([=](SeqBank& b) { b.pattern[static_cast<size_t>(slot)].lengthBars = bars; });
    }
    else if (index == patternIndex)
    {
        const int slot = std::clamp(static_cast<int>(value), 0, kSeqSlots - 1);
        storeParam(lengthIndex, static_cast<float>(core->store().snapshotPattern(slot).lengthBars),
                   static_cast<int>(ParamSource::Internal));
    }
}

bool Engine::setParamNormalisedRt(int index, float normalised01, int source)
{
    if (index < 0 || index >= model().size())
        return false;
    return setParamRt(index, crucible::fromNormalised(info(index), normalised01), source);
}

bool Engine::setParamRt(int index, float natural, int source)
{
    if (index < 0 || index >= model().size() || std::isnan(natural))
        return false;
    const auto& d = info(index);
    storeParam(index, storedValue(d, natural), std::clamp(source, 0, 2));
    if (needsService[static_cast<size_t>(index)])
    {
        pending[static_cast<size_t>(index)].store(true, std::memory_order_release);
        anyPending.store(true, std::memory_order_release);
    }
    return true;
}

bool Engine::servicePending() const
{
    return anyPending.load(std::memory_order_acquire);
}

void Engine::service()
{
    if constexpr (fx::LinearPhaseWorker::threadless())
        fx::LinearPhaseWorker::instance().pump(); // no background threads here: design work runs on this thread
    if (!anyPending.exchange(false, std::memory_order_acquire))
        return;
    for (int i = 0; i < model().size(); ++i)
    {
        if (!pending[static_cast<size_t>(i)].exchange(false, std::memory_order_acquire))
            continue;
        const auto& d = info(i);
        const float value = params[static_cast<size_t>(i)].load(std::memory_order_relaxed);
        if (isTableParam(d))
        {
            WavetableBank::instance().ensure(static_cast<int>(value));
            spectral::FactoryTables::instance().ensure(static_cast<int>(value));
        }
        if (const int slot = oscillatorTypeSlot(d); slot >= 0)
            core->engine().armOscillator(slot, static_cast<int>(value));
        syncSequencerParam(i, value);
    }
}

bool Engine::setParam(std::string_view id, float natural)
{
    return setParam(model().indexOf(id), natural);
}

bool Engine::setParamText(int index, const std::string& text)
{
    float value = 0.0f;
    return textToValue(index, text, value) && setParam(index, value);
}

bool Engine::setParamText(std::string_view id, const std::string& text)
{
    return setParamText(model().indexOf(id), text);
}

bool Engine::setParamNormalised(int index, float normalised01)
{
    if (index < 0 || index >= model().size())
        return false;
    return setParam(index, crucible::fromNormalised(info(index), normalised01));
}

float Engine::getParam(int index) const
{
    if (index < 0 || index >= model().size())
        return 0.0f;
    return params[static_cast<size_t>(index)].load(std::memory_order_relaxed);
}

float Engine::getParam(std::string_view id) const
{
    return getParam(model().indexOf(id));
}

void Engine::resetParamsToDefaults()
{
    const auto& p = model().params();
    for (size_t i = 0; i < p.size(); ++i)
        storeParam(static_cast<int>(i), p[i].defaultValue, static_cast<int>(ParamSource::Internal));
}

std::vector<float> Engine::getAllParams() const
{
    std::vector<float> v(static_cast<size_t>(model().size()));
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = params[i].load(std::memory_order_relaxed);
    return v;
}

void Engine::setAllParams(const std::vector<float>& naturalValues)
{
    const int n = std::min(model().size(), static_cast<int>(naturalValues.size()));
    for (int i = 0; i < n; ++i)
        setParam(i, naturalValues[static_cast<size_t>(i)]);
}

bool Engine::push(ExternalEvent::Type type, int note, float value, int channel)
{
    if (!std::isfinite(value))
        return false;
    ExternalEvent e;
    e.type = type;
    e.channel = static_cast<std::uint8_t>(std::clamp(channel, 0, 15));
    e.note = static_cast<std::uint8_t>(std::clamp(note, 0, 127));
    e.value = value;
    return core->pushExternal(e);
}

bool Engine::noteOn(int note, float velocity)
{
    if (note < 0 || note > 127)
        return false;
    if (velocity <= 0.0f)
        return noteOff(note);
    return push(ExternalEvent::Type::NoteOn, note, std::min(velocity, 1.0f));
}

bool Engine::noteOff(int note)
{
    if (note < 0 || note > 127)
        return false;
    return push(ExternalEvent::Type::NoteOff, note, 0.0f);
}

bool Engine::pitchBend(float normalised)
{
    return push(ExternalEvent::Type::PitchBend, 0, std::clamp(normalised, -1.0f, 1.0f));
}

bool Engine::modWheel(float amount01)
{
    return push(ExternalEvent::Type::ModWheel, 0, std::clamp(amount01, 0.0f, 1.0f));
}

bool Engine::aftertouch(float amount01)
{
    return push(ExternalEvent::Type::Aftertouch, 0, std::clamp(amount01, 0.0f, 1.0f));
}

bool Engine::sustain(bool down)
{
    return push(ExternalEvent::Type::Sustain, 0, down ? 1.0f : 0.0f);
}

bool Engine::allNotesOff()
{
    return push(ExternalEvent::Type::AllNotesOff, 0, 0.0f);
}

bool Engine::killAllVoices()
{
    return push(ExternalEvent::Type::KillAll, 0, 0.0f);
}

bool Engine::midi(std::uint8_t status, std::uint8_t data1, std::uint8_t data2)
{
    const std::uint8_t type = status & 0xf0U;
    const int ch = status & 0x0f;
    switch (type)
    {
    case 0x90U:
        return data2 > 0 ? push(ExternalEvent::Type::NoteOn, data1 & 0x7f, static_cast<float>(data2 & 0x7f) / 127.0f, ch)
                         : push(ExternalEvent::Type::NoteOff, data1 & 0x7f, 0.0f, ch);
    case 0x80U:
        return push(ExternalEvent::Type::NoteOff, data1 & 0x7f, 0.0f, ch);
    case 0xe0U:
        return push(ExternalEvent::Type::PitchBend, 0,
                    (static_cast<float>((data1 & 0x7fU) | ((data2 & 0x7fU) << 7U)) - 8192.0f) / 8192.0f, ch);
    case 0xd0U:
        return push(ExternalEvent::Type::Aftertouch, 0, static_cast<float>(data1 & 0x7f) / 127.0f, ch);
    case 0xa0U:
        return push(ExternalEvent::Type::PolyPressure, data1 & 0x7f, static_cast<float>(data2 & 0x7f) / 127.0f, ch);
    case 0xb0U:
    {
        const int cc = data1 & 0x7f;
        const float value = static_cast<float>(data2 & 0x7f) / 127.0f;
        if (cc == 1)
            return modWheel(value);
        if (cc == 64)
            return sustain(data2 >= 64);
        if (cc == 120 || cc == 123)
            return allNotesOff();
        const bool mapped = applyCc(ch, cc, data2 & 0x7f, false);
        push(ExternalEvent::Type::Controller, cc, value, ch);
        return mapped;
    }
    default:
        return false;
    }
}

void Engine::mapCC(int cc, int paramIndex)
{
    if (cc < 0 || cc > 127 || isReservedCc(cc))
        return;
    ccStore.removeCc(cc, -1);
    if (paramIndex >= 0 && paramIndex < model().size())
    {
        CcBinding b;
        b.cc = cc;
        b.param = info(paramIndex).id;
        ccStore.add(b);
    }
}

int Engine::ccMapping(int cc) const
{
    if (cc < 0 || cc > 127)
        return -1;
    for (const auto& b : ccStore.bindings())
        if (b.cc == cc && b.channel == -1)
            return model().indexOf(b.param);
    return -1;
}

bool Engine::applyCc(int channel, int cc, int data2, bool realtime)
{
    // The control-thread form marks its changes as MIDI; the audio-thread form passes the source explicitly.
    std::optional<SourceScope> scope;
    if (!realtime)
        scope.emplace(*this, ParamSource::Midi);
    ccStore.noteCc(channel, cc, data2);
    CcLearned learned;
    if (ccStore.offer(channel, cc, data2, learned))
        return realtime ? setParamNormalisedRt(learned.paramIndex, learned.value, static_cast<int>(ParamSource::Midi))
                        : setParamNormalised(learned.paramIndex, learned.value);
    if (isReservedCc(cc))
        return false;
    bool changed = false;
    const CcTable* table = ccStore.acquire();
    for (size_t i = 0; table != nullptr && i < table->bindings.size(); ++i)
    {
        const auto& b = table->bindings[i];
        const int target = table->paramIndex[i];
        if (b.cc != cc || (b.channel >= 0 && b.channel != channel) || target < 0)
            continue;
        const float current =
            b.mode == CcMode::Absolute
                ? 0.0f
                : crucible::toNormalised(info(target), params[static_cast<size_t>(target)].load(std::memory_order_relaxed));
        const float normalised = applyCcBinding(b, data2, current);
        changed = (realtime ? setParamNormalisedRt(target, normalised, static_cast<int>(ParamSource::Midi))
                            : setParamNormalised(target, normalised)) ||
                  changed;
    }
    ccStore.release();
    return changed;
}

int Engine::paramChanges(int* indices, int* sources, int cap)
{
    if (indices == nullptr || cap <= 0)
        return 0;
    int count = 0;
    const int latency = latencySamples();
    if (latency != reportedLatency)
    {
        reportedLatency = latency;
        indices[count] = kLatencyChanged;
        if (sources != nullptr)
            sources[count] = static_cast<int>(ParamSource::Internal);
        ++count;
    }
    const double tail = tailSeconds();
    if (tail != reportedTail && count < cap) // a full buffer leaves the tail change for the next call
    {
        reportedTail = tail;
        indices[count] = kTailChanged;
        if (sources != nullptr)
            sources[count] = static_cast<int>(ParamSource::Internal);
        ++count;
    }
    const std::uint32_t routes = modRoutes().generation();
    if (routes != reportedRoutes && count < cap)
    {
        reportedRoutes = routes;
        indices[count] = kRoutesChanged;
        if (sources != nullptr)
            sources[count] = static_cast<int>(ParamSource::Internal);
        ++count;
    }
    if (!anyChanged.exchange(false, std::memory_order_acquire))
        return count;
    const int n = model().size();
    for (int i = 0; i < n; ++i)
    {
        if (changedBy[static_cast<size_t>(i)].load(std::memory_order_relaxed) == 0)
            continue;
        if (!info(i).exposed) // a host never saw the hidden parameters: the mark is dropped, not reported
        {
            changedBy[static_cast<size_t>(i)].store(0, std::memory_order_relaxed);
            continue;
        }
        if (count >= cap)
        {
            anyChanged.store(true, std::memory_order_release); // the rest waits for the next call
            break;
        }
        const std::uint8_t mark = changedBy[static_cast<size_t>(i)].exchange(0, std::memory_order_acquire);
        if (mark == 0)
            continue;
        indices[count] = i;
        if (sources != nullptr)
            sources[count] = mark - 1;
        ++count;
    }
    return count;
}

int Engine::latencySamples() const
{
    return core->engine().effects().reportedLatency();
}

double Engine::tailSeconds() const
{
    return core->engine().effects().reportedTail();
}

void Engine::reset()
{
    core->reset();
}

void Engine::process(float* left, float* right, int numSamples, const MidiInput* midiEvents, int numMidi,
                     const SeqHostTime* host, const float* auxLeft, const float* auxRight)
{
    const ScopedFlushDenormals noDenormals;

    // Mapped CCs inside the block act at its start, like control-thread parameter changes.
    for (int i = 0; i < numMidi; ++i)
    {
        const auto& m = midiEvents[i];
        if ((m.status & 0xf0U) == 0xb0U)
            applyCc(m.status & 0x0f, m.data1 & 0x7f, m.data2 & 0x7f, true);
    }

    for (size_t i = 0; i < snapshot.size(); ++i)
        snapshot[i] = params[i].load(std::memory_order_relaxed);

    SeqHostTime h = host != nullptr ? *host : SeqHostTime{};
    int done = 0;
    int nextMidi = 0;
    while (done < numSamples)
    {
        const int n = std::min(maxBlock, numSamples - done);
        int first = nextMidi;
        while (nextMidi < numMidi && midiEvents[nextMidi].position < done + n)
            ++nextMidi;
        // MidiInput positions are block relative. The first chunk uses the caller's array as it is (its positions already
        // are chunk relative); later chunks are re-based into the preallocated scratch.
        const MidiInput* slice = midiEvents;
        int count = nextMidi - first;
        if (done > 0)
        {
            count = std::min(count, static_cast<int>(midiSlice.size()));
            for (int i = 0; i < count; ++i)
            {
                midiSlice[static_cast<size_t>(i)] = midiEvents[first + i];
                midiSlice[static_cast<size_t>(i)].position = std::max(0, midiEvents[first + i].position - done);
            }
            slice = midiSlice.data();
        }
        else if (midiEvents != nullptr)
            slice = midiEvents + first;
        core->process(left + done, right + done, n, snapshot.data(), slice, count, h,
                      auxLeft != nullptr ? auxLeft + done : nullptr, auxRight != nullptr ? auxRight + done : nullptr);
        done += n;
        if (h.hasPpq && h.hasBpm) // later chunks start further along the host timeline
            h.ppq += static_cast<double>(n) / rate * h.bpm / 60.0;
    }
}

bool Engine::loadWavetable(int slot, const AudioData& audio, const WavetableImport& options, const std::string& name,
                           std::string& error)
{
    return userWavetables().load(slot, audio, options, name, error);
}

bool Engine::loadWavetableFile(int slot, const std::string& wavPath, const WavetableImport& options, std::string& error)
{
    return userWavetables().loadFile(slot, wavPath, options, error);
}

bool Engine::setFormantTable(int filterSlot, const FormantBand* bands, int vowels, int bandsPerVowel, float outputGain,
                             std::string& error)
{
    if (bands == nullptr)
        return core->engine().setFormantTable(filterSlot, nullptr, error);
    FormantTable table;
    if (!makeFormantTable(bands, vowels, bandsPerVowel, outputGain, table, error))
        return false;
    return core->engine().setFormantTable(filterSlot, &table, error);
}

bool Engine::setFormantSaturation(int filterSlot, bool on, std::string& error)
{
    return core->engine().setFormantSaturation(filterSlot, on, error);
}

bool Engine::formantTable(int filterSlot, FormantTable& out) const
{
    return core->engine().formantTable(filterSlot, out);
}

bool Engine::loadCloudWavetable(int slot, const CloudModel& cloud)
{
    CloudModel m = cloud;
    m.sanitise();
    return userWavetables().loadCycles(slot, cloudToCycles(m), WavetableImport::Normalise::Table,
                                       m.name.empty() ? "Cloud" : m.name);
}

bool Engine::loadSample(int slot, const AudioData& audio, const std::string& name, std::string& error)
{
    return samples().load(slot, audio, name, error);
}

bool Engine::loadSampleFile(int slot, const std::string& wavPath, std::string& error)
{
    return samples().loadFile(slot, wavPath, error);
}

bool Engine::loadConvolutionIr(const ImpulseResponse& ir, const std::string& name)
{
    auto* cx = core->engine().effects().findCapability<ConvolutionIr>();
    return cx != nullptr && cx->setImpulseResponse(ir, name);
}

bool Engine::loadConvolutionIrFile(const std::string& path, std::string& error)
{
    auto* cx = core->engine().effects().findCapability<ConvolutionIr>();
    if (cx == nullptr)
    {
        error = "no convolution module in the rack";
        return false;
    }
    ImpulseResponse ir;
    if (!importImpulseResponseFile(path, rate, ir, error))
        return false;
    return cx->setImpulseResponse(ir, path.substr(path.find_last_of("/\\") + 1));
}

bool Engine::loadIr(const std::string& instanceId, const std::string& path, std::string& error)
{
    auto& rack = core->engine().effects();
    if (rack.find(instanceId) == nullptr)
    {
        error = "unknown module instance: " + instanceId;
        return false;
    }
    const std::string name = path.substr(path.find_last_of("/\\") + 1);
    if (auto* cx = rack.findCapability<ConvolutionIr>(instanceId))
    {
        ImpulseResponse ir;
        if (!importImpulseResponseFile(path, rate, ir, error))
            return false;
        if (!cx->setImpulseResponse(ir, name))
        {
            error = "empty impulse response";
            return false;
        }
        return true;
    }
    error = instanceId + " takes no impulse response";
    return false;
}

bool Engine::clearIr(const std::string& instanceId, std::string& error)
{
    auto& rack = core->engine().effects();
    if (rack.find(instanceId) == nullptr)
    {
        error = "unknown module instance: " + instanceId;
        return false;
    }
    if (auto* cx = rack.findCapability<ConvolutionIr>(instanceId))
    {
        cx->clearImpulseResponse();
        return true;
    }
    error = instanceId + " has no impulse response to clear";
    return false;
}

bool Engine::setShaperCurve(const std::string& instanceId, const ShaperPoint* points, int count, bool symmetric,
                            std::string& error)
{
    auto& rack = core->engine().effects();
    if (rack.find(instanceId) == nullptr)
    {
        error = "unknown module instance: " + instanceId;
        return false;
    }
    auto* shaper = rack.findCapability<ShaperCurve>(instanceId);
    if (shaper == nullptr)
    {
        error = instanceId + " takes no shaper curve";
        return false;
    }
    if (points == nullptr && count == 0)
    {
        shaper->resetCurve();
        return true;
    }
    return shaper->setCurve(points, count, symmetric, error);
}

bool Engine::shaperCurve(const std::string& instanceId, std::vector<ShaperPoint>& out, bool& symmetric) const
{
    auto* shaper = core->engine().effects().findCapability<ShaperCurve>(instanceId);
    return shaper != nullptr && shaper->curve(out, symmetric);
}

void Engine::storeScene(int slot)
{
    sceneBank.store(slot, getAllParams());
}

bool Engine::recallScene(int slot)
{
    auto v = getAllParams();
    if (!sceneBank.recall(slot, v))
        return false;
    const SourceScope scope(*this, ParamSource::Internal);
    setAllParams(v);
    return true;
}

bool Engine::morphScenes(int slotA, int slotB, float t)
{
    auto v = getAllParams();
    if (!sceneBank.morph(slotA, slotB, t, v))
        return false;
    const SourceScope scope(*this, ParamSource::Internal);
    setAllParams(v);
    return true;
}

bool Engine::recordMotion(const MotionSettings& settings)
{
    const int i = model().indexOf(settings.param);
    if (i < 0)
        return false;
    const auto base = [&](int index)
    { return index < 0 ? 0.5f : crucible::toNormalised(model().params()[static_cast<size_t>(index)], getParam(index)); };
    return motion().arm(settings, base(i), base(settings.param2.empty() ? -1 : model().indexOf(settings.param2)));
}

void Engine::attachKeyBus(std::shared_ptr<KeyBus> b)
{
    core->attachKeyBus(b.get());
    bus = std::move(b); // the previous bus stays valid for a block in flight only if the host stops processing first
}

KeyScaleView Engine::keyScale() const
{
    KeyScaleView v = makeScaleView(core->keySettings());
    v.following = core->keyFollowing();
    if (bus)
        v.busGeneration = bus->read().generation;
    return v;
}

} // namespace crucible
