// SPDX-License-Identifier: MIT
#include <crucible/SynthEngine.h>

#include <crucible/Denormals.h>
#include <crucible/graph/Compiler.h>

#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>

namespace crucible
{
namespace
{
// Log-smoothed parameters may reach zero (the spectral drawbars); the smoothing runs on log2 of at least this.
constexpr float kLogFloor = 1.0e-4f;

float dbGain(float db)
{
    return std::pow(10.0f, db / 20.0f);
}

// The registry of the engine: the one in the configuration, else one built from the product layout, else one for the
// process default product. Every engine owns (or shares) the registry it plays.
EngineConfig resolveConfig(EngineConfig c)
{
    if (!c.registry && c.compiled)
        c.registry = c.compiled->registry;
    if (!c.registry)
        c.registry = ModuleRegistry::build(c.product ? *c.product : ModuleRegistry::processLayout());
    if (!c.registry->finalized())
        throw std::logic_error("EngineConfig::registry must be finalized");
    return c;
}

} // namespace

namespace
{
AdsrParams toAdsr(const std::vector<float>& v, const std::array<int, 7>& i)
{
    AdsrParams a;
    a.attack = v[static_cast<size_t>(i[0])];
    a.decay = v[static_cast<size_t>(i[1])];
    a.sustain = v[static_cast<size_t>(i[2])];
    a.release = v[static_cast<size_t>(i[3])];
    a.hold = v[static_cast<size_t>(i[4])];
    a.curve = v[static_cast<size_t>(i[5])];
    a.attackCurve = v[static_cast<size_t>(i[6])];
    return a;
}

} // namespace

SynthEngine::SynthEngine(const EngineConfig& config)
    : cfg(resolveConfig(config)), reg(cfg.registry), params_(&reg->parameters()), fx(*reg), routes(params_)
{
    const ParameterModel& model = *params_;
    classicHost.engine = this;
    productNow = cfg.compiled;
    const auto require = [&model](const std::string& id)
    {
        const int i = model.indexOf(id);
        assert(i >= 0);
        return std::max(i, 0);
    };
    const auto envIndices = [&require](const std::string& prefix)
    {
        return std::array<int, 7>{require(prefix + "_attack"),      require(prefix + "_decay"), require(prefix + "_sustain"),
                                  require(prefix + "_release"),     require(prefix + "_hold"),  require(prefix + "_curve"),
                                  require(prefix + "_attack_curve")};
    };
    genericDest.assign(model.modTargets().size(), 0.0f);
    cfg.maxVoices = std::clamp(cfg.maxVoices, 1, kMaxConfigVoices);
    cfg.maxBlockSize = std::max(cfg.maxBlockSize, 1);
    if (cfg.preloadWavetables)
        WavetableBank::instance().ensureAll();
    voices.resize(static_cast<size_t>(cfg.maxVoices + kStealSpareVoices));
    table = &model.params();
    const auto shape = reg->layout().voice;
    modelFilters = shape.filters;
    numOsc = cfg.numOscillators > 0 ? std::min(cfg.numOscillators, shape.oscillators) : shape.oscillators;
    numFilt = cfg.numFilters > 0 ? std::min(cfg.numFilters, shape.filters) : shape.filters;
    params.spectralSynth = &spectralSynth;
    for (size_t i = 0; i < voices.size(); ++i)
    {
        voices[i].configure(numOsc, numFilt, *reg);
        voices[i].setIndex(static_cast<int>(i));
    }
    for (int o = 0; o < numOsc; ++o)
    {
        const std::string p = "osc" + std::to_string(o + 1);
        idx.osc[static_cast<size_t>(o)] = {
            require(p + "_table"),      require(p + "_pos"),      require(p + "_semi"),       require(p + "_fine"),
            require(p + "_level"),      require(p + "_phase"),    require(p + "_uni_voices"), require(p + "_uni_detune"),
            require(p + "_uni_spread"), require(p + "_warp"),     require(p + "_warp_a"),     require(p + "_warp_b"),
            require(p + "_pan"),        require(p + "_route"),    require(p + "_warp_src"),   require(p + "_uni_mode"),
            require(p + "_uni_blend"),  require(p + "_uni_rand"), require(p + "_uni_scatter")};
        idx.osc[static_cast<size_t>(o)].wtInterp = model.indexOf(p + "_wt_interp");
        idx.osc[static_cast<size_t>(o)].specWarp = model.indexOf(p + "_wt_spec");
        idx.osc[static_cast<size_t>(o)].specAmt = model.indexOf(p + "_wt_spec_amt");
        params.osc[static_cast<size_t>(o)].specAmountTarget =
            idx.osc[static_cast<size_t>(o)].specAmt >= 0 ? model.voiceTargetOfParam(idx.osc[static_cast<size_t>(o)].specAmt)
                                                         : -1;
        idx.osc[static_cast<size_t>(o)].type = require(p + "_type");
        idx.osc[static_cast<size_t>(o)].xmode = require(p + "_xmod_mode");
        idx.osc[static_cast<size_t>(o)].xsrc = require(p + "_xmod_src");
        idx.osc[static_cast<size_t>(o)].xamt = require(p + "_xmod_amt");
        params.osc[static_cast<size_t>(o)].xamtTarget = model.voiceTargetOfParam(idx.osc[static_cast<size_t>(o)].xamt);
        {
            const auto& oi = idx.osc[static_cast<size_t>(o)];
            auto& ut = params.osc[static_cast<size_t>(o)].uniTargets;
            ut = {model.voiceTargetOfParam(oi.detune), model.voiceTargetOfParam(oi.spread),
                  model.voiceTargetOfParam(oi.uniBlend), model.voiceTargetOfParam(oi.uniScatter)};
        }
    }
    idx.subLevel = require("sub_level");
    idx.subOctave = require("sub_octave");
    idx.subWave = require("sub_wave");
    idx.noiseLevel = require("noise_level");
    idx.noiseType = require("noise_type");
    idx.masterGain = require("master_gain");
    for (int o = kNumOscs; o < numOsc; ++o)
    {
        const std::string p = "osc" + std::to_string(o + 1);
        auto& t = params.osc[static_cast<size_t>(o)].targets;
        const char* const suffix[6] = {"_pos", "_semi", "_level", "_warp_a", "_warp_b", "_pan"};
        for (size_t k = 0; k < 6; ++k)
            t[k] = model.voiceTargetOfParam(require(p + suffix[k]));
    }
    for (int f = kNumFilters; f < numFilt; ++f)
    {
        const std::string p = "filter" + std::to_string(f + 1);
        auto& t = params.filter[static_cast<size_t>(f)].targets;
        const char* const suffix[5] = {"_cutoff", "_res", "_drive", "_mix", "_pan"};
        for (size_t k = 0; k < 5; ++k)
            t[k] = model.voiceTargetOfParam(require(p + suffix[k]));
    }
    for (int f = 0; f < numFilt; ++f)
    {
        const std::string p = f == 0 ? "filter" : "filter" + std::to_string(f + 1);
        idx.filter[static_cast<size_t>(f)] = {require(p + "_on"),  require(p + "_type"),  require(p + "_cutoff"),
                                              require(p + "_res"), require(p + "_drive"), require(p + "_env"),
                                              require(p + "_key"), require(p + "_pan"),   require(p + "_mix")};
        idx.filter[static_cast<size_t>(f)].character = model.indexOf(p + "_character");
        idx.filter[static_cast<size_t>(f)].highpass = model.indexOf(p + "_hp");
        idx.filter[static_cast<size_t>(f)].fmSrc = model.indexOf(p + "_fm_src");
        idx.filter[static_cast<size_t>(f)].fmAmt = model.indexOf(p + "_fm_amt");
    }
    idx.filterRoute = require("filter_route");
    idx.subRoute = require("sub_route");
    idx.noiseRoute = require("noise_route");
    idx.env[0] = envIndices("amp");
    idx.env[1] = envIndices("menv");
    idx.env[2] = envIndices("env3");
    idx.env[3] = envIndices("env4");
    for (int l = 0; l < kNumLfos; ++l)
    {
        const std::string p = "lfo" + std::to_string(l + 1);
        idx.lfo[static_cast<size_t>(l)] = {require(p + "_rate"),  require(p + "_shape"), require(p + "_amount"),
                                           require(p + "_trig"),  require(p + "_sync"),  require(p + "_fade"),
                                           require(p + "_delay"), require(p + "_phase"), require(p + "_smooth")};
        idx.lfo[static_cast<size_t>(l)].audio = model.indexOf(p + "_audio");
        idx.lfo[static_cast<size_t>(l)].audioHz = model.indexOf(p + "_audio_hz");
        idx.lfo[static_cast<size_t>(l)].audioRatio = model.indexOf(p + "_audio_ratio");
    }
    for (int n = 0; n < kNumBankModulators; ++n)
    {
        const std::string p = "bank" + std::to_string(n + 1);
        auto& b = idx.bank[static_cast<size_t>(n)];
        b.type = model.indexOf(p + "_type");
        b.scope = model.indexOf(p + "_scope");
        b.inA = model.indexOf(p + "_in_a");
        b.inB = model.indexOf(p + "_in_b");
        b.mode = model.indexOf(p + "_mode");
        b.amount = model.indexOf(p + "_amount");
        b.rate = model.indexOf(p + "_rate");
        b.rise = model.indexOf(p + "_rise");
        b.fall = model.indexOf(p + "_fall");
        b.steps = model.indexOf(p + "_steps");
        b.seed = model.indexOf(p + "_seed");
        for (int k = 0; k < 16; ++k)
        {
            char id[16];
            std::snprintf(id, sizeof id, "_pt%02d", k + 1);
            b.pt[static_cast<size_t>(k)] = model.indexOf(p + id);
        }
    }
    for (int s = 0; s < kNumModSlots; ++s)
    {
        const std::string p = "mod" + std::to_string(s + 1);
        idx.slot[static_cast<size_t>(s)] = {require(p + "_src"), require(p + "_dst"), require(p + "_amt"),
                                            require(p + "_pol"), require(p + "_curve")};
    }
    for (int m = 0; m < kNumMacros; ++m)
        idx.macro[static_cast<size_t>(m)] = m < model.macros() ? require("macro" + std::to_string(m + 1)) : -1;
    vec.on = require("vector_on");
    vec.x = require("vector_x");
    vec.y = require("vector_y");
    vec.smooth = require("vector_smooth");
    vec.drift = require("vector_drift");
    vec.driftRate = require("vector_drift_rate");
    vec.orbitRate = require("vector_orbit_rate");
    vec.orbitDepth = require("vector_orbit_depth");
    vec.orbitShape = require("vector_orbit_shape");
    for (int k = 0; k < 4; ++k)
        vec.corner[static_cast<size_t>(k)] = require(std::string("vector_") + static_cast<char>('a' + k));
    {
        const auto& targets = model.modTargets();
        for (size_t k = 0; k < targets.size(); ++k)
        {
            if (targets[k] == vec.x)
                vec.targetX = static_cast<int>(k);
            if (targets[k] == vec.y)
                vec.targetY = static_cast<int>(k);
        }
    }
    idx.stripL = require("strip_l");
    idx.stripR = require("strip_r");
    idx.morph = model.indexOf("scene_morph");

    idx.voiceMode = require("voice_mode");
    idx.glide = require("voice_glide");
    idx.bendRange = require("voice_bend");
    idx.poly = require("voice_poly");
    idx.tuneAmount = require("tune_amount");
    idx.glideMode = require("voice_glide_mode");
    idx.glideType = require("voice_glide_type");
    idx.monoPriority = require("voice_mono_priority");
    idx.mpeOn = require("mpe_on");
    idx.mpeRange = require("mpe_bend_range");
    idx.drift = require("voice_drift");
    idx.transpose = require("voice_transpose");
    {
        static constexpr const char* envPrefix[kNumEnvs] = {"amp", "menv", "env3", "env4"};
        for (size_t e = 0; e < static_cast<size_t>(kNumEnvs); ++e)
        {
            const std::string p = envPrefix[e];
            idx.envShape[e] = {model.indexOf(p + "_mode"), model.indexOf(p + "_retrig"), model.indexOf(p + "_overshoot"),
                               model.indexOf(p + "_vel_time"), model.indexOf(p + "_key_time")};
        }
        idx.cardOsc = model.indexOf("voice_card_osc");
        idx.cardFilter = model.indexOf("voice_card_filter");
        idx.cardEnv = model.indexOf("voice_card_env");
        idx.cardLevel = model.indexOf("voice_card_level");
        idx.noiseOn = model.indexOf("voice_noise_on");
        idx.noiseDb = model.indexOf("voice_noise_db");
        idx.humOn = model.indexOf("voice_hum_on");
        idx.humDb = model.indexOf("voice_hum_db");
        idx.humHz = model.indexOf("voice_hum_hz");
    }

    smoothers.resize(table->size());
    for (size_t i = 0; i < table->size(); ++i)
        if ((*table)[i].smoothing != Smoothing::None)
            smoothedParams.push_back(static_cast<int>(i));
    paramDest.assign(table->size(), 0);
    for (size_t i = 0; i < table->size(); ++i)
    {
        const ModDest d = routeVoiceDestination((*table)[i].id);
        if (d != ModDest::None && d != ModDest::MasterGain && !(d >= ModDest::Lfo1Rate && d <= ModDest::Lfo4Rate))
            paramDest[i] = static_cast<int>(d);
    }
    modulated = std::make_unique<std::atomic<float>[]>(table->size());
    published = std::make_unique<std::atomic<float>[]>(table->size());
    current = model.defaultValues();
    hostValues = current;
    logMemo.assign(current.size(), Memo{});
    expMemo.assign(current.size(), Memo{});
    overlayExact.assign(current.size(), -1.0f);
    lastExclude.assign(current.size(), 0);
    varying.reserve(table->size());
    varyingValue.reserve(table->size());
    hostDependent.reserve(table->size());
    for (size_t i = 0; i < table->size(); ++i)
        if ((*table)[i].scene)
            sceneManaged.push_back(static_cast<int>(i));
    for (size_t i = 0; i < current.size(); ++i)
    {
        modulated[i].store(current[i]);
        published[i].store(current[i]);
    }
    for (size_t l = 0; l < kNumLfos; ++l)
    {
        customActive[l].store(0);
        for (int i = 0; i < kLfoShapePoints; ++i)
            customShapes[l][0][static_cast<size_t>(i)] = customShapes[l][1][static_cast<size_t>(i)] =
                std::sin(kTwoPi * static_cast<float>(i) / kLfoShapePoints);
    }
}

void SynthEngine::prepare(double rate)
{
    sampleRate = rate;
    userTables.setSampleRate(rate); // zoned user wavetables are rebuilt for the new rate (control thread)
    for (auto& v : voices)
        v.prepare(rate, static_cast<int>(cfg.quality));
    filterBank.prepare(rate, cfg.quality, static_cast<int>(voices.size()), numFilt, cfg.seed);
    noiseHum.prepare(rate, VoiceCard::mix(cfg.seed ^ 0x6e6f6973U));
    stagedVoice.assign(voices.size(), 0);
    initialised = false;
    // Tables the oscillators start on are generated here, on the control thread, so the first process() call allocates
    // nothing. Later table changes are generated by Engine (setParam, service); the audio thread never generates.
    {
        for (int o = 0; o < numOsc; ++o)
        {
            const size_t t = static_cast<size_t>(idx.osc[static_cast<size_t>(o)].table);
            if (t < table->size())
            {
                const int def = static_cast<int>((*table)[t].defaultValue);
                WavetableBank::instance().ensure(def);
                spectral::FactoryTables::instance().ensure(def);
            }
        }
    }
    controlPhase = 0;
    noteCounter = 0;
    heldCount = 0;
    voiceChannel.fill(-1);
    voiceNotePressure.fill(-1.0f);
    voiceGlideScale.fill(1.0f);
    chBend.fill(0.0f);
    chPressure.fill(0.0f);
    chSlide.fill(0.0f);
    lastNote = -1.0f;
    sustainDown = false;
    for (auto& s : smoothers)
        s.reset(rate / kChunkSize, 0.03);
    vecWasOn = false;
    lastLive = false;
    std::fill(overlayExact.begin(), overlayExact.end(), -1.0f);
    overlayLive.store(false, std::memory_order_relaxed);
    orbitPhase = 0.0;
    driftX = driftY = driftTargetX = driftTargetY = 0.0f;
    driftTimer = 0.0;
    vecRng = (cfg.seed ^ 0x9e3779b9U) | 1U;
    ModuleContext context;
    context.sampleRate = rate;
    context.maxBlockSize = cfg.maxBlockSize;
    context.quality = cfg.quality;
    context.oversampling = cfg.oversampling;
    context.seed = cfg.seed;
    context.renderOffline = &renderOffline;
    fx.prepare(context);
    fadeLength = std::max(1, static_cast<int>(0.010 * rate));
    fade = Fade::Idle;
    fadePos = 0;
    fadeState.store(0, std::memory_order_relaxed);
    incomingProgram = nullptr;
    programHost.hold(1, nullptr);
    installProgram();
    motionRecorder.setSampleRate(rate);
    clipPlayer.prepare(rate, static_cast<int>(voices.size()));
    voiceRouteSmooth.assign(voices.size() * kMaxRoutes, 0.0f);
    for (auto& st : sharedBank)
        st = ModulatorState{};
    noteOnCount = noteOnSeen = 0;
    sharedMixPrev.fill(0.0f);
    sharedMixPrevCount = 0;
    for (size_t l = 0; l < kNumLfos; ++l)
        freeLfos[l].reset(0x9e3779b9U * static_cast<std::uint32_t>(l + 1), 0.0f);
}

void SynthEngine::ensureSelectedTables(const float* values, int count)
{
    const auto n = std::min(static_cast<size_t>(std::max(count, 0)), table->size());
    for (int o = 0; o < numOsc; ++o)
    {
        const size_t t = static_cast<size_t>(idx.osc[static_cast<size_t>(o)].table);
        if (t < n && std::isfinite(values[t]))
        {
            WavetableBank::instance().ensure(static_cast<int>(values[t]));
            spectral::FactoryTables::instance().ensure(static_cast<int>(values[t]));
        }
    }
}

void SynthEngine::setParameterValues(const float* values, int count)
{
    const auto n = std::min(static_cast<size_t>(count), table->size());
    // Non-finite host values (a NaN passes through std::clamp) take the parameter's default instead of poisoning the voices.
    for (size_t i = 0; i < n; ++i)
        hostValues[i] = std::isfinite(values[i]) ? values[i] : (*table)[i].defaultValue;
    values = hostValues.data();
    // While the vector overlay is live it owns the scene-managed parameters; setting their targets from the host
    // values here would restart their ramps against the overlay's on every block.
    const bool overlay = initialised && overlayLive.load(std::memory_order_relaxed) && static_cast<size_t>(vec.on) < n &&
                         values[vec.on] > 0.5f;
    for (size_t i = 0; i < n; ++i)
    {
        const auto& def = (*table)[i];
        if (overlay && def.scene && (vectorExclude == nullptr || vectorExclude[i] == 0))
            continue;
        const float v = std::clamp(values[i], def.min, def.max);
        const float target = def.smoothing == Smoothing::Log ? memoised(logMemo[i], std::max(v, kLogFloor), std::log2) : v;
        if (!initialised || def.smoothing == Smoothing::None)
            smoothers[i].setCurrentAndTargetValue(target);
        else
            smoothers[i].setTargetValue(target);
    }
    initialised = true;
    // A parameter still gliding keeps its smoothed value (the next control cell steps it on); the others read their target.
    for (size_t i = 0; i < n; ++i)
    {
        const float v = smoothers[i].isSmoothing() ? smoothers[i].getCurrentValue() : smoothers[i].getTargetValue();
        current[i] = (*table)[i].smoothing == Smoothing::Log ? memoised(expMemo[i], v, std::exp2) : v;
    }
    buildParams();
}

void SynthEngine::stepSmoothers()
{
    // Only the smoothed parameters: a parameter that never glides (choices, switches, the modulator bank's settings) costs
    // nothing per control cell.
    for (const int index : smoothedParams)
    {
        const auto i = static_cast<size_t>(index);
        const auto& def = (*table)[i];
        const float v = smoothers[i].getNextValue();
        current[i] = def.smoothing == Smoothing::Log ? memoised(expMemo[i], v, std::exp2) : v;
        // A log-smoothed parameter the vector overlay has settled on reports the exact scene value.
        if (def.smoothing == Smoothing::Log && overlayExact[i] > 0.0f && !smoothers[i].isSmoothing())
            current[i] = overlayExact[i];
    }
}

CardAmounts SynthEngine::cardAmountsNow() const
{
    CardAmounts a;
    const auto at = [this](int i) { return i >= 0 ? std::clamp(current[static_cast<size_t>(i)], 0.0f, 1.0f) : 0.0f; };
    a.osc = at(idx.cardOsc);
    a.filter = at(idx.cardFilter);
    a.env = at(idx.cardEnv);
    a.level = at(idx.cardLevel);
    return a;
}

void SynthEngine::buildParams()
{
    const auto& v = current;
    const auto at = [&v](int i) { return v[static_cast<size_t>(i)]; };

    params.numOsc = numOsc;
    params.numFilter = numFilt;
    params.modelFilters = modelFilters;
    for (size_t o = 0; o < static_cast<size_t>(numOsc); ++o)
    {
        const auto& i = idx.osc[o];
        auto& d = params.osc[o];
        d.type = static_cast<int>(at(i.type));
        d.xmode = static_cast<int>(at(i.xmode));
        d.xsrc = static_cast<int>(at(i.xsrc));
        d.xamt = at(i.xamt);
        d.table = static_cast<int>(at(i.table));
        d.position = at(i.pos);
        d.semitones = at(i.semi);
        d.fineCents = at(i.fine);
        d.level = at(i.level);
        d.phaseDegrees = at(i.phase);
        d.unisonVoices = static_cast<int>(at(i.voices));
        d.detuneCents = at(i.detune);
        d.spread = at(i.spread);
        d.warp = static_cast<int>(at(i.warp));
        d.warpA = at(i.warpA);
        d.warpB = at(i.warpB);
        d.pan = at(i.pan);
        d.route = static_cast<int>(at(i.route));
        d.warpSource = static_cast<int>(at(i.warpSrc));
        d.uniMode = static_cast<int>(at(i.uniMode));
        d.uniBlend = at(i.uniBlend);
        d.uniRand = at(i.uniRand);
        d.uniScatter = at(i.uniScatter);
        d.wtInterp = i.wtInterp >= 0 ? static_cast<int>(at(i.wtInterp)) : 0;
        d.specWarp = i.specWarp >= 0 ? static_cast<int>(at(i.specWarp)) : 0;
        d.specAmount = i.specAmt >= 0 ? at(i.specAmt) : 0.0f;
    }
    params.subLevel = at(idx.subLevel);
    params.subOctave = static_cast<int>(at(idx.subOctave));
    params.subWave = static_cast<int>(at(idx.subWave));
    params.noiseLevel = at(idx.noiseLevel);
    params.noiseType = static_cast<int>(at(idx.noiseType));
    for (size_t f = 0; f < static_cast<size_t>(numFilt); ++f)
    {
        const auto& i = idx.filter[f];
        auto& d = params.filter[f];
        d.on = at(i.on) > 0.5f;
        d.model = static_cast<int>(at(i.type));
        d.cutoff = at(i.cutoff);
        d.resonance = at(i.res);
        d.drive = at(i.drive);
        d.envAmount = at(i.env);
        d.keytrack = at(i.key);
        d.pan = at(i.pan);
        d.mix = at(i.mix);
        d.character = i.character >= 0 ? at(i.character) : 0.0f;
        d.highpass = i.highpass >= 0 ? static_cast<int>(at(i.highpass)) : 0;
        d.fmSource = i.fmSrc >= 0 ? static_cast<int>(at(i.fmSrc)) : 0;
        d.fmAmount = i.fmAmt >= 0 ? at(i.fmAmt) : 0.0f;
    }
    params.filterRoute = static_cast<int>(at(idx.filterRoute));
    params.subRoute = static_cast<int>(at(idx.subRoute));
    params.noiseRoute = static_cast<int>(at(idx.noiseRoute));
    for (size_t e = 0; e < kNumEnvs; ++e)
    {
        params.env[e] = toAdsr(v, idx.env[e]);
        const auto& s = idx.envShape[e];
        auto& d = params.envShape[e];
        if (s.mode >= 0)
        {
            d.mode = at(s.mode) > 0.5f ? EnvMode::AnalogRc : EnvMode::Digital;
            d.retrigger = static_cast<Retrigger>(std::clamp(static_cast<int>(at(s.retrigger)), 0, 3));
            d.overshoot = at(s.overshoot);
            d.velocityTime = at(s.velocityTime);
            d.keyTime = at(s.keyTime);
        }
    }
    params.card = cardAmountsNow();
    for (size_t l = 0; l < kNumLfos; ++l)
    {
        const auto& i = idx.lfo[l];
        auto& d = params.lfo[l];
        d.amount = at(i.amount);
        d.settings.shape = static_cast<int>(at(i.shape));
        d.settings.trigger = static_cast<LfoTrigger>(static_cast<int>(at(i.trig)));
        d.settings.fadeSeconds = at(i.fade);
        d.settings.delaySeconds = at(i.delay);
        d.settings.startPhase = at(i.phase);
        d.settings.smooth = at(i.smooth);
        const int sync = std::clamp(static_cast<int>(at(i.sync)), 0, 13);
        d.settings.rateHz = sync == 0 ? at(i.rate) : static_cast<float>(tempoBpm / 60.0) / lfoSyncBeats(sync);
        d.audioMode = i.audio >= 0 ? static_cast<int>(at(i.audio)) : 0;
        d.audioHz = i.audioHz >= 0 ? at(i.audioHz) : 440.0f;
        d.audioRatio = i.audioRatio >= 0 ? at(i.audioRatio) : 1.0f;
    }
    for (size_t s = 0; s < kNumModSlots; ++s)
        params.slots[s] = {static_cast<int>(at(idx.slot[s].src)), static_cast<int>(at(idx.slot[s].dst)),
                           at(idx.slot[s].amount), static_cast<int>(at(idx.slot[s].pol)), at(idx.slot[s].curve)};
    params.glide = at(idx.glide);
    params.driftAmount = at(idx.drift);
    params.transpose = at(idx.transpose);
    voiceMode = static_cast<int>(at(idx.voiceMode));
    // The parameter's maximum selects the whole pool (so larger pools need no parameter change).
    const int poly = static_cast<int>(at(idx.poly));
    polyLimit = poly >= kNumVoices ? cfg.maxVoices : std::min(poly, cfg.maxVoices);
    params.seed = cfg.seed;
    params.values = current.data();
    bendRange = at(idx.bendRange);
    tuneAmount = at(idx.tuneAmount);
    glideMode = static_cast<int>(at(idx.glideMode));
    glideType = static_cast<int>(at(idx.glideType));
    monoPriority = static_cast<int>(at(idx.monoPriority));
    mpeOn = at(idx.mpeOn) > 0.5f;
    mpeRange = at(idx.mpeRange);
    const float masterDb = at(idx.masterGain);
    masterGain = masterDb > -60.0f ? std::pow(10.0f, masterDb * 0.05f) : 0.0f;
    buildModulators();
    params.routeSet = pinnedRoutes != nullptr && !pinnedRoutes->routes.empty() ? pinnedRoutes : nullptr;
}

// The snapshot of the bank's parameters for this chunk; the voices get it only while some modulator is wanted.
void SynthEngine::buildModulators()
{
    const auto at = [this](int i) { return current[static_cast<size_t>(i)]; };
    bool any = false;
    for (size_t n = 0; n < static_cast<size_t>(kNumBankModulators); ++n)
        any = any || (idx.bank[n].type >= 0 && static_cast<int>(at(idx.bank[n].type)) != 0);
    if (!any)
    {
        bankSet.active = bankSet.needed = 0;
        params.modulators = nullptr;
        return;
    }
    for (size_t n = 0; n < static_cast<size_t>(kNumBankModulators); ++n)
    {
        const auto& b = idx.bank[n];
        auto& c = bankSet.mod[n];
        if (b.type < 0)
        {
            c.type = ModulatorType::Off;
            continue;
        }
        c.type = static_cast<ModulatorType>(
            std::clamp(static_cast<int>(at(b.type)), 0, static_cast<int>(ModulatorType::Count) - 1));
        c.scope = static_cast<ModulatorScope>(std::clamp(static_cast<int>(at(b.scope)), 0, 2));
        c.inA = static_cast<int>(at(b.inA));
        c.inB = static_cast<int>(at(b.inB));
        c.mode = static_cast<int>(at(b.mode));
        c.amount = at(b.amount);
        c.rate = at(b.rate);
        c.rise = at(b.rise);
        c.fall = at(b.fall);
        c.steps = static_cast<int>(at(b.steps));
        c.seed = static_cast<std::uint32_t>(std::max(static_cast<int>(at(b.seed)), 1));
        if (c.type == ModulatorType::Remap)
            for (size_t k = 0; k < 16; ++k)
                c.curve[k] = at(b.pt[k]);
    }
    bankSet.finalize(pinnedRoutes != nullptr ? pinnedRoutes->modulatorMask : 0U);
    params.modulators = bankSet.needed != 0U ? &bankSet : nullptr;
}

float SynthEngine::modulatorValue(int n) const
{
    return n >= 0 && n < kNumBankModulators ? bankPublished[static_cast<size_t>(n)].load(std::memory_order_relaxed) : 0.0f;
}

Voice* SynthEngine::allocateVoice()
{
    int activeCount = 0;
    for (const auto& v : voices)
        if (v.isActive() && !v.isReleasing())
            ++activeCount;

    // Free voices within the configured pool first.
    const size_t pool = static_cast<size_t>(cfg.maxVoices);
    if (activeCount < polyLimit)
        for (size_t i = 0; i < pool; ++i)
            if (!voices[i].isActive())
                return &voices[i];

    // Steal: a releasing voice first (oldest), else the oldest playing one.
    Voice* best = nullptr;
    for (auto& v : voices)
    {
        if (!v.isActive() || v.isFading())
            continue;
        if (best == nullptr)
        {
            best = &v;
            continue;
        }
        const bool better =
            (v.isReleasing() && !best->isReleasing()) || (v.isReleasing() == best->isReleasing() && v.age() < best->age());
        if (better)
            best = &v;
    }
    if (best == nullptr)
    {
        // Everything is already fading: reuse the oldest of those (one that does not wait for a note already).
        for (auto& v : voices)
            if (v.isActive() && (best == nullptr || (best->hasPendingStart() && !v.hasPendingStart()) ||
                                 (best->hasPendingStart() == v.hasPendingStart() && v.age() < best->age())))
                best = &v;
        return best != nullptr ? best : &voices[0];
    }
    // The victim fades out while the new note starts on a free voice (the spare voices exist for this).
    for (auto& v : voices)
        if (!v.isActive())
        {
            best->fadeOut();
            return &v;
        }
    return best; // no free voice at all: the caller starts the note once this voice has faded
}

void SynthEngine::startNote(int note, float velocity, bool overlap)
{
    // Glide starts from the previous note: always, or only when a key is still held (legato).
    const bool glides = params.glide > 0.0f && lastNote >= 0.0f && (glideMode == 0 || overlap);
    const float startPitch = glides ? lastNote : static_cast<float>(note);
    const auto claim = [&](const Voice* v, float from)
    {
        const auto vi = static_cast<size_t>(v - voices.data());
        voiceChannel[vi] = pendingChannel;
        voiceNotePressure[vi] = -1.0f;
        voiceGlideScale[vi] = glideType == 1 ? std::abs(static_cast<float>(note) - from) / 12.0f : 1.0f;
    };
    if (voiceMode == 0)
    {
        // The same key struck again while it still sounds held: the older voice fades out (about 5 ms) rather than
        // running coherently beside the new one. Any other voice at the same pitch (releasing tails) is detuned by a
        // few cents so the two beat slowly.
        int samePitch = 0;
        for (auto& v : voices)
            if (v.isActive() && v.note() == note)
            {
                if (!v.isReleasing())
                    v.fadeOut();
                else
                    ++samePitch;
            }
        const float detune =
            samePitch == 0 ? 0.0f : 0.03f * static_cast<float>((samePitch + 1) / 2) * (samePitch % 2 == 1 ? 1.0f : -1.0f);
        if (auto* v = allocateVoice())
        {
            // A voice that still sounds (every voice was busy: a steal) fades out for about 5 ms, then the note starts.
            if (v->isActive())
                v->noteOnAfterFade(note, velocity, params, ++noteCounter, startPitch, detune);
            else
                v->noteOn(note, velocity, params, ++noteCounter, startPitch, detune);
            claim(v, startPitch);
        }
    }
    else
    {
        Voice& v = voices[0];
        for (size_t i = 1; i < voices.size(); ++i)
            if (voices[i].isActive())
                voices[i].noteOff();
        if (v.isActive() && !v.isReleasing() && voiceMode == 2)
        {
            const float from = static_cast<float>(v.note());
            v.glideTo(note);
            claim(&v, from);
        }
        else
        {
            const float from = v.isActive() && glides ? static_cast<float>(v.note()) : startPitch;
            v.noteOn(note, velocity, params, ++noteCounter, from);
            claim(&v, from);
        }
    }
    lastNote = static_cast<float>(note);
}

void SynthEngine::noteOn(int note, float velocity, int channel)
{
    const VoiceCountPublisher publish{*this};
    ++noteOnCount;
    sharedVelocity = std::clamp(velocity, 0.0f, 1.0f);
    sharedNote = static_cast<float>(note);
    if (graphVoices())
    {
        if (activeProgram->voiceKind() == graph::VoiceKind::Graph)
        {
            activeProgram->setCardAmounts(params.card);
            activeProgram->noteOn(note, velocity);
        }
        clipPlayer.noteOn();
        return;
    }
    {
        const TuningTable* t = tuningStore.acquire();
        const bool playable = t == nullptr || note < 0 || note > 127 || t->mapped[static_cast<size_t>(note)] != 0;
        tuningStore.release();
        if (!playable)
            return; // the keyboard map leaves this key unmapped
    }
    pendingChannel = mpeOn && validChannel(channel) ? channel : -1;
    // A repeated note replaces its entry in the held stack.
    for (int i = 0; i < heldCount; ++i)
        if (held[static_cast<size_t>(i)].note == note)
        {
            for (int j = i; j + 1 < heldCount; ++j)
                held[static_cast<size_t>(j)] = held[static_cast<size_t>(j + 1)];
            --heldCount;
            break;
        }
    int sounding = 0;
    for (int i = 0; i < heldCount; ++i)
        sounding += held[static_cast<size_t>(i)].released ? 0 : 1;
    const bool overlap = sounding > 0;
    if (heldCount < static_cast<int>(held.size()))
        held[static_cast<size_t>(heldCount++)] = {note, velocity, false, pendingChannel};
    // Mono priority: a new note that loses against the one that sounds only joins the held stack.
    if (voiceMode != 0 && monoPriority != 0 && overlap && voices[0].isActive() && !voices[0].isReleasing())
    {
        const int playing = voices[0].note();
        if ((monoPriority == 1 && note > playing) || (monoPriority == 2 && note < playing))
            return;
    }
    startNote(note, velocity, overlap);
    clipPlayer.noteOn();
}

void SynthEngine::noteOff(int note)
{
    const VoiceCountPublisher publish{*this};
    if (graphVoices())
    {
        if (activeProgram->voiceKind() == graph::VoiceKind::Graph)
            activeProgram->noteOff(note);
        return;
    }
    int pos = -1;
    for (int i = 0; i < heldCount; ++i)
        if (held[static_cast<size_t>(i)].note == note && !held[static_cast<size_t>(i)].released)
            pos = i;
    if (pos < 0)
        return;

    if (sustainDown)
    {
        held[static_cast<size_t>(pos)].released = true;
        return;
    }

    const bool wasTop = pos == heldCount - 1;
    const bool wasSounding = voiceMode != 0 && voices[0].isActive() && voices[0].note() == note;
    for (int j = pos; j + 1 < heldCount; ++j)
        held[static_cast<size_t>(j)] = held[static_cast<size_t>(j + 1)];
    --heldCount;

    if (voiceMode == 0)
    {
        for (auto& v : voices)
            if (v.isActive() && (!v.isReleasing() || v.hasPendingStart()) && v.note() == note)
                v.noteOff();
        return;
    }

    Voice& v = voices[0];
    if (heldCount == 0)
    {
        v.noteOff();
    }
    else if (monoPriority == 0 ? wasTop : wasSounding)
    {
        const int at = monoPriority == 0 ? heldCount - 1 : pickMonoNote(-1);
        const auto prev = held[static_cast<size_t>(at)];
        pendingChannel = prev.channel;
        if (voiceMode == 2)
        {
            const float from = static_cast<float>(v.note());
            v.glideTo(prev.note);
            voiceChannel[0] = prev.channel;
            voiceGlideScale[0] = glideType == 1 ? std::abs(static_cast<float>(prev.note) - from) / 12.0f : 1.0f;
        }
        else
            startNote(prev.note, prev.velocity, true);
    }
}

// Index in the held stack of the note mono priority picks (lowest or highest).
int SynthEngine::pickMonoNote(int) const
{
    int best = heldCount - 1;
    for (int i = 0; i < heldCount; ++i)
    {
        const int n = held[static_cast<size_t>(i)].note;
        const int b = held[static_cast<size_t>(best)].note;
        if ((monoPriority == 1 && n < b) || (monoPriority == 2 && n > b))
            best = i;
    }
    return best;
}

void SynthEngine::allNotesOff()
{
    const VoiceCountPublisher publish{*this};
    if (activeProgram != nullptr && activeProgram->voiceKind() == graph::VoiceKind::Graph)
        activeProgram->allNotesOff();
    heldCount = 0;
    for (auto& v : voices)
        if (v.isActive())
            v.noteOff();
}

void SynthEngine::killAllVoices()
{
    const VoiceCountPublisher publish{*this};
    if (activeProgram != nullptr && activeProgram->voiceKind() == graph::VoiceKind::Graph)
        activeProgram->allNotesOff();
    heldCount = 0;
    sustainDown = false;
    for (auto& v : voices)
        if (v.isActive())
            v.fadeOutNow();
}

void SynthEngine::reset()
{
    const VoiceCountPublisher publish{*this};
    if (activeProgram != nullptr)
        activeProgram->killAll();
    for (auto& v : voices)
        v.kill();
    heldCount = 0;
    sustainDown = false;
    fx.reset();
}

void SynthEngine::setSustainPedal(bool down)
{
    sustainDown = down;
    if (down)
        return;
    for (int i = heldCount - 1; i >= 0; --i)
        if (held[static_cast<size_t>(i)].released)
        {
            const int note = held[static_cast<size_t>(i)].note;
            held[static_cast<size_t>(i)].released = false;
            noteOff(note);
        }
}

void SynthEngine::setNotePressure(int note, float v)
{
    for (size_t i = 0; i < voices.size(); ++i)
        if (voices[i].isActive() && voices[i].note() == note)
            voiceNotePressure[i] = std::clamp(v, 0.0f, 1.0f);
}

SynthEngine::Expression SynthEngine::expressionOf(size_t vi) const
{
    Expression e;
    const int ch = voiceChannel[vi];
    e.pressure = voiceNotePressure[vi] >= 0.0f ? voiceNotePressure[vi]
                 : ch >= 0                     ? chPressure[static_cast<size_t>(ch)]
                                               : aftertouch;
    e.slide = ch >= 0 ? chSlide[static_cast<size_t>(ch)] : controllers[74];
    e.bend = ch >= 0 ? chBend[static_cast<size_t>(ch)] : 0.0f;
    return e;
}

// Event and audio thread: the count UIs read through activeVoiceCount(). The voices and the active program belong to
// the audio thread, so other threads only see this published value.
void SynthEngine::publishVoiceCount()
{
    const int graphVoicesNow = activeProgram != nullptr ? activeProgram->activeVoices() : 0;
    const int total = graphVoicesNow + static_cast<int>(std::count_if(voices.begin(), voices.end(), [](const Voice& v)
                                                                      { return v.isActive() && !v.isFading(); }));
    publishedVoiceCount.store(total, std::memory_order_relaxed);
}

std::uint64_t SynthEngine::batchedFilterBlocks() const
{
    return filterBank.processedQuadBlocks() + (activeProgram != nullptr ? activeProgram->quadFilterBlocks() : 0);
}

int SynthEngine::activeVoiceCount() const
{
    return publishedVoiceCount.load(std::memory_order_relaxed);
}

void SynthEngine::setCustomLfoShape(int lfo, const float* points, int count)
{
    if (lfo < 0 || lfo >= kNumLfos || points == nullptr || count < 2)
        return;
    const auto l = static_cast<size_t>(lfo);
    const int writing = 1 - customActive[l].load();
    auto& dst = customShapes[l][static_cast<size_t>(writing)];
    for (int i = 0; i < kLfoShapePoints; ++i)
    {
        const float x = static_cast<float>(i) / kLfoShapePoints * static_cast<float>(count);
        const int a = std::min(static_cast<int>(x), count - 1);
        const int b = (a + 1) % count;
        const float f = x - static_cast<float>(a);
        dst[static_cast<size_t>(i)] = std::clamp(points[a] + (points[b] - points[a]) * f, -1.0f, 1.0f);
    }
    customActive[l].store(writing);
}

void SynthEngine::updateGlobalMod(int numSamples)
{
    const auto& v = current;
    auto& g = globalMod;
    for (size_t m = 0; m < kNumMacros; ++m)
        g.macro[m] = idx.macro[m] >= 0 ? v[static_cast<size_t>(idx.macro[m])] : 0.0f;
    g.strip[0] = v[static_cast<size_t>(idx.stripL)];
    g.strip[1] = v[static_cast<size_t>(idx.stripR)];
    g.morph = idx.morph >= 0 ? v[static_cast<size_t>(idx.morph)] : 0.0f;
    g.modWheel = modWheel;
    g.aftertouch = aftertouch;
    g.pitchBend = pitchBendNorm;
    for (size_t l = 0; l < kNumLfos; ++l)
    {
        g.customShape[l] = customShapes[l][static_cast<size_t>(customActive[l].load())].data();
        const auto& lp = params.lfo[l];
        if (lp.settings.trigger == LfoTrigger::Free)
        {
            LfoSettings st = lp.settings;
            st.fadeSeconds = 0.0f;
            st.delaySeconds = 0.0f;
            g.freeLfo[l] = freeLfos[l].advance(st, numSamples, sampleRate, g.customShape[l]) * lp.amount;
        }
        else
        {
            g.freeLfo[l] = 0.0f;
        }
    }

    // Slots that target master and effect parameters can only use sources that exist without a voice.
    std::array<float, kNumSourcesExt> sources{};
    for (size_t m = 0; m < kNumMacros; ++m)
        sources[static_cast<size_t>(macroSource(static_cast<int>(m)))] = g.macro[m];
    sources[static_cast<size_t>(ModSource::StripL)] = g.strip[0];
    sources[static_cast<size_t>(ModSource::StripR)] = g.strip[1];
    sources[static_cast<size_t>(ModSource::ModWheel)] = g.modWheel;
    sources[static_cast<size_t>(ModSource::Aftertouch)] = g.aftertouch;
    sources[static_cast<size_t>(ModSource::SceneMorph)] = g.morph;
    sources[static_cast<size_t>(ModSource::PitchBend)] = g.pitchBend;
    for (size_t l = 0; l < kNumLfos; ++l)
        sources[static_cast<size_t>(kLfoSources[l])] = g.freeLfo[l];
    // The layer and global modulators of the bank: one evaluation for all voices.
    g.bank.fill(0.0f);
    if (params.modulators != nullptr && bankSet.anyShared)
    {
        ModulatorContext ctx;
        ctx.numSamples = numSamples;
        ctx.sampleRate = sampleRate;
        ctx.voice = false;
        ctx.gate = heldCount > 0 || (graphVoices() && activeProgram->activeVoices() > 0);
        ctx.velocity = sharedVelocity;
        ctx.note = sharedNote;
        ctx.noteOn = noteOnCount != noteOnSeen;
        ctx.audio = sharedMixPrev.data();
        ctx.audioCount = sharedMixPrevCount;
        noteOnSeen = noteOnCount;
        evaluateModulators(bankSet, sharedBank.data(), ctx, sources.data());
        for (size_t n = 0; n < static_cast<size_t>(kNumBankModulators); ++n)
            if (bankSet.mod[n].scope != ModulatorScope::Voice)
                g.bank[n] = sources[static_cast<size_t>(kExtModulatorFirst) + n];
    }
    globalDest.fill(0.0f);
    accumulateModulation(params.slots, sources.data(), globalDest.data());

    // Matrix slots aimed at generic targets (destinations after the fixed ModDest list).
    std::fill(genericDest.begin(), genericDest.end(), 0.0f);
    for (const auto& slot : params.slots)
    {
        const int target = slot.destination - kNumModDests;
        if (slot.source > 0 && slot.source < kNumModSources && target >= 0 &&
            target < static_cast<int>(genericDest.size()) && slot.amount != 0.0f)
            genericDest[static_cast<size_t>(target)] += evaluateSlot(slot, sources[static_cast<size_t>(slot.source)]);
    }

    // Assignable routes.
    params.routes = nullptr;
    params.numRoutes = 0;
    params.globalExtra = nullptr;
    params.targetRoutes = nullptr;
    params.numTargetRoutes = 0;
    params.globalExtraTargets = nullptr;
    if (pinnedRoutes != nullptr && !pinnedRoutes->routes.empty())
    {
        const auto& set = *pinnedRoutes;
        params.targetRoutes = set.voiceTargetSlots.data();
        params.numTargetRoutes = static_cast<int>(set.voiceTargetSlots.size());
        params.routes = set.voiceSlots.data();
        params.numRoutes = static_cast<int>(set.voiceSlots.size());
        accumulateRoutesGlobal(set, sources.data(), numSamples);
    }
}

namespace
{
// A clip or expression route is evaluated per voice when it aims at a per-voice destination and, for clips, the
// clip is voice-triggered.
bool isPerVoiceRoute(const RouteSet& set, size_t route, const ClipBank* bank)
{
    const int dest = set.voiceDest[route];
    if (dest <= 0 || dest == static_cast<int>(ModDest::MasterGain))
        return false;
    const int source = set.routes[route].source;
    if (isMpeSource(source))
        return true;
    if (bank == nullptr)
        return false;
    const Clip* c = bank->clipOf(source);
    return c != nullptr && c->trigger == ClipTrigger::Voice;
}

float mpeValue(const SynthEngine::Expression& e, int source)
{
    return source == kSourceMpePressure ? e.pressure : source == kSourceMpeSlide ? e.slide : e.bend;
}
} // namespace

size_t SynthEngine::newestVoice() const
{
    size_t best = 0;
    std::uint64_t age = 0;
    for (size_t i = 0; i < voices.size(); ++i)
        if (voices[i].isActive() && voices[i].age() >= age)
        {
            age = voices[i].age();
            best = i;
        }
    return best;
}

const float* SynthEngine::voiceRouteExtra(size_t vi, const Voice& v, int numSamples)
{
    clipPlayer.beginVoice(static_cast<int>(vi), v.age(), v.isActive(), v.isReleasing());
    voiceTargetExtraPtr = baseExtraTargets;
    if (!v.isActive() || pinnedRoutes == nullptr || pinnedRoutes->dynamicRoutes.empty())
        return baseExtra;
    bool any = false;
    bool anyTargets = false;
    Expression expr;
    bool haveExpr = false;
    for (const int ri : pinnedRoutes->dynamicRoutes)
    {
        const auto r = static_cast<size_t>(ri);
        if (!isPerVoiceRoute(*pinnedRoutes, r, pinnedClips))
            continue;
        if (!any)
        {
            if (baseExtra != nullptr)
                std::copy(baseExtra, baseExtra + kNumModDests, voiceExtra.begin());
            else
                voiceExtra.fill(0.0f);
            any = true;
        }
        const auto& route = pinnedRoutes->routes[r];
        float raw;
        if (isMpeSource(route.source))
        {
            if (!haveExpr)
            {
                expr = expressionOf(vi);
                haveExpr = true;
            }
            raw = mpeValue(expr, route.source);
        }
        else
            raw = clipPlayer.voiceValue(static_cast<int>(vi), route.source);
        // The amount moves with the routes aimed at it, evaluated once per chunk by accumulateRoutesGlobal().
        const float amount =
            pinnedRoutes->amountMods.empty() ? route.amount : std::clamp(route.amount + routeDelta[r], -1.0f, 1.0f);
        const ModSlotParams slot{route.source == kSourceMpeBend ? static_cast<int>(ModSource::Key)
                                                                : static_cast<int>(ModSource::ModWheel),
                                 1, amount, route.polarity, route.curve};
        float value = evaluateSlot(slot, raw);
        if (isClipSource(route.source))
            value *= clipPlayer.gate((route.source - kSourceClip) / kMaxClipLanes);
        if (route.smoothingMs > 0.0f)
        {
            float& state = voiceRouteSmooth[vi * kMaxRoutes + r];
            const float a = 1.0f - std::exp(-static_cast<float>(numSamples) /
                                            (route.smoothingMs * 0.001f * static_cast<float>(sampleRate)));
            state += (value - state) * a;
            value = state;
        }
        const int vt = r < pinnedRoutes->voiceTarget.size() ? pinnedRoutes->voiceTarget[r] : -1;
        if (vt >= 0)
        {
            // A clip lane or MPE source aimed at a module parameter that the voice modulates itself.
            if (!anyTargets)
            {
                if (baseExtraTargets != nullptr)
                    std::copy(baseExtraTargets, baseExtraTargets + ParameterMap::kMaxVoiceTargets, voiceTargetExtra.begin());
                else
                    voiceTargetExtra.fill(0.0f);
                anyTargets = true;
            }
            voiceTargetExtra[static_cast<size_t>(vt)] += value;
        }
        else
            voiceExtra[static_cast<size_t>(pinnedRoutes->voiceDest[r])] += value;
    }
    if (anyTargets)
        voiceTargetExtraPtr = voiceTargetExtra.data();
    return any ? voiceExtra.data() : baseExtra;
}

void SynthEngine::accumulateRoutesGlobal(const RouteSet& set, const float* sources, int numSamples)
{
    routeVoiceGlobal.fill(0.0f);
    bool anyVoiceGlobal = false;
    bool anyTargetGlobal = false;
    std::fill(routeTargetGlobal.begin(), routeTargetGlobal.end(), 0.0f);
    const size_t n = std::min(set.routes.size(), static_cast<size_t>(kMaxRoutes));
    // Route amounts that other routes move (modulation of modulation), in dependency order.
    const bool haveDelta = !set.amountMods.empty();
    if (haveDelta)
    {
        routeDelta.fill(0.0f);
        for (const auto& m : set.amountMods)
        {
            ModSlotParams ms = m.slot;
            ms.amount = std::clamp(ms.amount + routeDelta[static_cast<size_t>(m.route)], -1.0f, 1.0f);
            routeDelta[static_cast<size_t>(m.target)] +=
                evaluateSlot(ms, sources[static_cast<size_t>(ms.source)], extSourceBipolar(&bankSet, ms.source));
        }
    }
    for (size_t i = 0; i < n; ++i)
    {
        const auto& r = set.routes[i];
        const int ext = extSourceIndex(r.source);
        const bool cc = r.source >= kSourceCC && r.source < kSourceCC + 128;
        const bool clip = isClipSource(r.source);
        const bool mpe = isMpeSource(r.source);
        if ((clip || mpe) && isPerVoiceRoute(set, i, pinnedClips))
            continue; // evaluated per voice, see voiceRouteExtra()
        float raw = 0.0f;
        if (cc)
            raw = controllers[static_cast<size_t>(r.source - kSourceCC)];
        else if (clip)
            raw = clipPlayer.globalValue(r.source);
        else if (mpe)
            raw = mpeValue(expressionOf(newestVoice()), r.source);
        else if (ext > 0)
            raw = sources[static_cast<size_t>(ext)];
        const float amount = haveDelta ? std::clamp(r.amount + routeDelta[i], -1.0f, 1.0f) : r.amount;
        ModSlotParams slot{r.source == kSourceMpeBend ? static_cast<int>(ModSource::Key)
                           : cc || clip || mpe        ? static_cast<int>(ModSource::ModWheel)
                                                      : (ext > 0 ? ext : r.source),
                           1, amount, r.polarity, r.curve};
        float v = evaluateSlot(slot, raw, extSourceBipolar(&bankSet, slot.source));
        if (clip)
            v *= clipPlayer.gate((r.source - kSourceClip) / kMaxClipLanes);
        if (r.smoothingMs > 0.0f)
        {
            const float a =
                1.0f - std::exp(-static_cast<float>(numSamples) / (r.smoothingMs * 0.001f * static_cast<float>(sampleRate)));
            routeSmooth[i] += (v - routeSmooth[i]) * a;
            v = routeSmooth[i];
        }
        const int voiceTarget = i < set.voiceTarget.size() ? set.voiceTarget[i] : -1;
        if (voiceTarget >= 0)
        {
            if (cc || clip || mpe)
            {
                routeTargetGlobal[static_cast<size_t>(voiceTarget)] += v;
                anyTargetGlobal = true;
            }
            continue;
        }
        const int dest = set.voiceDest[i];
        const int global = set.globalTarget[i];
        if (global >= 0 && global < static_cast<int>(genericDest.size()))
            genericDest[static_cast<size_t>(global)] += v;
        else if (dest > 0)
        {
            if (cc || clip || mpe)
            {
                routeVoiceGlobal[static_cast<size_t>(dest)] += v;
                anyVoiceGlobal = true;
            }
            globalDest[static_cast<size_t>(dest)] += v; // master gain and other global destinations
        }
    }
    if (anyVoiceGlobal)
        params.globalExtra = routeVoiceGlobal.data();
    if (anyTargetGlobal)
        params.globalExtraTargets = routeTargetGlobal.data();
}

bool SynthEngine::sampleSnapshot(int slot, SampleView& out) const
{
    const bool found = samples.withSample(slot,
                                          [&](const SampleData& d)
                                          {
                                              out.name = d.name;
                                              out.sampleRate = d.sampleRate;
                                              out.frames = d.frames;
                                              out.detectedNote = d.detectedNote;
                                              out.peakMin = d.peakMin;
                                              out.peakMax = d.peakMax;
                                          });
    if (!found)
        return false;
    out.slices = samples.slices(slot);
    out.zones = zonesBank.zones(slot);
    out.visual = OscVisual{};
    return true;
}

bool SynthEngine::sampleView(int slot, SampleView& out) const
{
    if (!sampleSnapshot(slot, out))
        return false;
    for (const auto& v : voices)
        if (v.isActive())
            if (const auto* inst = v.oscillatorInstance(slot, 1))
                inst->visual(out.visual);
    return true;
}

bool SynthEngine::cloudView(int slot, CloudSnapshot& out) const
{
    out = CloudSnapshot{};
    if (!clouds.withModel(slot, [&](const CloudModel& m) { out.points = m.points; }))
        return false;
    out.energy.assign(out.points.size(), 0.0f);
    const auto types = reg->oscillatorTypes();
    int cloudType = -1;
    for (size_t t = 0; t < types.size(); ++t)
        if (types[t] != nullptr && types[t]->id == "osc.cloud")
            cloudType = static_cast<int>(t);
    std::vector<float> e;
    for (const auto& v : voices)
    {
        if (!v.isActive())
            continue;
        auto* inst = v.oscillatorInstance(slot, cloudType);
        auto* view = dynamic_cast<CloudOscillatorView*>(inst);
        if (view == nullptr)
            continue;
        auto& osc = view->cloudOscillator();
        if (!osc.isActive())
            continue;
        osc.energies(e, out.energy.size());
        for (size_t i = 0; i < e.size(); ++i)
            out.energy[i] = std::max(out.energy[i], e[i]);
        out.position = osc.lastMorphPosition();
        ++out.activeNotes;
    }
    return true;
}

float SynthEngine::modulatedValue(int paramIndex) const
{
    if (paramIndex < 0 || static_cast<size_t>(paramIndex) >= current.size())
        return 0.0f;
    return modulated[static_cast<size_t>(paramIndex)].load(std::memory_order_relaxed);
}

float SynthEngine::baseValue(int paramIndex) const
{
    if (paramIndex < 0 || static_cast<size_t>(paramIndex) >= current.size())
        return 0.0f;
    return published[static_cast<size_t>(paramIndex)].load(std::memory_order_relaxed);
}

// A parameter's value under one voice's modulation: dest is the fixed matrix destination (ModDest) the parameter reads, or
// 0 for a per-voice parameter target, and m the voice's modulation of that destination or target.
float SynthEngine::applyVoiceMod(int paramIndex, float base, int dest, float m) const
{
    const ParamDef& def = (*table)[static_cast<size_t>(paramIndex)];
    const float range = def.max - def.min;
    if (dest == 0)
        return std::clamp(base + m * range, def.min, def.max);
    switch (static_cast<ModDest>(dest))
    {
    case ModDest::Osc1Pitch:
    case ModDest::Osc2Pitch:
    case ModDest::Osc3Pitch:
    case ModDest::Pitch:
        return std::clamp(base + 24.0f * m, def.min, def.max);
    case ModDest::Cutoff:
    case ModDest::Cutoff2:
        return std::clamp(base * std::exp2(5.0f * m), def.min, def.max);
    default:
        return std::clamp(base + m * range, def.min, def.max);
    }
}

float SynthEngine::voiceModulatedValue(int paramIndex) const
{
    const float base = modulatedValue(paramIndex);
    if (paramIndex < 0 || static_cast<size_t>(paramIndex) >= paramDest.size())
        return base;
    const int vt = params_->voiceTargetOfParam(paramIndex);
    if (vt >= 0)
        return applyVoiceMod(paramIndex, base, 0, voiceTargetMod[static_cast<size_t>(vt)].load(std::memory_order_relaxed));
    const int dest = paramDest[static_cast<size_t>(paramIndex)];
    if (dest <= 0 || dest >= static_cast<int>(voiceDestMod.size()))
        return base;
    return applyVoiceMod(paramIndex, base, dest, voiceDestMod[static_cast<size_t>(dest)].load(std::memory_order_relaxed));
}

void SynthEngine::clipPositions(std::array<double, kMaxClips>& out) const
{
    for (int c = 0; c < kMaxClips; ++c)
        out[static_cast<size_t>(c)] = clipPlayer.publishedGlobalPosition(c);
}

void SynthEngine::voiceClipPositions(int clip, std::vector<double>& out) const
{
    out.assign(static_cast<size_t>(clipPlayer.voiceCount()), -1.0);
    for (int v = 0; v < clipPlayer.voiceCount(); ++v)
        out[static_cast<size_t>(v)] = clipPlayer.publishedVoicePosition(v, clip);
}

void SynthEngine::renderClassicVoices(float* left, float* right, int n, float bend)
{
    // A filter slot with one of the nonlinear models (index 17 and up) keeps its state in the bank for all voices (it runs
    // there while the slot is on, so switching the slot off freezes it as it freezes a voice's own filter); with none,
    // every voice renders alone exactly as before.
    const int slots = std::min(params.numFilter, numFilt);
    bool useBank = cfg.batchAnalogFilters && filterBank.ready();
    bool anyBanked = false;
    for (int f = 0; f < slots && useBank; ++f)
        anyBanked = anyBanked || FilterChannel::isAnalogModel(params.filter[static_cast<size_t>(f)].model);
    useBank = useBank && anyBanked;
    if (useBank)
        filterBank.beginChunk();
    for (size_t vi = 0; vi < voices.size(); ++vi)
    {
        params.globalExtra = voiceRouteExtra(vi, voices[vi], n);
        params.globalExtraTargets = voiceTargetExtraPtr;
        float pitch = bend;
        if (voices[vi].isActive())
        {
            if (pinnedTuning != nullptr)
                pitch += tuneAmount * pinnedTuning->semitones[static_cast<size_t>(std::clamp(voices[vi].note(), 0, 127))];
            if (voiceChannel[vi] >= 0)
                pitch += chBend[static_cast<size_t>(voiceChannel[vi])] * mpeRange;
        }
        const float baseGlide = params.glide;
        params.glide = baseGlide * voiceGlideScale[vi];
        if (useBank)
            stagedVoice[vi] = voices[vi].beginChunk(params, globalMod, pitch, n, &filterBank, static_cast<int>(vi)) ? 1 : 0;
        else
            voices[vi].renderChunk(params, globalMod, pitch, n, left, right);
        params.glide = baseGlide;
    }
    if (!useBank)
        return;
    for (int f = 0; f < slots; ++f)
    {
        for (size_t vi = 0; vi < voices.size(); ++vi)
            if (stagedVoice[vi] != 0)
                voices[vi].filterIn(f);
        const auto& fp = params.filter[static_cast<size_t>(f)];
        if (fp.on && FilterChannel::isAnalogModel(fp.model))
            filterBank.process(f,
                               std::clamp(fp.model, 0, static_cast<int>(FilterModel::Count) - 1) -
                                   static_cast<int>(FilterModel::TransistorLadder),
                               n);
        for (size_t vi = 0; vi < voices.size(); ++vi)
            if (stagedVoice[vi] != 0)
                voices[vi].filterOut(f);
    }
    for (size_t vi = 0; vi < voices.size(); ++vi)
        if (stagedVoice[vi] != 0)
            voices[vi].endChunk(left, right);
}

void SynthEngine::ClassicHost::renderClassicChunk(float* left, float* right, int n)
{
    engine->renderClassicVoices(left, right, n, engine->chunkBend);
}

void SynthEngine::renderProgramVoices(float* left, float* right, int n, float bend)
{
    activeProgram->setCardAmounts(params.card);
    switch (activeProgram->voiceKind())
    {
    case graph::VoiceKind::Classic:
    {
        // The classic voice is a node of the graph; it calls back into the engine's voice pool.
        chunkBend = bend;
        graph::AudioBuffer out{left, right};
        graph::NodeIO io;
        io.out = &out;
        io.numOut = 1;
        activeProgram->classicNode()->process(io, n);
        break;
    }
    case graph::VoiceKind::None:
        break;
    case graph::VoiceKind::Graph:
        activeProgram->render(left, right, n, current.data(), &params, bend);
        break;
    }
}

// Builds the program of the product on the control thread and makes it the active one at once (no fade: nothing has
// played yet, or the engine is being prepared).
void SynthEngine::installProgram()
{
    if (!productNow)
        return;
    ModuleContext context;
    context.sampleRate = sampleRate;
    context.maxBlockSize = cfg.maxBlockSize;
    context.quality = cfg.quality;
    context.oversampling = cfg.oversampling;
    context.seed = cfg.seed;
    context.renderOffline = &renderOffline;
    context.batchAnalogFilters = cfg.batchAnalogFilters;
    auto program = std::make_unique<graph::Program>(productNow, fx, context, cfg.seed, &classicHost);
    if (!program->ok())
        throw std::logic_error("the compiled product does not fit the engine: " + program->error());
    graph::Program* raw = programHost.publish(std::move(program));
    activeProgram = raw;
    programHost.hold(0, raw);
    programHost.endPoll();
    fx.setPlan(&raw->fxPlan());
    if (graphVoices())
        for (auto& v : voices)
            v.kill();
    programHost.collect();
}

bool SynthEngine::reconfigure(std::shared_ptr<const graph::CompiledProduct> product, std::string* error)
{
    const auto fail = [&](const std::string& m)
    {
        if (error != nullptr)
            *error = m;
        return false;
    };
    if (!product)
        return fail("no product");
    std::vector<std::string> ids;
    for (const auto& p : params_->params())
        ids.push_back(p.id);
    if (product->parameterIds() != ids)
        return fail("the product has different parameters from the one this engine plays");
    ModuleContext context;
    context.sampleRate = sampleRate;
    context.maxBlockSize = cfg.maxBlockSize;
    context.quality = cfg.quality;
    context.oversampling = cfg.oversampling;
    context.seed = cfg.seed;
    context.renderOffline = &renderOffline;
    context.batchAnalogFilters = cfg.batchAnalogFilters;
    auto program = std::make_unique<graph::Program>(product, fx, context, cfg.seed, &classicHost);
    if (!program->ok())
        return fail(program->error());
    productNow = std::move(product);
    fx.publishReported(hostValues.data(), &program->fxPlan()); // before audio runs, a host reads this; a block corrects it
    programHost.publish(std::move(program));
    return true;
}

// Audio thread, at the start of a chunk: picks up a newly published program, starts the fade, and swaps the program in
// once the output is silent.
void SynthEngine::pollProgram()
{
    graph::Program* p = programHost.poll();
    if (p != nullptr && p != activeProgram && p != incomingProgram)
    {
        incomingProgram = p;
        programHost.hold(1, p);
    }
    // A program that arrived while an earlier switch was still fading in waits for the fade to end, then gets its own.
    if (incomingProgram != nullptr && fade == Fade::Idle)
    {
        fade = Fade::Out;
        fadePos = 0;
        fadeState.store(1, std::memory_order_relaxed);
    }
    programHost.endPoll();
    if (fade == Fade::Out && fadePos >= fadeLength)
        swapProgram();
}

void SynthEngine::swapProgram()
{
    graph::Program* old = activeProgram;
    // The old program is silenced while it is still held: once hold(0) moves on, the control thread may free it.
    if (old != nullptr)
        old->killAll();
    activeProgram = incomingProgram;
    incomingProgram = nullptr;
    programHost.hold(0, activeProgram);
    programHost.hold(1, nullptr);
    fx.setPlan(&activeProgram->fxPlan());
    // The classic voices do not sound in a graph product; their notes end with the fade.
    if (graphVoices())
        for (auto& v : voices)
            v.kill();
    heldCount = 0;
    fade = Fade::In;
    fadePos = 0;
    fadeState.store(2, std::memory_order_relaxed);
}

void SynthEngine::applyFade(float* left, float* right, int n)
{
    const float inv = 1.0f / static_cast<float>(fadeLength);
    for (int i = 0; i < n; ++i)
    {
        const float x = std::min(1.0f, static_cast<float>(fadePos + i) * inv);
        const float g = fade == Fade::Out ? 1.0f - x : x;
        left[i] *= g;
        right[i] *= g;
    }
    fadePos += n;
    if (fade == Fade::In && fadePos >= fadeLength)
    {
        fade = Fade::Idle;
        fadeState.store(0, std::memory_order_relaxed);
    }
}

bool SynthEngine::setFormantTable(int slot, const FormantTable* user, std::string& error)
{
    if (slot < 0 || slot >= kMaxFilterSlots)
    {
        error = "filter_slot out of range";
        return false;
    }
    std::lock_guard<std::mutex> lock(formantMutex);
    auto next = std::make_unique<FormantSet>();
    if (const FormantSet* cur = formantSets.latest())
        *next = *cur;
    next->used[static_cast<size_t>(slot)] = user != nullptr;
    if (user != nullptr)
        next->table[static_cast<size_t>(slot)] = *user;
    formantSets.publish(std::move(next));
    return true;
}

bool SynthEngine::setFormantSaturation(int slot, bool on, std::string& error)
{
    if (slot < 0 || slot >= kMaxFilterSlots)
    {
        error = "filter_slot out of range";
        return false;
    }
    std::lock_guard<std::mutex> lock(formantMutex);
    const FormantSet* cur = formantSets.latest();
    if (cur == nullptr || !cur->used[static_cast<size_t>(slot)])
    {
        error = "the slot has no user formant table";
        return false;
    }
    auto next = std::make_unique<FormantSet>();
    *next = *cur;
    next->table[static_cast<size_t>(slot)].saturate = on;
    formantSets.publish(std::move(next));
    return true;
}

bool SynthEngine::formantTable(int slot, FormantTable& out) const
{
    if (slot < 0 || slot >= kMaxFilterSlots)
        return false;
    bool found = false;
    formantSets.withLatest(
        [&](const FormantSet& s)
        {
            if (s.used[static_cast<size_t>(slot)])
            {
                out = s.table[static_cast<size_t>(slot)];
                found = true;
            }
        });
    return found;
}

void SynthEngine::render(float* left, float* right, int numSamples)
{
    const ScopedFlushDenormals noDenormals; // recursive filters and delay lines must never run on denormals
    std::fill(left, left + numSamples, 0.0f);
    std::fill(right, right + numSamples, 0.0f);
    userTables.acquire(pinnedTables);
    pinnedFormants = formantSets.acquire();
    samples.acquire(pinnedSamples);
    samples.acquireSlices(pinnedSlices);
    zonesBank.acquire(pinnedZones);
    clouds.acquire(pinnedClouds);
    for (size_t i = 0; i < static_cast<size_t>(kMaxOscSlots); ++i)
    {
        params.sample[i] = pinnedSamples[i];
        params.slices[i] = pinnedSlices[i];
        params.zones[i] = i < pinnedZones.size() ? pinnedZones[i] : nullptr;
        params.cloud[i] = pinnedClouds[i];
    }
    pinnedBank = sceneSource.load(std::memory_order_acquire);
    pinnedScenes = pinnedBank != nullptr ? pinnedBank->acquire() : nullptr;
    pinnedRoutes = routes.acquire();
    pinnedClips = clipStore.acquire();
    pinnedTuning = tuningStore.acquire();

    // The control rate runs on an absolute 32-sample grid: a call (a host block, or a segment between two MIDI
    // events) that starts or ends inside a grid cell renders a shorter chunk and the parameter smoothers step only
    // when a cell begins, so smoothing times do not depend on how the host cuts the audio.
    for (int pos = 0; pos < numSamples;)
    {
        const int n = std::min(kChunkSize - controlPhase, numSamples - pos);
        updateVector(n);
        if (controlPhase == 0)
            stepSmoothers();
        buildParams();
        params.userTables = &pinnedTables;
        for (size_t f = 0; f < params.formants.size(); ++f)
            params.formants[f] = pinnedFormants != nullptr && pinnedFormants->used[f] ? &pinnedFormants->table[f] : nullptr;
        const float bend = pitchBendNorm * bendRange;
        clipPlayer.applyTransport(clipWanted.load(std::memory_order_relaxed),
                                  clipRestart.exchange(0, std::memory_order_acq_rel), n);
        clipPlayer.beginChunk(pinnedClips, n, tempoBpm, heldCount > 0);
        if (motionRecorder.armed())
        {
            const int mc = motionRecorder.targetClip();
            motionRecorder.capture(n, tempoBpm, current.data(),
                                   motionRecorder.followsHead() ? clipPlayer.globalPosition(mc) : -1.0);
        }
        updateGlobalMod(n);
        baseExtra = params.globalExtra;
        baseExtraTargets = params.globalExtraTargets;
        pollProgram();
        if (activeProgram == nullptr)
            renderClassicVoices(left + pos, right + pos, n, bend);
        else
            renderProgramVoices(left + pos, right + pos, n, bend);
        params.globalExtra = baseExtra;
        params.globalExtraTargets = baseExtraTargets;
        if (params.modulators != nullptr && bankSet.needsAudio && bankSet.anyShared)
        {
            // The layer and global followers and pitch trackers listen to the voices' sum of the chunk just rendered.
            for (int i = 0; i < n; ++i)
                sharedMixPrev[static_cast<size_t>(i)] = 0.5f * (left[pos + i] + right[pos + i]);
            sharedMixPrevCount = n;
        }
        clipPlayer.endChunk();
        const float gain = masterGain * dbGain(24.0f * globalDest[static_cast<size_t>(ModDest::MasterGain)]);
        for (int i = pos; i < pos + n; ++i)
        {
            left[i] *= gain;
            right[i] *= gain;
        }
        // The noise floor and hum of the analog stage: levels are those of the output, ahead of the effects.
        if (idx.noiseOn >= 0)
        {
            const auto at = [this](int i) { return current[static_cast<size_t>(i)]; };
            noiseHum.add(left + pos, right + pos, n, at(idx.noiseOn) > 0.5f, at(idx.noiseDb), at(idx.humOn) > 0.5f,
                         at(idx.humDb), at(idx.humHz) > 0.5f ? 60.0f : 50.0f);
        }
        fx.setNote(lastNote);
        fx.process(left + pos, right + pos, n, current.data(), globalDest.data(), genericDest.data(), tempoBpm,
                   auxLeft != nullptr ? auxLeft + pos : nullptr, auxRight != nullptr ? auxRight + pos : nullptr);
        if (fade != Fade::Idle)
            applyFade(left + pos, right + pos, n);
        pos += n;
        controlPhase = (controlPhase + n) % kChunkSize;
    }
    auxLeft = auxRight = nullptr;
    fx.publishReported(current.data());
    clipPlayer.publishPositions();
    publishVoiceCount();
    {
        const size_t nv = newestVoice();
        const bool any = nv < voices.size() && voices[nv].isActive();
        const auto& vd = voices[nv].lastDestMod();
        const auto& vtm = voices[nv].lastTargetMod();
        const size_t nt = params_->voiceTargets().size();
        for (size_t d = 0; d < voiceDestMod.size(); ++d)
            voiceDestMod[d].store(any ? vd[d] : 0.0f, std::memory_order_relaxed);
        for (size_t t = 0; t < nt; ++t)
            voiceTargetMod[t].store(any ? vtm[t] : 0.0f, std::memory_order_relaxed);
        for (size_t m = 0; m < static_cast<size_t>(kNumBankModulators); ++m)
            bankPublished[m].store(bankSet.mod[m].scope == ModulatorScope::Voice
                                       ? (any ? voices[nv].lastModulators()[m] : 0.0f)
                                       : globalMod.bank[m],
                                   std::memory_order_relaxed);
    }
    params.userTables = nullptr;
    params.formants.fill(nullptr);
    formantSets.release();
    pinnedFormants = nullptr;
    userTables.release();
    samples.release();
    samples.releaseSlices();
    zonesBank.release();
    clouds.release();
    params.sample.fill(nullptr);
    params.slices.fill(nullptr);
    params.zones.fill(nullptr);
    params.cloud.fill(nullptr);
    params.routes = nullptr;
    params.numRoutes = 0;
    params.globalExtra = nullptr;
    params.targetRoutes = nullptr;
    params.numTargetRoutes = 0;
    params.globalExtraTargets = nullptr;
    params.routeSet = nullptr;
    params.modulators = nullptr;
    if (pinnedBank != nullptr)
        pinnedBank->release();
    pinnedBank = nullptr;
    pinnedScenes = nullptr;
    pinnedRoutes = nullptr;
    routes.release();
    pinnedClips = nullptr;
    clipStore.release();
    pinnedTuning = nullptr;
    tuningStore.release();

    // Read-back for UIs: global modulation applied to the generic targets.
    const auto& defs = *table;
    for (size_t i = 0; i < current.size(); ++i)
    {
        modulated[i].store(current[i], std::memory_order_relaxed);
        published[i].store(current[i], std::memory_order_relaxed);
    }
    const auto& targets = params_->modTargets();
    for (size_t k = 0; k < targets.size(); ++k)
    {
        const auto g = static_cast<size_t>(targets[k]);
        if (params_->voiceTargetOfParam(targets[k]) >= 0)
            continue; // modulated per voice: the read-back keeps the unmodulated value
        const float v = std::clamp(current[g] + genericDest[k] * (defs[g].max - defs[g].min), defs[g].min, defs[g].max);
        modulated[g].store(v, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Vector field overlay and clip transport.

namespace
{
constexpr double kPi = 3.14159265358979323846;

float rand01(std::uint32_t& s)
{
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return static_cast<float>(s >> 8) * (1.0f / 16777216.0f);
}
} // namespace

void SynthEngine::updateVector(int numSamples)
{
    const bool on = current[static_cast<size_t>(vec.on)] > 0.5f;
    const SceneSnapshot* scenes = pinnedScenes;
    std::array<int, 4> slots{};
    bool anyCorner = false;
    for (size_t k = 0; k < 4; ++k)
    {
        slots[k] = static_cast<int>(current[static_cast<size_t>(vec.corner[k])] + 0.5f);
        anyCorner = anyCorner || (scenes != nullptr && slots[k] >= 0 && slots[k] < kNumScenes &&
                                  scenes->filled[static_cast<size_t>(slots[k])]);
    }
    const bool live = on && anyCorner;
    if (!live)
    {
        cacheValid = false;
        if (lastLive)
        {
            // Back to the host's values: the smoothers glide there.
            std::fill(overlayExact.begin(), overlayExact.end(), -1.0f);
            for (const int i : sceneManaged)
            {
                const auto& def = (*table)[static_cast<size_t>(i)];
                if (vectorExclude != nullptr && vectorExclude[i] != 0)
                    continue;
                const float v = std::clamp(hostValues[static_cast<size_t>(i)], def.min, def.max);
                const float target = def.smoothing == Smoothing::Log ? std::log2(std::max(v, kLogFloor)) : v;
                if (def.smoothing == Smoothing::None)
                {
                    smoothers[static_cast<size_t>(i)].setCurrentAndTargetValue(target);
                    current[static_cast<size_t>(i)] = v;
                }
                else
                    smoothers[static_cast<size_t>(i)].setTargetValue(target);
            }
            lastLive = false;
            overlayLive.store(false, std::memory_order_relaxed);
        }
        vecWasOn = on;
        return;
    }

    const double seconds = static_cast<double>(numSamples) / sampleRate;
    const auto at = [&](int index) { return current[static_cast<size_t>(index)]; };
    const auto& defs = *table;
    // The puck as set (host / routes / recorded paths), then orbit and drift around it.
    float px = at(vec.x), py = at(vec.y);
    if (vec.targetX >= 0)
        px = std::clamp(px + genericDest[static_cast<size_t>(vec.targetX)] *
                                 (defs[static_cast<size_t>(vec.x)].max - defs[static_cast<size_t>(vec.x)].min),
                        0.0f, 1.0f);
    if (vec.targetY >= 0)
        py = std::clamp(py + genericDest[static_cast<size_t>(vec.targetY)] *
                                 (defs[static_cast<size_t>(vec.y)].max - defs[static_cast<size_t>(vec.y)].min),
                        0.0f, 1.0f);

    const float depth = at(vec.orbitDepth);
    if (depth > 0.0f)
    {
        orbitPhase += static_cast<double>(at(vec.orbitRate)) * seconds;
        orbitPhase -= std::floor(orbitPhase);
        const double t = 2.0 * kPi * orbitPhase;
        float ox = 0.0f, oy = 0.0f;
        switch (static_cast<int>(at(vec.orbitShape) + 0.5f))
        {
        case 0:
            ox = static_cast<float>(std::cos(t));
            oy = static_cast<float>(std::sin(t));
            break;
        case 1:
            ox = static_cast<float>(std::sin(t));
            oy = static_cast<float>(0.5 * std::sin(2.0 * t));
            break;
        case 2:
            ox = static_cast<float>(std::sin(t));
            break;
        default:
            oy = static_cast<float>(std::sin(t));
            break;
        }
        px += ox * depth;
        py += oy * depth;
    }
    const float driftAmount = at(vec.drift);
    if (driftAmount > 0.0f)
    {
        const double period = 1.0 / std::max(static_cast<double>(at(vec.driftRate)), 0.02);
        driftTimer += seconds;
        if (driftTimer >= period)
        {
            driftTimer -= period;
            driftTargetX = rand01(vecRng) * 2.0f - 1.0f;
            driftTargetY = rand01(vecRng) * 2.0f - 1.0f;
        }
        const float a = 1.0f - std::exp(-static_cast<float>(seconds / (period * 0.5)));
        driftX += (driftTargetX - driftX) * a;
        driftY += (driftTargetY - driftY) * a;
        px += driftX * driftAmount;
        py += driftY * driftAmount;
    }
    px = std::clamp(px, 0.0f, 1.0f);
    py = std::clamp(py, 0.0f, 1.0f);

    if (!lastLive || !vecWasOn)
    {
        vecX = px;
        vecY = py;
    }
    const float tau = at(vec.smooth);
    const float a = tau > 0.0f ? 1.0f - std::exp(-static_cast<float>(seconds) / tau) : 1.0f;
    vecX += (px - vecX) * a;
    vecY += (py - vecY) * a;
    if (std::abs(px - vecX) < 1.0e-6f)
        vecX = px;
    if (std::abs(py - vecY) < 1.0e-6f)
        vecY = py;
    shownX.store(vecX, std::memory_order_relaxed);
    shownY.store(vecY, std::memory_order_relaxed);

    const auto apply = [&](int i, float v)
    {
        const auto& def = defs[static_cast<size_t>(i)];
        v = std::clamp(v, def.min, def.max);
        const float target = def.smoothing == Smoothing::Log ? std::log2(std::max(v, kLogFloor)) : v;
        if (def.smoothing == Smoothing::None)
        {
            smoothers[static_cast<size_t>(i)].setCurrentAndTargetValue(target);
            current[static_cast<size_t>(i)] = v;
        }
        else
        {
            smoothers[static_cast<size_t>(i)].setTargetValue(target);
            overlayExact[static_cast<size_t>(i)] = def.smoothing == Smoothing::Log ? v : -1.0f;
        }
    };
    const auto w = vectorWeights(vecX, vecY);
    if (vectorExclude != nullptr)
    {
        // A lock appearing or going away changes which parameters the overlay owns: start over.
        if (std::memcmp(lastExclude.data(), vectorExclude, defs.size()) != 0)
        {
            std::memcpy(lastExclude.data(), vectorExclude, defs.size());
            lastExcludeUsed = true;
            cacheValid = false;
        }
    }
    else if (lastExcludeUsed)
    {
        std::fill(lastExclude.begin(), lastExclude.end(), char(0));
        lastExcludeUsed = false;
        cacheValid = false;
    }
    if (!cacheValid || cachedGeneration != scenes->generation || cachedSlots != slots)
    {
        // Sort the scene-managed parameters: equal at all stored corners (set once), differing (blended every
        // chunk), or depending on the host value because a corner is empty.
        varying.clear();
        hostDependent.clear();
        std::array<bool, 4> stored{};
        bool allStored = true;
        for (size_t k = 0; k < 4; ++k)
        {
            stored[k] = slots[k] >= 0 && slots[k] < kNumScenes && scenes->filled[static_cast<size_t>(slots[k])];
            allStored = allStored && stored[k];
        }
        for (const int i : sceneManaged)
        {
            const auto& def = defs[static_cast<size_t>(i)];
            if (!allStored)
            {
                hostDependent.push_back(i);
                continue;
            }
            VectorParam vp;
            vp.index = i;
            vp.isFloat = def.type == ParamType::Float;
            vp.min = def.min;
            vp.range = def.max - def.min;
            vp.lo = def.min;
            vp.hi = def.max;
            vp.smoothing = def.smoothing;
            vp.linear = def.skew == 1.0f;
            vp.invSkew = 1.0f / def.skew;
            bool same = true;
            for (size_t k = 0; k < 4; ++k)
            {
                const auto s = static_cast<size_t>(slots[k]);
                vp.natural[k] = scenes->natural[s][static_cast<size_t>(i)];
                vp.norm[k] = scenes->normalised[s][static_cast<size_t>(i)];
                same = same && vp.natural[k] == vp.natural[0];
            }
            if (same)
            {
                if (vectorExclude == nullptr || vectorExclude[i] == 0)
                    apply(i, vp.natural[0]);
            }
            else if (vectorExclude == nullptr || vectorExclude[i] == 0)
                varying.push_back(vp);
        }
        varyingValue.resize(varying.size());
        cachedGeneration = scenes->generation;
        cachedSlots = slots;
        cacheValid = true;
        lastWeightX = -1.0f;
    }
    if (hostDependent.empty() && vecX == lastWeightX && vecY == lastWeightY && lastLive)
    {
        vecWasOn = true;
        return; // the puck has not moved: every target is already where the blend puts it
    }
    lastWeightX = vecX;
    lastWeightY = vecY;
    size_t heaviest = 0;
    int exact = -1;
    for (size_t k = 0; k < 4; ++k)
    {
        if (w.w[k] >= 1.0f)
            exact = static_cast<int>(k);
        if (w.w[k] > w.w[heaviest])
            heaviest = k;
    }
    // Every varying parameter: blend in normalised space (floats), or take the heaviest / exact corner. Locks are
    // already left out of the list (a change of the lock mask rebuilds it).
    const float w0 = w.w[0], w1 = w.w[1], w2 = w.w[2], w3 = w.w[3];
    const size_t count = varying.size();
    float* const out = varyingValue.data();
    if (exact >= 0)
        for (size_t k = 0; k < count; ++k)
            out[k] = varying[k].natural[static_cast<size_t>(exact)];
    else
        for (size_t k = 0; k < count; ++k)
        {
            const auto& vp = varying[k];
            if (!vp.isFloat)
            {
                out[k] = vp.natural[heaviest];
                continue;
            }
            const float n = w0 * vp.norm[0] + w1 * vp.norm[1] + w2 * vp.norm[2] + w3 * vp.norm[3];
            const float c = std::clamp(n, 0.0f, 1.0f);
            out[k] = vp.min + vp.range * (vp.linear || c == 0.0f || c == 1.0f ? c : std::pow(c, vp.invSkew));
        }
    for (size_t k = 0; k < count; ++k)
    {
        const auto& vp = varying[k];
        const float v = std::clamp(out[k], vp.lo, vp.hi);
        const size_t i = static_cast<size_t>(vp.index);
        if (vp.smoothing == Smoothing::None)
        {
            smoothers[i].setCurrentAndTargetValue(v);
            current[i] = v;
        }
        else if (vp.smoothing == Smoothing::Log)
        {
            smoothers[i].setTargetValue(std::log2(std::max(v, kLogFloor)));
            overlayExact[i] = v;
        }
        else
        {
            smoothers[i].setTargetValue(v);
            overlayExact[i] = -1.0f;
        }
    }
    for (const int i : hostDependent)
    {
        if (vectorExclude != nullptr && vectorExclude[i] != 0)
            continue;
        apply(i, vectorMorphValue(defs[static_cast<size_t>(i)], static_cast<size_t>(i), *scenes, slots, w,
                                  hostValues[static_cast<size_t>(i)]));
    }
    lastLive = true;
    vecWasOn = true;
    overlayLive.store(true, std::memory_order_relaxed);
}

void SynthEngine::startClip(int clip)
{
    if (clip < 0 || clip >= kMaxClips)
        return;
    const std::uint32_t bit = 1U << static_cast<unsigned>(clip);
    clipWanted.fetch_or(bit, std::memory_order_acq_rel);
    clipRestart.fetch_or(bit, std::memory_order_acq_rel);
}

void SynthEngine::stopClip(int clip)
{
    if (clip < 0 || clip >= kMaxClips)
        return;
    clipWanted.fetch_and(~(1U << static_cast<unsigned>(clip)), std::memory_order_acq_rel);
}

void SynthEngine::setClipRunMask(std::uint32_t mask)
{
    const std::uint32_t before = clipWanted.exchange(mask, std::memory_order_acq_rel);
    const std::uint32_t started = mask & ~before;
    if (started != 0)
        clipRestart.fetch_or(started, std::memory_order_acq_rel);
}

bool SynthEngine::isClipPlaying(int clip) const
{
    if (clip < 0 || clip >= kMaxClips ||
        (clipWanted.load(std::memory_order_acquire) & (1U << static_cast<unsigned>(clip))) == 0)
        return false;
    return clip < static_cast<int>(clipStore.snapshot().clips.size());
}

} // namespace crucible
