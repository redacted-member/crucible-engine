// SPDX-License-Identifier: MIT
#pragma once

// Assignable modulation ("drag a source onto a knob"). A route connects any source to any parameter that can be
// modulated, with amount, polarity, curve and smoothing. Routes are independent of the sixteen matrix slots
// (which remain parameters) and are stored with the patch. The list is edited on a control thread and read by the
// audio thread without locks.
//
// Targets:
//   - parameters with a fixed voice destination (oscillator, filter, LFO-rate, master gain... see
//     routeVoiceDestination()) are modulated per voice, exactly like a matrix slot to that destination;
//   - every parameter in ParameterMap::modTargets() (effect parameters and future module parameters) is modulated
//     globally: value + depth x (max - min), clamped.
// Sources: every ModSource, plus MIDI CC 0..127 (kSourceCC + cc) for controller-driven modulation, plus every lane
// of every clip (clipLaneSource(), see Clips.h), which plays unipolar 0..1, plus the per-note MPE sources, plus the
// modulator bank (kSourceModulator + n, Modulators.h) and the audio-rate sources (kSourceAudio + a). Voice-only
// sources (envelopes, velocity, key, per-note random, voice modulators, audio sources) evaluate to 0 for global targets.
//
// Modulation of modulation: a route whose target is "route:<n>:amount" (routeAmountTargetId()) adds its output to the
// amount of route n (the route's position in the list, 0-based), so depth can follow an LFO, an envelope or a modulator.
// The sum is clamped to -1 .. 1. Routes that modulate route amounts form a graph that must stay acyclic: add() refuses a
// route that would close a cycle, and the positions of later routes shift when a route is removed, so a cycle that
// only appears then is dropped when the set is compiled. Sources of such routes are the ModSources, the modulator bank
// and the audio-rate sources (not CC, clip lanes or expression).
//
// Audio-rate targets: a route from an audio-rate source (lfoN_audio, an oscillator's output, a per-sample envelope) to
// oscillator pitch (oscN_semi, voice_transpose), the FM amount (oscN_an_xmod_amt, oscN_xmod_amt), the pulse width
// (oscN_an_pw), the wavetable position (oscN_pos), a filter cutoff or the pseudo target "voice:amp" runs per sample;
// any other target gets the chunk mean of the source.

#include <crucible/ParameterMap.h>
#include <crucible/Clips.h>
#include <crucible/ModMatrix.h>
#include <crucible/Modulators.h>
#include <crucible/RtShared.h>

#include <array>
#include <atomic>
#include <functional>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kSourceCC = 1000; // route source id of MIDI CC n is kSourceCC + n
// Per-note expression of the voice that plays the note (MPE member channel or poly aftertouch): pressure 0..1,
// slide 0..1 (CC 74), bend -1..1 (of the member channel's range). Global targets follow the newest voice.
constexpr int kSourceMpe = 3000;
constexpr int kSourceMpePressure = kSourceMpe;
constexpr int kSourceMpeSlide = kSourceMpe + 1;
constexpr int kSourceMpeBend = kSourceMpe + 2;
constexpr bool isMpeSource(int source)
{
    return source >= kSourceMpe && source <= kSourceMpeBend;
}

struct ModRoute
{
    int source = 0;           // ModSource value or kSourceCC + cc
    std::string target;       // parameter id
    float amount = 0.0f;      // -1..1
    int polarity = 0;         // ModPolarity
    float curve = 0.0f;       // -1..1
    float smoothingMs = 0.0f; // one-pole smoothing of the route output (global targets)
};

// Display name of a route source ("LFO 1", "CC 74", ...).
std::string routeSourceName(int source);
std::string routeSourceName(int source, const ParameterModel& model);
// Same, naming clip lanes after the bank ("Clip 1 / Cutoff sweep").
std::string routeSourceName(int source, const ClipBank& clips, const ParameterModel* model = nullptr);
struct RouteSourceInfo
{
    int id = 0;
    std::string name;
    bool bipolar = false;   // natural range -1..1 (otherwise 0..1)
    bool voiceOnly = false; // needs a voice: evaluates to 0 for global targets (envelopes, velocity, key, random)
};
// Every source a UI can offer for drag-to-assign: matrix sources, MIDI CCs 0..127 and the lanes of the given clips.
std::vector<RouteSourceInfo> routeSourceList(const ClipBank& clips, const ParameterModel* model = nullptr);
// True for sources a route may use: matrix sources, MIDI CCs and clip lanes.
bool isRouteSource(int source, const ParameterModel* model = nullptr);
// The fixed voice destination a parameter maps to, or ModDest::None.
ModDest routeVoiceDestination(const std::string& paramId);
// "route:<n>:amount" for route n, and the n of such a target (or -1).
std::string routeAmountTargetId(int route);
int routeAmountTarget(const std::string& target);
// The pseudo target of the voice amplifier (audio-rate AM, chunk-rate amp level).
constexpr const char* kTargetVoiceAmp = "voice:amp";

// A route that modulates the amount of another route (see the header comment). slot.source is an extended source index
// (Modulators.h) and slot.amount the route's own amount.
struct AmountMod
{
    int route = 0;  // index of this route
    int target = 0; // index of the route whose amount it moves
    ModSlotParams slot;
};

enum class AudioTarget
{
    OscPitch,
    OscFmAmount,    // oscN_an_xmod_amt: the analog oscillator's own FM / ring amount
    OscCrossAmount, // oscN_xmod_amt: the voice's cross modulation from an earlier slot (wavetable oscillators)
    OscPulseWidth,
    OscPosition,
    FilterCutoff,
    Amp
};

// A route from an audio-rate source to an audio-rate target. The chunk-rate fallback (modDest / voiceTarget, as for an
// ordinary route) is what an oscillator that cannot take per-sample input is given: the chunk mean.
struct AudioRoute
{
    int source = 0; // audio source index, 0 .. kNumAudioSources - 1
    AudioTarget target = AudioTarget::Amp;
    int slot = -1; // oscillator or filter slot (0-based); -1 = every oscillator (voice_transpose) or the voice
    float amount = 0.0f;
    int route = 0;
    int modDest = 0;
    int voiceTarget = -1;
};

// Compiled route list (immutable once published).
struct RouteSet
{
    std::vector<ModRoute> routes;
    // Per route: ModDest for voice targets (or None), index into ParameterMap::modTargets() for global ones (-1).
    std::vector<int> voiceDest;
    std::vector<int> globalTarget;
    // Indices of the routes whose source is a clip lane or per-note expression (evaluated outside the matrix).
    std::vector<int> dynamicRoutes;
    // Voice targets as matrix slots (legacy sources only), handed to every voice.
    std::vector<ModSlotParams> voiceSlots;
    // Per route: dense per-voice target (ParameterMap::voiceTargets()) or -1. Such routes are not global targets.
    std::vector<int> voiceTarget;
    // Routes to per-voice targets as matrix slots: destination is the dense voice target.
    std::vector<ModSlotParams> voiceTargetSlots;
    // The route behind each entry of voiceSlots and voiceTargetSlots (for amount modulation).
    std::vector<int> voiceSlotRoute, voiceTargetSlotRoute;
    // Routes that modulate route amounts, in an order where a route comes after the routes that move its own amount.
    std::vector<AmountMod> amountMods;
    // Audio-rate routes (see the header comment).
    std::vector<AudioRoute> audioRoutes;
    // Bit n: modulator n is read by a route; bit a: audio source a is read by a route.
    std::uint32_t modulatorMask = 0;
    std::uint32_t audioMask = 0;
};

class ModRouteStore
{
public:
    // model: the parameter model of the product whose parameters the routes target; it must outlive the store.
    // Null = the process default product (deprecated singleton shim).
    explicit ModRouteStore(const ParameterModel* model = nullptr);
    const ParameterModel& parameterModel() const { return *params; }

    // Control thread. add() replaces an existing route with the same source and target. Returns false when the
    // target cannot be modulated or the list is full.
    bool add(const ModRoute& route);
    bool remove(int source, const std::string& target);
    void clear();
    std::vector<ModRoute> routes() const;
    // Routes that target one parameter (for modulation rings and amount editors).
    std::vector<ModRoute> routesFor(const std::string& target) const;
    void replaceAll(const std::vector<ModRoute>& routes);
    // Any thread. Counts the changes to the route list (add that alters a route, remove, clear, replaceAll to another list);
    // equal values mean the list did not change in between.
    std::uint32_t generation() const { return changes.load(std::memory_order_acquire); }
    // Control thread. The route at position k in routes() order, false when k is out of range.
    bool routeAt(int k, ModRoute& out) const;
    int routeCount() const;
    void collectGarbage() { shared.collectGarbage(); }

    // Plain-text form, one route per ';': "source,target,amount,polarity,curve,smoothingMs".
    std::string serialise() const;
    bool deserialise(const std::string& text);

    // Audio thread.
    const RouteSet* acquire() noexcept { return shared.acquire(); }
    void release() noexcept { shared.release(); }

    // Whether a parameter of this store product accepts routes ("route:<n>:amount" and "voice:amp" included).
    bool accepts(const std::string& paramId) const;
    // DEPRECATED: the same for the process default product.
    static bool canTarget(const std::string& paramId);

private:
    void publishLocked();
    const ParameterModel* params;
    mutable std::mutex mutex;
    std::vector<ModRoute> list;
    RtShared<RouteSet> shared;
    std::atomic<std::uint32_t> changes{0};
};

} // namespace crucible
