// SPDX-License-Identifier: MIT
#pragma once

// Patch format: a versioned JSON document holding everything that defines a sound.
//
// {
//   "format": "crucible-patch", "version": 2,
//   "engine": "CrucibleEngine 0.1.0",
//   "meta": {"id", "name", "author", "product", "category", "tags": [...], "rating": 0..5, "favourite": bool,
//            "created", "modified", "description", "comment"},
//   "modules": {"fx.saturator": 1, ...},              // module versions at save time (for migrations)
//   "params": {"filter_cutoff": 1200, ...},        // natural units by stable id; missing ids load as defaults
//   "scenes": [{"slot", "values"}], "sequencer": [{"slot", "bars", "steps"}], "arp": "...", "routes": "...",
//   "tuning": "...",                               // Tuning::serialise() (scale, map, offsets); absent = 12-TET
//   "ccMap": "...",                                // CcMapStore::serialise() (CC bindings); absent = none
//   "clips": [...],                                // automation clips (see Clips.h); absent in older patches
//   "clipsStopped": 5,                             // bit mask of stopped clips (bit k = clip k); absent = all run
//   "userWaves": [{"slot", "name", "frames": base64 float32, "zones": base64 float32}],   // "zones" only for a table
//                  with 2+ per-pitch zones (UserWavetableBank::packZones); older readers use "frames" (the lowest zone)
//   "formantTables": [{"slot", "vowels", "bands", "data": base64 float32, "saturate": true}],   // user vowel tables of the Formant filter, one
//                  entry per filter slot that has one, written only then; data = output gain, then vowels x bands x
//                  (freq Hz, bandwidth Hz, gain dB); "saturate" only when the state saturation is on. Absent = every slot uses the built-in vowels (also in older patches)
//   "convolutionIrs": [{"instance": "fx.convolution.2", "name", "rate", "layout": "mono"|"stereo"|"truestereo",
//                       "channels": [base64, ...]}],   // one entry per fx.convolution instance holding a response
//   "shaperCurves": [{"instance": "fx.shaper", "symmetric": true, "points": base64 float32 (x, y, curve) per point}],
//                  // one entry per fx.shaper instance whose curve is not the default identity; absent = identity (also in
//                  older patches, which need no migration)
//   "assets": [{"kind", "name", "slot", "index", "rate", "channels", "frames": base64 | "text"}],   // further embedded
//   content;
//                                                  // kind "sample" (oscillator slot audio; index n >= 1 = the n-th extra
//                                                  sample of a
//                                                  // multisample slot, absent = the slot's own sample), "zones" (text: the
//                                                  zone list of
//                                                  // a slot) and "cloud" are applied by applyPatch
//   "assetRefs": [{"kind", "name", "slot", "index", "rate", "channel", "hash", "instance"}]   // content kept in a library
//   asset store
// }
// Version history: 1 = first format; 2 = meta id / modified / description / product, generic assets and asset
// references (see PresetLibrary.h). Multisample zones added fields to version 2 only ("index", kind "zones", written
// for zoned slots only), so older files and every patch without zones read and write as before; the oscillator
// module (osc.sample) is version 2 and old patches get the zone parameters at their defaults (Off).
// 3 = the responses of every fx.convolution instance, keyed by instance id ("convolutionIrs"); version 2 files held the
// first instance's response in a single "convolutionIr" object, which the migration moves to the instance "fx.convolution".
//
// Loading migrates older format versions step by step (see migratePatchJson) and ignores unknown parameters, so a
// patch survives both engine upgrades and parameter additions.

#include <crucible/Engine.h>
#include <crucible/Json.h>

#include <map>
#include <string>
#include <vector>

namespace crucible
{

constexpr int kPatchFormatVersion = 3;

// 128-bit content hash as 32 hex characters (FNV-1a based; identifies content, not a security measure).
std::string contentHash(const void* data, size_t size);
std::string contentHash(const std::string& text);

struct PatchMeta
{
    std::string id; // stable identity of a preset in a library; assigned by the library when empty
    std::string name;
    std::string author;
    std::string category;
    std::vector<std::string> tags;
    int rating = 0; // 0..5
    bool favourite = false;
    std::string product;     // instrument the patch was made for ("ExampleSynth"), informational
    std::string created;     // ISO 8601, set by the caller
    std::string modified;    // ISO 8601
    std::string description; // shown in browsers
    std::string comment;     // free notes
};

// Content that is not a plain parameter: cloud models and samples (text or float data); wavetables and the impulse
// response have their own fields. kind is "wavetable", "ir", "cloud", "sample", "zones" or "slices" (the slice markers of
// the sample in the same slot, as floats).
struct PatchAsset
{
    std::string kind;
    std::string name;
    int slot = 0;
    double rate = 0.0;
    int channels = 1;
    std::vector<float> frames; // interleaved when channels > 1
    std::string text;          // text-form content (clouds)
    int index = 0;             // sample assets: 0 = the slot's own sample, n >= 1 = extra sample n of a multisample slot
};

// A piece of content held in an asset store instead of the patch file (hash = content hash).
struct PatchAssetRef
{
    std::string kind;
    std::string name;
    int slot = 0;
    double rate = 0.0;
    int channel = 0; // impulse response: 0 left, 1 right; -1 text content
    int channels = 1;
    std::string hash;
    int index = 0; // see PatchAsset::index
    // kind "ir" with an instance id: one channel of that fx.convolution instance's response (channels = how many it has,
    // channel = which one); an "ir" reference without an instance (the removed first-generation convolver) is ignored on load.
    std::string instance;
};

struct Patch
{
    int version = kPatchFormatVersion;
    PatchMeta meta;
    std::vector<std::pair<std::string, float>> params;
    // Parameters a patch file gave as text ("osc1_type": "Analog") instead of a number; migratedParams() resolves them
    // against the product's model (ParameterModel::textToValue). Saving writes them back as text. Entries that do not
    // resolve are ignored.
    std::vector<std::pair<std::string, std::string>> paramTexts;
    std::map<std::string, int> moduleVersions;
    std::vector<SceneRecord> scenes;
    std::vector<PatternRecord> patterns;
    std::string arpPattern;
    std::string routes;
    ClipBank clips;
    std::uint32_t clipsStopped = 0; // bit k = clip k is stopped (SynthEngine::clipRunMask); 0 = every clip runs
    std::string tuning;             // Tuning::serialise(), empty for 12-TET
    std::string ccMap;              // CcMapStore::serialise()
    std::vector<UserWavetableBank::Record> userWaves;
    struct FormantRecord
    {
        int slot = 0;
        FormantTable table;
    };
    std::vector<FormantRecord> formants; // user vowel tables of the Formant filter (filter slot 0..)
    std::vector<PatchAsset> assets;
    std::vector<PatchAssetRef> assetRefs;
    // Response of one convolution module instance (fx.convolution, fx.convolution.2, ...), as imported: layout "mono"
    // (1 channel), "stereo" (2) or "truestereo" (4: LL, LR, RL, RR). Instances without a response have no entry.
    struct ConvolutionIrData
    {
        std::string instance; // module instance id
        std::vector<std::vector<float>> channels;
        std::string layout;
        double rate = 48000.0;
        std::string name;
    };
    std::vector<ConvolutionIrData> convolutionIrs;
    // Transfer curve of one waveshaper instance (fx.shaper, fx.shaper.2, ...) as given to setShaperCurve. Instances still on
    // the default identity curve have no entry.
    struct ShaperCurveData
    {
        std::string instance; // module instance id
        bool symmetric = false;
        std::vector<ShaperPoint> points;
    };
    std::vector<ShaperCurveData> shaperCurves;

    float param(const std::string& id, float fallback) const;
    // The stored parameters after the migrations of every module whose version is newer than the one the patch was
    // saved with (ModuleMigration); applyPatch() uses this. Hosts that apply the parameters themselves call it too.
    PatchValues migratedParams(const ModuleRegistry& registry) const;
};

// Control thread.
Patch capturePatch(Engine& engine, const PatchMeta& meta = {});
// Parameters not in the patch are reset to their defaults.
void applyPatch(Engine& engine, const Patch& patch);

// The same for hosts that own the parameters themselves (the plugin keeps them in its parameter tree): naturalParams
// holds model().params().size() natural values, scenes may be null. The CC map belongs to Engine and is not
// captured (ccMap stays empty). Control thread.
Patch capturePatch(EngineCore& core, const std::vector<float>& naturalParams, const SceneBank* scenes,
                   const PatchMeta& meta = {});
// Applies everything except the parameters, scenes and the CC map: sequencer patterns, arp pattern, routes, clips,
// tuning, user wavetables, samples, clouds and the impulse response (each replaced by the patch's content).
void applyPatchContent(EngineCore& core, const Patch& patch);

json::Value patchToJsonValue(const Patch& patch);
std::string patchToJson(const Patch& patch);
bool patchFromJson(const std::string& text, Patch& out, std::string& error);
bool savePatchFile(const std::string& path, const Patch& patch);
bool loadPatchFile(const std::string& path, Patch& out, std::string& error);

// Brings a parsed document of any older format version up to kPatchFormatVersion; false for unknown formats or
// versions newer than this engine understands.
bool migratePatchJson(json::Value& doc, std::string& error);

} // namespace crucible
