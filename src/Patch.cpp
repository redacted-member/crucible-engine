// SPDX-License-Identifier: MIT
#include <crucible/Patch.h>
#include <crucible/Version.h>
#include <crucible/module/ConvolutionIr.h>
#include <crucible/module/ShaperCurve.h>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <functional>
#include <iterator>

#include "Utf8Path.h"

namespace crucible
{
std::string contentHash(const void* data, size_t size)
{
    // Two FNV-1a 64 lanes with different offsets: 128 bits, enough to key content-addressed files.
    std::uint64_t a = 0xcbf29ce484222325ULL, b = 0x84222325cbf29ce4ULL;
    const auto* p = static_cast<const std::uint8_t*>(data);
    for (size_t i = 0; i < size; ++i)
    {
        a = (a ^ p[i]) * 0x100000001b3ULL;
        b = (b ^ (p[i] + 0x9eU)) * 0x100000001b3ULL;
    }
    static const char* hex = "0123456789abcdef";
    std::string out(32, '0');
    for (int i = 0; i < 16; ++i)
    {
        out[static_cast<size_t>(15 - i)] = hex[(a >> (4 * i)) & 0xfU];
        out[static_cast<size_t>(31 - i)] = hex[(b >> (4 * i)) & 0xfU];
    }
    return out;
}

std::string contentHash(const std::string& text)
{
    return contentHash(text.data(), text.size());
}

namespace
{
using json::base64ToFloats;
using json::floatsToBase64;

// Migration steps: entry n turns a version n document into version n + 1. Version 1 is the first format.
const std::vector<std::function<void(json::Value&)>>& migrations()
{
    static const std::vector<std::function<void(json::Value&)>> steps = {
        // [0] unused (there is no version 0)
        [](json::Value&) {},
        // [1] -> 2: meta gains id, modified, description and product. The id is derived from the content so the
        // same file always migrates to the same identity.
        [](json::Value& doc)
        {
            json::Value meta = doc["meta"].isObject() ? doc["meta"] : json::Value::object();
            if (!meta.has("id") || meta["id"].asString("").empty())
                meta.set("id", "legacy-" + contentHash(doc["params"].dump(0) + meta["name"].asString("") +
                                                       meta["created"].asString("")));
            if (!meta.has("modified"))
                meta.set("modified", meta["created"].asString(""));
            if (!meta.has("description"))
                meta.set("description", meta["comment"].asString(""));
            if (!meta.has("product"))
                meta.set("product", "");
            doc.set("meta", meta);
        },
        // [2] -> 3: the single "convolutionIr" object (the first fx.convolution instance) becomes the entry of
        // "convolutionIrs" keyed "fx.convolution".
        [](json::Value& doc)
        {
            if (!doc["convolutionIr"].isObject())
                return;
            json::Value entry = json::Value::object();
            entry.set("instance", "fx.convolution");
            for (const auto& [key, value] : doc["convolutionIr"].entries())
                entry.set(key, value);
            json::Value list = json::Value::array();
            list.push(entry);
            json::Value out = json::Value::object();
            for (const auto& [key, value] : doc.entries())
                if (key != "convolutionIr")
                    out.set(key, value);
            out.set("convolutionIrs", list);
            doc = std::move(out);
        },
    };
    return steps;
}
} // namespace

float Patch::param(const std::string& id, float fallback) const
{
    for (const auto& [k, v] : params)
        if (k == id)
            return v;
    return fallback;
}

namespace
{
void captureContent(EngineCore& core, Patch& p)
{
    p.patterns = seqBankSerialise(core.store().snapshot(), &core.model());
    p.arpPattern = arpPatternSerialise(core.arpPattern().snapshot());
    SynthEngine& synth = core.engine();
    p.routes = synth.modRoutes().serialise();
    p.clips = synth.clips().snapshot();
    {
        const std::uint32_t existing = (1U << std::min<size_t>(p.clips.clips.size(), kMaxClips)) - 1U;
        p.clipsStopped = ~synth.clipRunMask() & existing;
    }
    {
        const Tuning t = synth.tuning().get();
        if (!t.isEqualTemperament() || t.noteOffsetCents(0) != 0.0 || t.scaleSize() != 12)
            p.tuning = t.serialise();
        else
        {
            bool offsets = false;
            for (int n = 0; n < 128; ++n)
                offsets = offsets || t.noteOffsetCents(n) != 0.0;
            if (offsets)
                p.tuning = t.serialise();
        }
    }
    p.userWaves = synth.userWavetables().serialise();
    for (int slot = 0; slot < kMaxFilterSlots; ++slot)
    {
        Patch::FormantRecord r;
        r.slot = slot;
        if (synth.formantTable(slot, r.table))
            p.formants.push_back(r);
    }
    // Samples and clouds of the oscillator slots travel as generic assets (kind "sample": interleaved float frames,
    // kind "cloud": text).
    for (auto& r : synth.sampleBank().serialise())
    {
        PatchAsset a;
        a.kind = "sample";
        a.name = std::move(r.name);
        a.slot = r.slot;
        a.rate = r.rate;
        a.channels = r.channels;
        a.frames = std::move(r.frames);
        const int slot = r.slot;
        const std::string sampleName = a.name;
        p.assets.push_back(std::move(a));
        const auto markers = synth.sampleBank().slices(slot);
        if (!markers.empty())
        {
            PatchAsset s;
            s.kind = "slices";
            s.name = sampleName;
            s.slot = slot;
            s.frames = markers;
            p.assets.push_back(std::move(s));
        }
    }
    // Multisample slots: the extra samples (index >= 1) and one "zones" text per slot. Nothing is written for slots
    // without zones, so patches of plain sample slots keep their exact content.
    for (auto& r : synth.zoneBank().serialiseSamples())
    {
        PatchAsset a;
        a.kind = "sample";
        a.name = std::move(r.name);
        a.slot = r.slot;
        a.index = r.index;
        a.rate = r.rate;
        a.channels = r.channels;
        a.frames = std::move(r.frames);
        p.assets.push_back(std::move(a));
    }
    for (int slot = 0; slot < kMaxOscSlots; ++slot)
    {
        std::string text = synth.zoneBank().serialiseZones(slot);
        if (text.empty())
            continue;
        PatchAsset a;
        a.kind = "zones";
        a.name = "zones";
        a.slot = slot;
        a.text = std::move(text);
        p.assets.push_back(std::move(a));
    }
    auto& bank = synth.cloudBank();
    for (int slot = 0; slot < kMaxOscSlots; ++slot)
        if (bank.isUser(slot) && bank.model(slot) != nullptr)
        {
            PatchAsset a;
            a.kind = "cloud";
            a.name = bank.model(slot)->name;
            a.slot = slot;
            a.text = bank.model(slot)->serialise();
            p.assets.push_back(std::move(a));
        }
    auto& rack = synth.effects();
    for (int i = 0; i < rack.size(); ++i)
        if (auto* cx = dynamic_cast<ConvolutionIr*>(rack.module(i)))
        {
            ImpulseResponse r;
            Patch::ConvolutionIrData d;
            if (cx->impulseResponse(r, d.name))
            {
                d.instance = rack.descriptor(i).id;
                d.rate = r.sampleRate;
                d.layout = r.layout == IrLayout::TrueStereo ? "truestereo"
                           : r.layout == IrLayout::Stereo   ? "stereo"
                                                            : "mono";
                d.channels = std::move(r.channels);
                p.convolutionIrs.push_back(std::move(d));
            }
        }
    for (int i = 0; i < rack.size(); ++i)
        if (auto* shaper = dynamic_cast<ShaperCurve*>(rack.module(i)))
        {
            Patch::ShaperCurveData d;
            if (shaper->curve(d.points, d.symmetric))
            {
                d.instance = rack.descriptor(i).id;
                p.shaperCurves.push_back(std::move(d));
            }
        }
}

void captureParams(Patch& p, const std::vector<float>& values, const ModuleRegistry& registry)
{
    const auto& defs = registry.parameters().params();
    for (size_t i = 0; i < defs.size() && i < values.size(); ++i)
        p.params.emplace_back(defs[i].id, values[i]);
    for (const auto& m : registry.modules())
        p.moduleVersions[m.id] = m.version;
}
} // namespace

Patch capturePatch(Engine& engine, const PatchMeta& meta)
{
    // Parameter events from the audio thread only mark seq_length / seq_pattern (and other setup work) as pending; apply
    // it here, on the control thread, so the patch holds the synced state (the audio thread never allocates or locks).
    engine.service();
    Patch p;
    p.meta = meta;
    captureParams(p, engine.getAllParams(), engine.registry());
    p.scenes = engine.scenes().serialise();
    captureContent(engine.coreEngine(), p);
    p.ccMap = engine.ccMap().serialise();
    return p;
}

Patch capturePatch(EngineCore& core, const std::vector<float>& naturalParams, const SceneBank* scenes, const PatchMeta& meta)
{
    Patch p;
    p.meta = meta;
    captureParams(p, naturalParams, core.engine().registry());
    if (scenes != nullptr)
        p.scenes = scenes->serialise();
    captureContent(core, p);
    return p;
}

void applyPatchContent(EngineCore& core, const Patch& p)
{
    SynthEngine& synth = core.engine();
    SeqBank bank;
    seqBankDeserialise(p.patterns, bank, &core.model());
    core.store().replace(bank);
    ArpPattern arp;
    if (!p.arpPattern.empty())
        arpPatternDeserialise(p.arpPattern, arp);
    core.arpPattern().replace(arp);
    synth.clips().replace(p.clips);
    synth.setClipRunMask(~p.clipsStopped);
    {
        Tuning t;
        if (!Tuning::deserialise(p.tuning, t))
            t = Tuning();
        synth.tuning().set(t);
    }
    synth.modRoutes().deserialise(p.routes);
    synth.userWavetables().deserialise(p.userWaves);
    {
        std::string ignored;
        for (int slot = 0; slot < kMaxFilterSlots; ++slot)
            synth.setFormantTable(slot, nullptr, ignored);
        for (const auto& r : p.formants)
            synth.setFormantTable(r.slot, &r.table, ignored);
    }
    std::vector<SampleBank::Record> samples;
    std::vector<CloudBank::Record> clouds;
    std::vector<ZoneBank::Record> extraSamples;
    std::vector<std::pair<int, std::string>> zoneTexts;
    for (const auto& a : p.assets)
    {
        if (a.kind == "sample" && a.index > 0)
            extraSamples.push_back({a.slot, a.index, a.name, a.rate, a.channels, a.frames});
        else if (a.kind == "zones")
            zoneTexts.emplace_back(a.slot, a.text);
        else if (a.kind == "sample")
            samples.push_back({a.slot, a.name, a.rate, a.channels, a.frames});
        else if (a.kind == "cloud")
            clouds.push_back({a.slot, a.text});
    }
    synth.sampleBank().deserialise(samples);
    synth.zoneBank().deserialise(extraSamples, zoneTexts);
    // Slice markers belong to the samples loaded above (loading a sample drops markers), so they come second.
    for (const auto& a : p.assets)
        if (a.kind == "slices")
            synth.sampleBank().setSlices(a.slot, a.frames);
    synth.cloudBank().deserialise(clouds);
    auto& rack = synth.effects();
    for (int i = 0; i < rack.size(); ++i)
    {
        auto* cx = dynamic_cast<ConvolutionIr*>(rack.module(i));
        if (cx == nullptr)
            continue;
        const std::string& id = rack.descriptor(i).id;
        const auto entry = std::find_if(p.convolutionIrs.begin(), p.convolutionIrs.end(),
                                        [&](const Patch::ConvolutionIrData& d) { return d.instance == id; });
        bool ok = entry != p.convolutionIrs.end();
        size_t want = 1;
        if (ok)
        {
            want = entry->layout == "truestereo" ? 4 : entry->layout == "stereo" ? 2 : 1;
            ok = entry->channels.size() == want && !entry->channels[0].empty();
            for (const auto& ch : entry->channels)
                ok = ok && ch.size() == entry->channels[0].size();
        }
        if (!ok)
            cx->clearImpulseResponse();
        else
        {
            ImpulseResponse r;
            r.sampleRate = entry->rate;
            r.layout = want == 4 ? IrLayout::TrueStereo : want == 2 ? IrLayout::Stereo : IrLayout::Mono;
            r.channels = entry->channels;
            cx->setImpulseResponse(r, entry->name);
        }
    }
    for (int i = 0; i < rack.size(); ++i)
    {
        auto* shaper = dynamic_cast<ShaperCurve*>(rack.module(i));
        if (shaper == nullptr)
            continue;
        const std::string& id = rack.descriptor(i).id;
        const auto entry = std::find_if(p.shaperCurves.begin(), p.shaperCurves.end(),
                                        [&](const Patch::ShaperCurveData& d) { return d.instance == id; });
        std::string ignored;
        // A missing or invalid entry leaves the instance on the identity curve.
        if (entry == p.shaperCurves.end() ||
            !shaper->setCurve(entry->points.data(), static_cast<int>(entry->points.size()), entry->symmetric, ignored))
            shaper->resetCurve();
    }
}

PatchValues Patch::migratedParams(const ModuleRegistry& registry) const
{
    PatchValues values(params.begin(), params.end());
    registry.migrate(moduleVersions, values);
    const ParameterModel& model = registry.parameters();
    // A text goes through the same renames as a number, and its value is parsed against the id it ends up with. Steps that
    // add or drop entries of their own (custom) show in a migration of the empty list; what is left over is the text's
    // entry.
    PatchValues baseline;
    if (!paramTexts.empty())
        registry.migrate(moduleVersions, baseline);
    for (const auto& [id, text] : paramTexts)
    {
        PatchValues one{{id, 0.0f}};
        registry.migrate(moduleVersions, one);
        for (const auto& b : baseline)
        {
            const auto it = std::find_if(one.begin(), one.end(),
                                         [&](const auto& e) { return e.first == b.first && e.second == b.second; });
            if (it != one.end())
                one.erase(it);
        }
        if (one.size() != 1)
            continue;
        float v = 0.0f;
        const int index = model.indexOf(one[0].first);
        if (index >= 0 && model.textToValue(index, text, v))
            values.emplace_back(one[0].first, v);
    }
    return values;
}

void applyPatch(Engine& engine, const Patch& p)
{
    const Engine::SourceScope scope(engine, Engine::ParamSource::Internal);
    engine.resetParamsToDefaults();
    // Parameters saved by an older version of a module go through that module's migrations first.
    for (const auto& [id, v] : p.migratedParams(engine.registry()))
        engine.setParam(id, v); // unknown ids are ignored
    engine.scenes().deserialise(p.scenes);
    engine.ccMap().deserialise(p.ccMap);
    applyPatchContent(engine.coreEngine(), p);
}

json::Value patchToJsonValue(const Patch& p)
{
    using json::Value;
    Value doc = Value::object();
    doc.set("format", "crucible-patch");
    doc.set("version", kPatchFormatVersion);
    doc.set("engine", std::string("CrucibleEngine ") + CRUCIBLE_VERSION_STRING);
    Value meta = Value::object();
    meta.set("id", p.meta.id);
    meta.set("name", p.meta.name);
    meta.set("author", p.meta.author);
    meta.set("category", p.meta.category);
    Value tags = Value::array();
    for (const auto& t : p.meta.tags)
        tags.push(t);
    meta.set("tags", tags);
    meta.set("rating", p.meta.rating);
    meta.set("favourite", p.meta.favourite);
    meta.set("product", p.meta.product);
    meta.set("created", p.meta.created);
    meta.set("modified", p.meta.modified);
    meta.set("description", p.meta.description);
    meta.set("comment", p.meta.comment);
    doc.set("meta", meta);
    Value modules = Value::object();
    for (const auto& [id, v] : p.moduleVersions)
        modules.set(id, v);
    doc.set("modules", modules);
    Value params = Value::object();
    for (const auto& [id, v] : p.params)
        params.set(id, static_cast<double>(v));
    for (const auto& [id, text] : p.paramTexts)
        params.set(id, text);
    doc.set("params", params);
    Value scenes = Value::array();
    for (const auto& s : p.scenes)
    {
        Value o = Value::object();
        o.set("slot", s.slot);
        o.set("values", s.values);
        scenes.push(o);
    }
    doc.set("scenes", scenes);
    Value seq = Value::array();
    for (const auto& r : p.patterns)
    {
        Value o = Value::object();
        o.set("slot", r.slot);
        o.set("bars", r.bars);
        o.set("steps", r.steps);
        seq.push(o);
    }
    doc.set("sequencer", seq);
    doc.set("arp", p.arpPattern);
    doc.set("routes", p.routes);
    if (!p.tuning.empty())
        doc.set("tuning", p.tuning);
    if (!p.ccMap.empty())
        doc.set("ccMap", p.ccMap);
    if (!p.clips.clips.empty())
        doc.set("clips", clipBankToJson(p.clips));
    if (p.clipsStopped != 0)
        doc.set("clipsStopped", static_cast<double>(p.clipsStopped));
    Value waves = Value::array();
    for (const auto& w : p.userWaves)
    {
        Value o = Value::object();
        o.set("slot", w.slot);
        o.set("name", w.name);
        o.set("frames", floatsToBase64(w.frames));
        if (w.zones.size() >= 2)
            o.set("zones", floatsToBase64(UserWavetableBank::packZones(w)));
        waves.push(o);
    }
    doc.set("userWaves", waves);
    if (!p.formants.empty())
    {
        Value arr = Value::array();
        for (const auto& r : p.formants)
        {
            Value o = Value::object();
            o.set("slot", r.slot);
            o.set("vowels", r.table.vowels);
            o.set("bands", r.table.bands);
            std::vector<float> data{r.table.outputGain};
            for (int v = 0; v < r.table.vowels; ++v)
                for (int b = 0; b < r.table.bands; ++b)
                {
                    const FormantBand& e = r.table.at(v, b);
                    data.insert(data.end(), {e.freqHz, e.bandwidthHz, e.gainDb});
                }
            o.set("data", floatsToBase64(data));
            if (r.table.saturate)
                o.set("saturate", true);
            arr.push(o);
        }
        doc.set("formantTables", arr);
    }
    if (!p.assets.empty())
    {
        Value arr = Value::array();
        for (const auto& a : p.assets)
        {
            Value o = Value::object();
            o.set("kind", a.kind);
            o.set("name", a.name);
            o.set("slot", a.slot);
            if (a.index != 0)
                o.set("index", a.index);
            o.set("rate", a.rate);
            o.set("channels", a.channels);
            if (!a.frames.empty())
                o.set("frames", floatsToBase64(a.frames));
            if (!a.text.empty())
                o.set("text", a.text);
            arr.push(o);
        }
        doc.set("assets", arr);
    }
    if (!p.assetRefs.empty())
    {
        Value arr = Value::array();
        for (const auto& a : p.assetRefs)
        {
            Value o = Value::object();
            o.set("kind", a.kind);
            o.set("name", a.name);
            o.set("slot", a.slot);
            if (a.index != 0)
                o.set("index", a.index);
            o.set("rate", a.rate);
            o.set("channel", a.channel);
            o.set("channels", a.channels);
            o.set("hash", a.hash);
            if (!a.instance.empty())
                o.set("instance", a.instance);
            arr.push(o);
        }
        doc.set("assetRefs", arr);
    }
    if (!p.convolutionIrs.empty())
    {
        Value arr = Value::array();
        for (const auto& c : p.convolutionIrs)
        {
            if (c.channels.empty())
                continue;
            Value cx = Value::object();
            cx.set("instance", c.instance);
            cx.set("name", c.name);
            cx.set("rate", c.rate);
            cx.set("layout", c.layout);
            Value chans = Value::array();
            for (const auto& ch : c.channels)
                chans.push(floatsToBase64(ch));
            cx.set("channels", chans);
            arr.push(cx);
        }
        if (arr.size() > 0)
            doc.set("convolutionIrs", arr);
    }
    if (!p.shaperCurves.empty())
    {
        Value arr = Value::array();
        for (const auto& c : p.shaperCurves)
        {
            if (c.points.empty())
                continue;
            Value o = Value::object();
            o.set("instance", c.instance);
            if (c.symmetric)
                o.set("symmetric", true);
            std::vector<float> data;
            for (const auto& q : c.points)
                data.insert(data.end(), {q.x, q.y, q.curve});
            o.set("points", floatsToBase64(data));
            arr.push(o);
        }
        if (arr.size() > 0)
            doc.set("shaperCurves", arr);
    }
    return doc;
}

std::string patchToJson(const Patch& patch)
{
    return patchToJsonValue(patch).dump(2) + "\n";
}

bool migratePatchJson(json::Value& doc, std::string& error)
{
    if (!doc.isObject() || doc["format"].asString() != "crucible-patch")
    {
        error = "not a crucible patch";
        return false;
    }
    int version = static_cast<int>(doc["version"].asNumber(0.0));
    if (version < 1 || version > kPatchFormatVersion)
    {
        error = "unsupported patch version " + std::to_string(version);
        return false;
    }
    const auto& steps = migrations();
    while (version < kPatchFormatVersion)
    {
        steps[static_cast<size_t>(version)](doc);
        doc.set("version", ++version);
    }
    return true;
}

bool patchFromJson(const std::string& text, Patch& out, std::string& error)
{
    json::Value doc;
    if (!json::parse(text, doc, error) || !migratePatchJson(doc, error))
        return false;
    Patch p;
    const auto& meta = doc["meta"];
    p.meta.name = meta["name"].asString("");
    p.meta.author = meta["author"].asString("");
    p.meta.category = meta["category"].asString("");
    for (const auto& t : meta["tags"].elements())
        if (t.isString())
            p.meta.tags.push_back(t.asString());
    p.meta.rating = std::clamp(static_cast<int>(meta["rating"].asNumber(0.0)), 0, 5);
    p.meta.favourite = meta["favourite"].asBool(false);
    p.meta.id = meta["id"].asString("");
    p.meta.product = meta["product"].asString("");
    p.meta.modified = meta["modified"].asString("");
    p.meta.description = meta["description"].asString("");
    p.meta.created = meta["created"].asString("");
    p.meta.comment = meta["comment"].asString("");
    for (const auto& [id, v] : doc["modules"].entries())
        p.moduleVersions[id] = static_cast<int>(v.asNumber(1.0));
    for (const auto& [id, v] : doc["params"].entries())
        if (v.isNumber())
            p.params.emplace_back(id, static_cast<float>(v.asNumber()));
        else if (v.isString())
            p.paramTexts.emplace_back(id, v.asString(""));
    for (const auto& s : doc["scenes"].elements())
        p.scenes.push_back({static_cast<int>(s["slot"].asNumber(-1.0)), s["values"].asString("")});
    for (const auto& s : doc["sequencer"].elements())
        p.patterns.push_back({static_cast<int>(s["slot"].asNumber(-1.0)), static_cast<int>(s["bars"].asNumber(1.0)),
                              s["steps"].asString("")});
    p.arpPattern = doc["arp"].asString("");
    p.routes = doc["routes"].asString("");
    p.tuning = doc["tuning"].asString("");
    p.ccMap = doc["ccMap"].asString("");
    clipBankFromJson(doc["clips"], p.clips);
    {
        const double stopped = doc["clipsStopped"].asNumber(0.0);
        p.clipsStopped = stopped >= 1.0 && stopped < 4294967296.0 ? static_cast<std::uint32_t>(stopped) : 0U;
    }
    for (const auto& w : doc["userWaves"].elements())
    {
        UserWavetableBank::Record r;
        r.slot = static_cast<int>(w["slot"].asNumber(-1.0));
        r.name = w["name"].asString("");
        if (base64ToFloats(w["frames"].asString(""), r.frames))
        {
            std::vector<float> blob;
            if (w["zones"].isString() && base64ToFloats(w["zones"].asString(""), blob))
                UserWavetableBank::unpackZones(blob, r); // a malformed blob leaves the one-zone frames
            p.userWaves.push_back(std::move(r));
        }
    }
    for (const auto& t : doc["formantTables"].elements())
    {
        const int slot = static_cast<int>(t["slot"].asNumber(-1.0));
        const int vowels = static_cast<int>(t["vowels"].asNumber(0.0));
        const int bands = static_cast<int>(t["bands"].asNumber(0.0));
        std::vector<float> data;
        if (slot < 0 || slot >= kMaxFilterSlots || vowels < 2 || vowels > kMaxFormantVowels || bands < 1 ||
            bands > kMaxFormantBands || !base64ToFloats(t["data"].asString(""), data) ||
            data.size() != static_cast<size_t>(1 + vowels * bands * 3))
            continue; // a malformed entry leaves the slot on the built-in vowels
        std::vector<FormantBand> flat(static_cast<size_t>(vowels * bands));
        for (size_t i = 0; i < flat.size(); ++i)
            flat[i] = {data[1 + 3 * i], data[2 + 3 * i], data[3 + 3 * i]};
        Patch::FormantRecord r;
        r.slot = slot;
        std::string tableError;
        if (makeFormantTable(flat.data(), vowels, bands, data[0], r.table, tableError))
        {
            r.table.saturate = t["saturate"].asBool(false);
            p.formants.push_back(std::move(r));
        }
    }
    for (const auto& a : doc["assets"].elements())
    {
        PatchAsset asset;
        asset.kind = a["kind"].asString("");
        asset.name = a["name"].asString("");
        asset.slot = static_cast<int>(a["slot"].asNumber(0.0));
        asset.index = static_cast<int>(a["index"].asNumber(0.0));
        asset.rate = a["rate"].asNumber(0.0);
        asset.channels = std::max(1, static_cast<int>(a["channels"].asNumber(1.0)));
        base64ToFloats(a["frames"].asString(""), asset.frames);
        asset.text = a["text"].asString("");
        p.assets.push_back(std::move(asset));
    }
    for (const auto& a : doc["assetRefs"].elements())
    {
        PatchAssetRef ref;
        ref.kind = a["kind"].asString("");
        ref.name = a["name"].asString("");
        ref.slot = static_cast<int>(a["slot"].asNumber(0.0));
        ref.index = static_cast<int>(a["index"].asNumber(0.0));
        ref.rate = a["rate"].asNumber(0.0);
        ref.channel = static_cast<int>(a["channel"].asNumber(0.0));
        ref.channels = std::max(1, static_cast<int>(a["channels"].asNumber(1.0)));
        ref.hash = a["hash"].asString("");
        ref.instance = a["instance"].asString("");
        if (!ref.hash.empty())
            p.assetRefs.push_back(std::move(ref));
    }
    for (const auto& cx : doc["convolutionIrs"].elements())
    {
        Patch::ConvolutionIrData d;
        d.instance = cx["instance"].asString("fx.convolution");
        d.name = cx["name"].asString("");
        d.rate = cx["rate"].asNumber(48000.0);
        d.layout = cx["layout"].asString("mono");
        for (const auto& c : cx["channels"].elements())
        {
            std::vector<float> ch;
            base64ToFloats(c.asString(""), ch);
            d.channels.push_back(std::move(ch));
        }
        p.convolutionIrs.push_back(std::move(d));
    }
    for (const auto& sc : doc["shaperCurves"].elements())
    {
        Patch::ShaperCurveData d;
        d.instance = sc["instance"].asString("fx.shaper");
        d.symmetric = sc["symmetric"].asBool(false);
        std::vector<float> data;
        if (!base64ToFloats(sc["points"].asString(""), data) || data.empty() || data.size() % 3 != 0)
            continue; // a malformed entry leaves the instance on the identity curve
        for (size_t i = 0; i < data.size(); i += 3)
            d.points.push_back({data[i], data[i + 1], data[i + 2]});
        p.shaperCurves.push_back(std::move(d));
    }
    out = std::move(p);
    return true;
}

bool savePatchFile(const std::string& path, const Patch& patch)
{
    std::ofstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    f << patchToJson(patch);
    return static_cast<bool>(f);
}

bool loadPatchFile(const std::string& path, Patch& out, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
    {
        error = "cannot read " + path;
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return patchFromJson(text, out, error);
}

} // namespace crucible
