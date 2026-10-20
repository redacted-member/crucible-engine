// SPDX-License-Identifier: MIT
#include <crucible/graph/Product.h>

#include <algorithm>
#include <fstream>
#include <sstream>

#include "Utf8Path.h"

namespace crucible::graph
{
namespace
{
using json::Value;

struct Reader
{
    std::string error;
    bool fail(const std::string& path, const std::string& message)
    {
        if (error.empty())
            error = path + ": " + message;
        return false;
    }
    bool ok() const { return error.empty(); }
};

bool readString(Reader& r, const Value& obj, const char* key, std::string& out, const std::string& path, bool required)
{
    if (!obj.has(key))
        return !required || r.fail(path, std::string("missing \"") + key + "\"");
    if (!obj[key].isString())
        return r.fail(path, std::string("\"") + key + "\" must be a string");
    out = obj[key].asString();
    return true;
}

bool readNumber(Reader& r, const Value& obj, const char* key, double& out, const std::string& path)
{
    if (!obj.has(key))
        return true;
    if (!obj[key].isNumber())
        return r.fail(path, std::string("\"") + key + "\" must be a number");
    out = obj[key].asNumber();
    return true;
}

bool readPair(Reader& r, const Value& obj, const char* key, int& lo, int& hi, const std::string& path)
{
    if (!obj.has(key))
        return true;
    const Value& v = obj[key];
    if (!v.isArray() || v.size() != 2 || !v[0].isNumber() || !v[1].isNumber())
        return r.fail(path, std::string("\"") + key + "\" must be [low, high]");
    lo = static_cast<int>(v[0].asNumber());
    hi = static_cast<int>(v[1].asNumber());
    return true;
}

bool readParams(Reader& r, const Value& obj, const char* key, std::vector<std::pair<std::string, double>>& out,
                const std::string& path)
{
    if (!obj.has(key))
        return true;
    if (!obj[key].isObject())
        return r.fail(path, std::string("\"") + key + "\" must be an object");
    for (const auto& [k, v] : obj[key].entries())
    {
        if (!v.isNumber())
            return r.fail(path + "." + key, "\"" + k + "\" must be a number");
        out.emplace_back(k, v.asNumber());
    }
    return true;
}

bool readNodes(Reader& r, const Value& arr, std::vector<NodeDef>& out, const std::string& path)
{
    if (!arr.isArray())
        return r.fail(path, "must be an array");
    for (size_t i = 0; i < arr.size(); ++i)
    {
        const std::string here = path + "[" + std::to_string(i) + "]";
        const Value& n = arr[i];
        if (!n.isObject())
            return r.fail(here, "must be an object");
        NodeDef d;
        double slot = 0.0;
        if (!readString(r, n, "id", d.id, here, true) || !readString(r, n, "type", d.type, here, true) ||
            !readString(r, n, "scope", d.scope, here, false) || !readNumber(r, n, "slot", slot, here) ||
            !readParams(r, n, "params", d.params, here))
            return false;
        d.slot = static_cast<int>(slot);
        out.push_back(std::move(d));
    }
    return true;
}

bool readSurface(Reader& r, const Value& v, SurfaceDef& out)
{
    const std::string path = "surface";
    if (!v.isObject())
        return r.fail(path, "must be an object");
    out.present = true;
    if (v.has("expose"))
    {
        if (!v["expose"].isString() || (v["expose"].asString() != "all" && v["expose"].asString() != "listed"))
            return r.fail(path + ".expose", "must be \"all\" or \"listed\"");
        out.listed = v["expose"].asString() == "listed";
    }
    if (v.has("sections"))
    {
        const Value& arr = v["sections"];
        if (!arr.isArray())
            return r.fail(path + ".sections", "must be an array");
        for (size_t i = 0; i < arr.size(); ++i)
        {
            const std::string here = path + ".sections[" + std::to_string(i) + "]";
            if (!arr[i].isObject())
                return r.fail(here, "must be an object");
            SurfaceSectionDef s;
            if (!readString(r, arr[i], "id", s.id, here, true) || !readString(r, arr[i], "name", s.name, here, false))
                return false;
            if (s.name.empty())
                s.name = s.id;
            out.sections.push_back(std::move(s));
        }
    }
    if (v.has("pages"))
    {
        const Value& arr = v["pages"];
        if (!arr.isArray())
            return r.fail(path + ".pages", "must be an array");
        for (size_t i = 0; i < arr.size(); ++i)
        {
            const std::string here = path + ".pages[" + std::to_string(i) + "]";
            if (!arr[i].isObject())
                return r.fail(here, "must be an object");
            SurfacePageDef p;
            if (!readString(r, arr[i], "id", p.id, here, true) || !readString(r, arr[i], "name", p.name, here, false) ||
                !readString(r, arr[i], "section", p.section, here, false))
                return false;
            if (p.name.empty())
                p.name = p.id;
            if (!arr[i].has("params") || !arr[i]["params"].isArray())
                return r.fail(here, "\"params\" must be an array of parameter ids");
            const Value& ids = arr[i]["params"];
            for (size_t k = 0; k < ids.size(); ++k)
            {
                if (!ids[k].isString())
                    return r.fail(here + ".params[" + std::to_string(k) + "]", "must be a parameter id string");
                p.params.push_back(ids[k].asString());
            }
            out.pages.push_back(std::move(p));
        }
    }
    return true;
}

bool readEdges(Reader& r, const Value& arr, std::vector<EdgeDef>& out, const std::string& path)
{
    if (!arr.isArray())
        return r.fail(path, "must be an array");
    for (size_t i = 0; i < arr.size(); ++i)
    {
        const std::string here = path + "[" + std::to_string(i) + "]";
        const Value& e = arr[i];
        if (!e.isObject())
            return r.fail(here, "must be an object");
        EdgeDef d;
        double amount = 1.0;
        if (!readString(r, e, "from", d.from, here, true) || !readString(r, e, "to", d.to, here, true) ||
            !readNumber(r, e, "amount", amount, here))
            return false;
        d.amount = static_cast<float>(amount);
        out.push_back(std::move(d));
    }
    return true;
}

bool readVoice(Reader& r, const Value& v, VoiceDef& out)
{
    if (v.isString())
    {
        if (v.asString() == "classic")
            out.kind = VoiceKind::Classic;
        else if (v.asString() == "none")
            out.kind = VoiceKind::None;
        else
            return r.fail("voice", "must be \"classic\", \"none\" or an object");
        return true;
    }
    if (!v.isObject())
        return r.fail("voice", "must be \"classic\", \"none\" or an object");
    out.kind = VoiceKind::Graph;
    if (v.has("nodes") && !readNodes(r, v["nodes"], out.nodes, "voice.nodes"))
        return false;
    if (v.has("edges") && !readEdges(r, v["edges"], out.edges, "voice.edges"))
        return false;
    if (v.has("layers"))
    {
        const Value& layers = v["layers"];
        if (!layers.isArray())
            return r.fail("voice.layers", "must be an array");
        for (size_t i = 0; i < layers.size(); ++i)
        {
            const std::string here = "voice.layers[" + std::to_string(i) + "]";
            const Value& l = layers[i];
            if (!l.isObject())
                return r.fail(here, "must be an object");
            LayerDef d;
            double voices = d.voices;
            if (!readString(r, l, "id", d.id, here, true) || !readPair(r, l, "notes", d.noteLo, d.noteHi, here) ||
                !readPair(r, l, "velocity", d.velLo, d.velHi, here) ||
                !readString(r, l, "choke", d.chokeGroup, here, false) ||
                !readString(r, l, "round_robin", d.roundRobin, here, false) || !readNumber(r, l, "voices", voices, here))
                return false;
            d.voices = static_cast<int>(voices);
            d.oneShot = l["oneShot"].asBool(false);
            if (l.has("nodes") && !readNodes(r, l["nodes"], d.nodes, here + ".nodes"))
                return false;
            if (l.has("edges") && !readEdges(r, l["edges"], d.edges, here + ".edges"))
                return false;
            out.layers.push_back(std::move(d));
        }
    }
    // The classic voice written as a one-node graph is the same thing as "classic".
    if ((out.nodes.size() == 1 && out.nodes[0].type == "classic.voice" && out.edges.empty() && out.layers.empty()))
    {
        out.kind = VoiceKind::Classic;
        out.nodes.clear();
    }
    return true;
}

bool readFx(Reader& r, const Value& v, FxDef& out)
{
    if (!v.isObject())
        return r.fail("fx", "must be an object");
    out.present = true;
    if (v.has("instances"))
    {
        if (!v["instances"].isArray())
            return r.fail("fx.instances", "must be an array");
        for (size_t i = 0; i < v["instances"].size(); ++i)
        {
            const std::string here = "fx.instances[" + std::to_string(i) + "]";
            const Value& e = v["instances"][i];
            FxInstanceDef d;
            if (!e.isObject() || !readString(r, e, "id", d.id, here, true) || !readString(r, e, "type", d.type, here, true))
                return e.isObject() ? false : r.fail(here, "must be an object");
            out.instances.push_back(std::move(d));
        }
    }
    if (v.has("lanes"))
    {
        if (!v["lanes"].isArray())
            return r.fail("fx.lanes", "must be an array");
        for (size_t i = 0; i < v["lanes"].size(); ++i)
        {
            const std::string here = "fx.lanes[" + std::to_string(i) + "]";
            const Value& l = v["lanes"][i];
            if (!l.isObject())
                return r.fail(here, "must be an object");
            FxLaneDef d;
            std::string order = "listed";
            double level = 1.0;
            if (!readString(r, l, "id", d.id, here, true) || !readString(r, l, "order", order, here, false) ||
                !readNumber(r, l, "level", level, here))
                return false;
            if (order != "listed" && order != "position")
                return r.fail(here, "\"order\" must be \"listed\" or \"position\"");
            d.byPosition = order == "position";
            d.level = static_cast<float>(level);
            d.toMaster = l["master"].isBool() ? l["master"].asBool(true) : true;
            if (l.has("input"))
            {
                const Value& in = l["input"];
                if (in.isString())
                    d.inputs.push_back({in.asString(), 1.0f});
                else if (in.isArray())
                {
                    for (size_t k = 0; k < in.size(); ++k)
                    {
                        if (in[k].isString())
                            d.inputs.push_back({in[k].asString(), 1.0f});
                        else if (in[k].isObject() && in[k]["from"].isString())
                            d.inputs.push_back({in[k]["from"].asString(), static_cast<float>(in[k]["gain"].asNumber(1.0))});
                        else
                            return r.fail(here + ".input", "entries are source names or {\"from\", \"gain\"}");
                    }
                }
                else
                    return r.fail(here, "\"input\" must be a name or an array");
            }
            if (l.has("chain"))
            {
                if (!l["chain"].isArray())
                    return r.fail(here, "\"chain\" must be an array");
                for (size_t k = 0; k < l["chain"].size(); ++k)
                {
                    if (!l["chain"][k].isString())
                        return r.fail(here + ".chain", "entries must be instance ids");
                    d.chain.push_back(l["chain"][k].asString());
                }
            }
            out.lanes.push_back(std::move(d));
        }
    }
    if (v.has("sends"))
    {
        if (!v["sends"].isArray())
            return r.fail("fx.sends", "must be an array");
        for (size_t i = 0; i < v["sends"].size(); ++i)
        {
            const std::string here = "fx.sends[" + std::to_string(i) + "]";
            const Value& e = v["sends"][i];
            FxSendDef d;
            double gain = 1.0;
            if (!e.isObject() || !readString(r, e, "from", d.from, here, true) ||
                !readString(r, e, "to", d.to, here, true) || !readNumber(r, e, "gain", gain, here))
                return e.isObject() ? false : r.fail(here, "must be an object");
            d.gain = static_cast<float>(gain);
            out.sends.push_back(std::move(d));
        }
    }
    if (v["master"].isObject())
        out.masterLevel = static_cast<float>(v["master"]["level"].asNumber(1.0));
    return true;
}

Value nodesToJson(const std::vector<NodeDef>& nodes)
{
    Value arr = Value::array();
    for (const auto& n : nodes)
    {
        Value o = Value::object();
        o.set("id", n.id);
        o.set("type", n.type);
        if (n.slot > 0)
            o.set("slot", n.slot);
        if (n.scope != "voice")
            o.set("scope", n.scope);
        if (!n.params.empty())
        {
            Value p = Value::object();
            for (const auto& [k, v] : n.params)
                p.set(k, v);
            o.set("params", std::move(p));
        }
        arr.push(std::move(o));
    }
    return arr;
}

Value edgesToJson(const std::vector<EdgeDef>& edges)
{
    Value arr = Value::array();
    for (const auto& e : edges)
    {
        Value o = Value::object();
        o.set("from", e.from);
        o.set("to", e.to);
        if (e.amount != 1.0f)
            o.set("amount", e.amount);
        arr.push(std::move(o));
    }
    return arr;
}

Value pairToJson(int lo, int hi)
{
    Value a = Value::array();
    a.push(lo);
    a.push(hi);
    return a;
}
} // namespace

bool productFromJson(const Value& root, ProductDef& out, std::string& error)
{
    Reader r;
    out = ProductDef{};
    if (!root.isObject())
    {
        error = "$: a crucible-product is a JSON object";
        return false;
    }
    if (!root["format"].isString() || root["format"].asString() != kProductFormat)
    {
        error = std::string("format: expected \"") + kProductFormat + "\"";
        return false;
    }
    if (!root["version"].isNumber() || static_cast<int>(root["version"].asNumber()) != kProductFormatVersion)
    {
        error = "version: unsupported product format version (this build reads version " +
                std::to_string(kProductFormatVersion) + ")";
        return false;
    }
    out.version = kProductFormatVersion;
    double osc = 0, filt = 0, macros = 0;
    if (!readString(r, root, "id", out.id, "$", true) || !readString(r, root, "name", out.name, "$", false) ||
        !readString(r, root, "description", out.description, "$", false))
    {
        error = r.error;
        return false;
    }
    if (root.has("layout"))
    {
        if (!root["layout"].isObject())
        {
            error = "layout: must be an object";
            return false;
        }
        if (!readNumber(r, root["layout"], "oscillators", osc, "layout") ||
            !readNumber(r, root["layout"], "filters", filt, "layout") ||
            !readNumber(r, root["layout"], "macros", macros, "layout"))
        {
            error = r.error;
            return false;
        }
        out.oscillators = static_cast<int>(osc);
        out.filters = static_cast<int>(filt);
        out.macros = static_cast<int>(macros);
    }
    if (root.has("voice") && !readVoice(r, root["voice"], out.voice))
    {
        error = r.error;
        return false;
    }
    if (root.has("fx") && !readFx(r, root["fx"], out.fx))
    {
        error = r.error;
        return false;
    }
    if (root.has("surface") && !readSurface(r, root["surface"], out.surface))
    {
        error = r.error;
        return false;
    }
    if (root.has("idMap"))
    {
        if (!root["idMap"].isObject())
        {
            error = "idMap: must be an object";
            return false;
        }
        for (const auto& [k, v] : root["idMap"].entries())
        {
            if (!v.isString())
            {
                error = "idMap." + k + ": must be a string";
                return false;
            }
            out.idMap.emplace_back(k, v.asString());
        }
    }
    if (!readParams(r, root, "defaults", out.defaults, "$"))
    {
        error = r.error;
        return false;
    }
    return true;
}

bool productFromText(const std::string& text, ProductDef& out, std::string& error)
{
    Value root;
    std::string parseError;
    if (!json::parse(text, root, parseError))
    {
        error = "json: " + parseError;
        return false;
    }
    return productFromJson(root, out, error);
}

bool loadProduct(const std::string& path, ProductDef& out, std::string& error)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
    {
        error = path + ": cannot open";
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return productFromText(ss.str(), out, error);
}

Value productToJson(const ProductDef& p)
{
    Value root = Value::object();
    root.set("format", kProductFormat);
    root.set("version", p.version);
    root.set("id", p.id);
    if (!p.name.empty())
        root.set("name", p.name);
    if (!p.description.empty())
        root.set("description", p.description);
    if (p.oscillators > 0 || p.filters > 0 || p.macros > 0)
    {
        Value l = Value::object();
        if (p.oscillators > 0)
            l.set("oscillators", p.oscillators);
        if (p.filters > 0)
            l.set("filters", p.filters);
        if (p.macros > 0)
            l.set("macros", p.macros);
        root.set("layout", std::move(l));
    }
    switch (p.voice.kind)
    {
    case VoiceKind::Classic:
        root.set("voice", "classic");
        break;
    case VoiceKind::None:
        root.set("voice", "none");
        break;
    case VoiceKind::Graph:
    {
        Value v = Value::object();
        if (!p.voice.nodes.empty())
            v.set("nodes", nodesToJson(p.voice.nodes));
        if (!p.voice.edges.empty())
            v.set("edges", edgesToJson(p.voice.edges));
        if (!p.voice.layers.empty())
        {
            Value layers = Value::array();
            for (const auto& l : p.voice.layers)
            {
                Value o = Value::object();
                o.set("id", l.id);
                o.set("notes", pairToJson(l.noteLo, l.noteHi));
                o.set("velocity", pairToJson(l.velLo, l.velHi));
                if (!l.chokeGroup.empty())
                    o.set("choke", l.chokeGroup);
                if (!l.roundRobin.empty())
                    o.set("round_robin", l.roundRobin);
                o.set("voices", l.voices);
                if (l.oneShot)
                    o.set("oneShot", true);
                o.set("nodes", nodesToJson(l.nodes));
                o.set("edges", edgesToJson(l.edges));
                layers.push(std::move(o));
            }
            v.set("layers", std::move(layers));
        }
        root.set("voice", std::move(v));
        break;
    }
    }
    if (p.fx.present)
    {
        Value fx = Value::object();
        Value inst = Value::array();
        for (const auto& i : p.fx.instances)
        {
            Value o = Value::object();
            o.set("id", i.id);
            o.set("type", i.type);
            inst.push(std::move(o));
        }
        fx.set("instances", std::move(inst));
        Value lanes = Value::array();
        for (const auto& l : p.fx.lanes)
        {
            Value o = Value::object();
            o.set("id", l.id);
            o.set("order", l.byPosition ? "position" : "listed");
            Value in = Value::array();
            for (const auto& s : l.inputs)
            {
                if (s.gain == 1.0f)
                    in.push(s.source);
                else
                {
                    Value e = Value::object();
                    e.set("from", s.source);
                    e.set("gain", s.gain);
                    in.push(std::move(e));
                }
            }
            o.set("input", std::move(in));
            Value chain = Value::array();
            for (const auto& c : l.chain)
                chain.push(c);
            o.set("chain", std::move(chain));
            if (l.level != 1.0f)
                o.set("level", l.level);
            if (!l.toMaster)
                o.set("master", false);
            lanes.push(std::move(o));
        }
        fx.set("lanes", std::move(lanes));
        if (!p.fx.sends.empty())
        {
            Value sends = Value::array();
            for (const auto& s : p.fx.sends)
            {
                Value o = Value::object();
                o.set("from", s.from);
                o.set("to", s.to);
                o.set("gain", s.gain);
                sends.push(std::move(o));
            }
            fx.set("sends", std::move(sends));
        }
        if (p.fx.masterLevel != 1.0f)
        {
            Value m = Value::object();
            m.set("level", p.fx.masterLevel);
            fx.set("master", std::move(m));
        }
        root.set("fx", std::move(fx));
    }
    if (p.surface.present)
    {
        Value s = Value::object();
        s.set("expose", p.surface.listed ? "listed" : "all");
        if (!p.surface.sections.empty())
        {
            Value arr = Value::array();
            for (const auto& sec : p.surface.sections)
            {
                Value o = Value::object();
                o.set("id", sec.id);
                o.set("name", sec.name);
                arr.push(std::move(o));
            }
            s.set("sections", std::move(arr));
        }
        if (!p.surface.pages.empty())
        {
            Value arr = Value::array();
            for (const auto& page : p.surface.pages)
            {
                Value o = Value::object();
                o.set("id", page.id);
                o.set("name", page.name);
                if (!page.section.empty())
                    o.set("section", page.section);
                Value ids = Value::array();
                for (const auto& id : page.params)
                    ids.push(id);
                o.set("params", std::move(ids));
                arr.push(std::move(o));
            }
            s.set("pages", std::move(arr));
        }
        root.set("surface", std::move(s));
    }
    if (!p.idMap.empty())
    {
        Value m = Value::object();
        for (const auto& [k, v] : p.idMap)
            m.set(k, v);
        root.set("idMap", std::move(m));
    }
    if (!p.defaults.empty())
    {
        Value d = Value::object();
        for (const auto& [k, v] : p.defaults)
            d.set(k, v);
        root.set("defaults", std::move(d));
    }
    return root;
}

std::string productToText(const ProductDef& product, int indent)
{
    return productToJson(product).dump(indent);
}

} // namespace crucible::graph
