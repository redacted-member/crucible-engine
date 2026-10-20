// SPDX-License-Identifier: MIT
#include <crucible/Introspection.h>

#include <crucible/Engine.h>
#include <crucible/Json.h>
#include <crucible/ParamText.h>
#include <crucible/Views.h>
#include <crucible/Version.h>

#include <algorithm>
#include <cctype>

namespace crucible
{
namespace
{
using json::Value;

const char* typeName(ParamType t)
{
    switch (t)
    {
    case ParamType::Float:
        return "float";
    case ParamType::Int:
        return "int";
    case ParamType::Choice:
        return "choice";
    case ParamType::Bool:
        return "bool";
    }
    return "float";
}

bool typeFromName(const std::string& n, ParamType& out)
{
    for (const auto t : {ParamType::Float, ParamType::Int, ParamType::Choice, ParamType::Bool})
        if (n == typeName(t))
        {
            out = t;
            return true;
        }
    return false;
}

const char* smoothingName(Smoothing s)
{
    return s == Smoothing::None ? "none" : (s == Smoothing::Linear ? "linear" : "log");
}

bool smoothingFromName(const std::string& n, Smoothing& out)
{
    for (const auto t : {Smoothing::None, Smoothing::Linear, Smoothing::Log})
        if (n == smoothingName(t))
        {
            out = t;
            return true;
        }
    return false;
}

const char* cpuName(CpuClass c)
{
    return c == CpuClass::Light ? "light" : (c == CpuClass::Medium ? "medium" : "heavy");
}

// "osc12_pos" -> 12, else 0.
int slotOf(const std::string& id)
{
    if (id.rfind("osc", 0) != 0)
        return 0;
    size_t i = 3;
    int n = 0;
    while (i < id.size() && std::isdigit(static_cast<unsigned char>(id[i])))
        n = n * 10 + (id[i++] - '0');
    return i > 3 && i < id.size() && id[i] == '_' ? n : 0;
}

Value stringArray(const std::vector<std::string>& v)
{
    Value a = Value::array();
    for (const auto& s : v)
        a.push(s);
    return a;
}

Value indexArray(const std::vector<int>& v)
{
    Value a = Value::array();
    for (const int i : v)
        a.push(i);
    return a;
}

Value portArray(const std::vector<PortDef>& v)
{
    Value a = Value::array();
    for (const auto& p : v)
    {
        Value o = Value::object();
        o.set("name", p.name);
        o.set("type", portTypeName(p.type));
        a.push(o);
    }
    return a;
}

std::vector<std::string> readStrings(const Value& v)
{
    std::vector<std::string> out;
    for (const auto& e : v.elements())
        out.push_back(e.asString(""));
    return out;
}

std::vector<int> readInts(const Value& v)
{
    std::vector<int> out;
    for (const auto& e : v.elements())
        out.push_back(static_cast<int>(e.asNumber()));
    return out;
}

std::vector<PortDef> readPorts(const Value& v)
{
    std::vector<PortDef> out;
    for (const auto& e : v.elements())
    {
        PortDef p;
        p.name = e["name"].asString("");
        portTypeFromName(e["type"].asString(""), p.type);
        out.push_back(p);
    }
    return out;
}

const char* viewKindName(ViewKind kind)
{
    switch (kind)
    {
    case ViewKind::Scalar:
        return "scalar";
    case ViewKind::Vector:
        return "vector";
    case ViewKind::Points:
        return "points";
    case ViewKind::Mask:
        return "mask";
    }
    return "vector";
}

// Lists the hub's views and hands each to the module that owns it: an effect or graph instance owns the views of its own
// instance id; an oscillator module owns the views of its slots ("osc1"...) whose id starts with its family ("sample_",
// "cloud_"); the voice module owns the filter views (instances "filter", "filter2"...).
void addViews(ProductDescription& d, const ViewHub& hub)
{
    for (int v = 0; v < hub.count(); ++v)
    {
        const ViewInfo& i = hub.info(v);
        if (i.instance == "params")
            continue;
        ViewDescription vd;
        vd.instance = i.instance;
        vd.id = i.id;
        vd.description = i.description;
        vd.kind = viewKindName(i.kind);
        vd.maxElements = i.maxElements;
        vd.stride = i.stride;
        vd.live = i.live;
        for (size_t m = 0; m < d.modules.size() && vd.module.empty(); ++m)
        {
            ModuleInfo& info = d.modules[m];
            bool owns = false;
            if (info.kind == "voice")
                owns = i.instance.rfind("filter", 0) == 0 && i.id.rfind("response", 0) == 0;
            else
            {
                const bool oscillator = info.id.rfind("osc.", 0) == 0;
                const std::string family = info.id.substr(info.id.rfind('.') + 1) + "_";
                for (const auto& inst : info.instances)
                    if (inst.id == i.instance)
                        owns = !oscillator || i.id.rfind(family, 0) == 0;
            }
            if (!owns)
                continue;
            vd.module = info.id;
            if (std::find(info.views.begin(), info.views.end(), vd.id) == info.views.end())
                info.views.push_back(vd.id);
        }
        d.views.push_back(std::move(vd));
    }
}
} // namespace

ProductDescription describe(const ModuleRegistry& registry, ModuleRack* live, const ViewHub* hub)
{
    const ParameterModel& model = registry.parameters();
    ProductDescription d;
    d.engine = versionString();
    d.product = registry.layout().id;
    d.oscillators = registry.layout().voice.oscillators;
    d.filters = registry.layout().voice.filters;
    d.macros = model.macros();
    d.sections = model.sections();
    d.pages = model.pages();
    d.params = model.params();
    for (const auto& m : registry.modules())
    {
        ModuleInfo info;
        info.id = m.id;
        info.name = m.name;
        info.kind = moduleKindName(m.kind);
        info.category = m.category;
        info.version = m.version;
        info.cpu = cpuName(m.cpu);
        info.description = m.description;
        info.status = moduleStatusName(m.status);
        info.supersededBy = m.supersededBy;
        info.inputs = m.inputs;
        info.outputs = m.outputs;
        info.views = m.views;
        info.capabilities = m.capabilities;
        info.instanceOf = m.instanceOf;
        if (live != nullptr && m.kind == ModuleKind::Effect)
        {
            const auto add = [&](bool has, const char* id)
            {
                if (has && std::find(info.capabilities.begin(), info.capabilities.end(), id) == info.capabilities.end())
                    info.capabilities.push_back(id);
            };
            add(live->findCapability<GainReductionMeter>(m.id) != nullptr, "gain_reduction");
            add(live->findCapability<StepPosition>(m.id) != nullptr, "step_position");
        }
        for (const auto& p : m.params)
            info.params.push_back(model.indexOf(p.id));
        if (m.perOscillator)
        {
            for (int slot = 1; slot <= d.oscillators; ++slot)
            {
                ModuleInstanceInfo inst;
                inst.id = "osc" + std::to_string(slot);
                inst.name = "Osc " + std::to_string(slot);
                for (const int i : info.params)
                    if (slotOf(d.params[static_cast<size_t>(i)].id) == slot)
                        inst.params.push_back(i);
                info.instances.push_back(std::move(inst));
            }
        }
        else
            info.instances.push_back({m.id, m.name, info.params});
        d.modules.push_back(std::move(info));
    }
    if (hub != nullptr)
        addViews(d, *hub);
    return d;
}

std::string describeJSON(const ModuleRegistry& registry, ModuleRack* live, const ViewHub* hub)
{
    return describe(registry, live, hub).toJson();
}

std::string ProductDescription::toJson(int indent) const
{
    Value doc = Value::object();
    doc.set("schema", schema);
    doc.set("format", "crucible-product-description");
    doc.set("engine", engine);
    Value prod = Value::object();
    prod.set("id", product);
    prod.set("oscillators", oscillators);
    prod.set("filters", filters);
    prod.set("macros", macros);
    doc.set("product", prod);
    Value secs = Value::array();
    for (const auto& s : sections)
    {
        Value o = Value::object();
        o.set("id", s.id);
        o.set("name", s.name);
        secs.push(o);
    }
    doc.set("sections", secs);
    Value pgs = Value::array();
    for (const auto& p : pages)
    {
        Value o = Value::object();
        o.set("id", p.id);
        o.set("name", p.name);
        o.set("section", p.section);
        o.set("title", p.layoutTitle);
        o.set("description", p.layoutDescription);
        pgs.push(o);
    }
    doc.set("pages", pgs);
    Value mods = Value::array();
    for (const auto& m : modules)
    {
        Value o = Value::object();
        o.set("id", m.id);
        o.set("name", m.name);
        o.set("kind", m.kind);
        o.set("category", m.category);
        o.set("version", m.version);
        o.set("cpu", m.cpu);
        o.set("description", m.description);
        o.set("status", m.status);
        o.set("superseded_by", m.supersededBy);
        o.set("inputs", portArray(m.inputs));
        o.set("outputs", portArray(m.outputs));
        o.set("views", stringArray(m.views));
        o.set("capabilities", stringArray(m.capabilities));
        o.set("params", indexArray(m.params));
        if (!m.instanceOf.empty())
            o.set("instance_of", m.instanceOf);
        Value insts = Value::array();
        for (const auto& i : m.instances)
        {
            Value io = Value::object();
            io.set("id", i.id);
            io.set("name", i.name);
            io.set("params", indexArray(i.params));
            insts.push(io);
        }
        o.set("instances", insts);
        mods.push(o);
    }
    doc.set("modules", mods);
    Value vs = Value::array();
    for (const auto& v : views)
    {
        Value o = Value::object();
        o.set("instance", v.instance);
        o.set("id", v.id);
        o.set("description", v.description);
        o.set("kind", v.kind);
        o.set("max_elements", v.maxElements);
        o.set("stride", v.stride);
        o.set("live", v.live);
        o.set("module", v.module);
        vs.push(o);
    }
    doc.set("views", vs);
    Value ps = Value::array();
    for (size_t i = 0; i < params.size(); ++i)
    {
        const auto& p = params[i];
        Value o = Value::object();
        o.set("index", static_cast<int>(i));
        o.set("id", p.id);
        o.set("name", p.name);
        o.set("shortName", p.shortName);
        o.set("module", p.module);
        o.set("group", p.group);
        o.set("page", p.page);
        o.set("pageId", p.pageId);
        o.set("type", typeName(p.type));
        o.set("min", p.min);
        o.set("max", p.max);
        o.set("default", p.defaultValue);
        o.set("interval", p.interval);
        o.set("skew", p.skew);
        o.set("unit", p.unit);
        o.set("steps", p.steps);
        o.set("choices", stringArray(p.choices));
        if (!p.unavailable.empty())
            o.set("unavailable", indexArray(p.unavailable));
        o.set("smoothing", smoothingName(p.smoothing));
        o.set("polarity", polarityName(p.polarity));
        o.set("uiHint", uiHintName(p.uiHint));
        o.set("format", textFormatName(p.textFormat));
        o.set("modulatable", p.modulatable);
        o.set("voiceMod", p.voiceMod);
        o.set("routable", p.routable);
        o.set("automatable", p.automatable);
        o.set("exposed", p.exposed);
        o.set("scene", p.scene);
        o.set("since", p.since);
        o.set("destinationChoice", p.destinationChoice);
        ps.push(o);
    }
    doc.set("params", ps);
    return doc.dump(indent);
}

bool ProductDescription::fromJson(const std::string& text, ProductDescription& out, std::string& error)
{
    Value doc;
    if (!json::parse(text, doc, error))
        return false;
    if (!doc.isObject() || doc["format"].asString("") != "crucible-product-description")
    {
        error = "not a crucible-product-description document";
        return false;
    }
    ProductDescription d;
    d.schema = static_cast<int>(doc["schema"].asNumber(2));
    d.engine = doc["engine"].asString("");
    const Value& prod = doc["product"];
    d.product = prod["id"].asString("");
    d.oscillators = static_cast<int>(prod["oscillators"].asNumber());
    d.filters = static_cast<int>(prod["filters"].asNumber());
    d.macros = static_cast<int>(prod["macros"].asNumber());
    for (const auto& s : doc["sections"].elements())
        d.sections.push_back({s["id"].asString(""), s["name"].asString("")});
    for (const auto& p : doc["pages"].elements())
    {
        PageDef pg;
        pg.id = p["id"].asString("");
        pg.name = p["name"].asString("");
        pg.section = static_cast<int>(p["section"].asNumber());
        pg.layoutTitle = p["title"].asString("");
        pg.layoutDescription = p["description"].asString("");
        d.pages.push_back(pg);
    }
    for (const auto& m : doc["modules"].elements())
    {
        ModuleInfo info;
        info.id = m["id"].asString("");
        info.name = m["name"].asString("");
        info.kind = m["kind"].asString("");
        info.category = m["category"].asString("");
        info.version = static_cast<int>(m["version"].asNumber(1));
        info.cpu = m["cpu"].asString("");
        info.description = m["description"].asString("");
        info.status = m["status"].asString("current");
        info.supersededBy = m["superseded_by"].asString("");
        info.inputs = readPorts(m["inputs"]);
        info.outputs = readPorts(m["outputs"]);
        info.views = readStrings(m["views"]);
        info.capabilities = readStrings(m["capabilities"]);
        info.params = readInts(m["params"]);
        info.instanceOf = m["instance_of"].asString("");
        for (const auto& i : m["instances"].elements())
            info.instances.push_back({i["id"].asString(""), i["name"].asString(""), readInts(i["params"])});
        d.modules.push_back(std::move(info));
    }
    for (const auto& o : doc["views"].elements())
    {
        ViewDescription v;
        v.instance = o["instance"].asString("");
        v.id = o["id"].asString("");
        v.description = o["description"].asString("");
        v.kind = o["kind"].asString("vector");
        v.maxElements = static_cast<int>(o["max_elements"].asNumber());
        v.stride = static_cast<int>(o["stride"].asNumber(1));
        v.live = o["live"].asBool(true);
        v.module = o["module"].asString("");
        d.views.push_back(std::move(v));
    }
    for (const auto& o : doc["params"].elements())
    {
        ParamDef p;
        p.id = o["id"].asString("");
        p.name = o["name"].asString("");
        p.shortName = o["shortName"].asString("");
        p.module = o["module"].asString("");
        p.group = o["group"].asString("");
        p.page = static_cast<int>(o["page"].asNumber());
        p.pageId = o["pageId"].asString("");
        if (!typeFromName(o["type"].asString(""), p.type) || !smoothingFromName(o["smoothing"].asString(""), p.smoothing) ||
            !polarityFromName(o["polarity"].asString(""), p.polarity) ||
            !uiHintFromName(o["uiHint"].asString(""), p.uiHint) ||
            !textFormatFromName(o["format"].asString(""), p.textFormat))
        {
            error = "bad enum in parameter " + p.id;
            return false;
        }
        p.min = static_cast<float>(o["min"].asNumber());
        p.max = static_cast<float>(o["max"].asNumber());
        p.defaultValue = static_cast<float>(o["default"].asNumber());
        p.interval = static_cast<float>(o["interval"].asNumber());
        p.skew = static_cast<float>(o["skew"].asNumber(1));
        p.unit = o["unit"].asString("");
        p.steps = static_cast<int>(o["steps"].asNumber());
        p.choices = readStrings(o["choices"]);
        for (const auto& u : o["unavailable"].elements())
            p.unavailable.push_back(static_cast<int>(u.asNumber()));
        p.modulatable = o["modulatable"].asBool();
        p.voiceMod = o["voiceMod"].asBool();
        p.routable = o["routable"].asBool();
        p.automatable = o["automatable"].asBool(true);
        p.exposed = o["exposed"].asBool(p.automatable);
        p.scene = o["scene"].asBool(true);
        p.since = static_cast<int>(o["since"].asNumber(1));
        p.destinationChoice = o["destinationChoice"].asBool();
        d.params.push_back(std::move(p));
    }
    out = std::move(d);
    return true;
}

std::string productLayoutToJson(const ProductLayout& layout)
{
    Value doc = Value::object();
    doc.set("id", layout.id);
    doc.set("oscillators", layout.voice.oscillators);
    doc.set("filters", layout.voice.filters);
    doc.set("macros", layout.macros);
    Value secs = Value::array();
    for (const auto& s : layout.sections)
    {
        Value o = Value::object();
        o.set("id", s.id);
        o.set("name", s.name);
        secs.push(o);
    }
    doc.set("sections", secs);
    Value pgs = Value::array();
    for (const auto& p : layout.pages)
    {
        Value o = Value::object();
        o.set("id", p.id);
        o.set("name", p.name);
        o.set("section", p.section);
        o.set("title", p.layoutTitle);
        o.set("description", p.layoutDescription);
        pgs.push(o);
    }
    doc.set("pages", pgs);
    return doc.dump(2);
}

bool productLayoutFromJson(const std::string& text, ProductLayout& out, std::string& error)
{
    Value doc;
    if (!json::parse(text, doc, error))
        return false;
    if (!doc.isObject())
    {
        error = "a product layout is a JSON object";
        return false;
    }
    ProductLayout l = ModuleRegistry::neutralLayout();
    l.id = doc["id"].asString(l.id);
    l.voice.oscillators = static_cast<int>(doc["oscillators"].asNumber(l.voice.oscillators));
    l.voice.filters = static_cast<int>(doc["filters"].asNumber(l.voice.filters));
    l.macros = static_cast<int>(doc["macros"].asNumber(l.macros));
    if (doc.has("sections"))
    {
        l.sections.clear();
        for (const auto& s : doc["sections"].elements())
            l.sections.push_back({s["id"].asString(""), s["name"].asString("")});
    }
    if (doc.has("pages"))
    {
        l.pages.clear();
        for (const auto& p : doc["pages"].elements())
        {
            PageDef pg;
            pg.id = p["id"].asString("");
            pg.name = p["name"].asString("");
            pg.section = static_cast<int>(p["section"].asNumber());
            pg.layoutTitle = p["title"].asString("");
            pg.layoutDescription = p["description"].asString("");
            l.pages.push_back(pg);
        }
    }
    out = std::move(l);
    return true;
}

std::string Engine::describeJSON() const
{
    return crucible::describeJSON(registry(), &core->engine().effects(), &core->views());
}

} // namespace crucible
