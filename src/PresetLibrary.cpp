// SPDX-License-Identifier: MIT
#include <crucible/PresetLibrary.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <random>
#include <set>

#include "Utf8Path.h"

namespace crucible
{
namespace fs = std::filesystem;

namespace
{
constexpr const char* kExtension = ".crucible-patch";

// Files written under the earlier project name keep loading.
bool isPatchExtension(const fs::path& extension)
{
    return extension == kExtension;
}

std::string lower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string sanitiseFileName(const std::string& name)
{
    std::string out;
    for (const char c : name)
        out.push_back((static_cast<unsigned char>(c) < 32 || std::strchr("/\\:*?\"<>|", c) != nullptr) ? '_' : c);
    while (!out.empty() && (out.back() == ' ' || out.back() == '.'))
        out.pop_back();
    while (!out.empty() && (out.front() == ' ' || out.front() == '.'))
        out.erase(out.begin());
    if (out.size() > 100)
        out.resize(100);
    return out.empty() ? "Untitled" : out;
}

std::string sanitiseBank(const std::string& bank)
{
    const auto b = sanitiseFileName(bank);
    return b;
}

bool readFile(const std::string& path, std::string& out)
{
    std::ifstream f(crucible::utf8Path(path), std::ios::binary);
    if (!f)
        return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

bool writeFileAtomic(const std::string& path, const void* data, size_t size)
{
    std::error_code ec;
    fs::create_directories(crucible::utf8Path(path).parent_path(), ec);
    const std::string tmp = path + ".tmp";
    {
        std::ofstream f(crucible::utf8Path(tmp), std::ios::binary | std::ios::trunc);
        if (!f)
            return false;
        f.write(static_cast<const char*>(data), static_cast<std::streamsize>(size));
        if (!f)
            return false;
    }
    fs::rename(crucible::utf8Path(tmp), crucible::utf8Path(path), ec);
    if (ec)
    {
        fs::remove(crucible::utf8Path(tmp), ec);
        return false;
    }
    return true;
}

std::vector<std::uint8_t> floatBytes(const std::vector<float>& v)
{
    std::vector<std::uint8_t> bytes(v.size() * 4);
    for (size_t i = 0; i < v.size(); ++i)
    {
        std::uint32_t u;
        std::memcpy(&u, &v[i], 4);
        for (int b = 0; b < 4; ++b)
            bytes[i * 4 + static_cast<size_t>(b)] = static_cast<std::uint8_t>((u >> (8 * b)) & 0xffU);
    }
    return bytes;
}

bool bytesToFloats(const std::vector<std::uint8_t>& bytes, std::vector<float>& out)
{
    if (bytes.size() % 4 != 0)
        return false;
    out.resize(bytes.size() / 4);
    for (size_t i = 0; i < out.size(); ++i)
    {
        const std::uint32_t u =
            static_cast<std::uint32_t>(bytes[i * 4]) | (static_cast<std::uint32_t>(bytes[i * 4 + 1]) << 8) |
            (static_cast<std::uint32_t>(bytes[i * 4 + 2]) << 16) | (static_cast<std::uint32_t>(bytes[i * 4 + 3]) << 24);
        std::memcpy(&out[i], &u, 4);
    }
    return true;
}

std::string isoNow()
{
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &t);
#else
    gmtime_r(&t, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm);
    return buf;
}

std::string randomId()
{
    static std::atomic<std::uint64_t> counter{0};
    std::random_device rd;
    std::uint64_t parts[4] = {static_cast<std::uint64_t>(rd()) << 32 | rd(),
                              static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count()),
                              counter.fetch_add(1), static_cast<std::uint64_t>(rd())};
    return contentHash(parts, sizeof parts);
}

std::int64_t fileStamp(const fs::path& p)
{
    std::error_code ec;
    const auto t = fs::last_write_time(p, ec);
    return ec ? 0 : static_cast<std::int64_t>(t.time_since_epoch().count());
}
} // namespace

// ---- Asset store.

std::string AssetStore::put(const void* data, size_t size) const
{
    if (!valid())
        return {};
    const std::string hash = contentHash(data, size);
    const std::string path = crucible::pathToUtf8(crucible::utf8Path(dir) / (hash + ".bin"));
    std::error_code ec;
    if (fs::exists(crucible::utf8Path(path), ec))
        return hash;
    return writeFileAtomic(path, data, size) ? hash : std::string{};
}

bool AssetStore::get(const std::string& hash, std::vector<std::uint8_t>& out) const
{
    if (!valid() || hash.empty() || hash.find_first_of("/\\.") != std::string::npos)
        return false;
    std::string text;
    if (!readFile(crucible::pathToUtf8(crucible::utf8Path(dir) / (hash + ".bin")), text))
        return false;
    out.assign(text.begin(), text.end());
    return true;
}

bool AssetStore::has(const std::string& hash) const
{
    std::error_code ec;
    return valid() && !hash.empty() && fs::exists(crucible::utf8Path(dir) / (hash + ".bin"), ec);
}

bool externaliseAssets(Patch& p, const AssetStore& store)
{
    if (!store.valid())
        return false;
    const auto putFloats = [&](const std::vector<float>& v)
    {
        const auto bytes = floatBytes(v);
        return store.put(bytes.data(), bytes.size());
    };
    std::vector<PatchAssetRef> refs;
    for (const auto& w : p.userWaves)
    {
        PatchAssetRef r;
        r.kind = "wavetable";
        r.name = w.name;
        r.slot = w.slot;
        r.hash = putFloats(w.frames);
        if (r.hash.empty())
            return false;
        refs.push_back(r);
        if (w.zones.size() >= 2)
        {
            PatchAssetRef z;
            z.kind = "wavetable-zones";
            z.name = w.name;
            z.slot = w.slot;
            z.hash = putFloats(UserWavetableBank::packZones(w));
            if (z.hash.empty())
                return false;
            refs.push_back(z);
        }
    }
    for (const auto& c : p.convolutionIrs)
        for (size_t ch = 0; ch < c.channels.size(); ++ch)
        {
            PatchAssetRef r;
            r.kind = "ir";
            r.instance = c.instance;
            r.name = c.name;
            r.rate = c.rate;
            r.channel = static_cast<int>(ch);
            r.channels = static_cast<int>(c.channels.size());
            r.hash = putFloats(c.channels[ch]);
            if (r.hash.empty())
                return false;
            refs.push_back(r);
        }
    for (const auto& a : p.assets)
    {
        PatchAssetRef r;
        r.kind = a.kind;
        r.name = a.name;
        r.slot = a.slot;
        r.index = a.index;
        r.rate = a.rate;
        r.channels = a.channels;
        if (!a.text.empty())
        {
            r.channel = -1;
            r.hash = store.put(a.text.data(), a.text.size());
        }
        else
            r.hash = putFloats(a.frames);
        if (r.hash.empty())
            return false;
        refs.push_back(r);
    }
    p.userWaves.clear();
    p.convolutionIrs.clear();
    p.assets.clear();
    p.assetRefs.insert(p.assetRefs.end(), refs.begin(), refs.end());
    return true;
}

void resolveAssets(Patch& p, const std::vector<const AssetStore*>& stores, std::vector<std::string>& missing)
{
    std::vector<PatchAssetRef> unresolved;
    std::vector<std::pair<int, std::vector<float>>> zoneBlobs;
    for (const auto& r : p.assetRefs)
    {
        std::vector<std::uint8_t> bytes;
        bool found = false;
        for (const auto* s : stores)
            if (s != nullptr && s->get(r.hash, bytes))
            {
                found = true;
                break;
            }
        std::vector<float> floats;
        if (!found || (r.channel != -1 && !bytesToFloats(bytes, floats)))
        {
            missing.push_back(r.kind + " " + r.name);
            unresolved.push_back(r);
            continue;
        }
        if (r.kind == "wavetable")
        {
            UserWavetableBank::Record rec;
            rec.slot = r.slot;
            rec.name = r.name;
            rec.frames = std::move(floats);
            p.userWaves.push_back(std::move(rec));
        }
        else if (r.kind == "wavetable-zones")
            zoneBlobs.emplace_back(r.slot, std::move(floats));
        else if (r.kind == "ir" && !r.instance.empty())
        {
            auto entry = std::find_if(p.convolutionIrs.begin(), p.convolutionIrs.end(),
                                      [&](const Patch::ConvolutionIrData& d) { return d.instance == r.instance; });
            if (entry == p.convolutionIrs.end())
            {
                p.convolutionIrs.emplace_back();
                entry = p.convolutionIrs.end() - 1;
                entry->instance = r.instance;
            }
            const auto want = static_cast<size_t>(std::max(r.channels, 1));
            if (entry->channels.size() < want)
                entry->channels.resize(want);
            entry->name = r.name;
            entry->rate = r.rate > 0.0 ? r.rate : 48000.0;
            entry->layout = want == 4 ? "truestereo" : want == 2 ? "stereo" : "mono";
            if (r.channel >= 0 && static_cast<size_t>(r.channel) < want)
                entry->channels[static_cast<size_t>(r.channel)] = std::move(floats);
        }
        else if (r.kind == "ir")
        {
            // A first-generation convolver response (no instance): dropped with the module.
        }
        else
        {
            PatchAsset a;
            a.kind = r.kind;
            a.name = r.name;
            a.slot = r.slot;
            a.index = r.index;
            a.rate = r.rate;
            a.channels = r.channels;
            if (r.channel == -1)
                a.text.assign(bytes.begin(), bytes.end());
            else
                a.frames = std::move(floats);
            p.assets.push_back(std::move(a));
        }
    }
    for (auto& [slot, blob] : zoneBlobs)
        for (auto& w : p.userWaves)
            if (w.slot == slot)
                UserWavetableBank::unpackZones(blob, w); // a malformed blob leaves the one-zone frames
    p.assetRefs = std::move(unresolved);
}

// ---- Library.

PresetLibrary::PresetLibrary(std::string factory, std::string user)
    : factoryRoot(std::move(factory)), userRoot(std::move(user)), now(&isoNow), newId(&randomId)
{
    if (!userRoot.empty())
        assets = AssetStore(crucible::pathToUtf8(crucible::utf8Path(userRoot) / "assets"));
    if (!factoryRoot.empty())
        factoryAssets = AssetStore(crucible::pathToUtf8(crucible::utf8Path(factoryRoot) / "assets"));
}

PresetEntry PresetLibrary::makeEntry(const Patch& p, const std::string& bank, bool factory, const std::string& path) const
{
    PresetEntry e;
    e.id = p.meta.id;
    e.name = p.meta.name;
    e.author = p.meta.author;
    e.product = p.meta.product;
    e.category = p.meta.category;
    e.description = p.meta.description;
    e.tags = p.meta.tags;
    e.rating = p.meta.rating;
    e.favourite = p.meta.favourite;
    e.created = p.meta.created;
    e.modified = p.meta.modified;
    e.bank = bank;
    e.factory = factory;
    e.builtin = path.empty();
    e.path = path;
    if (e.name.empty())
        e.name = path.empty() ? "Untitled" : crucible::pathToUtf8(crucible::utf8Path(path).stem());
    if (e.id.empty())
        e.id = "file-" + contentHash(bank + "/" + e.name + "/" + p.meta.created);
    return e;
}

bool PresetLibrary::readEntry(const std::string& path, const std::string& bank, bool factory, PresetEntry& out) const
{
    Patch p;
    std::string error;
    if (!loadPatchFile(path, p, error))
        return false;
    out = makeEntry(p, bank, factory, path);
    return true;
}

namespace
{
json::Value entryToJson(const PresetEntry& e)
{
    using json::Value;
    Value o = Value::object();
    o.set("id", e.id);
    o.set("name", e.name);
    o.set("author", e.author);
    o.set("product", e.product);
    o.set("category", e.category);
    o.set("description", e.description);
    Value tags = Value::array();
    for (const auto& t : e.tags)
        tags.push(t);
    o.set("tags", tags);
    o.set("rating", e.rating);
    o.set("favourite", e.favourite);
    o.set("created", e.created);
    o.set("modified", e.modified);
    return o;
}

PresetEntry entryFromJson(const json::Value& o)
{
    PresetEntry e;
    e.id = o["id"].asString("");
    e.name = o["name"].asString("");
    e.author = o["author"].asString("");
    e.product = o["product"].asString("");
    e.category = o["category"].asString("");
    e.description = o["description"].asString("");
    for (const auto& t : o["tags"].elements())
        e.tags.push_back(t.asString(""));
    e.rating = static_cast<int>(o["rating"].asNumber(0.0));
    e.favourite = o["favourite"].asBool(false);
    e.created = o["created"].asString("");
    e.modified = o["modified"].asString("");
    return e;
}
} // namespace

int PresetLibrary::scan()
{
    // Cache of earlier results by path.
    struct Cached
    {
        std::int64_t size = 0;
        std::int64_t stamp = 0;
        PresetEntry entry;
    };
    std::map<std::string, Cached> cache;
    const std::string cachePath =
        userRoot.empty() ? std::string{} : crucible::pathToUtf8(crucible::utf8Path(userRoot) / ".cache" / "index.json");
    if (!cachePath.empty())
    {
        std::string text, error;
        json::Value doc;
        if (readFile(cachePath, text) && json::parse(text, doc, error))
            for (const auto& o : doc["files"].elements())
            {
                Cached c;
                c.size = static_cast<std::int64_t>(o["size"].asNumber(0.0));
                c.stamp = static_cast<std::int64_t>(std::strtoll(o["stamp"].asString("0").c_str(), nullptr, 10));
                c.entry = entryFromJson(o["entry"]);
                cache[o["path"].asString("")] = std::move(c);
            }
    }

    std::vector<PresetEntry> found;
    std::map<std::string, Cached> fresh;
    const auto scanRoot = [&](const std::string& root, bool factory)
    {
        if (root.empty())
            return;
        std::error_code ec;
        if (!fs::is_directory(crucible::utf8Path(root), ec))
            return;
        std::vector<fs::path> files;
        for (fs::recursive_directory_iterator
                 it(crucible::utf8Path(root), fs::directory_options::skip_permission_denied, ec),
             end;
             !ec && it != end; it.increment(ec))
        {
            const auto name = crucible::pathToUtf8(it->path().filename());
            if (it->is_directory(ec) && (name == "assets" || (!name.empty() && name[0] == '.')))
            {
                it.disable_recursion_pending();
                continue;
            }
            if (it->is_regular_file(ec) && (isPatchExtension(it->path().extension()) || it->path().extension() == fileExt))
                files.push_back(it->path());
        }
        std::sort(files.begin(), files.end());
        for (const auto& f : files)
        {
            const auto rel = fs::relative(f, crucible::utf8Path(root), ec);
            const std::string bank =
                rel.has_parent_path() ? crucible::pathToUtf8(*rel.begin()) : (factory ? "Factory" : "User");
            const std::string key = crucible::pathToUtf8(f);
            const std::int64_t size = static_cast<std::int64_t>(fs::file_size(f, ec));
            const std::int64_t stamp = fileStamp(f);
            PresetEntry e;
            const auto hit = cache.find(key);
            if (hit != cache.end() && hit->second.size == size && hit->second.stamp == stamp)
                e = hit->second.entry;
            else if (!readEntry(key, bank, factory, e))
                continue;
            e.bank = bank;
            e.factory = factory;
            e.builtin = false;
            e.path = key;
            fresh[key] = {size, stamp, e};
            found.push_back(std::move(e));
        }
    };
    scanRoot(factoryRoot, true);
    scanRoot(userRoot, false);

    if (!cachePath.empty())
    {
        json::Value doc = json::Value::object();
        json::Value files = json::Value::array();
        for (const auto& [path, c] : fresh)
        {
            json::Value o = json::Value::object();
            o.set("path", path);
            o.set("size", static_cast<double>(c.size));
            o.set("stamp", std::to_string(c.stamp));
            o.set("entry", entryToJson(c.entry));
            files.push(o);
        }
        doc.set("files", files);
        const std::string text = doc.dump(0);
        writeFileAtomic(cachePath, text.data(), text.size());
    }

    for (const auto& b : builtins)
    {
        PresetEntry e = makeEntry(b.patch, b.bank, true, {});
        e.builtin = true;
        found.push_back(std::move(e));
    }

    // Ids must be unique inside the library: later duplicates (copied files) get a suffix.
    std::set<std::string> seen;
    for (auto& e : found)
    {
        std::string id = e.id;
        for (int n = 2; !seen.insert(id).second; ++n)
            id = e.id + "~" + std::to_string(n);
        e.id = id;
    }
    list = std::move(found);
    loadOverlay();
    for (auto& e : list)
    {
        const auto o = overlay.find(e.id);
        if (e.factory && o != overlay.end())
        {
            e.favourite = o->second.favourite;
            e.rating = o->second.rating;
        }
    }
    return static_cast<int>(list.size());
}

void PresetLibrary::addBuiltin(const std::string& bank, Patch patch)
{
    if (patch.meta.id.empty())
        patch.meta.id = "builtin-" + contentHash(bank + "/" + patch.meta.name);
    builtins.push_back({bank, patch});
    PresetEntry e = makeEntry(patch, bank, true, {});
    e.builtin = true;
    const auto o = overlay.find(e.id);
    if (o != overlay.end())
    {
        e.favourite = o->second.favourite;
        e.rating = o->second.rating;
    }
    list.push_back(std::move(e));
}

const PresetEntry* PresetLibrary::find(const std::string& id) const
{
    for (const auto& e : list)
        if (e.id == id)
            return &e;
    return nullptr;
}

PresetEntry* PresetLibrary::findMutable(const std::string& id)
{
    for (auto& e : list)
        if (e.id == id)
            return &e;
    return nullptr;
}

std::vector<PresetEntry> PresetLibrary::search(const PresetQuery& q) const
{
    std::vector<std::string> words;
    {
        std::string w;
        for (const char c : lower(q.text) + " ")
        {
            if (c == ' ')
            {
                if (!w.empty())
                    words.push_back(w);
                w.clear();
            }
            else
                w.push_back(c);
        }
    }
    std::set<std::string> cats, tags;
    for (const auto& c : q.categories)
        cats.insert(lower(c));
    for (const auto& t : q.tags)
        tags.insert(lower(t));

    std::vector<PresetEntry> out;
    for (const auto& e : list)
    {
        if (q.source == PresetQuery::Source::Factory && !e.factory)
            continue;
        if (q.source == PresetQuery::Source::User && e.factory)
            continue;
        if (!q.bank.empty() && lower(e.bank) != lower(q.bank))
            continue;
        if (q.favouritesOnly && !e.favourite)
            continue;
        if (e.rating < q.minRating)
            continue;
        if (!cats.empty() && cats.count(lower(e.category)) == 0)
            continue;
        std::set<std::string> have;
        for (const auto& t : e.tags)
            have.insert(lower(t));
        bool all = true;
        for (const auto& t : tags)
            all = all && have.count(t) != 0;
        if (!all)
            continue;
        if (!words.empty())
        {
            std::string hay = lower(e.name + " " + e.author + " " + e.category + " " + e.description + " " + e.bank);
            for (const auto& t : e.tags)
                hay += " " + lower(t);
            bool ok = true;
            for (const auto& w : words)
                ok = ok && hay.find(w) != std::string::npos;
            if (!ok)
                continue;
        }
        out.push_back(e);
    }
    const auto key = [&](const PresetEntry& a, const PresetEntry& b)
    {
        switch (q.sort)
        {
        case PresetSort::Name:
            return lower(a.name) < lower(b.name);
        case PresetSort::Category:
            return lower(a.category) != lower(b.category) ? lower(a.category) < lower(b.category)
                                                          : lower(a.name) < lower(b.name);
        case PresetSort::Rating:
            return a.rating < b.rating;
        case PresetSort::Modified:
            return a.modified < b.modified;
        case PresetSort::Created:
            return a.created < b.created;
        }
        return false;
    };
    std::stable_sort(out.begin(), out.end(),
                     [&](const PresetEntry& a, const PresetEntry& b) { return q.descending ? key(b, a) : key(a, b); });
    return out;
}

std::vector<PresetBank> PresetLibrary::banks() const
{
    std::map<std::pair<bool, std::string>, int> counts;
    for (const auto& e : list)
        ++counts[{!e.factory, e.bank}];
    // Empty user bank folders count too.
    std::error_code ec;
    if (!userRoot.empty() && fs::is_directory(crucible::utf8Path(userRoot), ec))
        for (const auto& d : fs::directory_iterator(crucible::utf8Path(userRoot), ec))
            if (d.is_directory(ec) && crucible::pathToUtf8(d.path().filename())[0] != '.' && d.path().filename() != "assets")
                counts.try_emplace({true, crucible::pathToUtf8(d.path().filename())}, 0);
    std::vector<PresetBank> out;
    for (const auto& [k, n] : counts)
        out.push_back({k.second, !k.first, n});
    std::stable_sort(out.begin(), out.end(),
                     [](const PresetBank& a, const PresetBank& b) { return a.factory && !b.factory; });
    return out;
}

namespace
{
std::vector<std::pair<std::string, int>> tally(const std::map<std::string, std::pair<std::string, int>>& m)
{
    std::vector<std::pair<std::string, int>> out;
    for (const auto& [k, v] : m)
        out.emplace_back(v.first, v.second);
    std::stable_sort(out.begin(), out.end(), [](const auto& a, const auto& b) { return a.second > b.second; });
    return out;
}
} // namespace

std::vector<std::pair<std::string, int>> PresetLibrary::categories() const
{
    std::map<std::string, std::pair<std::string, int>> m;
    for (const auto& e : list)
        if (!e.category.empty())
        {
            auto& slot = m[lower(e.category)];
            slot.first = slot.second == 0 ? e.category : slot.first;
            ++slot.second;
        }
    return tally(m);
}

std::vector<std::pair<std::string, int>> PresetLibrary::tags() const
{
    std::map<std::string, std::pair<std::string, int>> m;
    for (const auto& e : list)
        for (const auto& t : e.tags)
        {
            auto& slot = m[lower(t)];
            slot.first = slot.second == 0 ? t : slot.first;
            ++slot.second;
        }
    return tally(m);
}

bool PresetLibrary::load(const std::string& id, Patch& out, std::string& error, std::vector<std::string>* missingOut) const
{
    const PresetEntry* e = find(id);
    if (e == nullptr)
    {
        error = "no such preset";
        return false;
    }
    Patch p;
    if (e->builtin)
    {
        const std::string raw = id.rfind("~", 0) == std::string::npos ? id : id.substr(0, id.find('~'));
        bool ok = false;
        for (const auto& b : builtins)
            if (b.patch.meta.id == raw && b.bank == e->bank)
            {
                p = b.patch;
                ok = true;
                break;
            }
        if (!ok)
        {
            error = "built-in preset missing";
            return false;
        }
    }
    else if (!loadPatchFile(e->path, p, error))
        return false;
    p.meta.id = e->id;
    if (e->factory)
    {
        p.meta.favourite = e->favourite;
        p.meta.rating = e->rating;
    }
    std::vector<std::string> missing;
    resolveAssets(p, {&assets, &factoryAssets}, missing);
    if (missingOut != nullptr)
        *missingOut = missing;
    out = std::move(p);
    return true;
}

bool PresetLibrary::writePatch(const std::string& path, const Patch& patch, std::string& error) const
{
    const std::string text = patchToJson(patch);
    if (!writeFileAtomic(path, text.data(), text.size()))
    {
        error = "cannot write " + path;
        return false;
    }
    return true;
}

std::string PresetLibrary::uniquePath(const std::string& bank, const std::string& name, const std::string& ignore) const
{
    const fs::path dir = crucible::utf8Path(userRoot) / crucible::utf8Path(sanitiseBank(bank));
    const std::string stem = sanitiseFileName(name);
    for (int n = 1;; ++n)
    {
        const fs::path candidate = dir / crucible::utf8Path(stem + (n == 1 ? "" : " " + std::to_string(n)) + fileExt);
        std::error_code ec;
        if (crucible::pathToUtf8(candidate) == ignore || !fs::exists(candidate, ec))
            return crucible::pathToUtf8(candidate);
    }
}

std::string PresetLibrary::saveImpl(Patch patch, const PresetSaveOptions& options, bool asNew, std::string& error)
{
    if (userRoot.empty())
    {
        error = "no user library";
        return {};
    }
    if (patch.meta.name.empty())
        patch.meta.name = "Untitled";
    const std::string stamp = now();
    PresetEntry* existing = (!asNew && !patch.meta.id.empty()) ? findMutable(patch.meta.id) : nullptr;
    if (existing != nullptr && existing->factory)
        existing = nullptr;
    if (existing == nullptr)
    {
        patch.meta.id = newId();
        if (patch.meta.created.empty() || asNew)
            patch.meta.created = stamp;
    }
    patch.meta.modified = stamp;
    if (options.referenceAssets && !externaliseAssets(patch, assets))
    {
        error = "cannot write assets";
        return {};
    }
    const std::string bank = existing != nullptr ? existing->bank : options.bank.empty() ? "User" : options.bank;
    const std::string path = uniquePath(bank, patch.meta.name, existing != nullptr ? existing->path : std::string{});
    if (!writePatch(path, patch, error))
        return {};
    if (existing != nullptr && existing->path != path)
    {
        std::error_code ec;
        fs::remove(crucible::utf8Path(existing->path), ec);
    }
    PresetEntry entry = makeEntry(patch, sanitiseBank(bank), false, path);
    if (existing != nullptr)
        *existing = entry;
    else
        list.push_back(entry);
    return entry.id;
}

std::string PresetLibrary::save(Patch patch, const PresetSaveOptions& options, std::string& error)
{
    return saveImpl(std::move(patch), options, false, error);
}

std::string PresetLibrary::saveAs(Patch patch, const PresetSaveOptions& options, std::string& error)
{
    patch.meta.id.clear();
    return saveImpl(std::move(patch), options, true, error);
}

std::string PresetLibrary::importFile(const std::string& path, const PresetSaveOptions& options, std::string& error)
{
    Patch p;
    if (!loadPatchFile(path, p, error))
        return {};
    return saveAs(std::move(p), options, error);
}

bool PresetLibrary::updateMeta(const std::string& id, const std::function<void(PatchMeta&)>& change)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr || e->factory)
        return false;
    Patch p;
    std::string error;
    if (!loadPatchFile(e->path, p, error))
        return false;
    const std::string keepId = e->id;
    p.meta.id = keepId;
    change(p.meta);
    p.meta.id = keepId;
    p.meta.modified = now();
    if (!writePatch(e->path, p, error))
        return false;
    *e = makeEntry(p, e->bank, false, e->path);
    return true;
}

bool PresetLibrary::rename(const std::string& id, const std::string& newName)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr || e->factory || newName.empty())
        return false;
    Patch p;
    std::string error;
    if (!loadPatchFile(e->path, p, error))
        return false;
    p.meta.id = e->id;
    p.meta.name = newName;
    p.meta.modified = now();
    const std::string path = uniquePath(e->bank, newName, e->path);
    if (!writePatch(path, p, error))
        return false;
    if (path != e->path)
    {
        std::error_code ec;
        fs::remove(crucible::utf8Path(e->path), ec);
    }
    *e = makeEntry(p, e->bank, false, path);
    return true;
}

bool PresetLibrary::moveToBank(const std::string& id, const std::string& bank)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr || e->factory || bank.empty())
        return false;
    const std::string target = sanitiseBank(bank);
    if (target == e->bank)
        return true;
    const std::string path = uniquePath(target, e->name, {});
    std::error_code ec;
    fs::create_directories(crucible::utf8Path(path).parent_path(), ec);
    fs::rename(crucible::utf8Path(e->path), crucible::utf8Path(path), ec);
    if (ec)
        return false;
    e->path = path;
    e->bank = target;
    return true;
}

bool PresetLibrary::remove(const std::string& id)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr || e->factory)
        return false;
    std::error_code ec;
    const fs::path trash = crucible::utf8Path(userRoot) / ".trash";
    fs::create_directories(trash, ec);
    fs::path target = trash / crucible::utf8Path(sanitiseFileName(e->name) + " " + e->id.substr(0, 8) + fileExt);
    fs::rename(crucible::utf8Path(e->path), target, ec);
    if (ec)
        return false;
    list.erase(list.begin() + (e - list.data()));
    return true;
}

int PresetLibrary::purgeTrash()
{
    int n = 0;
    std::error_code ec;
    const fs::path trash = crucible::utf8Path(userRoot) / ".trash";
    if (userRoot.empty() || !fs::is_directory(trash, ec))
        return 0;
    for (const auto& f : fs::directory_iterator(trash, ec))
        if (fs::remove(f.path(), ec))
            ++n;
    return n;
}

bool PresetLibrary::createBank(const std::string& bank)
{
    if (userRoot.empty() || bank.empty())
        return false;
    std::error_code ec;
    fs::create_directories(crucible::utf8Path(userRoot) / crucible::utf8Path(sanitiseBank(bank)), ec);
    return !ec;
}

void PresetLibrary::loadOverlay()
{
    overlay.clear();
    if (userRoot.empty())
        return;
    std::string text, error;
    json::Value doc;
    if (!readFile(crucible::pathToUtf8(crucible::utf8Path(userRoot) / "overlay.json"), text) ||
        !json::parse(text, doc, error))
        return;
    for (const auto& [id, v] : doc.entries())
        overlay[id] = {v["favourite"].asBool(false), std::clamp(static_cast<int>(v["rating"].asNumber(0.0)), 0, 5)};
}

void PresetLibrary::saveOverlay() const
{
    if (userRoot.empty())
        return;
    json::Value doc = json::Value::object();
    for (const auto& [id, o] : overlay)
    {
        json::Value v = json::Value::object();
        v.set("favourite", o.favourite);
        v.set("rating", o.rating);
        doc.set(id, v);
    }
    const std::string text = doc.dump(2);
    writeFileAtomic(crucible::pathToUtf8(crucible::utf8Path(userRoot) / "overlay.json"), text.data(), text.size());
}

bool PresetLibrary::setFavourite(const std::string& id, bool favourite)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr)
        return false;
    if (!e->factory)
        return updateMeta(id, [&](PatchMeta& m) { m.favourite = favourite; });
    overlay[id].favourite = favourite;
    e->favourite = favourite;
    saveOverlay();
    return true;
}

bool PresetLibrary::setRating(const std::string& id, int rating)
{
    PresetEntry* e = findMutable(id);
    if (e == nullptr)
        return false;
    rating = std::clamp(rating, 0, 5);
    if (!e->factory)
        return updateMeta(id, [&](PatchMeta& m) { m.rating = rating; });
    overlay[id].rating = rating;
    e->rating = rating;
    saveOverlay();
    return true;
}

} // namespace crucible
