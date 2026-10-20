// SPDX-License-Identifier: MIT
#include <crucible/module/ConvolutionIr.h>
#include <crucible/Engine.h>
#include <crucible/Patch.h>
#include <crucible/PresetLibrary.h>
#include "TestSupport.h"

#include <chrono>
#include <filesystem>
#include <fstream>

using namespace crucible;
using namespace crucible::testing;
namespace fs = std::filesystem;

namespace
{
struct TempDir
{
    fs::path path;
    TempDir()
    {
        static int counter = 0;
        path = fs::temp_directory_path() /
               ("crucible_presets_" +
                std::to_string(static_cast<long long>(std::chrono::steady_clock::now().time_since_epoch().count())) + "_" +
                std::to_string(counter++));
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir()
    {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
    std::string str(const std::string& sub = {}) const { return sub.empty() ? path.string() : (path / sub).string(); }
};

Patch makePatch(const std::string& name, const std::string& category, std::vector<std::string> tags, int rating = 0,
                float cutoff = 1000.0f)
{
    Patch p;
    p.meta.name = name;
    p.meta.author = "Tester";
    p.meta.category = category;
    p.meta.tags = std::move(tags);
    p.meta.rating = rating;
    p.meta.created = "2026-01-01T00:00:00Z";
    p.params.emplace_back("filter_cutoff", cutoff);
    return p;
}

std::string slurp(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

std::vector<std::string> names(const std::vector<PresetEntry>& v)
{
    std::vector<std::string> out;
    for (const auto& e : v)
        out.push_back(e.name);
    return out;
}
} // namespace

class PresetTests : public UnitTest
{
public:
    PresetTests() : UnitTest("Preset metadata, library and migration", "Engine") {}

    void runTest() override
    {
        beginTest("format version 2: new metadata round trips; version 1 documents migrate");
        {
            Patch p = makePatch("Glass Pad", "Pad", {"airy", "slow"}, 4);
            p.meta.id = "abc";
            p.meta.product = "ExampleSynth";
            p.meta.modified = "2026-02-02T00:00:00Z";
            p.meta.description = "A pad";
            p.meta.comment = "notes";
            Patch back;
            std::string error;
            expect(patchFromJson(patchToJson(p), back, error), error);
            expect(back.meta.id == "abc" && back.meta.product == "ExampleSynth" && back.meta.modified == p.meta.modified &&
                       back.meta.description == "A pad" && back.meta.comment == "notes" && back.meta.tags.size() == 2,
                   "metadata survives");
            const std::string v1 = R"({"format": "crucible-patch", "version": 1,
                "meta": {"name": "Old", "author": "Me", "category": "Bass", "tags": ["a"], "rating": 3, "favourite": true,
                         "created": "2025-05-05T10:00:00Z", "comment": "from v1"},
                "params": {"filter_cutoff": 321}})";
            Patch old, old2;
            expect(patchFromJson(v1, old, error), error);
            expect(patchFromJson(v1, old2, error));
            expectEquals(old.version, kPatchFormatVersion);
            expect(!old.meta.id.empty() && old.meta.id == old2.meta.id, "a stable id is derived: " + old.meta.id);
            expectEquals(old.meta.modified, std::string("2025-05-05T10:00:00Z"));
            expectEquals(old.meta.description, std::string("from v1"));
            expect(old.meta.rating == 3 && old.meta.favourite && old.param("filter_cutoff", 0.0f) == 321.0f, "content kept");
            json::Value doc;
            expect(json::parse(patchToJson(old), doc, error) && doc["version"].asNumber() == kPatchFormatVersion,
                   "written as the current version");
            expect(!patchFromJson(R"({"format": "crucible-patch", "version": 4})", old, error), "newer than this engine");
            expect(!patchFromJson(R"({"format": "crucible-patch", "version": 0})", old, error), "version 0 invalid");
        }

        beginTest("library: scan, search, filter, sort, categories and tags");
        {
            TempDir dir;
            fs::create_directories(dir.path / "factory" / "Pads");
            fs::create_directories(dir.path / "factory" / "Basses");
            std::string error;
            auto f1 = makePatch("Glass Pad", "Pad", {"airy", "slow"}, 4);
            auto f2 = makePatch("Deep Sub", "Bass", {"sub", "dark"}, 5);
            auto f3 = makePatch("Warm Pad", "Pad", {"warm", "slow"}, 2);
            f1.meta.id = "f1";
            f2.meta.id = "f2";
            f3.meta.id = "f3";
            f1.meta.modified = "2026-03-01";
            f2.meta.modified = "2026-01-01";
            f3.meta.modified = "2026-02-01";
            expect(savePatchFile(dir.str("factory/Pads/Glass Pad.crucible-patch"), f1));
            expect(savePatchFile(dir.str("factory/Basses/Deep Sub.crucible-patch"), f2));
            expect(savePatchFile(dir.str("factory/Pads/Warm Pad.crucible-patch"), f3));
            PresetLibrary lib(dir.str("factory"), dir.str("user"));
            std::uint64_t counter = 0;
            lib.setIdGenerator([&] { return "id" + std::to_string(++counter); });
            lib.setClock([] { return std::string("2026-06-01T12:00:00Z"); });
            Patch builtin = makePatch("Built In", "Lead", {"bright"}, 1);
            lib.addBuiltin("Core", builtin);
            expectEquals(lib.scan(), 4);
            std::string id = lib.save(makePatch("My Lead", "Lead", {"bright", "mine"}, 3, 5000.0f), {}, error);
            expect(!id.empty(), error);
            expectEquals(static_cast<int>(lib.entries().size()), 5);

            PresetQuery q;
            expectEquals(static_cast<int>(lib.search(q).size()), 5);
            q.text = "pad";
            expect(names(lib.search(q)) == (std::vector<std::string>{"Glass Pad", "Warm Pad"}), "text search in names");
            q.text = "dark sub";
            expect(names(lib.search(q)) == std::vector<std::string>{"Deep Sub"}, "every word must match, across fields");
            q = {};
            q.tags = {"slow"};
            expectEquals(static_cast<int>(lib.search(q).size()), 2);
            q.tags = {"SLOW", "warm"};
            expect(names(lib.search(q)) == std::vector<std::string>{"Warm Pad"}, "tags are all-of, case-insensitive");
            q = {};
            q.categories = {"lead"};
            expectEquals(static_cast<int>(lib.search(q).size()), 2);
            q.source = PresetQuery::Source::User;
            expect(names(lib.search(q)) == std::vector<std::string>{"My Lead"}, "source filter");
            q = {};
            q.bank = "pads";
            expectEquals(static_cast<int>(lib.search(q).size()), 2);
            q = {};
            q.minRating = 4;
            expectEquals(static_cast<int>(lib.search(q).size()), 2);
            q = {};
            q.sort = PresetSort::Rating;
            q.descending = true;
            expect(lib.search(q).front().name == "Deep Sub", "sorted by rating");
            q.sort = PresetSort::Modified;
            expect(lib.search(q).front().name == "My Lead", "sorted by modified, newest first");
            q.sort = PresetSort::Name;
            q.descending = false;
            expect(lib.search(q).front().name == "Built In");
            const auto cats = lib.categories();
            expect(cats.size() == 3 && cats.front().second == 2 && cats.back().second == 1, "categories by use");
            const auto tags = lib.tags();
            expect(tags.size() >= 6 && tags.front().second == 2, "tags counted");
            bool user = false, factory = false;
            for (const auto& b : lib.banks())
            {
                user = user || (b.name == "User" && !b.factory && b.count == 1);
                factory = factory || (b.name == "Pads" && b.factory && b.count == 2);
            }
            expect(user && factory, "banks listed with their kind");
            expect(lib.find("f1") != nullptr && lib.find("f1")->factory && lib.find("f1")->path.size() > 0);
            expect(lib.find("builtin-" + contentHash("Core/Built In")) != nullptr, "built-in presets get an id");
        }

        beginTest("library: save, rename, move, edit, remove, favourites on factory presets, persistence");
        {
            TempDir dir;
            fs::create_directories(dir.path / "factory" / "Core");
            std::string error;
            auto f = makePatch("Factory One", "Pad", {"a"}, 2);
            f.meta.id = "fac1";
            expect(savePatchFile(dir.str("factory/Core/Factory One.crucible-patch"), f));
            PresetLibrary lib(dir.str("factory"), dir.str("user"));
            std::uint64_t counter = 0;
            lib.setIdGenerator([&] { return "u" + std::to_string(++counter); });
            std::string clockValue = "2026-06-01T00:00:00Z";
            lib.setClock([&] { return clockValue; });
            lib.scan();
            const std::string a = lib.save(makePatch("Sound/One", "Pad", {}, 0, 1111.0f), {}, error);
            expect(!a.empty() && lib.find(a)->name == "Sound/One", error);
            expect(fs::exists(lib.find(a)->path) && lib.find(a)->path.find("Sound_One") != std::string::npos,
                   "file name sanitised");
            // Saving a patch with the same name creates a second file, saving with the same id overwrites.
            const std::string b = lib.save(makePatch("Sound/One", "Pad", {}, 0, 2222.0f), {}, error);
            expect(!b.empty() && b != a && lib.entries().size() == 3, "same name, new preset");
            expect(lib.find(b)->path != lib.find(a)->path, "unique file names");
            Patch loaded;
            expect(lib.load(a, loaded, error), error);
            expectEquals(loaded.param("filter_cutoff", 0.0f), 1111.0f);
            expectEquals(loaded.meta.id, a);
            expectEquals(loaded.meta.created, std::string("2026-01-01T00:00:00Z"));
            clockValue = "2026-07-01T00:00:00Z";
            loaded.params[0].second = 3333.0f;
            expectEquals(lib.save(loaded, {}, error), a, "saving a loaded preset updates it");
            expectEquals(lib.entries().size(), static_cast<size_t>(3));
            Patch again;
            lib.load(a, again, error);
            expect(again.param("filter_cutoff", 0.0f) == 3333.0f && again.meta.modified == "2026-07-01T00:00:00Z" &&
                       again.meta.created == "2026-01-01T00:00:00Z",
                   "modified moves, created stays");

            const std::string oldPath = lib.find(a)->path;
            expect(lib.rename(a, "Renamed"), "rename");
            expect(lib.find(a)->name == "Renamed" && !fs::exists(oldPath) && fs::exists(lib.find(a)->path),
                   "file follows the name");
            expect(!lib.rename("fac1", "Nope") && !lib.remove("fac1") && !lib.moveToBank("fac1", "X"),
                   "factory presets are read-only");
            expect(!lib.updateMeta("fac1", [](PatchMeta& m) { m.name = "x"; }));
            expect(lib.updateMeta(a,
                                  [](PatchMeta& m)
                                  {
                                      m.tags = {"x", "y"};
                                      m.category = "Keys";
                                      m.author = "Ada";
                                      m.description = "d";
                                  }));
            expect(lib.find(a)->tags.size() == 2 && lib.find(a)->category == "Keys" && lib.find(a)->id == a,
                   "meta edit keeps identity");
            expect(lib.createBank("Live Set"));
            expect(lib.moveToBank(a, "Live Set") && lib.find(a)->bank == "Live Set", "move to bank");
            expect(fs::exists(dir.path / "user" / "Live Set"), "bank folder");
            expect(lib.setFavourite(a, true) && lib.setRating(a, 5) && lib.find(a)->favourite && lib.find(a)->rating == 5);
            expect(lib.setFavourite("fac1", true) && lib.setRating("fac1", 4), "factory favourite and rating");
            expect(!fs::exists(dir.path / "factory" / "Core" / "Factory One.crucible-patch.tmp"));
            expect(slurp(dir.str("factory/Core/Factory One.crucible-patch")).find("\"favourite\": true") ==
                       std::string::npos,
                   "factory file untouched");

            // A copy of a factory preset.
            Patch fp;
            expect(lib.load("fac1", fp, error) && fp.meta.favourite && fp.meta.rating == 4);
            const std::string copy = lib.saveAs(fp, {"Copies", false}, error);
            expect(!copy.empty() && copy != "fac1" && lib.find(copy)->bank == "Copies" && !lib.find(copy)->factory,
                   "saveAs copies");

            // Everything is found again by a new library on the same folders (cache cold and warm).
            for (int pass = 0; pass < 2; ++pass)
            {
                PresetLibrary re(dir.str("factory"), dir.str("user"));
                expectEquals(re.scan(), 4);
                const auto* e = re.find(a);
                expect(e != nullptr && e->name == "Renamed" && e->favourite && e->rating == 5 && e->bank == "Live Set" &&
                           e->tags.size() == 2 && e->author == "Ada",
                       "user preset restored");
                const auto* fe = re.find("fac1");
                expect(fe != nullptr && fe->favourite && fe->rating == 4, "factory overlay restored");
            }
            expect(fs::exists(dir.path / "user" / ".cache" / "index.json"), "index cache written");
            // A file changed behind the library's back is re-read.
            {
                Patch edited;
                loadPatchFile(lib.find(a)->path, edited, error);
                edited.meta.name = "Edited Outside";
                edited.meta.id = a;
                expect(savePatchFile(lib.find(a)->path, edited));
                PresetLibrary re(dir.str("factory"), dir.str("user"));
                re.scan();
                expect(re.find(a) != nullptr && re.find(a)->name == "Edited Outside", "cache invalidated by file change");
            }

            expect(lib.remove(a) && lib.find(a) == nullptr, "remove");
            expect(fs::exists(dir.path / "user" / ".trash"), "moved to trash");
            expectEquals(lib.purgeTrash(), 1);
            expect(!lib.load(a, fp, error) && !error.empty(), "gone");
            // Imports.
            expect(savePatchFile(dir.str("outside.crucible-patch"), makePatch("Imported", "FX", {"noise"})));
            const std::string imp = lib.importFile(dir.str("outside.crucible-patch"), {}, error);
            expect(!imp.empty() && lib.find(imp)->name == "Imported", error);
            expect(lib.importFile(dir.str("missing.crucible-patch"), {}, error).empty());
            // A copied file does not collide with the original id.
            fs::copy_file(lib.find(imp)->path, dir.path / "user" / "User" / "Imported copy.crucible-patch");
            PresetLibrary dup(dir.str("factory"), dir.str("user"));
            const int count = dup.scan();
            std::set<std::string> ids;
            for (const auto& e : dup.entries())
                ids.insert(e.id);
            expectEquals(static_cast<int>(ids.size()), count, "ids are unique after copying a file");
        }

        beginTest("assets: embedded or referenced, shared between presets, missing content reported");
        {
            TempDir dir;
            std::string error;
            Engine e;
            e.prepare(kRate, 128);
            std::vector<float> frames(64 * 2048);
            for (size_t i = 0; i < frames.size(); ++i)
                frames[i] = std::sin(static_cast<float>(i) * 0.01f);
            UserWavetableBank::Record rec;
            rec.slot = 2;
            rec.name = "Sine Stack";
            rec.frames = frames;
            e.userWavetables().deserialise({rec});
            std::vector<float> ir(2400, 0.0f);
            ir[5] = 1.0f;
            ImpulseResponse irData;
            irData.sampleRate = kRate;
            irData.layout = IrLayout::Stereo;
            irData.channels = {ir, ir};
            e.coreEngine().engine().effects().findCapability<ConvolutionIr>()->setImpulseResponse(irData, "taps");
            PatchMeta meta;
            meta.name = "With Assets";
            Patch p = capturePatch(e, meta);
            expect(p.userWaves.size() == 1 && p.convolutionIrs.size() == 1);
            p.assets.push_back({"cloud", "Cloud A", 1, 0.0, 1, {}, CloudModel::harmonicMorph(8).serialise()});
            p.assets.push_back({"sample", "Hit", 0, 44100.0, 2, {0.1f, -0.1f, 0.2f, -0.2f}, {}});

            PresetLibrary lib("", dir.str("user"));
            lib.setIdGenerator([] { return std::string("idX"); });
            const auto embedded = lib.save(p, {}, error);
            expect(!embedded.empty(), error);
            const auto embeddedSize = fs::file_size(lib.find(embedded)->path);
            Patch q = p;
            q.meta.name = "Referenced";
            std::uint64_t n = 0;
            lib.setIdGenerator([&] { return "ref" + std::to_string(++n); });
            PresetSaveOptions ro;
            ro.referenceAssets = true;
            const auto ref1 = lib.save(q, ro, error);
            q.meta.name = "Referenced Two";
            const auto ref2 = lib.save(q, ro, error);
            expect(!ref1.empty() && !ref2.empty(), error);
            const auto refSize = fs::file_size(lib.find(ref1)->path);
            const std::string text = slurp(lib.find(ref1)->path);
            expect(text.find("assetRefs") != std::string::npos && text.find("\"frames\"") == std::string::npos,
                   "references only");
            int files = 0;
            std::uint64_t assetBytes = 0;
            for (const auto& f : fs::directory_iterator(dir.path / "user" / "assets"))
                if (f.path().extension() == ".bin")
                {
                    ++files;
                    assetBytes += fs::file_size(f.path());
                }
            // The embedded preset carries every asset (encoded, so larger than the raw bytes); the referenced one carries
            // none, so the difference is at least the raw asset size whatever the size of the parameter table.
            expect(embeddedSize > refSize && embeddedSize - refSize >= assetBytes,
                   "referenced file drops the asset payload: " + String(static_cast<double>(refSize)) + " vs " +
                       String(static_cast<double>(embeddedSize)) + ", assets " + String(static_cast<double>(assetBytes)));
            expectEquals(files, 4,
                         "wavetable, impulse response (identical channels share a file), cloud, sample: content is shared");

            for (const auto& id : {embedded, ref1, ref2})
            {
                Patch loaded;
                std::vector<std::string> missing;
                expect(lib.load(id, loaded, error, &missing), error);
                expect(missing.empty() && loaded.assetRefs.empty(), "all references resolved");
                expect(loaded.userWaves.size() == 1 && loaded.userWaves[0].frames == p.userWaves[0].frames &&
                           loaded.userWaves[0].slot == 2,
                       "wavetable restored exactly");
                expect(loaded.convolutionIrs.size() == 1 && loaded.convolutionIrs[0].channels[0] == ir &&
                           loaded.convolutionIrs[0].name == "taps" && loaded.convolutionIrs[0].rate == kRate,
                       "impulse response restored");
                bool cloud = false, sample = false;
                for (const auto& a : loaded.assets)
                {
                    cloud = cloud || (a.kind == "cloud" && a.text == p.assets[0].text);
                    sample = sample ||
                             (a.kind == "sample" && a.frames == p.assets[1].frames && a.channels == 2 && a.rate == 44100.0);
                }
                expect(cloud && sample, "cloud and sample restored");
            }
            // Applying a referenced preset gives the same sound as the original engine.
            Patch loaded;
            lib.load(ref1, loaded, error);
            Engine other;
            other.prepare(kRate, 128);
            applyPatch(other, loaded);
            expect(other.userWavetables().isLoaded(2), "wavetable applied from the store");
            expect(other.coreEngine().engine().effects().findCapability<ConvolutionIr>() != nullptr);

            // Missing content is reported and the reference is kept.
            for (const auto& f : fs::directory_iterator(dir.path / "user" / "assets"))
                fs::remove(f.path());
            std::vector<std::string> missing;
            Patch broken;
            expect(lib.load(ref1, broken, error, &missing), "still loads");
            expectEquals(static_cast<int>(missing.size()), 5);
            expectEquals(static_cast<int>(broken.assetRefs.size()), 5);
            AssetStore store(dir.str("user/assets"));
            expect(!store.has("deadbeef") && store.put("abc", 3).size() == 32 && store.has(store.put("abc", 3)));
            std::vector<std::uint8_t> bytes;
            expect(!store.get("../../etc/passwd", bytes) && store.get(store.put("abc", 3), bytes) && bytes.size() == 3,
                   "store api");

            // Several convolution instances: every channel of every instance goes through the store and comes back keyed.
            Patch multi;
            multi.meta.name = "Two Convolutions";
            multi.convolutionIrs.push_back(
                {"fx.convolution", {{0.5f, 0.25f, 0.125f}, {0.1f, 0.2f, 0.3f}}, "stereo", 44100.0, "room.wav"});
            multi.convolutionIrs.push_back({"fx.convolution.2", {{1.0f, -0.5f}}, "mono", 48000.0, "plate.wav"});
            PresetSaveOptions byReference;
            byReference.referenceAssets = true;
            const auto multiId = lib.save(multi, byReference, error);
            expect(!multiId.empty(), error);
            const std::string multiText = slurp(lib.find(multiId)->path);
            expect(multiText.find("\"convolutionIrs\"") == std::string::npos &&
                       multiText.find("\"instance\"") != std::string::npos,
                   "responses are references, not embedded");
            Patch multiLoaded;
            std::vector<std::string> multiMissing;
            expect(lib.load(multiId, multiLoaded, error, &multiMissing) && multiMissing.empty(), error);
            expect(multiLoaded.convolutionIrs.size() == 2, "both instances restored");
            if (multiLoaded.convolutionIrs.size() == 2)
            {
                const auto& first = multiLoaded.convolutionIrs[0];
                const auto& second = multiLoaded.convolutionIrs[1];
                expect(first.instance == "fx.convolution" && first.channels == multi.convolutionIrs[0].channels &&
                           first.layout == "stereo" && first.name == "room.wav" && first.rate == 44100.0,
                       "first instance restored exactly");
                expect(second.instance == "fx.convolution.2" && second.channels == multi.convolutionIrs[1].channels &&
                           second.layout == "mono" && second.name == "plate.wav" && second.rate == 48000.0,
                       "second instance restored exactly");
            }
        }
    }
};

static PresetTests presetTests;
