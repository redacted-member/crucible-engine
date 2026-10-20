// SPDX-License-Identifier: MIT
#pragma once

// Preset library: a searchable collection of patches on disk (plus built-in ones in memory), independent of any UI.
//
// Layout. A library has an optional read-only factory root and a writable user root. Each root holds banks, one
// folder per bank, and a bank holds `*.crucible-patch` files (the JSON patch format, Patch.h). The user root also keeps
//   assets/     content-addressed files for patches saved with referenced assets (wavetables, impulse responses,
//               clouds, samples), shared between presets,
//   overlay.json  favourite / rating of factory presets (factory files never change),
//   .cache/     an index cache keyed by file size and modification time, so scanning a large library does not
//               parse every patch again,
//   .trash/     removed presets (remove() moves them here; purgeTrash() deletes them).
//
// Identity. Every preset has a stable id (PatchMeta::id) that survives rename, move and metadata edits. Patches
// without one (older formats, hand-made files) get an id derived from their content when scanned.
//
// Threads: control thread only; none of this runs in the audio path.

#include <crucible/Patch.h>

#include <functional>
#include <map>
#include <string>
#include <vector>

namespace crucible
{

struct PresetEntry
{
    std::string id;
    std::string name;
    std::string author;
    std::string product;
    std::string category;
    std::string description;
    std::vector<std::string> tags;
    int rating = 0;
    bool favourite = false;
    std::string created;
    std::string modified;
    std::string bank;
    bool factory = false; // read-only (factory root or built-in)
    bool builtin = false; // held in memory, no file
    std::string path;     // empty for built-in presets
};

enum class PresetSort
{
    Name,
    Category,
    Rating,
    Modified,
    Created
};

struct PresetQuery
{
    std::string text;                    // every word must occur in name, author, category, tags, description or bank
    std::vector<std::string> categories; // any of (case-insensitive)
    std::vector<std::string> tags;       // all of (case-insensitive)
    std::string bank;                    // empty = all banks
    enum class Source
    {
        Any,
        Factory,
        User
    } source = Source::Any;
    bool favouritesOnly = false;
    int minRating = 0;
    PresetSort sort = PresetSort::Name;
    bool descending = false; // Rating, Modified and Created are most useful descending
};

struct PresetSaveOptions
{
    std::string bank = "User";
    // Store wavetables, impulse responses and extra assets in the asset store and keep only references in the file.
    bool referenceAssets = false;
};

struct PresetBank
{
    std::string name;
    bool factory = false;
    int count = 0;
};

// Content-addressed file store: put() returns the content hash, get() returns the bytes.
class AssetStore
{
public:
    explicit AssetStore(std::string directory = {}) : dir(std::move(directory)) {}
    bool valid() const { return !dir.empty(); }
    std::string put(const void* data, size_t size) const;
    bool get(const std::string& hash, std::vector<std::uint8_t>& out) const;
    bool has(const std::string& hash) const;
    const std::string& directory() const { return dir; }

private:
    std::string dir;
};

// Moves the embedded wavetables, impulse response and assets of a patch into the store (replaced by references).
bool externaliseAssets(Patch& patch, const AssetStore& store);
// Brings referenced content back into the patch. Content that is missing from every store is listed in `missing`
// ("kind name") and its reference is kept.
void resolveAssets(Patch& patch, const std::vector<const AssetStore*>& stores, std::vector<std::string>& missing);

class PresetLibrary
{
public:
    // Either root may be empty. The user root is created on demand.
    PresetLibrary(std::string factoryRoot, std::string userRoot);

    // Extension of the files this library writes (default ".crucible-patch"). Files with the default or the earlier
    // project extension are always read, whatever is written.
    void setFileExtension(std::string extension) { fileExt = std::move(extension); }

    // Injectable for tests: time stamps (ISO 8601 UTC) and id generation.
    void setClock(std::function<std::string()> clock) { now = std::move(clock); }
    void setIdGenerator(std::function<std::string()> generator) { newId = std::move(generator); }

    // Reads both roots (using the cache) and the built-in presets. Returns the number of presets.
    int scan();
    // Adds an in-memory factory preset to a bank (before or after scan()); survives rescans.
    void addBuiltin(const std::string& bank, Patch patch);

    const std::vector<PresetEntry>& entries() const { return list; }
    const PresetEntry* find(const std::string& id) const;
    std::vector<PresetEntry> search(const PresetQuery& query) const;
    std::vector<PresetBank> banks() const;
    // Distinct categories / tags with their preset counts, most used first.
    std::vector<std::pair<std::string, int>> categories() const;
    std::vector<std::pair<std::string, int>> tags() const;

    // Loads a preset with referenced assets resolved. `missing` receives unresolved content.
    bool load(const std::string& id, Patch& out, std::string& error, std::vector<std::string>* missing = nullptr) const;
    // Saves into the user root and returns the preset id ("" on failure). A patch whose id belongs to an existing
    // user preset replaces it (and follows a changed name); otherwise it becomes a new preset.
    std::string save(Patch patch, const PresetSaveOptions& options, std::string& error);
    // Always creates a new preset (new id), e.g. "Save as" or copying a factory preset.
    std::string saveAs(Patch patch, const PresetSaveOptions& options, std::string& error);
    // Imports a patch file as a new user preset.
    std::string importFile(const std::string& path, const PresetSaveOptions& options, std::string& error);

    // User presets only (factory and built-in presets are read-only; copy them with saveAs()).
    bool rename(const std::string& id, const std::string& newName);
    bool remove(const std::string& id);
    bool moveToBank(const std::string& id, const std::string& bank);
    bool updateMeta(const std::string& id, const std::function<void(PatchMeta&)>& change);
    bool createBank(const std::string& bank);
    int purgeTrash();
    // Favourite and rating work for every preset (factory ones keep them in overlay.json).
    bool setFavourite(const std::string& id, bool favourite);
    bool setRating(const std::string& id, int rating);

    const AssetStore& userAssets() const { return assets; }

private:
    struct Builtin
    {
        std::string bank;
        Patch patch;
    };
    PresetEntry makeEntry(const Patch& patch, const std::string& bank, bool factory, const std::string& path) const;
    bool readEntry(const std::string& path, const std::string& bank, bool factory, PresetEntry& out) const;
    bool writePatch(const std::string& path, const Patch& patch, std::string& error) const;
    std::string uniquePath(const std::string& bank, const std::string& name, const std::string& ignore) const;
    std::string saveImpl(Patch patch, const PresetSaveOptions& options, bool asNew, std::string& error);
    void loadOverlay();
    void saveOverlay() const;
    PresetEntry* findMutable(const std::string& id);

    std::string factoryRoot, userRoot;
    AssetStore assets, factoryAssets;
    std::function<std::string()> now, newId;
    std::string fileExt = ".crucible-patch";
    std::vector<PresetEntry> list;
    std::vector<Builtin> builtins;
    struct Overlay
    {
        bool favourite = false;
        int rating = 0;
    };
    std::map<std::string, Overlay> overlay;
};

} // namespace crucible
