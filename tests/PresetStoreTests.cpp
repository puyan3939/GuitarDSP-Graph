// Regression tests for issue #79: PresetStore's directory-backed save/list/
// load/delete cycle. Runs against a throwaway temp directory per test so it
// never touches a real user preset directory; every operation here is
// blocking filesystem I/O and, per PresetStore.h's real-time contract, is
// only ever exercised from this control-thread-only test binary.
#include "guitardsp/app/PresetStore.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

using namespace guitardsp::app;
namespace fs = std::filesystem;

namespace {
bool require(bool condition, const char* message) {
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    return condition;
}

// A fresh, empty directory under the system temp path, removed on
// destruction so repeated test runs don't accumulate stale preset files.
class ScopedTempDir {
public:
    ScopedTempDir() {
        path_ = fs::temp_directory_path()
            / ("guitardsp_preset_store_tests_" + std::to_string(counter()));
    }
    ~ScopedTempDir() {
        std::error_code ec;
        fs::remove_all(path_, ec);
    }
    ScopedTempDir(const ScopedTempDir&) = delete;
    ScopedTempDir& operator=(const ScopedTempDir&) = delete;

    [[nodiscard]] std::string string() const { return path_.string(); }

private:
    static std::size_t counter() {
        static std::size_t next = 0;
        return next++;
    }
    fs::path path_;
};

LiveRigPreset makePreset(const std::string& name, float pedalDrive) {
    LiveRigPreset preset;
    preset.name = name;
    preset.settings.pedalDrive = pedalDrive;
    return preset;
}
} // namespace

int main() {
    bool ok = true;

    {
        const ScopedTempDir dir;
        const PresetStore store(dir.string());
        ok &= require(store.list().empty(),
                      "listing a not-yet-created preset directory returns an empty list, not an error");
    }

    {
        const ScopedTempDir dir;
        const PresetStore store(dir.string());

        std::string error;
        ok &= require(store.save(makePreset("Lead Crunch", 0.7f), &error),
                      "saving a preset into a fresh directory succeeds");
        ok &= require(error.empty(), "successful save clears the error string");
        ok &= require(fs::is_directory(dir.string()),
                      "save() creates the preset directory on demand");

        ok &= require(store.save(makePreset("Clean Verse", 0.2f)),
                      "saving a second preset succeeds");

        const auto summaries = store.list();
        ok &= require(summaries.size() == 2, "list() reports both saved presets");
        ok &= require(std::is_sorted(summaries.begin(), summaries.end(),
                                     [](const PresetSummary& a, const PresetSummary& b) {
                                         return a.name < b.name;
                                     }),
                      "list() is sorted by display name");

        LiveRigPreset loaded;
        ok &= require(store.load("Lead Crunch", loaded, &error), "loading a saved preset by name succeeds");
        ok &= require(loaded.name == "Lead Crunch" && std::abs(loaded.settings.pedalDrive - 0.7f) < 1.0e-5f,
                      "loaded preset matches what was saved");

        ok &= require(store.save(makePreset("Lead Crunch", 0.95f)),
                      "re-saving under the same name succeeds (overwrite)");
        ok &= require(store.load("Lead Crunch", loaded),
                      "loading after overwrite succeeds");
        ok &= require(std::abs(loaded.settings.pedalDrive - 0.95f) < 1.0e-5f,
                      "overwrite actually replaced the previous preset content");
        ok &= require(store.list().size() == 2,
                      "overwriting an existing preset name does not create a duplicate entry");

        ok &= require(store.remove("Clean Verse", &error), "deleting an existing preset succeeds");
        ok &= require(store.list().size() == 1, "list() reflects the deletion");

        ok &= require(!store.remove("Clean Verse", &error),
                      "deleting an already-removed preset fails rather than silently succeeding");
        ok &= require(!error.empty(), "failed delete reports a non-empty error");

        LiveRigPreset missing;
        ok &= require(!store.load("Does Not Exist", missing, &error),
                      "loading a preset that was never saved fails");
        ok &= require(!error.empty(), "failed load reports a non-empty error");
    }

    {
        // Names containing path separators/traversal segments must not be
        // able to save or load outside the preset directory.
        const ScopedTempDir dir;
        const PresetStore store(dir.string());
        std::string error;
        ok &= require(store.save(makePreset("../../etc/passwd", 0.5f), &error),
                      "a name containing path traversal segments still saves (sanitized), not rejected outright");

        std::size_t regularFiles = 0;
        for (const auto& entry : fs::directory_iterator(dir.string()))
            if (entry.is_regular_file()) ++regularFiles;
        ok &= require(regularFiles == 1,
                      "the sanitized file lands inside the preset directory itself, not a parent");

        const fs::path outside = fs::path(dir.string()).parent_path() / "passwd";
        ok &= require(!fs::exists(outside), "no file was created outside the preset directory");
    }

    return ok ? 0 : 1;
}
