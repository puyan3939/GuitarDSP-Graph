#include "guitardsp/app/PresetStore.h"

#include "guitardsp/app/LiveRigPresetJson.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <system_error>

namespace guitardsp::app {

namespace {

namespace fs = std::filesystem;

// Maps an arbitrary display name to a safe filename component: only
// alphanumerics/space/-/_ survive, everything else (including '/', '\\' and
// '.' -- which would otherwise allow ".." traversal) collapses to '_'.
// Leading/trailing whitespace is trimmed and the result is capped to a sane
// length; an all-unsafe or empty name falls back to "preset" rather than
// producing an empty filename.
std::string sanitizeFileStem(const std::string& name) {
    std::string result;
    result.reserve(name.size());
    for (const char c : name) {
        const unsigned char uc = static_cast<unsigned char>(c);
        if (std::isalnum(uc) || c == ' ' || c == '-' || c == '_')
            result.push_back(c);
        else
            result.push_back('_');
    }
    const auto first = result.find_first_not_of(' ');
    const auto last = result.find_last_not_of(' ');
    result = first == std::string::npos ? std::string() : result.substr(first, last - first + 1);
    constexpr std::size_t maxLength = 128;
    if (result.size() > maxLength) result.resize(maxLength);
    return result.empty() ? "preset" : result;
}

} // namespace

PresetStore::PresetStore(std::string directory) : directory_(std::move(directory)) {}

std::string PresetStore::filePathForName(const std::string& name) const {
    const fs::path path = fs::path(directory_) / (sanitizeFileStem(name) + ".json");
    return path.string();
}

std::vector<PresetSummary> PresetStore::list() const {
    std::vector<PresetSummary> summaries;
    std::error_code ec;
    if (!fs::exists(directory_, ec) || !fs::is_directory(directory_, ec)) return summaries;

    for (const auto& entry : fs::directory_iterator(directory_, ec)) {
        if (ec) break;
        if (!entry.is_regular_file()) continue;
        const fs::path& path = entry.path();
        if (path.extension() != ".json") continue;

        LiveRigPreset preset;
        if (!loadPresetFromFile(path.string(), preset)) continue;
        const std::string displayName = preset.name.empty() ? path.stem().string() : preset.name;
        summaries.push_back(PresetSummary{displayName, path.filename().string()});
    }

    std::sort(summaries.begin(), summaries.end(),
              [](const PresetSummary& a, const PresetSummary& b) { return a.name < b.name; });
    return summaries;
}

bool PresetStore::save(const LiveRigPreset& preset, std::string* error) const {
    if (preset.name.empty()) {
        if (error != nullptr) *error = "preset name must not be empty";
        return false;
    }
    std::error_code ec;
    fs::create_directories(directory_, ec);
    if (ec && !fs::is_directory(directory_)) {
        if (error != nullptr) *error = "unable to create preset directory: " + ec.message();
        return false;
    }
    return savePresetToFile(preset, filePathForName(preset.name), error);
}

bool PresetStore::load(const std::string& name, LiveRigPreset& outPreset, std::string* error) const {
    return loadPresetFromFile(filePathForName(name), outPreset, error);
}

bool PresetStore::remove(const std::string& name, std::string* error) const {
    std::error_code ec;
    const bool removed = fs::remove(filePathForName(name), ec);
    if (ec) {
        if (error != nullptr) *error = "unable to delete preset: " + ec.message();
        return false;
    }
    if (!removed) {
        if (error != nullptr) *error = "preset not found: " + name;
        return false;
    }
    if (error != nullptr) error->clear();
    return true;
}

} // namespace guitardsp::app
