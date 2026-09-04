// Generates the committed golden reference files under tests/golden/ (see
// docs/GOLDEN_REFERENCE.md). This tool is control-thread/offline-only: it is
// not part of GuitarDSPGraphCore, is not built by default
// (GUITARDSP_BUILD_GOLDEN_TOOLS=OFF unless requested), and must only be run
// against the GOLDEN CMake preset so the files it writes are the "fixed
// point" the rest of the policy assumes.
//
// Usage:
//   golden_gen [--commit <hash>] [--preset <name>] [--params-dir <dir>]
//              [--output-dir <dir>] [--wav-dir <dir>] [--circuit <name>]
//
// With no --circuit filter, all six circuits are (re)generated.

#include "guitardsp/circuit/CompressorCircuit.h"
#include "guitardsp/circuit/DS1Circuit.h"
#include "guitardsp/circuit/FullAmpCircuit.h"
#include "guitardsp/circuit/JsonValue.h"
#include "guitardsp/circuit/PowerAmpCircuit.h"
#include "guitardsp/circuit/PreampCircuit.h"
#include "guitardsp/circuit/TS808Circuit.h"

#include "../../tests/golden/GoldenSignals.h"
#include "../../tests/golden/HexFloat.h"
#include "WavWriter.h"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace guitardsp;
using guitardsp::golden::generateGoldenSignal;
using guitardsp::golden::goldenSignalList;
using guitardsp::golden::toHexLine;

namespace {

#ifndef GUITARDSP_GOLDEN_PARAMS_DIR
#define GUITARDSP_GOLDEN_PARAMS_DIR "tests/golden/params"
#endif
#ifndef GUITARDSP_GOLDEN_OUTPUT_DIR
#define GUITARDSP_GOLDEN_OUTPUT_DIR "tests/golden"
#endif

std::string compilerIdentity() {
#if defined(__clang__)
    return "clang " __clang_version__;
#elif defined(__GNUC__)
    return "gcc " + std::to_string(__GNUC__) + "." + std::to_string(__GNUC_MINOR__) + "." +
           std::to_string(__GNUC_PATCHLEVEL__);
#elif defined(_MSC_VER)
    return "msvc " + std::to_string(_MSC_VER);
#else
    return "unknown-compiler";
#endif
}

// FNV-1a 64-bit. Only used to fingerprint the params JSON files in
// MANIFEST.json (detect an accidental edit), not for anything
// security-sensitive.
std::string fnv1a64Hex(const std::string& data) {
    std::uint64_t hash = 1469598103934665603ull;
    for (unsigned char c : data) {
        hash ^= c;
        hash *= 1099511628211ull;
    }
    std::ostringstream oss;
    oss.width(16);
    oss.fill('0');
    oss << std::hex << hash;
    return oss.str();
}

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::ostringstream oss;
    oss << in.rdbuf();
    return oss.str();
}

void writeHexFile(const std::string& path, const std::vector<float>& samples) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path);
    for (float v : samples) out << toHexLine(v) << '\n';
}

double rms(const std::vector<float>& samples) {
    if (samples.empty()) return 0.0;
    double sumSquares = 0.0;
    for (float v : samples) sumSquares += static_cast<double>(v) * static_cast<double>(v);
    return std::sqrt(sumSquares / static_cast<double>(samples.size()));
}

struct GeneratedFile {
    std::string relativePath;
    std::size_t sampleCount = 0;
    std::size_t byteSize = 0;
    double rmsValue = 0.0;
    bool knownBad = false;
};

// TS808's "full" variant (drive = tone = level = 1.0) hits a genuine
// Newton-solver divergence at that extreme parameter corner (see the
// "Known caveat" section of docs/GOLDEN_REFERENCE.md and the issue #88
// implementation report) -- captured as-is rather than fixed here, but
// excluded from golden_reference's pass/fail judgment via this flag so a
// known, already-reported issue doesn't mask a newly introduced one.
bool isKnownBad(const std::string& circuitName, const std::string& variantName) {
    return circuitName == "ts808" && variantName == "full";
}

struct Args {
    std::string commit = "unknown";
    std::string preset = "golden";
    std::string paramsDir = GUITARDSP_GOLDEN_PARAMS_DIR;
    std::string outputDir = GUITARDSP_GOLDEN_OUTPUT_DIR;
    std::optional<std::string> wavDir;
    std::optional<std::string> circuitFilter;
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string flag = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + flag);
            return argv[++i];
        };
        if (flag == "--commit") args.commit = next();
        else if (flag == "--preset") args.preset = next();
        else if (flag == "--params-dir") args.paramsDir = next();
        else if (flag == "--output-dir") args.outputDir = next();
        else if (flag == "--wav-dir") args.wavDir = next();
        else if (flag == "--circuit") args.circuitFilter = next();
        else throw std::runtime_error("unknown argument " + flag);
    }
    return args;
}

// Runs `input` through a freshly prepared+control-set circuit instance.
// A fresh instance per (circuit, variant, signal) file keeps each golden
// file independent of file-generation order.
template <typename Circuit, typename SetControlsFn>
std::vector<float> renderCircuit(SetControlsFn setControls, const std::vector<float>& input) {
    Circuit circuit;
    if (!circuit.prepare(golden::kSampleRateHz)) {
        throw std::runtime_error("circuit prepare() failed");
    }
    setControls(circuit);
    std::vector<float> output(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) output[i] = circuit.processSample(input[i]);
    return output;
}

struct CircuitSpec {
    std::string name;
    // Renders one variant's worth of output for the given signal name.
    std::function<std::vector<float>(const circuit::JsonValue& variantParams, const std::vector<float>& input)>
        render;
};

std::vector<GeneratedFile> generateCircuit(const CircuitSpec& spec, const Args& args,
                                            std::map<std::string, std::string>* paramsFileHashes) {
    std::vector<GeneratedFile> files;
    const std::string paramsPath = args.paramsDir + "/" + spec.name + ".json";
    const std::string paramsText = readFile(paramsPath);
    (*paramsFileHashes)[spec.name] = fnv1a64Hex(paramsText);
    const circuit::JsonValue params = circuit::parseJson(paramsText);
    const circuit::JsonValue& variants = params["variants"];
    if (!variants.isObject()) throw std::runtime_error(spec.name + ".json has no \"variants\" object");

    for (const auto& [variantName, variantParams] : variants.entries()) {
        for (const auto& signal : goldenSignalList()) {
            const std::vector<float> input = generateGoldenSignal(signal.name);
            const std::vector<float> output = spec.render(variantParams, input);
            const std::string fileName =
                spec.name + "_" + signal.name + "_" + variantName + ".txt";
            const std::string outPath = args.outputDir + "/" + fileName;
            writeHexFile(outPath, output);

            GeneratedFile file;
            file.relativePath = "tests/golden/" + fileName;
            file.sampleCount = output.size();
            file.byteSize = output.size() * 9; // 8 hex chars + '\n'
            file.rmsValue = rms(output);
            file.knownBad = isKnownBad(spec.name, variantName);
            files.push_back(file);

            if (args.wavDir) {
                const std::string wavPath =
                    *args.wavDir + "/" + spec.name + "_" + signal.name + "_" + variantName + ".wav";
                golden::writeMonoFloatWav(wavPath, output, golden::kSampleRateHz);
            }

            std::cout << "wrote " << outPath << " (" << output.size() << " samples, rms="
                      << file.rmsValue << ")\n";
        }
    }
    return files;
}

} // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "argument error: " << e.what() << '\n';
        return 2;
    }

    std::vector<CircuitSpec> specs;
    specs.push_back({"ts808", [](const circuit::JsonValue& p, const std::vector<float>& input) {
                          return renderCircuit<circuit::TS808Circuit>(
                              [&](circuit::TS808Circuit& c) {
                                  c.setControls(p["drive"].asFloat(), p["tone"].asFloat(), p["level"].asFloat());
                              },
                              input);
                      }});
    specs.push_back({"ds1", [](const circuit::JsonValue& p, const std::vector<float>& input) {
                          return renderCircuit<circuit::DS1Circuit>(
                              [&](circuit::DS1Circuit& c) {
                                  c.setControls(p["distortion"].asFloat(), p["tone"].asFloat(),
                                                p["level"].asFloat());
                              },
                              input);
                      }});
    specs.push_back({"preamp", [](const circuit::JsonValue& p, const std::vector<float>& input) {
                          return renderCircuit<circuit::PreampCircuit>(
                              [&](circuit::PreampCircuit& c) {
                                  c.setControls(p["bass"].asFloat(), p["treble"].asFloat());
                              },
                              input);
                      }});
    specs.push_back({"fullamp", [](const circuit::JsonValue& p, const std::vector<float>& input) {
                          return renderCircuit<circuit::FullAmpCircuit>(
                              [&](circuit::FullAmpCircuit& c) {
                                  c.setControls(p["bass"].asFloat(), p["treble"].asFloat());
                              },
                              input);
                      }});
    specs.push_back({"poweramp", [](const circuit::JsonValue&, const std::vector<float>& input) {
                          return renderCircuit<circuit::PowerAmpCircuit>([](circuit::PowerAmpCircuit&) {}, input);
                      }});
    specs.push_back({"compressor", [](const circuit::JsonValue&, const std::vector<float>& input) {
                          return renderCircuit<circuit::CompressorCircuit>([](circuit::CompressorCircuit&) {},
                                                                            input);
                      }});

    std::map<std::string, std::string> paramsFileHashes;
    std::vector<GeneratedFile> allFiles;
    std::size_t totalBytes = 0;
    try {
        for (const CircuitSpec& spec : specs) {
            if (args.circuitFilter && *args.circuitFilter != spec.name) continue;
            for (GeneratedFile& f : generateCircuit(spec, args, &paramsFileHashes)) {
                totalBytes += f.byteSize;
                allFiles.push_back(std::move(f));
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "generation failed: " << e.what() << '\n';
        return 1;
    }

    if (!args.circuitFilter) {
        // Only write the manifest on a full (unfiltered) run -- a filtered
        // run is for local iteration, not for producing the file the policy
        // in docs/GOLDEN_REFERENCE.md treats as authoritative.
        std::ofstream manifest(args.outputDir + "/MANIFEST.json", std::ios::binary);
        manifest << "{\n";
        manifest << "    \"commit\": \"" << args.commit << "\",\n";
        manifest << "    \"compiler\": \"" << compilerIdentity() << "\",\n";
        manifest << "    \"cmakePreset\": \"" << args.preset << "\",\n";
        manifest << "    \"sampleRateHz\": " << static_cast<long long>(golden::kSampleRateHz) << ",\n";
        manifest << "    \"paramsFileHashes\": {\n";
        std::size_t i = 0;
        for (const auto& [name, hash] : paramsFileHashes) {
            manifest << "        \"" << name << "\": \"" << hash << "\"";
            manifest << (++i == paramsFileHashes.size() ? "\n" : ",\n");
        }
        manifest << "    },\n";
        manifest << "    \"knownBad\": [\n";
        std::vector<std::string> knownBadPaths;
        for (const GeneratedFile& f : allFiles)
            if (f.knownBad) knownBadPaths.push_back(f.relativePath);
        for (std::size_t k = 0; k < knownBadPaths.size(); ++k) {
            manifest << "        \"" << knownBadPaths[k] << "\"";
            manifest << (k + 1 == knownBadPaths.size() ? "\n" : ",\n");
        }
        manifest << "    ],\n";
        manifest << "    \"fileCount\": " << allFiles.size() << ",\n";
        manifest << "    \"totalBytes\": " << totalBytes << "\n";
        manifest << "}\n";
        std::cout << "wrote " << args.outputDir << "/MANIFEST.json\n";
    }

    std::cout << "total golden files: " << allFiles.size() << ", total bytes: " << totalBytes << '\n';
    return 0;
}
