// Compares a golden reference file against a candidate output file (see
// docs/GOLDEN_REFERENCE.md section 6). Both files are the golden text
// format: one "%08x" float32-bit-pattern hex line per sample.
//
// Usage:
//   parity_check <golden.txt> <candidate.txt> [--tolerance 1e-6]
//
// Prints max absolute error, RMS error, the index of the first sample that
// exceeds the tolerance (the most important number for tracking down
// exactly where a regression starts), and whether the two files are bit-for-
// bit identical. Exits 0 if the candidate is within tolerance of the
// golden file (and same length), 1 otherwise.

#include "../../tests/golden/HexFloat.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

using guitardsp::golden::parseHexLine;

namespace {

std::vector<float> readHexFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open " + path);
    std::vector<float> values;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(in, line)) {
        ++lineNumber;
        if (line.empty()) continue;
        float value = 0.0f;
        if (!parseHexLine(line, &value)) {
            throw std::runtime_error(path + ":" + std::to_string(lineNumber) +
                                      " is not a valid 8-hex-digit golden line: \"" + line + "\"");
        }
        values.push_back(value);
    }
    return values;
}

struct Options {
    std::string goldenPath;
    std::string candidatePath;
    double tolerance = 1.0e-6;
};

Options parseArgs(int argc, char** argv) {
    Options opts;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--tolerance") {
            if (i + 1 >= argc) throw std::runtime_error("--tolerance requires a value");
            opts.tolerance = std::stod(argv[++i]);
        } else {
            positional.push_back(arg);
        }
    }
    if (positional.size() != 2) {
        throw std::runtime_error("usage: parity_check <golden.txt> <candidate.txt> [--tolerance 1e-6]");
    }
    opts.goldenPath = positional[0];
    opts.candidatePath = positional[1];
    return opts;
}

} // namespace

int main(int argc, char** argv) {
    Options opts;
    try {
        opts = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "argument error: " << e.what() << '\n';
        return 2;
    }

    std::vector<float> golden;
    std::vector<float> candidate;
    try {
        golden = readHexFile(opts.goldenPath);
        candidate = readHexFile(opts.candidatePath);
    } catch (const std::exception& e) {
        std::cerr << "read error: " << e.what() << '\n';
        return 2;
    }

    if (golden.size() != candidate.size()) {
        std::cout << "length mismatch: golden=" << golden.size() << " candidate=" << candidate.size() << '\n';
        return 1;
    }

    double maxAbsError = 0.0;
    double sumSquaredError = 0.0;
    std::optional<std::size_t> firstExceedingIndex;
    bool bitExact = true;

    for (std::size_t i = 0; i < golden.size(); ++i) {
        const double g = static_cast<double>(golden[i]);
        const double c = static_cast<double>(candidate[i]);
        if (guitardsp::golden::floatToBits(golden[i]) != guitardsp::golden::floatToBits(candidate[i])) {
            bitExact = false;
        }
        const double diff = std::abs(g - c);
        maxAbsError = std::max(maxAbsError, diff);
        sumSquaredError += diff * diff;
        if (!firstExceedingIndex && diff > opts.tolerance) {
            firstExceedingIndex = i;
        }
    }

    const double rmsError =
        golden.empty() ? 0.0 : std::sqrt(sumSquaredError / static_cast<double>(golden.size()));

    std::cout << "samples: " << golden.size() << '\n';
    std::cout << "max_abs_error: " << maxAbsError << '\n';
    std::cout << "rms_error: " << rmsError << '\n';
    std::cout << "tolerance: " << opts.tolerance << '\n';
    std::cout << "first_exceeding_sample: "
              << (firstExceedingIndex ? std::to_string(*firstExceedingIndex) : std::string("none")) << '\n';
    std::cout << "bit_exact: " << (bitExact ? "true" : "false") << '\n';

    return firstExceedingIndex ? 1 : 0;
}
