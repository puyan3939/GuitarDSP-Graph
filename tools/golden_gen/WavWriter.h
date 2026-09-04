#pragma once

// Minimal 32-bit float PCM mono WAV writer, for the optional "listen to
// this golden file" output described in docs/GOLDEN_REFERENCE.md. Only ever
// invoked from golden_gen (offline, control-thread tooling) via --wav-dir;
// the golden reference comparison itself never reads these files, and they
// must not be committed.

#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace guitardsp::golden {

inline void writeU32(std::ofstream& out, std::uint32_t value) {
    char bytes[4] = {static_cast<char>(value & 0xff), static_cast<char>((value >> 8) & 0xff),
                      static_cast<char>((value >> 16) & 0xff), static_cast<char>((value >> 24) & 0xff)};
    out.write(bytes, 4);
}

inline void writeU16(std::ofstream& out, std::uint16_t value) {
    char bytes[2] = {static_cast<char>(value & 0xff), static_cast<char>((value >> 8) & 0xff)};
    out.write(bytes, 2);
}

inline void writeMonoFloatWav(const std::string& path, const std::vector<float>& samples, double sampleRateHz) {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path);

    const std::uint32_t dataBytes = static_cast<std::uint32_t>(samples.size() * sizeof(float));
    const std::uint16_t bitsPerSample = 32;
    const std::uint16_t channels = 1;
    const std::uint32_t sr = static_cast<std::uint32_t>(sampleRateHz);
    const std::uint32_t byteRate = sr * channels * (bitsPerSample / 8);
    const std::uint16_t blockAlign = static_cast<std::uint16_t>(channels * (bitsPerSample / 8));

    out.write("RIFF", 4);
    writeU32(out, 36 + dataBytes);
    out.write("WAVE", 4);

    out.write("fmt ", 4);
    writeU32(out, 16);
    writeU16(out, 3); // IEEE float
    writeU16(out, channels);
    writeU32(out, sr);
    writeU32(out, byteRate);
    writeU16(out, blockAlign);
    writeU16(out, bitsPerSample);

    out.write("data", 4);
    writeU32(out, dataBytes);
    out.write(reinterpret_cast<const char*>(samples.data()), static_cast<std::streamsize>(dataBytes));
}

} // namespace guitardsp::golden
