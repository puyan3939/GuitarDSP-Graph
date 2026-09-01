#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <numbers>
#include <vector>

namespace guitardsp::hq {

class Radix2FFT {
public:
    static bool isPowerOfTwo(std::size_t n) noexcept {
        return n > 0 && (n & (n - 1)) == 0;
    }

    static void transform(std::vector<std::complex<float>>& data, bool inverse = false) {
        const std::size_t n = data.size();
        if (!isPowerOfTwo(n)) return;

        for (std::size_t i = 1, j = 0; i < n; ++i) {
            std::size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) j ^= bit;
            j ^= bit;
            if (i < j) std::swap(data[i], data[j]);
        }

        for (std::size_t len = 2; len <= n; len <<= 1) {
            const float angle = (inverse ? 2.0f : -2.0f) * std::numbers::pi_v<float> / static_cast<float>(len);
            const std::complex<float> wlen{std::cos(angle), std::sin(angle)};
            for (std::size_t i = 0; i < n; i += len) {
                std::complex<float> w{1.0f, 0.0f};
                const std::size_t half = len >> 1;
                for (std::size_t j = 0; j < half; ++j) {
                    const auto u = data[i + j];
                    const auto v = data[i + j + half] * w;
                    data[i + j] = u + v;
                    data[i + j + half] = u - v;
                    w *= wlen;
                }
            }
        }

        if (inverse) {
            const float inv = 1.0f / static_cast<float>(n);
            for (auto& x : data) x *= inv;
        }
    }
};

inline std::size_t nextPowerOfTwo(std::size_t n) noexcept {
    if (n <= 1) return 1;
    --n;
    for (std::size_t shift = 1; shift < sizeof(std::size_t) * 8; shift <<= 1) n |= n >> shift;
    return n + 1;
}

} // namespace guitardsp::hq
