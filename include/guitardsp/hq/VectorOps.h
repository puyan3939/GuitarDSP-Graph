#pragma once

#include <algorithm>
#include <cstddef>

#if defined(__SSE2__)
  #include <emmintrin.h>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
  #include <arm_neon.h>
#endif

namespace guitardsp::hq {

struct VectorOps {
    static constexpr bool hasNativeSIMD() noexcept {
#if defined(__SSE2__) || defined(__ARM_NEON) || defined(__ARM_NEON__)
        return true;
#else
        return false;
#endif
    }

    static void clear(float* dst, std::size_t n) noexcept {
        std::fill(dst, dst + n, 0.0f);
    }

    static void copy(const float* src, float* dst, std::size_t n) noexcept {
        std::copy(src, src + n, dst);
    }

    static void multiply(float* data, float gain, std::size_t n) noexcept {
        std::size_t i = 0;
#if defined(__SSE2__)
        const __m128 g = _mm_set1_ps(gain);
        for (; i + 4 <= n; i += 4) {
            const __m128 x = _mm_loadu_ps(data + i);
            _mm_storeu_ps(data + i, _mm_mul_ps(x, g));
        }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
        const float32x4_t g = vdupq_n_f32(gain);
        for (; i + 4 <= n; i += 4) {
            const float32x4_t x = vld1q_f32(data + i);
            vst1q_f32(data + i, vmulq_f32(x, g));
        }
#endif
        for (; i < n; ++i) data[i] *= gain;
    }

    static void add(const float* src, float* dst, std::size_t n) noexcept {
        std::size_t i = 0;
#if defined(__SSE2__)
        for (; i + 4 <= n; i += 4) {
            const __m128 a = _mm_loadu_ps(dst + i);
            const __m128 b = _mm_loadu_ps(src + i);
            _mm_storeu_ps(dst + i, _mm_add_ps(a, b));
        }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
        for (; i + 4 <= n; i += 4) {
            const float32x4_t a = vld1q_f32(dst + i);
            const float32x4_t b = vld1q_f32(src + i);
            vst1q_f32(dst + i, vaddq_f32(a, b));
        }
#endif
        for (; i < n; ++i) dst[i] += src[i];
    }

    static void addScaled(const float* src, float* dst, float gain, std::size_t n) noexcept {
        std::size_t i = 0;
#if defined(__SSE2__)
        const __m128 g = _mm_set1_ps(gain);
        for (; i + 4 <= n; i += 4) {
            const __m128 a = _mm_loadu_ps(dst + i);
            const __m128 b = _mm_loadu_ps(src + i);
            _mm_storeu_ps(dst + i, _mm_add_ps(a, _mm_mul_ps(b, g)));
        }
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
        const float32x4_t g = vdupq_n_f32(gain);
        for (; i + 4 <= n; i += 4) {
            const float32x4_t a = vld1q_f32(dst + i);
            const float32x4_t b = vld1q_f32(src + i);
            vst1q_f32(dst + i, vmlaq_f32(a, b, g));
        }
#endif
        for (; i < n; ++i) dst[i] += src[i] * gain;
    }

    static float peakAbs(const float* src, std::size_t n) noexcept {
        float peak = 0.0f;
        for (std::size_t i = 0; i < n; ++i) peak = std::max(peak, src[i] >= 0.0f ? src[i] : -src[i]);
        return peak;
    }
};

} // namespace guitardsp::hq
