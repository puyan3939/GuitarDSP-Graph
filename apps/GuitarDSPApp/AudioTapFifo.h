#pragma once

#include <juce_core/juce_core.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace guitardsp::app {

// Lock-free, allocation-free single-producer (audio thread) / single-consumer
// (message thread) sample relay used to feed a UI-thread waveform display
// without ever touching a lock or the heap from the audio callback.
//
// prepare() must run on the message thread before the audio callback that
// will call push() is attached (mirrors RealtimeAudioEngine::configure(),
// which is called from the same audioDeviceAboutToStart()); push() and
// drain() never allocate, lock, or resize the backing buffer.
class AudioTapFifo {
public:
    void prepare(int capacitySamples) {
        capacitySamples = std::max(capacitySamples, 1);
        buffer_.assign(static_cast<std::size_t>(capacitySamples), 0.0f);
        fifo_.setTotalSize(capacitySamples);
    }

    // Audio thread. If the consumer has fallen behind and the buffer is
    // full, the newest samples are dropped rather than blocking or growing
    // the buffer.
    void push(const float* samples, int numSamples) noexcept {
        if (samples == nullptr || numSamples <= 0 || buffer_.empty()) return;
        int start1 = 0, size1 = 0, start2 = 0, size2 = 0;
        fifo_.prepareToWrite(numSamples, start1, size1, start2, size2);
        if (size1 > 0) std::copy_n(samples, size1, buffer_.data() + start1);
        if (size2 > 0) std::copy_n(samples + size1, size2, buffer_.data() + start2);
        fifo_.finishedWrite(size1 + size2);
    }

    // Message thread only. Invokes visit(const float* samples, int count)
    // once per contiguous run currently queued, oldest first.
    template <typename Visitor>
    void drain(Visitor&& visit) {
        const int ready = fifo_.getNumReady();
        if (ready <= 0) return;
        int start1 = 0, size1 = 0, start2 = 0, size2 = 0;
        fifo_.prepareToRead(ready, start1, size1, start2, size2);
        if (size1 > 0) visit(buffer_.data() + start1, size1);
        if (size2 > 0) visit(buffer_.data() + start2, size2);
        fifo_.finishedRead(size1 + size2);
    }

private:
    juce::AbstractFifo fifo_{1};
    std::vector<float> buffer_;
};

} // namespace guitardsp::app
