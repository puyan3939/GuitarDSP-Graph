#include <juce_audio_formats/juce_audio_formats.h>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_gui_extra/juce_gui_extra.h>

#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/ReferenceCabinetIR.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

class MainComponent final : public juce::Component,
                            public juce::AudioIODeviceCallback,
                            private juce::Timer {
public:
    MainComponent()
        : deviceSelector_(deviceManager_, 1, 2, 1, 2,
                          false, false, true, false) {
        formatManager_.registerBasicFormats();

        addAndMakeVisible(deviceSelector_);
        addAndMakeVisible(pedalBox_);
        addAndMakeVisible(ampBox_);
        addAndMakeVisible(qualityBox_);
        addAndMakeVisible(ampEnabled_);
        addAndMakeVisible(cabEnabled_);
        addAndMakeVisible(mute_);
        addAndMakeVisible(inputTrim_);
        addAndMakeVisible(outputTrim_);
        addAndMakeVisible(loadIrButton_);
        addAndMakeVisible(statusLabel_);
        addAndMakeVisible(irLabel_);
        addAndMakeVisible(meterLabel_);

        pedalBox_.addItem("Bypass", 1);
        pedalBox_.addItem("TS808 Circuit", 2);
        pedalBox_.addItem("DS-1 Circuit", 3);
        pedalBox_.setSelectedId(2, juce::dontSendNotification);

        ampBox_.addItem("Reference Amp", 1);
        ampBox_.addItem("British Plexi Family", 2);
        ampBox_.addItem("American Clean Family", 3);
        ampBox_.setSelectedId(1, juce::dontSendNotification);

        qualityBox_.addItem("Eco (2x nonlinear)", 1);
        qualityBox_.addItem("Live (4x nonlinear)", 2);
        qualityBox_.addItem("High (8x nonlinear)", 3);
        qualityBox_.addItem("Studio (16x nonlinear)", 4);
        qualityBox_.setSelectedId(3, juce::dontSendNotification);

        ampEnabled_.setButtonText("Amp");
        ampEnabled_.setToggleState(true, juce::dontSendNotification);
        cabEnabled_.setButtonText("Speaker + Cab IR");
        cabEnabled_.setToggleState(true, juce::dontSendNotification);
        mute_.setButtonText("Mute output");

        inputTrim_.setSliderStyle(juce::Slider::LinearHorizontal);
        inputTrim_.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 22);
        inputTrim_.setRange(-24.0, 12.0, 0.1);
        inputTrim_.setValue(0.0, juce::dontSendNotification);
        inputTrim_.setTextValueSuffix(" dB in");

        outputTrim_.setSliderStyle(juce::Slider::LinearHorizontal);
        outputTrim_.setTextBoxStyle(juce::Slider::TextBoxRight, false, 80, 22);
        outputTrim_.setRange(-60.0, 0.0, 0.1);
        outputTrim_.setValue(-12.0, juce::dontSendNotification);
        outputTrim_.setTextValueSuffix(" dB out");

        loadIrButton_.setButtonText("Load measured cabinet IR WAV");
        statusLabel_.setText("Audio device not started", juce::dontSendNotification);
        irLabel_.setText("Cab IR: synthetic reference fallback (NOT measured)",
                         juce::dontSendNotification);
        meterLabel_.setText("Input: -inf dBFS    Output: -inf dBFS",
                            juce::dontSendNotification);

        pedalBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        ampBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        qualityBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        ampEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        cabEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        mute_.onClick = [this] { engine_.setMuted(mute_.getToggleState()); };
        inputTrim_.onValueChange = [this] {
            engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        };
        outputTrim_.onValueChange = [this] {
            engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        };
        loadIrButton_.onClick = [this] { chooseImpulseResponse(); };

        engine_.setInputTrimDb(0.0f);
        engine_.setOutputTrimDb(-12.0f);
        engine_.setSafetyCeiling(0.98f);
        updateSettingsFromControls();

        const auto error = deviceManager_.initialise(1, 2, nullptr, true);
        if (error.isNotEmpty())
            statusLabel_.setText("Audio init error: " + error, juce::dontSendNotification);
        deviceManager_.addAudioCallback(this);

        setSize(980, 760);
        startTimerHz(8);
    }

    ~MainComponent() override {
        stopTimer();
        deviceManager_.removeAudioCallback(this);
        engine_.collectRetired();
    }

    void resized() override {
        auto area = getLocalBounds().reduced(12);
        auto titleArea = area.removeFromTop(34);
        statusLabel_.setBounds(titleArea.removeFromLeft(620));
        meterLabel_.setBounds(titleArea);

        deviceSelector_.setBounds(area.removeFromTop(330));
        area.removeFromTop(8);

        auto row = area.removeFromTop(34);
        pedalBox_.setBounds(row.removeFromLeft(220));
        row.removeFromLeft(8);
        ampBox_.setBounds(row.removeFromLeft(240));
        row.removeFromLeft(8);
        qualityBox_.setBounds(row.removeFromLeft(220));
        row.removeFromLeft(12);
        ampEnabled_.setBounds(row.removeFromLeft(70));
        cabEnabled_.setBounds(row);

        area.removeFromTop(8);
        row = area.removeFromTop(34);
        inputTrim_.setBounds(row.removeFromLeft(440));
        row.removeFromLeft(12);
        outputTrim_.setBounds(row.removeFromLeft(440));

        area.removeFromTop(8);
        row = area.removeFromTop(34);
        loadIrButton_.setBounds(row.removeFromLeft(300));
        row.removeFromLeft(12);
        irLabel_.setBounds(row);

        area.removeFromTop(8);
        mute_.setBounds(area.removeFromTop(30).removeFromLeft(160));
    }

    void audioDeviceAboutToStart(juce::AudioIODevice* device) override {
        if (device == nullptr) return;
        currentSampleRate_ = device->getCurrentSampleRate();
        currentBlockSize_ = device->getCurrentBufferSizeSamples();
        const int outputChannels = device->getActiveOutputChannels().countNumberOfSetBits();
        processingChannels_ = outputChannels >= 2 ? 2 : 1;

        auto settings = settingsForCurrentDevice();
        const bool ok = engine_.configure(currentSampleRate_, currentBlockSize_,
                                          processingChannels_, settings);
        engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        engine_.setMuted(mute_.getToggleState());

        const juce::String message = ok
            ? "Audio ready: " + juce::String(currentSampleRate_, 0) + " Hz / "
                + juce::String(currentBlockSize_) + " samples / graph latency "
                + juce::String(engine_.stats().graphLatencySamples) + " samples"
            : "Failed to prepare DSP rig";
        juce::MessageManager::callAsync([safe = juce::Component::SafePointer<MainComponent>(this), message] {
            if (safe != nullptr) safe->statusLabel_.setText(message, juce::dontSendNotification);
        });
    }

    void audioDeviceStopped() override {
        currentSampleRate_ = 0.0;
        currentBlockSize_ = 0;
        juce::MessageManager::callAsync([safe = juce::Component::SafePointer<MainComponent>(this)] {
            if (safe != nullptr)
                safe->statusLabel_.setText("Audio device stopped", juce::dontSendNotification);
        });
    }

    void audioDeviceIOCallbackWithContext(const float* const* inputChannelData,
                                          int numInputChannels,
                                          float* const* outputChannelData,
                                          int numOutputChannels,
                                          int numSamples,
                                          const juce::AudioIODeviceCallbackContext&) override {
        engine_.process(inputChannelData, numInputChannels,
                        outputChannelData, numOutputChannels, numSamples);
    }

private:
    void timerCallback() override {
        engine_.collectRetired();
        const auto stats = engine_.stats();
        const auto inputDb = stats.inputPeak > 1.0e-9f
            ? juce::Decibels::gainToDecibels(stats.inputPeak) : -100.0f;
        const auto outputDb = stats.outputPeak > 1.0e-9f
            ? juce::Decibels::gainToDecibels(stats.outputPeak) : -100.0f;
        meterLabel_.setText("Input: " + juce::String(inputDb, 1) + " dBFS    Output: "
                                + juce::String(outputDb, 1) + " dBFS    Safety clips: "
                                + juce::String(static_cast<juce::int64>(stats.clippedSamples)),
                            juce::dontSendNotification);
    }

    void updateSettingsFromControls() {
        switch (pedalBox_.getSelectedId()) {
            case 1: settings_.pedal = guitardsp::app::PedalModel::bypass; break;
            case 3: settings_.pedal = guitardsp::app::PedalModel::ds1Circuit; break;
            default: settings_.pedal = guitardsp::app::PedalModel::ts808Circuit; break;
        }
        switch (ampBox_.getSelectedId()) {
            case 2: settings_.amp = guitardsp::app::AmpModel::britishPlexiFamily; break;
            case 3: settings_.amp = guitardsp::app::AmpModel::americanCleanFamily; break;
            default: settings_.amp = guitardsp::app::AmpModel::reference; break;
        }
        switch (qualityBox_.getSelectedId()) {
            case 1: settings_.quality = guitardsp::graph::ProcessingQuality::eco; break;
            case 2: settings_.quality = guitardsp::graph::ProcessingQuality::live; break;
            case 4: settings_.quality = guitardsp::graph::ProcessingQuality::studio; break;
            default: settings_.quality = guitardsp::graph::ProcessingQuality::high; break;
        }
        settings_.ampEnabled = ampEnabled_.getToggleState();
        settings_.cabinetEnabled = cabEnabled_.getToggleState();
    }

    guitardsp::app::LiveRigSettings settingsForCurrentDevice() const {
        auto result = settings_;
        if (!loadedIr_.empty() && loadedIrSampleRate_ > 0.0 && currentSampleRate_ > 0.0) {
            result.cabinetImpulse = guitardsp::app::resampleImpulseWindowedSinc(
                loadedIr_, loadedIrSampleRate_, currentSampleRate_);
        } else {
            result.cabinetImpulse.clear();
        }
        return result;
    }

    void rebuildRig() {
        if (!engine_.configured() || currentSampleRate_ <= 0.0) return;
        auto preparedSettings = settingsForCurrentDevice();
        if (!engine_.rebuildRig(preparedSettings)) {
            statusLabel_.setText("Rig rebuild failed; previous graph kept active",
                                 juce::dontSendNotification);
            return;
        }
        const auto stats = engine_.stats();
        statusLabel_.setText("Rig queued at block boundary / latency "
                                + juce::String(stats.graphLatencySamples) + " samples",
                             juce::dontSendNotification);
    }

    void chooseImpulseResponse() {
        fileChooser_ = std::make_unique<juce::FileChooser>(
            "Choose a measured cabinet IR WAV/AIFF", juce::File{}, "*.wav;*.aif;*.aiff");
        fileChooser_->launchAsync(juce::FileBrowserComponent::openMode
                                    | juce::FileBrowserComponent::canSelectFiles,
                                  [safe = juce::Component::SafePointer<MainComponent>(this)](
                                      const juce::FileChooser& chooser) {
            if (safe == nullptr) return;
            const auto file = chooser.getResult();
            if (file.existsAsFile()) safe->loadImpulseResponse(file);
        });
    }

    void loadImpulseResponse(const juce::File& file) {
        auto reader = std::unique_ptr<juce::AudioFormatReader>(formatManager_.createReaderFor(file));
        if (!reader) {
            irLabel_.setText("IR load failed: unsupported file", juce::dontSendNotification);
            return;
        }

        constexpr std::int64_t maximumIrSamples = 262144;
        const auto samplesToRead64 = std::min<std::int64_t>(reader->lengthInSamples,
                                                            maximumIrSamples);
        if (samplesToRead64 <= 0) return;
        const int samplesToRead = static_cast<int>(samplesToRead64);
        juce::AudioBuffer<float> buffer(1, samplesToRead);
        if (!reader->read(&buffer, 0, samplesToRead, 0, true, false)) {
            irLabel_.setText("IR load failed while reading audio", juce::dontSendNotification);
            return;
        }

        loadedIr_.assign(buffer.getReadPointer(0), buffer.getReadPointer(0) + samplesToRead);
        loadedIrSampleRate_ = reader->sampleRate;
        irLabel_.setText("Cab IR: " + file.getFileName() + " / "
                            + juce::String(loadedIrSampleRate_, 0) + " Hz / measured external file",
                         juce::dontSendNotification);
        rebuildRig();
    }

    juce::AudioDeviceManager deviceManager_;
    juce::AudioDeviceSelectorComponent deviceSelector_;
    juce::AudioFormatManager formatManager_;
    guitardsp::app::RealtimeAudioEngine engine_;
    guitardsp::app::LiveRigSettings settings_;

    juce::ComboBox pedalBox_;
    juce::ComboBox ampBox_;
    juce::ComboBox qualityBox_;
    juce::ToggleButton ampEnabled_;
    juce::ToggleButton cabEnabled_;
    juce::ToggleButton mute_;
    juce::Slider inputTrim_;
    juce::Slider outputTrim_;
    juce::TextButton loadIrButton_;
    juce::Label statusLabel_;
    juce::Label irLabel_;
    juce::Label meterLabel_;
    std::unique_ptr<juce::FileChooser> fileChooser_;

    std::vector<float> loadedIr_;
    double loadedIrSampleRate_ = 0.0;
    double currentSampleRate_ = 0.0;
    int currentBlockSize_ = 0;
    int processingChannels_ = 2;
};

class MainWindow final : public juce::DocumentWindow {
public:
    MainWindow()
        : DocumentWindow("GuitarDSP Graph",
                         juce::Desktop::getInstance().getDefaultLookAndFeel()
                             .findColour(juce::ResizableWindow::backgroundColourId),
                         DocumentWindow::allButtons) {
        setUsingNativeTitleBar(true);
        setContentOwned(new MainComponent(), true);
        setResizable(true, true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }

    void closeButtonPressed() override {
        juce::JUCEApplication::getInstance()->systemRequestedQuit();
    }
};

class GuitarDSPApplication final : public juce::JUCEApplication {
public:
    const juce::String getApplicationName() override { return "GuitarDSP Graph"; }
    const juce::String getApplicationVersion() override { return "0.31.0"; }
    bool moreThanOneInstanceAllowed() override { return false; }

    void initialise(const juce::String&) override {
        window_ = std::make_unique<MainWindow>();
    }

    void shutdown() override { window_.reset(); }

    void systemRequestedQuit() override { quit(); }
    void anotherInstanceStarted(const juce::String&) override {}

private:
    std::unique_ptr<MainWindow> window_;
};

} // namespace

START_JUCE_APPLICATION(GuitarDSPApplication)
