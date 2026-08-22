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
                            private juce::Timer,
                            private juce::ChangeListener {
public:
    MainComponent()
        : deviceSelector_(deviceManager_, 1, 2, 1, 2,
                          false, false, true, false) {
        formatManager_.registerBasicFormats();

        addAndMakeVisible(deviceSelector_);
        addAndMakeVisible(pedalBox_);
        addAndMakeVisible(ampBox_);
        addAndMakeVisible(qualityBox_);
        addAndMakeVisible(inputRoutingBox_);
        addAndMakeVisible(ampEnabled_);
        addAndMakeVisible(cabEnabled_);
        addAndMakeVisible(safeDry_);
        addAndMakeVisible(mute_);
        addAndMakeVisible(inputTrim_);
        addAndMakeVisible(outputTrim_);
        addAndMakeVisible(loadIrButton_);
        addAndMakeVisible(resetDiagnosticsButton_);
        addAndMakeVisible(statusLabel_);
        addAndMakeVisible(irLabel_);
        addAndMakeVisible(meterLabel_);
        addAndMakeVisible(routingLabel_);
        addAndMakeVisible(performanceLabel_);
        addAndMakeVisible(latencyLabel_);
        addAndMakeVisible(safetyLabel_);
        addAndMakeVisible(pedalControlsTitle_);
        addAndMakeVisible(ampControlsTitle_);

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

        inputRoutingBox_.addItem("Auto mono: strongest input", 1);
        inputRoutingBox_.addItem("Input 1 / left", 2);
        inputRoutingBox_.addItem("Input 2 / right", 3);
        inputRoutingBox_.addItem("Independent stereo", 4);
        inputRoutingBox_.setSelectedId(1, juce::dontSendNotification);

        ampEnabled_.setButtonText("Amp");
        ampEnabled_.setToggleState(true, juce::dontSendNotification);
        cabEnabled_.setButtonText("Speaker + Cab IR");
        cabEnabled_.setToggleState(true, juce::dontSendNotification);
        safeDry_.setButtonText("Safe dry monitor");
        safeDry_.setToggleState(true, juce::dontSendNotification);
        mute_.setButtonText("Mute output");
        mute_.setToggleState(true, juce::dontSendNotification);

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

        auto configureToneControl = [this](juce::Slider& slider,
                                            const juce::String& name,
                                            double initialValue) {
            addAndMakeVisible(slider);
            slider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
            slider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 118, 22);
            slider.setRange(0.0, 100.0, 1.0);
            slider.setValue(initialValue * 100.0, juce::dontSendNotification);
            slider.setTextValueSuffix("% " + name);
            slider.onValueChange = [this] { toneControlsPending_ = true; };
        };
        configureToneControl(pedalDrive_, "DRIVE", settings_.pedalDrive);
        configureToneControl(pedalTone_, "TONE", settings_.pedalTone);
        configureToneControl(pedalLevel_, "LEVEL", settings_.pedalLevel);
        configureToneControl(ampGain_, "GAIN", settings_.ampGain);
        configureToneControl(ampBass_, "BASS", settings_.ampBass);
        configureToneControl(ampMid_, "MID", settings_.ampMid);
        configureToneControl(ampTreble_, "TREBLE", settings_.ampTreble);
        configureToneControl(ampMaster_, "MASTER", settings_.ampMaster);
        configureToneControl(ampPresence_, "PRES", settings_.ampPresence);

        pedalControlsTitle_.setText("CIRCUIT PEDAL", juce::dontSendNotification);
        ampControlsTitle_.setText("AMPLIFIER", juce::dontSendNotification);
        loadIrButton_.setButtonText("Load measured cabinet IR WAV");
        resetDiagnosticsButton_.setButtonText("Reset CPU / XRUN / clip counters");
        statusLabel_.setText("Audio device not started", juce::dontSendNotification);
        irLabel_.setText("Cab IR: synthetic reference fallback (NOT measured)",
                         juce::dontSendNotification);
        meterLabel_.setText("Input: -inf dBFS    Output: -inf dBFS",
                            juce::dontSendNotification);

        pedalBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        ampBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        qualityBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        inputRoutingBox_.onChange = [this] { updateInputRouting(); };
        ampEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        cabEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        safeDry_.onClick = [this] { rebuildRig(); };
        mute_.onClick = [this] { engine_.setMuted(mute_.getToggleState()); };
        inputTrim_.onValueChange = [this] {
            engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        };
        outputTrim_.onValueChange = [this] {
            engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        };
        loadIrButton_.onClick = [this] { chooseImpulseResponse(); };
        resetDiagnosticsButton_.onClick = [this] {
            engine_.resetDiagnostics();
            xRunBaseline_ = std::max(0, deviceManager_.getXRunCount());
        };

        engine_.setInputTrimDb(0.0f);
        engine_.setOutputTrimDb(-12.0f);
        engine_.setSafetyCeiling(0.98f);
        engine_.setMuted(true);
        updateSettingsFromControls();
        updateInputRouting();

        const auto stateFile = audioStateFile();
        auto savedState = stateFile.existsAsFile()
            ? juce::XmlDocument::parse(stateFile)
            : std::unique_ptr<juce::XmlElement>{};
        deviceManager_.addChangeListener(this);
        const auto error = deviceManager_.initialise(2, 2, savedState.get(), true, "*WAVIO*");
        if (error.isNotEmpty())
            statusLabel_.setText("Audio init error: " + error, juce::dontSendNotification);
        deviceManager_.addAudioCallback(this);

        setSize(1120, 880);
        startTimerHz(20);
    }

    ~MainComponent() override {
        stopTimer();
        persistAudioDeviceState();
        deviceManager_.removeChangeListener(this);
        deviceManager_.removeAudioCallback(this);
        engine_.collectRetired();
    }

    void resized() override {
        auto area = getLocalBounds().reduced(12);
        statusLabel_.setBounds(area.removeFromTop(30));

        deviceSelector_.setBounds(area.removeFromTop(290));
        area.removeFromTop(8);

        auto row = area.removeFromTop(34);
        pedalBox_.setBounds(row.removeFromLeft(195));
        row.removeFromLeft(8);
        ampBox_.setBounds(row.removeFromLeft(210));
        row.removeFromLeft(8);
        qualityBox_.setBounds(row.removeFromLeft(205));
        row.removeFromLeft(8);
        inputRoutingBox_.setBounds(row);

        area.removeFromTop(6);
        row = area.removeFromTop(30);
        safeDry_.setBounds(row.removeFromLeft(180));
        ampEnabled_.setBounds(row.removeFromLeft(90));
        cabEnabled_.setBounds(row.removeFromLeft(180));
        mute_.setBounds(row.removeFromLeft(150));
        resetDiagnosticsButton_.setBounds(row);

        area.removeFromTop(8);
        row = area.removeFromTop(24);
        pedalControlsTitle_.setBounds(row.removeFromLeft(360));
        ampControlsTitle_.setBounds(row);

        row = area.removeFromTop(108);
        auto pedalArea = row.removeFromLeft(360);
        pedalDrive_.setBounds(pedalArea.removeFromLeft(118));
        pedalTone_.setBounds(pedalArea.removeFromLeft(118));
        pedalLevel_.setBounds(pedalArea.removeFromLeft(118));
        const int ampWidth = std::max(90, row.getWidth() / 6);
        ampGain_.setBounds(row.removeFromLeft(ampWidth));
        ampBass_.setBounds(row.removeFromLeft(ampWidth));
        ampMid_.setBounds(row.removeFromLeft(ampWidth));
        ampTreble_.setBounds(row.removeFromLeft(ampWidth));
        ampMaster_.setBounds(row.removeFromLeft(ampWidth));
        ampPresence_.setBounds(row);

        area.removeFromTop(8);
        row = area.removeFromTop(34);
        inputTrim_.setBounds(row.removeFromLeft(row.getWidth() / 2 - 6));
        row.removeFromLeft(12);
        outputTrim_.setBounds(row);

        area.removeFromTop(8);
        row = area.removeFromTop(34);
        loadIrButton_.setBounds(row.removeFromLeft(300));
        row.removeFromLeft(12);
        irLabel_.setBounds(row);

        area.removeFromTop(10);
        routingLabel_.setBounds(area.removeFromTop(27));
        meterLabel_.setBounds(area.removeFromTop(27));
        performanceLabel_.setBounds(area.removeFromTop(27));
        latencyLabel_.setBounds(area.removeFromTop(27));
        safetyLabel_.setBounds(area.removeFromTop(27));
    }

    void audioDeviceAboutToStart(juce::AudioIODevice* device) override {
        if (device == nullptr) return;
        currentSampleRate_ = device->getCurrentSampleRate();
        currentBlockSize_ = device->getCurrentBufferSizeSamples();
        currentInputLatencySamples_ = std::max(0, device->getInputLatencyInSamples());
        currentOutputLatencySamples_ = std::max(0, device->getOutputLatencyInSamples());
        const int outputChannels = device->getActiveOutputChannels().countNumberOfSetBits();
        // A guitar selected from either physical jack is one signal. Run the
        // complete component-level rig once, then let RealtimeAudioEngine fan the
        // result out to both hardware outputs. Independent stereo deliberately
        // retains two separate circuit and capacitor histories.
        const bool independentStereo = inputRoutingBox_.getSelectedId() == 4;
        processingChannels_ = independentStereo && outputChannels >= 2 ? 2 : 1;

        auto settings = settingsForCurrentDevice();
        const bool ok = engine_.configure(currentSampleRate_, currentBlockSize_,
                                          processingChannels_, settings);
        engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        engine_.setMuted(mute_.getToggleState());
        updateInputRouting();

        const juce::String message = ok
            ? "Audio ready: " + device->getName() + " / "
                + juce::String(currentSampleRate_, 0) + " Hz / "
                + juce::String(currentBlockSize_) + " samples / graph latency "
                + juce::String(engine_.stats().graphLatencySamples) + " samples"
                + (mute_.getToggleState() ? " / OUTPUT MUTED" : "")
            : "Failed to prepare DSP rig";
        juce::MessageManager::callAsync([safe = juce::Component::SafePointer<MainComponent>(this), message] {
            if (safe != nullptr) safe->statusLabel_.setText(message, juce::dontSendNotification);
        });
    }

    void audioDeviceStopped() override {
        currentSampleRate_ = 0.0;
        currentBlockSize_ = 0;
        currentInputLatencySamples_ = 0;
        currentOutputLatencySamples_ = 0;
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
        const juce::ScopedNoDenormals noDenormals;
        engine_.process(inputChannelData, numInputChannels,
                        outputChannelData, numOutputChannels, numSamples);
    }

private:
    void timerCallback() override {
        engine_.collectRetired();
        if (toneControlsPending_) applyToneControls();
        const auto stats = engine_.stats();
        const auto dbText = [](float peak) {
            return peak > 1.0e-9f
                ? juce::String(juce::Decibels::gainToDecibels(peak), 1) + " dBFS"
                : juce::String("-inf dBFS");
        };

        juce::String selected = "none";
        if (stats.inputRoutingMode == guitardsp::app::InputRoutingMode::stereo)
            selected = "independent stereo";
        else if (stats.selectedInputChannel >= 0)
            selected = "Input " + juce::String(stats.selectedInputChannel + 1);

        routingLabel_.setText(
            "Physical input 1: " + dbText(stats.physicalInputPeaks[0])
                + "    Physical input 2: " + dbText(stats.physicalInputPeaks[1])
                + "    Selected: " + selected,
            juce::dontSendNotification);
        meterLabel_.setText(
            "DSP input: " + dbText(stats.inputPeak)
                + "    Output: " + dbText(stats.outputPeak)
                + "    ADC clips: "
                + juce::String(static_cast<juce::int64>(stats.inputClippedSamples))
                + "    Output safety clips: "
                + juce::String(static_cast<juce::int64>(stats.clippedSamples)),
            juce::dontSendNotification);

        const double driverCpu = 100.0 * std::max(0.0, deviceManager_.getCpuUsage());
        const double callbackCpu = 100.0 * static_cast<double>(stats.performance.averageLoad);
        const double callbackPeak = 100.0 * static_cast<double>(stats.performance.peakLoad);
        const int xruns = std::max(0, deviceManager_.getXRunCount() - xRunBaseline_);
        performanceLabel_.setText(
            "Driver CPU: " + juce::String(driverCpu, 1)
                + "%    Callback average: " + juce::String(callbackCpu, 1)
                + "%    Peak: " + juce::String(callbackPeak, 1)
                + "%    Deadline misses: "
                + juce::String(static_cast<juce::int64>(stats.performance.deadlineMisses))
                + "    XRUNs: " + juce::String(xruns),
            juce::dontSendNotification);

        if (currentSampleRate_ > 0.0) {
            const auto toMilliseconds = [this](int samples) {
                return 1000.0 * static_cast<double>(samples) / currentSampleRate_;
            };
            const int ioLatency = currentInputLatencySamples_ + currentOutputLatencySamples_;
            const int totalReportedLatency = ioLatency + stats.graphLatencySamples;
            latencyLabel_.setText(
                "Buffer: " + juce::String(currentBlockSize_) + " samples / "
                    + juce::String(toMilliseconds(currentBlockSize_), 2)
                    + " ms    Device I/O: " + juce::String(toMilliseconds(ioLatency), 2)
                    + " ms    DSP: "
                    + juce::String(toMilliseconds(stats.graphLatencySamples), 2)
                    + " ms    Reported total: "
                    + juce::String(toMilliseconds(totalReportedLatency), 2) + " ms",
                juce::dontSendNotification);
        }

        const bool fault = xruns > 0 || stats.performance.deadlineMisses > 0
            || stats.nonFiniteInputSamples > 0 || stats.nonFiniteOutputSamples > 0;
        const bool overload = driverCpu > 80.0 || callbackPeak > 90.0;
        safetyLabel_.setColour(juce::Label::textColourId,
            fault ? juce::Colours::orangered
                  : overload || mute_.getToggleState() ? juce::Colours::orange
                                                        : juce::Colours::lightgreen);
        safetyLabel_.setText(
            (mute_.getToggleState() ? "OUTPUT MUTED    " : "OUTPUT ACTIVE    ")
                + juce::String(safeDry_.getToggleState()
                    ? "Safe dry monitor    " : "Full pedal / amp / cabinet path    ")
                + "Nonfinite input: "
                + juce::String(static_cast<juce::int64>(stats.nonFiniteInputSamples))
                + "    Nonfinite output: "
                + juce::String(static_cast<juce::int64>(stats.nonFiniteOutputSamples)),
            juce::dontSendNotification);
    }

    void changeListenerCallback(juce::ChangeBroadcaster* source) override {
        if (source == &deviceManager_) persistAudioDeviceState();
    }

    static juce::File audioStateFile() {
        return juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
            .getChildFile("GuitarDSP")
            .getChildFile("GuitarDSPGraphAudioDevice.xml");
    }

    void persistAudioDeviceState() {
        const auto state = deviceManager_.createStateXml();
        if (state == nullptr) return;
        const auto file = audioStateFile();
        if (file.getParentDirectory().createDirectory().wasOk())
            file.replaceWithText(state->toString());
    }

    void updateInputRouting() {
        using guitardsp::app::InputRoutingMode;
        InputRoutingMode mode = InputRoutingMode::autoMono;
        switch (inputRoutingBox_.getSelectedId()) {
            case 2: mode = InputRoutingMode::input1; break;
            case 3: mode = InputRoutingMode::input2; break;
            case 4: mode = InputRoutingMode::stereo; break;
            default: break;
        }
        engine_.setInputRoutingMode(mode);

        const auto* device = deviceManager_.getCurrentAudioDevice();
        if (device == nullptr || !engine_.configured()) return;
        const int outputs = device->getActiveOutputChannels().countNumberOfSetBits();
        const int requiredChannels = mode == InputRoutingMode::stereo && outputs >= 2
            ? 2 : 1;
        if (requiredChannels == processingChannels_) return;

        // Removing the callback first synchronizes with any in-flight audio call.
        // Adding it again invokes audioDeviceAboutToStart(), which safely prepares
        // all graph buffers and component histories at the new channel count.
        deviceManager_.removeAudioCallback(this);
        deviceManager_.addAudioCallback(this);
    }

    void applyToneControls() {
        toneControlsPending_ = false;
        const auto normalized = [](const juce::Slider& slider) {
            return static_cast<float>(slider.getValue() * 0.01);
        };
        settings_.pedalDrive = normalized(pedalDrive_);
        settings_.pedalTone = normalized(pedalTone_);
        settings_.pedalLevel = normalized(pedalLevel_);
        settings_.ampGain = normalized(ampGain_);
        settings_.ampBass = normalized(ampBass_);
        settings_.ampMid = normalized(ampMid_);
        settings_.ampTreble = normalized(ampTreble_);
        settings_.ampMaster = normalized(ampMaster_);
        settings_.ampPresence = normalized(ampPresence_);

        if (!engine_.configured() || safeDry_.getToggleState()) return;
        using guitardsp::graph::NodeCategory;
        engine_.setNodeParameter(NodeCategory::drive, 0, settings_.pedalDrive);
        engine_.setNodeParameter(NodeCategory::drive, 1, settings_.pedalTone);
        engine_.setNodeParameter(NodeCategory::drive, 2, settings_.pedalLevel);
        engine_.setNodeParameter(NodeCategory::amp, 0, settings_.ampGain);
        engine_.setNodeParameter(NodeCategory::amp, 1, settings_.ampBass);
        engine_.setNodeParameter(NodeCategory::amp, 2, settings_.ampMid);
        engine_.setNodeParameter(NodeCategory::amp, 3, settings_.ampTreble);
        engine_.setNodeParameter(NodeCategory::amp, 4, settings_.ampMaster);
        engine_.setNodeParameter(NodeCategory::amp, 5, settings_.ampPresence);
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
        if (safeDry_.getToggleState()) {
            result.pedal = guitardsp::app::PedalModel::bypass;
            result.ampEnabled = false;
            result.cabinetEnabled = false;
            result.cabinetImpulse.clear();
            return result;
        }
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
    juce::ComboBox inputRoutingBox_;
    juce::ToggleButton ampEnabled_;
    juce::ToggleButton cabEnabled_;
    juce::ToggleButton safeDry_;
    juce::ToggleButton mute_;
    juce::Slider inputTrim_;
    juce::Slider outputTrim_;
    juce::Slider pedalDrive_;
    juce::Slider pedalTone_;
    juce::Slider pedalLevel_;
    juce::Slider ampGain_;
    juce::Slider ampBass_;
    juce::Slider ampMid_;
    juce::Slider ampTreble_;
    juce::Slider ampMaster_;
    juce::Slider ampPresence_;
    juce::TextButton loadIrButton_;
    juce::TextButton resetDiagnosticsButton_;
    juce::Label statusLabel_;
    juce::Label irLabel_;
    juce::Label meterLabel_;
    juce::Label routingLabel_;
    juce::Label performanceLabel_;
    juce::Label latencyLabel_;
    juce::Label safetyLabel_;
    juce::Label pedalControlsTitle_;
    juce::Label ampControlsTitle_;
    std::unique_ptr<juce::FileChooser> fileChooser_;

    std::vector<float> loadedIr_;
    double loadedIrSampleRate_ = 0.0;
    double currentSampleRate_ = 0.0;
    int currentBlockSize_ = 0;
    int currentInputLatencySamples_ = 0;
    int currentOutputLatencySamples_ = 0;
    int processingChannels_ = 2;
    int xRunBaseline_ = 0;
    bool toneControlsPending_ = false;
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
    const juce::String getApplicationVersion() override { return "0.32.0"; }
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
