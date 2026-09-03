#include <juce_audio_formats/juce_audio_formats.h>
#include <juce_audio_utils/juce_audio_utils.h>
#include <juce_gui_extra/juce_gui_extra.h>

#include "AudioTapFifo.h"
#include "SpectrumAnalyserComponent.h"
#include "ThdAnalyser.h"
#include "guitardsp/app/LiveRig.h"
#include "guitardsp/app/LiveRigPresetJson.h"
#include "guitardsp/app/PresetStore.h"
#include "guitardsp/app/RealtimeAudioEngine.h"
#include "guitardsp/app/ReferenceCabinetIR.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

// Holds a loaded measured cabinet IR plus the analysis needed to describe it
// in the UI. Guitar and bass cabinets each keep an independent instance so
// loading one never disturbs the other.
struct MeasuredImpulseState {
    std::vector<float> impulse;
    juce::String name;
    // Full path of the source file, so a saved preset can remember which
    // measured IR was loaded and reload/re-resample it later (see
    // LiveRigPreset.h's doc comment on why the resampled `impulse` data
    // itself isn't what gets persisted).
    juce::String fullPath;
    double sampleRate = 0.0;
    double calibrationGainDb = 0.0;
    double rawPeakDb = 0.0;
    double matchedPeakDb = 0.0;
    bool allFinite = true;
};

class RoutingGraphView final : public juce::Component {
public:
    void setState(guitardsp::app::SignalRouting routing,
                  bool octaveEnabled, bool bassCabinetEnabled) {
        routing_ = routing;
        octaveEnabled_ = octaveEnabled;
        bassCabinetEnabled_ = bassCabinetEnabled;
        repaint();
    }

    void paint(juce::Graphics& graphics) override {
        const auto bounds = getLocalBounds().toFloat().reduced(4.0f);
        graphics.setColour(juce::Colours::black.withAlpha(0.24f));
        graphics.fillRoundedRectangle(bounds, 7.0f);
        if (bounds.getWidth() < 100.0f) return;

        const float middle = bounds.getCentreY();
        const float left = bounds.getX() + 12.0f;
        const float right = bounds.getRight() - 12.0f;
        const auto link = [&](float startX, float startY, float endX, float endY) {
            juce::Path path;
            path.startNewSubPath(startX, startY);
            path.lineTo(endX, endY);
            graphics.setColour(juce::Colours::lightseagreen.withAlpha(0.75f));
            graphics.strokePath(path, juce::PathStrokeType(2.0f));
        };
        const auto node = [&](float centerX, float centerY, float width,
                              const juce::String& text, bool bass) {
            const juce::Rectangle<float> rectangle(centerX - width * 0.5f,
                                                    centerY - 13.0f, width, 26.0f);
            graphics.setColour((bass ? juce::Colours::darkorange
                                     : juce::Colours::steelblue).withAlpha(0.58f));
            graphics.fillRoundedRectangle(rectangle, 4.0f);
            graphics.setColour(juce::Colours::white);
            graphics.drawFittedText(text, rectangle.toNearestInt(),
                                    juce::Justification::centred, 1);
        };

        if (routing_ == guitardsp::app::SignalRouting::serialGuitar) {
            const float span = right - left;
            const std::array<float, 5> points{{left + 44.0f, left + span * 0.27f,
                                                left + span * 0.50f,
                                                left + span * 0.73f, right - 44.0f}};
            for (std::size_t index = 1; index < points.size(); ++index)
                link(points[index - 1], middle, points[index], middle);
            node(points[0], middle, 80.0f, "INPUT", false);
            node(points[1], middle, 92.0f, "PEDAL", false);
            node(points[2], middle, 92.0f, "GUITAR AMP", false);
            node(points[3], middle, 92.0f, "GUITAR CAB", false);
            node(points[4], middle, 80.0f, "OUTPUT", false);
            return;
        }

        const float top = middle - 22.0f;
        const float bottom = middle + 22.0f;
        const float span = right - left;
        const float split = left + span * 0.18f;
        const float merge = left + span * 0.82f;
        link(left + 40.0f, middle, split, middle);
        link(split, middle, split + 55.0f, top);
        link(split, middle, split + 55.0f, bottom);
        link(split + 55.0f, top, merge - 55.0f, top);
        link(split + 55.0f, bottom, merge - 55.0f, bottom);
        link(merge - 55.0f, top, merge, middle);
        link(merge - 55.0f, bottom, merge, middle);
        link(merge, middle, right - 40.0f, middle);

        node(left + 40.0f, middle, 76.0f, "INPUT", false);
        node(split, middle, 80.0f,
             routing_ == guitardsp::app::SignalRouting::crossoverOctaveBass
                 ? "X-OVER" : "SPLIT", false);
        node(left + span * 0.43f, top, 180.0f, "PEDAL + GUITAR AMP + CAB", false);
        juce::String lower = octaveEnabled_ ? "OCT -1 + BASS AMP" : "BASS AMP";
        if (bassCabinetEnabled_) lower += " + CAB";
        node(left + span * 0.57f, bottom, 200.0f, lower, true);
        node(merge, middle, 74.0f, "MIX", false);
        node(right - 40.0f, middle, 76.0f, "OUTPUT", false);
    }

private:
    guitardsp::app::SignalRouting routing_ = guitardsp::app::SignalRouting::serialGuitar;
    bool octaveEnabled_ = true;
    bool bassCabinetEnabled_ = true;
};

class MainComponent final : public juce::Component,
                            public juce::AudioIODeviceCallback,
                            private juce::Timer,
                            private juce::ChangeListener {
public:
    MainComponent()
        : deviceSelector_(deviceManager_, 1, 2, 1, 2,
                          false, false, true, false),
          presetStore_(juce::File::getSpecialLocation(juce::File::userApplicationDataDirectory)
                           .getChildFile("GuitarDSP").getChildFile("Presets")
                           .getFullPathName().toStdString()) {
        formatManager_.registerBasicFormats();

        addChildComponent(deviceSelector_);
        addAndMakeVisible(pedalBox_);
        addAndMakeVisible(ampBox_);
        addAndMakeVisible(qualityBox_);
        addAndMakeVisible(inputRoutingBox_);
        addAndMakeVisible(signalRoutingBox_);
        addAndMakeVisible(powerTubeBox_);
        addAndMakeVisible(toneStackBox_);
        addAndMakeVisible(toneDriverBox_);
        addAndMakeVisible(feedbackVoicingBox_);
        addAndMakeVisible(ampEnabled_);
        addAndMakeVisible(cabEnabled_);
        addAndMakeVisible(octaveEnabled_);
        addAndMakeVisible(bassCabinetEnabled_);
        addAndMakeVisible(safeDry_);
        addAndMakeVisible(mute_);
        addAndMakeVisible(matchIrLevel_);
        addAndMakeVisible(inputTrim_);
        addAndMakeVisible(outputTrim_);
        addAndMakeVisible(testSignalLabel_);
        addAndMakeVisible(testSignalFrequency_);
        addAndMakeVisible(testSignalLevel_);
        addAndMakeVisible(loadIrButton_);
        addAndMakeVisible(loadBassIrButton_);
        addAndMakeVisible(bassIrLabel_);
        addAndMakeVisible(resetDiagnosticsButton_);
        addAndMakeVisible(audioSettingsButton_);
        addAndMakeVisible(rigPageButton_);
        addAndMakeVisible(cabinetPageButton_);
        addAndMakeVisible(routingPageButton_);
        addAndMakeVisible(advancedPageButton_);
        addAndMakeVisible(presetsPageButton_);
        addAndMakeVisible(slotAButton_);
        addAndMakeVisible(slotBButton_);
        addAndMakeVisible(presetBox_);
        addAndMakeVisible(loadPresetButton_);
        addAndMakeVisible(deletePresetButton_);
        addAndMakeVisible(presetNameEditor_);
        addAndMakeVisible(savePresetButton_);
        addAndMakeVisible(presetStatusLabel_);
        addAndMakeVisible(routingGraphView_);
        addAndMakeVisible(inputWaveform_);
        addAndMakeVisible(outputWaveform_);
        addAndMakeVisible(inputSpectrum_);
        addAndMakeVisible(outputSpectrum_);
        addAndMakeVisible(inputMonitorTapBox_);
        addAndMakeVisible(outputMonitorTapBox_);
        addAndMakeVisible(spectrumToggle_);
        addAndMakeVisible(statusLabel_);
        addAndMakeVisible(irLabel_);
        addAndMakeVisible(meterLabel_);
        addAndMakeVisible(routingLabel_);
        addAndMakeVisible(performanceLabel_);
        addAndMakeVisible(latencyLabel_);
        addAndMakeVisible(safetyLabel_);
        addAndMakeVisible(thdLabel_);
        addAndMakeVisible(pedalControlsTitle_);
        addAndMakeVisible(ampControlsTitle_);

        pedalBox_.addItem("Bypass", 1);
        pedalBox_.addItem("TS808 Circuit", 2);
        pedalBox_.addItem("DS-1 Circuit", 3);
        pedalBox_.setSelectedId(2, juce::dontSendNotification);

        ampBox_.addItem("Reference Amp", 1);
        ampBox_.addItem("British Plexi Family", 2);
        ampBox_.addItem("American Clean Family", 3);
        ampBox_.addItem("Preamp Circuit (12AX7)", 4);
        ampBox_.addItem("Full Amp Circuit (12AX7 + EL34)", 5);
        ampBox_.setSelectedId(1, juce::dontSendNotification);

        qualityBox_.addItem("Eco (2x nonlinear)", 1);
        qualityBox_.addItem("Live (4x nonlinear)", 2);
        qualityBox_.addItem("High (8x nonlinear)", 3);
        qualityBox_.addItem("Studio (16x nonlinear)", 4);
        qualityBox_.setSelectedId(1, juce::dontSendNotification);

        inputRoutingBox_.addItem("Auto mono: strongest input", 1);
        inputRoutingBox_.addItem("Input 1 / left", 2);
        inputRoutingBox_.addItem("Input 2 / right", 3);
        inputRoutingBox_.addItem("Independent stereo", 4);
        inputRoutingBox_.addItem("Test signal (sine)", 5);
        inputRoutingBox_.setSelectedId(1, juce::dontSendNotification);

        signalRoutingBox_.addItem("Serial guitar rig", 1);
        signalRoutingBox_.addItem("Parallel octave + bass amp", 2);
        signalRoutingBox_.addItem("Crossover octave + bass amp", 3);
        signalRoutingBox_.setSelectedId(1, juce::dontSendNotification);

        powerTubeBox_.addItem("Power tubes: EL34", 1);
        powerTubeBox_.addItem("Power tubes: 6L6GC", 2);
        powerTubeBox_.addItem("Power tubes: KT88", 3);
        powerTubeBox_.setSelectedId(1, juce::dontSendNotification);
        toneStackBox_.addItem("Tone stack: Reference", 1);
        toneStackBox_.addItem("Tone stack: British", 2);
        toneStackBox_.addItem("Tone stack: American", 3);
        toneStackBox_.setSelectedId(1, juce::dontSendNotification);
        toneDriverBox_.addItem("Driver: Reference", 1);
        toneDriverBox_.addItem("Driver: Cathode follower", 2);
        toneDriverBox_.addItem("Driver: Plate driven", 3);
        toneDriverBox_.setSelectedId(1, juce::dontSendNotification);
        feedbackVoicingBox_.addItem("Feedback: Reference", 1);
        feedbackVoicingBox_.addItem("Feedback: British", 2);
        feedbackVoicingBox_.addItem("Feedback: American", 3);
        feedbackVoicingBox_.setSelectedId(1, juce::dontSendNotification);

        ampEnabled_.setButtonText("Amp");
        ampEnabled_.setToggleState(true, juce::dontSendNotification);
        cabEnabled_.setButtonText("Speaker + Cab IR");
        cabEnabled_.setToggleState(true, juce::dontSendNotification);
        octaveEnabled_.setButtonText("Octave -1");
        octaveEnabled_.setToggleState(true, juce::dontSendNotification);
        bassCabinetEnabled_.setButtonText("Bass cabinet IR");
        bassCabinetEnabled_.setToggleState(true, juce::dontSendNotification);
        safeDry_.setButtonText("Safe dry monitor");
        safeDry_.setToggleState(true, juce::dontSendNotification);
        mute_.setButtonText("Mute output");
        mute_.setToggleState(true, juce::dontSendNotification);
        matchIrLevel_.setButtonText("Match measured IR loudness");
        matchIrLevel_.setToggleState(true, juce::dontSendNotification);
        audioSettingsButton_.setButtonText("AUDIO SETTINGS");
        audioSettingsButton_.setClickingTogglesState(true);

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

        testSignalLabel_.setText("TEST SIGNAL", juce::dontSendNotification);
        testSignalFrequency_.setSliderStyle(juce::Slider::LinearHorizontal);
        testSignalFrequency_.setTextBoxStyle(juce::Slider::TextBoxRight, false, 66, 22);
        testSignalFrequency_.setRange(20.0, 5000.0, 1.0);
        testSignalFrequency_.setSkewFactorFromMidPoint(440.0);
        testSignalFrequency_.setValue(440.0, juce::dontSendNotification);
        testSignalFrequency_.setTextValueSuffix(" Hz");
        testSignalFrequency_.onValueChange = [this] {
            engine_.setTestSignalFrequencyHz(static_cast<float>(testSignalFrequency_.getValue()));
        };
        testSignalLevel_.setSliderStyle(juce::Slider::LinearHorizontal);
        testSignalLevel_.setTextBoxStyle(juce::Slider::TextBoxRight, false, 56, 22);
        testSignalLevel_.setRange(0.0, 100.0, 1.0);
        testSignalLevel_.setValue(50.0, juce::dontSendNotification);
        testSignalLevel_.setTextValueSuffix("% lvl");
        testSignalLevel_.onValueChange = [this] {
            engine_.setTestSignalAmplitude(static_cast<float>(testSignalLevel_.getValue() * 0.01));
        };

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
        configureToneControl(cabinetMix_, "IR MIX", settings_.cabinetMix);
        configureToneControl(speakerCompression_, "COMP", settings_.speakerCompression);
        configureToneControl(speakerExcursion_, "EXCURS", settings_.speakerExcursion);
        configureToneControl(speakerResonance_, "RESON", settings_.speakerResonance);
        configureToneControl(guitarBranchLevel_, "GUITAR", settings_.guitarBranchLevel);
        configureToneControl(bassBranchLevel_, "BASS MIX", settings_.bassBranchLevel);
        configureToneControl(octaveMix_, "OCT MIX", settings_.octaveMix);
        configureToneControl(octaveLevel_, "OCT LVL", settings_.octaveLevel);
        configureToneControl(bassGain_, "BASS DRV", settings_.bassGain);
        configureToneControl(bassTone_, "BASS TONE", settings_.bassTone);
        configureToneControl(bassLevel_, "BASS OUT", settings_.bassLevel);

        auto configureDbControl = [this](juce::Slider& slider, const juce::String& name,
                                         double minimum, double maximum, double value) {
            addAndMakeVisible(slider);
            slider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
            slider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 118, 22);
            slider.setRange(minimum, maximum, 0.1);
            slider.setValue(value, juce::dontSendNotification);
            slider.setTextValueSuffix(" dB " + name);
            slider.onValueChange = [this] { toneControlsPending_ = true; };
        };
        configureDbControl(cabinetOutput_, "CAB", -18.0, 12.0,
                           settings_.cabinetOutputDb);
        configureDbControl(ampOutput_, "AMP", -30.0, 6.0,
                           settings_.ampOutputDb);

        auto configureFrequencyControl = [this](juce::Slider& slider,
                                                 const juce::String& name,
                                                 double minimum,
                                                 double maximum,
                                                 double midpoint,
                                                 double value) {
            addAndMakeVisible(slider);
            slider.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
            slider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 132, 22);
            slider.setRange(minimum, maximum, 1.0);
            slider.setSkewFactorFromMidPoint(midpoint);
            slider.setValue(value, juce::dontSendNotification);
            slider.setTextValueSuffix(" Hz " + name);
            slider.onValueChange = [this] { toneControlsPending_ = true; };
        };
        configureFrequencyControl(cabinetLowCut_, "LOW CUT", 35.0, 240.0,
                                  95.0, settings_.cabinetLowCutHz);
        configureFrequencyControl(cabinetHighCut_, "HIGH CUT", 1800.0, 16000.0,
                                  6500.0, settings_.cabinetHighCutHz);

        addAndMakeVisible(crossoverFrequency_);
        crossoverFrequency_.setSliderStyle(juce::Slider::RotaryHorizontalVerticalDrag);
        crossoverFrequency_.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 118, 22);
        crossoverFrequency_.setRange(40.0, 1200.0, 1.0);
        crossoverFrequency_.setSkewFactorFromMidPoint(240.0);
        crossoverFrequency_.setValue(settings_.crossoverFrequency,
                                    juce::dontSendNotification);
        crossoverFrequency_.setTextValueSuffix(" Hz X-OVER");
        crossoverFrequency_.onValueChange = [this] { toneControlsPending_ = true; };

        rigPageButton_.setButtonText("01   CIRCUIT PEDAL");
        advancedPageButton_.setButtonText("02   GUITAR AMPLIFIER");
        cabinetPageButton_.setButtonText("03   SPEAKER + CABINET");
        routingPageButton_.setButtonText("04   ROUTING + BASS");
        presetsPageButton_.setButtonText("05   PRESETS");

        slotAButton_.setButtonText("A");
        slotBButton_.setButtonText("B");
        presetNameEditor_.setTextToShowWhenEmpty("preset name", juce::Colours::grey);
        loadPresetButton_.setButtonText("LOAD");
        deletePresetButton_.setButtonText("DELETE");
        savePresetButton_.setButtonText("SAVE AS");
        presetStatusLabel_.setText(
            "Presets are stored in " + presetDirectoryDescription(), juce::dontSendNotification);
        presetStatusLabel_.setColour(juce::Label::textColourId, juce::Colours::lightgrey);

        pedalControlsTitle_.setText("SIGNAL CHAIN", juce::dontSendNotification);
        ampControlsTitle_.setText("CIRCUIT PEDAL", juce::dontSendNotification);
        pedalControlsTitle_.setFont(juce::FontOptions(16.0f, juce::Font::bold));
        ampControlsTitle_.setFont(juce::FontOptions(18.0f, juce::Font::bold));
        loadIrButton_.setButtonText("LOAD MEASURED IR");
        loadBassIrButton_.setButtonText("LOAD MEASURED BASS IR");
        resetDiagnosticsButton_.setButtonText("RESET METERS");
        statusLabel_.setText("Audio device not started", juce::dontSendNotification);
        irLabel_.setText("BUILT-IN REFERENCE / calibrated guitar cabinet / not measured",
                         juce::dontSendNotification);
        bassIrLabel_.setText("BUILT-IN REFERENCE / calibrated bass cabinet / not measured",
                             juce::dontSendNotification);
        meterLabel_.setText("Input: -inf dBFS    Output: -inf dBFS",
                            juce::dontSendNotification);
        thdLabel_.setText("THD: -- (select \"Test signal (sine)\" input routing to measure)",
                          juce::dontSendNotification);
        for (auto* waveform : {&inputWaveform_, &outputWaveform_}) {
            waveform->setColours(juce::Colour::fromRGB(20, 29, 34),
                                 juce::Colours::lightseagreen);
            waveform->setBufferSize(512);
            waveform->setSamplesPerBlock(256);
        }
        for (auto* spectrum : {&inputSpectrum_, &outputSpectrum_}) {
            spectrum->setColours(juce::Colour::fromRGB(20, 29, 34),
                                 juce::Colours::lightseagreen);
            spectrum->setVisible(false);
        }
        spectrumToggle_.setButtonText("SPECTRUM");
        spectrumToggle_.setClickingTogglesState(true);
        spectrumToggle_.onClick = [this] {
            showSpectrum_ = spectrumToggle_.getToggleState();
            inputWaveform_.setVisible(!showSpectrum_);
            outputWaveform_.setVisible(!showSpectrum_);
            inputSpectrum_.setVisible(showSpectrum_);
            outputSpectrum_.setVisible(showSpectrum_);
        };
        inputMonitorTapBox_.onChange = [this] {
            const int index = inputMonitorTapBox_.getSelectedItemIndex();
            if (index >= 0 && index < static_cast<int>(inputMonitorTapOptions_.size()))
                inputMonitorTap_.store(inputMonitorTapOptions_[static_cast<std::size_t>(index)],
                                       std::memory_order_relaxed);
        };
        outputMonitorTapBox_.onChange = [this] {
            const int index = outputMonitorTapBox_.getSelectedItemIndex();
            if (index >= 0 && index < static_cast<int>(outputMonitorTapOptions_.size()))
                outputMonitorTap_.store(outputMonitorTapOptions_[static_cast<std::size_t>(index)],
                                        std::memory_order_relaxed);
        };

        pedalBox_.onChange = [this] {
            updateSettingsFromControls();
            setControlPage(currentPage_);
            rebuildRig();
        };
        ampBox_.onChange = [this] {
            updateSettingsFromControls();
            updateAdvancedControlAvailability();
            setControlPage(currentPage_);
            rebuildRig();
        };
        qualityBox_.onChange = [this] { updateSettingsFromControls(); rebuildRig(); };
        inputRoutingBox_.onChange = [this] { updateInputRouting(); };
        ampEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        cabEnabled_.onClick = [this] { updateSettingsFromControls(); rebuildRig(); };
        signalRoutingBox_.onChange = [this] {
            updateSettingsFromControls();
            updateRoutingGraph();
            rebuildRig();
        };
        octaveEnabled_.onClick = [this] {
            updateSettingsFromControls();
            updateRoutingGraph();
            rebuildRig();
        };
        bassCabinetEnabled_.onClick = [this] {
            updateSettingsFromControls();
            updateRoutingGraph();
            rebuildRig();
        };
        powerTubeBox_.onChange = [this] { toneControlsPending_ = true; };
        toneStackBox_.onChange = [this] { toneControlsPending_ = true; };
        toneDriverBox_.onChange = [this] { toneControlsPending_ = true; };
        feedbackVoicingBox_.onChange = [this] { toneControlsPending_ = true; };
        safeDry_.onClick = [this] { updateMonitorTapOptions(); rebuildRig(); };
        mute_.onClick = [this] { engine_.setMuted(mute_.getToggleState()); };
        matchIrLevel_.onClick = [this] {
            settings_.matchMeasuredCabinetLevel = matchIrLevel_.getToggleState();
            updateImpulseLabel(loadedGuitarIr_, irLabel_,
                               "BUILT-IN REFERENCE / calibrated guitar cabinet / not measured");
            updateImpulseLabel(loadedBassIr_, bassIrLabel_,
                               "BUILT-IN REFERENCE / calibrated bass cabinet / not measured");
            rebuildRig();
        };
        audioSettingsButton_.onClick = [this] {
            const bool expanded = audioSettingsButton_.getToggleState();
            deviceSelector_.setVisible(expanded);
            audioSettingsButton_.setButtonText(expanded
                ? "CLOSE AUDIO SETTINGS" : "AUDIO SETTINGS");
            resized();
            repaint();
        };
        inputTrim_.onValueChange = [this] {
            engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        };
        outputTrim_.onValueChange = [this] {
            engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        };
        loadIrButton_.onClick = [this] { chooseImpulseResponse(); };
        loadBassIrButton_.onClick = [this] { chooseBassImpulseResponse(); };
        resetDiagnosticsButton_.onClick = [this] {
            engine_.resetDiagnostics();
            xRunBaseline_ = std::max(0, deviceManager_.getXRunCount());
        };
        rigPageButton_.onClick = [this] { setControlPage(ControlPage::pedal); };
        cabinetPageButton_.onClick = [this] { setControlPage(ControlPage::cabinet); };
        routingPageButton_.onClick = [this] { setControlPage(ControlPage::routing); };
        advancedPageButton_.onClick = [this] { setControlPage(ControlPage::amplifier); };
        presetsPageButton_.onClick = [this] { setControlPage(ControlPage::presets); };

        slotAButton_.onClick = [this] { selectRigSlot(true); };
        slotBButton_.onClick = [this] { selectRigSlot(false); };

        presetBox_.onChange = [this] {
            const bool has = presetBox_.getSelectedId() > 0;
            loadPresetButton_.setEnabled(has);
            deletePresetButton_.setEnabled(has);
            if (has) presetNameEditor_.setText(presetBox_.getText(), juce::dontSendNotification);
        };
        savePresetButton_.onClick = [this] { onSavePresetClicked(); };
        loadPresetButton_.onClick = [this] { onLoadPresetClicked(); };
        deletePresetButton_.onClick = [this] { onDeletePresetClicked(); };

        engine_.setInputTrimDb(0.0f);
        engine_.setOutputTrimDb(-12.0f);
        engine_.setSafetyCeiling(0.98f);
        engine_.setMuted(true);
        updateSettingsFromControls();
        updateInputRouting();
        updateRoutingGraph();
        updateAdvancedControlAvailability();
        updateSlotButtonAppearance();
        refreshPresetList();
        setControlPage(ControlPage::pedal);

        const auto stateFile = audioStateFile();
        auto savedState = stateFile.existsAsFile()
            ? juce::XmlDocument::parse(stateFile)
            : std::unique_ptr<juce::XmlElement>{};
        deviceManager_.addChangeListener(this);
        const auto error = deviceManager_.initialise(2, 2, savedState.get(), true, "*WAVIO*");
        if (error.isNotEmpty())
            statusLabel_.setText("Audio init error: " + error, juce::dontSendNotification);
        enforceMinimumBufferSize();
        deviceManager_.addAudioCallback(this);

        setSize(1240, 900);
        startTimerHz(20);
    }

    ~MainComponent() override {
        stopTimer();
        persistAudioDeviceState();
        deviceManager_.removeChangeListener(this);
        deviceManager_.removeAudioCallback(this);
        engine_.collectRetired();
    }

    void paint(juce::Graphics& graphics) override {
        graphics.fillAll(juce::Colour::fromRGB(20, 29, 34));
        graphics.setColour(juce::Colour::fromRGB(29, 42, 48));
        graphics.fillRoundedRectangle(chainPanel_.toFloat(), 11.0f);
        graphics.fillRoundedRectangle(inspectorPanel_.toFloat(), 11.0f);
        graphics.setColour(juce::Colour::fromRGB(78, 188, 180).withAlpha(0.34f));
        graphics.drawRoundedRectangle(chainPanel_.toFloat(), 11.0f, 1.0f);
        graphics.drawRoundedRectangle(inspectorPanel_.toFloat(), 11.0f, 1.0f);

        auto guide = chainPanel_.reduced(16);
        guide.removeFromTop(368);
        if (guide.getHeight() > 55) {
            graphics.setColour(juce::Colours::white.withAlpha(0.55f));
            graphics.setFont(13.0f);
            graphics.drawFittedText(
                "Select a signal block to edit its full controls.\n"
                "Routing can split guitar and octave/bass paths.",
                guide.removeFromTop(75), juce::Justification::topLeft, 4);
        }
    }

    void resized() override {
        auto area = getLocalBounds().reduced(14);
        auto row = area.removeFromTop(36);
        audioSettingsButton_.setBounds(row.removeFromRight(190).reduced(0, 3));
        row.removeFromRight(12);
        slotBButton_.setBounds(row.removeFromRight(40).reduced(0, 3));
        row.removeFromRight(4);
        slotAButton_.setBounds(row.removeFromRight(40).reduced(0, 3));
        row.removeFromRight(12);
        statusLabel_.setBounds(row);

        if (deviceSelector_.isVisible()) {
            deviceSelector_.setBounds(area.removeFromTop(260));
            area.removeFromTop(8);
        }

        row = area.removeFromTop(36);
        const int selectorWidth = std::max(160, row.getWidth() / 5);
        pedalBox_.setBounds(row.removeFromLeft(selectorWidth).reduced(0, 2));
        row.removeFromLeft(8);
        ampBox_.setBounds(row.removeFromLeft(selectorWidth + 22).reduced(0, 2));
        row.removeFromLeft(8);
        qualityBox_.setBounds(row.removeFromLeft(selectorWidth + 12).reduced(0, 2));
        row.removeFromLeft(8);
        inputRoutingBox_.setBounds(row.reduced(0, 2));

        area.removeFromTop(5);
        row = area.removeFromTop(32);
        safeDry_.setBounds(row.removeFromLeft(180));
        ampEnabled_.setBounds(row.removeFromLeft(90));
        cabEnabled_.setBounds(row.removeFromLeft(190));
        mute_.setBounds(row.removeFromLeft(140));
        resetDiagnosticsButton_.setBounds(row.removeFromRight(158).reduced(0, 2));
        row.removeFromLeft(10);
        if (row.getWidth() > 260) {
            testSignalLabel_.setBounds(row.removeFromLeft(84));
            testSignalFrequency_.setBounds(row.removeFromLeft(std::min(180, row.getWidth() / 2)));
            row.removeFromLeft(8);
            testSignalLevel_.setBounds(row);
        } else {
            testSignalLabel_.setBounds({});
            testSignalFrequency_.setBounds({});
            testSignalLevel_.setBounds({});
        }
        area.removeFromTop(10);

        auto waveformArea = area.removeFromTop(96);
        auto inputWaveArea = waveformArea.removeFromLeft(waveformArea.getWidth() / 2 - 6);
        waveformArea.removeFromLeft(12);
        auto& outputWaveArea = waveformArea;
        inputMonitorTapBox_.setBounds(inputWaveArea.removeFromTop(18));
        inputWaveform_.setBounds(inputWaveArea);
        inputSpectrum_.setBounds(inputWaveArea);
        auto outputWaveLabelRow = outputWaveArea.removeFromTop(18);
        spectrumToggle_.setBounds(outputWaveLabelRow.removeFromRight(110).reduced(0, 1));
        outputMonitorTapBox_.setBounds(outputWaveLabelRow);
        outputWaveform_.setBounds(outputWaveArea);
        outputSpectrum_.setBounds(outputWaveArea);
        area.removeFromTop(10);

        auto diagnostics = area.removeFromBottom(178);
        row = diagnostics.removeFromTop(34);
        inputTrim_.setBounds(row.removeFromLeft(row.getWidth() / 2 - 8));
        row.removeFromLeft(16);
        outputTrim_.setBounds(row);
        diagnostics.removeFromTop(5);
        routingLabel_.setBounds(diagnostics.removeFromTop(23));
        meterLabel_.setBounds(diagnostics.removeFromTop(23));
        performanceLabel_.setBounds(diagnostics.removeFromTop(23));
        latencyLabel_.setBounds(diagnostics.removeFromTop(23));
        safetyLabel_.setBounds(diagnostics.removeFromTop(23));
        thdLabel_.setBounds(diagnostics.removeFromTop(23));

        area.removeFromBottom(10);
        const int chainWidth = std::clamp(area.getWidth() / 4, 238, 292);
        chainPanel_ = area.removeFromLeft(chainWidth);
        area.removeFromLeft(12);
        inspectorPanel_ = area;

        auto chain = chainPanel_.reduced(13);
        pedalControlsTitle_.setBounds(chain.removeFromTop(40));
        chain.removeFromTop(8);
        rigPageButton_.setBounds(chain.removeFromTop(54));
        chain.removeFromTop(9);
        advancedPageButton_.setBounds(chain.removeFromTop(54));
        chain.removeFromTop(9);
        cabinetPageButton_.setBounds(chain.removeFromTop(54));
        chain.removeFromTop(9);
        routingPageButton_.setBounds(chain.removeFromTop(54));
        chain.removeFromTop(9);
        presetsPageButton_.setBounds(chain.removeFromTop(54));

        auto panel = inspectorPanel_.reduced(18, 12);
        ampControlsTitle_.setBounds(panel.removeFromTop(40));
        panel.removeFromTop(5);

        const auto placeKnobs = [](juce::Rectangle<int> knobRow,
                                   std::initializer_list<juce::Slider*> sliders) {
            const int count = static_cast<int>(sliders.size());
            if (count <= 0) return;
            const int width = knobRow.getWidth() / count;
            for (auto* slider : sliders)
                slider->setBounds(knobRow.removeFromLeft(width).reduced(3, 0));
        };

        if (currentPage_ == ControlPage::pedal) {
            placeKnobs(panel.removeFromTop(std::min(180, panel.getHeight())),
                       {&pedalDrive_, &pedalTone_, &pedalLevel_});
        } else if (currentPage_ == ControlPage::amplifier) {
            if (selectedAmpIsCircuitLevel()) {
                placeKnobs(panel.removeFromTop(std::min(180, panel.getHeight())),
                           {&ampGain_, &ampBass_, &ampTreble_});
            } else {
                const int knobHeight = std::clamp((panel.getHeight() - 105) / 2,
                                                   86, 155);
                placeKnobs(panel.removeFromTop(knobHeight),
                           {&ampGain_, &ampBass_, &ampMid_, &ampTreble_});
                placeKnobs(panel.removeFromTop(knobHeight),
                           {&ampMaster_, &ampPresence_, &ampOutput_});
                panel.removeFromTop(8);
                row = panel.removeFromTop(32);
                powerTubeBox_.setBounds(row.removeFromLeft(row.getWidth() / 2).reduced(3, 0));
                toneStackBox_.setBounds(row.reduced(3, 0));
                panel.removeFromTop(6);
                row = panel.removeFromTop(32);
                toneDriverBox_.setBounds(row.removeFromLeft(row.getWidth() / 2).reduced(3, 0));
                feedbackVoicingBox_.setBounds(row.reduced(3, 0));
            }
        } else if (currentPage_ == ControlPage::cabinet) {
            const int knobHeight = std::clamp((panel.getHeight() - 106) / 2,
                                               86, 158);
            placeKnobs(panel.removeFromTop(knobHeight),
                       {&cabinetMix_, &cabinetLowCut_, &cabinetHighCut_,
                        &speakerResonance_});
            placeKnobs(panel.removeFromTop(knobHeight),
                       {&speakerCompression_, &speakerExcursion_, &cabinetOutput_});
            panel.removeFromTop(8);
            row = panel.removeFromTop(34);
            loadIrButton_.setBounds(row.removeFromLeft(220));
            row.removeFromLeft(12);
            matchIrLevel_.setBounds(row);
            irLabel_.setBounds(panel.removeFromTop(34));
        } else if (currentPage_ == ControlPage::routing) {
            row = panel.removeFromTop(34);
            signalRoutingBox_.setBounds(row.removeFromLeft(std::min(320, row.getWidth() / 2)));
            row.removeFromLeft(8);
            octaveEnabled_.setBounds(row.removeFromLeft(130));
            bassCabinetEnabled_.setBounds(row);
            panel.removeFromTop(5);
            routingGraphView_.setBounds(panel.removeFromTop(
                std::clamp(panel.getHeight() / 4, 72, 105)));
            panel.removeFromTop(5);
            const int knobHeight = std::clamp((panel.getHeight() - 46) / 2, 78, 148);
            placeKnobs(panel.removeFromTop(knobHeight),
                       {&guitarBranchLevel_, &bassBranchLevel_, &octaveMix_,
                        &octaveLevel_});
            placeKnobs(panel.removeFromTop(knobHeight),
                       {&bassGain_, &bassTone_, &bassLevel_, &crossoverFrequency_});
            panel.removeFromTop(6);
            row = panel.removeFromTop(34);
            loadBassIrButton_.setBounds(row.removeFromLeft(220));
            row.removeFromLeft(12);
            bassIrLabel_.setBounds(row);
        } else {
            row = panel.removeFromTop(34);
            presetBox_.setBounds(row.removeFromLeft(std::min(320, row.getWidth() / 2)));
            row.removeFromLeft(8);
            loadPresetButton_.setBounds(row.removeFromLeft(110));
            row.removeFromLeft(8);
            deletePresetButton_.setBounds(row);
            panel.removeFromTop(10);
            row = panel.removeFromTop(34);
            presetNameEditor_.setBounds(row.removeFromLeft(std::max(200, row.getWidth() - 150)));
            row.removeFromLeft(8);
            savePresetButton_.setBounds(row);
            panel.removeFromTop(10);
            presetStatusLabel_.setBounds(panel.removeFromTop(60));
        }
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
        // Sized for a quarter second of headroom at the device sample rate so a
        // stalled UI thread cannot make push() block or allocate; prepare() runs
        // here, on the message thread, strictly before the callback below can push.
        const int waveformCapacity = std::max(8192,
            static_cast<int>(currentSampleRate_ * 0.25));
        inputTapFifo_.prepare(waveformCapacity);
        outputTapFifo_.prepare(waveformCapacity);
        // Sized to the device's own block size, which bounds numSamples for
        // every audioDeviceIOCallbackWithContext() call below; prepared here
        // on the message thread, before the callback that writes into it.
        monitorTapBufferA_.assign(static_cast<std::size_t>(std::max(1, currentBlockSize_)), 0.0f);
        monitorTapBufferB_.assign(static_cast<std::size_t>(std::max(1, currentBlockSize_)), 0.0f);
        inputSpectrum_.setSampleRate(currentSampleRate_);
        outputSpectrum_.setSampleRate(currentSampleRate_);
        inputSpectrum_.resetAnalysis();
        outputSpectrum_.resetAnalysis();
        outputThd_.setSampleRate(currentSampleRate_);
        outputThd_.reset();
        engine_.setInputTrimDb(static_cast<float>(inputTrim_.getValue()));
        engine_.setOutputTrimDb(static_cast<float>(outputTrim_.getValue()));
        engine_.setMuted(mute_.getToggleState());
        updateInputRouting();

        const juce::String message = ok
            ? "Audio ready: " + device->getName() + " / "
                + juce::String(currentSampleRate_, 0) + " Hz / "
                + juce::String(currentBlockSize_) + " samples / graph latency "
                + juce::String(engine_.stats().graphLatencySamples) + " samples"
                + (mute_.getToggleState() || testSignalActive_.load(std::memory_order_relaxed)
                       ? " / OUTPUT MUTED" : "")
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
        // monitorTapBufferA_/monitorTapBufferB_ are sized to currentBlockSize_
        // in audioDeviceAboutToStart(), which bounds numSamples here; only
        // ever touched from this callback.
        const bool monitorBuffersFit = static_cast<std::size_t>(numSamples)
            <= monitorTapBufferA_.size() && static_cast<std::size_t>(numSamples)
            <= monitorTapBufferB_.size();
        float* monitorTapA = monitorBuffersFit ? monitorTapBufferA_.data() : nullptr;
        float* monitorTapB = monitorBuffersFit ? monitorTapBufferB_.data() : nullptr;
        engine_.process(inputChannelData, numInputChannels,
                        outputChannelData, numOutputChannels, numSamples,
                        inputMonitorTap_.load(std::memory_order_relaxed), monitorTapA,
                        outputMonitorTap_.load(std::memory_order_relaxed), monitorTapB);

        // Lock-free hand-off to the UI thread for the two monitor windows'
        // waveform/spectrum displays -- each window shows whatever tap point
        // its dropdown currently selects (see RealtimeAudioEngine::process()'s
        // MonitorTapPoint doc comment for exactly what each point reads). No
        // allocation, no lock (see AudioTapFifo.h).
        if (monitorTapA != nullptr) inputTapFifo_.push(monitorTapA, numSamples);
        if (monitorTapB != nullptr) outputTapFifo_.push(monitorTapB, numSamples);
    }

private:
    enum class ControlPage { pedal, amplifier, cabinet, routing, presets };

    void setControlPage(ControlPage page) {
        currentPage_ = page;
        const bool pedal = page == ControlPage::pedal;
        const bool amplifier = page == ControlPage::amplifier;
        const bool cabinet = page == ControlPage::cabinet;
        const bool routing = page == ControlPage::routing;
        const bool presets = page == ControlPage::presets;

        const juce::String title = pedal ? "CIRCUIT PEDAL  /  " + pedalBox_.getText()
            : amplifier ? "GUITAR AMPLIFIER  /  " + ampBox_.getText()
            : cabinet ? "SPEAKER + CABINET RESPONSE"
            : routing ? "PARALLEL ROUTING + BASS AMP"
                      : "PRESETS  /  slot " + juce::String(abActiveIsA_ ? "A" : "B") + " active";
        ampControlsTitle_.setText(title, juce::dontSendNotification);

        for (auto* control : std::array<juce::Component*, 3>{{
                 &pedalDrive_, &pedalTone_, &pedalLevel_}})
            control->setVisible(pedal);

        // PreampCircuitNode/FullAmpCircuitNode only expose Drive/Bass/Treble
        // (see selectedAmpIsCircuitLevel()); the rest of the parameterized
        // amp-family controls don't apply to them and stay hidden.
        const bool circuitLevelAmp = amplifier && selectedAmpIsCircuitLevel();
        for (auto* control : std::array<juce::Component*, 3>{{
                 &ampGain_, &ampBass_, &ampTreble_}})
            control->setVisible(amplifier);
        for (auto* control : std::array<juce::Component*, 8>{{
                 &ampMid_, &ampMaster_, &ampPresence_, &ampOutput_,
                 &powerTubeBox_, &toneStackBox_, &toneDriverBox_,
                 &feedbackVoicingBox_}})
            control->setVisible(amplifier && !circuitLevelAmp);

        for (auto* control : std::array<juce::Component*, 10>{{
                 &cabinetMix_, &cabinetLowCut_, &cabinetHighCut_,
                 &speakerCompression_, &speakerExcursion_, &speakerResonance_,
                 &cabinetOutput_, &loadIrButton_, &matchIrLevel_, &irLabel_}})
            control->setVisible(cabinet);

        for (auto* control : std::array<juce::Component*, 14>{{
                 &signalRoutingBox_, &octaveEnabled_, &bassCabinetEnabled_,
                 &routingGraphView_, &guitarBranchLevel_, &bassBranchLevel_,
                 &octaveMix_, &octaveLevel_, &bassGain_, &bassTone_, &bassLevel_,
                 &crossoverFrequency_, &loadBassIrButton_, &bassIrLabel_}})
            control->setVisible(routing);

        for (auto* control : std::array<juce::Component*, 6>{{
                 &presetBox_, &loadPresetButton_, &deletePresetButton_,
                 &presetNameEditor_, &savePresetButton_, &presetStatusLabel_}})
            control->setVisible(presets);

        rigPageButton_.setColour(juce::TextButton::buttonColourId,
            pedal ? juce::Colour::fromRGB(43, 119, 134)
                  : juce::Colour::fromRGB(43, 55, 61));
        advancedPageButton_.setColour(juce::TextButton::buttonColourId,
            amplifier ? juce::Colour::fromRGB(43, 119, 134)
                      : juce::Colour::fromRGB(43, 55, 61));
        cabinetPageButton_.setColour(juce::TextButton::buttonColourId,
            cabinet ? juce::Colour::fromRGB(43, 119, 134)
                    : juce::Colour::fromRGB(43, 55, 61));
        routingPageButton_.setColour(juce::TextButton::buttonColourId,
            routing ? juce::Colour::fromRGB(43, 119, 134)
                    : juce::Colour::fromRGB(43, 55, 61));
        presetsPageButton_.setColour(juce::TextButton::buttonColourId,
            presets ? juce::Colour::fromRGB(43, 119, 134)
                    : juce::Colour::fromRGB(43, 55, 61));
        if (presets) refreshPresetList();
        if (getWidth() > 0 && getHeight() > 0) resized();
        repaint();
    }

    void updateAdvancedControlAvailability() {
        const bool reference = settings_.amp == guitardsp::app::AmpModel::reference;
        powerTubeBox_.setEnabled(reference);
        toneStackBox_.setEnabled(reference);
        toneDriverBox_.setEnabled(reference);
        feedbackVoicingBox_.setEnabled(reference);
    }

    [[nodiscard]] bool selectedAmpIsCircuitLevel() const noexcept {
        return settings_.amp == guitardsp::app::AmpModel::preampCircuit
            || settings_.amp == guitardsp::app::AmpModel::fullAmpCircuit;
    }

    void updateRoutingGraph() {
        routingGraphView_.setState(settings_.signalRouting,
                                   settings_.octaveEnabled,
                                   settings_.bassCabinetEnabled);
        const bool parallel = settings_.signalRouting
            != guitardsp::app::SignalRouting::serialGuitar;
        octaveEnabled_.setEnabled(parallel);
        bassCabinetEnabled_.setEnabled(parallel);
        for (auto* slider : std::array<juce::Slider*, 7>{{
                 &guitarBranchLevel_, &bassBranchLevel_, &octaveMix_,
                 &octaveLevel_, &bassGain_, &bassTone_, &bassLevel_}})
            slider->setEnabled(parallel);
        crossoverFrequency_.setEnabled(
            settings_.signalRouting == guitardsp::app::SignalRouting::crossoverOctaveBass);
    }

    [[nodiscard]] juce::String presetDirectoryDescription() const {
        return juce::String(presetStore_.directory());
    }

    // Pushes settings_ (already updated by the caller -- a preset load or an
    // A/B slot switch) onto every UI control that mirrors it, without
    // triggering each control's own onChange/onClick handler (which would
    // otherwise fire a full rig rebuild once per widget). A single rebuild
    // happens at the end instead. Mirrors the constructor's own
    // dontSendNotification + one explicit updateSettingsFromControls()
    // pattern.
    void refreshControlsFromSettings() {
        using guitardsp::app::AmpModel;
        using guitardsp::app::PedalModel;
        using guitardsp::app::SignalRouting;

        pedalBox_.setSelectedId(
            settings_.pedal == PedalModel::bypass ? 1
                : settings_.pedal == PedalModel::ds1Circuit ? 3 : 2,
            juce::dontSendNotification);
        ampBox_.setSelectedId(
            settings_.amp == AmpModel::britishPlexiFamily ? 2
                : settings_.amp == AmpModel::americanCleanFamily ? 3
                : settings_.amp == AmpModel::preampCircuit ? 4
                : settings_.amp == AmpModel::fullAmpCircuit ? 5 : 1,
            juce::dontSendNotification);
        qualityBox_.setSelectedId(
            settings_.quality == guitardsp::graph::ProcessingQuality::eco ? 1
                : settings_.quality == guitardsp::graph::ProcessingQuality::live ? 2
                : settings_.quality == guitardsp::graph::ProcessingQuality::studio ? 4 : 3,
            juce::dontSendNotification);
        ampEnabled_.setToggleState(settings_.ampEnabled, juce::dontSendNotification);
        cabEnabled_.setToggleState(settings_.cabinetEnabled, juce::dontSendNotification);
        signalRoutingBox_.setSelectedId(
            settings_.signalRouting == SignalRouting::parallelOctaveBass ? 2
                : settings_.signalRouting == SignalRouting::crossoverOctaveBass ? 3 : 1,
            juce::dontSendNotification);
        octaveEnabled_.setToggleState(settings_.octaveEnabled, juce::dontSendNotification);
        bassCabinetEnabled_.setToggleState(settings_.bassCabinetEnabled, juce::dontSendNotification);
        matchIrLevel_.setToggleState(settings_.matchMeasuredCabinetLevel, juce::dontSendNotification);
        powerTubeBox_.setSelectedId(static_cast<int>(settings_.ampPowerTube) + 1, juce::dontSendNotification);
        toneStackBox_.setSelectedId(static_cast<int>(settings_.ampToneStack) + 1, juce::dontSendNotification);
        toneDriverBox_.setSelectedId(static_cast<int>(settings_.ampToneDriver) + 1, juce::dontSendNotification);
        feedbackVoicingBox_.setSelectedId(
            static_cast<int>(settings_.ampFeedbackVoicing) + 1, juce::dontSendNotification);

        const auto setPercent = [](juce::Slider& slider, float value) {
            slider.setValue(static_cast<double>(value) * 100.0, juce::dontSendNotification);
        };
        setPercent(pedalDrive_, settings_.pedalDrive);
        setPercent(pedalTone_, settings_.pedalTone);
        setPercent(pedalLevel_, settings_.pedalLevel);
        setPercent(ampGain_, settings_.ampGain);
        setPercent(ampBass_, settings_.ampBass);
        setPercent(ampMid_, settings_.ampMid);
        setPercent(ampTreble_, settings_.ampTreble);
        setPercent(ampMaster_, settings_.ampMaster);
        setPercent(ampPresence_, settings_.ampPresence);
        setPercent(cabinetMix_, settings_.cabinetMix);
        setPercent(speakerCompression_, settings_.speakerCompression);
        setPercent(speakerExcursion_, settings_.speakerExcursion);
        setPercent(speakerResonance_, settings_.speakerResonance);
        setPercent(guitarBranchLevel_, settings_.guitarBranchLevel);
        setPercent(bassBranchLevel_, settings_.bassBranchLevel);
        setPercent(octaveMix_, settings_.octaveMix);
        setPercent(octaveLevel_, settings_.octaveLevel);
        setPercent(bassGain_, settings_.bassGain);
        setPercent(bassTone_, settings_.bassTone);
        setPercent(bassLevel_, settings_.bassLevel);
        cabinetOutput_.setValue(settings_.cabinetOutputDb, juce::dontSendNotification);
        ampOutput_.setValue(settings_.ampOutputDb, juce::dontSendNotification);
        cabinetLowCut_.setValue(settings_.cabinetLowCutHz, juce::dontSendNotification);
        cabinetHighCut_.setValue(settings_.cabinetHighCutHz, juce::dontSendNotification);
        crossoverFrequency_.setValue(settings_.crossoverFrequency, juce::dontSendNotification);

        toneControlsPending_ = false;
        // Re-derives settings_'s discrete fields from the widgets just set
        // (a no-op given they were set from settings_ itself) and refreshes
        // everything downstream of it -- monitor tap options included.
        updateSettingsFromControls();
        updateAdvancedControlAvailability();
        updateRoutingGraph();
        setControlPage(currentPage_);
        rebuildRig();
    }

    // A/B comparison: swaps the currently active LiveRigSettings with the
    // held-aside inactive slot and rebuilds the rig from it. See
    // RigABState.h's doc comment for why this always goes through a full
    // rebuildRig() rather than the per-knob real-time path -- A and B can
    // differ in pedal/amp model or routing, which setNodeParameter/
    // setNodeTypeParameter (issue #47's real-time knob path) can't express,
    // only a topology rebuild can. RealtimeGraphHost's block-boundary hot
    // swap (see its class doc comment) is what keeps that rebuild low
    // latency and real-time safe, exactly as it already is for every other
    // topology-affecting control on this page.
    void selectRigSlot(bool wantA) {
        if (wantA == abActiveIsA_) return;
        std::swap(settings_, abInactiveSettings_);
        abActiveIsA_ = wantA;
        refreshControlsFromSettings();
        updateSlotButtonAppearance();
    }

    void updateSlotButtonAppearance() {
        slotAButton_.setColour(juce::TextButton::buttonColourId,
            abActiveIsA_ ? juce::Colour::fromRGB(43, 119, 134) : juce::Colour::fromRGB(43, 55, 61));
        slotBButton_.setColour(juce::TextButton::buttonColourId,
            !abActiveIsA_ ? juce::Colour::fromRGB(43, 119, 134) : juce::Colour::fromRGB(43, 55, 61));
    }

    // Rebuilds presetBox_'s item list from what's actually on disk right
    // now (PresetStore::list() is control-thread filesystem I/O, cheap
    // enough to call on every page switch / save / delete). Keeps
    // `preferredSelection` selected if given and still present, otherwise
    // tries to keep whatever was already showing.
    void refreshPresetList(const juce::String& preferredSelection = {}) {
        const juce::String target = preferredSelection.isNotEmpty()
            ? preferredSelection : presetBox_.getText();
        presetBox_.clear(juce::dontSendNotification);
        presetSummaries_ = presetStore_.list();
        int selectId = 0;
        for (std::size_t i = 0; i < presetSummaries_.size(); ++i) {
            const int id = static_cast<int>(i) + 1;
            presetBox_.addItem(juce::String(presetSummaries_[i].name), id);
            if (juce::String(presetSummaries_[i].name) == target) selectId = id;
        }
        presetBox_.setSelectedId(selectId, juce::dontSendNotification);
        const bool has = selectId != 0;
        loadPresetButton_.setEnabled(has);
        deletePresetButton_.setEnabled(has);
    }

    // Reloads a preset's saved measured-IR reference (see LiveRigPreset.h):
    // an empty path means "use the built-in reference IR", a path that no
    // longer resolves on this machine falls back to the same built-in
    // reference rather than silently keeping whatever was previously loaded,
    // and otherwise re-runs the normal file-load/resample path so the IR is
    // correct for the current device sample rate.
    void applyCabinetIrFromPreset(const std::string& path, MeasuredImpulseState& state,
                                  juce::Label& label, const juce::String& fallbackText) {
        if (path.empty()) {
            state = MeasuredImpulseState{};
            updateImpulseLabel(state, label, fallbackText);
            return;
        }
        const juce::File file(path);
        if (file.existsAsFile()) {
            loadImpulseResponse(file, state, label, fallbackText);
        } else {
            state = MeasuredImpulseState{};
            label.setText("Preset IR not found: " + file.getFullPathName(), juce::dontSendNotification);
        }
    }

    void onSavePresetClicked() {
        const auto name = presetNameEditor_.getText().trim();
        if (name.isEmpty()) {
            presetStatusLabel_.setText("Enter a preset name before saving", juce::dontSendNotification);
            return;
        }
        guitardsp::app::LiveRigPreset preset;
        preset.name = name.toStdString();
        preset.settings = settings_;
        preset.guitarCabinetIrPath = loadedGuitarIr_.impulse.empty()
            ? std::string() : loadedGuitarIr_.fullPath.toStdString();
        preset.bassCabinetIrPath = loadedBassIr_.impulse.empty()
            ? std::string() : loadedBassIr_.fullPath.toStdString();

        std::string error;
        if (presetStore_.save(preset, &error)) {
            presetStatusLabel_.setText("Saved preset \"" + name + "\"", juce::dontSendNotification);
            refreshPresetList(name);
        } else {
            presetStatusLabel_.setText("Save failed: " + juce::String(error), juce::dontSendNotification);
        }
    }

    void onLoadPresetClicked() {
        const int id = presetBox_.getSelectedId();
        if (id <= 0 || static_cast<std::size_t>(id - 1) >= presetSummaries_.size()) return;
        const std::string name = presetSummaries_[static_cast<std::size_t>(id - 1)].name;

        guitardsp::app::LiveRigPreset preset;
        std::string error;
        if (!presetStore_.load(name, preset, &error)) {
            presetStatusLabel_.setText("Load failed: " + juce::String(error), juce::dontSendNotification);
            return;
        }

        settings_ = preset.settings;
        applyCabinetIrFromPreset(preset.guitarCabinetIrPath, loadedGuitarIr_, irLabel_,
            "BUILT-IN REFERENCE / calibrated guitar cabinet / not measured");
        applyCabinetIrFromPreset(preset.bassCabinetIrPath, loadedBassIr_, bassIrLabel_,
            "BUILT-IN REFERENCE / calibrated bass cabinet / not measured");
        refreshControlsFromSettings();
        presetNameEditor_.setText(juce::String(name), juce::dontSendNotification);
        presetStatusLabel_.setText("Loaded preset \"" + juce::String(name) + "\"", juce::dontSendNotification);
    }

    void onDeletePresetClicked() {
        const int id = presetBox_.getSelectedId();
        if (id <= 0 || static_cast<std::size_t>(id - 1) >= presetSummaries_.size()) return;
        const std::string name = presetSummaries_[static_cast<std::size_t>(id - 1)].name;

        std::string error;
        if (presetStore_.remove(name, &error)) {
            presetStatusLabel_.setText("Deleted preset \"" + juce::String(name) + "\"", juce::dontSendNotification);
            refreshPresetList();
        } else {
            presetStatusLabel_.setText("Delete failed: " + juce::String(error), juce::dontSendNotification);
        }
    }

    [[nodiscard]] std::string_view selectedGuitarAmpType() const noexcept {
        switch (settings_.amp) {
            case guitardsp::app::AmpModel::britishPlexiFamily:
                return "British Plexi Family Reference";
            case guitardsp::app::AmpModel::americanCleanFamily:
                return "American Clean Family Reference";
            case guitardsp::app::AmpModel::preampCircuit:
                return "Preamp Circuit";
            case guitardsp::app::AmpModel::fullAmpCircuit:
                return "Full Amp Circuit";
            case guitardsp::app::AmpModel::reference:
            default:
                return "Reference Amp Topology";
        }
    }

    void timerCallback() override {
        engine_.collectRetired();
        inputTapFifo_.drain([this](const float* samples, int count) {
            const float* channelData[]{samples};
            inputWaveform_.pushBuffer(channelData, 1, count);
            inputSpectrum_.pushSamples(samples, count);
        });
        // The fundamental is only well-defined while the test-signal input
        // routing drives a known frequency; set it before draining so the
        // window that just filled uses the current slider value.
        outputThd_.setFundamentalHz(testSignalFrequency_.getValue());
        outputTapFifo_.drain([this](const float* samples, int count) {
            const float* channelData[]{samples};
            outputWaveform_.pushBuffer(channelData, 1, count);
            outputSpectrum_.pushSamples(samples, count);
            outputThd_.pushSamples(samples, count);
        });
        if (toneControlsPending_) applyToneControls();
        const auto stats = engine_.stats();
        const auto dbText = [](float peak) {
            return peak > 1.0e-9f
                ? juce::String(juce::Decibels::gainToDecibels(peak), 1) + " dBFS"
                : juce::String("-inf dBFS");
        };

        juce::String selected = "none";
        if (stats.inputRoutingMode == guitardsp::app::InputRoutingMode::testSignal)
            selected = "test signal (" + juce::String(testSignalFrequency_.getValue(), 0) + " Hz)";
        else if (stats.inputRoutingMode == guitardsp::app::InputRoutingMode::stereo)
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

        if (stats.inputRoutingMode != guitardsp::app::InputRoutingMode::testSignal) {
            thdLabel_.setText(
                "THD: -- (select \"Test signal (sine)\" input routing to measure)",
                juce::dontSendNotification);
        } else if (!outputThd_.hasMetrics()) {
            thdLabel_.setText("THD: measuring...", juce::dontSendNotification);
        } else {
            const auto& metrics = outputThd_.metrics();
            juce::String breakdown;
            for (std::size_t i = 0; i < metrics.harmonicMagnitudes.size(); ++i) {
                const float ratio = metrics.fundamental > 1.0e-9f
                    ? metrics.harmonicMagnitudes[i] / metrics.fundamental : 0.0f;
                const float ratioDb = juce::Decibels::gainToDecibels(ratio, -160.0f);
                breakdown << "  H" << juce::String(static_cast<int>(i) + 2)
                          << ": " << juce::String(ratioDb, 1) << " dB";
            }
            thdLabel_.setText(
                "THD (output tap, " + juce::String(testSignalFrequency_.getValue(), 0)
                    + " Hz fundamental): " + juce::String(100.0f * metrics.thd, 2)
                    + " %  (" + juce::String(metrics.thdDb, 1) + " dB)" + breakdown,
                juce::dontSendNotification);
        }

        const double driverCpu = 100.0 * std::max(0.0, deviceManager_.getCpuUsage());
        const double callbackCpu = 100.0 * static_cast<double>(stats.performance.averageLoad);
        const double callbackPeak = 100.0 * static_cast<double>(stats.performance.peakLoad);
        const double callbackP99 = 100.0
            * static_cast<double>(stats.performance.percentile99Load);
        const int xruns = std::max(0, deviceManager_.getXRunCount() - xRunBaseline_);
        performanceLabel_.setText(
            "Driver CPU: " + juce::String(driverCpu, 1)
                + "%    Callback avg: " + juce::String(callbackCpu, 1)
                + "%    P99: " + juce::String(callbackP99, 1)
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
            // Device I/O is the ALSA ring latency the driver reports for the open
            // period configuration (in + out), not a multiple of the block the UI
            // shows. It is printed in samples as well so a period size the WAVIO
            // driver silently refuses to change is visible here.
            latencyLabel_.setText(
                "Buffer: " + juce::String(currentBlockSize_) + " samples / "
                    + juce::String(toMilliseconds(currentBlockSize_), 2)
                    + " ms    Device I/O: " + juce::String(ioLatency) + " smp / "
                    + juce::String(toMilliseconds(ioLatency), 2)
                    + " ms    DSP: "
                    + juce::String(toMilliseconds(stats.graphLatencySamples), 2)
                    + " ms    Reported total: "
                    + juce::String(toMilliseconds(totalReportedLatency), 2) + " ms"
#if GUITARDSP_NATIVE_CPU_ENABLED
                    + "    Build: Native CPU",
#else
                    + "    Build: Portable",
#endif
                juce::dontSendNotification);
        }

        const bool fault = xruns > 0 || stats.performance.deadlineMisses > 0
            || stats.nonFiniteInputSamples > 0 || stats.nonFiniteOutputSamples > 0;
        const bool overload = driverCpu > 80.0 || callbackP99 > 90.0;
        // Test-signal mode always forces the physical output silent (see
        // RealtimeAudioEngine::process()), independent of the manual Mute
        // toggle, so reflect that here too.
        const bool outputMuted = mute_.getToggleState()
            || stats.inputRoutingMode == guitardsp::app::InputRoutingMode::testSignal;
        safetyLabel_.setColour(juce::Label::textColourId,
            fault ? juce::Colours::orangered
                  : overload || outputMuted ? juce::Colours::orange
                                             : juce::Colours::lightgreen);
        safetyLabel_.setText(
            (outputMuted ? "OUTPUT MUTED    " : "OUTPUT ACTIVE    ")
                + juce::String(safeDry_.getToggleState()
                    ? "Safe dry monitor    " : "Full pedal / amp / cabinet path    ")
                + "Nonfinite input: "
                + juce::String(static_cast<juce::int64>(stats.nonFiniteInputSamples))
                + "    Nonfinite output: "
                + juce::String(static_cast<juce::int64>(stats.nonFiniteOutputSamples)),
            juce::dontSendNotification);
    }

    void changeListenerCallback(juce::ChangeBroadcaster* source) override {
        if (source != &deviceManager_) return;
        // Re-apply the floor only when a different device was selected, so opening
        // AUDIO SETTINGS and deliberately choosing a smaller period on a machine
        // that can sustain it is still possible without it bouncing straight back.
        const auto* device = deviceManager_.getCurrentAudioDevice();
        const juce::String deviceName = device != nullptr ? device->getName() : juce::String();
        if (deviceName != lastBufferFloorDevice_) enforceMinimumBufferSize();
        persistAudioDeviceState();
    }

    // The Onkyo WAVIO / ICE1724 path reports XRUNs at 512-sample ALSA periods once
    // a component-level pedal or preamp circuit is in the chain (ALSA ring latency
    // is period x (periods - 1), ~3x the block, per direction). Open the device at
    // a larger period so scheduling jitter has headroom. This is a startup / device
    // -switch floor: the AUDIO SETTINGS dropdown can still raise it further or, once
    // a device is running, lower it. A device that offers nothing at or above the
    // floor (or no device at all) is left untouched.
    static constexpr int kMinimumBufferSize = 1024;

    void enforceMinimumBufferSize() {
        if (applyingBufferFloor_) return;
        auto* device = deviceManager_.getCurrentAudioDevice();
        if (device == nullptr) return;

        lastBufferFloorDevice_ = device->getName();

        auto setup = deviceManager_.getAudioDeviceSetup();
        if (setup.bufferSize >= kMinimumBufferSize) return;

        int target = 0;
        for (const int size : device->getAvailableBufferSizes())
            if (size >= kMinimumBufferSize) { target = size; break; }
        if (target == 0 || target == setup.bufferSize) return;

        setup.bufferSize = target;
        applyingBufferFloor_ = true;
        const auto error = deviceManager_.setAudioDeviceSetup(setup, true);
        applyingBufferFloor_ = false;
        if (error.isNotEmpty())
            statusLabel_.setText("Could not raise audio buffer to "
                                     + juce::String(target) + " samples: " + error,
                                 juce::dontSendNotification);
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
            case 5: mode = InputRoutingMode::testSignal; break;
            default: break;
        }
        engine_.setInputRoutingMode(mode);

        const bool testSignalMode = mode == InputRoutingMode::testSignal;
        testSignalActive_.store(testSignalMode, std::memory_order_release);
        testSignalFrequency_.setEnabled(testSignalMode);
        testSignalLevel_.setEnabled(testSignalMode);
        engine_.setTestSignalFrequencyHz(static_cast<float>(testSignalFrequency_.getValue()));
        engine_.setTestSignalAmplitude(static_cast<float>(testSignalLevel_.getValue() * 0.01));

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
        settings_.ampOutputDb = static_cast<float>(ampOutput_.getValue());
        settings_.ampPowerTube = static_cast<float>(powerTubeBox_.getSelectedId() - 1);
        settings_.ampToneStack = static_cast<float>(toneStackBox_.getSelectedId() - 1);
        settings_.ampToneDriver = static_cast<float>(toneDriverBox_.getSelectedId() - 1);
        settings_.ampFeedbackVoicing = static_cast<float>(
            feedbackVoicingBox_.getSelectedId() - 1);
        settings_.cabinetMix = normalized(cabinetMix_);
        settings_.speakerCompression = normalized(speakerCompression_);
        settings_.speakerExcursion = normalized(speakerExcursion_);
        settings_.speakerResonance = normalized(speakerResonance_);
        settings_.cabinetOutputDb = static_cast<float>(cabinetOutput_.getValue());
        settings_.cabinetLowCutHz = static_cast<float>(cabinetLowCut_.getValue());
        settings_.cabinetHighCutHz = static_cast<float>(cabinetHighCut_.getValue());
        settings_.guitarBranchLevel = normalized(guitarBranchLevel_);
        settings_.bassBranchLevel = normalized(bassBranchLevel_);
        settings_.octaveMix = normalized(octaveMix_);
        settings_.octaveLevel = normalized(octaveLevel_);
        settings_.bassGain = normalized(bassGain_);
        settings_.bassTone = normalized(bassTone_);
        settings_.bassLevel = normalized(bassLevel_);
        settings_.crossoverFrequency = static_cast<float>(crossoverFrequency_.getValue());

        if (!engine_.configured() || safeDry_.getToggleState()) return;
        using guitardsp::graph::NodeCategory;
        engine_.setNodeParameter(NodeCategory::drive, 0, settings_.pedalDrive);
        engine_.setNodeParameter(NodeCategory::drive, 1, settings_.pedalTone);
        engine_.setNodeParameter(NodeCategory::drive, 2, settings_.pedalLevel);
        const auto ampType = selectedGuitarAmpType();
        if (selectedAmpIsCircuitLevel()) {
            // PreampCircuitNode/FullAmpCircuitNode only expose Drive/Bass/Treble
            // (indices 0-2); reusing Gain/Bass/Treble avoids sending Mid onto the
            // node's Treble parameter the way a flat 0..3 index mapping would.
            engine_.setNodeTypeParameter(ampType, 0, settings_.ampGain);
            engine_.setNodeTypeParameter(ampType, 1, settings_.ampBass);
            engine_.setNodeTypeParameter(ampType, 2, settings_.ampTreble);
        } else {
            engine_.setNodeTypeParameter(ampType, 0, settings_.ampGain);
            engine_.setNodeTypeParameter(ampType, 1, settings_.ampBass);
            engine_.setNodeTypeParameter(ampType, 2, settings_.ampMid);
            engine_.setNodeTypeParameter(ampType, 3, settings_.ampTreble);
            engine_.setNodeTypeParameter(ampType, 4, settings_.ampMaster);
            engine_.setNodeTypeParameter(ampType, 5, settings_.ampPresence);
            engine_.setNodeTypeParameter(ampType, 6, settings_.ampOutputDb);
            if (settings_.amp == guitardsp::app::AmpModel::reference) {
                engine_.setNodeTypeParameter(ampType, 7, settings_.ampPowerTube);
                engine_.setNodeTypeParameter(ampType, 8, settings_.ampToneStack);
                engine_.setNodeTypeParameter(ampType, 9, settings_.ampToneDriver);
                engine_.setNodeTypeParameter(ampType, 10, settings_.ampFeedbackVoicing);
            }
        }

        constexpr std::string_view guitarCab = "Speaker Dynamics + Partitioned Cab";
        engine_.setNodeTypeParameter(guitarCab, 0, settings_.speakerCompression);
        engine_.setNodeTypeParameter(guitarCab, 1, settings_.speakerExcursion);
        engine_.setNodeTypeParameter(guitarCab, 2, settings_.speakerResonance);
        engine_.setNodeTypeParameter(guitarCab, 3, settings_.cabinetOutputDb);
        engine_.setNodeTypeParameter(guitarCab, 4, settings_.cabinetMix);
        engine_.setNodeTypeParameter(guitarCab, 5, settings_.cabinetLowCutHz);
        engine_.setNodeTypeParameter(guitarCab, 6, settings_.cabinetHighCutHz);
        engine_.setNodeTypeParameter("Guitar Branch Level", 0, settings_.guitarBranchLevel);
        engine_.setNodeTypeParameter("Bass Branch Level", 0, settings_.bassBranchLevel);
        engine_.setNodeTypeParameter("Monophonic Octave Down", 0, settings_.octaveMix);
        engine_.setNodeTypeParameter("Monophonic Octave Down", 1, settings_.octaveLevel);
        engine_.setNodeTypeParameter("Bass Amp Reference", 0, settings_.bassGain);
        engine_.setNodeTypeParameter("Bass Amp Reference", 1, settings_.bassTone);
        engine_.setNodeTypeParameter("Bass Amp Reference", 2, settings_.bassLevel);
        engine_.setNodeTypeParameter("CrossoverSplit", 0, settings_.crossoverFrequency);
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
            case 4: settings_.amp = guitardsp::app::AmpModel::preampCircuit; break;
            case 5: settings_.amp = guitardsp::app::AmpModel::fullAmpCircuit; break;
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
        switch (signalRoutingBox_.getSelectedId()) {
            case 2:
                settings_.signalRouting = guitardsp::app::SignalRouting::parallelOctaveBass;
                break;
            case 3:
                settings_.signalRouting = guitardsp::app::SignalRouting::crossoverOctaveBass;
                break;
            default:
                settings_.signalRouting = guitardsp::app::SignalRouting::serialGuitar;
                break;
        }
        settings_.octaveEnabled = octaveEnabled_.getToggleState();
        settings_.bassCabinetEnabled = bassCabinetEnabled_.getToggleState();
        settings_.matchMeasuredCabinetLevel = matchIrLevel_.getToggleState();
        updateMonitorTapOptions();
    }

    // Rebuilds each monitor window's tap-selection dropdown from the tap
    // points the current rig settings actually make available (see
    // availableMonitorTapPoints()), so a bypassed pedal, a disabled amp/
    // cabinet, serial routing, etc. never leave a stale, nonexistent tap
    // selectable. Preserves each window's current selection if it's still
    // available; otherwise falls back to the physical input/output default.
    void updateMonitorTapOptions() {
        auto effective = settings_;
        if (safeDry_.getToggleState()) {
            effective.pedal = guitardsp::app::PedalModel::bypass;
            effective.signalRouting = guitardsp::app::SignalRouting::serialGuitar;
            effective.ampEnabled = false;
            effective.cabinetEnabled = false;
        }
        const auto points = guitardsp::app::availableMonitorTapPoints(effective);

        auto populate = [&](juce::ComboBox& box,
                            std::vector<guitardsp::app::MonitorTapPoint>& options,
                            std::atomic<guitardsp::app::MonitorTapPoint>& selection,
                            guitardsp::app::MonitorTapPoint fallback) {
            const auto current = selection.load(std::memory_order_relaxed);
            options = points;
            box.clear(juce::dontSendNotification);
            int selectId = 0;
            for (std::size_t i = 0; i < options.size(); ++i) {
                const int id = static_cast<int>(i) + 1;
                box.addItem(guitardsp::app::monitorTapPointLabel(options[i]), id);
                if (options[i] == current) selectId = id;
            }
            if (selectId == 0) {
                const auto it = std::find(options.begin(), options.end(), fallback);
                selectId = it != options.end()
                    ? static_cast<int>(std::distance(options.begin(), it)) + 1
                    : (options.empty() ? 0 : 1);
                if (selectId > 0)
                    selection.store(options[static_cast<std::size_t>(selectId - 1)],
                                    std::memory_order_relaxed);
            }
            box.setSelectedId(selectId, juce::dontSendNotification);
        };
        populate(inputMonitorTapBox_, inputMonitorTapOptions_, inputMonitorTap_,
                guitardsp::app::MonitorTapPoint::physicalInput);
        populate(outputMonitorTapBox_, outputMonitorTapOptions_, outputMonitorTap_,
                guitardsp::app::MonitorTapPoint::physicalOutput);
    }

    // Offline-resamples a measured IR to the active device sample rate and
    // optionally applies the same loudness-matching calibration as the
    // guitar cabinet (see ReferenceCabinetIR.h). Empty when no measured IR
    // has been loaded, or before the device sample rate is known.
    std::vector<float> prepareDeviceImpulse(const MeasuredImpulseState& state,
                                            bool matchLevel) const {
        if (state.impulse.empty() || state.sampleRate <= 0.0 || currentSampleRate_ <= 0.0)
            return {};
        auto resampled = guitardsp::app::resampleImpulseWindowedSinc(
            state.impulse, state.sampleRate, currentSampleRate_);
        if (matchLevel) {
            auto calibrated = guitardsp::app::calibrateMeasuredCabinetImpulse(
                resampled, currentSampleRate_);
            return std::move(calibrated.impulse);
        }
        for (float& sample : resampled)
            if (!std::isfinite(sample)) sample = 0.0f;
        return resampled;
    }

    guitardsp::app::LiveRigSettings settingsForCurrentDevice() const {
        auto result = settings_;
        if (safeDry_.getToggleState()) {
            result.pedal = guitardsp::app::PedalModel::bypass;
            result.signalRouting = guitardsp::app::SignalRouting::serialGuitar;
            result.ampEnabled = false;
            result.cabinetEnabled = false;
            result.cabinetImpulse.clear();
            result.bassCabinetImpulse.clear();
            return result;
        }
        result.cabinetImpulse = prepareDeviceImpulse(loadedGuitarIr_, result.matchMeasuredCabinetLevel);
        result.bassCabinetImpulse = prepareDeviceImpulse(loadedBassIr_, result.matchMeasuredCabinetLevel);
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
            "Choose a measured guitar cabinet IR WAV/AIFF", juce::File{}, "*.wav;*.aif;*.aiff");
        fileChooser_->launchAsync(juce::FileBrowserComponent::openMode
                                    | juce::FileBrowserComponent::canSelectFiles,
                                  [safe = juce::Component::SafePointer<MainComponent>(this)](
                                      const juce::FileChooser& chooser) {
            if (safe == nullptr) return;
            const auto file = chooser.getResult();
            if (file.existsAsFile())
                safe->loadImpulseResponse(file, safe->loadedGuitarIr_, safe->irLabel_,
                    "BUILT-IN REFERENCE / calibrated guitar cabinet / not measured");
        });
    }

    void chooseBassImpulseResponse() {
        bassFileChooser_ = std::make_unique<juce::FileChooser>(
            "Choose a measured bass cabinet IR WAV/AIFF", juce::File{}, "*.wav;*.aif;*.aiff");
        bassFileChooser_->launchAsync(juce::FileBrowserComponent::openMode
                                        | juce::FileBrowserComponent::canSelectFiles,
                                      [safe = juce::Component::SafePointer<MainComponent>(this)](
                                          const juce::FileChooser& chooser) {
            if (safe == nullptr) return;
            const auto file = chooser.getResult();
            if (file.existsAsFile())
                safe->loadImpulseResponse(file, safe->loadedBassIr_, safe->bassIrLabel_,
                    "BUILT-IN REFERENCE / calibrated bass cabinet / not measured");
        });
    }

    void updateImpulseLabel(const MeasuredImpulseState& state, juce::Label& label,
                            const juce::String& fallbackText) {
        if (state.impulse.empty()) {
            label.setText(fallbackText, juce::dontSendNotification);
            label.setColour(juce::Label::textColourId, juce::Colours::lightgrey);
            return;
        }

        const bool matched = matchIrLevel_.getToggleState();
        const double peak = matched ? state.matchedPeakDb : state.rawPeakDb;
        juce::String description = state.name + "  /  "
            + juce::String(state.sampleRate, 0) + " Hz  /  ";
        if (matched) {
            description += "matched "
                + juce::String(state.calibrationGainDb, 1) + " dB";
        } else {
            description += "RAW LEVEL";
        }
        description += "  /  peak " + juce::String(peak, 1) + " dB";
        if (!state.allFinite) description += "  /  invalid samples removed";
        label.setText(description, juce::dontSendNotification);
        label.setColour(juce::Label::textColourId,
            !matched && peak > 8.0 ? juce::Colours::orange
                                   : juce::Colours::lightgrey);
    }

    void loadImpulseResponse(const juce::File& file, MeasuredImpulseState& state,
                             juce::Label& label, const juce::String& fallbackText) {
        auto reader = std::unique_ptr<juce::AudioFormatReader>(formatManager_.createReaderFor(file));
        if (!reader) {
            label.setText("IR load failed: unsupported file", juce::dontSendNotification);
            return;
        }

        constexpr std::int64_t maximumIrSamples = 262144;
        const auto samplesToRead64 = std::min<std::int64_t>(reader->lengthInSamples,
                                                            maximumIrSamples);
        if (samplesToRead64 <= 0) return;
        const int samplesToRead = static_cast<int>(samplesToRead64);
        juce::AudioBuffer<float> buffer(1, samplesToRead);
        if (!reader->read(&buffer, 0, samplesToRead, 0, true, false)) {
            label.setText("IR load failed while reading audio", juce::dontSendNotification);
            return;
        }

        state.impulse.assign(buffer.getReadPointer(0), buffer.getReadPointer(0) + samplesToRead);
        state.sampleRate = reader->sampleRate;
        state.name = file.getFileName();
        state.fullPath = file.getFullPathName();
        const auto calibration = guitardsp::app::calibrateMeasuredCabinetImpulse(
            state.impulse, state.sampleRate);
        state.calibrationGainDb = calibration.appliedGainDb;
        state.rawPeakDb = calibration.before.maximumGainDb;
        state.matchedPeakDb = calibration.after.maximumGainDb;
        state.allFinite = calibration.before.allFinite;
        if (!state.allFinite) {
            for (float& sample : state.impulse)
                if (!std::isfinite(sample)) sample = 0.0f;
        }
        updateImpulseLabel(state, label, fallbackText);
        rebuildRig();
    }

    juce::AudioDeviceManager deviceManager_;
    juce::AudioDeviceSelectorComponent deviceSelector_;
    juce::AudioFormatManager formatManager_;
    guitardsp::app::RealtimeAudioEngine engine_;
    guitardsp::app::LiveRigSettings settings_;
    // A/B comparison (issue #79): settings_ always holds the *active* slot
    // (A or B); the other slot's settings sit here untouched until toggled
    // back in. See selectRigSlot()/RigABState.h.
    guitardsp::app::LiveRigSettings abInactiveSettings_;
    bool abActiveIsA_ = true;
    guitardsp::app::PresetStore presetStore_;
    // Message-thread-only: index i of presetBox_'s item (id i+1) corresponds
    // to presetSummaries_[i], rebuilt by refreshPresetList().
    std::vector<guitardsp::app::PresetSummary> presetSummaries_;

    juce::ComboBox pedalBox_;
    juce::ComboBox ampBox_;
    juce::ComboBox qualityBox_;
    juce::ComboBox inputRoutingBox_;
    juce::ComboBox signalRoutingBox_;
    juce::ComboBox powerTubeBox_;
    juce::ComboBox toneStackBox_;
    juce::ComboBox toneDriverBox_;
    juce::ComboBox feedbackVoicingBox_;
    juce::ToggleButton ampEnabled_;
    juce::ToggleButton cabEnabled_;
    juce::ToggleButton octaveEnabled_;
    juce::ToggleButton bassCabinetEnabled_;
    juce::ToggleButton safeDry_;
    juce::ToggleButton mute_;
    juce::ToggleButton matchIrLevel_;
    juce::Slider inputTrim_;
    juce::Slider outputTrim_;
    juce::Label testSignalLabel_;
    juce::Slider testSignalFrequency_;
    juce::Slider testSignalLevel_;
    juce::Slider pedalDrive_;
    juce::Slider pedalTone_;
    juce::Slider pedalLevel_;
    juce::Slider ampGain_;
    juce::Slider ampBass_;
    juce::Slider ampMid_;
    juce::Slider ampTreble_;
    juce::Slider ampMaster_;
    juce::Slider ampPresence_;
    juce::Slider cabinetMix_;
    juce::Slider cabinetLowCut_;
    juce::Slider cabinetHighCut_;
    juce::Slider speakerCompression_;
    juce::Slider speakerExcursion_;
    juce::Slider speakerResonance_;
    juce::Slider cabinetOutput_;
    juce::Slider ampOutput_;
    juce::Slider guitarBranchLevel_;
    juce::Slider bassBranchLevel_;
    juce::Slider octaveMix_;
    juce::Slider octaveLevel_;
    juce::Slider bassGain_;
    juce::Slider bassTone_;
    juce::Slider bassLevel_;
    juce::Slider crossoverFrequency_;
    juce::TextButton loadIrButton_;
    juce::TextButton loadBassIrButton_;
    juce::TextButton resetDiagnosticsButton_;
    juce::TextButton audioSettingsButton_;
    juce::TextButton rigPageButton_;
    juce::TextButton cabinetPageButton_;
    juce::TextButton routingPageButton_;
    juce::TextButton advancedPageButton_;
    juce::TextButton presetsPageButton_;
    juce::TextButton slotAButton_;
    juce::TextButton slotBButton_;
    juce::ComboBox presetBox_;
    juce::TextButton loadPresetButton_;
    juce::TextButton deletePresetButton_;
    juce::TextEditor presetNameEditor_;
    juce::TextButton savePresetButton_;
    juce::Label presetStatusLabel_;
    RoutingGraphView routingGraphView_;
    guitardsp::app::AudioTapFifo inputTapFifo_;
    guitardsp::app::AudioTapFifo outputTapFifo_;
    // Audio-callback-readable flag mirroring the InputRoutingMode combo box.
    std::atomic<bool> testSignalActive_{false};
    // Scratch buffers process() writes each monitor window's selected tap
    // signal into every callback (see RealtimeAudioEngine::process()'s
    // MonitorTapPoint doc comment); sized to the device block size in
    // audioDeviceAboutToStart().
    std::vector<float> monitorTapBufferA_;
    std::vector<float> monitorTapBufferB_;
    // Audio-callback-readable mirrors of inputMonitorTapBox_/
    // outputMonitorTapBox_'s current selection; the id<->MonitorTapPoint
    // mapping can change (see inputMonitorTapOptions_/outputMonitorTapOptions_
    // below) so the combo box's selected id alone isn't audio-thread safe.
    std::atomic<guitardsp::app::MonitorTapPoint> inputMonitorTap_{
        guitardsp::app::MonitorTapPoint::physicalInput};
    std::atomic<guitardsp::app::MonitorTapPoint> outputMonitorTap_{
        guitardsp::app::MonitorTapPoint::physicalOutput};
    // Message-thread-only: index i of each combo box corresponds to
    // options[i], rebuilt by updateMonitorTapOptions() whenever the rig
    // settings change what SIGNAL CHAIN stages exist.
    std::vector<guitardsp::app::MonitorTapPoint> inputMonitorTapOptions_;
    std::vector<guitardsp::app::MonitorTapPoint> outputMonitorTapOptions_;
    juce::AudioVisualiserComponent inputWaveform_{1};
    juce::AudioVisualiserComponent outputWaveform_{1};
    guitardsp::app::SpectrumAnalyserComponent inputSpectrum_{2048};
    guitardsp::app::SpectrumAnalyserComponent outputSpectrum_{2048};
    // THD readout for the output monitor tap. Message-thread-only, driven
    // from the same outputTapFifo_.drain() callback as outputSpectrum_; see
    // ThdAnalyser.h. Only meaningful while the test-signal input routing is
    // active, since THD needs a known fundamental frequency to probe.
    guitardsp::app::ThdAnalyser outputThd_{4096};
    juce::TextButton spectrumToggle_;
    bool showSpectrum_ = false;
    juce::ComboBox inputMonitorTapBox_;
    juce::ComboBox outputMonitorTapBox_;
    juce::Label statusLabel_;
    juce::Label irLabel_;
    juce::Label bassIrLabel_;
    juce::Label meterLabel_;
    juce::Label routingLabel_;
    juce::Label performanceLabel_;
    juce::Label latencyLabel_;
    juce::Label safetyLabel_;
    juce::Label thdLabel_;
    juce::Label pedalControlsTitle_;
    juce::Label ampControlsTitle_;
    std::unique_ptr<juce::FileChooser> fileChooser_;
    std::unique_ptr<juce::FileChooser> bassFileChooser_;

    MeasuredImpulseState loadedGuitarIr_;
    MeasuredImpulseState loadedBassIr_;
    double currentSampleRate_ = 0.0;
    int currentBlockSize_ = 0;
    int currentInputLatencySamples_ = 0;
    int currentOutputLatencySamples_ = 0;
    int processingChannels_ = 2;
    int xRunBaseline_ = 0;
    bool toneControlsPending_ = false;
    bool applyingBufferFloor_ = false;
    juce::String lastBufferFloorDevice_;
    juce::Rectangle<int> chainPanel_;
    juce::Rectangle<int> inspectorPanel_;
    ControlPage currentPage_ = ControlPage::pedal;
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
    const juce::String getApplicationVersion() override { return "0.35.0"; }
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
