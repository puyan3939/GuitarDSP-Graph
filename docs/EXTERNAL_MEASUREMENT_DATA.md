# External measurement data registry

This project distinguishes between circuit/component references, simulated data, and physical measurements. Do not treat a public plot or schematic as measured hardware data unless the source explicitly says it was measured.

## Marshall JVM 410H measured audio dataset

- Source: Zenodo record 7970723
- DOI: 10.5281/zenodo.7970723
- File: `MarshallJVM410H.zip`
- Size: approximately 4.4 GB
- MD5 published by Zenodo: `4d08dac89b44f618f2d6c2a69739e104`
- Authors: Stepan Miklanek, Alec Wright, Vesa Valimaki, Jiri Schimmel
- Purpose: audio dataset used for the DAFx-23 work *Neural Grey-Box Guitar Amplifier Modelling with Limited Data*
- Sample rate used by the upstream realtime/model code: 44.1 kHz
- Capture note from the project page: recorded from the speaker output of a Marshall JVM 410H connected to a reactive load; presented references do not apply a speaker cabinet IR.

The upstream public loader encodes controls directly in directory/file names. Examples include:

```text
B0_M5_T5_G5/B0_M5_T5_G5-speakerout.wav
B5_M0_T5_G5/B5_M0_T5_G5-speakerout.wav
B5_M5_T10_G5/B5_M5_T10_G5-speakerout.wav
B6.5_M8.5_T3.5_G5/B6.5_M8.5_T3.5_G5-speakerout.wav
```

The published loader interprets B/M/T/G as bass, middle, treble and gain values divided by 10 for conditioning. It constructs a paired input WAV from the same condition name. This makes the dataset directly useful for whole-amplifier and control-response validation, while still requiring careful level/alignment handling before sample-wise model scoring.

The archive is too large for Git or CI. The repository provides an explicit local workflow:

```bash
# Download + checksum only
tools/fetch_marshall_jvm410h.sh external-data/marshall-jvm410h

# Download/checksum + extract
tools/fetch_marshall_jvm410h.sh external-data/marshall-jvm410h extract

# Download/checksum + extract + generate a CSV condition manifest
tools/fetch_marshall_jvm410h.sh external-data/marshall-jvm410h index
```

`tools/index_jvm410h_dataset.py` scans the extracted condition directories and emits `jvm410h_manifest.csv` with setting, bass, mid, treble, gain, paired input path and speaker-output path. The manifest is intended as the input contract for future offline fitting/evaluation tools.

CI must not fetch the multi-gigabyte archive.

## BYU guitar amplifier directivity measurements

BYU Scholar's Archive publishes CSV directivity measurements for several physical guitar amplifiers including Fender Blues Deluxe, Blues Junior, Deluxe Reverb, Princeton Reverb, Twin Reverb and Vox AC30. Individual CSV files are around 350 kB according to the archive listing.

These data are relevant to speaker/cabinet directivity and microphone-position work. They are not isolated preamp/tone-stack transfer measurements, so they must not be used as direct targets for DS-1, tone-stack, preamp or power-stage fitting without a model of the acoustic measurement chain.

Before importing any file, record its publication page, file name, acquisition geometry, units and any license/redistribution terms supplied by the archive.

## Exact/simulated tone-stack reference

David T. Yeh and Julius O. Smith, *Discretization of the '59 Fender Bassman Tone Stack*, DAFx-06, derives the exact third-order continuous transfer function for the Bassman FMV tone stack and compares it against SPICE. This is a circuit-analysis reference, not a physical hardware measurement dataset.

The project implementation `YehSmithToneStack` follows the published symbolic coefficients and bilinear discretization. The paper's component values and equations are appropriate for numerical regression of the tone-stack solver. Hardware-equivalence claims still require measured amplifier data.

## Data policy

For every imported external dataset create a sidecar metadata file containing at minimum:

- source URL / DOI
- original file name and checksum
- authors and publication
- license or redistribution status
- sample rate / bit depth for audio
- control positions and channel/mode
- source/load/reactive-load conditions
- whether cabinet, microphone, EQ or post-processing is present
- any gain normalization applied during import

Raw third-party datasets should normally stay outside Git. Small derived numeric fixtures may be committed only when the source/license permits redistribution and provenance is retained.
