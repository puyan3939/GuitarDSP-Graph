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
- Capture note from the project page: recorded from the speaker output of a Marshall JVM 410H connected to a reactive load; presented references do not apply a speaker cabinet IR.

This is useful for whole-amplifier and tone-control validation. It is large enough that it should not be committed to this repository. Keep it in a local `external-data/` directory and validate the checksum before use.

Suggested local download command:

```bash
mkdir -p external-data/marshall-jvm410h
curl -L --fail --continue-at - \
  -o external-data/marshall-jvm410h/MarshallJVM410H.zip \
  https://zenodo.org/records/7970723/files/MarshallJVM410H.zip
printf '%s  %s\n' 4d08dac89b44f618f2d6c2a69739e104 \
  external-data/marshall-jvm410h/MarshallJVM410H.zip | md5sum -c -
```

The downloader is intentionally manual because the archive is multi-gigabyte. CI must not fetch it.

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
