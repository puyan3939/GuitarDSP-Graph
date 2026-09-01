#!/usr/bin/env python3
"""Index the public Marshall JVM 410H dataset after local extraction.

Expected directory names follow the published grey-box loader convention, e.g.
B5_M5_T5_G5/B5_M5_T5_G5-speakerout.wav and the paired input.wav file.
The script writes a compact CSV manifest suitable for later fitting tools.
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

SETTING_RE = re.compile(r"^B(?P<bass>[0-9.]+)_M(?P<mid>[0-9.]+)_T(?P<treble>[0-9.]+)_G(?P<gain>[0-9.]+)$")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path, help="Extracted JVM dataset root")
    parser.add_argument("--output", type=Path, default=Path("jvm410h_manifest.csv"))
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for speaker in sorted(args.root.rglob("*-speakerout.wav")):
        setting = speaker.parent.name
        match = SETTING_RE.match(setting)
        if not match:
            continue

        input_candidates = [speaker.parent / f"{setting}-input.wav", speaker.parent / "input.wav"]
        input_path = next((p for p in input_candidates if p.exists()), None)
        if input_path is None:
            # The upstream loader constructs the paired input filename from the
            # speaker-output name. Keep the expected path in the manifest even if
            # an archive variant has not been fully extracted yet.
            input_path = input_candidates[0]

        values = match.groupdict()
        rows.append({
            "setting": setting,
            "bass": values["bass"],
            "mid": values["mid"],
            "treble": values["treble"],
            "gain": values["gain"],
            "input_wav": str(input_path.relative_to(args.root)),
            "speakerout_wav": str(speaker.relative_to(args.root)),
        })

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=[
            "setting", "bass", "mid", "treble", "gain", "input_wav", "speakerout_wav"
        ])
        writer.writeheader()
        writer.writerows(rows)

    print(f"indexed {len(rows)} JVM conditions -> {args.output}")
    return 0 if rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
