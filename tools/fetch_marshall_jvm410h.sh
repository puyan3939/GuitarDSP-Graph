#!/usr/bin/env bash
set -euo pipefail

out_dir="${1:-external-data/marshall-jvm410h}"
mkdir -p "$out_dir"
archive="$out_dir/MarshallJVM410H.zip"
url="https://zenodo.org/records/7970723/files/MarshallJVM410H.zip"
expected_md5="4d08dac89b44f618f2d6c2a69739e104"

echo "Downloading approximately 4.4 GB measured Marshall JVM 410H dataset"
echo "Destination: $archive"
curl -L --fail --continue-at - -o "$archive" "$url"
printf '%s  %s\n' "$expected_md5" "$archive" | md5sum -c -
echo "Dataset verified. Keep external-data outside source control."
