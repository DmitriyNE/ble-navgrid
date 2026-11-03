#!/bin/bash
# Captures raw CS lines from an initiator and its reflector over their probes for a while, then
# runs both host estimators on the pair. Both TAGs must run an image built with
# NAVGRID_LOG_STEPS (firmware/README). The DK's own probe serial is `probe-rs list`'s.
#
#   scripts/cs-capture.sh <out-dir> <initiator probe> <reflector probe> <elf> [seconds]
#
# Writes <out-dir>/initiator.log, reflector.log and estimates.jsonl, and prints a summary.
set -e
out=$1; init=$2; refl=$3; elf=$4; secs=${5:-60}
[ -n "$elf" ] || { sed -n 2,7p "$0"; exit 1; }
mkdir -p "$out"
cd "$(dirname "$0")/.."
cargo build -q --release -p navgrid
(timeout "$secs" probe-rs attach --chip nRF54L15 --probe "$init" --speed 100 "$elf" > "$out/initiator.log" 2>&1 &)
timeout "$secs" probe-rs attach --chip nRF54L15 --probe "$refl" --speed 100 "$elf" > "$out/reflector.log" 2>&1 || true
sleep 2
echo "lines: initiator $(grep -c CSR "$out/initiator.log") CSR, reflector $(grep -c CSR "$out/reflector.log") CSR"
./target/release/navgrid cs "$out/initiator.log" "$out/reflector.log" --json --ipt > "$out/estimates.jsonl" 2> "$out/cs.err" || true
python3 - "$out/estimates.jsonl" <<'EOF'
import json, sys
ps, fs, st = [], [], []
peaks = {}
for l in open(sys.argv[1]):
    try:
        o = json.loads(l)
    except ValueError:
        continue
    c = o.get("combined") or o.get("estimate")
    if c and "distance_m" in c:
        ps.append(c["distance_m"])
    if o.get("first_path_m") is not None:
        fs.append(o["first_path_m"])
    for f in o.get("first_paths", []):
        peaks[f["peaks"]] = peaks.get(f["peaks"], 0) + 1
        st.append(f["strength"])
def q(v):
    v = sorted(v)
    if not v:
        return "none"
    return f"n={len(v)} median {v[len(v)//2]:.2f} m, p10 {v[len(v)//10]:.2f}, p90 {v[9*len(v)//10]:.2f}"
print("phase slope:", q(ps))
print("first path: ", q(fs))
print("components per antenna path:", dict(sorted(peaks.items())),
      "first-path strength median", f"{sorted(st)[len(st)//2]:.2f}" if st else None)
EOF
