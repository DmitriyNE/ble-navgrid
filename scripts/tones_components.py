#!/usr/bin/env python3
"""Aggregated super-resolution components: for one capture, node and model order, list every
procedure's matrix-pencil components per antenna path and print an amplitude-weighted histogram
of component distances (0.25 m bins, 0..16 m) per path, plus the median singular values."""
import argparse
import sys

import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from tones_estimators import N_SLOTS, cadzow_fill, components, hankel, load, matrix_pencil, phase_slope, vec

ap = argparse.ArgumentParser()
ap.add_argument("file")
ap.add_argument("--node", default="N005")
ap.add_argument("--order", type=int, default=4)
ap.add_argument("--fill-rank", type=int, default=4)
ap.add_argument("--radius", type=float, default=0.1)
ap.add_argument("--min-channels", type=int, default=55)
ap.add_argument("--list", action="store_true", help="list every procedure's components")
ap.add_argument("--bin", type=float, default=0.25)
ap.add_argument("--max", type=float, default=16.0)
args = ap.parse_args()

groups, names = load(args.file)
per_path = {}
sv_all = {}
for (node, proc, ant), h in sorted(groups.items()):
    if names.get(node) != args.node or len(h) < args.min_channels:
        continue
    x, m = vec(h)
    y = cadzow_fill(x, m, args.fill_rank)
    _, sv, _ = np.linalg.svd(hankel(y, N_SLOTS // 2), full_matrices=False)
    sv_all.setdefault(ant, []).append(sv[:8] / sv[0])
    z, a, _ = matrix_pencil(y, order=args.order)
    cs = components(z, a, 0.0, args.radius)
    per_path.setdefault(ant, []).append((proc, phase_slope(h), cs))
    if args.list:
        ps = phase_slope(h)
        print(f"{proc:5d} p{ant} slope {ps[0] if ps else float('nan'):5.2f} | " +
              ", ".join(f"{d:5.2f} ({am:.2f})" for d, am, r in cs))

short = args.file.split("/")[-1].replace(".jsonl.gz", "")
edges = np.arange(0, args.max + args.bin, args.bin)
print(f"### {short} {args.node}, pencil order {args.order}, fill rank {args.fill_rank}: amplitude-weighted component histogram per path")
print("      median singular values:", {ant: " ".join(f"{v:.2f}" for v in np.median(np.array(s), axis=0)) for ant, s in sorted(sv_all.items())})
for ant, rows in sorted(per_path.items()):
    hist = np.zeros(len(edges) - 1)
    n = 0
    for proc, ps, cs in rows:
        n += 1
        for d, am, r in cs:
            i = int(d / args.bin)
            if 0 <= i < len(hist):
                hist[i] += am
    hist /= max(n, 1)
    top = hist.max() or 1
    print(f"  path {ant} ({n} procedures), bars = mean amplitude per procedure, max {top:.2f}:")
    for i, v in enumerate(hist):
        if v >= 0.05 * top:
            print(f"    {edges[i]:5.2f}-{edges[i + 1]:5.2f} {'#' * int(40 * v / top):40s} {v:.2f}")
