#!/usr/bin/env python3
"""Estimator comparison on raw-tone captures (Tones records in `navgrid monitor --json` logs).

For every (node, procedure, antenna path) the round-trip channel response H[k] over the CS
channels is rebuilt from the 3-byte PCTs, then:
  slope   the device's phase slope (adjacent-channel phasor sum, refined), as the crate does it
  ifft    matched filter / delay profile on a 5 cm grid, Hann window: strongest and earliest peak
  mp      matrix pencil (Hua & Sarkar) on the 75-sample vector, gaps filled from a first fit
  music   root-MUSIC with forward-backward smoothing on the same vector
Super-resolution components come with an amplitude (least squares) and a pole radius; a
component counts as a path when its amplitude is at least `--frac` of the strongest and its
pole radius is within `--radius` of 1.  "early" is the shortest such path, "strong" the
strongest.  Distances are one-way: round-trip phase falls by 4π·Δf·d/c per channel.
"""
import argparse
import gzip
import json
import sys

import numpy as np

C = 299_792_458.0
DF = 1.0e6
F0 = 2.402e9
N_SLOTS = 75  # channels 2..76
CS_CHANNELS = [k for k in range(79) if 2 <= k <= 22 or 26 <= k <= 76]
UNAMBIGUOUS_M = C / (2 * DF)  # 150 m


def pct(b):
    raw = b[0] | (b[1] << 8) | (b[2] << 16)
    i = raw & 0xFFF
    q = (raw >> 12) & 0xFFF
    if i >= 2048:
        i -= 4096
    if q >= 2048:
        q -= 4096
    return complex(i, q)


def load(path):
    op = gzip.open if path.endswith(".gz") else open
    groups = {}
    names = {}
    with op(path, "rt") as f:
        for line in f:
            try:
                v = json.loads(line)
            except ValueError:
                continue
            t = v.get("Tones")
            if not t:
                continue
            names[t["node"]] = v.get("number", str(t["node"]))
            g = groups.setdefault((t["node"], t["procedure"], t["path"]), {})
            p = t["pct"]
            for i in range(len(p) // 3):
                b = p[3 * i : 3 * i + 3]
                if b != [0, 0, 0]:
                    g[t["first"] + i] = pct(b)
    return groups, names


def d_from_pole(z):
    d = -np.angle(z) * C / (4 * np.pi * DF)
    return np.mod(d, UNAMBIGUOUS_M)


# ---------------------------------------------------------------- phase slope (as the crate)
def phase_slope(h):
    """h: dict channel -> complex. Returns (distance_m, coherence) or None."""
    ks = sorted(h)
    pair_sum = 0j
    pair_w = 0.0
    pairs = 0
    for k in ks:
        if k + 1 in h:
            a, b = h[k], h[k + 1]
            pair_sum += b * np.conj(a)
            pair_w += abs(a) * abs(b)
            pairs += 1
    if pairs < 20 or pair_w <= 0:
        return None
    coarse = np.angle(pair_sum)
    kk = np.array(ks, float)
    hh = np.array([h[k] for k in ks])
    der = hh * np.exp(-1j * coarse * kk)
    off = np.exp(-1j * np.angle(der.sum()))
    psi = np.angle(der * off)
    w = np.abs(hh)
    sw, swk, swp, swkk, swkp = w.sum(), (w * kk).sum(), (w * psi).sum(), (w * kk * kk).sum(), (w * kk * psi).sum()
    den = sw * swkk - swk * swk
    refine = (sw * swkp - swk * swp) / den if den > 0 else 0.0
    per = -(coarse + refine)
    if per < 0:
        per += 2 * np.pi
    return per * C / (4 * np.pi * DF), abs(pair_sum) / pair_w


# ---------------------------------------------------------------- delay profile
GRID = np.arange(0.0, 60.0, 0.05)


def profile(h):
    ks = np.array(sorted(h))
    hh = np.array([h[k] for k in ks])
    # Hann over the channel span, like the crate
    lo, hi = ks.min(), ks.max()
    w = 0.5 - 0.5 * np.cos(2 * np.pi * (ks - lo + 0.5) / (hi - lo + 1))
    f = F0 + ks * DF
    steer = np.exp(1j * 4 * np.pi * np.outer(GRID, f) / C)  # (grid, k)
    p = np.abs(steer @ (w * hh))
    p /= p.max() if p.max() > 0 else 1
    peaks = [(GRID[i], p[i]) for i in range(1, len(p) - 1) if p[i] >= p[i - 1] and p[i] > p[i + 1]]
    peaks.sort(key=lambda t: -t[1])
    return peaks[:8]


# ---------------------------------------------------------------- super-resolution helpers
def vec(h):
    """75-slot vector for channels 2..76 and a presence mask."""
    x = np.zeros(N_SLOTS, complex)
    m = np.zeros(N_SLOTS, bool)
    for k, v in h.items():
        if 2 <= k <= 76:
            x[k - 2] = v
            m[k - 2] = True
    return x, m


def longest_run(m):
    best = (0, 0)
    i = 0
    while i < len(m):
        if m[i]:
            j = i
            while j < len(m) and m[j]:
                j += 1
            if j - i > best[1] - best[0]:
                best = (i, j)
            i = j
        else:
            i += 1
    return best


def pick_order(s, thresh, cap):
    return int(max(1, min(cap, np.sum(s > thresh * s[0]))))


def matrix_pencil(x, order=None, thresh=0.05, cap=8, L=None):
    N = len(x)
    L = L or N // 2
    Y = np.array([x[i : i + L + 1] for i in range(N - L)])
    U, s, Vh = np.linalg.svd(Y, full_matrices=False)
    M = order or pick_order(s, thresh, cap)
    V = Vh.conj().T[:, :M]
    V1, V2 = V[:-1], V[1:]
    # Y1 = U S V1^H, Y2 = U S V2^H: the poles are the eigenvalues of (V1^H)^+ V2^H, i.e. the
    # conjugates of those of V1^+ V2.
    z = np.conj(np.linalg.eigvals(np.linalg.pinv(V1) @ V2))
    n = np.arange(N)
    Z = np.power.outer(z, n).T  # (N, M)
    a, *_ = np.linalg.lstsq(Z, x, rcond=None)
    return z, a, s[: min(10, len(s))]


def hankel(y, L):
    return np.array([y[i : i + L + 1] for i in range(len(y) - L)])


def cadzow_fill(x, m, rank, iters=40, L=None):
    """Fill the missing slots so the Hankel matrix has the given rank (Cadzow iterations: truncate,
    average the anti-diagonals, put the known samples back)."""
    N = len(x)
    L = L or N // 2
    idx = np.arange(N)
    y = x.copy()
    y[~m] = np.interp(idx[~m], idx[m], x[m].real) + 1j * np.interp(idx[~m], idx[m], x[m].imag)
    for _ in range(iters):
        U, s, Vh = np.linalg.svd(hankel(y, L), full_matrices=False)
        Yr = (U[:, :rank] * s[:rank]) @ Vh[:rank]
        z = np.zeros(N, complex)
        cnt = np.zeros(N)
        for i in range(N - L):
            z[i : i + L + 1] += Yr[i]
            cnt[i : i + L + 1] += 1
        y = z / cnt
        y[m] = x[m]
    return y


def root_music(x, order, L=None):
    N = len(x)
    L = L or (2 * N) // 3
    K = N - L + 1
    X = np.array([x[k : k + L] for k in range(K)]).T  # (L, K)
    R = X @ X.conj().T / K
    J = np.fliplr(np.eye(L))
    R = (R + J @ R.conj() @ J) / 2
    w, E = np.linalg.eigh(R)  # ascending
    En = E[:, : L - order]
    Cm = En @ En.conj().T
    coeffs = [np.trace(Cm, offset=mm) for mm in range(L - 1, -L, -1)]  # z^(2L-2) .. z^0
    roots = np.roots(coeffs)
    inside = roots[np.abs(roots) < 1]
    inside = inside[np.argsort(-np.abs(inside))][:order]
    n = np.arange(N)
    Z = np.power.outer(inside, n).T
    a, *_ = np.linalg.lstsq(Z, x, rcond=None)
    return inside, a, w[::-1][:10]


def components(z, a, frac, radius):
    """[(d, |a| rel, |z|)] of the components that count as paths, shortest first."""
    amp = np.abs(a)
    top = amp.max() if amp.size else 0
    out = []
    for zi, ai in zip(z, amp):
        d = float(d_from_pole(zi))
        if top > 0 and ai >= frac * top and abs(abs(zi) - 1) <= radius and d <= UNAMBIGUOUS_M / 2:
            out.append((d, float(ai / top), float(abs(zi))))
    out.sort()
    return out


# ---------------------------------------------------------------- driver
def stats(v):
    v = np.array(sorted(v), float)
    if v.size == 0:
        return f"{'-':^17}"
    return f"{np.median(v):5.2f}[{v[int(0.1 * (v.size - 1))]:4.1f}-{v[int(0.9 * (v.size - 1))]:5.1f}]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--min-channels", type=int, default=55)
    ap.add_argument("--frac", type=float, default=0.2, help="a path is at least this fraction of the strongest")
    ap.add_argument("--radius", type=float, default=0.1, help="pole radius tolerance around 1")
    ap.add_argument("--orders", default="2,3,4,5", help="model orders to run; 0 = by singular values")
    ap.add_argument("--order-thresh", type=float, default=0.1, help="order 0: count singular values above this × the largest")
    ap.add_argument("--order-cap", type=int, default=5)
    ap.add_argument("--min-distance", type=float, default=1.2, help="components below this are not paths (the device offset)")
    ap.add_argument("--fill-rank", type=int, default=4, help="rank for the Cadzow completion")
    ap.add_argument("--ifft-early", type=float, default=0.3, help="earliest profile peak above this")
    ap.add_argument("--node", default=None, help="only this node number, e.g. N005")
    ap.add_argument("--dump", default=None, help="procedure:path to print in full")
    ap.add_argument("--csv", default=None, help="write every estimate to this CSV")
    args = ap.parse_args()
    orders = [int(o) for o in args.orders.split(",")]

    csv = open(args.csv, "w") if args.csv else None
    if csv:
        print("file,node,proc,path,channels,slope,coherence,ifft_strong,ifft_early," +
              ",".join(f"mp{o}_early,mp{o}_strong,music{o}_early,music{o}_strong" for o in orders), file=csv)
    for path in args.files:
        groups, names = load(path)
        rows = []
        for (node, proc, ant), h in sorted(groups.items()):
            name = names.get(node, str(node))
            if args.node and name != args.node:
                continue
            if len(h) < args.min_channels:
                continue
            x, m = vec(h)
            ps = phase_slope(h)
            peaks = profile(h)
            y = cadzow_fill(x, m, args.fill_rank)
            early_ifft = min((d for d, p in peaks if p >= args.ifft_early), default=None)
            row = dict(
                node=name, proc=proc, ant=ant, n=len(h),
                slope=ps[0] if ps else None, coh=ps[1] if ps else None,
                ifft_strong=peaks[0][0] if peaks else None, ifft_early=early_ifft,
            )
            dump = args.dump == f"{proc}:{ant}"
            if dump:
                print(f"== {name} procedure {proc} path {ant}: {len(h)} channels")
                print("  slope %.2f m (coherence %.3f)" % ps if ps else "  slope -")
                print("  profile peaks:", ", ".join(f"{d:.2f} ({p:.2f})" for d, p in peaks))
                _, sv, _ = np.linalg.svd(hankel(y, N_SLOTS // 2), full_matrices=False)
                print("  singular values after completion:", " ".join(f"{v / sv[0]:.3f}" for v in sv[:10]))
            for o in orders:
                oo = o
                if o == 0:
                    _, sv, _ = np.linalg.svd(hankel(y, N_SLOTS // 2), full_matrices=False)
                    oo = int(max(2, min(args.order_cap, np.sum(sv > args.order_thresh * sv[0]))))
                z, a, _ = matrix_pencil(y, order=oo)
                mp = [c for c in components(z, a, args.frac, args.radius) if c[0] >= args.min_distance]
                zm, am, _ = root_music(y, oo)
                mu = [c for c in components(zm, am, args.frac, args.radius) if c[0] >= args.min_distance]
                row[f"mp{o}_early"] = mp[0][0] if mp else None
                row[f"mp{o}_strong"] = max(mp, key=lambda t: t[1])[0] if mp else None
                row[f"music{o}_early"] = mu[0][0] if mu else None
                row[f"music{o}_strong"] = max(mu, key=lambda t: t[1])[0] if mu else None
                if dump:
                    fmt = lambda cs: ", ".join(f"{d:.2f} m a={am_:.2f} |z|={r:.3f}" for d, am_, r in cs)
                    print(f"  order {o}: matrix pencil: {fmt(sorted(components(z, a, 0.0, 9.0)))}")
                    print(f"  order {o}: root-MUSIC:    {fmt(sorted(components(zm, am, 0.0, 9.0)))}")
            if dump:
                ks = sorted(h)
                d0 = ps[0] if ps else 0.0
                res = {k: np.angle(h[k] * np.exp(1j * 4 * np.pi * (F0 + k * DF) * d0 / C)) for k in ks}
                off = np.angle(sum(np.exp(1j * r) for r in res.values()))
                print("  |H| per channel:", " ".join(f"{k}:{abs(h[k]):.0f}" for k in ks))
                print("  phase residual after the slope (rad):", " ".join(f"{k}:{np.angle(np.exp(1j * (res[k] - off))):+.2f}" for k in ks))
                print()
            rows.append(row)
            if csv:
                vals = [path, name, proc, ant, len(h), row["slope"], row["coh"], row["ifft_strong"], row["ifft_early"]]
                for o in orders:
                    vals += [row[f"mp{o}_early"], row[f"mp{o}_strong"], row[f"music{o}_early"], row[f"music{o}_strong"]]
                print(",".join("" if v is None else (f"{v:.3f}" if isinstance(v, float) else str(v)) for v in vals), file=csv)
        if not rows:
            continue
        short = path.split("/")[-1].replace(".jsonl.gz", "")
        for kind in ("early", "strong"):
            print(f"### {short}: shortest path ({kind}) — median[p10-p90], {len(rows)} estimates" if kind == "early"
                  else f"### {short}: strongest path")
            cols = ["slope", f"ifft_{kind}"] + [f"mp{o}_{kind}" for o in orders] + [f"music{o}_{kind}" for o in orders]
            print(f"{'who':13}" + "".join(f"{c:^18}" for c in cols))
            nodes = sorted({r["node"] for r in rows})
            for node in nodes:
                sel = [r for r in rows if r["node"] == node]
                ants = sorted({r["ant"] for r in sel})
                for ant in ants + ["all"]:
                    s_ = sel if ant == "all" else [r for r in sel if r["ant"] == ant]
                    line = f"{node + ' p' + str(ant):10}{len(s_):3d}"
                    for c in cols:
                        line += " " + stats([r[c] for r in s_ if r[c] is not None])
                    print(line)
            print()


def selftest():
    """Two paths at 5 and 9 m (amplitudes 1 and 0.5) plus noise, 10 random channels missing."""
    rng = np.random.default_rng(1)
    h = {}
    for k in CS_CHANNELS:
        f = F0 + k * DF
        v = 1000 * (np.exp(-1j * 4 * np.pi * f * 5.0 / C) + 0.5 * np.exp(-1j * 4 * np.pi * f * 9.0 / C))
        h[k] = v + 20 * (rng.standard_normal() + 1j * rng.standard_normal())
    for k in rng.choice(CS_CHANNELS, 10, replace=False):
        del h[k]
    print("selftest: slope %.2f coherence %.3f" % phase_slope(h))
    print("selftest: profile peaks", ", ".join(f"{d:.2f} ({p:.2f})" for d, p in profile(h)[:4]))
    x, m = vec(h)
    y = cadzow_fill(x, m, 4)
    for o in (2, 3, 4):
        z, a, _ = matrix_pencil(y, order=o)
        zm, am, _ = root_music(y, o)
        fmt = lambda cs: ", ".join(f"{d:.2f} m a={am_:.2f} |z|={r:.3f}" for d, am_, r in cs)
        print(f"selftest order {o}: pencil {fmt(sorted(components(z, a, 0.0, 9.0)))}")
        print(f"selftest order {o}: music  {fmt(sorted(components(zm, am, 0.0, 9.0)))}")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        selftest()
    else:
        main()
