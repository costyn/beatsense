#!/usr/bin/env python3
# SPDX-License-Identifier: EUPL-1.2
"""Plot the CSV written by analyze_wav: onsets, tempo, beat phase, confidence, levels, and (if the CSV has them) breakdowns and the groove level.

    python3 tools/plot.py song.csv [out.png] [--from S] [--to S]

--from / --to zoom into a time window (seconds). In a window shorter than 30 s the predicted beats (where beat_phase wraps) are
drawn as vertical lines through every panel. Needs matplotlib (pip install matplotlib). With an output file it saves instead of
opening a window.
"""
import argparse
import csv

# Breakdown thresholds of the analyzer: Config::breakdownEnter / breakdownExit in lib/beatsense/src/beatsense/analyzer.h
BREAKDOWN_ENTER = 0.4
BREAKDOWN_EXIT = 0.8


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("out", nargs="?", help="save to this image instead of opening a window")
    ap.add_argument("--from", dest="t0", type=float, default=None, metavar="S", help="window start (s)")
    ap.add_argument("--to", dest="t1", type=float, default=None, metavar="S", help="window end (s)")
    args = ap.parse_args()

    import matplotlib

    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(args.csv) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("empty CSV")
    t_all = [float(r["time_s"]) for r in rows]
    lo = args.t0 if args.t0 is not None else t_all[0]
    hi = args.t1 if args.t1 is not None else t_all[-1]
    # keep one row before the window so the first beat wrap inside it is detected
    first = max(0, next((i for i, t in enumerate(t_all) if t >= lo), len(t_all)) - 1)
    rows = [r for r, t in zip(rows[first:], t_all[first:]) if t <= hi]
    if not rows:
        raise SystemExit("no rows in the window")
    col = lambda name: [float(r[name]) for r in rows]
    t = col("time_s")
    short = (hi - lo) < 30.0

    # beat ticks: the phase sawtooth wraps from ~1 back to ~0
    phase = col("beat_phase")
    ticks = [t[i] for i in range(1, len(t)) if phase[i] < phase[i - 1] - 32768 and t[i] >= lo]

    # breakdown columns are missing in captures from before they existed
    has_bd = "breakdown" in rows[0] and "bd_ratio" in rows[0]
    npanels = 6 if has_bd else 5
    fig, ax = plt.subplots(npanels, 1, sharex=True, figsize=(12, 9 if not has_bd else 10.5))
    # fixed draw order low -> mid -> high, thin and transparent so overlapping series stay readable
    for name, color, z in (("onset_low", "tab:red", 1), ("onset_mid", "tab:green", 2), ("onset_high", "tab:blue", 3)):
        ax[0].plot(t, col(name), label=name, color=color, linewidth=0.6, alpha=0.6, zorder=z)
    ax[0].set_ylabel("onsets")
    ax[0].legend(loc="upper right", ncol=3)

    ax[1].plot(t, col("bpm"), color="black")
    ax[1].set_ylabel("BPM")
    bpms = [b for b in col("bpm") if b > 0]
    if bpms:  # keep a tiny wobble from filling the axis: show at least 10 BPM
        mid, span = (max(bpms) + min(bpms)) / 2, max(10.0, max(bpms) - min(bpms) + 2.0)
        ax[1].set_ylim(mid - span / 2, mid + span / 2)

    ax[2].plot(t, [p / 65536 for p in phase], color="tab:purple", linewidth=0.7)
    ax[2].set_ylabel("beat phase")

    ax[3].plot(t, [c / 255 for c in col("confidence")], label="confidence")
    ax[3].plot(t, [(int(float(s)) >> 1) & 1 for s in col("status")], label="locked", linestyle="--")
    ax[3].plot(t, [e / 255 for e in col("energy")], label="energy", alpha=0.6)
    ax[3].legend(loc="upper right", ncol=3)

    for name, color, z in (("level_low", "tab:red", 1), ("level_mid", "tab:green", 2), ("level_high", "tab:blue", 3)):
        ax[4].plot(t, col(name), label=name, color=color, linewidth=0.6, alpha=0.6, zorder=z)
    ax[4].set_ylabel("levels")
    ax[4].legend(loc="upper right", ncol=3)

    if has_bd:
        ax[5].plot(t, col("bd_ratio"), color="tab:orange", linewidth=0.8)
        ax[5].axhline(BREAKDOWN_ENTER, color="tab:red", linestyle="--", linewidth=0.8, label="enter %.1f" % BREAKDOWN_ENTER)
        ax[5].axhline(BREAKDOWN_EXIT, color="tab:green", linestyle="--", linewidth=0.8, label="exit %.1f" % BREAKDOWN_EXIT)
        ax[5].set_ylim(0, 1.5)
        ax[5].set_ylabel("low short/long, groove")
        # groove level (0-255 in the frame) drawn as 0-1 on the same axis; CSVs from before it existed simply don't have the column
        if "groove" in rows[0]:
            ax[5].plot(t, [g / 255 for g in col("groove")], color="tab:blue", linewidth=1.0, label="groove")
        ax[5].legend(loc="upper right", ncol=3)
        # shade the breakdown segments (runs of breakdown == 1) in every panel
        flags = [int(float(v)) for v in col("breakdown")]
        start = None
        for i, fl in enumerate(flags + [0]):
            if fl and start is None:
                start = t[i]
            elif not fl and start is not None:
                end = t[i] if i < len(t) else t[-1]
                for a in ax:
                    a.axvspan(start, end, color="tab:orange", alpha=0.12, linewidth=0, zorder=0)
                start = None
    ax[-1].set_xlabel("time (s)")

    if short:
        for a in ax:
            for x in ticks:
                a.axvline(x, color="grey", alpha=0.35, linewidth=0.6, zorder=0)
    ax[0].set_xlim(lo, hi)

    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=110)
    else:
        plt.show()


if __name__ == "__main__":
    main()
