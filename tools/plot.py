#!/usr/bin/env python3
# SPDX-License-Identifier: EUPL-1.2
"""Plot the CSV written by analyze_wav: onsets, tempo, beat phase, confidence, levels.

    python3 tools/plot.py song.csv [out.png]

Needs matplotlib (pip install matplotlib). With an output file it saves instead of opening a window.
"""
import csv
import sys


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    import matplotlib

    if len(sys.argv) > 2:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(sys.argv[1]) as f:
        rows = list(csv.DictReader(f))
    col = lambda name: [float(r[name]) for r in rows]
    t = col("time_s")

    fig, ax = plt.subplots(5, 1, sharex=True, figsize=(12, 9))
    for name, color in (("onset_low", "tab:red"), ("onset_mid", "tab:green"), ("onset_high", "tab:blue")):
        ax[0].plot(t, col(name), label=name, color=color, linewidth=0.7)
    ax[0].set_ylabel("onsets")
    ax[0].legend(loc="upper right", ncol=3)

    ax[1].plot(t, col("bpm"), color="black")
    ax[1].set_ylabel("BPM")

    # beat phase as a sawtooth; the vertical lines mark where the beat counter increments
    ax[2].plot(t, [p / 65536 for p in col("beat_phase")], color="tab:purple", linewidth=0.7)
    counts = col("beat_count")
    for i in range(1, len(t)):
        if counts[i] != counts[i - 1]:
            ax[2].axvline(t[i], color="grey", alpha=0.3, linewidth=0.5)
    ax[2].set_ylabel("beat phase")

    ax[3].plot(t, [c / 255 for c in col("confidence")], label="confidence")
    ax[3].plot(t, [(int(s) >> 1) & 1 for s in col("status")], label="locked", linestyle="--")
    ax[3].plot(t, [e / 255 for e in col("energy")], label="energy", alpha=0.6)
    ax[3].legend(loc="upper right", ncol=3)

    for name, color in (("level_low", "tab:red"), ("level_mid", "tab:green"), ("level_high", "tab:blue")):
        ax[4].plot(t, col(name), label=name, color=color, linewidth=0.7)
    ax[4].set_ylabel("levels")
    ax[4].set_xlabel("time (s)")

    fig.tight_layout()
    if len(sys.argv) > 2:
        fig.savefig(sys.argv[2], dpi=110)
    else:
        plt.show()


if __name__ == "__main__":
    main()
