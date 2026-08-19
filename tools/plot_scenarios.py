#!/usr/bin/env python3
"""Render scenario traces to PNG for the README.

    python3 tools/plot_scenarios.py --out docs/img
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "python"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import omni_drive_core as odc  # noqa: E402
from odc_sim import scenarios  # noqa: E402

INK = "#1b1f24"
ACCENT = "#c8471f"
MUTED = "#8a8f98"
GRID = "#e3e5e8"
BANDS = {
    int(odc.DriveState.SPEED_LIMITED): ("#f5c518", "speed limited"),
    int(odc.DriveState.PROTECTIVE_STOP): ("#e8825a", "protective stop"),
    int(odc.DriveState.FAULT): ("#c0392b", "fault"),
}


def style(ax, title=None, xlabel=None, ylabel=None):
    ax.set_facecolor("white")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
    ax.grid(True, color=GRID, linewidth=0.7)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=8)
    if title:
        ax.set_title(title, color=INK, fontsize=10, loc="left", pad=8)
    if xlabel:
        ax.set_xlabel(xlabel, color=MUTED, fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, color=MUTED, fontsize=8)


def shade_states(ax, r):
    """Highlight every interval where the core was not in normal operation."""
    seen = set()
    for value, (colour, label) in BANDS.items():
        mask = r.state == value
        if not mask.any():
            continue
        edges = np.diff(mask.astype(int))
        starts = list(np.nonzero(edges == 1)[0] + 1)
        ends = list(np.nonzero(edges == -1)[0] + 1)
        if mask[0]:
            starts.insert(0, 0)
        if mask[-1]:
            ends.append(len(mask) - 1)
        for a, b in zip(starts, ends, strict=False):
            ax.axvspan(r.t[a], r.t[b], color=colour, alpha=0.18,
                       label=label if label not in seen else None)
            seen.add(label)


def plot_tracking(r, name, path):
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))

    ax = axes[0]
    ax.plot(r.ref_pose[:, 0], r.ref_pose[:, 1], color=MUTED, lw=2.2,
            label="reference")
    ax.plot(r.true_pose[:, 0], r.true_pose[:, 1], color=INK, lw=1.4,
            label="ground truth")
    ax.plot(r.est_pose[:, 0], r.est_pose[:, 1], color=ACCENT, lw=1.0, ls="--",
            label="EKF estimate")
    ax.set_aspect("equal", adjustable="datalim")
    style(ax, "path", "x [m]", "y [m]")
    ax.legend(frameon=False, fontsize=7)

    ax = axes[1]
    shade_states(ax, r)
    ax.plot(r.t, r.position_error, color=INK, lw=1.2, label="tracking error")
    ax.plot(r.t, r.estimator_error, color=ACCENT, lw=1.0, label="estimator error")
    style(ax, "error", "t [s]", "[m]")
    ax.legend(frameon=False, fontsize=7)

    ax = axes[2]
    shade_states(ax, r)
    for i, lbl in enumerate(("FL", "FR", "RL", "RR")):
        ax.plot(r.t, r.wheel_omega[:, i], lw=0.9, label=lbl)
    style(ax, "wheel speeds", "t [s]", "[rad/s]")
    ax.legend(frameon=False, fontsize=7, ncol=4)

    fig.suptitle(name, color=INK, fontsize=11, x=0.008, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=130, facecolor="white")
    plt.close(fig)


def plot_fault(r, name, path):
    fig, axes = plt.subplots(1, 3, figsize=(13, 3.6))

    ax = axes[0]
    shade_states(ax, r)
    ax.plot(r.t, np.abs(r.consistency), color=INK, lw=1.1)
    ax.axhline(odc.SlipDetectorParams().kin_enter, color=ACCENT, ls=":", lw=1.2)
    ax.text(r.t[-1], odc.SlipDetectorParams().kin_enter, " trip", color=ACCENT,
            fontsize=7, va="center")
    style(ax, "kinematic inconsistency  |w_FL + w_FR - w_RL - w_RR| / 2",
          "t [s]", "[rad/s]")

    ax = axes[1]
    shade_states(ax, r)
    for i, lbl in enumerate(("FL", "FR", "RL", "RR")):
        ax.plot(r.t, np.abs(r.current[:, i]), lw=0.9, label=lbl)
    style(ax, "motor current", "t [s]", "[A]")
    ax.legend(frameon=False, fontsize=7, ncol=4)

    ax = axes[2]
    shade_states(ax, r)
    ax.plot(r.t, r.speed_scale, color=INK, lw=1.3, label="speed scale")
    ax.plot(r.t, np.linalg.norm(r.cmd_twist[:, :2], axis=1), color=ACCENT,
            lw=1.1, label="|v| commanded")
    style(ax, "safety response", "t [s]", "")
    ax.legend(frameon=False, fontsize=7)

    fig.suptitle(name, color=INK, fontsize=11, x=0.008, ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(path, dpi=130, facecolor="white")
    plt.close(fig)


TRACKING = {"figure_eight", "lateral_strafe", "straight_line", "spin_in_place"}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=ROOT / "docs" / "img")
    ap.add_argument("--only", nargs="*", default=None)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    suite = scenarios.all_scenarios()
    if args.only:
        suite = [s for s in suite if s.name in args.only]

    for sc in suite:
        r = sc.execute(seed=0)
        path = args.out / f"{sc.name}.png"
        (plot_tracking if sc.name in TRACKING else plot_fault)(r, sc.name, path)
        print(f"wrote {path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
