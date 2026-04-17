"""
Analysis module – generates thesis-quality plots from run_metrics.csv.

Graphs produced:
  (a) speed_with_safety_overrides.png  – Speed vs Time with shaded override spans.
  (b) roi_lookahead_over_time.png      – Dynamic ROI look-ahead vs Time (shows
                                         how the safety zone expands with speed).
  (c) distance_to_obstacle_vs_speed.png – Nearest obstacle distance vs Speed,
                                          colour-coded by ROI look-ahead to
                                          visualise the expansion relationship.
"""

import csv
import os
import matplotlib
matplotlib.use("Agg")           # non-interactive backend; safe on all platforms
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

CSV_PATH = "run_metrics.csv"
OUTPUT_DIR = "analysis_results"

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── load data ────────────────────────────────────────────────────────────────
time_sec = []
speed = []
override = []
collision = []
roi_lookahead_px = []
min_obstacle_dist_px = []

with open(CSV_PATH, newline="", encoding="utf-8") as f:
    for row in csv.DictReader(f):
        time_sec.append(float(row["time_sec"]))
        speed.append(float(row["speed_mps"]))
        override.append(int(row["override"]))
        collision.append(int(row.get("collision", 0)))
        roi_lookahead_px.append(int(row.get("roi_lookahead_px", 0)))
        dist_raw = row.get("min_obstacle_dist_px", "")
        min_obstacle_dist_px.append(None if dist_raw in ("", None) else float(dist_raw))

time_arr = np.array(time_sec)
speed_arr = np.array(speed)
override_arr = np.array(override)
roi_arr = np.array(roi_lookahead_px, dtype=float)


def _shade_override_spans(ax, times, overrides, color="red", alpha=0.15):
    """Draw translucent vertical bands for every contiguous override period."""
    in_span = False
    t_start = None
    for t, ov in zip(times, overrides):
        if ov and not in_span:
            t_start = t
            in_span = True
        elif not ov and in_span:
            ax.axvspan(t_start, t, color=color, alpha=alpha, label="_nolegend_")
            in_span = False
    if in_span:
        ax.axvspan(t_start, times[-1], color=color, alpha=alpha, label="_nolegend_")


# ── Graph (a): Speed vs Time with override spans ──────────────────────────────
fig, ax = plt.subplots(figsize=(11, 4))

ax.plot(time_arr, speed_arr, linewidth=1.8, color="steelblue", label="Speed (m/s)")

_shade_override_spans(ax, time_arr, override_arr)

# Scatter override START points (rising edge) as large red triangles.
override_starts = [
    i for i in range(1, len(override_arr))
    if override_arr[i] == 1 and override_arr[i - 1] == 0
]
if override_starts:
    ax.scatter(
        time_arr[override_starts], speed_arr[override_starts],
        marker="^", s=80, zorder=5, label="Override triggered",
    )

# Scatter collision events.
col_times = [t for t, c in zip(time_arr, collision) if c == 1]
col_speeds = [s for s, c in zip(speed_arr, collision) if c == 1]
if col_times:
    ax.scatter(col_times, col_speeds, marker="o", s=60,
               facecolors="none", linewidths=1.5,
               label="Collision event")

span_patch = mpatches.Patch(color="red", alpha=0.25, label="Override active (braking)")
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles=handles + [span_patch], loc="upper right", fontsize=9)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Speed (m/s)")
ax.set_title("Speed vs. Time — Safety Layer Override Events")
ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
fig.tight_layout()
graph_a = os.path.join(OUTPUT_DIR, "speed_with_safety_overrides.png")
fig.savefig(graph_a, dpi=300, bbox_inches="tight")
plt.close(fig)


# ── Graph (b): Dynamic ROI look-ahead vs Time ─────────────────────────────────
fig, ax1 = plt.subplots(figsize=(11, 4))

ax1.plot(time_arr, roi_arr, linewidth=1.5, color="darkorange",
         label="ROI look-ahead (px)")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("ROI look-ahead (px)", color="darkorange")
ax1.tick_params(axis="y", labelcolor="darkorange")

ax2 = ax1.twinx()
ax2.plot(time_arr, speed_arr, linewidth=1.2, color="steelblue",
         alpha=0.6, label="Speed (m/s)")
ax2.set_ylabel("Speed (m/s)", color="steelblue")
ax2.tick_params(axis="y", labelcolor="steelblue")

_shade_override_spans(ax1, time_arr, override_arr)

lines1, lab1 = ax1.get_legend_handles_labels()
lines2, lab2 = ax2.get_legend_handles_labels()
span_patch = mpatches.Patch(color="red", alpha=0.25, label="Override active")
ax1.legend(lines1 + lines2 + [span_patch], lab1 + lab2 + ["Override active"],
           loc="upper right", fontsize=9)

ax1.set_title("Dynamic ROI Look-ahead vs. Time\n"
              r"$d \approx v\,t_r + v^2/(2a)$ approximated as linear pixel look-ahead")
ax1.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
fig.tight_layout()
graph_b = os.path.join(OUTPUT_DIR, "roi_lookahead_over_time.png")
fig.savefig(graph_b, dpi=300, bbox_inches="tight")
plt.close(fig)


# ── Graph (c): Distance to Obstacle vs Speed ──────────────────────────────────
f_speed, f_dist, f_la = [], [], []
for s, d, la in zip(speed_arr, min_obstacle_dist_px, roi_arr):
    if d is not None:
        f_speed.append(float(s))
        f_dist.append(float(d))
        f_la.append(float(la))

fig, ax = plt.subplots(figsize=(6, 5))
if f_speed:
    sc = ax.scatter(f_speed, f_dist, c=f_la, cmap="viridis",
                    s=20, alpha=0.8)
    cbar = fig.colorbar(sc, ax=ax)
    cbar.set_label("ROI look-ahead (px)")
else:
    ax.text(0.5, 0.5, "No obstacle distance samples in this run.\n"
            "Run a scenario with an obstacle in the forward lane.",
            ha="center", va="center", transform=ax.transAxes, fontsize=9)

ax.set_xlabel("Speed (m/s)")
ax.set_ylabel("Distance to nearest obstacle (px)")
ax.set_title("Distance to Obstacle vs. Speed\n"
             "(colour = ROI look-ahead — shows safety-zone expansion)")
ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
fig.tight_layout()
graph_c = os.path.join(OUTPUT_DIR, "distance_to_obstacle_vs_speed.png")
fig.savefig(graph_c, dpi=300, bbox_inches="tight")
plt.close(fig)


# ── Summary ───────────────────────────────────────────────────────────────────
n_override = int(override_arr.sum())
n_collision = int(sum(collision))
n_total = len(time_arr)
print("=" * 55)
print("Run summary")
print("=" * 55)
print(f"  Total log steps      : {n_total}")
print(f"  Override steps       : {n_override}  ({100*n_override/max(n_total,1):.1f} %)")
print(f"  Collision events     : {n_collision}")
print(f"  Max speed (m/s)      : {speed_arr.max():.2f}")
print(f"  Override start time  : "
      + (f"{time_arr[override_arr > 0][0]:.2f} s" if n_override else "n/a"))
print("=" * 55)
print(f"Saved graphs to: {OUTPUT_DIR}/")
print(f"  {graph_a}")
print(f"  {graph_b}")
print(f"  {graph_c}")
