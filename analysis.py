import csv
import matplotlib.pyplot as plt
import os

CSV_PATH = "run_metrics.csv"
OUTPUT_DIR = "analysis_results"

# Create output directory if it does not exist
os.makedirs(OUTPUT_DIR, exist_ok=True)

time_sec = []
obstacle_ratio = []
speed = []
override = []

with open(CSV_PATH, newline="", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time_sec.append(float(row["time_sec"]))
        obstacle_ratio.append(float(row["obstacle_ratio"]))
        speed.append(float(row["speed_mps"]))
        override.append(int(row["override"]))


# --------------------------------------------------
# Graph 1: Obstacle Ratio over Time
# --------------------------------------------------
plt.figure()
plt.plot(time_sec, obstacle_ratio)
plt.xlabel("Time (seconds)")
plt.ylabel("Obstacle ratio")
plt.title("Obstacle ratio over time")

graph1_path = os.path.join(OUTPUT_DIR, "obstacle_ratio_over_time.png")
plt.savefig(graph1_path, dpi=300, bbox_inches="tight")
plt.show()


# --------------------------------------------------
# Graph 2: Speed over Time with Safety Overrides
# --------------------------------------------------
plt.figure()
plt.plot(time_sec, speed, label="Speed (m/s)")

# Mark safety override events
override_times = [t for t, o in zip(time_sec, override) if o == 1]
override_speeds = [s for s, o in zip(speed, override) if o == 1]

plt.scatter(override_times, override_speeds, label="Safety override")
plt.xlabel("Time (seconds)")
plt.ylabel("Speed (m/s)")
plt.title("Vehicle speed with safety overrides")
plt.legend()

graph2_path = os.path.join(OUTPUT_DIR, "speed_with_safety_overrides.png")
plt.savefig(graph2_path, dpi=300, bbox_inches="tight")
plt.show()


print("Analysis completed.")
print(f"Saved graphs to folder: {OUTPUT_DIR}")
print(f"- {graph1_path}")
print(f"- {graph2_path}")
