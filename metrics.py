"""
Metrics logging module.

Logs driving data and safety events to a CSV file for later analysis.
Counts collisions and emergency overrides.
"""

import csv
import time


class MetricsLogger:
    def __init__(self, csv_path):
        self.csv_path = csv_path
        self.start_time = time.time()

        self.collision_count = 0
        self.last_collision_frame = -1

        self.emergency_override_count = 0  # number of timesteps where override was active

        self._file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow([
            "time_sec",
            "frame",
            "speed_mps",
            "obstacle_ratio",
            "override",
            "brake",
            "collision"
        ])

    def on_collision(self, event):
        """Callback for collision events."""
        self.collision_count += 1
        self.last_collision_frame = getattr(event, "frame", -1)

    def log_step(self, time_sec, frame, speed_mps, obstacle_ratio, override, brake, collision):
        """Write a single timestep to CSV and update counters."""
        if override:
            self.emergency_override_count += 1

        self._writer.writerow([
            f"{time_sec:.3f}",
            frame,
            f"{speed_mps:.3f}",
            f"{obstacle_ratio:.5f}",
            int(override),
            f"{brake:.3f}",
            int(collision)
        ])

    def close(self):
        """Close the CSV file."""
        self._file.close()
