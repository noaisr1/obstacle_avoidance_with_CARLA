"""
Perception module based on semantic segmentation.

This module processes semantic segmentation images and computes
an obstacle ratio within a predefined ROI.

Compatibility note:
Some CARLA versions do not provide CityObjectLabel.Vehicles.
In those versions, vehicles are split into Car/Truck/Bus/etc.
"""

import numpy as np
import carla


class SegmentationPerception:
    def __init__(self, img_w, img_h, roi_box):
        self.img_w = img_w
        self.img_h = img_h
        self.x0, self.x1, self.y0, self.y1 = roi_box

        # Build a set of label IDs that represent "obstacles"
        # Always include pedestrians, and include vehicles depending on CARLA version.
        self.obstacle_label_ids = set()

        # Pedestrians (usually exists as "Pedestrians")
        if hasattr(carla.CityObjectLabel, "Pedestrians"):
            self.obstacle_label_ids.add(int(getattr(carla.CityObjectLabel, "Pedestrians")))

        # Some CARLA versions expose a generic Vehicles label (not available in all versions)
        if hasattr(carla.CityObjectLabel, "Vehicles"):
            self.obstacle_label_ids.add(int(getattr(carla.CityObjectLabel, "Vehicles")))
        else:
            # Fallback for versions where vehicles are split into categories
            for name in ["Car", "Truck", "Bus", "Motorcycle", "Bicycle", "Rider", "Train"]:
                if hasattr(carla.CityObjectLabel, name):
                    self.obstacle_label_ids.add(int(getattr(carla.CityObjectLabel, name)))

        self.last_frame = -1
        self.last_ratio = 0.0

    def on_image(self, image: carla.Image):
        """
        Callback for segmentation camera.
        Computes obstacle ratio for the current frame.
        """
        # Raw conversion keeps per-pixel label id in the image data
        image.convert(carla.ColorConverter.Raw)

        data = np.frombuffer(image.raw_data, dtype=np.uint8)
        data = data.reshape((self.img_h, self.img_w, 4))  # BGRA

        # Semantic label id is stored in the R channel when using Raw
        labels = data[:, :, 2]

        x0 = int(self.x0 * self.img_w)
        x1 = int(self.x1 * self.img_w)
        y0 = int(self.y0 * self.img_h)
        y1 = int(self.y1 * self.img_h)

        roi = labels[y0:y1, x0:x1]

        if roi.size == 0 or not self.obstacle_label_ids:
            self.last_ratio = 0.0
            self.last_frame = image.frame
            return

        # Count pixels whose label id is in our obstacle set
        mask = np.isin(roi, list(self.obstacle_label_ids))
        self.last_ratio = float(mask.mean())
        self.last_frame = image.frame

    def get_obstacle_ratio(self):
        """Return the last processed frame and obstacle ratio."""
        return self.last_frame, self.last_ratio
