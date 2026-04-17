"""
Perception module based on Semantic Segmentation and IPM.
Transforms the camera view into a Bird's Eye View (BEV) map.
"""

import numpy as np
import cv2
import carla
from config import IPM_SRC_POINTS_NORM, IPM_DST_POINTS_NORM, OBSTACLE_LABEL_IDS

class SegmentationPerception:
    def __init__(self, img_w, img_h):
        self.img_w = img_w
        self.img_h = img_h

        # Compute the perspective transform matrix (image -> BEV).
        # Use (w-1, h-1) so normalized coordinates never map to out-of-bounds.
        src = self._points_norm_to_px(IPM_SRC_POINTS_NORM)
        dst = self._points_norm_to_px(IPM_DST_POINTS_NORM)
        self.M = cv2.getPerspectiveTransform(src, dst)
        self.last_bev_map = None

        self.obstacle_ids = list(OBSTACLE_LABEL_IDS)
        self._label_channel = None
        self._color_lut = self._build_color_lut()
        self.last_debug = {
            "label_channel": None,
            "mask_nonzero": 0,
        }
        self.last_mask = None
        self.last_labels = None
        self.last_bev_visual = None

    def _build_color_lut(self):
        """
        Build a BGR color lookup table indexed by CARLA 0.9.14+ semantic label ID.
        Obstacle labels are mapped to bright red so they stand out in the BEV display.
        Non-obstacle labels use muted colors so the road structure is always visible.
        """
        lut = np.zeros((256, 3), dtype=np.uint8)
        lut[1]  = (100, 60, 100)   # Roads      - dark purple-gray
        lut[2]  = (90,  90, 90)    # Sidewalks  - medium gray
        lut[3]  = (50,  50, 50)    # Buildings  - very dark gray
        lut[4]  = (80,  80, 110)   # Walls      - dark blue-gray
        lut[5]  = (120, 100, 100)  # Fences     - muted brown
        lut[6]  = (100, 100, 100)  # Poles      - gray
        lut[7]  = (30,  160, 240)  # TrafficLight  - amber (BGR)
        lut[8]  = (0,   200, 200)  # TrafficSigns  - yellow (BGR)
        lut[9]  = (0,   100, 0)    # Vegetation    - dark green
        lut[10] = (80,  180, 80)   # Terrain       - light green
        lut[11] = (160, 120, 60)   # Sky           - light blue (BGR)
        lut[24] = (200, 200, 200)  # RoadLines     - light gray
        # All obstacle labels map to bright red (BGR: R dominant)
        for obs_id in self.obstacle_ids:
            lut[obs_id] = (0, 0, 255)
        return lut

    def _points_norm_to_px(self, points_norm):
        w = float(self.img_w - 1)
        h = float(self.img_h - 1)
        return np.float32([[p[0] * w, p[1] * h] for p in points_norm])

    def _select_label_channel(self, bgra):
        """
        Identify which BGRA channel carries semantic label IDs.

        CARLA documentation states that with ColorConverter.Raw the semantic tag
        is stored in the Red channel. In a BGRA numpy array that is channel index 2.
        As a safety net, we also evaluate channels 0 and 1: if any other channel
        contains more obstacle-ID hits, we use that instead.  The result is cached
        after the first call so there is no per-frame overhead.

        We also test for common non-obstacle labels (Roads=1, Buildings=3,
        Vegetation=9, Sky=11) to identify the label channel even when no
        obstacles are currently visible.
        """
        # Non-obstacle IDs that are always present in any typical CARLA scene.
        probe_ids = list(self.obstacle_ids) + [1, 3, 9, 11]
        best_channel = 2  # R in BGRA -- CARLA documented default
        best_hits = -1
        for ch in (2, 0, 1):  # try R first, then B and G as fallback
            labels = bgra[:, :, ch]
            hits = int(np.count_nonzero(np.isin(labels, probe_ids)))
            if hits > best_hits:
                best_hits = hits
                best_channel = ch
        return best_channel

    def on_image(self, image):
        """
        Callback for the semantic segmentation camera.
        Processes the raw frame into a BEV binary obstacle map.
        """
        image.convert(carla.ColorConverter.Raw)
        data = np.frombuffer(image.raw_data, dtype=np.uint8)
        data = data.reshape((self.img_h, self.img_w, 4))

        if self._label_channel is None:
            self._label_channel = self._select_label_channel(data)
        labels = data[:, :, self._label_channel]
        self.last_labels = labels.copy()

        # Binary obstacle mask: obstacle pixels = 255, everything else = 0.
        mask = (np.isin(labels, self.obstacle_ids).astype(np.uint8) * 255)
        self.last_mask = mask.copy()
        self.last_debug["label_channel"] = int(self._label_channel)
        self.last_debug["mask_nonzero"] = int(np.count_nonzero(mask))

        warp_kwargs = dict(dsize=(self.img_w, self.img_h), flags=cv2.INTER_NEAREST)

        # Supervisor input: binary obstacle BEV (single channel).
        self.last_bev_map = cv2.warpPerspective(mask, self.M, **warp_kwargs)

        # Visualization: colored semantic BEV so the window is never all-black.
        color_img = self._color_lut[labels]          # (H, W, 3) BGR
        self.last_bev_visual = cv2.warpPerspective(color_img, self.M, **warp_kwargs)

    def get_bev_map(self):
        """Return the latest binary obstacle BEV map (used by the supervisor)."""
        return self.last_bev_map

    def get_bev_visual(self):
        """
        Return a colored BGR BEV image for display.
        Road labels are shown in muted colors; obstacle labels in bright red.
        This image is for visualization only and is never used by the supervisor.
        """
        return self.last_bev_visual