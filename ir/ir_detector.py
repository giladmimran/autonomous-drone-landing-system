"""One aggregate beacon track; brightness is not proof of infrared identity."""
import math

class IRDetector:
    def __init__(self, settings):
        self.s = settings
        self.center = None
        self.last_on = None
        self.previous = None
        self.on_centroids = []

    def process(self, frame, now):
        import cv2
        import numpy as np
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY).astype(np.float32)
        s = self.s
        motion_ok = False
        if self.previous is not None and self.center is not None:
            (dx, dy), response = cv2.phaseCorrelate(self.previous, gray)
            motion_ok = response >= s['motion_response'] and math.hypot(dx, dy) <= s['max_motion_pixels']
            if motion_ok:
                self.center = self.center[0] + dx, self.center[1] + dy
        self.previous = gray
        mask = (gray >= s['brightness_threshold']).astype(np.uint8) * 255
        count, labels, stats, centers = cv2.connectedComponentsWithStats(mask)
        candidates = [i for i in range(1, count) if s['min_blob_area'] <= stats[i, cv2.CC_STAT_AREA] <= s['max_blob_area']]
        # Initial acquisition requires exactly one spatial cluster. Reject ambiguity.
        if self.center is None:
            if not candidates:
                return self.result(None, 0, 0, False)
            seed = max(candidates, key=lambda i: stats[i, cv2.CC_STAT_AREA])
            center = centers[seed]
            if any(np.linalg.norm(centers[i] - center) > s['array_radius_pixels'] for i in candidates):
                return self.result(None, 0, 0, False)
            self.center = tuple(center)
        x, y = self.center
        yy, xx = np.ogrid[:gray.shape[0], :gray.shape[1]]
        radius = s['array_radius_pixels']
        inside = (xx-x)**2 + (yy-y)**2 <= radius**2
        ring = ((xx-x)**2 + (yy-y)**2 <= (2*radius)**2) & ~inside
        if not inside.any() or not ring.any():
            self.center = None
            return self.result(None, 0, 0, False)
        background = float(np.median(gray[ring]))
        intensity = float(np.percentile(gray[inside], s['intensity_percentile']))
        contrast = intensity - background
        selected = [i for i in candidates if np.linalg.norm(centers[i] - self.center) <= radius]
        if contrast >= s['on_contrast'] and selected:
            weights = np.array([stats[i, cv2.CC_STAT_AREA] for i in selected])
            self.center = tuple(np.average(centers[selected], axis=0, weights=weights))
            self.last_on = now
            self.on_centroids.append((now, self.center))
            self.on_centroids = self.on_centroids[-s['history_size']:]
            return self.result(True, 1.0, intensity, True)
        if (motion_ok and self.last_on is not None and now-self.last_on <= s['max_off_seconds']
                and contrast <= s['off_contrast']):
            return self.result(False, 1.0, intensity, False)
        if self.last_on is None or now-self.last_on > s['max_off_seconds']:
            self.center = None
        return self.result(None, 0, intensity, False)

    def result(self, state, quality, intensity, measured):
        return {'state': state, 'quality': quality, 'intensity': intensity,
                'centroid': self.center, 'measured': measured,
                'last_on': self.last_on, 'on_observations': len(self.on_centroids),
                'landing_valid': False}
