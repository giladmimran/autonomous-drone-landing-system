def image_bearing(centroid, camera_matrix, distortion):
    """Camera ray only: caller must validate calibration, freshness and extrinsics."""
    import cv2
    import numpy as np
    point = np.asarray(centroid, dtype=float).reshape(1, 1, 2)
    x, y = cv2.undistortPoints(point, camera_matrix, distortion)[0, 0]
    ray = np.array([x, y, 1.0])
    return ray / np.linalg.norm(ray)

def prepare_target():
    return {'valid': False, 'reason': 'Range, attitude, extrinsics and ground geometry unvalidated',
            'transmission_enabled': False}
