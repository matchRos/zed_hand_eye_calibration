import cv2
import numpy as np

image_path = "left0000.jpg"

img = cv2.imread(image_path)
if img is None:
    raise RuntimeError(f"Could not load image: {image_path}")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
detector = cv2.aruco.ArucoDetector(aruco_dict)

corners, ids, _ = detector.detectMarkers(img)

if ids is None:
    print("No markers detected")
    raise SystemExit

ids = ids.flatten()

# Camera intrinsics
K = np.array([
    [366.14324951171875, 0.0, 322.98895263671875],
    [0.0, 366.14324951171875, 180.08743286132812],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

D = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# Board geometry
square_len = 0.02   # 20 mm
marker_len = 0.015  # 15 mm
margin = (square_len - marker_len) / 2.0

# 9 x 14 Charuco-like board => markers on alternating squares
# IDs appear to run from bottom-right (0) to top-left (62)
# We'll map each marker ID to a board square center.
obj_points = []
img_points = []

for marker_corners, marker_id in zip(corners, ids):
    marker_id = int(marker_id)

    # 7 marker columns x 9 marker rows = 63 markers
    marker_col_from_right = marker_id % 7
    marker_row_from_bottom = marker_id // 7

    # convert to full checkerboard square indices
    col = 8 - (2 * marker_col_from_right)   # 8,6,4,2,0, ... mirrored
    row = 13 - (2 * marker_row_from_bottom) # 13,11,9,... mirrored

    x0 = col * square_len + margin
    y0 = row * square_len + margin

    marker_obj = np.array([
        [x0,              y0,               0.0],
        [x0 + marker_len, y0,               0.0],
        [x0 + marker_len, y0 + marker_len,  0.0],
        [x0,              y0 + marker_len,  0.0],
    ], dtype=np.float32)

    obj_points.append(marker_obj)
    img_points.append(marker_corners[0].astype(np.float32))

obj_points = np.concatenate(obj_points, axis=0)
img_points = np.concatenate(img_points, axis=0)

ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, D)

print("solvePnP ok:", ok)
print("rvec:\n", rvec)
print("tvec:\n", tvec)

out = img.copy()
if ok:
    cv2.drawFrameAxes(out, K, D, rvec, tvec, 0.05)

cv2.imshow("pose", out)
cv2.waitKey(0)
cv2.destroyAllWindows()
