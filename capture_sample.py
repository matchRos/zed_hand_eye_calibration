#!/usr/bin/env python3
import os
import json
import cv2
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SampleCapture:
    def __init__(self):
        rospy.init_node("handeye_sample_capture")

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.latest_image = None
        self.sample_idx = 0

        self.image_topic = "/zedm/zed_node/left/image_rect_color"
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        self.output_dir = os.path.expanduser("~/handeye_samples")
        os.makedirs(self.output_dir, exist_ok=True)

        self.K = np.array([
            [366.14324951171875, 0.0, 322.98895263671875],
            [0.0, 366.14324951171875, 180.08743286132812],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict)

        self.square_len = 0.02
        self.marker_len = 0.015
        self.margin = (self.square_len - self.marker_len) / 2.0

    def image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def marker_id_to_object_points(self, marker_id):
        marker_col_from_right = marker_id % 7
        marker_row_from_bottom = marker_id // 7

        col = 8 - (2 * marker_col_from_right)
        row = 13 - (2 * marker_row_from_bottom)

        x0 = col * self.square_len + self.margin
        y0 = row * self.square_len + self.margin

        return np.array([
            [x0,                   y0,                    0.0],
            [x0 + self.marker_len, y0,                    0.0],
            [x0 + self.marker_len, y0 + self.marker_len,  0.0],
            [x0,                   y0 + self.marker_len,  0.0],
        ], dtype=np.float32)

    def estimate_board_pose(self, image):
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            return None

        ids = ids.flatten()
        obj_points = []
        img_points = []

        for marker_corners, marker_id in zip(corners, ids):
            obj_points.append(self.marker_id_to_object_points(int(marker_id)))
            img_points.append(marker_corners[0].astype(np.float32))

        obj_points = np.concatenate(obj_points, axis=0)
        img_points = np.concatenate(img_points, axis=0)

        if len(ids) < 6:
            return None

        ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.K, self.D)
        if not ok:
            return None

        return {
            "num_markers": int(len(ids)),
            "rvec": rvec.flatten().tolist(),
            "tvec": tvec.flatten().tolist(),
        }

    def get_tool_pose(self):
        self.listener.waitForTransform("base_link", "tool0", rospy.Time(0), rospy.Duration(1.0))
        trans, rot = self.listener.lookupTransform("base_link", "tool0", rospy.Time(0))
        return {
            "translation": list(trans),
            "quaternion": list(rot),
        }

    def run(self):
        rospy.sleep(1.0)
        print("Press 's' to save one sample, 'q' to quit.")

        while not rospy.is_shutdown():
            if self.latest_image is None:
                rospy.sleep(0.1)
                continue

            view = self.latest_image.copy()
            cv2.imshow("handeye_capture", view)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            if key == ord('s'):
                image = self.latest_image.copy()

                try:
                    tool_pose = self.get_tool_pose()
                except Exception as e:
                    print("TF error:", e)
                    continue

                board_pose = self.estimate_board_pose(image)
                if board_pose is None:
                    print("Board pose could not be estimated")
                    continue

                image_name = f"sample_{self.sample_idx:03d}.jpg"
                json_name = f"sample_{self.sample_idx:03d}.json"

                cv2.imwrite(os.path.join(self.output_dir, image_name), image)

                data = {
                    "image_file": image_name,
                    "tool_pose": tool_pose,
                    "board_pose": board_pose,
                }

                with open(os.path.join(self.output_dir, json_name), "w") as f:
                    json.dump(data, f, indent=2)

                print(f"Saved sample {self.sample_idx:03d} with {board_pose['num_markers']} markers")
                self.sample_idx += 1

        cv2.destroyAllWindows()

if __name__ == "__main__":
    SampleCapture().run()
