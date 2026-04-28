# Hand-Eye Calibration Workflow

This repository contains scripts for calibrating the transformation between a ZED camera mounted on a UR5e and the robot `tool0` frame.

The goal is to estimate:

```text
tool0 -> zedm_left_camera_optical_frame
```

---

## Required ROS Nodes

Before running the calibration scripts, start the robot driver and the ZED ROS node.

### 1. Start the UR5e driver

Make sure the UR5e driver is running and publishing TF.

Check that the transform is available:

```bash
rosrun tf tf_echo base_link tool0
```

Expected result:

```text
base_link -> tool0
```

### 2. Start the ZED node

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch zed_wrapper zedm.launch
```

The calibration uses the left rectified image topic:

```text
/zedm/zed_node/left/image_rect_color
```

---

## Python Environment

Activate the hand-eye calibration environment:

```bash
source ~/venvs/handeye/bin/activate
```

---

## Calibration Workflow

Run the scripts in the following order.

---

### Step 1: Check ChArUco Target Detection

Use this step to verify that the calibration board is visible and that the markers are detected correctly.

```bash
cd ~/catkin_ws/src/hand_eye_calibration
~/venvs/handeye/bin/python detect_aruco.py
```

Expected result:

- marker IDs, ChArUco corner count, and reprojection error are printed
- detected markers, ChArUco corners, and target axes are shown in the image window

Continue only if marker detection works reliably.

The configured target is the Calib.io ChArUco 300x200 Coarse board:

```text
14 columns x 9 rows
checker size: 20 mm
marker size: 15 mm
dictionary: ArUco DICT_5X5_100
```

---

### Step 2: Test TF Readout

Use this step to verify that the robot pose can be read from TF.

```bash
~/venvs/handeye/bin/python get_tf.py
```

Expected result:

```text
Translation: [...]
Rotation (quat): [...]
```

Continue only if the transform `base_link -> tool0` is printed correctly.

---

### Step 3: Collect Calibration Samples

Run the sample collection script:

```bash
~/venvs/handeye/bin/python capture_sample.py
```

A camera window opens.

For each sample:

1. Move the robot to a new pose.
2. Make sure the calibration board is clearly visible.
3. Press `s` to save one sample.
4. Repeat for multiple robot poses.

Press `q` to quit.

Recommended number of samples:

```text
20-40 samples
```

The samples are saved to:

```text
~/handeye_samples/
```

New samples include a `board_model` tag. `compute_handeye.py` ignores older samples without this tag, so pre-fix JSON files from the former custom board model are not mixed into the new calibration.

Each sample consists of:

```text
sample_XXX.jpg
sample_XXX.json
```

Example:

```text
sample_000.jpg
sample_000.json
sample_001.jpg
sample_001.json
...
```

---

## Good Sample Poses

Use poses with:

- strong rotational variation
- different camera-board distances
- board visible in different image regions
- clear marker visibility
- little or no motion blur

Avoid:

- only translating the robot
- using nearly identical poses
- only frontal views
- partially occluded boards
- very blurry images

---

### Step 4: Compute the Hand-Eye Calibration

After collecting enough samples, run:

```bash
~/venvs/handeye/bin/python compute_handeye.py
```

The script prints the estimated transformation.

Relevant output:

```text
static_transform_publisher args:
X Y Z QX QY QZ QW tool0 zedm_left_camera_optical_frame 100
```

This is the desired transformation:

```text
tool0 -> zedm_left_camera_optical_frame
```

---

## Using the Result in ROS

Publish the result as a static transform:

```bash
rosrun tf static_transform_publisher X Y Z QX QY QZ QW tool0 zedm_left_camera_optical_frame 100
```

Replace:

```text
X Y Z
```

with the first three values from `static_transform_publisher args`.

Replace:

```text
QX QY QZ QW
```

with the quaternion values from `static_transform_publisher args`.

Then replace the `zed_camera_static_tf` args in:

```text
usb_c_insertion/launch/launch_ur.launch
```

---

## Validation

After calibration, validate the result with additional robot poses.

For each validation pose, compute:

```text
base_link -> tool0
tool0 -> zedm_left_camera_optical_frame
zedm_left_camera_optical_frame -> board
```

The resulting board pose in `base_link` should stay approximately constant.

Expected behavior:

- the board remains stable in the robot base frame
- no large jumps between different validation poses
- the estimated position is physically plausible

---

## Typical Problems

| Problem | Likely Cause |
|--------|--------------|
| No markers detected | wrong dictionary, bad image, poor lighting |
| Board pose is wrong | incorrect board geometry or ID mapping |
| Hand-eye result is unstable | insufficient rotation diversity |
| Result changes strongly with samples | bad samples or poor marker visibility |
| Python crash with `cv_bridge` | NumPy 2.x incompatibility |
| TF error | robot driver not running or wrong frame names |

---

## Final Output

The final calibration result is:

```text
tool0 -> zedm_left_camera_optical_frame
```

This transform can be used as a static TF in ROS.
