# Camera Calibration Pipeline with Structured JSON Export

A production-ready Python pipeline for camera intrinsic and extrinsic calibration using checkerboard patterns. This repository provides automated distortion coefficient calculation and exports structured configuration files for downstream computer vision tasks, such as real-time image undistortion, aerial photogrammetry, and Structure-from-Motion (SfM) initialization.

---

## Features

- **Sub-Pixel Corner Refinement:** Utilizes OpenCV's corner detection with sub-pixel optimization (`cv2.cornerSubPix`) to achieve high-precision calibration parameters.
- **Robust Matrix Estimation:** Computes the camera intrinsic matrix (`K`) and five radial/tangential distortion coefficients (`D`).
- **Cross-Platform JSON Export:** Serializes parameters into a clean, flat JSON structure for effortless integration with high-performance C++ backend modules.
- **Calibration Quality Evaluation:** Calculates reprojection error to assess calibration accuracy.
- **Portable Configuration Format:** Exports all parameters in JSON for seamless integration into computer vision pipelines.

---

## Repository Structure

```text
camera-calibration-json/
├── data/
│   └── calibration_images/    # Input checkerboard photos
├── src/
│   └── calibrate.py           # Core calibration logic
├── calibration_config.json    # Generated output configuration
├── requirements.txt
└── README.md
```

---

## Mathematical Framework

The calibration pipeline maps 3D world points to 2D image coordinates using the pinhole camera model.

Let the world point be represented as:

$$
\mathbf{X}_w =
\begin{bmatrix}
X_w \\
Y_w \\
Z_w
\end{bmatrix}
$$

and the corresponding image point as:

$$
\mathbf{x} =
\begin{bmatrix}
u \\
v
\end{bmatrix}
$$

The projection process is defined by:

$$
\mathbf{x} = K [R \mid t] \mathbf{X}_w
$$

where:

- $K$ is the camera intrinsic matrix
- $R$ is the rotation matrix
- $t$ is the translation vector

### Camera Intrinsic Matrix

The intrinsic matrix is defined as:

$$
K =
\begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

where:

- $f_x, f_y$ are the focal lengths in pixels
- $c_x, c_y$ are the principal point coordinates

---

### Lens Distortion Model

Real camera lenses introduce geometric distortions that deviate from the ideal pinhole model.

The radial and tangential distortion model is defined as:

$$
x_{\text{distorted}} =
x(1 + k_1r^2 + k_2r^4 + k_3r^6)
+ 2p_1xy
+ p_2(r^2 + 2x^2)
$$

where:

- $k_1, k_2, k_3$ — radial distortion coefficients
- $p_1, p_2$ — tangential distortion coefficients
- $r^2 = x^2 + y^2$ — squared distance from the optical center

The calibration accuracy is evaluated using the reprojection error:

$$
e =
\frac{1}{N}
\sum_{i=1}^{N}
\left\|
\mathbf{x}_i - \hat{\mathbf{x}}_i
\right\|_2
$$

where:

- $\mathbf{x}_i$ — detected image point
- $\hat{\mathbf{x}}_i$ — projected point after calibration
- $N$ — number of calibration points

The OpenCV calibration process estimates these parameters automatically from multiple checkerboard observations.
---

### Reprojection Error

Calibration quality is evaluated using the mean reprojection error.

After estimating the calibration parameters, each known 3D point is projected back into the image plane and compared to its detected image position.

The mean reprojection error is:

$$
e =
\frac{1}{N}
\sum_{i=1}^{N}
\left\|
\mathbf{x}_i
-
\hat{\mathbf{x}}_i
\right\|_2
$$

where:

- $\mathbf{x}_i$ is the detected image point
- $\hat{\mathbf{x}}_i$ is the projected image point using the estimated parameters
- $N$ is the total number of calibration points

Lower reprojection error indicates higher calibration accuracy.

---

## Setup & Dependencies

Ensure Python 3.10+ is installed.

Install required packages:

```bash
pip install -r requirements.txt
```

### Dependencies

- opencv-python
- numpy

---

## Usage

### 1. Add Calibration Images

Place checkerboard calibration photos inside:

```text
data/calibration_images/
```

Images should:

- Contain a clearly visible checkerboard pattern
- Be captured from different angles
- Cover multiple regions of the image frame
- Include at least 10–20 photos for robust calibration

### 2. Run Calibration

Execute:

```bash
python src/calibrate.py
```

### 3. Obtain Calibration Parameters

After successful execution, a JSON configuration file will be generated:

```text
calibration_config.json
```

---

## Output Format

Example output:

```json
{
  "camera_matrix": [
    [1153.42, 0.0, 960.0],
    [0.0, 1153.42, 540.0],
    [0.0, 0.0, 1.0]
  ],
  "distortion_coefficients": [
    -0.2834,
    0.0732,
    0.0001,
    -0.0002,
    0.0
  ],
  "status": "success"
}
```

### Output Fields

| Field | Description |
|---------|-------------|
| `camera_matrix` | Intrinsic camera matrix |
| `distortion_coefficients` | Radial and tangential distortion parameters |
| `status` | Calibration execution status |

---

## Applications

The generated calibration parameters can be used for:

- Image undistortion
- Robotics vision systems
- UAV navigation
- Structure-from-Motion (SfM)
- Photogrammetry
- SLAM pipelines
- Augmented Reality (AR)
- 3D reconstruction systems

---

## References

1. OpenCV Camera Calibration Documentation
2. Richard Hartley & Andrew Zisserman, *Multiple View Geometry in Computer Vision*
3. Zhang, Z. (2000), *A Flexible New Technique for Camera Calibration*