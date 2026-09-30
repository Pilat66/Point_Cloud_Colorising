# Point Cloud Colorisation Toolbox
Colourise lidar data with calibrated camera images

Demo video: https://www.youtube.com/watch?v=wmdwf09vh_M

Two modes:

| Mode | Colours | Needs |
|---|---|---|
| **Scan** | each instantaneous lidar sweep | sensor input |
| **Map** | a pre-built point cloud map | a PCD map + a CSV of timestamped poses |

**Scan** — each lidar sweep coloured from the cameras, live:

<img width="640" height="250" alt="scan_colorise" src="https://github.com/user-attachments/assets/2a585200-6eb8-4f87-ade6-46ab152d2ee0" />

**Map** — a fixed map colouring in as the platform drives through:

<img width="640" height="250" alt="map_colorise" src="https://github.com/user-attachments/assets/035d8c65-3009-4f38-b5ed-3f27e1649512" />


Intrinsics, distortion and extrinsics all come from `configs/calibration.yaml`.
Nothing is read from the TF tree.

---

## Build

```bash
sudo apt install libyaml-cpp-dev
cd ~/catkin_ws
catkin_make --pkg point_cloud_projection
source devel/setup.bash
```

---

## Mode 1 — scan colourisation

Terminal 1:

```bash
roslaunch point_cloud_projection ColoriseScan.launch
```

Terminal 2 (or run live sensors instead):

```bash
rosbag play <bag>
```

Output is published on `/colorised_points` in the `os_sensor` frame.

The odometry topic is only needed for `compensation_mode: odom`. Play slower
than real time if the node cannot keep up.

## Mode 2 — map colourisation

Supply two things, named in `configs/config.yaml`:

| Setting | What |
|---|---|
| `map_pcd_path` | a point cloud map (PCD, XYZI) in the map frame |
| `odom_csv_path` | timestamped poses of the **lidar** in that same frame |

The CSV is one pose per line, header row optional:

```
time,x,y,z,qx,qy,qz,qw
1770728769.869573,0.000000,0.000000,0.000000,0.012707,-0.006916,0.000088,0.999895
```

Terminal 1 (loading the map and building its kd-tree takes a few seconds):

```bash
roslaunch point_cloud_projection ColoriseMap.launch
```

Terminal 2:

```bash
rosbag play <bag> 
```

Ctrl-C saves to `save_pcd_path`.

Poses come from the CSV, so no odometry or TF topics are needed. 

---

## Compensation mode

If the cameras are **not** hardware-synchronised with the lidar: 

```bash
roslaunch point_cloud_projection ColoriseScan.launch compensation_mode:=odom
roslaunch point_cloud_projection ColoriseMap.launch  compensation_mode:=imu
```

`imu` is rotation-only by necessity, not as a shortcut: Integrating the 
accelerometer twice would add noise, not information. 

---

### Cameras

Adding one can be done with a config change:

```yaml
cameras:
  - name:           forwardLeft
    image_topic:    /forwardLeft/image_raw/compressed
    intrinsics_key: cam1                          # block in calibration.yaml
    extrinsic_key:  T_os_sensor_forwardLeft_cam   # 4x4 in calibration.yaml
    extrinsic_is_lidar_from_cam: true
```

### Fisheye gates

Equidistant fisheye at 2048x1536. Near the border the projection is optically
soft and numerically ill-conditioned, and close in it is inaccurate, so colour
sampled in either regime is unreliable. Three independent gates; a point must
pass **all** of them. All can be overridden per camera inside a `cameras:` entry.

| Setting | Meaning | Values |
|---|---|---|
| `edge_margin_px` | pixels trimmed from every image edge (2D) | `0` none, `60` mild, **`150` default**, `300` aggressive |
| `max_view_angle_deg` | half-angle from the optical axis, rejected in 3D **before** projection | `90` everything, **`75` default**, `60` conservative, `45` centre only |
| `min_camera_dist` | metres; points nearer the camera are left uncoloured | `0` colour everything, `1.0` mild, **`2.0` default**, `3.0` conservative |

**Its cost differs sharply between the two modes**, so consider a separate
config for the scan node (`config_path:=...`) if the default hurts:

- **Map: nearly free.** Raising it 0 → 2 m changed total coloured points by
  0.19%. A point passed at 1 m is still seen from 5 m on other frames and gets
  coloured properly then, so the gate discards only the worst *observation* of a
  point, not the point.
- **Scan: expensive.** Per-sweep coverage drops from ~19% to ~11-13%. A single
  sweep has no second chance — points close to the sensor are simply lost.

### Occlusion rejection

`occlusion_check` runs a z-buffer over the projected points. Without it a wall
and everything behind it land on the same pixels and all get painted with the
wall's colour. It matters most for map colourisation, where the whole map is
projected at once.

`occlusion_max_depth` (default `+inf`, i.e. off) and `occlusion_ray_margin`
(default `1` cell) add the "no lidar rays" rule: set a finite
`occlusion_max_depth` (e.g. `40`) to treat a z-buffer cell whose minimum depth
exceeds it — including empty cells, where that minimum is `+inf` — as an area
without lidar rays; the mask is then dilated by `occlusion_ray_margin` cells and
any point landing in the dilated mask is not coloured. The rule is
density-sensitive: on a sparse cloud the gaps between points look like empty
cells and coverage collapses, so enable it knowingly on dense data.

---

## Offline colourisation (Python, ROS-free)

`colorise_offline.py` reproduces the Map-mode projection pipeline (
`projectAndSample` from `src/colorise.cpp`) as a standalone script. It needs only
Python + `numpy` + `opencv-python` (+ `scipy` for the radius cull,
+ `laspy` only for `.las` input). No ROS, no catkin build.

It reads exactly the artefacts this repo keeps in `./data`:

| Input | Default | Notes |
|---|---|---|
| `--cloud` | `data/all_raw_points.pcd` | PCD (`x y z [nx ny nz] intensity [curvature …]`), any field set |
| `--photos` | `data` | dir of images, capture time in the **filename** (epoch ns, e.g. `1788882722018994333.png`) |
| `--trajectory` | `data/trajectory.csv` | lidar poses `time;x;y;z;roll;pitch;yaw` (Euler) or `time,x,y,z,qx,qy,qz,qw` |
| `--calib` | `data/calib.json` | camera intrinsics (its `camera` block: `width`/`height`, `intrinsics`, `distortion_coeffs`) + camera↔lidar extrinsic (see convention note below) |
| `--output` | `coloured.pcd` | coloured cloud written as `x y z intensity rgb` PCD |

```bash
python3 colorise_offline.py --cloud data/all_raw_points.pcd --photos data \
    --trajectory data/trajectory.csv \
    --calib data/calib.json --output coloured.pcd
```

Key flags: `--max-range`, `--min-camera-dist`, `--max-view-angle`, `--edge-margin`,
`--min-color-frames`, `--keep-uncolored`, `--occlusion`, `--euler-order/--euler-units`,
`--time-shift` (align photos from a different run), `--preview <png>`.

### Extrinsic direction (calib.json)

Per `direct_visual_lidar_calibration` / FAST-LIVO2 convention, `T_camera_lidar`
maps points **from the lidar frame into the camera frame** ("A_from_B") and is
used as-is (no inversion). The example `data/calib.json` stores the same
quantity under the mirror name `results.T_lidar_camera` (7 numbers
`x,y,z,qx,qy,qz,qw`); the resolver inverts it automatically. Direction is
auto-detected from the key name and can be forced with `--extrinsic-direction`.

> Note: photos and trajectory must come from the same run. The tool prints a
> warning (and covers nothing) when the photo time range does not intersect the
> trajectory; use `--time-shift` only for a known, constant offset.

### Brightness alignment of the photo series

`align_brightness.py` levels a photo series to its brightest frame before
colourisation. The reference is the frame with the highest value of the chosen
brightness statistic; every other frame is brought to it. Results go to a
sub-folder (`--out-dir`, default `aligned`) next to the originals, the reference
is copied as is.

```bash
python3 align_brightness.py data/1 --stat top25
```

| Flag | Meaning |
|---|---|
| `--stat` | brightness statistic (channel **L** of LAB, 0-255): `mean`, `median`, `top50` (mean of the brightest 50% of pixels), `top25` (brightest 25%, **default**), `max` (robust maximum = 99th percentile — a raw pixel maximum sits at 255 in the sky and does not separate frames) |
| `--method` | `gain` (**default**) scales L by `stat(ref)/stat(src)`; `hist` matches the L histograms instead |
| `--out-dir` | output folder name or absolute path, default `aligned` |
| `--max-gain` | clamp the gain to `[1/N, N]`, default `4.0`, `0` disables the clamp |
| `--no-resize` | do not resize frames to the reference size (such frames are skipped) |
| `--ext`, `--dry-run` | image extensions, and analysis only (nothing is written) |

Only the L channel is touched, so the colours stay as captured. With
`--method gain` a residual spread is expected: frames whose highlights are
clipped at 255 cannot be pushed above the value they already reached. The run
prints the spread before and after, so the effect is visible per folder.

`--stat mean --method hist` reproduces the behaviour of the previous version of
the script. Dependencies: `numpy` + `opencv-python` (`scikit-image` is not
needed).



### Thinning the photo series

`thin_photos.py` copies photos from a source folder to a destination folder
keeping only frames that are at least `--interval` seconds apart (default 1 s).
Frame times come from the file names (epoch in ns/ms/s, same parsing as
`colorise_offline`); names are preserved, so the thinned folder can be fed to
the colourisers as is.

```bash
python3 thin_photos.py data/img-dir thinned --interval 1.0
```
