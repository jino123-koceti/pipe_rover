# coin_lio (ROS 2 port)

ROS 2 Humble port of **COIN-LIO: Complementary Intensity-Augmented LiDAR
Inertial Odometry**, integrated into the `pipe_rover` workspace.

## Upstream

- **Paper**: Patrick Pfreundschuh, Helen Oleynikova, Cesar Cadena, Roland
  Siegwart, and Olov Andersson. *"COIN-LIO: Complementary Intensity-Augmented
  LiDAR Inertial Odometry."* In Proc. IEEE International Conference on
  Robotics and Automation (ICRA), 2024.
- **Original repository**: https://github.com/ethz-asl/COIN-LIO
- **Authors**: Autonomous Systems Lab (ASL), ETH Zurich
- **Upstream license**: BSD 3-Clause (for new COIN-LIO code) +
  GNU GPL v2 (for portions derived from FAST-LIO2). See [`LICENSE`](LICENSE).

If you use COIN-LIO in academic work, please cite the original paper.

## Modifications in this fork (ROS 2 port)

This directory is **not** the upstream repository. It has been ported and
extended for the `pipe_rover` project. The main changes relative to upstream
are:

- **ROS 1 → ROS 2 Humble migration**
  - `rclpy`/`rclcpp` instead of `roscpp`
  - Subscriptions, publishers, parameters, services rewritten for ROS 2 API
  - `nav_msgs::msg::*`, `sensor_msgs::msg::*`, etc.
  - `tf2_ros::TransformBroadcaster` ROS 2 style

- **Ouster metadata loading via ROS 2 parameters**
  - `beam_altitude_angles` (128 beam angles)
  - `pixel_shift_by_row` (destagger offsets)
  - `lidar_origin_to_beam_origin_mm`
  - Loaded from `config/ouster128.yaml` instead of JSON metadata file

- **Photometric line-removal FIR filter integration**
  - `config/line_removal.yaml` loaded as a separate parameter file
  - 33-tap highpass + lowpass kernels

- **LiDAR callback throttling** to reduce CPU load when other nodes
  share the LiDAR topic (configurable via `lidar_process_rate_hz`).

- **Dense RGB-D mapper bridge** — coin_lio publishes `/Odometry` and
  `/cloud_registered`, consumed by `vill_slam` packages for downstream
  pose-graph SLAM and RGB colorization.

- **PCL `setVerbosityLevel(L_ERROR)`** at startup to silence the
  "Failed to find match for field 'ring'" warnings that flood logs with
  Ouster point clouds.

- **Removed verbose `[DEBUG]` `std::cout` traces** that were used during
  initial porting.

## Source attribution inside this directory

| File | Origin | License |
|------|--------|---------|
| `src/laserMapping.cpp` | Derived from FAST-LIO2 + COIN-LIO additions | GPL v2 (for FAST-LIO2 base) / BSD-3 (for COIN-LIO additions). See LICENSE. |
| `src/projector.cpp` | COIN-LIO new code (Patrick Pfreundschuh) | BSD-3-Clause |
| `src/feature_manager.cpp` | COIN-LIO new code | BSD-3-Clause |
| `src/image_processing.cpp` | COIN-LIO new code | BSD-3-Clause |
| `src/preprocess.cpp` | FAST-LIO2 base | GPL v2 |
| `include/IKFoM_toolkit/` | iKalman Filter on Manifolds (HKU-MARS) | BSD-2-Clause |
| `include/ikd-Tree/` | Incremental k-d Tree (HKU-MARS) | GPL v2 |

## Configuration files

| File | Purpose |
|------|---------|
| `config/ouster128.yaml` | Main parameters for Ouster OS1-128 (lidar topic, ESKF noise, photometric scale, Ouster metadata) |
| `config/line_removal.yaml` | FIR kernel coefficients for intensity image line artifact removal |
| `config/params.yaml` | Reference parameter set (less actively used) |

## Citation

```bibtex
@inproceedings{pfreundschuh2024coinlio,
  title     = {COIN-LIO: Complementary Intensity-Augmented LiDAR Inertial Odometry},
  author    = {Pfreundschuh, Patrick and Oleynikova, Helen and Cadena, Cesar
               and Siegwart, Roland and Andersson, Olov},
  booktitle = {IEEE International Conference on Robotics and Automation (ICRA)},
  year      = {2024}
}
```

And for FAST-LIO2 (the base):
```bibtex
@article{xu2022fastlio2,
  title   = {{FAST-LIO2}: Fast Direct LiDAR-Inertial Odometry},
  author  = {Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal = {IEEE Transactions on Robotics},
  year    = {2022}
}
```
