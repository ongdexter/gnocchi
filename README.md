# GNOCCHI

**G**lobal **N**avigation from **O**dometry **C**onstraints and
**C**ovariance-weighted **H**eading with **I**ncremental Smoothing.

Globally referenced pose estimation for the air/ground team. Odometry enters the
graph **only as relative constraints**; GNSS position and heading enter as
**priors**, weighted by the accuracy the receiver reports. One frame, poses
only — no velocity or IMU-bias states.

With no odometry configured the graph drops out entirely and the node becomes a
GNSS-to-local-frame converter. That is the UAV profile.

| | UGV (`ugv.yaml`) | UAV (`uav.yaml`) |
|---|---|---|
| mode | fused pose graph | GPS-only |
| odometry | `rko_lio/odometry` | none |
| output frame | `ugv_map` | `uav_map` |
| rate | odometry rate (~30 Hz) | fix rate, ~42 Hz |
| TF | `world->map` static, `map->odom` live | none (`publish_tf: false`) |

## Run

```bash
ros2 launch gnocchi ugv.launch.py    # UGV
ros2 launch gnocchi uav.launch.py    # UAV
```

Both take `use_sim_time`, and `config` if you need to point at a config file
other than each launch file's default (`ugv.yaml` / `uav.yaml`).

For indoor UGV testing, set `enable_gnss: false` in `config/ugv.yaml` and
restart gnocchi. With `publish_tf: true`, the node immediately publishes static
identity transforms `world -> ugv_map -> odom`, using the configured frame names.
This mode ignores the datum and `frames.map_yaw_deg`, subscribes to no sensors,
and publishes no pose or odometry estimates. The odometry source still owns
`odom -> base_link`. `enable_gnss` defaults to `true`; set it back to `true` and
restart for normal GNSS operation. `publish_tf: false` suppresses TF in either mode.

When replaying a bag, start the node **before** playback and play with
`--clock` — it needs a fix and a heading before it can anchor the origin, and
`optimizer.min_keyframes` (default 5) keyframes of motion after that before it
publishes anything.

**Inputs** (`config/ugv.yaml`): `rko_lio/odometry`, `mavros/global_position/
raw/fix`, and dual-antenna heading from every configured GPS instance
(default `mavros/gpsstatus/gps1/raw` and `gps2/raw` — whichever reports a
valid moving-baseline yaw is used). There is no IMU input: lidar odometry is
itself gravity-aligned and supplies roll/pitch, and yaw always comes from the
heading prior, never the odometry frame.

**Inputs** (`config/uav.yaml`): `mavros/global_position/global` and the same
dual-antenna heading topics. No odometry, and — since there's no IMU either —
no attitude source at all beyond yaw: the GPS-only output pose is always
level (roll/pitch = 0), with yaw from the heading prior. This profile is
UNVERIFIED on real UAV data.

**Outputs**, both carrying the same pose from the same call:

| topic | type | frame |
|---|---|---|
| `odom_map` | `nav_msgs/Odometry` | `ugv_map` / `uav_map` |
| `pose_map` | `geometry_msgs/PoseStamped` | same |

plus, when `publish_tf: true`, the static `world -> *_map` and the live
`*_map -> odom` (or `-> base_link` in GPS-only mode, since there is no odom
frame — only enable this then if nothing else publishes `base_link`).

Set `debug.publish_gnss_only: true` to also publish `debug/gnss_only`
(`nav_msgs/Odometry`, same frame) — the pose GNSS alone would give, for
overlaying against the fused estimate in RViz.

## Frames

```
world                ENU / UTM, absolute
  |  static: translation to the datum, + 90 deg yaw
ugv_map              NWU: +X north, +Y west. Origin = first accepted fix
  |  live: the alignment this node estimates
odom                 odometry frame, rotated from ENU by whatever heading
  |                  the robot started at
base_link            published by the odometry source
```

The graph works in local ENU about the datum and is rotated into the map frame
only on output, so `frames.map_yaw_deg` changes the convention without touching
the estimator (0 gives an ENU map frame).

GNSS enters the graph relative to a metric **datum** — the easting/northing/
altitude of the first accepted fix — so downstream consumers never see raw UTM
coordinates, which would blow float32 precision. If `extrinsics.gnss_antenna`
is set, that lever arm (FLU, in `base_link`) is removed from every fix before
it reaches the graph, rotated by yaw only (there is no roll/pitch source to
rotate it by, and it stays zero until the first heading arrives).

`config/` holds only the two deployment profiles, `ugv.yaml` and `uav.yaml`.
Each file's own comments explain its tuning in detail — several sigmas (e.g.
`odom.rotation_floor`) were re-derived from real gyro comparisons, so treat
them as measured, not as defaults to copy elsewhere without re-checking.

Heading weight always comes from the receiver's own reported accuracy
(`hdg_acc` on the GPSRAW message), bounded by `heading.gnss_sigma_floor` /
`gnss_sigma_max` — there is no config knob that fixes it to a constant.
