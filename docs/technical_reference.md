# GNOCCHI technical reference

Usage and hardware selection are in the [README](../README.md). This document
contains estimator behavior, frame conventions, tuning notes, and validation results.

Globally referenced pose estimation for the air/ground team. Odometry enters the
graph **only as relative constraints**; GNSS position and heading enter as
**priors**, weighted by the accuracy the receiver reports. One frame, poses
only — no velocity or IMU-bias states.

With no odometry configured the graph drops out entirely and the node becomes a
GNSS-to-local-frame converter. That is the UAV profile.

| | UGV (`ugv.yaml`) | UAV (`uav.yaml`) | low-alt UAV (`lowalt_uav.yaml`) |
|---|---|---|---|
| mode | fused pose graph | GPS-only | fused pose graph |
| odometry | `bievr_lio/odom` | none | `zed/zed_node/odom` (VIO) |
| heading | dual-antenna | dual-antenna | PX4 `compass_hdg`, or fitted from motion |
| output frame | `ugv_map` | `uav_map` | `lowalt_uav_map` |
| rate | odometry rate (~30 Hz) | fix rate, ~42 Hz | odometry rate |
| TF | `world->map` static, `map->odom` live | `world->map` static | `world->map` static, `map->odom` live |

## Launch and publication behavior

```bash
ros2 launch gnocchi ugv.launch.py    # UGV
ros2 launch gnocchi uav.launch.py    # UAV
ros2 launch gnocchi lowalt_uav.launch.py    # low-altitude UAV
```

The low-altitude UAV profile is UNVERIFIED on real data. It uses FCU compass
heading or motion initialization instead of dual-antenna heading: see the header of `config/lowalt_uav.yaml` for the two yaw
options and the ZED launch settings it depends on (`publish_map_tf:=false`).

All three launches take `use_sim_time` and `config` (a filename under the
installed package's `config/` directory).

`publish_on_trigger` is enabled in the UAV config and disabled in the UGV
config (the node default is `false`). When enabled, `pose_map`, `pose_map_throttled`, `odom_map`, `odom_map_throttled`,
the optional debug pose, and live TF wait for a `std_msgs/msg/Bool` message
with `data: true` on `initialization_trigger_topic` (default `/initialization_trigger`).
Publishing starts with the next eligible sensor update and stays enabled until
restart; false or repeated triggers do not stop or reset it. Sensor processing,
datum initialization, and static TF continue while waiting. This also gates
odometry passthrough when GNSS is disabled.

For indoor UGV testing, set `enable_gnss: false` in `config/ugv.yaml` and
restart gnocchi. With `publish_tf: true`, the node immediately publishes static
identity transforms `world -> ugv_map -> odom`, using the configured frame names.
This mode ignores the datum and `frames.map_yaw_deg` and relays configured
odometry as `odom_map` / `pose_map`, with the output frame names applied. It
subscribes to no GNSS or heading inputs. If `topics.odom` is empty, it is idle.
The odometry source still owns `odom -> base_link`. `enable_gnss` defaults to `true`; set it back to `true` and
restart for normal GNSS operation. `publish_tf: false` suppresses TF in either mode.

When replaying a bag, start the node **before** playback and play with
`--clock` — it needs a fix and either an external heading or successful
position alignment before it can initialize the graph, and
`optimizer.min_keyframes` (default 5) keyframes of motion after that before it
publishes anything.

**Inputs** (`config/ugv.yaml`): `bievr_lio/odom`, both GNSS receivers
(`gnss.receivers`: `mavros/gpsstatus/gps1/raw` and `gps2/raw`, see below), and
dual-antenna heading from every configured GPS instance (the same two topics —
whichever reports a valid moving-baseline yaw is used). The graph has no IMU input: lidar odometry
is itself gravity-aligned and supplies roll/pitch. In the default UGV profile,
external heading priors constrain global yaw; in position-alignment mode, global
yaw is initialized from motion and then constrained by GPS positions and relative
odometry. The FCU gyro (`mavros/imu/data`) is used
only by the GNSS-only fallback below.

**Inputs** (`config/uav.yaml`): `mavros/global_position/global` and the same
dual-antenna heading topics. No odometry, and — since there's no IMU either —
no attitude source at all beyond yaw: the GPS-only output pose is always
level (roll/pitch = 0), with yaw from the heading prior. This profile is
UNVERIFIED on real UAV data.

**Outputs**, carrying the same pose from the same call:

| topic | type | frame |
|---|---|---|
| `odom_map` | `nav_msgs/Odometry` | `ugv_map` / `uav_map` |
| `pose_map` | `geometry_msgs/PoseStamped` | same |
| `odom_map_throttled` | `nav_msgs/Odometry` | same |
| `pose_map_throttled` | `geometry_msgs/PoseStamped` | same |

`pose_map_throttled` and `odom_map_throttled` sample their full-rate outputs
together at at most 1 Hz for other robots,
preserving their measurement timestamps and frames, including all odometry
covariance and twist fields. The shared throttle publishes the first eligible
pose immediately, then the first new pose at least one second after the previous
publication. It uses the node clock (including simulated time), resumes immediately
after a backward clock jump, and does not repeat stale poses when inputs stop.
This applies to fused, GPS-only, and GNSS-disabled passthrough modes. The relative
topic defaults to `/ugv/pose_map_throttled` or `/uav/pose_map_throttled` under the
provided launches; override `topics.output_pose_throttled` to rename it. QoS is
reliable, volatile, with depth 1. The odometry topic similarly defaults to
`/ugv/odom_map_throttled` or `/uav/odom_map_throttled` and is configurable via
`topics.output_odom_throttled`. The original outputs retain their full rate.

plus, when `publish_tf: true`, the static `world -> *_map` and the live
`*_map -> odom` in fused mode. GPS-only mode publishes only the static
`world -> *_map` edge; it does not publish a live edge to `base_link`.

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
rotate it by, and it stays zero until the first heading arrives). In
position-alignment mode, timestamp-matched odometry attitudes rotate the lever
arm instead, including roll/pitch.

**Two receivers** (`gnss.receivers`, UGV). mavros' `global_position/raw/fix`
is whichever receiver PX4 numbered gps1, and that changes between boots: in the
field bags it was the moving-baseline rover in seven and the base in three, and
the two antennas are 0.47 m apart across the robot. So gnocchi takes both
receivers' `GPSRAW` fixes and fuses their **midpoint**, which is `base_link`
horizontally on the UGV (fitting LIO to GNSS put it within 5 cm laterally on
three bags); `extrinsics.gnss_antenna` is then the midpoint's offset, i.e. at
most a height. The stamps are mavros receive times and the receivers run at
different rates, so the other receiver is extrapolated to each fix's stamp
(`gnss.pair_max_dt`). While both have fixes, gnocchi learns each antenna's
offset from the midpoint in the body frame (logged once as "Antenna offsets").
When one loses its fix (the rover does under trees), the other is used alone,
moved by that offset, after `gnss.min_offset_samples` paired fixes. In replay,
cutting either receiver for 30 s moved the output by at most 6 mm. With a
single topic in `gnss.receivers`, that receiver is used alone at
`extrinsics.gnss_antenna`. Without `gnss.receivers` (or with `[""]`: parameter files cannot hold an empty list), `topics.gnss` (any
`NavSatFix`, e.g. the FCU's `global_position/global`) is used instead.

`config/` holds only the deployment profiles: `ugv.yaml`, `uav.yaml` and `lowalt_uav.yaml`.
Each file's own comments explain its tuning in detail — several sigmas (e.g.
`odom.rotation_floor`) were re-derived from real gyro comparisons, so treat
them as measured, not as defaults to copy elsewhere without re-checking.

Heading weight always comes from the receiver's own reported accuracy
(`hdg_acc` on the GPSRAW message), bounded by `heading.gnss_sigma_floor` /
`heading.gnss_sigma_max` — there is no config knob that fixes it to a constant.


## Initialization without heading hardware

For the UGV, set `heading.initialize_from_positions: true` in `config/ugv.yaml`
and restart. The default remains `false`. This mode ignores both dual-antenna
heading and compass inputs, regardless of `heading.use_gnss_heading`, and requires
gravity-aligned LIO/VIO (`topics.odom` cannot be empty when GNSS is enabled).

Before publishing globally aligned poses, GNOCCHI fits a horizontal yaw and
translation between timestamp-matched LIO antenna positions and GNSS positions.
LIO attitudes rotate the configured `extrinsics.gnss_antenna` offset; LIO poses
are interpolated at GPS timestamps, without extrapolation. Reverse travel is
supported: this estimates frame rotation, not GPS course as vehicle heading.
Roll/pitch continue to come from gravity-aligned LIO.

The initialization window defaults to 60 seconds, at least 10 fixes spaced one
second apart, and at least 10 metres of horizontal span. Fits exceeding 1.5 m
horizontal RMS or 20 degrees estimated angular uncertainty are rejected. Angular
uncertainty has a 10-degree floor and does not shrink as if correlated fixes were
independent. These are conservative heuristics, not calibrated covariance.
See `heading.alignment` in the UGV YAML for the thresholds. LIO interpolation
requires gaps of at most 0.5 seconds. Stationary motion, short baselines, missing
GPS, or inconsistent fits leave initialization pending, with a throttled log.
Move using local odometry/manual control until alignment succeeds; a controller
that requires `odom_map` cannot initiate that motion itself.

After initialization the alignment is not repeatedly applied as a heading prior.
GPS position factors and LIO relative constraints maintain the estimate, including
yaw correction where translational motion makes it observable. GPS outages and
stationary periods cannot generally constrain yaw. A large GPS correction can
still jump the global output. GNSS-only debug poses are unavailable in this mode
because they require an independent heading. Historical map data is not repaired.

Validation: `colcon build --packages-select gnocchi --cmake-args -DBUILD_TESTING=ON`
then `colcon test --packages-select gnocchi`. The position-alignment test covers
interpolated timing, antenna offset, reversing, stationary/bad-fit rejection,
clock rewind during initialization, external-heading exclusion, and ongoing
correction of synthetic yaw drift using GPS positions alone.


## UAV input and publication status

GPS-only mode logs `UAV GNSS status` every five wall-clock seconds, including
when no sensor data arrives. It reports cumulative received/valid GNSS and
heading counts, the most recently accepted heading source, and the count of
full-rate pose/odometry publications. Ages measure time since the last valid
receipt or publication using a steady clock; `-1` means none yet. Counts describe
messages accepted by the existing input checks, not independently verified accuracy.

The status identifies waiting for the initialization trigger, missing valid GNSS
or heading, stale inputs (older than five seconds), and active publishing. A stale
heading warning explicitly notes that publication may still use the cached heading;
this monitoring does not change that policy. Nonfinite compass headings are rejected.
The normal `pose_map` and `odom_map` outputs are counted together, once per pair.


## Selecting FCU compass heading

Set `heading.use_gnss_heading: false` and
`heading.initialize_from_positions: false` to use the `std_msgs/Float64` stream
at `heading.compass_topic` (default `mavros/global_position/compass_hdg`).
The relative topic resolves under the robot namespace. Its fixed uncertainty is
`heading.compass_sigma_deg`, default 5 degrees. Values are interpreted as degrees
clockwise from north and converted into ENU yaw. Nonfinite values are rejected.
Both UGV fusion and UAV GPS-only publication support this source; GNSS heading
subscriptions are disabled when compass is selected.

For UGV fusion, `heading.time_offset` applies to the selected heading source.
The shipped -0.240 s value was measured for GNSS heading: start with 0.0 for
compass unless a compass-specific offset has been measured. Verify the FCU's
north reference and magnetic calibration before interpreting global yaw.


GNSS heading input topics are configured with `heading.gnss_topics` (a list),
alongside `heading.compass_topic`. This replaces `topics.gnss_heading`; migrate
that key in custom parameter files and launch overrides. `heading.use_gnss_heading`
selects which source is subscribed.


## LIO failure handling (UGV)

`lio_health.enabled` is enabled in the supplied UGV config and defaults to false
in the node. It compares timestamp-matched LIO and GNSS horizontal increments
in a two-second window. Three or more fixes spanning at least 0.5 s are required;
GPS discontinuities above `gps_max_speed` reset the evidence. A discrepancy over
3 m, or LIO speed over 5 m/s, must persist for 0.5 s before LIO is flagged.
Tune these thresholds to the platform; they are not a statistical fault classifier.
The detector uses increments rather than absolute GPS innovations, so an isolated
GPS position offset does not automatically mark LIO as failed.

While degraded, relative translation sigma is at least 10 m per keyframe and
rotation sigma at least 0.5 rad. GNSS fixes validated by the consistency window
are allowed position factors up to 10 Hz without Huber attenuation, allowing
recovery from an estimate far from GPS. When GPS becomes stale or discontinuous,
Huber protection remains enabled. Neither source can be independently verified
from these two streams alone: sustained plausible GPS drift can still mislead
this mechanism. Existing acceptance checks and the horizontal uncertainty floor
still apply. Horizontal fusion now uses the largest horizontal reported variance;
vertical variance does not suppress horizontal fixes when altitude fusion is off.
This is a conservative scalar horizontal model, not full covariance fusion.

Thirty seconds of consistent motion are required before normal between-factor
weighting resumes. Absolute LIO attitude and height priors remain disabled after
any detected failure until restart: local agreement at a stop cannot establish
that an accumulated attitude/height error has recovered. With GPS altitude fusion
disabled, height during/after a LIO failure is not independently constrained.
Failure also adds a covariance allowance to published estimates; this is not a
calibrated posterior under arbitrary sensor faults. Raw LIO covariance is still
not consumed, since a pose covariance is not a relative-motion covariance and
RKO-LIO's recorded matrices were all zero in the evaluated bag.

`/ugv/lio_degraded` (`std_msgs/Bool`) is published on every LIO callback, including
before graph initialization. True means a detected fault; false is not a guarantee
of valid GPS or localization. Health changes and degraded operation are logged,
including whether GPS has recent validation. Optimizer exceptions are counted
and logged; a failed optimization invalidates new pose publication until a
successful update, rather than continuing to label a stale estimate as current.

These corrections can move the global pose abruptly, including during the
confirmation delay. They do not repair old map points or guarantee safe control
through a localization failure. A navigation consumer should use the fault flag
to stop/replan or enter its own degraded mode. If GPS is lost while LIO is bad,
this node has no independent sensor with which to recover position.

Validation: `colcon build --packages-select gnocchi --cmake-args -DBUILD_TESTING=ON`
and `colcon test --packages-select gnocchi`. `test/verify_lio_recovery.py` is an
additional ROS integration check requiring an isolated ROS domain and working
transport. It verifies the published fault flag and uncertainty allowance.


## GPS height recovery after LIO failure

The UGV config enables `gps_height.enabled`. Healthy operation retains LIO height;
`gnss.fuse_altitude` remains false. While healthy, a rolling median learns the
local-pose minus GPS-height offset using fixes with reported vertical sigma at
most 2 m and consistent recent height motion. The reference uses up to 30 s of
samples and excludes the latest 3 s when a LIO failure is detected, to reduce
contamination during fault confirmation. At least five reference samples are
needed. The offset then freezes until restart; failed LIO cannot redefine it.

After a detected LIO failure, GPS height plus that offset is used as a separate
robust vertical factor with sigma at least 3 m. Recent horizontally validated GPS
and at least three height samples spanning 0.5 s are required. Height samples
more than 0.5 s old, missing/invalid vertical uncertainty, reported sigma above
2 m, or abrupt height motion above 2 m/s suspend aiding. These gates are tunable
under `gps_height`. This is relative height recovery, not accurate absolute
ellipsoid or terrain elevation. Persistent GPS altitude bias changes remain a
limitation. With insufficient healthy reference data, aiding stays unavailable.

`gps_height.recovery_ramp_time` (default 5 seconds, 0 for the previous immediate
correction) gradually removes the height mismatch when aiding first becomes
available. The first target matches the graph's predicted height; its temporary
offset from the GPS target fades linearly to zero. After that, the original GPS
height target and covariance apply unchanged. GPS altitude changes and actual
robot motion are not low-pass filtered or velocity-clamped. If aiding becomes
unavailable, the transition restarts from the current prediction
when valid aiding returns. Horizontal GPS factors and recovery gates are unchanged.
This reduces the activation discontinuity; it is not a hard bound on every
published pose step, and does not retroactively reproject stored map points.

Height aiding remains available after the horizontal LIO health flag recovers,
because the failed absolute LIO height prior remains disabled until restart.
It does not run if full `gnss.fuse_altitude` is explicitly enabled. The separate
vertical factor prevents an altitude outlier from directly attenuating the
horizontal GPS factor's robust weight. Published vertical variance is given an
allowance of at least the configured sigma floor squared while aiding is active;
this allowance is not a calibrated failure-mode covariance. Degraded logs report
`GPS height aiding` and the cumulative height-factor count. UAV GPS-only behavior
is unchanged.


## GNSS-only fallback when lidar odometry fails (UGV)

`fallback.enabled` (on in `ugv.yaml`, off by default in the node) keeps `odom_map` /
`pose_map` publishing when lidar odometry stops, restarts or (`fallback.on_lio_degraded`)
is flagged by `lio_health`:

- **Trigger:** no odometry for `odom_timeout` (0.5 s); an odometry step larger than
  `odom_jump` (a restarted source starting again at its origin); or, with
  `on_lio_degraded`, a `lio_health` fault. If odometry never arrives, GNSS-only
  output starts `startup_wait` (10 s) after the first fix.
- **Output:** the fix, lever-arm corrected, published at the fix rate. The step
  from the last fused pose is faded out over `blend_time` (10 s). Height is held
  at the last fused value (`altitude_tau: 0`); GNSS altitude was worse than holding
  on the test bags. Roll and pitch are level: FCU attitude was no closer to LIO
  than that. `lio_degraded` is true while this runs. `map -> odom` TF is not
  published, because there is no live odom frame to align.
- **Yaw:** a yaw + gyro-bias Kalman filter (`yaw_filter.hpp`) runs all the time.
  While the graph is healthy it follows the fused yaw. Otherwise it follows the
  selected external heading (lag-corrected by `heading.time_offset`). Between those
  updates it integrates the FCU gyro (`fallback.imu_topic`; empty = heading only,
  held at most 1 s). After `max_heading_age` (30 s) without a heading it stops
  publishing rather than trust the gyro further.
  In position-alignment mode there is no external heading: only the last healthy
  fused yaw and optional gyro can bridge a failure, for the configured age limit.
  Without any initialized yaw, GNSS-only fallback cannot publish.
- **Recovery:** when odometry returns after more than `rebuild_after` (2 s), or
  with a jump, the graph is rebuilt in the new odom frame at the current height.
  GNSS-only output continues until the new graph has `optimizer.min_keyframes`.
  A shorter stall in the same frame continues the old graph.

Measured by bag replay (`ugv_gnocchi_long`; reference = LIO rigidly aligned to
the fused run; LIO cut from 60 s to the end):

| | horizontal rms / max | yaw p95 / max | height rms |
|---|---|---|---|
| fused (LIO healthy) | 0.32 / 0.79 m | 1.8 / 4.3 deg | 0.01 m |
| GNSS-only, 280 s | 0.59 / 1.07 m | 2.5 / 7.8 deg | 0.57 m |
| same, plus three 20-30 s heading outages | 0.59 / 1.07 m | 3.9 / 8.8 deg | 0.58 m |
| LIO restarted twice (60 s and 30 s gaps) | 0.52 / 1.03 m | 2.1 / 7.6 deg | 0.60 m |

Without the gyro, holding the last heading through 30 s outages was off by tens
of degrees. With it, yaw stayed within about 2 deg (median). One exception: in
`tree_tarp_badloc`, during violent oscillating motion, the FCU gyro was off by
about 7 deg/s against both GNSS heading and LIO. That is why gyro-only bridging
is capped. On the field bags where LIO degrades, `on_lio_degraded` cut the largest
single output step from 4.45 m to 0.65 m (`long_road_bad`) and from 2.46 m to
0.51 m (`tree_tarp_badloc`).

GNSS-only accuracy is GNSS accuracy: about 0.5 m of slow wander with this
receiver (not RTK for position), with good fix-to-fix smoothness (3 cm median).
It does not repair map points from before the failure.
