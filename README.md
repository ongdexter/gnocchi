# GNOCCHI

**G**lobal **N**avigation from **O**dometry **C**onstraints and
**C**ovariance-weighted **H**eading with **I**ncremental Smoothing.

GNOCCHI combines GPS positions with LIO/VIO odometry to publish globally aligned
poses. It also supports GPS-only operation when an external heading is available,
and local odometry passthrough when GPS is disabled.

## Usage

Choose a starting profile, then edit its `ros__parameters` for your hardware:

| Profile | Default inputs | Launch |
|---|---|---|
| [UGV](config/ugv.yaml) | BIEVR-LIO + dual-antenna GNSS heading | `ros2 launch gnocchi ugv.launch.py` |
| [UAV](config/uav.yaml) | GNSS + dual-antenna heading, no odometry | `ros2 launch gnocchi uav.launch.py` |
| [Low-altitude UAV](config/lowalt_uav.yaml) | ZED VIO + single-antenna GNSS + FCU compass heading | `ros2 launch gnocchi lowalt_uav.launch.py` |

The UAV and low-altitude UAV profiles are unverified on real vehicle data.

### Select the heading source

With GPS enabled (`enable_gnss: true`), set the following options:

| Available hardware | Options to set | Startup |
|---|---|---|
| GPS + LIO/VIO + dual-antenna heading | `topics.odom: <odom topic>`, `heading.initialize_from_positions: false`, `heading.use_gnss_heading: true`; set `heading.gnss_topics` | Wait for a valid fix and heading, then enough odometry keyframes. |
| GPS + LIO/VIO + compass/FCU heading | `topics.odom: <odom topic>`, `heading.initialize_from_positions: false`, `heading.use_gnss_heading: false`; set `heading.compass_topic` and start with `heading.time_offset: 0.0` | Wait for a valid fix and heading, then enough odometry keyframes. |
| GPS + LIO/VIO, no compass or dual-antenna heading | `topics.odom: <odom topic>`, `heading.initialize_from_positions: true` | Move using local odometry/manual control until position alignment succeeds. External heading inputs are ignored. |
| GPS + external heading, no LIO/VIO | `topics.odom: ""`, `heading.initialize_from_positions: false`; select dual-antenna or compass heading as above | Publishes at the fix rate; roll/pitch are level. |
| GPS only, no odometry or external heading | Unsupported for globally oriented pose output | Add LIO/VIO or an external heading source. |

**Without heading hardware:** the existing motion-initialization mode fits GPS
and odometry tracks to find global yaw. Defaults require at least 10 fixes spaced
one second apart and 10 m of horizontal span within a 60 s window, with a
consistent fit. Globally aligned output waits for initialization. Afterwards,
GPS positions and relative odometry continue fusion; yaw correction needs
translational motion, while yaw at rest or during GPS outages follows odometry.
This supports reversing and sideways motion: GPS course over ground is not
used directly as body heading. There is no explicit COG/velocity input.

For indoor/local operation, set `enable_gnss: false` and keep `topics.odom`
configured. GNOCCHI relays odometry into the output map frame without global
alignment. With `publish_tf: true`, `world -> map -> odom` are static identities.

### Select the GPS position input

Position input selection is independent of heading selection:

| Position source | Options to set |
|---|---|
| One receiver publishing `mavros_msgs/msg/GPSRAW` | `gnss.receivers: ["mavros/gpsstatus/gps1/raw"]` (use your receiver's topic) |
| Two receivers publishing `GPSRAW` | `gnss.receivers: ["mavros/gpsstatus/gps1/raw", "mavros/gpsstatus/gps2/raw"]`; GNOCCHI uses their midpoint |
| A `sensor_msgs/msg/NavSatFix` topic | `gnss.receivers: [""]` and `topics.gnss: <fix topic>` |

Set `extrinsics.gnss_antenna` to the antenna offset in the odometry child/body
frame (FLU); for two receivers, use their midpoint's offset. Check `frames.odom`,
`frames.base_link`, and `frames.expect_odom_child` against your odometry source.
For ZED VIO, use its continuous `zed/zed_node/odom` stream and launch the ZED with
`publish_map_tf:=false` so GNOCCHI can own `map -> odom`.

### Start publishing

Build after editing the package configs, then source the workspace and launch:

```bash
cd <workspace>
colcon build --packages-select gnocchi
source install/setup.bash
ros2 launch gnocchi ugv.launch.py
```

All launches accept `use_sim_time:=true` and `config:=<filename.yaml>` for a
config under the installed package's `config/` directory. For bag replay, start
GNOCCHI before playback and use `ros2 bag play <bag> --clock`.

The UAV profile also waits for an initialization trigger (`publish_on_trigger:
true`); the UGV and low-altitude UAV profiles do not. Send the trigger once when
ready, or set `publish_on_trigger: false` and restart:

```bash
ros2 topic pub --once /initialization_trigger std_msgs/msg/Bool '{data: true}'
```

The trigger enables publication; valid sensor inputs and estimator initialization
are still required. Fused profiles also wait for `optimizer.min_keyframes`
(default 5) keyframes before publishing.

## Outputs

Topics resolve under the launch namespace (`/ugv`, `/uav`, or `/lowalt_uav`):

| Topic | Type | Rate |
|---|---|---|
| `odom_map` | `nav_msgs/msg/Odometry` | Odometry rate when fused; fix rate in GPS-only mode |
| `pose_map` | `geometry_msgs/msg/PoseStamped` | Same as `odom_map` |
| `odom_map_throttled`, `pose_map_throttled` | Corresponding types above | At most 1 Hz |

The default map convention is NWU (+X north, +Y west), with its origin at the
first accepted fix. Set `frames.map_yaw_deg: 0.0` for ENU instead. With
`publish_tf: true`, GNOCCHI publishes static `world -> map` and, for fused
operation, live `map -> odom`. The odometry source owns `odom -> body`.
GPS-only operation publishes only the static TF edge.

See the [technical reference](docs/technical_reference.md) for frame conventions,
initialization thresholds, covariance and timing, failure handling, fallback,
height recovery, and validation results. Deployment-specific tuning notes remain
in the [configuration files](config/).
