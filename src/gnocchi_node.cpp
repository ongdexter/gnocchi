#include "gnocchi/pose_graph.hpp"
#include "gnocchi/utm.hpp"
#include "gnocchi/yaw_filter.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace gnocchi
{

class GnocchiNode : public rclcpp::Node
{
public:
    GnocchiNode() : rclcpp::Node("gnocchi")
    {
        Options o;
        auto get = [&](const std::string& name, double def) {
            return this->declare_parameter(name, def);
        };
        o.keyframe_distance = get("keyframe.distance", o.keyframe_distance);
        o.keyframe_angle = get("keyframe.angle_deg", 10.0) * M_PI / 180.0;
        o.keyframe_max_time = get("keyframe.max_time", o.keyframe_max_time);
        o.odom_trans_floor = get("odom.translation_floor", o.odom_trans_floor);
        o.odom_rot_floor = get("odom.rotation_floor", o.odom_rot_floor);
        o.odom_trans_sigma_per_m = get("odom.translation_sigma_per_m", o.odom_trans_sigma_per_m);
        o.odom_rot_sigma_per_m = get("odom.rotation_sigma_per_m", o.odom_rot_sigma_per_m);
        o.odom_rot_sigma_per_rad = get("odom.rotation_sigma_per_rad", o.odom_rot_sigma_per_rad);
        o.gps_height.enabled = this->declare_parameter("gps_height.enabled", false);
        o.gps_height.sigma_floor = get("gps_height.sigma_floor", 3.0);
        gps_height_sigma_floor_ = o.gps_height.sigma_floor;
        o.gps_height.max_reported_sigma = get("gps_height.max_reported_sigma", 2.0);
        o.gps_height.max_speed = get("gps_height.max_speed", 2.0);
        o.gps_height.reference_window = get("gps_height.reference_window", 30.0);
        o.gps_height.reference_delay = get("gps_height.reference_delay", 3.0);
        o.gps_height.recovery_ramp_time = get("gps_height.recovery_ramp_time", 5.0);
        o.health.enabled = this->declare_parameter("lio_health.enabled", false);
        o.health.max_speed = get("lio_health.max_speed", 5.0);
        o.health.gps_max_speed = get("lio_health.gps_max_speed", 5.0);
        o.health.disagreement = get("lio_health.disagreement", 3.0);
        o.health.window = get("lio_health.window", 2.0);
        o.health.confirmation = get("lio_health.confirmation", 0.5);
        o.health.recovery = get("lio_health.recovery", 30.0);
        o.health.degraded_rotation_sigma = get("lio_health.degraded_rotation_sigma", 0.5);
        o.health.degraded_sigma = get("lio_health.degraded_translation_sigma", 10.0);
        fuse_gps_altitude_ = this->declare_parameter("gnss.fuse_altitude", false);
        o.gnss_sigma = get("gnss.sigma", o.gnss_sigma);
        o.gnss_max_sigma = get("gnss.max_sigma", o.gnss_max_sigma);
        o.gnss_robust_k = get("gnss.robust_k", o.gnss_robust_k);
        // Bounds on the receiver's reported hdg_acc.
        gnss_heading_sigma_floor_ = get("heading.gnss_sigma_floor", 0.005);
        gnss_heading_sigma_max_ = get("heading.gnss_sigma_max", 0.35);
        // Dual-antenna moving-baseline heading needs a receiver that actually
        // supports it. Where it isn't available, fall back to the FCU's own
        // compass_hdg estimate -- worse accuracy, no per-message reported
        // sigma (it's a bare Float64), so a fixed sigma is assumed instead.
        position_alignment_ = this->declare_parameter("heading.initialize_from_positions", false);
        o.alignment.enabled = position_alignment_;
        o.alignment.window = get("heading.alignment.window", 60.0);
        o.alignment.min_baseline = get("heading.alignment.min_baseline", 10.0);
        o.alignment.max_residual = get("heading.alignment.max_residual", 1.5);
        o.alignment.max_gap = get("heading.alignment.max_gap", 0.5);
        o.alignment.sigma_floor = get("heading.alignment.sigma_floor_deg", 10.0) * M_PI / 180.0;
        o.alignment.max_sigma = get("heading.alignment.max_sigma_deg", 20.0) * M_PI / 180.0;
        o.alignment.min_samples = this->declare_parameter("heading.alignment.min_samples", 10);
        use_gnss_heading_ = this->declare_parameter("heading.use_gnss_heading", true);
        compass_heading_sigma_ = get("heading.compass_sigma_deg", 5.0) * M_PI / 180.0;
        o.odom_attitude_sigma = get("odom.attitude_sigma", o.odom_attitude_sigma);
        o.odom_z_sigma = get("odom.z_sigma", o.odom_z_sigma);
        o.gnss_fuse_altitude = fuse_gps_altitude_;
        o.gnss_sigma_z = get("gnss.sigma_z", o.gnss_sigma_z);
        o.origin_z_sigma = get("origin_z_sigma", o.origin_z_sigma);
        o.max_measurement_age = get("max_measurement_age", o.max_measurement_age);
        o.gnss_min_interval = get("gnss.min_interval", o.gnss_min_interval);
        o.heading_min_interval = get("heading.min_interval", o.heading_min_interval);
        o.heading_time_offset = get("heading.time_offset", o.heading_time_offset);
        o.heading_robust_k = get("heading.robust_k", o.heading_robust_k);
        o.heading_max_gap = get("heading.max_gap", o.heading_max_gap);
        o.heading_max_rate = get("heading.max_rate", o.heading_max_rate);
        o.use_fixed_lag = this->declare_parameter("optimizer.fixed_lag", false);
        o.lag_time = get("optimizer.lag_time", o.lag_time);
        o.min_keyframes = static_cast<size_t>(this->declare_parameter("optimizer.min_keyframes", 5));
        options_ = o;

        // GNSS-only fallback while lidar odometry is missing (see README).
        fallback_enabled_ = this->declare_parameter("fallback.enabled", false);
        odom_timeout_ = get("fallback.odom_timeout", 0.5);
        startup_wait_ = get("fallback.startup_wait", 10.0);
        odom_jump_ = get("fallback.odom_jump", 2.0);
        rebuild_after_ = get("fallback.rebuild_after", 2.0);
        on_lio_degraded_ = this->declare_parameter("fallback.on_lio_degraded", false);
        blend_time_ = get("fallback.blend_time", 10.0);
        max_heading_age_ = get("fallback.max_heading_age", 30.0);
        altitude_tau_ = get("fallback.altitude_tau", 5.0);
        fallback_z_sigma_ = get("fallback.z_sigma", 1.0);
        fallback_tilt_sigma_ = get("fallback.tilt_sigma_deg", 3.0) * M_PI / 180.0;
        fused_yaw_sigma_floor_ = get("fallback.fused_yaw_sigma_floor_deg", 0.5) * M_PI / 180.0;
        YawFilterOptions yo;
        yo.gyro_noise = get("fallback.gyro_noise", yo.gyro_noise);
        yo.bias_random_walk = get("fallback.gyro_bias_random_walk", yo.bias_random_walk);
        yo.max_bias = get("fallback.gyro_max_bias_deg", 2.0) * M_PI / 180.0;
        yaw_filter_ = YawFilter(yo);


        // GNSS antenna in body frame (FLU). The fix reports the antenna, the estimate
        // reports base_link. Neither PX4 source is compensated on this airframe.
        debug_gnss_only_ = this->declare_parameter("debug.publish_gnss_only", false);

        lever_arm_ = Eigen::Vector3d(
            this->declare_parameter("extrinsics.gnss_antenna.x", 0.0),
            this->declare_parameter("extrinsics.gnss_antenna.y", 0.0),
            this->declare_parameter("extrinsics.gnss_antenna.z", 0.0));

        o.alignment.antenna = lever_arm_;

        world_frame_ = this->declare_parameter("frames.world", std::string("world"));
        map_frame_ = this->declare_parameter("frames.map", std::string("ugv_map"));
        odom_frame_ = this->declare_parameter("frames.odom", std::string("odom"));
        base_frame_ = this->declare_parameter("frames.base_link", std::string("base_link"));
        expected_odom_child_ = this->declare_parameter("frames.expect_odom_child", base_frame_);
        // Yaw of the local map frame's +X axis measured from ENU +X. 90 gives
        // NWU (+X north, +Y west); 0 would leave the map frame ENU.
        map_yaw_ = this->declare_parameter("frames.map_yaw_deg", 90.0) * M_PI / 180.0;
        publish_tf_ = this->declare_parameter("publish_tf", true);
        throttled_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->declare_parameter("topics.output_pose_throttled",
                                    std::string("pose_map_throttled")), rclcpp::QoS(1));
        throttled_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->declare_parameter("topics.output_odom_throttled",
                                    std::string("odom_map_throttled")), rclcpp::QoS(1));
        publishing_enabled_ = !this->declare_parameter("publish_on_trigger", false);
        const auto initialization_trigger_topic = this->declare_parameter(
            "initialization_trigger_topic", std::string("/initialization_trigger"));
        if (!publishing_enabled_)
        {
            initialization_trigger_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                initialization_trigger_topic, rclcpp::QoS(10),
                [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
                    if (msg->data && !publishing_enabled_.exchange(true))
                        RCLCPP_INFO(this->get_logger(),
                                    "Initialization trigger received; pose publishing enabled");
                });
            RCLCPP_INFO(this->get_logger(), "Waiting for initialization trigger on %s before publishing poses",
                        initialization_trigger_topic.c_str());
        }
        const bool enable_gnss = this->declare_parameter("enable_gnss", true);
        if (!enable_gnss)
        {
            if (publish_tf_)
            {
                static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
                geometry_msgs::msg::TransformStamped world_to_map;
                world_to_map.header.stamp = this->now();
                world_to_map.header.frame_id = world_frame_;
                world_to_map.child_frame_id = map_frame_;
                world_to_map.transform.rotation.w = 1.0;
                auto map_to_odom = world_to_map;
                map_to_odom.header.frame_id = map_frame_;
                map_to_odom.child_frame_id = odom_frame_;
                static_tf_->sendTransform(
                    std::vector<geometry_msgs::msg::TransformStamped>{world_to_map, map_to_odom});
            }
            const auto odom_topic = this->declare_parameter("topics.odom", std::string("odom"));
            if (odom_topic.empty())
            {
                RCLCPP_INFO(this->get_logger(), "GNSS and odometry both disabled; node is idle");
                return;
            }
            // LIO passthrough
            pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                this->declare_parameter("topics.output_odom", std::string("odom_map")), 10);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                this->declare_parameter("topics.output_pose", std::string("pose_map")), 10);
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onOdomPassthrough, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),
                        "GNSS disabled: LIO passthrough. %s -> %s -> %s identity (static), "
                        "relaying %s as odom_map/pose_map unfused and unheaded",
                        world_frame_.c_str(), map_frame_.c_str(), odom_frame_.c_str(),
                        odom_topic.c_str());
            return;
        }
        // The graph runs in local ENU about the datum; this rotates its output
        // into the map frame's axes.
        map_from_local_ = gtsam::Rot3::Rz(-map_yaw_);

        // No odometry topic means no clock: no keyframes, no graph. The node becomes a
        // GNSS-to-local-frame converter. This is the UAV profile.
        gps_only_ = this->declare_parameter("topics.odom", std::string("odom")).empty();
        if (position_alignment_ && gps_only_)
            throw std::invalid_argument("heading.initialize_from_positions requires topics.odom");
        graph_ = std::make_unique<PoseGraph>(o);
        degraded_pub_ = this->create_publisher<std_msgs::msg::Bool>("lio_degraded", 10);
        if (gps_only_)
            status_timer_ = this->create_wall_timer(std::chrono::seconds(5),
                                                  [this]() { reportGpsOnlyStatus(); });
        if (publish_tf_)
        {
            static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
            // gps_only has no odom frame to hand a live correction to, and no
            // other node's output it may claim instead (e.g. a VIO source
            // publishing its own map -> base_link) -- so it only ever gets the
            // static world -> map edge below, never a live broadcaster.
            if (!gps_only_)
            {
                tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            }
        }

        // One mutually exclusive group for every input, so the graph is touched from
        // one callback at a time.
        sensor_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto opts = rclcpp::SubscriptionOptions();
        opts.callback_group = sensor_group_;

        if (!gps_only_)
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                this->get_parameter("topics.odom").as_string(), rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onOdom, this, std::placeholders::_1), opts);
        }
        const auto imu_topic = this->declare_parameter("fallback.imu_topic", std::string(""));
        if (fallback_enabled_ && !gps_only_)
        {
            if (!imu_topic.empty())
                imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                    imu_topic, rclcpp::SensorDataQoS(),
                    std::bind(&GnocchiNode::onImu, this, std::placeholders::_1), opts);
            watchdog_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(50),
                                             [this]() { checkOdometry(); }, sensor_group_);
            RCLCPP_INFO(this->get_logger(),
                        "GNSS-only fallback armed: after %.1f s without odometry, publish GNSS position "
                        "with %s yaw (gyro bridges heading gaps up to %.0f s)",
                        odom_timeout_, imu_topic.empty() ? "dual-antenna/compass" : "gyro-propagated",
                        imu_topic.empty() ? 0.0 : max_heading_age_);
        }
        // gnss.receivers: one GPSRAW topic (a single receiver, at extrinsics.gnss_antenna)
        // or two (a dual-antenna install, fused as their midpoint; see onReceiverFix).
        // Empty: topics.gnss, any NavSatFix (e.g. the FCU's global_position/global).
        // ROS 2 parameter files cannot hold an empty list, so [""] also means none.
        auto receiver_topics = this->declare_parameter<std::vector<std::string>>(
            "gnss.receivers", std::vector<std::string>{});
        receiver_topics.erase(std::remove(receiver_topics.begin(), receiver_topics.end(), std::string()),
                              receiver_topics.end());
        if (receiver_topics.empty())
        {
            gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
                this->declare_parameter("topics.gnss", std::string("fix")),
                rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onGnss, this, std::placeholders::_1), opts);
        }
        else
        {
            if (receiver_topics.size() > 2)
                throw std::invalid_argument("gnss.receivers takes one topic per antenna, at most two");
            pair_max_dt_ = get("gnss.pair_max_dt", pair_max_dt_);
            min_offset_samples_ = static_cast<size_t>(
                this->declare_parameter("gnss.min_offset_samples", static_cast<int>(min_offset_samples_)));
            receivers_.resize(receiver_topics.size());
            for (size_t i = 0; i < receivers_.size(); ++i)
            {
                receivers_[i].topic = receiver_topics[i];
                receiver_subs_.push_back(this->create_subscription<mavros_msgs::msg::GPSRAW>(
                    receiver_topics[i], rclcpp::SensorDataQoS(),
                    [this, i](const mavros_msgs::msg::GPSRAW::ConstSharedPtr m) { onReceiverFix(m, i); },
                    opts));
            }
            if (receivers_.size() == 1)
                RCLCPP_INFO(this->get_logger(), "GNSS position: %s, antenna at extrinsics.gnss_antenna",
                            receiver_topics[0].c_str());
            else
                RCLCPP_INFO(this->get_logger(),
                            "GNSS position: midpoint of %s and %s (either alone, offset by what it "
                            "learns from paired fixes, when the other has no fix)",
                            receiver_topics[0].c_str(), receiver_topics[1].c_str());
        }
        // Optional: a fused relative-altitude topic (e.g. mavros' rel_alt, itself
        // barometer+GPS+IMU fused by the FCU) in place of the raw single-fix GNSS
        // altitude for the output pose's z. Empty (default) keeps the old behavior.
        const auto rel_altitude_topic = this->declare_parameter(
            "topics.rel_altitude", std::string(""));
        if (!rel_altitude_topic.empty())
        {
            rel_altitude_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                rel_altitude_topic, rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onRelAltitude, this, std::placeholders::_1), opts);
        }
        if (position_alignment_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Heading initialized from LIO/GNSS position alignment; external heading inputs disabled. "
                        "Move with valid GNSS to initialize; thereafter only position factors correct yaw.");
        }
        else if (use_gnss_heading_)
        {
            // GPS_RAW_INT carries the receiver's own yaw and its accuracy. Every instance is
            // subscribed; which one has the moving-baseline heading is wiring, not fixed.
            const auto heading_topics = this->declare_parameter<std::vector<std::string>>(
                "heading.gnss_topics",
                {"mavros/gpsstatus/gps1/raw", "mavros/gpsstatus/gps2/raw"});
            for (const auto& topic : heading_topics)
            {
                gnss_heading_subs_.push_back(
                    this->create_subscription<mavros_msgs::msg::GPSRAW>(
                        topic, rclcpp::SensorDataQoS(),
                        [this, topic](const mavros_msgs::msg::GPSRAW::ConstSharedPtr m) {
                            onGnssHeading(m, topic);
                        },
                        opts));
            }
        }
        else
        {
            const auto compass_topic = this->declare_parameter(
                "heading.compass_topic", std::string("mavros/global_position/compass_hdg"));
            compass_heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                compass_topic, rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onCompassHeading, this, std::placeholders::_1), opts);
        }


        // Both outputs carry the same pose from the same call.
        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->declare_parameter("topics.output_odom", std::string("odom_map")), 10);
        if (debug_gnss_only_)
        {
            gnss_only_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
                this->declare_parameter("debug.gnss_only_topic",
                                        std::string("debug/gnss_only")), 10);
            RCLCPP_INFO(this->get_logger(),
                        "debug: publishing the GNSS-only pose alongside the fused one");
        }
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            this->declare_parameter("topics.output_pose", std::string("pose_map")), 10);


        const char* heading_source = position_alignment_ ? "position-alignment initialization" : use_gnss_heading_
            ? "dual-antenna GNSS moving-baseline heading"
            : "FCU compass_hdg heading (fixed sigma)";
        if (gps_only_)
            RCLCPP_INFO(this->get_logger(),
                        "gnocchi up: GPS-only (no odometry, no graph), heading from %s. "
                        "%s -> %s (static, %.0f deg), output in %s at the fix rate",
                        heading_source,
                        world_frame_.c_str(), map_frame_.c_str(), map_yaw_ * 180.0 / M_PI,
                        map_frame_.c_str());
        else
            RCLCPP_INFO(this->get_logger(),
                        "gnocchi up: heading from %s. "
                        "%s -> %s (static, %.0f deg) -> %s (live), output in %s",
                        heading_source,
                        world_frame_.c_str(), map_frame_.c_str(), map_yaw_ * 180.0 / M_PI,
                        odom_frame_.c_str(), map_frame_.c_str());
    }

private:
    static int64_t ns(const builtin_interfaces::msg::Time& t)
    {
        return static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec;
    }

    // Frames are checked, not assumed. A silently wrong extrinsic is the most
    // expensive kind of bug in this pipeline, so a mismatch is loud.
    bool frameOk(const std::string& got, const std::string& want, const char* what)
    {
        if (want.empty() || got.empty() || got == want) return true;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "%s arrived in frame '%s' but '%s' was expected; "
                             "check the extrinsics before trusting the output",
                             what, got.c_str(), want.c_str());
        return true;
    }

    void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        frameOk(msg->header.frame_id, odom_frame_, "odometry");
        frameOk(msg->child_frame_id, expected_odom_child_, "odometry child");

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        const auto& q = msg->pose.pose.orientation;
        const auto& p = msg->pose.pose.position;
        pose.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
        pose.translation() = Eigen::Vector3d(p.x, p.y, p.z);
        if (!pose.matrix().allFinite()) return;

        const int64_t stamp = ns(msg->header.stamp);
        if (fallback_enabled_)
        {
            const rclcpp::Time now = this->now();
            // A restarted odometry source begins again at its own origin. If it came
            // back before the watchdog noticed, the jump gives it away.
            if (!in_fallback_ && have_odom_pose_)
            {
                const double jump = (pose.translation() - last_odom_pose_.translation()).norm();
                const double dt = std::abs(static_cast<double>(stamp - last_odom_ns_)) * 1e-9;
                if (jump > odom_jump_ && jump > 5.0 * dt)
                    enterFallback("odometry jumped " + std::to_string(jump) + " m");
            }
            if (in_fallback_ && !graph_rebuilt_ && !degraded_fallback_ && have_odom_pose_ &&
                static_cast<double>(stamp - last_odom_ns_) * 1e-9 <= rebuild_after_ &&
                (pose.translation() - last_odom_pose_.translation()).norm() <= odom_jump_)
            {
                // A short stall of the same source: its frame is unchanged, so the
                // existing chain simply continues.
                in_fallback_ = false;
                RCLCPP_WARN(this->get_logger(), "Odometry resumed after %.2f s in the same frame; fused output resumed",
                            static_cast<double>(stamp - last_odom_ns_) * 1e-9);
            }
            if (in_fallback_ && !graph_rebuilt_)
            {
                // Odometry is back, possibly in a new frame: the old chain cannot be
                // continued, so start a new graph at the current height.
                graph_ = std::make_unique<PoseGraph>(options_);
                if (have_fallback_z_) graph_->setInitialZ(fallback_z_);
                graph_rebuilt_ = true;
                have_odom_pose_ = false;
                RCLCPP_WARN(this->get_logger(),
                            "Odometry resumed; rebuilding the pose graph (GNSS-only output continues "
                            "until it has %zu keyframes)", options_.min_keyframes);
            }
            last_odom_rx_ = now;
            have_odom_rx_ = true;
        }
        last_odom_pose_ = pose;
        last_odom_ns_ = stamp;
        have_odom_pose_ = true;

        if (!graph_->initialized())
        {
            const std::string why = graph_->initBlockedReason(stamp);
            if (!why.empty())
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "not publishing yet: %s", why.c_str());
        }
        const bool was_initialized = graph_->initialized();
        if (graph_->addOdometry(stamp, pose))
        {
            // If these fall behind the keyframe count the inputs are stale and
            // the graph is running on odometry alone.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "keyframes %zu, gnss attached %zu, heading attached %zu",
                                 graph_->keyframes(), graph_->gnssAttached(),
                                 graph_->headingAttached());
        }
        if (position_alignment_ && !was_initialized && graph_->initialized())
            RCLCPP_INFO(this->get_logger(), "Position alignment initialized: ENU-from-odom yaw %.1f deg; no ongoing heading priors",
                        graph_->worldFromOdom().rotation().yaw() * 180.0 / M_PI);
        if (in_fallback_)
        {
            if (!graph_->publishable()) return;
            exitFallback();
        }
        // Optionally ride out a detected LIO fault on GNSS too: the faded entry
        // replaces the graph's abrupt GPS-dominant correction.
        if (fallback_enabled_ && on_lio_degraded_ && graph_->publishable())
        {
            const bool degraded = graph_->lioDegraded();
            if (degraded && !degraded_fallback_)
            {
                degraded_fallback_ = true;
                fused_healthy_ = false;
                fallback_entry_pending_ = true;
                RCLCPP_ERROR(this->get_logger(), "LIO degraded: publishing GNSS-only pose until it recovers");
            }
            else if (!degraded && degraded_fallback_)
            {
                degraded_fallback_ = false;
                RCLCPP_WARN(this->get_logger(), "LIO recovered: fused output resumed after %zu GNSS-only poses",
                            fallback_poses_);
                fallback_poses_ = 0;
            }
        }
        std_msgs::msg::Bool health;
        health.data = graph_->lioDegraded();
        degraded_pub_->publish(health);
        if (health.data != last_lio_degraded_) {
            RCLCPP_WARN(this->get_logger(), "LIO health changed: %s; motion disagreement %.2f m; GPS validated=%s",
                        health.data ? "DEGRADED (GPS-dominant recovery)" : "healthy", graph_->motionDisagreement(),
                        graph_->gpsValidated() ? "true" : "false");
            last_lio_degraded_ = health.data;
        }
        if (graph_->lioDegraded())
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "LIO degraded; GPS validated=%s; motion disagreement %.2f m; GPS height aiding=%s (factors=%zu). Navigation pose may jump during recovery.",
                                graph_->gpsValidated() ? "true" : "false", graph_->motionDisagreement(),
                                graph_->gpsHeightAiding() ? "true" : "false", graph_->gpsHeightFactors());
        if (!graph_->optimizerError().empty())
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Optimizer failures=%zu, last error: %s", graph_->optimizerFailures(), graph_->optimizerError().c_str());
        if (degraded_fallback_) return;
        publishFused(stamp);
    }

    // enable_gnss:=false path only: map/world/odom are identity (see
    // constructor), so the incoming pose needs no transform at all, just
    // relabeling into gnocchi's output frames/topics.
    void onOdomPassthrough(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        if (!publishing_enabled_) return;
        frameOk(msg->header.frame_id, odom_frame_, "odometry");
        frameOk(msg->child_frame_id, expected_odom_child_, "odometry child");

        nav_msgs::msg::Odometry out = *msg;
        out.header.frame_id = map_frame_;
        out.child_frame_id = base_frame_;
        publishOutputs(out);
    }

    void publishOutputs(const nav_msgs::msg::Odometry& msg)
    {
        pub_->publish(msg);
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg.header;
        pose_msg.pose = msg.pose.pose;
        pose_pub_->publish(pose_msg);
        if (gps_only_) {
            ++poses_published_;
            last_pose_published_ = StatusClock::now();
        }
        // Sample fresh output at most once per second of node time. Preserve
        // the measurement stamp; never repeat stale poses when inputs stop.
        const auto now = this->now();
        if (!last_throttled_publish_ || now < *last_throttled_publish_ ||
            (now - *last_throttled_publish_).nanoseconds() >= 1000000000LL)
        {
            throttled_pose_pub_->publish(pose_msg);
            throttled_odom_pub_->publish(msg);
            last_throttled_publish_ = now;
        }
    }

    void publishFused(int64_t stamp_ns)
    {
        if (!graph_->publishable() || !have_odom_pose_) return;
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        t.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        const gtsam::Pose3 pose = graph_->poseFor(last_odom_pose_);
        publish(pose, t);
        if (!fallback_enabled_) return;
        last_fused_ = pose;
        last_fused_ns_ = stamp_ns;
        // While the graph is healthy its yaw is the best there is: it keeps the
        // fallback's yaw and gyro bias current through dual-antenna gaps too.
        fused_healthy_ = !graph_->lioDegraded();
        if (fused_healthy_)
            yaw_filter_.measure(stamp_ns, pose.rotation().yaw(),
                                std::max(std::sqrt(std::max(graph_->state().covariance(2, 2), 0.0)),
                                         fused_yaw_sigma_floor_));
    }

    // ---- GNSS-only fallback -------------------------------------------------

    void onImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
        yaw_filter_.gyro(ns(msg->header.stamp), msg->angular_velocity.z);
    }

    // Feeds the yaw filter from the raw heading whenever the graph is not
    // already supplying a better yaw.
    void headingToFilter(int64_t stamp_ns, double yaw, double sigma)
    {
        if (!fallback_enabled_ || fused_healthy_) return;
        yaw_filter_.measure(stamp_ns, yaw, sigma);
    }

    void checkOdometry()
    {
        if (in_fallback_ || !datum_set_ || !have_first_fix_rx_) return;
        const rclcpp::Time now = this->now();
        if (have_odom_rx_)
        {
            const double age = (now - last_odom_rx_).seconds();
            if (age > odom_timeout_ && last_fused_ns_)
                enterFallback("no odometry for " + std::to_string(age).substr(0, 4) + " s");
            return;
        }
        // Never had odometry: after a grace period, GNSS alone beats no pose at all.
        if ((now - first_fix_rx_).seconds() > startup_wait_)
            enterFallback("no odometry since startup");
    }

    void enterFallback(const std::string& why)
    {
        in_fallback_ = true;
        graph_rebuilt_ = false;
        fused_healthy_ = false;
        // Already on GNSS (degraded LIO): keep the running output, no new entry step.
        if (!degraded_fallback_) fallback_entry_pending_ = true;
        degraded_fallback_ = false;
        RCLCPP_ERROR(this->get_logger(),
                     "LIDAR ODOMETRY LOST (%s): publishing GNSS-only pose; roll/pitch level, yaw %s. "
                     "Local accuracy is GNSS accuracy (~0.5 m).",
                     why.c_str(), imu_sub_ ? "from heading + gyro" : "from heading only");
    }

    void exitFallback()
    {
        in_fallback_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "Rebuilt pose graph is publishable; fused lidar/GNSS output resumed after %zu GNSS-only poses",
                    fallback_poses_);
        fallback_poses_ = 0;
    }

    // Body position the fix implies with the filter's yaw, for every fix, so the
    // fallback can line up with the last fused pose when it starts.
    void recordRaw(int64_t stamp_ns, const Eigen::Vector3d& antenna)
    {
        if (!yaw_filter_.initialized()) return;
        Eigen::Vector3d body = antenna;
        if (!lever_arm_.isZero())
            body -= gtsam::Rot3::Rz(yaw_filter_.yawAt(stamp_ns)).matrix() * lever_arm_;
        raw_.push_back({stamp_ns, body});
        while (raw_.size() > 2 && static_cast<double>(stamp_ns - raw_.front().stamp_ns) * 1e-9 > 5.0)
            raw_.pop_front();
    }

    std::optional<Eigen::Vector3d> rawAt(int64_t stamp_ns) const
    {
        if (raw_.empty()) return std::nullopt;
        if (stamp_ns <= raw_.front().stamp_ns) return raw_.front().body;
        for (size_t i = 1; i < raw_.size(); ++i)
            if (raw_[i].stamp_ns >= stamp_ns)
            {
                const double w = static_cast<double>(stamp_ns - raw_[i - 1].stamp_ns) /
                                 static_cast<double>(raw_[i].stamp_ns - raw_[i - 1].stamp_ns);
                return raw_[i - 1].body + w * (raw_[i].body - raw_[i - 1].body);
            }
        return raw_.back().body;
    }

    void publishFallback(int64_t stamp_ns, const builtin_interfaces::msg::Time& stamp, double sigma)
    {
        if (!yaw_filter_.valid(stamp_ns, max_heading_age_))
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                  "GNSS-only fallback has no usable yaw (last heading %.1f s ago); "
                                  "not publishing", yaw_filter_.measurementAge(stamp_ns));
            return;
        }
        const Eigen::Vector3d raw = raw_.back().body;
        if (fallback_entry_pending_)
        {
            // Start where the fused estimate left off and fade the difference out,
            // rather than stepping to GNSS.
            fallback_entry_pending_ = false;
            fallback_entry_ns_ = stamp_ns;
            const auto at_fused = last_fused_ns_ ? rawAt(last_fused_ns_) : std::nullopt;
            if (at_fused)
            {
                entry_offset_ = last_fused_.translation() - *at_fused;
                entry_offset_.z() = 0.0;
                z_entry_ = last_fused_.z();
                alt_ref_ = at_fused->z();
            }
            else
            {
                entry_offset_.setZero();
                z_entry_ = have_fallback_z_ ? fallback_z_ : raw.z();
                alt_ref_ = raw.z();
            }
            alt_lp_ = 0.0;
            last_alt_ns_ = stamp_ns;
            RCLCPP_WARN(this->get_logger(), "GNSS-only fallback: fading a %.2f m offset to the last fused pose over %.0f s",
                        entry_offset_.norm(), blend_time_);
        }
        const double since = static_cast<double>(stamp_ns - fallback_entry_ns_) * 1e-9;
        const double fade = blend_time_ > 0.0 ? std::clamp(1.0 - since / blend_time_, 0.0, 1.0) : 0.0;
        const Eigen::Vector3d offset = fade * entry_offset_;
        // Height: the GNSS altitude change since entry, low-passed (it wanders by
        // most of a metre), on top of the last fused height.
        if (altitude_tau_ > 0.0)
        {
            const double dt = std::max(static_cast<double>(stamp_ns - last_alt_ns_) * 1e-9, 0.0);
            const double a = 1.0 - std::exp(-dt / altitude_tau_);
            alt_lp_ += a * ((raw.z() - alt_ref_) - alt_lp_);
        }
        last_alt_ns_ = stamp_ns;
        const double z = z_entry_ + (altitude_tau_ > 0.0 ? alt_lp_ : 0.0);
        fallback_z_ = z;
        have_fallback_z_ = true;

        const double yaw = yaw_filter_.yawAt(stamp_ns);
        const gtsam::Pose3 pose(gtsam::Rot3::Rz(yaw),
                                gtsam::Point3(raw.x() + offset.x(), raw.y() + offset.y(), z));
        Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();
        const double s = std::max(sigma, options_.gnss_sigma);
        C(0, 0) = C(1, 1) = fallback_tilt_sigma_ * fallback_tilt_sigma_;
        C(2, 2) = std::pow(std::max(yaw_filter_.yawSigma(), 1e-3), 2);
        C(3, 3) = C(4, 4) = s * s + offset.squaredNorm();
        C(5, 5) = fallback_z_sigma_ * fallback_z_sigma_;
        std_msgs::msg::Bool health;
        health.data = true;
        degraded_pub_->publish(health);
        ++fallback_poses_;
        publish(pose, stamp, &C);
    }

    using StatusClock = std::chrono::steady_clock;
    using StatusTime = std::optional<StatusClock::time_point>;

    void reportGpsOnlyStatus()
    {
        const auto now = StatusClock::now();
        auto age = [&](const StatusTime& t) {
            return t ? std::chrono::duration<double>(now - *t).count() : -1.0;
        };
        const double fix_age = age(last_valid_fix_received_);
        const double heading_age = age(last_valid_heading_received_);
        const double output_age = age(last_pose_published_);
        std::string state;
        if (!publishing_enabled_) state = "waiting for initialization trigger";
        else if (!last_valid_fix_received_) state = "waiting for valid GNSS";
        else if (!have_heading_) state = "waiting for valid heading";
        else if (fix_age > 5.0) state = "GNSS input stale";
        else if (heading_age > 5.0) state = "heading input stale (cached heading still used)";
        else if (output_age < 0.0 || output_age > 5.0) state = "no recent pose output";
        else state = "publishing";
        RCLCPP_INFO(this->get_logger(),
                    "UAV GNSS status: %s | fix rx=%llu valid=%llu age=%.1fs last=%s | "
                    "heading rx=%llu valid=%llu age=%.1fs source=%s | poses=%llu age=%.1fs "
                    "(receipt ages; -1 means never)",
                    state.c_str(), static_cast<unsigned long long>(fix_received_),
                    static_cast<unsigned long long>(fix_valid_), fix_age, last_fix_status_.c_str(),
                    static_cast<unsigned long long>(heading_received_),
                    static_cast<unsigned long long>(heading_valid_), heading_age, heading_source_.c_str(),
                    static_cast<unsigned long long>(poses_published_), output_age);
    }

    void onGnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        ++fix_received_;
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
            last_fix_status_ = "rejected: no fix";
            return;
        }
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) {
            last_fix_status_ = "rejected: nonfinite position";
            return;
        }

        double northing = 0.0, easting = 0.0;
        char zone[8] = {};
        utm::toUtm(msg->latitude, msg->longitude, northing, easting, zone);

        const double variance = (fuse_gps_altitude_ || gps_only_)
            ? std::max({msg->position_covariance[0], msg->position_covariance[4], msg->position_covariance[8]})
            : std::max(msg->position_covariance[0], msg->position_covariance[4]);
        const double sigma = std::isfinite(variance) && variance > 1e-9 ? std::sqrt(variance) : 0.0;
        const double vertical_variance = msg->position_covariance[8];
        const double vertical_sigma = msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN &&
                                      std::isfinite(vertical_variance) && vertical_variance > 0
            ? std::sqrt(vertical_variance) : 0.0;
        acceptFix(ns(msg->header.stamp), msg->header.stamp, Eigen::Vector3d(easting, northing, msg->altitude),
                  zone, sigma, vertical_sigma);
        last_fix_status_ = "valid";
    }

    // Dual-antenna install (gnss.receivers). mavros' raw/fix is whichever receiver
    // PX4 numbered first, and that changes from boot to boot (the moving-baseline
    // rover in some field bags, the base in others, 0.47 m apart). The midpoint of
    // the two does not depend on that, and on this UGV it is base_link's horizontal
    // position, so extrinsics.gnss_antenna holds only the midpoint's height.
    //
    // The stamps are mavros receive times, not GNSS epochs, and the receivers run at
    // different rates, so the other receiver is extrapolated to this fix's stamp
    // with its own velocity. When the other has no fix (the rover loses it under
    // trees), this one is used alone, moved by its body-frame offset from the
    // midpoint as learned from paired fixes: no measured lever arm needed.
    void onReceiverFix(const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg, size_t i)
    {
        ++fix_received_;
        Receiver& r = receivers_[i];
        if (msg->fix_type < 3)  // GPS_FIX_TYPE_3D_FIX
        {
            r.valid = false;
            last_fix_status_ = r.topic + " rejected: fix_type " + std::to_string(msg->fix_type);
            return;
        }
        double northing = 0.0, easting = 0.0;
        char zone[8] = {};
        utm::toUtm(msg->lat * 1e-7, msg->lon * 1e-7, northing, easting, zone);
        // Ellipsoid height when the receiver reports it; either way both use the same one.
        const double alt = static_cast<double>(msg->alt_ellipsoid != 0 ? msg->alt_ellipsoid : msg->alt) * 1e-3;
        const Eigen::Vector3d p(easting, northing, alt);
        const int64_t t = ns(msg->header.stamp);

        const double since_last = static_cast<double>(t - r.stamp_ns) * 1e-9;
        r.velocity = r.valid && since_last > 0.0 && since_last < 0.5
            ? Eigen::Vector3d((p - r.position) / since_last) : Eigen::Vector3d::Zero();
        r.valid = true;
        r.stamp_ns = t;
        r.position = p;
        const double h = msg->h_acc > 0 ? msg->h_acc * 1e-3 : 0.0;  // mm; 0 = not reported
        r.vertical_sigma = msg->v_acc > 0 ? msg->v_acc * 1e-3 : 0.0;
        r.sigma = (fuse_gps_altitude_ || gps_only_) ? std::max(h, r.vertical_sigma) : h;

        if (receivers_.size() == 1)
        {
            acceptFix(t, msg->header.stamp, p, zone, r.sigma, r.vertical_sigma);
            last_fix_status_ = "valid";
            return;
        }
        const Receiver& o = receivers_[1 - i];
        const double to_other = static_cast<double>(t - o.stamp_ns) * 1e-9;
        if (o.valid && std::abs(to_other) <= pair_max_dt_)
        {
            // Moving-baseline RTK: the two errors are almost entirely shared, so the
            // midpoint is as uncertain as the worse receiver, not less.
            const Eigen::Vector3d mid = 0.5 * (p + o.position + o.velocity * to_other);
            learnOffsets(t, p - mid, i);
            acceptFix(t, msg->header.stamp, mid, zone, std::max(r.sigma, o.sigma),
                      std::max(r.vertical_sigma, o.vertical_sigma));
            last_fix_status_ = "valid (antenna midpoint)";
            return;
        }
        if (r.samples >= min_offset_samples_ && have_heading_)
        {
            const Eigen::Vector2d off = Eigen::Rotation2Dd(heading_) * r.offset;
            acceptFix(t, msg->header.stamp, p - Eigen::Vector3d(off.x(), off.y(), 0.0), zone,
                      r.sigma, r.vertical_sigma);
            last_fix_status_ = "valid (" + r.topic + " alone)";
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "Only %s has a fix: using it alone, offset (%.2f, %.2f) m FLU from the midpoint",
                                 r.topic.c_str(), r.offset.x(), r.offset.y());
            return;
        }
        last_fix_status_ = "waiting: only " + r.topic + " has a fix and its offset is not learned yet";
    }

    // Body-frame (FLU) offset of each antenna from the midpoint, a running mean
    // over paired fixes. The heading prior arrives ~0.24 s late (heading.time_offset),
    // so only learn while it is barely turning.
    void learnOffsets(int64_t t, const Eigen::Vector3d& from_mid, size_t i)
    {
        if (!have_heading_ || std::abs(heading_rate_) > 0.1 ||
            std::abs(static_cast<double>(t - gnss_heading_ns_)) * 1e-9 > 1.0)
            return;
        const Eigen::Vector2d body = Eigen::Rotation2Dd(-heading_) * from_mid.head<2>();
        for (size_t k = 0; k < 2; ++k)
        {
            Receiver& r = receivers_[k];
            const Eigen::Vector2d sample = k == i ? body : Eigen::Vector2d(-body);
            r.samples = std::min<size_t>(r.samples + 1, 1000);  // beyond 1000: an EMA
            r.offset += (sample - r.offset) / static_cast<double>(r.samples);
        }
        if (!offsets_reported_ && receivers_[0].samples >= min_offset_samples_)
        {
            offsets_reported_ = true;
            const Eigen::Vector2d baseline = receivers_[0].offset - receivers_[1].offset;
            RCLCPP_INFO(this->get_logger(),
                        "Antenna offsets from the midpoint (FLU): %s (%.3f, %.3f), %s (%.3f, %.3f); "
                        "baseline %.3f m",
                        receivers_[0].topic.c_str(), receivers_[0].offset.x(), receivers_[0].offset.y(),
                        receivers_[1].topic.c_str(), receivers_[1].offset.x(), receivers_[1].offset.y(),
                        baseline.norm());
        }
    }

    // A fix in UTM easting/northing/altitude, from either input path.
    void acceptFix(int64_t fix_ns, const builtin_interfaces::msg::Time& stamp, const Eigen::Vector3d& utm,
                   const char* zone, double sigma, double vertical_sigma)
    {
        ++fix_valid_;
        last_valid_fix_received_ = StatusClock::now();

        if (!datum_set_)
        {
            // Local metric frame: absolute UTM northings destroy float32 precision downstream.
            datum_ = utm;
            zone_ = zone;
            datum_set_ = true;
            RCLCPP_INFO(this->get_logger(), "datum: zone %s easting %.2f northing %.2f alt %.2f",
                        zone_.c_str(), utm.x(), utm.y(), utm.z());
            publishWorldToMap(stamp);
        }

        Eigen::Vector3d local = utm - datum_;
        // rel_altitude is already relative to home (and FCU-fused, not a raw
        // single fix), so it replaces the datum-differenced z outright rather
        // than needing datum_.z() subtracted from it too.
        if (have_rel_altitude_) local.z() = rel_altitude_;
        if (fallback_enabled_ && !gps_only_)
        {
            // Sim time reads 0 until the first /clock; do not start the grace period on it.
            const rclcpp::Time now = this->now();
            if (now.nanoseconds() > 0 && (!have_first_fix_rx_ || now < first_fix_rx_))
            {
                if (!have_first_fix_rx_ || now < first_fix_rx_) first_fix_rx_ = now;
                have_first_fix_rx_ = true;
            }
            recordRaw(fix_ns, local);
        }
        if (!position_alignment_) local -= leverArmOffset();
        graph_->setGnss(fix_ns, local, sigma, vertical_sigma);
        if (gps_only_) publishGpsOnly(local, sigma, stamp);
        if ((in_fallback_ || degraded_fallback_) && !raw_.empty() && raw_.back().stamp_ns == fix_ns &&
            sigma <= options_.gnss_max_sigma)
            publishFallback(fix_ns, stamp, sigma);
        if (debug_gnss_only_ && !position_alignment_) publishGnssOnlyDebug(local, stamp);
    }

    void onRelAltitude(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        rel_altitude_ = msg->data;
        have_rel_altitude_ = true;
    }

    // Antenna offset from the body origin, in local ENU. No roll/pitch source, so
    // the lever arm is only rotated by yaw; a non-level antenna mount will leak a
    // small residual offset into the fix.
    Eigen::Vector3d leverArmOffset() const
    {
        if (lever_arm_.isZero()) return Eigen::Vector3d::Zero();
        if (have_heading_) return gtsam::Rot3::Rz(heading_).matrix() * lever_arm_;
        return Eigen::Vector3d::Zero();
    }

    // GPS-only output: the fix offset from the datum, rotated into the map frame.
    // No IMU, so roll/pitch is always level (0); only yaw, from the heading prior,
    // is real attitude.
    std::optional<gtsam::Pose3> gnssOnlyPose(const Eigen::Vector3d& local) const
    {
        if (!have_heading_) return std::nullopt;
        return gtsam::Pose3(gtsam::Rot3::Rz(heading_), gtsam::Point3(local));
    }

    void publishGpsOnly(const Eigen::Vector3d& local, double sigma,
                        const builtin_interfaces::msg::Time& stamp)
    {
        const auto pose = gnssOnlyPose(local);
        if (!pose) return;
        gnss_sigma_ = sigma > 0.0 ? sigma : graph_->stateSigmaFallback();
        publish(*pose, stamp);
    }

    // Debug only: the same GNSS-only pose, published beside the fused estimate
    // in the same frame so the two can be overlaid directly.
    void publishGnssOnlyDebug(const Eigen::Vector3d& local,
                              const builtin_interfaces::msg::Time& stamp)
    {
        if (!publishing_enabled_) return;
        const auto pose = gnssOnlyPose(local);
        if (!pose) return;
        const gtsam::Pose3 m(map_from_local_ * pose->rotation(),
                             map_from_local_ * pose->translation());
        const gtsam::Quaternion q = m.rotation().toQuaternion();
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = base_frame_;
        msg.pose.pose.position.x = m.translation().x();
        msg.pose.pose.position.y = m.translation().y();
        msg.pose.pose.position.z = m.translation().z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        gnss_only_pub_->publish(msg);
    }

    // yaw: cdeg clockwise from north. 0 = receiver provides none, 65535 = cannot now.
    void onGnssHeading(const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg,
                       const std::string& topic)
    {
        ++heading_received_;
        if (msg->yaw == 0 || msg->yaw == 65535)
        {
            // Expected on the position-only receiver; only worth reporting if no
            // instance is supplying yaw at all.
            if (gnss_heading_ns_ == 0)
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                     "%s reports no yaw (yaw=%u, fix_type=%u). Waiting for "
                                     "an instance that does; a moving-baseline heading needs "
                                     "fix_type 5 or 6.",
                                     topic.c_str(), msg->yaw, msg->fix_type);
            return;
        }
        const double deg = static_cast<double>(msg->yaw) / 100.0;
        const double enu = M_PI / 2.0 - deg * M_PI / 180.0;

        // hdg_acc is degE5; 0 means the receiver reported no uncertainty.
        double sigma = gnss_heading_sigma_floor_;
        if (msg->hdg_acc > 0)
            sigma = static_cast<double>(msg->hdg_acc) * 1e-5 * M_PI / 180.0;
        sigma = std::max(sigma, gnss_heading_sigma_floor_);
        if (sigma > gnss_heading_sigma_max_)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "GNSS heading rejected: reported sigma %.2f deg exceeds "
                                 "the %.2f deg ceiling", sigma * 180.0 / M_PI,
                                 gnss_heading_sigma_max_ * 180.0 / M_PI);
            return;
        }

        RCLCPP_INFO_ONCE(this->get_logger(),
                         "GNSS heading from %s (sigma %.3f deg, fix_type %u)",
                         topic.c_str(), sigma * 180.0 / M_PI, msg->fix_type);
        const int64_t heading_ns = ns(msg->header.stamp);
        const double since = static_cast<double>(heading_ns - gnss_heading_ns_) * 1e-9;
        if (have_heading_ && since > 0.0 && since < 1.0)
            heading_rate_ = std::remainder(enu - heading_, 2.0 * M_PI) / since;
        heading_ = std::atan2(std::sin(enu), std::cos(enu));
        have_heading_ = true;
        ++heading_valid_;
        last_valid_heading_received_ = StatusClock::now();
        heading_source_ = topic;
        last_heading_sigma_ = sigma;
        gnss_heading_ns_ = ns(msg->header.stamp);
        graph_->setHeading(gnss_heading_ns_, heading_, sigma);
        headingToFilter(gnss_heading_ns_ + static_cast<int64_t>(options_.heading_time_offset * 1e9), heading_, sigma);
    }

    // Fallback when heading.use_gnss_heading is false: the FCU's own fused
    // heading estimate (typically magnetic compass, possibly GPS-course-aided
    // in forward flight), same compass-bearing convention as GPSRAW's yaw but
    // in plain degrees. A bare Float64 carries no accuracy or timestamp, so a
    // fixed sigma is assumed and the node's own clock stands in for a stamp.
    void onCompassHeading(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        ++heading_received_;
        if (!std::isfinite(msg->data)) return;
        const double enu = M_PI / 2.0 - msg->data * M_PI / 180.0;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Compass heading in use (fixed sigma %.2f deg)",
                         compass_heading_sigma_ * 180.0 / M_PI);
        heading_ = std::atan2(std::sin(enu), std::cos(enu));
        have_heading_ = true;
        ++heading_valid_;
        last_valid_heading_received_ = StatusClock::now();
        heading_source_ = "FCU compass_hdg";
        last_heading_sigma_ = compass_heading_sigma_;
        gnss_heading_ns_ = this->now().nanoseconds();
        graph_->setHeading(gnss_heading_ns_, heading_, compass_heading_sigma_);
        headingToFilter(gnss_heading_ns_ + static_cast<int64_t>(options_.heading_time_offset * 1e9), heading_,
                        compass_heading_sigma_);
    }


    // world -> map: a translation to the datum plus the map frame's yaw. Static,
    // because the datum is fixed once the first fix is accepted.
    void publishWorldToMap(const builtin_interfaces::msg::Time& stamp)
    {
        if (!publish_tf_) return;
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = map_frame_;
        tf.transform.translation.x = datum_.x();
        tf.transform.translation.y = datum_.y();
        tf.transform.translation.z = datum_.z();
        tf.transform.rotation.w = std::cos(map_yaw_ / 2.0);
        tf.transform.rotation.z = std::sin(map_yaw_ / 2.0);
        static_tf_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(), "published static %s -> %s",
                    world_frame_.c_str(), map_frame_.c_str());
    }

    void publish(const gtsam::Pose3& local_pose, const builtin_interfaces::msg::Time& stamp,
                 const Eigen::Matrix<double, 6, 6>* covariance = nullptr)
    {
        if (!publishing_enabled_) return;
        const gtsam::Pose3 pose(map_from_local_ * local_pose.rotation(),
                                map_from_local_ * local_pose.translation());
        const gtsam::Point3 t = pose.translation();
        const gtsam::Quaternion q = pose.rotation().toQuaternion();

        nav_msgs::msg::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        msg.child_frame_id = base_frame_;
        msg.pose.pose.position.x = t.x();
        msg.pose.pose.position.y = t.y();
        msg.pose.pose.position.z = t.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();

        // Rotate the covariance into the map frame, then reorder: GTSAM is [rot,trans],
        // ROS is [trans,rot].
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
        A.block<3, 3>(0, 0) = map_from_local_.matrix();
        A.block<3, 3>(3, 3) = map_from_local_.matrix();
        Eigen::Matrix<double, 6, 6> Craw = graph_->state().covariance;
        if (!gps_only_ && graph_->lioDegraded()) {
            // Health allowance, not a calibrated posterior under sensor failure.
            for (int i = 0; i < 3; ++i) {
                Craw(i, i) = std::max(Craw(i, i), 0.25);
                Craw(i + 3, i + 3) = std::max(Craw(i + 3, i + 3), graph_->gpsValidated() ? 1.0 : 25.0);
            }
        }
        if (!gps_only_ && graph_->gpsHeightAiding())
            Craw(5, 5) = std::max(Craw(5, 5), gps_height_sigma_floor_ * gps_height_sigma_floor_);
        if (covariance) Craw = *covariance;
        if (gps_only_)
        {
            Craw.setZero();
            const double s2 = gnss_sigma_ * gnss_sigma_;
            Craw(0, 0) = Craw(1, 1) = 0.05 * 0.05;              // roll, pitch
            Craw(2, 2) = last_heading_sigma_ * last_heading_sigma_;  // yaw
            Craw(3, 3) = Craw(4, 4) = s2;
            Craw(5, 5) = s2 * 4.0;                               // GNSS vertical is worse
        }
        Eigen::Matrix<double, 6, 6> C = A * Craw * A.transpose();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                msg.pose.covariance[i * 6 + j] = C(3 + i, 3 + j);          // trans-trans
                msg.pose.covariance[(i + 3) * 6 + (j + 3)] = C(i, j);      // rot-rot
                msg.pose.covariance[i * 6 + (j + 3)] = C(3 + i, j);        // trans-rot
                msg.pose.covariance[(i + 3) * 6 + j] = C(i, 3 + j);        // rot-trans
            }
        publishOutputs(msg);

        // gps_only has no odom frame, and nothing in this stack looks up base_link
        // via TF -- every consumer reads the pose off this topic instead. So gnocchi
        // only ever hands out world -> map; leave base_link to whatever owns it
        // (e.g. a VIO source's own map -> base_link).
        // In fallback there is no live odom frame to align.
        if (!publish_tf_ || gps_only_ || covariance) return;
        // map -> odom, the live alignment.
        const gtsam::Pose3 l = graph_->worldFromOdom();
        const gtsam::Pose3 c = gtsam::Pose3(map_from_local_ * l.rotation(),
                                             map_from_local_ * l.translation());
        const gtsam::Quaternion cq = c.rotation().toQuaternion();
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = map_frame_;
        tf.child_frame_id = odom_frame_;
        tf.transform.translation.x = c.translation().x();
        tf.transform.translation.y = c.translation().y();
        tf.transform.translation.z = c.translation().z();
        tf.transform.rotation.w = cq.w();
        tf.transform.rotation.x = cq.x();
        tf.transform.rotation.y = cq.y();
        tf.transform.rotation.z = cq.z();
        tf_->sendTransform(tf);
    }

    std::unique_ptr<PoseGraph> graph_;
    Options options_;

    // GNSS-only fallback state.
    bool fallback_enabled_ = false, in_fallback_ = false, graph_rebuilt_ = false;
    bool fused_healthy_ = false, fallback_entry_pending_ = false;
    bool on_lio_degraded_ = false, degraded_fallback_ = false;
    double rebuild_after_ = 2.0;
    double odom_timeout_ = 0.5, startup_wait_ = 10.0, odom_jump_ = 2.0, blend_time_ = 10.0;
    double max_heading_age_ = 30.0, altitude_tau_ = 5.0, fallback_z_sigma_ = 1.0;
    double fallback_tilt_sigma_ = 0.05, fused_yaw_sigma_floor_ = 0.01;
    YawFilter yaw_filter_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;
    rclcpp::Time last_odom_rx_{0, 0, RCL_ROS_TIME}, first_fix_rx_{0, 0, RCL_ROS_TIME};
    bool have_odom_rx_ = false, have_first_fix_rx_ = false;
    gtsam::Pose3 last_fused_;
    int64_t last_fused_ns_ = 0, fallback_entry_ns_ = 0, last_alt_ns_ = 0;
    struct RawFix { int64_t stamp_ns; Eigen::Vector3d body; };
    std::deque<RawFix> raw_;
    Eigen::Vector3d entry_offset_ = Eigen::Vector3d::Zero();
    double z_entry_ = 0.0, alt_ref_ = 0.0, alt_lp_ = 0.0, fallback_z_ = 0.0;
    bool have_fallback_z_ = false;
    size_t fallback_poses_ = 0;
    rclcpp::CallbackGroup::SharedPtr sensor_group_;
    std::atomic<bool> publishing_enabled_{true};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr initialization_trigger_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_altitude_sub_;
    bool have_rel_altitude_ = false;
    double rel_altitude_ = 0.0;
    std::vector<rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr> gnss_heading_subs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_heading_sub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    StatusTime last_valid_fix_received_, last_valid_heading_received_, last_pose_published_;
    uint64_t fix_received_ = 0, fix_valid_ = 0;
    uint64_t heading_received_ = 0, heading_valid_ = 0, poses_published_ = 0;
    std::string last_fix_status_ = "none", heading_source_ = "none";
    double gps_height_sigma_floor_ = 3.0;
    bool fuse_gps_altitude_ = false, last_lio_degraded_ = false;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr degraded_pub_;
    bool position_alignment_ = false;
    bool use_gnss_heading_ = true;
    double compass_heading_sigma_ = 5.0 * M_PI / 180.0;
    double gnss_heading_sigma_floor_ = 0.005, gnss_heading_sigma_max_ = 0.35;
    int64_t gnss_heading_ns_ = 0;
    double heading_rate_ = 0.0;  // rad/s, between consecutive GNSS headings

    // gnss.receivers: one entry per antenna.
    struct Receiver
    {
        std::string topic;
        bool valid = false;
        int64_t stamp_ns = 0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), velocity = Eigen::Vector3d::Zero();
        double sigma = 0.0, vertical_sigma = 0.0;
        Eigen::Vector2d offset = Eigen::Vector2d::Zero();  // FLU, from the midpoint
        size_t samples = 0;
    };
    std::vector<Receiver> receivers_;
    std::vector<rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr> receiver_subs_;
    double pair_max_dt_ = 0.25;
    size_t min_offset_samples_ = 100;
    bool offsets_reported_ = false;

    // Extrapolation state. None of this reaches the graph.
    Eigen::Isometry3d last_odom_pose_ = Eigen::Isometry3d::Identity();
    int64_t last_odom_ns_ = 0;
    bool have_odom_pose_ = false;
    bool gps_only_ = false, have_heading_ = false;
    double heading_ = 0.0, gnss_sigma_ = 1.0;
    // Sigma of the most recent GNSS heading, for the GPS-only covariance.
    double last_heading_sigma_ = 0.005;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr throttled_pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr throttled_odom_pub_;
    std::optional<rclcpp::Time> last_throttled_publish_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gnss_only_pub_;
    bool debug_gnss_only_ = false;

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
    std::string world_frame_, map_frame_, odom_frame_, base_frame_, expected_odom_child_, zone_;
    double map_yaw_ = M_PI / 2.0;
    gtsam::Rot3 map_from_local_;
    bool publish_tf_ = true, datum_set_ = false;
    Eigen::Vector3d datum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d lever_arm_ = Eigen::Vector3d::Zero();
};

}  // namespace gnocchi

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gnocchi::GnocchiNode>());
    rclcpp::shutdown();
    return 0;
}
