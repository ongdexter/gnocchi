#include "gnocchi/pose_graph.hpp"
#include "gnocchi/utm.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
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
        use_gnss_heading_ = this->declare_parameter("heading.use_gnss_heading", true);
        compass_heading_sigma_ = get("heading.compass_sigma_deg", 5.0) * M_PI / 180.0;
        o.odom_attitude_sigma = get("odom.attitude_sigma", o.odom_attitude_sigma);
        o.odom_z_sigma = get("odom.z_sigma", o.odom_z_sigma);
        o.gnss_fuse_altitude = this->declare_parameter("gnss.fuse_altitude", false);
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


        // GNSS antenna in body frame (FLU). The fix reports the antenna, the estimate
        // reports base_link. Neither PX4 source is compensated on this airframe.
        debug_gnss_only_ = this->declare_parameter("debug.publish_gnss_only", false);

        lever_arm_ = Eigen::Vector3d(
            this->declare_parameter("extrinsics.gnss_antenna.x", 0.0),
            this->declare_parameter("extrinsics.gnss_antenna.y", 0.0),
            this->declare_parameter("extrinsics.gnss_antenna.z", 0.0));

        world_frame_ = this->declare_parameter("frames.world", std::string("world"));
        map_frame_ = this->declare_parameter("frames.map", std::string("ugv_map"));
        odom_frame_ = this->declare_parameter("frames.odom", std::string("odom"));
        base_frame_ = this->declare_parameter("frames.base_link", std::string("base_link"));
        expected_odom_child_ = this->declare_parameter("frames.expect_odom_child", base_frame_);
        // Yaw of the local map frame's +X axis measured from ENU +X. 90 gives
        // NWU (+X north, +Y west); 0 would leave the map frame ENU.
        map_yaw_ = this->declare_parameter("frames.map_yaw_deg", 90.0) * M_PI / 180.0;
        publish_tf_ = this->declare_parameter("publish_tf", true);
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
        graph_ = std::make_unique<PoseGraph>(o);
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
        auto group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto opts = rclcpp::SubscriptionOptions();
        opts.callback_group = group;

        if (!gps_only_)
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                this->get_parameter("topics.odom").as_string(), rclcpp::SensorDataQoS(),
                std::bind(&GnocchiNode::onOdom, this, std::placeholders::_1), opts);
        }
        gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            this->declare_parameter("topics.gnss", std::string("fix")),
            rclcpp::SensorDataQoS(),
            std::bind(&GnocchiNode::onGnss, this, std::placeholders::_1), opts);
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
        if (use_gnss_heading_)
        {
            // GPS_RAW_INT carries the receiver's own yaw and its accuracy. Every instance is
            // subscribed; which one has the moving-baseline heading is wiring, not fixed.
            const auto heading_topics = this->declare_parameter<std::vector<std::string>>(
                "topics.gnss_heading",
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


        const char* heading_source = use_gnss_heading_
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
        if (graph_->addOdometry(stamp, pose))
        {
            // If these fall behind the keyframe count the inputs are stale and
            // the graph is running on odometry alone.
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
                                 "keyframes %zu, gnss attached %zu, heading attached %zu",
                                 graph_->keyframes(), graph_->gnssAttached(),
                                 graph_->headingAttached());
        }
        publishFused(stamp);
    }

    // enable_gnss:=false path only: map/world/odom are identity (see
    // constructor), so the incoming pose needs no transform at all, just
    // relabeling into gnocchi's output frames/topics.
    void onOdomPassthrough(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        frameOk(msg->header.frame_id, odom_frame_, "odometry");
        frameOk(msg->child_frame_id, expected_odom_child_, "odometry child");

        nav_msgs::msg::Odometry out = *msg;
        out.header.frame_id = map_frame_;
        out.child_frame_id = base_frame_;
        pub_->publish(out);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = out.header;
        pose_msg.pose = out.pose.pose;
        pose_pub_->publish(pose_msg);
    }

    void publishFused(int64_t stamp_ns)
    {
        if (!graph_->publishable() || !have_odom_pose_) return;
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
        t.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
        publish(graph_->poseFor(last_odom_pose_), t);
    }

    void onGnss(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
    {
        if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
        if (!std::isfinite(msg->latitude) || !std::isfinite(msg->longitude)) return;

        double northing = 0.0, easting = 0.0;
        char zone[8] = {};
        utm::toUtm(msg->latitude, msg->longitude, northing, easting, zone);

        if (!datum_set_)
        {
            // Local metric frame: absolute UTM northings destroy float32 precision downstream.
            datum_ = Eigen::Vector3d(easting, northing, msg->altitude);
            zone_ = zone;
            datum_set_ = true;
            RCLCPP_INFO(this->get_logger(), "datum: zone %s easting %.2f northing %.2f alt %.2f",
                        zone_.c_str(), easting, northing, msg->altitude);
            publishWorldToMap(msg->header.stamp);
        }

        const double variance = std::max({msg->position_covariance[0], msg->position_covariance[4],
                                          msg->position_covariance[8]});
        const double sigma = std::isfinite(variance) && variance > 1e-9 ? std::sqrt(variance) : 0.0;
        Eigen::Vector3d local = Eigen::Vector3d(easting, northing, msg->altitude) - datum_;
        // rel_altitude is already relative to home (and FCU-fused, not a raw
        // single fix), so it replaces the datum-differenced z outright rather
        // than needing datum_.z() subtracted from it too.
        if (have_rel_altitude_) local.z() = rel_altitude_;
        local -= leverArmOffset();
        graph_->setGnss(ns(msg->header.stamp), local, sigma);
        if (gps_only_) publishGpsOnly(local, sigma, msg->header.stamp);
        if (debug_gnss_only_) publishGnssOnlyDebug(local, msg->header.stamp);
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
        heading_ = std::atan2(std::sin(enu), std::cos(enu));
        have_heading_ = true;
        last_heading_sigma_ = sigma;
        gnss_heading_ns_ = ns(msg->header.stamp);
        graph_->setHeading(gnss_heading_ns_, heading_, sigma);
    }

    // Fallback when heading.use_gnss_heading is false: the FCU's own fused
    // heading estimate (typically magnetic compass, possibly GPS-course-aided
    // in forward flight), same compass-bearing convention as GPSRAW's yaw but
    // in plain degrees. A bare Float64 carries no accuracy or timestamp, so a
    // fixed sigma is assumed and the node's own clock stands in for a stamp.
    void onCompassHeading(const std_msgs::msg::Float64::ConstSharedPtr msg)
    {
        const double enu = M_PI / 2.0 - msg->data * M_PI / 180.0;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "Compass heading in use (fixed sigma %.2f deg)",
                         compass_heading_sigma_ * 180.0 / M_PI);
        heading_ = std::atan2(std::sin(enu), std::cos(enu));
        have_heading_ = true;
        last_heading_sigma_ = compass_heading_sigma_;
        gnss_heading_ns_ = this->now().nanoseconds();
        graph_->setHeading(gnss_heading_ns_, heading_, compass_heading_sigma_);
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

    void publish(const gtsam::Pose3& local_pose, const builtin_interfaces::msg::Time& stamp)
    {
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
        pub_->publish(msg);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg.header;
        pose_msg.pose = msg.pose.pose;
        pose_pub_->publish(pose_msg);

        // gps_only has no odom frame, and nothing in this stack looks up base_link
        // via TF -- every consumer reads the pose off this topic instead. So gnocchi
        // only ever hands out world -> map; leave base_link to whatever owns it
        // (e.g. a VIO source's own map -> base_link).
        if (!publish_tf_ || gps_only_) return;
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_altitude_sub_;
    bool have_rel_altitude_ = false;
    double rel_altitude_ = 0.0;
    std::vector<rclcpp::Subscription<mavros_msgs::msg::GPSRAW>::SharedPtr> gnss_heading_subs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr compass_heading_sub_;
    bool use_gnss_heading_ = true;
    double compass_heading_sigma_ = 5.0 * M_PI / 180.0;
    double gnss_heading_sigma_floor_ = 0.005, gnss_heading_sigma_max_ = 0.35;
    int64_t gnss_heading_ns_ = 0;

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
