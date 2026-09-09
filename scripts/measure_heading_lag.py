#!/usr/bin/env python3
"""Measure the GNSS heading's transport lag, from a bag or from live topics.

The dual-antenna heading is published with a stamp later than the epoch it
describes. Against a gravity-aligned lidar odometry yaw that reads

    r(t) = wrap(heading(t) - yaw_odom(t)) = b - L * omega(t)

a straight line in the yaw rate whose slope is -L. The intercept b is the
odom-to-ENU frame offset and is a nuisance here.

Two independent estimators are reported because agreement is the only real check:
a regression slope, and the time shift that minimises the residual spread. They
should agree to a few tens of ms.

    ros2 run gnocchi measure_heading_lag.py --bag /path/to/bag
    ros2 run gnocchi measure_heading_lag.py                 # live, ctrl-C to stop

L is observable only while turning. Drive some turns, and check the reported
yaw-rate spread before believing the answer.
"""
import argparse
import math
import sys

import numpy as np

ODOM_TOPIC = "/ugv/rko_lio/odometry"
HEADING_TOPICS = ["/ugv/mavros/gpsstatus/gps2/raw", "/ugv/mavros/gpsstatus/gps1/raw"]
MIN_SAMPLES = 100
MIN_SPREAD = 0.10        # rad/s; below this the fit has no leverage


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def yaw_from_quat(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n <= 0.0:
        return 0.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return math.atan2(2.0*(x*y + z*w), 1.0 - 2.0*(y*y + z*z))


def heading_enu(yaw_cdeg):
    """GPSRAW yaw is centidegrees clockwise from north."""
    return math.pi / 2.0 - math.radians(yaw_cdeg / 100.0)


def estimate(ot, oy, ht, hy, hacc=None):
    """Returns a report dict, or None if there is not enough to say anything."""
    if len(ot) < MIN_SAMPLES or len(ht) < 30:
        return None
    ot, ht = np.asarray(ot, float), np.asarray(ht, float)
    oy, hy = np.unwrap(np.asarray(oy, float)), np.unwrap(np.asarray(hy, float))
    keep = (ht >= ot[0]) & (ht <= ot[-1])
    ht, hy = ht[keep], hy[keep]
    if len(ht) < 30:
        return None

    omega = np.gradient(oy, ot)
    w = np.interp(ht, ot, omega)
    r = wrap(hy - np.interp(ht, ot, oy))
    r = wrap(r - np.median(r))

    spread = float(np.std(w))
    slope = float(np.polyfit(w, r, 1)[0])
    corr = float(np.corrcoef(w, r)[0, 1]) if spread > 1e-9 else float("nan")

    best_tau, best_std = 0.0, None
    for tau in np.arange(-0.60, 0.301, 0.005):
        rr = wrap(hy - np.interp(ht + tau, ot, oy))
        rr = wrap(rr - np.median(rr))
        s = float(np.degrees(np.std(rr)))
        if best_std is None or s < best_std:
            best_tau, best_std = float(tau), s

    return {
        "n": len(ht),
        "span": float(ot[-1] - ot[0]),
        "spread": spread,
        "peak_rate": float(np.max(np.abs(w))),
        "corr": corr,
        # slope = -L, and the config wants time_offset = -L
        "offset_slope": slope,
        "offset_shift": best_tau,
        "std_before": float(np.degrees(np.std(r))),
        "std_after": best_std,
        "hdg_acc": None if hacc is None or not len(hacc) else float(np.median(hacc)),
    }


def report(e, prefix=""):
    if e is None:
        print(f"{prefix}not enough data yet")
        return
    print(f"{prefix}{e['n']} headings over {e['span']:.0f} s   "
          f"yaw-rate spread {e['spread']:.2f} rad/s (peak {e['peak_rate']:.2f})")
    if e["spread"] < MIN_SPREAD:
        print(f"{prefix}  NOT ENOUGH TURNING -- the lag is unobservable when driving "
              f"straight. Drive some turns.")
        return
    print(f"{prefix}  correlation with yaw rate {e['corr']:+.3f}   "
          f"(a real lag shows a strong negative correlation)")
    print(f"{prefix}  residual {e['std_before']:.2f} -> {e['std_after']:.2f} deg"
          + (f"   receiver claims {e['hdg_acc']:.2f} deg" if e["hdg_acc"] else ""))
    a, b = e["offset_slope"], e["offset_shift"]
    print(f"{prefix}  heading.time_offset:  {a:+.3f} s (slope)   "
          f"{b:+.3f} s (shift)")
    if abs(a - b) > 0.06:
        print(f"{prefix}  WARNING: the two estimators disagree by "
              f"{abs(a-b)*1000:.0f} ms; treat the result as unreliable")
    else:
        print(f"{prefix}  ==> use  time_offset: {0.5*(a+b):+.3f}")
    print(f"{prefix}  (gnocchi wants the NEGATIVE of the lag; a lag of "
          f"{-0.5*(a+b)*1000:.0f} ms means time_offset {0.5*(a+b):+.3f})")


def from_bag(path, odom_topic, heading_topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if odom_topic not in types:
        sys.exit(f"{odom_topic} not in the bag. Available:\n  " +
                 "\n  ".join(sorted(types)))
    present = [t for t in heading_topics if t in types]
    if not present:
        sys.exit("none of the heading topics are in the bag: " + ", ".join(heading_topics))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[odom_topic] + present))

    ot, oy = [], []
    per_topic = {t: ([], [], []) for t in present}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == odom_topic:
            q = msg.pose.pose.orientation
            ot.append(stamp)
            oy.append(yaw_from_quat(q.x, q.y, q.z, q.w))
        else:
            if msg.yaw in (0, 65535):
                continue
            t, y, a = per_topic[topic]
            t.append(stamp); y.append(heading_enu(msg.yaw)); a.append(msg.hdg_acc * 1e-5)

    for topic in present:
        t, y, a = per_topic[topic]
        print(f"\n=== {topic} ===")
        if len(t) < 30:
            print("  no usable yaw from this instance "
                  "(a moving-baseline heading needs fix_type 5 or 6)")
            continue
        report(estimate(ot, oy, t, y, a), prefix="  ")


def live(odom_topic, heading_topics, period):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from mavros_msgs.msg import GPSRAW

    class Measure(Node):
        def __init__(self):
            super().__init__("measure_heading_lag")
            self.ot, self.oy = [], []
            self.h = {t: ([], [], []) for t in heading_topics}
            self.create_subscription(Odometry, odom_topic, self.on_odom,
                                     qos_profile_sensor_data)
            for t in heading_topics:
                self.create_subscription(
                    GPSRAW, t, lambda m, tt=t: self.on_hdg(m, tt),
                    qos_profile_sensor_data)
            self.create_timer(period, self.on_timer)
            self.get_logger().info(
                f"listening on {odom_topic} and {', '.join(heading_topics)}; "
                f"drive some turns, ctrl-C when done")

        def on_odom(self, m):
            q = m.pose.pose.orientation
            self.ot.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
            self.oy.append(yaw_from_quat(q.x, q.y, q.z, q.w))

        def on_hdg(self, m, topic):
            if m.yaw in (0, 65535):
                return
            t, y, a = self.h[topic]
            t.append(m.header.stamp.sec + m.header.stamp.nanosec*1e-9)
            y.append(heading_enu(m.yaw))
            a.append(m.hdg_acc * 1e-5)

        def on_timer(self):
            for topic in heading_topics:
                t, y, a = self.h[topic]
                if len(t) < 30:
                    continue
                print(f"\n=== {topic} ===")
                report(estimate(self.ot, self.oy, t, y, a), prefix="  ")

    rclpy.init()
    node = Measure()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nfinal estimate:")
        node.on_timer()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", help="measure from this bag instead of live topics")
    p.add_argument("--odom", default=ODOM_TOPIC)
    p.add_argument("--heading", nargs="+", default=HEADING_TOPICS)
    p.add_argument("--period", type=float, default=10.0,
                   help="seconds between live reports")
    a = p.parse_args()
    if a.bag:
        from_bag(a.bag, a.odom, a.heading)
    else:
        live(a.odom, a.heading, a.period)


if __name__ == "__main__":
    main()
