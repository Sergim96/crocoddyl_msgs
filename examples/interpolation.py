#!/usr/bin/env python3

###############################################################################
# BSD 3-Clause License
#
# Example: demonstrate REPLACE, APPEND and MERGE behavior of
# SolverTrajectoryRosSubscriber and compare references with/without
# interpolation (sample-based blending), including the publisher trajectories.
###############################################################################

import os
import time

import matplotlib.pyplot as plt
import numpy as np

ROS_VERSION = int(os.environ["ROS_VERSION"])
if ROS_VERSION == 2:
    import rclpy
    from rclpy.node import Node
else:
    import rospy

from crocoddyl_ros import (
    ControlParametrization,
    ControlType,
    SolverTrajectoryRosPublisher,
    SolverTrajectoryRosSubscriber,
)


def build_traj1(h=0.05, horizon=1.0, t0=None):
    """Build first trajectory (traj1)."""
    if ROS_VERSION == 2:
        if t0 is None:
            t0 = rclpy.clock.Clock().now().seconds_nanoseconds()[0]
    else:
        if t0 is None:
            t0 = rospy.Time.now().to_sec()

    ts = [t0 + i * h for i in range(int(horizon / h))]
    N = len(ts)
    dts = [h] * N

    nx = 4
    nu = 2

    xs, dxs, us, Ks = [], [], [], []
    for k in range(N):
        t = ts[k] - t0
        x = np.array(
            [
                np.sin(0.5 * t),
                np.cos(0.5 * t),
                0.1 * np.sin(1.0 * t),
                0.1 * np.cos(1.0 * t),
            ]
        )
        dx = np.array(
            [
                0.5 * np.cos(0.5 * t),
                -0.5 * np.sin(0.5 * t),
                0.1 * np.cos(1.0 * t),
                -0.1 * np.sin(1.0 * t),
            ]
        )
        u = np.array(
            [
                0.5 * np.sin(0.8 * t),
                0.5 * np.cos(0.8 * t),
            ]
        )
        K = 0.1 * np.random.randn(nu, nx)
        xs.append(x)
        dxs.append(dx)
        us.append(u)
        Ks.append(K)

    type_ = ControlType.EFFORT
    param_ = ControlParametrization.POLYZERO
    types = [type_] * N
    params = [param_] * N

    return t0, ts, dts, xs, dxs, us, Ks, types, params


def build_traj2(mode, t0_1, ts1, dts1, h=0.05, horizon=1.0):
    """Build second trajectory (traj2) with timing to trigger mode."""
    tN1 = ts1[-1] + dts1[-1]

    if mode == "append":
        # APPEND: new starts clearly after current ends
        gap = 0.1
        t0_2 = tN1 + gap
    elif mode == "replace":
        # REPLACE: same start time
        t0_2 = t0_1
    elif mode == "merge":
        # MERGE: partial overlap. Start in the middle of traj1.
        t0_2 = t0_1 + 0.5 * (tN1 - t0_1)
    else:
        raise ValueError(f"Unknown mode '{mode}'")

    ts2 = [t0_2 + i * h for i in range(int(horizon / h))]
    N2 = len(ts2)
    dts2 = [h] * N2

    nx = 4
    nu = 2

    xs2, dxs2, us2, Ks2 = [], [], [], []
    for k in range(N2):
        t = ts2[k] - t0_2
        # Different pattern so we see a visible change
        x = np.array(
            [
                1.0 + 0.5 * np.sin(0.5 * t),
                1.0 + 0.5 * np.cos(0.5 * t),
                0.2 * np.sin(1.0 * t),
                0.2 * np.cos(1.0 * t),
            ]
        )
        dx = np.array(
            [
                0.5 * np.cos(0.5 * t),
                -0.5 * np.sin(0.5 * t),
                0.2 * np.cos(1.0 * t),
                -0.2 * np.sin(1.0 * t),
            ]
        )
        u = np.array(
            [
                -0.5 * np.sin(0.8 * t) + 0.2,
                -0.5 * np.cos(0.8 * t) + 0.2,
            ]
        )
        K = 0.1 * np.random.randn(nu, nx)
        xs2.append(x)
        dxs2.append(dx)
        us2.append(u)
        Ks2.append(K)

    type_ = ControlType.EFFORT
    param_ = ControlParametrization.POLYZERO
    types2 = [type_] * N2
    params2 = [param_] * N2

    return t0_2, ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2


def run_case(mode, pub, topic="/crocoddyl/solver_trajectory",
             interpolation_window=5):
    """
    Run one of the three modes: 'replace', 'append', 'merge'.

    Returns a dict with logs for plotting, including publisher trajectories:
        {
          "t_no", "t_interp",
          "x0_no", "x0_interp",
          "u0_no", "u0_interp",
          "t0_2",
          "pub_t1", "pub_x0_1", "pub_u0_1",
          "pub_t2", "pub_x0_2", "pub_u0_2",
        }
    """
    print(f"\n==================== CASE: {mode.upper()} ====================")

    # Subscribers for this case
    sub_no_interp = SolverTrajectoryRosSubscriber(topic=topic)
    sub_interp = SolverTrajectoryRosSubscriber(
        topic=topic, interpolation=True, interpolation_window=interpolation_window
    )

    time.sleep(1.0)  # let ROS connections settle

    # --- Build first trajectory (traj1) ---
    h = 0.05
    horizon_1 = 1.0
    t0_1, ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1 = build_traj1(
        h=h, horizon=horizon_1
    )

    # Publisher "ground truth" for traj1
    pub_x0_1 = np.array([x[0] for x in xs1])
    pub_u0_1 = np.array([u[0] for u in us1])

    # Publish traj1 and integrate into queues
    print("Publishing first trajectory (traj1)...")
    while True:
        pub.publish(ts1, dts1, xs1, dxs1, us1, Ks1, types1, params1)
        if sub_no_interp.has_new_msg() and sub_interp.has_new_msg():
            sub_no_interp.process_queue()
            sub_interp.process_queue()
            break
        time.sleep(0.01)
    print("First trajectory integrated into both subscribers.")

    # --- Build second trajectory to trigger selected mode ---
    t0_2, ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2 = build_traj2(
        mode, t0_1, ts1, dts1, h=h, horizon=1.0
    )

    # Publisher "ground truth" for traj2
    pub_x0_2 = np.array([x[0] for x in xs2])
    pub_u0_2 = np.array([u[0] for u in us2])

    # Publish traj2 and integrate
    print(f"Publishing second trajectory (traj2) to trigger {mode.upper()}...")
    while True:
        pub.publish(ts2, dts2, xs2, dxs2, us2, Ks2, types2, params2)
        if sub_no_interp.has_new_msg() and sub_interp.has_new_msg():
            sub_no_interp.process_queue()
            sub_interp.process_queue()
            break
        time.sleep(0.01)
    print("Second trajectory integrated into both subscribers.")

    # --- Control loop: log references ---

    log_t_no, log_t_interp = [], []
    log_x0_no, log_u0_no = [], []
    log_x0_interp, log_u0_interp = [], []

    # End logging a bit after traj2 horizon
    t_end_logging = ts2[-1] + dts2[-1] + 0.5

    print("Starting control loop...")
    while True:
        if ROS_VERSION == 2:
            now = rclpy.clock.Clock().now().seconds_nanoseconds()[0]
        else:
            now = rospy.Time.now().to_sec()
        t_now = now

        # Non-interpolated
        if sub_no_interp.process_queue():
            t_ref, dt_ref, x_ref, dx_ref, u_ref, K_ref, ctype, cparam = \
                sub_no_interp.get_current_reference()
            log_t_no.append(t_now)
            log_x0_no.append(x_ref[0])
            log_u0_no.append(u_ref[0])

        # Interpolated
        if sub_interp.process_queue():
            t_ref_i, dt_ref_i, x_ref_i, dx_ref_i, u_ref_i, K_ref_i, ctype_i, cparam_i = \
                sub_interp.get_current_reference()
            log_t_interp.append(t_now)
            log_x0_interp.append(x_ref_i[0])
            log_u0_interp.append(u_ref_i[0])

        if t_now >= t_end_logging:
            break

        time.sleep(0.01)

    print(f"Control loop for {mode.upper()} finished.")

    return {
        "t_no": np.array(log_t_no),
        "t_interp": np.array(log_t_interp),
        "x0_no": np.array(log_x0_no),
        "x0_interp": np.array(log_x0_interp),
        "u0_no": np.array(log_u0_no),
        "u0_interp": np.array(log_u0_interp),
        "t0_2": t0_2,
        # publisher trajectories:
        "pub_t1": np.array(ts1),
        "pub_x0_1": pub_x0_1,
        "pub_u0_1": pub_u0_1,
        "pub_t2": np.array(ts2),
        "pub_x0_2": pub_x0_2,
        "pub_u0_2": pub_u0_2,
    }


def main():
    # --- Init ROS ---
    if ROS_VERSION == 2:
        rclpy.init()
    else:
        rospy.init_node("solver_trajectory_three_cases_demo", anonymous=True)

    topic = "/crocoddyl/solver_trajectory"
    pub = SolverTrajectoryRosPublisher()  # uses same default topic

    # Run the three cases
    modes = ["replace", "append", "merge"]
    results = {}

    for mode in modes:
        results[mode] = run_case(mode, pub, topic=topic, interpolation_window=5)

    # --- Plot results: 3 columns (replace / append / merge), 2 rows (x[0], u[0]) ---

    fig, axs = plt.subplots(2, 3, figsize=(18, 6), sharex=False)

    for j, mode in enumerate(modes):
        r = results[mode]

        t_no = r["t_no"]
        t_interp = r["t_interp"]
        x0_no = r["x0_no"]
        x0_interp = r["x0_interp"]
        u0_no = r["u0_no"]
        u0_interp = r["u0_interp"]
        t0_2 = r["t0_2"]

        pub_t1 = r["pub_t1"]
        pub_x0_1 = r["pub_x0_1"]
        pub_u0_1 = r["pub_u0_1"]
        pub_t2 = r["pub_t2"]
        pub_x0_2 = r["pub_x0_2"]
        pub_u0_2 = r["pub_u0_2"]

        # --- State x[0] ---
        ax_x = axs[0, j]
        # Publisher trajectories
        ax_x.plot(pub_t1, pub_x0_1, linestyle=":", alpha=0.6,
                  label="traj1 x[0] (pub)")
        ax_x.plot(pub_t2, pub_x0_2, linestyle="--", alpha=0.6,
                  label="traj2 x[0] (pub)")

        # Subscriber outputs
        ax_x.plot(t_no, x0_no, label="x[0] (no interp)")
        ax_x.plot(t_interp, x0_interp, label="x[0] (interp)")

        ax_x.axvline(t0_2, linestyle="-.", alpha=0.7, label="traj2 start")
        ax_x.set_title(mode.upper())
        ax_x.set_ylabel("x[0]")
        ax_x.grid(True)
        if j == 0:
            ax_x.legend()

        # --- Control u[0] ---
        ax_u = axs[1, j]
        # Publisher trajectories
        ax_u.plot(pub_t1, pub_u0_1, linestyle=":", alpha=0.6,
                  label="traj1 u[0] (pub)")
        ax_u.plot(pub_t2, pub_u0_2, linestyle="--", alpha=0.6,
                  label="traj2 u[0] (pub)")

        # Subscriber outputs
        ax_u.plot(t_no, u0_no, label="u[0] (no interp)")
        ax_u.plot(t_interp, u0_interp, label="u[0] (interp)")

        ax_u.axvline(t0_2, linestyle="-.", alpha=0.7, label="traj2 start")
        ax_u.set_ylabel("u[0]")
        ax_u.set_xlabel("time [s]")
        ax_u.grid(True)
        if j == 0:
            ax_u.legend()

    plt.tight_layout()
    plt.show()

    if ROS_VERSION == 2:
        rclpy.shutdown()
    else:
        rospy.signal_shutdown("Demo finished")


if __name__ == "__main__":
    main()
